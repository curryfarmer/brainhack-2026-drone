"""bench_flight — propless scripted-flight command + telemetry logger.

The bench instrument for the "motor readback" the operator asked for. The
reality (verified against finals/flight/pyhulax_adapter.py): the pyhulax SDK
exposes NO motor-RPM / actuator readback — only COMMANDS in, and TELEMETRY out
(battery_pct / altitude_m / yaw_deg / is_flying). So "motor readback" on this
airframe = log the COMMANDED action plus the telemetry RESPONSE it produced.

This is a THIN scripted-sequence runner. It does NOT re-implement the mission
orchestrator (finals/mission/) — it issues a flat list of commands against a
FlightAdapter and records each one. For every command it appends, to a per-run
JSONL, an `action_complete` event (the EXACT schema finals/agent.py emits and
finals/tools/replay_plot.py replays — so replay_plot draws the dead-reckoned
track from this log with NO changes) PLUS a `bench_command` event carrying the
PRE and POST telemetry snapshots (battery_pct, altitude_m, yaw_deg, is_flying,
and the adapter's `_airborne` flag when present), the monotonic ts and
elapsed_s. The forensic detail lives in `bench_command` precisely BECAUSE
replay_plot's reconstruct_action enforces an EXACT field set on
`action_complete` — telemetry must not pollute that record.

PROPS-OFF CAVEAT (read before believing the altitude column):
  With the propellers OFF, the motors spin but make NO lift, so altitude
  telemetry will NOT climb on takeoff and will read ~0 throughout. THAT IS
  EXPECTED. This log proves three things and NOT flight dynamics:
    1. command ACCEPTANCE   — each command completed (no FlightError);
    2. telemetry PLUMBING   — battery/alt/yaw/is_flying read before & after;
    3. yaw RESPONSE         — rotate DOES change yaw on a bench (the airframe
                              yaws in place), and is_flying flips on
                              takeoff/land. These are the live proofs.

SAFETY — this arms a REAL aircraft (motors WILL spin):
  - Default-deny: the run REFUSES without the explicit --props-off-confirmed
    flag (prints WHAT/WHY, exits non-zero). There is no other way to fly.
  - A loud banner reminds you the motors will spin and props must be OFF.
  - The 'q'+Enter abort key is armed (finals.guards.AbortListener); pressing it
    mid-sequence stops issuing commands and emergency-lands.
  - Short per-command timeouts; emergency_land() runs in a finally block and is
    never-raise by the adapter contract.

Backends (lazy-imported inside main, never at module top — the conventions
scan forbids a top-level SDK import; this keeps the bare venv importable):
  --mock  -> finals.flight.mock_adapter.MockAdapter  (pure-Python, no SDK; the
            dev/CI default-proof backend and the test double)
  (real)  -> finals.flight.pyhulax_adapter.PyhulaxAdapter  (the real drone)

USAGE
  # No hardware, no props, full CI-safe sequence (writes a JSONL, exit 0):
  python finals\\tools\\bench_flight.py --mock --props-off-confirmed

  # Custom scripted sequence (see the format below):
  python finals\\tools\\bench_flight.py --mock --props-off-confirmed \\
      --commands "takeoff:60,hover:2,rotate:90,move:forward:50,land"

  # REAL drone on the bench (PROPS OFF until the abort is proven):
  #   1. Join the drone Wi-Fi: SSID Hula-2502180050, pw 12345678.
  #   2. pyhulax ports tcp 8888 / udp_status 8668; Windows firewall OFF for
  #      inbound UDP (the telemetry heartbeat is UDP).
  #   3. A stale bind_client wedges connect -> power-cycle the drone to clear.
  #   4. Solo-AP IP is 192.168.100.1 (one drone).
  python finals\\tools\\bench_flight.py --props-off-confirmed \\
      --ip 192.168.100.1 --plane-id 6
  # Then replay the dead-reckoned track from the JSONL it wrote:
  python -m finals.tools.replay_plot runs_finals\\<ts>

SCRIPTED-SEQUENCE FORMAT (--commands, comma-separated steps):
  takeoff[:HEIGHT_CM]     default 60
  hover:SECONDS           required arg
  rotate:DEGREES          +ve = CCW (pyhulax/DeadReckoner convention)
  move:DIRECTION[:CM]     DIRECTION in forward/back/left/right/up/down; CM
                          default 50
  land                    no arg
Whitespace around steps/args is ignored. The DEFAULT sequence (no --commands)
is: takeoff:60, hover:2, rotate:90, land.

Pure stdlib + lazy adapter import; no cv2/numpy/pyhulax at module top —
finals/tools/ is inside the conventions scan.

Session: W4 (dev-bench build — propless flight logging).
"""
from __future__ import annotations

import argparse
import asyncio
import dataclasses
import enum
import math
import os
import sys
import threading
import time
from typing import List, Optional, Sequence, Tuple

# Repo importable whether launched as a path or `-m finals.tools.bench_flight`.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from finals.errors import FinalsError, FlightError
from finals.events import EventLog, create_run_dir
from finals.types import (Direction, Hover, Land, Move, Rotate, Takeoff,
                          Telemetry)

#: Default scripted sequence when --commands is absent (props-off bench loop).
_DEFAULT_COMMANDS = "takeoff:60,hover:2,rotate:90,land"
#: Short per-command deadlines (s) — a bench drone that hangs must trip fast.
_TAKEOFF_TIMEOUT_S = 20.0
_LAND_TIMEOUT_S = 20.0
_MOVE_TIMEOUT_S = 12.0
_ROTATE_TIMEOUT_S = 12.0
#: Direction tokens accepted by the move: step (lower-cased before lookup).
_DIRECTION_TOKENS = {d.name.lower(): d for d in Direction}


class BenchFlightError(FinalsError):
    """A bench_flight CLI / command-script problem. Message names WHAT was bad,
    WHICH step, and what to CHECK — never a bare ValueError."""


# ============================================================
# Command-script parsing
# ============================================================
def parse_commands(spec: str) -> List[Tuple[str, dict]]:
    """Parse a --commands string into [(verb, kwargs), ...]. Raises
    BenchFlightError (loud, naming the offending step) on any malformed step —
    a typo must never silently drop or mis-issue a command on a live airframe.

    Format per step (comma-separated, whitespace-tolerant):
      takeoff[:HEIGHT_CM]  hover:SECONDS  rotate:DEGREES
      move:DIRECTION[:CM]  land
    """
    if not isinstance(spec, str) or not spec.strip():
        raise BenchFlightError(
            "bench_flight: --commands is empty — give a comma-separated "
            "sequence like 'takeoff:60,hover:2,rotate:90,land' (or omit "
            "--commands for the default), CHECK the --commands value")
    steps: List[Tuple[str, dict]] = []
    for raw in spec.split(","):
        step = raw.strip()
        if not step:
            raise BenchFlightError(
                f"bench_flight: empty step in --commands {spec!r} (a stray "
                f"comma?) — CHECK the sequence string")
        parts = [p.strip() for p in step.split(":")]
        verb = parts[0].lower()
        args = parts[1:]
        if verb == "takeoff":
            height_cm = _parse_one_number(step, args, "height_cm", default=60.0,
                                          require_positive=True, as_int=True)
            steps.append(("takeoff", {"height_cm": height_cm}))
        elif verb == "hover":
            duration_s = _parse_one_number(step, args, "duration_s",
                                           default=None, require_positive=False,
                                           require_nonneg=True, as_int=False)
            steps.append(("hover", {"duration_s": duration_s}))
        elif verb == "rotate":
            angle_deg = _parse_one_number(step, args, "angle_deg", default=None,
                                          require_positive=False, as_int=False)
            steps.append(("rotate", {"angle_deg": angle_deg}))
        elif verb == "move":
            if not args:
                raise BenchFlightError(
                    f"bench_flight: 'move' step {step!r} needs a direction — "
                    f"move:DIRECTION[:CM] with DIRECTION in "
                    f"{sorted(_DIRECTION_TOKENS)}; CHECK the step")
            direction = _parse_direction(step, args[0])
            distance_cm = _parse_one_number(step, args[1:], "distance_cm",
                                            default=50.0, require_positive=True,
                                            as_int=True)
            steps.append(("move", {"direction": direction,
                                   "distance_cm": distance_cm}))
        elif verb == "land":
            if args:
                raise BenchFlightError(
                    f"bench_flight: 'land' step {step!r} takes no argument, "
                    f"got {args} — CHECK the step (just 'land')")
            steps.append(("land", {}))
        else:
            raise BenchFlightError(
                f"bench_flight: unknown command {verb!r} in step {step!r} — "
                f"known: takeoff, hover, rotate, move, land; CHECK the "
                f"--commands string")
    if not steps:
        raise BenchFlightError(
            f"bench_flight: --commands {spec!r} parsed to zero steps — CHECK "
            f"the sequence string")
    return steps


def _parse_one_number(step: str, args: Sequence[str], field: str, *,
                      default: Optional[float], require_positive: bool,
                      as_int: bool, require_nonneg: bool = False):
    """Parse the single numeric argument of a step. Loud on too-many-args, a
    missing required arg (default is None), non-finite, or a sign violation."""
    if len(args) > 1:
        raise BenchFlightError(
            f"bench_flight: step {step!r} has extra arguments {list(args[1:])} "
            f"— expected a single {field}; CHECK the step")
    if not args or args[0] == "":
        if default is None:
            raise BenchFlightError(
                f"bench_flight: step {step!r} is missing its required {field} "
                f"— e.g. 'hover:2' or 'rotate:90'; CHECK the step")
        value: float = default
    else:
        try:
            value = float(args[0])
        except ValueError:
            raise BenchFlightError(
                f"bench_flight: {field} {args[0]!r} in step {step!r} is not a "
                f"number — CHECK the step") from None
    if not math.isfinite(value):
        raise BenchFlightError(
            f"bench_flight: {field} {value!r} in step {step!r} must be finite "
            f"(NaN/Inf would poison telemetry/DR) — CHECK the step")
    if require_positive and value <= 0:
        raise BenchFlightError(
            f"bench_flight: {field} {value!r} in step {step!r} must be > 0 — "
            f"CHECK the step")
    if require_nonneg and value < 0:
        raise BenchFlightError(
            f"bench_flight: {field} {value!r} in step {step!r} must be >= 0 — "
            f"CHECK the step")
    return int(value) if as_int else value


def _parse_direction(step: str, token: str) -> Direction:
    direction = _DIRECTION_TOKENS.get(token.lower())
    if direction is None:
        raise BenchFlightError(
            f"bench_flight: unknown move direction {token!r} in step {step!r} "
            f"— known: {sorted(_DIRECTION_TOKENS)}; CHECK the step")
    return direction


# ============================================================
# Telemetry snapshot for the log
# ============================================================
def telemetry_snapshot(adapter) -> dict:
    """The PRE/POST telemetry dict logged on each command. Never raises — a
    telemetry getter that errors (never-connected, link dropped) is recorded
    as {"error": ...} so the forensic record survives instead of crashing the
    sequence. Captures the adapter's authoritative `_airborne` flag when the
    backend has one (PyhulaxAdapter does; MockAdapter does not)."""
    snap: dict = {"airborne": getattr(adapter, "_airborne", None)}
    try:
        t: Telemetry = adapter.telemetry()
    except FlightError as e:
        snap["error"] = str(e)
        return snap
    snap.update({
        "battery_pct": t.battery_pct,
        "altitude_m": t.altitude_m,
        "yaw_deg": t.yaw_deg,
        "is_flying": t.is_flying,
        "telemetry_age_s": round(t.age_s(), 3),
    })
    return snap


def _action_fields(action) -> dict:
    """Action dataclass -> JSON-friendly dict, enums by NAME — byte-for-byte
    the schema finals/agent.py._action_fields emits, so replay_plot's
    reconstruct_action accepts the action_complete record unchanged."""
    return {k: (v.name if isinstance(v, enum.Enum) else v)
            for k, v in dataclasses.asdict(action).items()}


# ============================================================
# The sequence runner
# ============================================================
async def _issue(adapter, verb: str, kwargs: dict):
    """Issue ONE command on the adapter and return the typed Action it maps to
    (for the action_complete log). Short deadlines per command."""
    if verb == "takeoff":
        await adapter.takeoff(height_cm=kwargs["height_cm"],
                              timeout_s=_TAKEOFF_TIMEOUT_S)
        return Takeoff(height_cm=kwargs["height_cm"])
    if verb == "land":
        await adapter.land(timeout_s=_LAND_TIMEOUT_S)
        return Land()
    if verb == "rotate":
        await adapter.rotate(kwargs["angle_deg"], timeout_s=_ROTATE_TIMEOUT_S)
        return Rotate(angle_deg=kwargs["angle_deg"])
    if verb == "move":
        await adapter.move(kwargs["direction"], kwargs["distance_cm"],
                           timeout_s=_MOVE_TIMEOUT_S)
        return Move(direction=kwargs["direction"],
                    distance_cm=kwargs["distance_cm"])
    if verb == "hover":
        await adapter.hover(kwargs["duration_s"])
        return Hover(duration_s=kwargs["duration_s"])
    # parse_commands is the only producer of verbs; an unknown one here is a
    # programming error, not operator input — fail loud, do not fly it.
    raise BenchFlightError(
        f"bench_flight: internal — _issue got unknown verb {verb!r}; the "
        f"command parser and the issuer disagree (a code bug)")


async def run_sequence(adapter, drone_id: str, steps: List[Tuple[str, dict]],
                       events: EventLog, *,
                       abort_event: Optional[threading.Event] = None,
                       clock=time.monotonic) -> dict:
    """Connect, log an `origin` event (replay_plot seed), then issue each step
    logging an `action_complete` (replay schema) + a `bench_command` (pre/post
    telemetry). Honors abort_event between commands. emergency_land() runs in a
    finally and is never-raise. Returns a summary dict.

    NOTE: this does NOT swallow command failures — a FlightError mid-sequence
    propagates after the finally safe-down (fail-loud), exactly like the agent.
    """
    summary = {"drone_id": drone_id, "commands_total": len(steps),
               "commands_completed": 0, "aborted": False, "failed": None}
    aborted = False
    try:
        await adapter.connect(timeout_s=10.0)
        # origin: replay_plot seeds the DeadReckoner from this. pyhulax has no
        # closed-loop position (position_m=None, yaw may be None pre-first-
        # poll) — replay_plot seeds 0.0 for nulls with a loud warning, which is
        # correct for a bench log.
        pre_origin = telemetry_snapshot(adapter)
        events.log(drone_id, "origin", position_m=None,
                   yaw_deg=pre_origin.get("yaw_deg"))
        for idx, (verb, kwargs) in enumerate(steps):
            if abort_event is not None and abort_event.is_set():
                aborted = True
                summary["aborted"] = True
                events.log(drone_id, "bench_abort", reason="operator 'q' abort",
                           before_command=verb, step_index=idx)
                break
            pre = telemetry_snapshot(adapter)
            t0 = clock()
            action = await _issue(adapter, verb, kwargs)
            elapsed_s = clock() - t0
            post = telemetry_snapshot(adapter)
            fields = _action_fields(action)
            # The replay-schema record: EXACTLY {action, elapsed_s, *fields}.
            events.log(drone_id, "action_complete",
                       action=type(action).__name__,
                       elapsed_s=round(elapsed_s, 4), **fields)
            # The forensic record: pre/post telemetry + the issued command.
            events.log(drone_id, "bench_command",
                       command=verb, args=fields, step_index=idx,
                       ts_mono=round(t0, 4), elapsed_s=round(elapsed_s, 4),
                       pre=pre, post=post)
            summary["commands_completed"] = idx + 1
    except FlightError as e:
        # A command failed loudly. Record it, then safe-down in the finally.
        summary["failed"] = str(e)
        events.log(drone_id, "bench_failed", error=str(e),
                   error_type=type(e).__name__)
        raise
    finally:
        # The ONE must-never-raise teardown. The adapter's emergency_land() is
        # never-raise by the FlightAdapter contract; the typed guard here is
        # belt-and-suspenders for a non-contract surprise (e.g. a closed event
        # loop) so a bench crash still ends with a safe-down attempt logged.
        try:
            await adapter.emergency_land()
        except (FinalsError, OSError, RuntimeError) as e:
            try:
                events.log(drone_id, "emergency_land_failed", error=str(e),
                           error_type=type(e).__name__)
            except FinalsError:
                pass  # log itself failed mid-teardown; nothing left to do
            print(f"[bench_flight] WARNING: emergency_land teardown raised "
                  f"{type(e).__name__}: {e} — relying on the onboard battery "
                  f"failsafe", file=sys.stderr, flush=True)
        try:
            await adapter.disconnect()
        except (FinalsError, OSError, RuntimeError) as e:
            print(f"[bench_flight] WARNING: disconnect raised "
                  f"{type(e).__name__}: {e}", file=sys.stderr, flush=True)
    if not aborted:
        summary["aborted"] = False
    return summary


# ============================================================
# CLI
# ============================================================
def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="bench_flight",
        description="Propless scripted-flight command + telemetry logger. "
                    "Issues a scripted sequence against a FlightAdapter and "
                    "logs each command with PRE/POST telemetry to a per-run "
                    "JSONL (replay_plot-compatible). Default-deny: REQUIRES "
                    "--props-off-confirmed (motors WILL spin).")
    p.add_argument("--props-off-confirmed", action="store_true",
                   help="REQUIRED to run. Confirms the propellers are OFF "
                        "before the motors spin. Without it the tool refuses "
                        "and exits non-zero.")
    p.add_argument("--mock", action="store_true",
                   help="use the pure-Python MockAdapter (no SDK, no hardware) "
                        "— the dev/CI backend. Default backend is the real "
                        "PyhulaxAdapter.")
    p.add_argument("--commands", default=None,
                   help="scripted sequence, comma-separated steps: "
                        "takeoff[:CM],hover:S,rotate:DEG,move:DIR[:CM],land "
                        f"(default: {_DEFAULT_COMMANDS!r}).")
    p.add_argument("--drone-id", default="bench",
                   help="drone id used in the log + run dir (default 'bench').")
    p.add_argument("--ip", default=None,
                   help="real drone IP for PyhulaxAdapter (e.g. 192.168.100.1 "
                        "on the solo AP). Ignored with --mock.")
    p.add_argument("--plane-id", type=int, default=None,
                   help="real drone Dola plane_id (used as the drone id when "
                        "given and --drone-id is left default). Ignored with "
                        "--mock.")
    p.add_argument("--out-dir", default="runs_finals",
                   help="base dir for the per-run output dir (default "
                        "'runs_finals'; a runs_finals/<ts>/ is created).")
    p.add_argument("--no-abort-key", action="store_true",
                   help="do NOT arm the 'q'+Enter abort key (for piped/"
                        "non-interactive runs where stdin is unavailable).")
    return p.parse_args(argv)


def _build_adapter(args: argparse.Namespace, drone_id: str):
    """Construct the backend. SDK import is LAZY (inside this function) so the
    module stays importable on a bare venv (conventions scan + bare-venv
    suite). --mock -> MockAdapter (no SDK); else PyhulaxAdapter (real)."""
    if args.mock:
        from finals.flight.mock_adapter import MockAdapter
        # A touch of battery decay so the props-off log shows the telemetry
        # plumbing moving (otherwise battery sits pinned at 100%).
        return MockAdapter(drone_id, battery_decay_pct_per_cmd=1.0)
    from finals.flight.pyhulax_adapter import PyhulaxAdapter
    return PyhulaxAdapter(drone_id, ip=args.ip)


def _print_banner(drone_id: str, backend: str, steps: List[Tuple[str, dict]],
                  run_dir: str) -> None:
    bar = "=" * 72
    print(bar)
    print("  HULA BENCH FLIGHT — scripted command + telemetry logger")
    print(f"  backend={backend}   drone={drone_id}")
    print(bar)
    if backend != "mock":
        print("  !! MOTORS WILL SPIN — PROPS MUST BE OFF !!")
        print("     With props off, ALTITUDE WILL NOT CLIMB (no lift) — that")
        print("     is EXPECTED. This log proves command ACCEPTANCE + telemetry")
        print("     PLUMBING + yaw RESPONSE (rotate yaws the airframe), not")
        print("     flight dynamics.")
        print(bar)
    print("  sequence:")
    for i, (verb, kwargs) in enumerate(steps):
        detail = ", ".join(f"{k}={getattr(v, 'name', v)}"
                           for k, v in kwargs.items())
        print(f"    {i + 1:>2}. {verb}({detail})")
    print(f"  run dir: {run_dir}")
    print(bar, flush=True)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)

    # DEFAULT-DENY props-off gate — the very first check, before any run dir or
    # adapter construction. WHAT/WHY + non-zero exit.
    if not args.props_off_confirmed:
        print(
            "\nbench_flight REFUSED: this tool spins the motors on a REAL "
            "aircraft.\n"
            "  WHAT: a scripted takeoff/hover/rotate/move/land sequence that "
            "commands the drone.\n"
            "  WHY refused: --props-off-confirmed was not given (default-deny "
            "safety gate).\n"
            "  CHECK: remove the propellers, then re-run with "
            "--props-off-confirmed.\n",
            file=sys.stderr, flush=True)
        return 2

    try:
        steps = parse_commands(args.commands
                               if args.commands is not None
                               else _DEFAULT_COMMANDS)
    except BenchFlightError as e:
        print(f"\n{e}\n", file=sys.stderr, flush=True)
        return 2

    drone_id = args.drone_id
    if (args.plane_id is not None and not args.mock
            and args.drone_id == "bench"):
        drone_id = str(args.plane_id)

    backend = "mock" if args.mock else "pyhulax"
    try:
        adapter = _build_adapter(args, drone_id)
    except (FinalsError, ValueError) as e:
        print(f"\nbench_flight ERROR: could not build the {backend} adapter: "
              f"{e}\n", file=sys.stderr, flush=True)
        return 2

    try:
        run_dir = create_run_dir(args.out_dir)
    except FinalsError as e:
        print(f"\nbench_flight ERROR: {e}\n", file=sys.stderr, flush=True)
        return 2

    _print_banner(drone_id, backend, steps, run_dir)

    # Arm the q-abort key (reused finals.guards.AbortListener) unless disabled.
    abort_event = threading.Event()
    listener = None
    if not args.no_abort_key:
        from finals.guards import AbortListener
        listener = AbortListener(abort_event)
        listener.start()

    rc = 0
    summary: Optional[dict] = None
    with EventLog(run_dir) as events:
        try:
            summary = asyncio.run(run_sequence(
                adapter, drone_id, steps, events, abort_event=abort_event))
        except FlightError as e:
            # run_sequence already safed down + logged bench_failed.
            print(f"\nbench_flight: a command FAILED loudly: {e}\n",
                  file=sys.stderr, flush=True)
            rc = 1
        except KeyboardInterrupt:
            print("\nbench_flight: interrupted (Ctrl+C) — the run_sequence "
                  "finally safed the drone down.\n", file=sys.stderr,
                  flush=True)
            rc = 1
        finally:
            if listener is not None:
                listener.stop()

    if summary is not None:
        bar = "=" * 72
        print(bar)
        print(f"  SUMMARY: {summary['commands_completed']}/"
              f"{summary['commands_total']} commands completed"
              f"{' (ABORTED)' if summary['aborted'] else ''}"
              f"{' (FAILED)' if summary['failed'] else ''}")
        print(f"  log: {os.path.join(run_dir, 'mission.jsonl')}")
        print(f"  replay: python -m finals.tools.replay_plot {run_dir}")
        print(bar, flush=True)
        if summary["aborted"] and rc == 0:
            rc = 1
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
