"""Finals C2 entry point — the composition root.

This is the ONLY file that imports concrete backends, and it imports each one
lazily inside its builder so a missing SDK fails loudly at wiring time but
only for the backend actually selected.

Usage (run from repo root):
  python -m finals.main --profile mock --dry-run
  python -m finals.main --profile sitl --phases takeoff_demo --budget 120
  python -m finals.main --profile replay
  python -m finals.main --profile bench
  python -m finals.main --profile real --i-know-this-arms-real-drones

Exit codes: 0 ok | 1 unexpected error / any drone FAILED | 2 config error |
3 preflight failure.

Session: S1 (CLI + config + wiring resolution + --dry-run); S4 (flight path:
run dir + crash hooks + EventLog + per-drone adapter/phase/agent wiring ->
preflight gate -> Orchestrator.run -> exit code). The no-drone replay/vision
path arrives with perception in S7 and still raises its pointer loudly.

Wiring notes (binding):
- BENCH SPECIAL CASE: BenchAdapter wraps an INNER backend, so the generic
  flight_cls(drone_id) construction does not fit it — _build_adapter builds
  the inner backend (PyhulaxAdapter, S9) first and wraps it (see the
  BenchAdapter docstring).
- Phase construction soft convention: a phase class MAY define
  from_config(drone_cfg, cfg) (TakeoffDemo does — zone tunables + altitude
  band); phases without it are built with no arguments.
- Preflight gate: bench/real run finals.preflight.run_preflight (S10 stub —
  raises its session pointer today); mock/sitl record a loud
  preflight-skipped event instead (nothing to gate in pure software).

Derives from: qualifier_run.py parse_args/_amain (CLI override conventions:
--weights/--budget/--no-detector/--display kept compatible).
"""
from __future__ import annotations

import argparse
import asyncio
import os
import sys
from typing import List, Optional, Type

from finals.config import DroneConfig, FinalsConfig, load_config
from finals.errors import ConfigError, PreflightError
from finals.events import EventLog, create_run_dir, install_crash_hooks
from finals.mission.agent import DroneAgent
from finals.mission.orchestrator import Orchestrator
from finals.mission.phase import MissionPhase
from finals.mission.phases import resolve_phase
from finals.sightings import SightingBus

_CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs")

EXIT_OK = 0
EXIT_ERROR = 1
EXIT_CONFIG = 2
EXIT_PREFLIGHT = 3


# ============================================================
# CLI
# ============================================================
def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="finals.main",
        description="BrainHack 2026 finals C2 runner (swarm challenge)",
    )
    p.add_argument("--profile", required=True,
                   choices=("mock", "sitl", "replay", "bench", "real"),
                   help="execution profile; selects finals/configs/<profile>.json")
    p.add_argument("--config", help="explicit config path (overrides the profile default)")
    p.add_argument("--weights", help="detector weights override (forces backend=ultralytics)")
    p.add_argument("--phases", help="comma-separated phase names forced onto ALL drones")
    p.add_argument("--budget", type=float, help="mission wall-clock budget override (s)")
    p.add_argument("--no-detector", action="store_true", help="disable detection entirely")
    p.add_argument("--display", action="store_true", help="show the detection window (debug)")
    p.add_argument("--dry-run", action="store_true",
                   help="load config, resolve backends + phases, print the plan, exit 0")
    p.add_argument("--i-know-this-arms-real-drones", action="store_true",
                   help="required confirmation gate for --profile real")
    return p.parse_args(argv)


# ============================================================
# Backend builders — lazy imports, one per backend
# ============================================================
def resolve_flight_adapter_cls(backend: str) -> Optional[Type]:
    """flight_backend name -> adapter class (None for 'none'). ConfigError on unknown."""
    if backend == "none":
        return None
    if backend == "mock":
        from finals.flight.mock_adapter import MockAdapter
        return MockAdapter
    if backend == "mavsdk_sitl":
        from finals.flight.sitl_adapter import MavsdkSitlAdapter
        return MavsdkSitlAdapter
    if backend == "pyhulax":
        from finals.flight.pyhulax_adapter import PyhulaxAdapter
        return PyhulaxAdapter
    if backend == "bench":
        from finals.flight.adapter import BenchAdapter
        return BenchAdapter
    raise ConfigError(
        f"unknown flight_backend {backend!r} — one of: none, mock, mavsdk_sitl, pyhulax, bench"
    )


def resolve_video_source_cls(backend: str) -> Optional[Type]:
    """frame_backend name -> VideoSource class (None for 'none'). ConfigError on unknown."""
    if backend == "none":
        return None
    if backend == "replay":
        from finals.vision.video import ReplaySource
        return ReplaySource
    if backend == "gazebo":
        from finals.vision.gazebo_video import GazeboRgbSource
        return GazeboRgbSource
    if backend == "pyhulax":
        from finals.vision.pyhulax_video import PyhulaxVideoSource
        return PyhulaxVideoSource
    raise ConfigError(
        f"unknown frame_backend {backend!r} — one of: none, replay, gazebo, pyhulax"
    )


# ============================================================
# Dry-run report
# ============================================================
def format_resolved_plan(cfg: FinalsConfig, flight_cls: Optional[Type],
                         video_cls: Optional[Type], config_path: str) -> str:
    lines = [
        "=" * 72,
        f"RESOLVED PLAN  profile={cfg.profile}  config={config_path}",
        "=" * 72,
        f"flight_backend : {cfg.flight_backend:<12} -> {flight_cls.__name__ if flight_cls else '(none)'}",
        f"frame_backend  : {cfg.frame_backend:<12} -> {video_cls.__name__ if video_cls else '(none)'}",
        "detection      : "
        + ("(no frame source — perception off)" if cfg.frame_backend == "none" else
           "aruco (primary, always on) + yolo: " + cfg.detector.backend
           + (f"  weights={cfg.detector.weights}  conf={cfg.detector.conf}  device={cfg.detector.device}"
              if cfg.detector.backend == "ultralytics" else "")),
        f"budget         : {cfg.mission_budget_s:.0f} s   tick: {cfg.tick_hz:.0f} Hz   "
        f"cmd timeout: {cfg.command_timeout_s:.0f} s   battery floor: {cfg.min_battery_pct:.0f} %",
        f"run_dir        : {cfg.run_dir}",
    ]
    if cfg.profile == "replay":
        lines.append(f"replay_dir     : {cfg.replay_dir}")
    if cfg.use_uwb:
        lines.append(f"uwb            : serial {cfg.uwb_serial_port}")
    lines.append("-" * 72)
    if not cfg.drones:
        lines.append("drones         : (none — laptop-only profile)")
    for d in cfg.drones:
        lines.append(
            f"drone {d.id:<8} plane_id={d.plane_id!s:<5} band={d.altitude_band_m!s:<5} "
            f"led={d.led_rgb!s:<16} phases={d.phases}"
        )
    lines.append("=" * 72)
    return "\n".join(lines)


# ============================================================
# Entry
# ============================================================
def run(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)

    if args.profile == "real" and not args.i_know_this_arms_real_drones:
        raise ConfigError(
            "refusing --profile real without --i-know-this-arms-real-drones. "
            "This profile arms physical aircraft; the flag is the first of "
            "three gates (flag -> preflight P0-P10 -> operator GO)."
        )

    config_path = args.config or os.path.join(_CONFIG_DIR, f"{args.profile}.json")
    overrides = {}
    if args.weights:
        overrides["weights"] = args.weights
    if args.budget is not None:
        overrides["budget_s"] = args.budget
    if args.phases:
        overrides["phases"] = [p.strip() for p in args.phases.split(",") if p.strip()]
    if args.no_detector:
        overrides["no_detector"] = True
    if args.display:
        overrides["display"] = True

    cfg = load_config(config_path, overrides)
    if cfg.profile != args.profile:
        raise ConfigError(
            f"--profile {args.profile} but {config_path} declares profile "
            f"{cfg.profile!r} — wrong file? Pass --config explicitly or fix the JSON."
        )

    # Resolve EVERYTHING before doing anything: unknown names die here, loudly.
    flight_cls = resolve_flight_adapter_cls(cfg.flight_backend)
    video_cls = resolve_video_source_cls(cfg.frame_backend)
    for d in cfg.drones:
        for phase_name in d.phases:
            resolve_phase(phase_name)  # ConfigError lists available phases

    if args.dry_run:
        print(format_resolved_plan(cfg, flight_cls, video_cls, config_path))
        return EXIT_OK

    return _run_mission(cfg)


# ============================================================
# Mission execution (S4)
# ============================================================
def _build_adapter(cfg: FinalsConfig, drone: DroneConfig):
    """One adapter per drone, from (FinalsConfig, DroneConfig) — NEVER bare
    flight_cls(drone_id): BenchAdapter wraps an inner backend (special case
    below, see its docstring), and MavsdkSitlAdapter will need per-drone
    (sitl_address, mavsdk_grpc_port) from here in S6/SIM-1
    (sim_sessions.md "Notes to roadmap sessions")."""
    if cfg.flight_backend == "bench":
        from finals.flight.adapter import BenchAdapter
        from finals.flight.pyhulax_adapter import PyhulaxAdapter   # S9
        return BenchAdapter(PyhulaxAdapter(drone.id))
    flight_cls = resolve_flight_adapter_cls(cfg.flight_backend)
    if flight_cls is None:      # 'none' is guarded before we get here
        raise ConfigError(
            f"profile {cfg.profile!r} has flight_backend 'none' — nothing "
            f"to fly; this path should have been routed to the no-drone "
            f"branch (wiring bug)")
    return flight_cls(drone.id)


def _build_phases(drone_cfg: DroneConfig,
                  cfg: FinalsConfig) -> List[MissionPhase]:
    """Phase names -> instances. Soft convention: classes MAY define
    from_config(drone_cfg, cfg); otherwise no-arg construction. Stub phases
    raise their session pointer loudly here, at wiring time."""
    phases: List[MissionPhase] = []
    for name in drone_cfg.phases:
        phase_cls = resolve_phase(name)
        factory = getattr(phase_cls, "from_config", None)
        phases.append(factory(drone_cfg, cfg) if factory is not None
                      else phase_cls())
    return phases


async def _amain(cfg: FinalsConfig, agents: List[DroneAgent],
                 events: EventLog, run_dir: str, bus: SightingBus) -> int:
    """Preflight gate -> Orchestrator.run, on ONE event loop."""
    if cfg.profile in ("bench", "real"):
        from finals.preflight import run_preflight     # S10 — pointer today
        await run_preflight(cfg.profile, agents, cfg)  # PreflightError -> 3
    else:
        # "mission" = the orchestrator's mission-level pseudo drone id.
        events.log("mission", "preflight", status="skipped",
                   reason=f"profile {cfg.profile!r} has no preflight gate "
                          f"(P0-P10 applies to bench/real; arrives S10)")
    orchestrator = Orchestrator(agents, events, run_dir,
                                budget_s=cfg.mission_budget_s, bus=bus)
    return await orchestrator.run()


def _run_mission(cfg: FinalsConfig) -> int:
    if cfg.flight_backend == "none":
        raise NotImplementedError(
            "finals.main: the no-drone (replay/vision) execution path is "
            "wired with perception in session S7 — see "
            "finals/docs/module_map.md. Only --dry-run works for this "
            "profile today.")

    run_dir = create_run_dir(cfg.run_dir)
    install_crash_hooks(run_dir)
    with EventLog(run_dir) as events:
        bus = SightingBus()
        agents = [
            DroneAgent(d.id, _build_adapter(cfg, d),
                       _build_phases(d, cfg), events, bus=bus,
                       command_timeout_s=cfg.command_timeout_s)
            for d in cfg.drones
        ]
        return asyncio.run(_amain(cfg, agents, events, run_dir, bus))


def main(argv: Optional[List[str]] = None) -> int:
    try:
        return run(argv)
    except ConfigError as e:
        print(f"\nCONFIG ERROR: {e}\n", file=sys.stderr)
        return EXIT_CONFIG
    except PreflightError as e:
        print(f"\nPREFLIGHT FAILED: {e}\n", file=sys.stderr)
        return EXIT_PREFLIGHT
    # Anything else (incl. NotImplementedError stub pointers and FinalsError
    # subtypes not handled above) propagates with a full traceback — fail loud.


if __name__ == "__main__":
    sys.exit(main())
