"""MockAdapter — the scriptable FlightAdapter test double.

This mock's contract semantics are treated as PRODUCTION code: the whole
test pyramid (S4 agent/orchestrator, S5 guards) stands on it, and a mock
that lies about complete-or-raise poisons every downstream test.

Per-command pipeline (binding order, pinned by tests/test_mock_adapter.py):
 1. RECORD the attempt in .calls as (name, kwargs) — refused attempts are
    recorded too (forensics over tidiness).
 2. CONNECT gate: any command before connect() raises FlightError.
 3. DEGRADED gate: after a FlightTimeout this adapter reports degraded
    (errors.py: "the SDK may still be executing it") — takeoff/move/rotate/
    hover/set_led are refused with FlightError; land/emergency_land/
    disconnect/telemetry stay allowed (the safe-down path must always work).
    connect() clears degraded on success; DR pose and is_flying PERSIST
    across reconnect (drones don't teleport).
 4. FLYING gate: takeoff while flying / move/rotate/hover while landed raise
    FlightError (always a mission-logic bug); land is idempotent per the ABC
    ("Safe to call repeatedly").
 5. ARGUMENT gate: physically impossible magnitudes are REFUSED with
    FlightError (takeoff/move with non-positive cm, negative hover duration,
    non-finite anything) — dead_reckon.py assigns "refusing impossible
    sequences" to the adapter, and a mock that completes a move a real
    backend would refuse lets sign-error bugs first surface on hardware.
 6. FAIL INJECTION: fail_on={"move": exc} raises exc on EVERY move;
    fail_at="move:3" raises a FlightTimeout on exactly the 3rd move attempt
    (attempts are counted HERE — calls refused by gates 2-5 do not consume
    the counter). Only a FlightTimeout marks the adapter degraded; an
    injected plain FlightError does not (the command failed CLEANLY).
    Config is validated at construction: "disconnect"/"emergency_land" are
    rejected (their contract is never-raise), typos fail loudly instead of
    silently never firing, and naming the same command in BOTH fail_on and
    fail_at is rejected (fail_on would silently mask fail_at).
 7. LATENCY vs deadline: latency_s > timeout_s raises FlightTimeout
    IMMEDIATELY (deterministic and fast — no wall-clock wait; the message
    still cites timeout_s) and marks degraded; otherwise the command awaits
    asyncio.sleep(latency_s) (bounded: latency_s is validated finite and
    >= 0 at construction AND re-checked before every sleep, since it is
    public-mutable). hover()/set_led() have no timeout_s in the ABC, so
    they are exempt from the comparison and just sleep latency_s (NEVER
    duration_s — simulated time is not wall time).
 8. EFFECTS on success only, in this order: the telemetry-freeze snapshot is
    latched FIRST if the clock has passed the threshold (so frozen values
    can never postdate the pinned freeze ts — commands completing after the
    freeze instant must not leak into it), then pose integration via the
    shared DeadReckoner (single source of truth for the math), is_flying
    update, battery decay (battery_decay_pct_per_cmd per completed command
    in _DRAINING — takeoff/move/rotate/land/hover — clamped at
    battery_floor_pct).

Telemetry (deliberate deviations, documented + pinned in tests):
- telemetry() is NOT recorded in .calls — a 10 Hz poller would swamp every
  command-order assertion; it increments .telemetry_calls instead. (The S3
  handover wording says ".calls records every call"; this deviation is the
  reviewed exception.)
- telemetry() before the FIRST connect() raises FlightError: fabricating a
  fresh-ts Telemetry would sail straight through the S5 staleness guards —
  there is no honest data to return. After disconnect() it returns the final
  snapshot captured at disconnect (stale; age_s grows) per the ABC's
  "may be stale".
- position_m = (north_m, east_m, alt_m) — third element UP-POSITIVE, see
  finals/flight/dead_reckon.py for the binding frame convention. Quality is
  ALWAYS PositionQuality.DEAD_RECKONING.
- Telemetry FREEZE (S5 guard hook, deterministic via the injectable clock):
  once clock() - connect_ts >= freeze_telemetry_after_s, the snapshot is
  built ONCE with ts pinned at connect_ts + freeze_telemetry_after_s and
  returned unchanged forever after — age_s grows while values stay constant.
  Reconnect restarts the window. NOTE: asyncio.sleep(latency_s) uses the
  event-loop clock, not the injected one; the injected clock covers
  telemetry stamping + freeze only.

emergency_land() never raises and needs no gates: it just records, lands
(is_flying=False, DR notes Land), and returns — there is nothing here worth
swallowing an exception over (blanket exception-catching is forbidden in
this file by tests/test_conventions.py; only guards.py/orchestrator.py may).

Derives from: nothing external — it IS the test double the whole pyramid
stands on (used by tests for agent, orchestrator, guards, phases).

Session: S3 (implemented).
"""
from __future__ import annotations

import asyncio
import math
import time
from typing import Callable, Dict, List, Optional, Tuple

from finals.errors import FlightError, FlightTimeout
from finals.flight.adapter import FlightAdapter
from finals.flight.dead_reckon import DeadReckoner
from finals.types import Direction, Hover, Land, Move, Rotate, Takeoff, Telemetry

# Commands that may carry scripted failures. disconnect/emergency_land are
# deliberately ABSENT: their contract is never-raise, and a mock that can be
# scripted to violate its own contract is worse than no mock.
_FAILABLE = frozenset({"connect", "takeoff", "move", "rotate", "land",
                       "hover", "set_led"})
# Commands that drain the simulated battery when they complete.
_DRAINING = frozenset({"takeoff", "move", "rotate", "land", "hover"})


class MockAdapter(FlightAdapter):
    """Scriptable in-memory FlightAdapter. See the module docstring for the
    binding pipeline order and deviations; constructor args:

    latency_s                 simulated per-command duration (await-ed when
                              <= the command's timeout_s; > raises FlightTimeout)
    fail_on                   {command_name: exception_instance} raised on
                              EVERY call of that command
    fail_at                   "command:N" — the Nth attempt of that command
                              raises a FlightTimeout (1-based)
    battery_start_pct /       linear decay curve: each completed flight
      battery_decay_pct_per_cmd /  command subtracts decay, clamped at floor
      battery_floor_pct
    freeze_telemetry_after_s  telemetry freezes this many clock-seconds
                              after connect (None = never)
    clock                     monotonic time source (inject a fake for
                              deterministic freeze/age tests)
    dead_reckoner             share/inspect the pose math (default: own)
    """

    def __init__(self, drone_id: str, *,
                 latency_s: float = 0.0,
                 fail_on: Optional[Dict[str, BaseException]] = None,
                 fail_at: Optional[str] = None,
                 battery_start_pct: float = 100.0,
                 battery_decay_pct_per_cmd: float = 0.0,
                 battery_floor_pct: float = 0.0,
                 freeze_telemetry_after_s: Optional[float] = None,
                 clock: Callable[[], float] = time.monotonic,
                 dead_reckoner: Optional[DeadReckoner] = None):
        super().__init__(drone_id)
        if not math.isfinite(latency_s) or latency_s < 0:
            raise ValueError(
                f"MockAdapter({drone_id!r}): latency_s must be finite and "
                f">= 0, got {latency_s!r} (inf would hang the deadline-less "
                f"hover/set_led path forever)")
        if battery_floor_pct > battery_start_pct:
            raise ValueError(
                f"MockAdapter({drone_id!r}): battery_floor_pct "
                f"({battery_floor_pct!r}) > battery_start_pct "
                f"({battery_start_pct!r}) — the decay curve would be inverted")
        if freeze_telemetry_after_s is not None and freeze_telemetry_after_s < 0:
            raise ValueError(
                f"MockAdapter({drone_id!r}): freeze_telemetry_after_s must "
                f"be >= 0 or None, got {freeze_telemetry_after_s!r}")

        self._fail_on: Dict[str, BaseException] = dict(fail_on or {})
        for name, exc in self._fail_on.items():
            if name not in _FAILABLE:
                raise ValueError(
                    f"MockAdapter({drone_id!r}): fail_on key {name!r} is not "
                    f"a failable command — allowed: {sorted(_FAILABLE)} "
                    f"(disconnect/emergency_land are never-raise by contract; "
                    f"anything else is a typo that would silently never fire)")
            if not isinstance(exc, BaseException):
                raise ValueError(
                    f"MockAdapter({drone_id!r}): fail_on[{name!r}] must be an "
                    f"exception INSTANCE, got {exc!r}")
        self._fail_at: Optional[Tuple[str, int]] = None
        if fail_at is not None:
            cmd, sep, nth = fail_at.partition(":")
            # isdecimal, NOT isdigit: isdigit() accepts Unicode digits like
            # '³' that int() then rejects with a raw, message-free ValueError
            # BEFORE the curated one below could fire.
            if not sep or cmd not in _FAILABLE or not nth.isdecimal() or int(nth) < 1:
                raise ValueError(
                    f"MockAdapter({drone_id!r}): fail_at must be "
                    f"'<command>:<N>' with command in {sorted(_FAILABLE)} and "
                    f"N a 1-based integer, got {fail_at!r}")
            if cmd in self._fail_on:
                raise ValueError(
                    f"MockAdapter({drone_id!r}): {cmd!r} appears in BOTH "
                    f"fail_on and fail_at — fail_on fires first on every "
                    f"call, so fail_at={fail_at!r} would silently never "
                    f"fire; script one or the other per command")
            self._fail_at = (cmd, int(nth))

        #: Public + mutable on purpose: tests script a drone going slow
        #: MID-mission by assigning latency_s between commands.
        self.latency_s = latency_s
        self._battery_pct = battery_start_pct
        self._battery_decay = battery_decay_pct_per_cmd
        self._battery_floor = battery_floor_pct
        self._freeze_after_s = freeze_telemetry_after_s
        self._clock = clock
        self.dr = dead_reckoner if dead_reckoner is not None else DeadReckoner()

        self.calls: List[Tuple[str, dict]] = []
        self.telemetry_calls = 0
        self.degraded = False
        self._connected = False
        self._ever_connected = False
        self._is_flying = False
        self._attempts: Dict[str, int] = {}      # fail-injection counters
        self._connect_ts: Optional[float] = None
        self._frozen: Optional[Telemetry] = None
        self._final_snapshot: Optional[Telemetry] = None  # set by disconnect()

    # ---------------- introspection (read-only conveniences) ----------------
    @property
    def is_flying(self) -> bool:
        return self._is_flying

    @property
    def battery_pct(self) -> float:
        return self._battery_pct

    # ---------------- pipeline helpers ----------------
    def _record(self, name: str, **kwargs) -> None:
        self.calls.append((name, kwargs))

    def _gate_connected(self, detail: str) -> None:
        if not self._connected:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — adapter not connected "
                f"— call connect() first (check startup/wiring order)")

    def _gate_not_degraded(self, detail: str) -> None:
        if self.degraded:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — adapter degraded after "
                f"a FlightTimeout (the previous command may still be "
                f"executing on the airframe) — safe-down (land/"
                f"emergency_land) and re-connect() to clear")

    def _inject_failure(self, name: str, detail: str) -> None:
        """Stage 5: consume one attempt; raise the scripted failure if any.
        Only a FlightTimeout degrades the adapter (clean failures don't)."""
        n = self._attempts.get(name, 0) + 1
        self._attempts[name] = n
        exc = self._fail_on.get(name)
        if exc is None and self._fail_at == (name, n):
            # Deliberately does NOT claim a deadline was exceeded: hover/
            # set_led carry none, and no actual limit applies to a scripted
            # failure — the message names the real cause (the injection).
            exc = FlightTimeout(
                f"{self.drone_id}: {detail} failed — scripted FlightTimeout "
                f"injected by MockAdapter fail_at='{name}:{n}' (attempt "
                f"#{n}) — check the test scenario expects this attempt to "
                f"fail")
        if exc is not None:
            if isinstance(exc, FlightTimeout):
                self.degraded = True
            raise exc

    async def _latency(self, detail: str, timeout_s: Optional[float]) -> None:
        """Stage 7: deadline check + simulated duration. timeout_s=None for
        commands whose ABC signature carries no deadline (hover/set_led)."""
        if not math.isfinite(self.latency_s) or self.latency_s < 0:
            # latency_s is public-mutable, so the constructor check alone
            # cannot keep the sleep below bounded (convention 2: every
            # awaited op bounded) — re-check before every await.
            raise ValueError(
                f"MockAdapter({self.drone_id!r}): latency_s became "
                f"{self.latency_s!r} — must stay finite and >= 0; check the "
                f"test that mutated it")
        if timeout_s is not None and self.latency_s > timeout_s:
            self.degraded = True
            # Raised immediately — waiting timeout_s of wall clock would make
            # every timeout test slow for zero extra honesty.
            raise FlightTimeout(
                f"{self.drone_id}: {detail} exceeded {timeout_s:.1f} s "
                f"(mock latency_s={self.latency_s:.1f}) — check the "
                f"command timeout vs the configured mock latency")
        if self.latency_s > 0:
            await asyncio.sleep(self.latency_s)     # bounded: checked above

    def _gate_executable(self, detail: str, ok: bool, why: str) -> None:
        """Stage 5: refuse physically impossible command arguments —
        dead_reckon.py's no-clamping policy assigns this job to the adapter,
        and completing a command a real backend would refuse lets sign-error
        bugs in phase math first surface on real hardware."""
        if not ok:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — {why} — check the "
                f"phase/servo math that produced it (sign error?)")

    def _drain_battery(self) -> None:
        self._battery_pct = max(self._battery_floor,
                                self._battery_pct - self._battery_decay)

    def _maybe_freeze(self) -> None:
        """Latch the frozen telemetry snapshot the FIRST time the clock is
        observed past the freeze threshold — called both from telemetry
        reads and BEFORE any post-threshold state mutation, so the frozen
        values can never postdate the pinned freeze ts (pairing a frozen ts
        with later state would fabricate forensic history)."""
        if (self._freeze_after_s is None or self._connect_ts is None
                or self._frozen is not None):
            return
        if self._clock() - self._connect_ts >= self._freeze_after_s:
            self._frozen = self._snapshot(
                ts=self._connect_ts + self._freeze_after_s)

    def _apply_effects(self, name: str, action=None, *,
                       flying: Optional[bool] = None) -> None:
        """Stage 8 — effects of a COMPLETED command (see module docstring):
        freeze latch first, then pose, flying, battery."""
        self._maybe_freeze()
        if action is not None:
            self.dr.note_action_complete(action)
        if flying is not None:
            self._is_flying = flying
        if name in _DRAINING:
            self._drain_battery()

    # ---------------- FlightAdapter contract ----------------
    async def connect(self, timeout_s: float = 10.0) -> None:
        detail = f"connect(timeout_s={timeout_s:g})"
        self._record("connect", timeout_s=timeout_s)
        self._inject_failure("connect", detail)
        await self._latency(detail, timeout_s)
        self._connected = True
        self._ever_connected = True
        self.degraded = False                     # reconnect = operator fix
        self._connect_ts = self._clock()          # freeze window restarts
        self._frozen = None
        self._final_snapshot = None

    async def disconnect(self) -> None:
        """Never raises. Captures the final telemetry snapshot so post-run
        forensic reads keep working (stale, age grows)."""
        self._record("disconnect")
        if self._connected:
            self._final_snapshot = self._build_telemetry()
        self._connected = False

    async def takeoff(self, height_cm: int = 80, timeout_s: float = 30.0) -> None:
        detail = f"takeoff({height_cm} cm)"
        self._record("takeoff", height_cm=height_cm, timeout_s=timeout_s)
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        if self._is_flying:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — already flying — "
                f"check the phase logic (double takeoff is a mission bug)")
        self._gate_executable(
            detail, math.isfinite(height_cm) and height_cm > 0,
            f"{height_cm!r} cm is not a physically executable takeoff height "
            f"(must be finite and > 0)")
        self._inject_failure("takeoff", detail)
        await self._latency(detail, timeout_s)
        self._apply_effects("takeoff", Takeoff(height_cm=height_cm),
                            flying=True)

    async def land(self, timeout_s: float = 30.0) -> None:
        detail = "land()"
        self._record("land", timeout_s=timeout_s)
        self._gate_connected(detail)
        # No degraded gate and no flying gate: land is the safe-down path and
        # the ABC pins it as "Safe to call repeatedly".
        self._inject_failure("land", detail)
        await self._latency(detail, timeout_s)
        self._apply_effects("land", Land(), flying=False)

    async def move(self, direction: Direction, distance_cm: int,
                   timeout_s: float = 15.0) -> None:
        detail = f"move({direction.name}, {distance_cm} cm)"
        self._record("move", direction=direction, distance_cm=distance_cm,
                     timeout_s=timeout_s)
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        if not self._is_flying:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — not flying — "
                f"takeoff() first (check the phase ordering)")
        self._gate_executable(
            detail, math.isfinite(distance_cm) and distance_cm > 0,
            f"{distance_cm!r} cm is not a physically executable distance "
            f"(must be finite and > 0; direction encodes the sign)")
        self._inject_failure("move", detail)
        await self._latency(detail, timeout_s)
        self._apply_effects("move", Move(direction=direction,
                                         distance_cm=distance_cm))

    async def rotate(self, angle_deg: float, timeout_s: float = 15.0) -> None:
        detail = f"rotate({angle_deg:g} deg)"
        self._record("rotate", angle_deg=angle_deg, timeout_s=timeout_s)
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        if not self._is_flying:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — not flying — "
                f"takeoff() first (check the phase ordering)")
        self._gate_executable(
            detail, math.isfinite(angle_deg),
            f"{angle_deg!r} deg is not a physically executable rotation "
            f"(must be finite — NaN/Inf would poison the dead-reckoned yaw)")
        self._inject_failure("rotate", detail)
        await self._latency(detail, timeout_s)
        self._apply_effects("rotate", Rotate(angle_deg=angle_deg))

    async def hover(self, duration_s: float) -> None:
        detail = f"hover({duration_s:g} s)"
        self._record("hover", duration_s=duration_s)
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        if not self._is_flying:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — not flying — "
                f"takeoff() first (check the phase ordering)")
        self._gate_executable(
            detail, math.isfinite(duration_s) and duration_s >= 0,
            f"{duration_s!r} s is not a physically executable hover duration "
            f"(must be finite and >= 0)")
        self._inject_failure("hover", detail)
        await self._latency(detail, None)   # ABC hover() carries no timeout_s
        self._apply_effects("hover", Hover(duration_s=duration_s))

    async def set_led(self, r: int, g: int, b: int) -> None:
        detail = f"set_led({r}, {g}, {b})"
        self._record("set_led", r=r, g=g, b=b)
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        self._inject_failure("set_led", detail)
        await self._latency(detail, None)   # ABC set_led() carries no timeout_s
        # Not in _DRAINING (LED draw is negligible) and no pose effect —
        # _apply_effects still latches the freeze snapshot if due.
        self._apply_effects("set_led")

    def telemetry(self) -> Telemetry:
        self.telemetry_calls += 1           # NOT in .calls — see module docstring
        if not self._ever_connected:
            raise FlightError(
                f"{self.drone_id}: telemetry() refused — never connected, so "
                f"there is no honest state to report (a fabricated fresh "
                f"timestamp would defeat staleness guards) — call connect() "
                f"first (check startup/wiring order)")
        if not self._connected:
            # Disconnected: the final snapshot, stale by construction.
            # disconnect() always captures it on the connected->disconnected
            # edge; not an assert because asserts vanish under python -O.
            if self._final_snapshot is None:
                raise FlightError(
                    f"{self.drone_id}: telemetry() internal invariant broken "
                    f"— disconnected without a final snapshot; check "
                    f"MockAdapter connect/disconnect state transitions")
            return self._final_snapshot
        return self._build_telemetry()

    async def emergency_land(self) -> None:
        """Best-effort safe-down: records + lands. Never raises — and in this
        mock there is genuinely nothing to swallow."""
        self._record("emergency_land")
        self._apply_effects("emergency_land", Land(), flying=False)

    # ---------------- telemetry assembly ----------------
    def _build_telemetry(self) -> Telemetry:
        self._maybe_freeze()
        if self._frozen is not None:
            return self._frozen          # ts pinned at the freeze instant
        return self._snapshot(ts=self._clock())

    def _snapshot(self, ts: float) -> Telemetry:
        pose = self.dr.pose
        return Telemetry(
            ts=ts,
            battery_pct=self._battery_pct,
            altitude_m=pose.alt_m,
            yaw_deg=pose.yaw_deg,
            is_flying=self._is_flying,
            position_m=(pose.north_m, pose.east_m, pose.alt_m),
            position_quality=DeadReckoner.QUALITY,
        )


# ============================================================
# Manual smoke demo
# ============================================================
if __name__ == "__main__":
    from finals.errors import FlightError as _FE, FlightTimeout as _FT

    async def _happy_path() -> None:
        print("=" * 64)
        print("DEMO 1 — alpha: takeoff -> square (4x FORWARD 100 + rotate "
              "+90) -> land")
        print("=" * 64)
        alpha = MockAdapter("alpha", battery_decay_pct_per_cmd=2.0)
        await alpha.connect()
        await alpha.takeoff(80)
        for leg in range(4):
            await alpha.move(Direction.FORWARD, 100)
            await alpha.rotate(90.0)
            print(f"  after leg {leg + 1}: pose={alpha.dr.pose}")
        await alpha.land()
        print("\ncall log:")
        for name, kwargs in alpha.calls:
            print(f"  {name:<10} {kwargs}")
        final = alpha.dr.pose
        print(f"\nfinal DR pose : {final}")
        print(f"expected      : ~ DRPose(north_m=0, east_m=0, alt_m=0, "
              f"yaw_deg=0)  (closed square, landed)")
        t = alpha.telemetry()
        print(f"telemetry     : pos={t.position_m}  quality="
              f"{t.position_quality.name}  battery={t.battery_pct:.0f}%  "
              f"age={t.age_s():.3f}s")

    async def _failure_injection() -> None:
        print()
        print("=" * 64)
        print("DEMO 2 — bravo: fail_at='move:3' (moves 1-2 succeed, 3rd "
              "times out)")
        print("=" * 64)
        bravo = MockAdapter("bravo", fail_at="move:3")
        await bravo.connect()
        await bravo.takeoff(80)
        for n in range(1, 4):
            try:
                await bravo.move(Direction.FORWARD, 100)
                print(f"  move #{n}: ok, pose={bravo.dr.pose}")
            except _FT as e:
                print(f"  move #{n}: {type(e).__name__}:")
                print(f"    {e}")
        print(f"  degraded={bravo.degraded}  pose unchanged="
              f"{bravo.dr.pose}")
        try:
            await bravo.move(Direction.FORWARD, 100)
        except _FE as e:
            print(f"  follow-up move: {type(e).__name__}:")
            print(f"    {e}")
        await bravo.emergency_land()
        print(f"  emergency_land: is_flying={bravo.is_flying}, recorded as "
              f"{bravo.calls[-1]}")

    asyncio.run(_happy_path())
    asyncio.run(_failure_injection())
