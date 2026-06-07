"""MavsdkSitlAdapter — the qualifier PX4 SITL VM backend (SIM-1 / roadmap S6).

Implements the SAME relative-move semantics as pyhulax on top of MAVSDK NED
position setpoints, so the VM exercises the exact mission logic the real
drones run. One instance per drone, parameterized per-drone
(sitl_address, grpc_port) — three of these fly concurrently in SIM-2.

VENDORED-WITH-FIXES (convention 9 — root drone_control.py is NEVER edited and
CANNOT be wrapped for multi-drone: its connect() hardcodes udpin://0.0.0.0:14540,
its _kill_stale_servers() pkill-9's EVERY mavsdk_server on the box, and it
builds System() internally so the gRPC port cannot be injected; the S6 stub
note that said "wraps drone_control.Drone.connect()" was STALE — overridden by
sim_sessions.md recap §2). Every fix, per convention 7:

- drone_control.py:55-63 arm_and_takeoff's BLIND 20 s sleep -> telemetry-polled
  climb to >= 0.9x target altitude with a hard deadline.
- drone_control.py:61 sends VelocityNedYaw(0,0,0, yaw_deg=0.0) before offboard
  start — that commands an absolute YAW SNAP TO NORTH (invisible in SITL only
  because vehicles spawn facing north). Replaced by ONE PositionNedYaw hold
  setpoint at the CURRENT pose/heading.
- drone_control.py:10-23 _kill_stale_servers global pkill (lethal to the other
  drones' servers mid-mission — recap §3) -> targeted
  pkill -9 -f "mavsdk_server.*-p <grpc_port>( |$)"; the SIM-1 dummy-server
  drill proves a server on another port SURVIVES this cleanup.
- drone_control.py:86-142 rotate_to_yaw velocity-setpoint PID: opens a NEW
  telemetry subscription EVERY 0.1 s iteration (get_yaw), has no deadline, and
  hand-tuned gains can oscillate around the tolerance. REPLACED (reviewed
  deviation from the SIM-1 handover, user-approved) by position-setpoint yaw:
  PX4's own controller slews yaw on PositionNedYaw setpoints
  (MPC_YAWRAUTO_MAX, default 45 deg/s), giving rotate() the exact same
  set-target/poll-arrival/deadline shape as move() and ONE setpoint type for
  the whole offboard session. CAVEAT (documented contract): PX4 takes the
  SHORTEST arc to an absolute yaw setpoint, so rotate(270) CCW executes as
  90 CW — the FINAL HEADING honors the contract; mission logic composes
  <= 180 deg steps in practice.
- qualifier_run.py:268-331 _go_to_waypoint (the PROVEN 10 Hz position-setpoint
  stream + arrival poll) gains the hard deadline it lacked -> FlightTimeout
  naming drone, move, waypoint, remaining distance, elapsed, what-to-check.
- get_position_with_task.py SharedState/position_monitor_task pattern kept,
  hardened: stream tasks that END (PX4 killed -> the MAVSDK streams just stop
  yielding, NO exception at any call site) set a loud dead-flag with the
  reason; every command poll checks it, so a kill mid-move becomes a typed
  failure in ~1 s instead of a silent hang (the kill-drill path).
- mapping_drone.py:357-373 land sequence (offboard stop -> action.land -> poll
  in_air False -> disarm) kept, with deadlines and typed errors.

Offboard keep-alive (verified vs the MAVSDK offboard guide): mavsdk_server
AUTO-RESENDS the last setpoint at 20 Hz (PX4 requires >= 2 Hz), so idle gaps
between commands (guard holds, Wait actions, hover) cannot trip the
offboard-loss failsafe while the server lives — no background setpoint-
streamer task exists here on purpose. The arrival loops still re-send their
setpoint each poll tick (the field-proven qualifier pattern, belt and
suspenders). Server death is detected via the dead-flag/staleness paths.

mavsdk imports are METHOD-LOCAL: this module must import and construct on the
Windows dev venv where mavsdk is not installed (the pure helpers below are
unit-tested there against DeadReckoner — the hypothesis property gate).

Blanket exception catching: this file is on the tests/test_conventions.py
whitelist (reviewed S6/SIM-1 widening, user-approved) for EXACTLY three
never-raise/never-silent sites (four catch blocks — emergency_land has two),
each logging the full traceback:
emergency_land per-step swallows (the ABC names emergency_land the one place
in the flight stack allowed to swallow), disconnect teardown, and the
telemetry-stream wrapper (a background task dying silently would be
invisible). asyncio.CancelledError is BaseException on 3.11 and passes
through untouched.

Session: S6, executed as SIM-1 (V1 gate) + SIM-2 (3x swarm).
"""
from __future__ import annotations

import asyncio
import math
import subprocess
import sys
import time
import traceback
from typing import List, Optional, Tuple

from finals.errors import FlightError, FlightTimeout
from finals.flight.adapter import FlightAdapter
from finals.flight.dead_reckon import normalize_yaw_deg
from finals.types import Direction, PositionQuality, Telemetry

#: Default client-side gRPC port for a single-drone sitl config
#: (instance i uses 50051 + i; see finals.config.resolve_sitl_endpoint).
DEFAULT_GRPC_PORT = 50051


def _require_finite(value: float, what: str) -> float:
    """NaN/Inf reaching the setpoint math would poison the NED target with no
    exception anywhere (same silent-corruption class dead_reckon.py guards);
    mirrored here because that helper is module-private there."""
    if not isinstance(value, (int, float)) or isinstance(value, bool) \
            or not math.isfinite(value):
        raise ValueError(
            f"sitl_adapter: {what} must be a finite number, got {value!r} — "
            f"check the upstream computation that produced it")
    return float(value)


def _body_offset_to_ned(direction: Direction, distance_cm: float,
                        psi_ned_deg: float) -> Tuple[float, float, float]:
    """Body-frame relative move -> NED delta (dN_m, dE_m, dDown_m) through the
    CURRENT NED yaw psi (CW-positive, the MAVSDK convention). cm -> m happens
    at THIS boundary (the FlightAdapter contract is cm; setpoints are m).

    Derivation (detection_to_world.py project_pixel_to_world, the same source
    dead_reckon.py reduces): north += cos(psi)*Xb - sin(psi)*Yb;
    east += sin(psi)*Xb + cos(psi)*Yb with Xb body-forward, Yb body-right:
        FORWARD d: (dN, dE) = ( d*cos(psi),  d*sin(psi))
        RIGHT   d: (dN, dE) = (-d*sin(psi),  d*cos(psi))
        BACK/LEFT = the negations; UP/DOWN move NED down by -/+ d.
    Equivalence with DeadReckoner's CCW+ deltas under psi = -yaw_deg is
    test-PINNED (tests/test_sitl_adapter.py hypothesis property) — a sign
    error here flies the square mirror-imaged.

    Pure stdlib math; importable and unit-testable WITHOUT mavsdk.
    """
    _require_finite(distance_cm, "distance_cm")
    if distance_cm <= 0:
        raise ValueError(
            f"sitl_adapter: distance_cm must be > 0, got {distance_cm!r} — "
            f"direction encodes the sign (check the phase/servo math)")
    _require_finite(psi_ned_deg, "psi_ned_deg")
    d_m = distance_cm / 100.0
    if direction is Direction.UP:
        return (0.0, 0.0, -d_m)
    if direction is Direction.DOWN:
        return (0.0, 0.0, d_m)
    psi = math.radians(psi_ned_deg)
    cos_p, sin_p = math.cos(psi), math.sin(psi)
    if direction is Direction.FORWARD:
        return (d_m * cos_p, d_m * sin_p, 0.0)
    if direction is Direction.BACK:
        return (-d_m * cos_p, -d_m * sin_p, 0.0)
    if direction is Direction.RIGHT:
        return (-d_m * sin_p, d_m * cos_p, 0.0)
    if direction is Direction.LEFT:
        return (d_m * sin_p, -d_m * cos_p, 0.0)
    raise ValueError(
        f"sitl_adapter: unsupported Direction {direction!r} — "
        f"FORWARD/BACK/LEFT/RIGHT/UP/DOWN are the known values")


def _rotate_target_psi(cur_psi_deg: float, angle_ccw_deg: float) -> float:
    """Absolute NED yaw target for a relative CCW+ rotation (the pyhulax
    contract): NED yaw is CW-positive, so +CCW DECREASES psi. Normalized to
    (-180, 180] by the test-pinned dead_reckon helper. Pure; no mavsdk."""
    _require_finite(cur_psi_deg, "cur_psi_deg")
    _require_finite(angle_ccw_deg, "angle_ccw_deg")
    return normalize_yaw_deg(cur_psi_deg - angle_ccw_deg)


def _yaw_error_deg(target_deg: float, current_deg: float) -> float:
    """Shortest signed yaw error in (-180, 180] (adapted from
    drone_control.py _yaw_error, whose while-loops normalize_yaw_deg already
    implements with the -180 edge pinned). Pure; no mavsdk."""
    return normalize_yaw_deg(target_deg - current_deg)


class _TelemetryState:
    """Latest-known stream values + per-field monotonic stamps, written by the
    poller tasks and read by commands/telemetry(). Single event loop, plain
    attributes — no locking needed (the agent guarantees one in-flight command
    and asyncio gives single-threaded access)."""

    __slots__ = ("north_m", "east_m", "down_m", "pos_ts",
                 "psi_deg", "psi_ts", "battery_pct", "battery_ts",
                 "in_air", "in_air_ts", "armed", "armed_ts", "dead")

    def __init__(self) -> None:
        self.north_m: Optional[float] = None
        self.east_m: Optional[float] = None
        self.down_m: Optional[float] = None
        self.pos_ts: Optional[float] = None
        self.psi_deg: Optional[float] = None
        self.psi_ts: Optional[float] = None
        self.battery_pct: Optional[float] = None
        self.battery_ts: Optional[float] = None
        self.in_air: Optional[bool] = None
        self.in_air_ts: Optional[float] = None
        self.armed: Optional[bool] = None
        self.armed_ts: Optional[float] = None
        #: Reason string once ANY stream ends/dies — never silent (kill drill).
        self.dead: Optional[str] = None


class MavsdkSitlAdapter(FlightAdapter):
    """PX4-SITL FlightAdapter over MAVSDK. See the module docstring for the
    vendored sources, fixes, and the no-streamer design. Constructor performs
    NO I/O and imports NO mavsdk — wiring (main._build_adapter) constructs it
    on any machine; connect() is where the SDK enters."""

    def __init__(self, drone_id: str, *,
                 sitl_address: str,
                 grpc_port: int = DEFAULT_GRPC_PORT,
                 arrival_m: float = 0.15,
                 yaw_tol_deg: float = 2.0,
                 fresh_s: float = 1.0,
                 poll_period_s: float = 0.1):
        if not isinstance(drone_id, str) or not drone_id:
            raise ValueError(
                f"MavsdkSitlAdapter: drone_id must be a non-empty str, got "
                f"{drone_id!r} — check the wiring")
        super().__init__(drone_id)
        if not isinstance(sitl_address, str) or not sitl_address:
            raise ValueError(
                f"MavsdkSitlAdapter({drone_id!r}): sitl_address must be a "
                f"non-empty str like 'udpin://0.0.0.0:14540', got "
                f"{sitl_address!r} — check config sitl_address / the "
                f"per-drone field")
        if (not isinstance(grpc_port, int) or isinstance(grpc_port, bool)
                or not 1024 <= grpc_port <= 65535):
            raise ValueError(
                f"MavsdkSitlAdapter({drone_id!r}): grpc_port must be an int "
                f"in [1024, 65535], got {grpc_port!r} — instance i uses "
                f"{DEFAULT_GRPC_PORT}+i (check config mavsdk_grpc_port)")
        for name, value in (("arrival_m", arrival_m),
                            ("yaw_tol_deg", yaw_tol_deg),
                            ("fresh_s", fresh_s),
                            ("poll_period_s", poll_period_s)):
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(
                    f"MavsdkSitlAdapter({drone_id!r}): {name} must be finite "
                    f"and > 0, got {value!r}")

        self._sitl_address = sitl_address
        self._grpc_port = grpc_port
        self._arrival_m = float(arrival_m)
        self._yaw_tol_deg = float(yaw_tol_deg)
        self._fresh_s = float(fresh_s)
        self._poll_period_s = float(poll_period_s)

        self._system = None                       # mavsdk System, set in connect
        self._state = _TelemetryState()
        self._poller_tasks: List["asyncio.Task"] = []
        self._connected = False
        self._ever_connected = False
        #: errors.py FlightTimeout contract: "adapter marks itself degraded".
        #: Cleared by a successful connect() (reconnect = operator fix).
        self.degraded = False
        self._offboard_active = False
        self._home_down_m: Optional[float] = None
        self._last_setpoint: Optional[Tuple[float, float, float, float]] = None
        self._final_snapshot: Optional[Telemetry] = None

    # ---------------- small helpers ----------------
    def _log(self, msg: str) -> None:
        print(f"[MavsdkSitlAdapter] {self.drone_id}: {msg}",
              file=sys.stderr, flush=True)

    def _check_hint(self) -> str:
        return (f"check the PX4 instance for {self._sitl_address} "
                f"(bash sim/launch_sitl.sh status; tail sim/run/px4_*.log) "
                f"and mavsdk_server on gRPC {self._grpc_port}")

    def _flight_error(self, detail: str, exc: BaseException) -> FlightError:
        return FlightError(
            f"{self.drone_id}: {detail} failed — "
            f"{type(exc).__name__}: {exc} — {self._check_hint()}")

    def _timeout(self, detail: str, elapsed_s: float, extra: str = "") -> FlightTimeout:
        self.degraded = True
        return FlightTimeout(
            f"{self.drone_id}: {detail} exceeded its deadline after "
            f"{elapsed_s:.1f} s{extra} — {self._check_hint()}")

    def _gate_connected(self, detail: str) -> None:
        if not self._connected:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — adapter not connected "
                f"— call connect() first (check startup/wiring order)")

    def _gate_not_degraded(self, detail: str) -> None:
        if self.degraded:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — adapter degraded after "
                f"a FlightTimeout (the SDK may still be executing the "
                f"previous command) — safe-down and re-connect() to clear")

    def _gate_offboard(self, detail: str) -> None:
        if not self._offboard_active:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — offboard not active "
                f"(not flying) — takeoff() first (check the phase ordering)")

    async def _bounded(self, coro, deadline: float, t0: float,
                       detail: str, stage: str):
        """One SDK call under the command's REMAINING deadline (convention 2:
        every awaited op is bounded inside the adapter — a wedged-but-
        listening server must surface as the adapter's own typed timeout,
        not the agent's mislabeled +2 s grace watchdog)."""
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            coro.close()              # avoid the never-awaited warning
            raise self._timeout(detail, time.monotonic() - t0,
                                f" before {stage} (deadline already spent)")
        try:
            return await asyncio.wait_for(coro, remaining)
        except asyncio.TimeoutError:
            raise self._timeout(detail, time.monotonic() - t0,
                                f" during {stage}") from None

    def _check_alive_fresh(self, detail: str) -> None:
        """The kill-drill detector: poller dead-flag first (fires ~1 s after a
        PX4 kill), then position/yaw staleness. Both raise typed FlightError
        long before the agent's 5 s backstop."""
        st = self._state
        if st.dead is not None:
            raise FlightError(
                f"{self.drone_id}: {detail} aborted — {st.dead} — PX4 "
                f"instance dead? {self._check_hint()}")
        if st.pos_ts is None or st.psi_ts is None:
            raise FlightError(
                f"{self.drone_id}: {detail} aborted — no position/attitude "
                f"telemetry received yet — {self._check_hint()}")
        age_s = time.monotonic() - min(st.pos_ts, st.psi_ts)
        if age_s > self._fresh_s:
            raise FlightError(
                f"{self.drone_id}: {detail} aborted — telemetry is STALE "
                f"(age {age_s:.2f} s > {self._fresh_s:.2f} s) — stream "
                f"stalled; {self._check_hint()}")

    def _cleanup_stale_server(self) -> None:
        """Targeted stale-server cleanup: ONLY this adapter's gRPC port — a
        global pkill would kill the other drones' servers (recap §3; the
        root drone_control.py bug). Best-effort: pkill absent (Windows) or
        slow is tolerated — a stale bind then surfaces in connect() with its
        own actionable error. The ( |$) anchor keeps -p 50051 from matching
        -p 505510."""
        pattern = f"mavsdk_server.*-p {self._grpc_port}( |$)"
        try:
            subprocess.run(["pkill", "-9", "-f", pattern], check=False,
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=2.0)
        except (FileNotFoundError, subprocess.TimeoutExpired) as e:
            self._log(f"stale-server cleanup skipped ({type(e).__name__}: "
                      f"{e}) — continuing; a stale bind will surface in "
                      f"connect() loudly")

    # ---------------- telemetry pollers ----------------
    async def _run_stream(self, name: str, stream_fn) -> None:
        """Wrapper for one telemetry stream task. A stream that ENDS (PX4 or
        mavsdk_server died — MAVSDK streams stop yielding with NO exception)
        or RAISES sets the loud dead-flag; never silent. CancelledError is
        BaseException (3.11) and passes through for disconnect()'s teardown."""
        try:
            await stream_fn()
        except Exception:  # whitelisted site 3 — logged, flagged, never silent
            tb = traceback.format_exc()
            self._state.dead = (self._state.dead
                                or f"{name} stream DIED: see stderr")
            self._log(f"{name} stream DIED:\n{tb}")
        else:
            self._state.dead = (self._state.dead
                                or f"{name} stream ENDED (PX4/mavsdk_server "
                                   f"gone?)")
            self._log(f"{name} stream ENDED — {self._state.dead}")

    def _start_pollers(self) -> None:
        st = self._state
        tel = self._system.telemetry

        async def _positions():
            async for pv in tel.position_velocity_ned():
                p = pv.position
                st.north_m, st.east_m, st.down_m = p.north_m, p.east_m, p.down_m
                st.pos_ts = time.monotonic()

        async def _attitude():
            async for att in tel.attitude_euler():
                st.psi_deg = att.yaw_deg
                st.psi_ts = time.monotonic()

        async def _battery():
            async for b in tel.battery():
                st.battery_pct = b.remaining_percent
                st.battery_ts = time.monotonic()

        async def _in_air():
            async for flying in tel.in_air():
                st.in_air = flying
                st.in_air_ts = time.monotonic()

        async def _armed():
            async for armed in tel.armed():
                st.armed = armed
                st.armed_ts = time.monotonic()

        loop = asyncio.get_running_loop()
        for name, fn in (("position_velocity_ned", _positions),
                         ("attitude_euler", _attitude),
                         ("battery", _battery),
                         ("in_air", _in_air),
                         ("armed", _armed)):
            self._poller_tasks.append(loop.create_task(
                self._run_stream(name, fn),
                name=f"sitl-poll:{self.drone_id}:{name}"))

    async def _stop_pollers(self) -> None:
        tasks, self._poller_tasks = self._poller_tasks, []
        for t in tasks:
            t.cancel()
        if tasks:
            # Bounded (convention 3); cancelled tasks resolve immediately.
            await asyncio.wait(tasks, timeout=5.0)

    # ---------------- FlightAdapter contract ----------------
    async def connect(self, timeout_s: float = 10.0) -> None:
        detail = f"connect({self._sitl_address}, gRPC {self._grpc_port})"
        deadline = time.monotonic() + timeout_s
        t0 = time.monotonic()

        if self._system is not None:
            # Reconnect after a failure: tear the old link down first. NOTE:
            # the stale-server cleanup below kills THIS adapter's previous
            # mavsdk_server — reconnecting a HEALTHY flying link would cut
            # it; the only documented path here is safe-down -> reconnect.
            await self._stop_pollers()
            self._system = None
            self._connected = False
        # Flight state resets with the link: a reconnect while the adapter
        # believed it was flying must not leave _gate_offboard lying (review
        # finding 4) — offboard is NOT active on a fresh link.
        self._offboard_active = False
        self._last_setpoint = None
        self._home_down_m = None
        self._state = _TelemetryState()

        self._cleanup_stale_server()
        await asyncio.sleep(0.5)     # port release after the kill (bounded;
                                     # drone_control.py:47 precedent)

        try:
            from mavsdk import System
        except ImportError as e:
            raise FlightError(
                f"{self.drone_id}: {detail} failed — mavsdk is not installed "
                f"in this environment ({e}) — the sitl profile cannot fly "
                f"without it (VM: pip install mavsdk into .venv)") from e
        import grpc
        try:
            self._system = System(port=self._grpc_port)
            # System.connect() awaits the spawned server's gRPC channel and
            # waits FOREVER if that server dies or fails to bind (stale bind
            # the pkill missed, bad binary) — the adapter must honor its OWN
            # deadline, not lean on the agent's grace watchdog.
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise asyncio.TimeoutError
            await asyncio.wait_for(
                self._system.connect(system_address=self._sitl_address),
                remaining)

            async def _wait_connected():
                async for state in self._system.core.connection_state():
                    if state.is_connected:
                        return
                raise FlightError(
                    f"{self.drone_id}: {detail} failed — connection_state "
                    f"stream ENDED before a heartbeat (mavsdk_server gone?) "
                    f"— {self._check_hint()}")

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise asyncio.TimeoutError
            await asyncio.wait_for(_wait_connected(), remaining)
        except asyncio.TimeoutError:
            raise self._timeout(
                detail, time.monotonic() - t0,
                " waiting for a MAVLink heartbeat") from None
        except (OSError, RuntimeError) as e:
            # mavsdk-python raises these when the server binary cannot spawn.
            raise self._flight_error(f"{detail} mavsdk_server spawn", e) from e
        except grpc.RpcError as e:
            raise self._flight_error(detail, e) from e

        # High-rate position/attitude (~10 Hz) so freshness gates and arrival
        # polls see live data. Best-effort: a refusal only lowers the rate.
        from mavsdk.telemetry import TelemetryError
        tel = self._system.telemetry
        for rate_name in ("set_rate_position_velocity_ned",
                          "set_rate_attitude_euler"):
            rate_fn = getattr(tel, rate_name, None)
            if rate_fn is None:
                self._log(f"{rate_name} not in this mavsdk build — "
                          f"using default stream rate")
                continue
            try:
                # Clamped to the connect deadline (review finding 3): a slow
                # rate-setter must not overrun the command's own budget.
                await asyncio.wait_for(
                    rate_fn(10.0),
                    min(5.0, max(0.1, deadline - time.monotonic())))
            except (TelemetryError, asyncio.TimeoutError) as e:
                self._log(f"{rate_name}(10 Hz) refused "
                          f"({type(e).__name__}: {e}) — default rate")

        self._start_pollers()

        # First position+yaw must arrive before connect() may succeed —
        # everything downstream assumes the state is live.
        while self._state.pos_ts is None or self._state.psi_ts is None:
            if self._state.dead is not None:
                raise FlightError(
                    f"{self.drone_id}: {detail} failed — {self._state.dead} "
                    f"— {self._check_hint()}")
            if time.monotonic() >= deadline:
                raise self._timeout(
                    detail, time.monotonic() - t0,
                    " waiting for first position/attitude telemetry")
            await asyncio.sleep(0.05)

        self._connected = True
        self._ever_connected = True
        self.degraded = False        # reconnect = operator fix (mock parity)
        self._final_snapshot = None
        self._log(f"connected in {time.monotonic() - t0:.1f} s "
                  f"(gRPC {self._grpc_port})")

    async def disconnect(self) -> None:
        """Never raises. Captures the final telemetry snapshot (post-run
        forensics — mock parity), tears the pollers down, drops the System.
        mavsdk-python has no public server close; the NEXT run's targeted
        cleanup is the proven recovery for the orphaned server."""
        try:
            if self._connected and self._state.pos_ts is not None \
                    and self._state.psi_ts is not None:
                self._final_snapshot = self._snapshot()
            self._connected = False
            await self._stop_pollers()
            self._system = None
        except Exception:  # whitelisted site 2 — disconnect must never raise
            self._log(f"disconnect teardown error (link abandoned):\n"
                      f"{traceback.format_exc()}")

    async def takeoff(self, height_cm: int = 80, timeout_s: float = 30.0) -> None:
        detail = f"takeoff({height_cm} cm)"
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        if not (isinstance(height_cm, (int, float))
                and not isinstance(height_cm, bool)
                and math.isfinite(height_cm) and height_cm > 0):
            raise FlightError(
                f"{self.drone_id}: {detail} refused — {height_cm!r} cm is "
                f"not a physically executable takeoff height (must be finite "
                f"and > 0) — check the phase math")
        if self._state.in_air:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — already flying — "
                f"check the phase logic (double takeoff is a mission bug)")

        deadline = time.monotonic() + timeout_s
        t0 = time.monotonic()
        height_m = height_cm / 100.0

        from mavsdk.action import ActionError
        from mavsdk.offboard import OffboardError, PositionNedYaw
        import grpc

        # EKF/health-ready WITH DEADLINE before arming (drone_control arms
        # blind; multi-instance load slows EKF settle — SIM-0 resource notes).
        async def _wait_health():
            async for h in self._system.telemetry.health():
                if (h.is_global_position_ok and h.is_home_position_ok
                        and h.is_armable):
                    return
            raise FlightError(
                f"{self.drone_id}: {detail} failed — the health stream "
                f"ENDED before EKF readiness (mavsdk_server gone?) — "
                f"{self._check_hint()}")

        try:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise asyncio.TimeoutError
            await asyncio.wait_for(_wait_health(), remaining)
        except asyncio.TimeoutError:
            raise self._timeout(
                detail, time.monotonic() - t0,
                " waiting for EKF health (global+home+armable; settle is "
                "slow under multi-instance load)") from None
        except grpc.RpcError as e:
            raise self._flight_error(f"{detail} health poll", e) from e
        ready_s = time.monotonic() - t0
        self._log(f"health ready in {ready_s:.1f} s")

        self._check_alive_fresh(detail)
        home_down_m = self._state.down_m
        self._home_down_m = home_down_m

        try:
            await self._bounded(
                self._system.action.set_takeoff_altitude(height_m),
                deadline, t0, detail, "set_takeoff_altitude")
            await self._bounded(self._system.action.arm(),
                                deadline, t0, detail, "arm")
            await self._bounded(self._system.action.takeoff(),
                                deadline, t0, detail, "takeoff command")
        except (ActionError, grpc.RpcError) as e:
            raise self._flight_error(detail, e) from e

        # Telemetry-polled climb (replaces the blind 20 s sleep).
        target_gain_m = 0.9 * height_m
        while True:
            self._check_alive_fresh(detail)
            gained_m = -(self._state.down_m - home_down_m)
            if gained_m >= target_gain_m:
                break
            if time.monotonic() >= deadline:
                raise self._timeout(
                    detail, time.monotonic() - t0,
                    f" climbing (reached {gained_m:.2f} m of "
                    f"{height_m:.2f} m)")
            await asyncio.sleep(self._poll_period_s)

        # Offboard entry: ONE hold setpoint at the current heading, target
        # altitude = the COMMANDED height (not the 0.9x poll threshold —
        # offboard finishes the climb). Then start offboard. The yaw-snap fix
        # (module docstring) lives exactly here.
        hold = (self._state.north_m, self._state.east_m,
                home_down_m - height_m, self._state.psi_deg)
        try:
            await self._bounded(
                self._system.offboard.set_position_ned(PositionNedYaw(*hold)),
                deadline, t0, detail, "hold setpoint")
            await asyncio.sleep(0.3)   # a few auto-resent frames pre-start
            await self._bounded(self._system.offboard.start(),
                                deadline, t0, detail, "offboard start")
        except (OffboardError, grpc.RpcError) as e:
            raise self._flight_error(f"{detail} offboard start", e) from e
        self._last_setpoint = hold
        self._offboard_active = True

        # Finish the climb under offboard before returning: exiting at the
        # 0.9x poll threshold would let the NEXT move re-anchor its target to
        # the shortfall and fly the whole mission below band (review finding
        # 2) — "returns when airborne at altitude" means AT the commanded
        # height, within the arrival tolerance.
        target_down_m = home_down_m - height_m
        while abs(self._state.down_m - target_down_m) >= self._arrival_m:
            self._check_alive_fresh(detail)
            if time.monotonic() >= deadline:
                raise self._timeout(
                    detail, time.monotonic() - t0,
                    f" settling to commanded height (at "
                    f"{-(self._state.down_m - home_down_m):.2f} m of "
                    f"{height_m:.2f} m)")
            await asyncio.sleep(self._poll_period_s)
        self._log(f"airborne at {-(self._state.down_m - home_down_m):.2f} m "
                  f"in {time.monotonic() - t0:.1f} s, offboard active")

    async def land(self, timeout_s: float = 30.0) -> None:
        detail = "land()"
        self._gate_connected(detail)
        # No degraded/flying gate: land is the safe-down path, "safe to call
        # repeatedly" per the ABC.
        deadline = time.monotonic() + timeout_s
        t0 = time.monotonic()

        from mavsdk.action import ActionError
        from mavsdk.offboard import OffboardError
        import grpc

        try:
            await self._bounded(self._system.offboard.stop(),
                                deadline, t0, detail, "offboard stop")
        except OffboardError as e:
            # Not active (already stopped / never started) is a normal
            # repeated-land path; anything else still surfaces below via
            # action.land.
            self._log(f"offboard.stop: {e} (continuing to action.land)")
        except grpc.RpcError as e:
            raise self._flight_error(f"{detail} offboard stop", e) from e
        self._offboard_active = False
        self._last_setpoint = None

        try:
            await self._bounded(self._system.action.land(),
                                deadline, t0, detail, "land command")
        except (ActionError, grpc.RpcError) as e:
            raise self._flight_error(detail, e) from e

        while self._state.in_air is not False:
            # Position/attitude keep streaming on the ground, so this cannot
            # false-trip — and a PX4 kill mid-land surfaces here in ~1 s
            # instead of burning the full deadline (review finding 5).
            self._check_alive_fresh(detail)
            if time.monotonic() >= deadline:
                raise self._timeout(
                    detail, time.monotonic() - t0,
                    f" waiting for touchdown "
                    f"(in_air={self._state.in_air!r})")
            await asyncio.sleep(self._poll_period_s)

        # Disarm: PX4 auto-disarms after landing (COM_DISARM_LAND); the
        # explicit disarm covers a disabled auto-disarm, and an ActionError
        # is tolerated ONLY when the vehicle is in fact already disarmed.
        try:
            await self._bounded(self._system.action.disarm(),
                                deadline, t0, detail, "disarm")
        except ActionError as e:
            if self._state.armed is False:
                self._log(f"disarm: {e} (already auto-disarmed — ok)")
            else:
                raise self._flight_error(f"{detail} disarm", e) from e
        except grpc.RpcError as e:
            raise self._flight_error(f"{detail} disarm", e) from e
        self._log(f"landed + disarmed in {time.monotonic() - t0:.1f} s")

    async def move(self, direction: Direction, distance_cm: int,
                   timeout_s: float = 15.0) -> None:
        detail = f"move({direction.name}, {distance_cm} cm)"
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        self._gate_offboard(detail)
        self._check_alive_fresh(detail)    # guarantees psi_deg is live below
        try:
            dn_m, de_m, dd_m = _body_offset_to_ned(
                direction, distance_cm, self._state.psi_deg)
        except ValueError as e:
            # _body_offset_to_ned validates direction + distance; surface its
            # actionable reason as the typed flight refusal.
            raise FlightError(
                f"{self.drone_id}: {detail} refused — {e}") from e
        target = (self._state.north_m + dn_m,
                  self._state.east_m + de_m,
                  self._state.down_m + dd_m,
                  self._state.psi_deg)
        await self._fly_to_setpoint(detail, target, timeout_s)

    async def rotate(self, angle_deg: float, timeout_s: float = 15.0) -> None:
        detail = f"rotate({angle_deg:g} deg)"
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        self._gate_offboard(detail)
        if not (isinstance(angle_deg, (int, float))
                and not isinstance(angle_deg, bool)
                and math.isfinite(angle_deg)):
            raise FlightError(
                f"{self.drone_id}: {detail} refused — {angle_deg!r} deg is "
                f"not a physically executable rotation (must be finite)")
        self._check_alive_fresh(detail)
        target_psi = _rotate_target_psi(self._state.psi_deg, angle_deg)
        target = (self._state.north_m, self._state.east_m,
                  self._state.down_m, target_psi)
        await self._fly_to_setpoint(detail, target, timeout_s,
                                    yaw_only=True)

    async def _fly_to_setpoint(self, detail: str,
                               target: Tuple[float, float, float, float],
                               timeout_s: float, *,
                               yaw_only: bool = False) -> None:
        """The shared arrival loop (vendored qualifier_run.py:268-331 with the
        deadline it lacked): re-send the setpoint each tick (field-proven,
        belt-and-suspenders over the server's 20 Hz auto-resend), poll
        arrival, and run the kill-drill detectors every tick."""
        from mavsdk.offboard import OffboardError, PositionNedYaw
        import grpc

        deadline = time.monotonic() + timeout_s
        t0 = time.monotonic()
        tn, te, td, tpsi = target
        st = self._state
        while True:
            self._check_alive_fresh(detail)
            try:
                await self._bounded(
                    self._system.offboard.set_position_ned(
                        PositionNedYaw(tn, te, td, tpsi)),
                    deadline, t0, detail, "setpoint send")
            except (OffboardError, grpc.RpcError) as e:
                raise self._flight_error(f"{detail} setpoint send", e) from e

            if yaw_only:
                err = abs(_yaw_error_deg(tpsi, st.psi_deg))
                if err < self._yaw_tol_deg:
                    break
                progress = f" (yaw error {err:.1f} deg of tol " \
                           f"{self._yaw_tol_deg:g})"
            else:
                dist_m = math.sqrt((tn - st.north_m) ** 2
                                   + (te - st.east_m) ** 2
                                   + (td - st.down_m) ** 2)
                if dist_m < self._arrival_m:
                    break
                progress = (f" ({dist_m:.2f} m from waypoint "
                            f"N={tn:.2f} E={te:.2f} D={td:.2f})")

            if time.monotonic() >= deadline:
                raise self._timeout(detail, time.monotonic() - t0, progress)
            await asyncio.sleep(self._poll_period_s)

        self._last_setpoint = target
        self._log(f"{detail} complete in {time.monotonic() - t0:.1f} s")

    async def hover(self, duration_s: float) -> None:
        detail = f"hover({duration_s:g} s)"
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        self._gate_offboard(detail)
        if not (isinstance(duration_s, (int, float))
                and not isinstance(duration_s, bool)
                and math.isfinite(duration_s) and duration_s >= 0):
            raise FlightError(
                f"{self.drone_id}: {detail} refused — {duration_s!r} s is "
                f"not a physically executable hover duration")
        self._check_alive_fresh(detail)

        from mavsdk.offboard import OffboardError, PositionNedYaw
        import grpc
        # Re-assert the hold (independent of which command ran last); the
        # server auto-resends it for the whole window.
        hold = self._last_setpoint or (self._state.north_m,
                                       self._state.east_m,
                                       self._state.down_m,
                                       self._state.psi_deg)
        try:
            # hover() carries no timeout_s in the ABC — a fixed 10 s bound
            # keeps this single send inside convention 2 anyway.
            await asyncio.wait_for(
                self._system.offboard.set_position_ned(PositionNedYaw(*hold)),
                10.0)
        except asyncio.TimeoutError:
            raise self._timeout(detail, 10.0, " sending the hold setpoint") \
                from None
        except (OffboardError, grpc.RpcError) as e:
            raise self._flight_error(f"{detail} hold setpoint", e) from e
        self._last_setpoint = hold

        # Complete-or-raise: a hover that "completes" after PX4 died would be
        # a lie — sleep in slices, checking the kill detectors each slice.
        end = time.monotonic() + duration_s
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                break
            self._check_alive_fresh(detail)
            await asyncio.sleep(min(0.5, remaining))

    def telemetry(self) -> Telemetry:
        if not self._ever_connected:
            raise FlightError(
                f"{self.drone_id}: telemetry() refused — never connected, so "
                f"there is no honest state to report (a fabricated fresh "
                f"timestamp would defeat staleness guards) — call connect() "
                f"first")
        if not self._connected:
            if self._final_snapshot is None:
                raise FlightError(
                    f"{self.drone_id}: telemetry() — disconnected before any "
                    f"position/attitude telemetry arrived; nothing honest to "
                    f"report — {self._check_hint()}")
            return self._final_snapshot
        st = self._state
        if st.pos_ts is None or st.psi_ts is None:
            raise FlightError(
                f"{self.drone_id}: telemetry() — no position/attitude "
                f"received yet — {self._check_hint()}")
        return self._snapshot()

    def _snapshot(self) -> Telemetry:
        """ts = min(position, yaw) stamps ONLY: those are the action-relevant
        high-rate fields; battery/in_air arrive at ~1 Hz and folding them in
        would manufacture false staleness for the agent's 5 s backstop."""
        st = self._state
        return Telemetry(
            ts=min(st.pos_ts, st.psi_ts),
            battery_pct=st.battery_pct,
            altitude_m=-st.down_m,                       # NED down -> up-positive
            yaw_deg=normalize_yaw_deg(-st.psi_deg),      # CW+ NED -> CCW+ contract
            is_flying=st.in_air,
            position_m=(st.north_m, st.east_m, -st.down_m),
            position_quality=PositionQuality.MEASURED,
            raw={"down_m": st.down_m, "psi_ned_deg": st.psi_deg,
                 "armed": st.armed, "poller_dead": st.dead,
                 "home_down_m": self._home_down_m},
        )

    async def emergency_land(self) -> None:
        """Best-effort safe-down; NEVER raises (the one sanctioned swallow
        site in the flight stack — adapter.py ABC). Each step is bounded and
        any failure is logged with its full traceback, then the next step
        still runs: a refused offboard-stop must not block the land command."""
        self._offboard_active = False
        self._last_setpoint = None
        if self._system is None:
            self._log("emergency_land: never connected — nothing to command")
            return

        async def _stop_offboard():
            await self._system.offboard.stop()

        async def _land():
            await self._system.action.land()

        # Per-step bounds sized so stop+land ALWAYS fit inside the agent's
        # SMALLEST shielded outer budget (command_timeout_s 15 + 2 s grace =
        # 17 s: 5+5=10 s — review finding 7); the touchdown wait and disarm
        # are best-effort tail under larger budgets.
        for step, fn in (("offboard.stop", _stop_offboard),
                         ("action.land", _land)):
            try:
                await asyncio.wait_for(fn(), 5.0)
            except Exception:  # whitelisted site 1 — logged, never raised
                self._log(f"emergency_land {step} failed (continuing):\n"
                          f"{traceback.format_exc()}")

        # Bounded touchdown wait + disarm attempt, same never-raise policy.
        end = time.monotonic() + 10.0
        while self._state.in_air is not False and time.monotonic() < end \
                and self._state.dead is None:
            await asyncio.sleep(0.2)

        async def _disarm():
            await self._system.action.disarm()

        try:
            await asyncio.wait_for(_disarm(), 3.0)
        except Exception:  # whitelisted site 1 — PX4 refuses disarm in air;
            #                auto-disarm usually beat us; both fine, logged.
            self._log(f"emergency_land disarm attempt: "
                      f"{traceback.format_exc(limit=1)}")
        self._log(f"emergency_land finished (in_air={self._state.in_air}, "
                  f"armed={self._state.armed}, poller_dead="
                  f"{self._state.dead!r})")
