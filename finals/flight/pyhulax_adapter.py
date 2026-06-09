"""PyhulaxAdapter — the real-drone FlightAdapter (HULA over Wi-Fi) + FakeDroneAPI.

Implements the SAME relative-move contract the SITL VM runs (sitl_adapter.py),
on top of the blocking pyhulax SDK. One instance per drone. The whole module is
unit-tested with pyhulax NOT installed, via an injectable FakeDroneAPI + a
name-based exception mapper (real and fake SDK errors map identically), exactly
the seam sitl_adapter.py proved for mavsdk.

THE CHOKE POINT. pyhulax's commands BLOCK and Wi-Fi-dropout-mid-call behaviour
is UNDOCUMENTED (open research question). So EVERY blocking SDK call runs as
`await asyncio.wait_for(loop.run_in_executor(self._pool, fn), timeout_s)` with a
per-drone single-thread executor: commands serialize (one outstanding blocking
call), every command gets a hard deadline, and a hung drone cannot stall the
orchestrator or the other two drones. On TimeoutError the worker thread MAY
still finish the move on the airframe — so the adapter marks itself `degraded`,
raises FlightTimeout, and the agent safes the drone down (mock/sitl parity).

CAVEAT (documented, not hidden): a command that times out leaves its blocking
call running on the pool's single worker; the NEXT command queues behind it.
Command-path gates short-circuit on `degraded` BEFORE touching the pool, so the
agent's safe-down is not what queues — but emergency_land/disconnect DO touch
the pool and are bounded (wait_for); if the worker is truly wedged they time out
and fall back to the drone's own failsafe. enable_battery_failsafe() is called
ALWAYS at connect precisely so that backstop exists.

VENDORED-WITH-FIXES (convention 7) from hula_connection.py:29-37 (the audited
connect/LED/video sequence) + the pyhulax reference docs
(https://pyhulax.xenops.ae). Bugs in the example code, fixed here:
- hula_connection.py connects with NO deadline and NO battery failsafe — here
  connect() is wall-clock bounded and enable_battery_failsafe() runs ALWAYS.
- mapping_drone.py:129 infinite telemetry wait + module-level mutable globals
  (battery_remain, current_*) + blind sleeps -> a lock-guarded _TelemetryState
  written by a single bounded 2 Hz poller thread; every command checks a
  dead-flag + staleness (_check_alive_fresh) so a dropped link surfaces as a
  typed FlightError in ~1 s, never a silent hang.
- mapping_drone.py print-and-continue swallows -> typed finals.errors carrying
  drone_id + action + cause + what-to-check.

SDK imports are METHOD-LOCAL (this module is in tests/test_conventions.py
SDK_ALLOWED): it imports and constructs on the bare Windows/VM dev venv where
pyhulax is absent. Do NOT add pyhulax to requirements.txt.

Blanket exception catching: this file is on the test_conventions.py
EXCEPT_EXCEPTION_WHITELIST (reviewed S9 widening, user-approved) for EXACTLY the
three never-raise/never-silent sites, each logging a full traceback — the
telemetry-poller tick (a poller dying silently is the mapping_drone.py bug
class), emergency_land (the ABC names it the one sanctioned swallow site), and
disconnect teardown. asyncio.CancelledError is BaseException on 3.11 and passes
through untouched.

Units note: the contract is distance_cm/height_cm (pyhulax docs), but
hula_connection.py:45 shows move(FORWARD, 0.5) — contradiction. The onsite "unit
hop" preflight gate settles it; a fix touches only this adapter boundary
(commented at move(), never silently decided).

Session: S9.
"""
from __future__ import annotations

import asyncio
import math
import sys
import threading
import time
import traceback
from concurrent.futures import ThreadPoolExecutor
from typing import Optional, Tuple

from finals.errors import FlightError, FlightTimeout
from finals.flight.adapter import FlightAdapter
from finals.flight.dead_reckon import normalize_yaw_deg
from finals.types import Direction, PositionQuality, Telemetry


# ============================================================
# Fake pyhulax exceptions (test doubles)
# ============================================================
# Named EXACTLY as the pyhulax SDK exceptions so the name-based mapper
# (_map_sdk_error, by type(e).__name__) is identical whether the error came
# from a real DroneAPI or from FakeDroneAPI. These let the whole timeout /
# degrade / mapping path be unit-tested with pyhulax NOT installed.
class _PyhulaxFakeError(Exception):
    """Base for FakeDroneAPI's pyhulax-mirror exceptions (tests, no pyhulax)."""


class CommandTimeout(_PyhulaxFakeError):
    pass


class CommandRejected(_PyhulaxFakeError):
    pass


class NotReady(_PyhulaxFakeError):
    pass


class LowBattery(_PyhulaxFakeError):
    pass


class DroneConnectionError(_PyhulaxFakeError):
    pass


class TelemetryUnavailable(_PyhulaxFakeError):
    pass


#: SDK exception names the mapper treats as a hard SDK-side timeout (-> degraded
#: + FlightTimeout). Everything else maps to a plain FlightError.
_SDK_TIMEOUT_NAMES = frozenset({"CommandTimeout"})

#: pyhulax exception class names pulled into the runtime catch tuple (real path).
_PYHULAX_EXC_NAMES = (
    "CommandTimeout", "CommandRejected", "NotReady", "LowBattery",
    "DroneConnectionError", "TelemetryUnavailable",
)
#: best-effort SDK base classes (catch-all for unlisted pyhulax errors).
_PYHULAX_BASE_NAMES = ("PyhulaxError", "HulaError", "DroneError")


def _pyhulax_sdk_error_types() -> Tuple[type, ...]:
    """The catch tuple for `except self._sdk_errors`: the module-local fake base
    ALWAYS (the FakeDroneAPI path), PLUS the real pyhulax exception classes when
    importable. Method-local import: tests (no pyhulax) get just the fake base,
    so an UNMAPPED real exception would propagate and fail loud — never silently
    swallowed."""
    types = [_PyhulaxFakeError]
    mod = None
    for modpath in ("pyhulax.core.exceptions", "pyhulax.exceptions",
                    "pyhulax.core", "pyhulax"):
        try:
            mod = __import__(modpath, fromlist=["_"])
            break
        except ImportError:
            continue
    if mod is None:
        return tuple(types)
    for name in (*_PYHULAX_EXC_NAMES, *_PYHULAX_BASE_NAMES):
        t = getattr(mod, name, None)
        if isinstance(t, type) and issubclass(t, BaseException):
            types.append(t)
    return tuple(types)


def _real_drone_api_factory():
    """Default api_factory: build the real pyhulax DroneAPI. Method-local — the
    only place the real SDK is imported; tests inject api=FakeDroneAPI() and
    never reach here."""
    from pyhulax import DroneAPI
    return DroneAPI()


def _to_sdk_direction(direction: Direction):
    """finals Direction -> pyhulax Direction (value parity is pinned by
    test_types.py). Degrades gracefully when pyhulax is absent: returns the
    finals Direction so FakeDroneAPI receives the same value the real SDK would
    decode (its move() records whatever it is handed)."""
    try:
        from pyhulax.core import Direction as HulaDirection
    except ImportError:
        return direction
    return HulaDirection(direction.value)


def _as_float(value) -> Optional[float]:
    return None if value is None else float(value)


def _extract_yaw(orient) -> Optional[float]:
    """pyhulax Orientation -> yaw in degrees, best-effort. The exact attribute
    name (`yaw` vs `yaw_deg`) is an onsite-verify point; yaw_deg is Optional in
    Telemetry, so a miss is an honest None, never a wrong number."""
    if orient is None:
        return None
    for attr in ("yaw", "yaw_deg"):
        v = getattr(orient, attr, None)
        if v is not None:
            return float(v)
    return None


def _extract_flying(state) -> Optional[bool]:
    """pyhulax DroneState -> is_flying, best-effort (attribute name is an
    onsite-verify point). Used ONLY for the Telemetry report — command gating
    uses the adapter's own authoritative _airborne flag, not this."""
    if state is None:
        return None
    for attr in ("is_flying", "in_air", "flying"):
        v = getattr(state, attr, None)
        if isinstance(v, bool):
            return v
    return None


class _TelemetryState:
    """Latest poller values + a single monotonic stamp + the loud dead-flag.
    A threading.Lock guards it because the asyncio thread reads telemetry() /
    _check_alive_fresh concurrently with the poller thread's writes (unlike
    sitl_adapter, whose streams are all on the one event loop)."""

    __slots__ = ("lock", "battery_pct", "altitude_cm", "yaw_deg", "is_flying",
                 "ts", "dead")

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.battery_pct: Optional[float] = None
        self.altitude_cm: Optional[float] = None
        self.yaw_deg: Optional[float] = None
        self.is_flying: Optional[bool] = None
        #: monotonic stamp of the last successful poll tick (None until first).
        self.ts: Optional[float] = None
        #: reason string once the poller dies/loses the link — never silent.
        self.dead: Optional[str] = None


class PyhulaxAdapter(FlightAdapter):
    """Real-drone FlightAdapter over the blocking pyhulax SDK. See the module
    docstring for the choke point, the no-streamer design, and the vendored
    sources. Constructor does NO I/O and imports NO pyhulax — wiring
    (main._build_adapter) constructs it on any machine; connect() is where the
    SDK enters (the pool, the api, the poller)."""

    def __init__(self, drone_id: str, *,
                 ip: Optional[str] = None,
                 api=None,
                 api_factory=None,
                 poll_hz: float = 2.0,
                 fresh_s: float = 2.0,
                 hover_margin_s: float = 5.0):
        if not isinstance(drone_id, str) or not drone_id:
            raise ValueError(
                f"PyhulaxAdapter: drone_id must be a non-empty str, got "
                f"{drone_id!r} — check the wiring")
        super().__init__(drone_id)
        if ip is not None and (not isinstance(ip, str) or not ip):
            raise ValueError(
                f"PyhulaxAdapter({drone_id!r}): ip must be None or a non-empty "
                f"str like '192.168.1.50', got {ip!r} — discovery resolves it")
        for name, value in (("poll_hz", poll_hz),
                            ("fresh_s", fresh_s),
                            ("hover_margin_s", hover_margin_s)):
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(
                    f"PyhulaxAdapter({drone_id!r}): {name} must be finite and "
                    f"> 0, got {value!r}")

        self._ip = ip
        self._poll_hz = float(poll_hz)
        self._fresh_s = float(fresh_s)
        self._hover_margin_s = float(hover_margin_s)

        self._api = api
        self._api_factory = api_factory
        self._pool: Optional[ThreadPoolExecutor] = None

        self._state = _TelemetryState()
        self._poll_thread: Optional[threading.Thread] = None
        self._poll_stop = threading.Event()

        self._connected = False
        self._ever_connected = False
        #: errors.py FlightTimeout contract; cleared by a successful connect().
        self.degraded = False
        #: authoritative flying state (set by our takeoff/land, NOT get_state).
        self._airborne = False
        self._final_snapshot: Optional[Telemetry] = None
        #: runtime catch tuple — fake base always, real pyhulax types if present.
        self._sdk_errors = _pyhulax_sdk_error_types()

    # ---------------- small helpers ----------------
    def _log(self, msg: str) -> None:
        print(f"[PyhulaxAdapter] {self.drone_id}: {msg}",
              file=sys.stderr, flush=True)

    def _check_hint(self) -> str:
        return (f"check the Wi-Fi link to {self._ip} (SSID / signal / range), "
                f"drone power + battery, and that no other client grabbed it")

    def _timeout(self, detail: str, timeout_s: float, stage: str) -> FlightTimeout:
        self.degraded = True
        return FlightTimeout(
            f"{self.drone_id}: {detail} exceeded {timeout_s:.1f} s during "
            f"{stage} (the blocking SDK call may still be executing on the "
            f"drone) — {self._check_hint()}")

    def _map_sdk_error(self, detail: str, exc: BaseException) -> FlightError:
        name = type(exc).__name__
        if name in _SDK_TIMEOUT_NAMES:
            self.degraded = True
            return FlightTimeout(
                f"{self.drone_id}: {detail} timed out in the SDK "
                f"({name}: {exc}) — {self._check_hint()}")
        return FlightError(
            f"{self.drone_id}: {detail} failed — {name}: {exc} — "
            f"{self._check_hint()}")

    def _gate_connected(self, detail: str) -> None:
        if not self._connected:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — adapter not connected "
                f"— call connect() first (check startup/wiring order)")

    def _gate_not_degraded(self, detail: str) -> None:
        if self.degraded:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — adapter degraded after "
                f"a FlightTimeout (the previous blocking SDK call may still be "
                f"running on the drone) — safe-down and re-connect() to clear")

    def _check_alive_fresh(self, detail: str) -> None:
        """The kill detector: dead-flag first (poller died / link gone), then
        staleness. Raises a typed FlightError fast so a dropped link never
        becomes a silent hang."""
        with self._state.lock:
            dead = self._state.dead
            ts = self._state.ts
        if dead is not None:
            raise FlightError(
                f"{self.drone_id}: {detail} aborted — {dead} — drone link "
                f"dead? {self._check_hint()}")
        if ts is None:
            raise FlightError(
                f"{self.drone_id}: {detail} aborted — no telemetry received "
                f"yet — {self._check_hint()}")
        age_s = time.monotonic() - ts
        if age_s > self._fresh_s:
            raise FlightError(
                f"{self.drone_id}: {detail} aborted — telemetry is STALE "
                f"(age {age_s:.2f} s > {self._fresh_s:.2f} s) — link stalled; "
                f"{self._check_hint()}")

    # ---------------- the choke point ----------------
    async def _run_blocking(self, fn, timeout_s: float):
        """One blocking SDK call on the single-thread pool, under timeout_s.
        Lets asyncio.TimeoutError and the raw SDK errors propagate — callers
        map them (so connect() can branch on DroneConnectionError before the
        mapping flattens it)."""
        loop = asyncio.get_running_loop()
        return await asyncio.wait_for(
            loop.run_in_executor(self._pool, fn), timeout_s)

    async def _call(self, fn, timeout_s: float, detail: str, stage: str):
        """_run_blocking + the typed mapping (convention 2): TimeoutError ->
        FlightTimeout (+degraded); SDK error -> mapped finals.error."""
        try:
            return await self._run_blocking(fn, timeout_s)
        except asyncio.TimeoutError:
            raise self._timeout(detail, timeout_s, stage) from None
        except self._sdk_errors as e:
            raise self._map_sdk_error(detail, e) from e

    # ---------------- telemetry poller ----------------
    def _poll_loop(self) -> None:
        period = 1.0 / self._poll_hz
        while not self._poll_stop.is_set():
            if self._poll_tick():
                return                 # dead-flag set; stop polling
            self._poll_stop.wait(period)

    def _poll_tick(self) -> bool:
        """One 2 Hz reading of the immediate getters into _TelemetryState.
        Returns True (stop) once the loud dead-flag is set. A transient
        TelemetryUnavailable just SKIPS the tick (staleness handles a
        persistent gap); a link-level SDK error or anything unexpected sets the
        dead-flag — never silent."""
        api = self._api
        try:
            battery = api.get_battery()
            altitude_cm = api.get_altitude()
            orient = api.get_orientation()
            state = api.get_state()
        except self._sdk_errors as e:
            if type(e).__name__ == "TelemetryUnavailable":
                return False           # transient; let staleness accrue
            with self._state.lock:
                self._state.dead = (self._state.dead
                                    or f"telemetry poller stopped — "
                                       f"{type(e).__name__}: {e}")
            self._log(f"telemetry poller stopped — {type(e).__name__}: {e}\n"
                      f"{traceback.format_exc()}")
            return True
        except Exception:  # whitelisted site 1 — a poller must never die silent
            with self._state.lock:
                self._state.dead = (self._state.dead
                                    or "telemetry poller DIED (unexpected): "
                                       "see stderr")
            self._log(f"telemetry poller DIED (unexpected):\n"
                      f"{traceback.format_exc()}")
            return True
        with self._state.lock:
            self._state.battery_pct = _as_float(battery)
            self._state.altitude_cm = _as_float(altitude_cm)
            self._state.yaw_deg = _extract_yaw(orient)
            self._state.is_flying = _extract_flying(state)
            self._state.ts = time.monotonic()
        return False

    def set_target_ip(self, ip: str) -> None:
        """Resolve the target IP AFTER construction (preflight P3 applies the
        discovery result before P4 connect — see finals/preflight.py). Refused
        once connected: the IP is a pre-connect input, and silently re-pointing
        a live adapter would split flight and video across two links."""
        if not isinstance(ip, str) or not ip:
            raise ValueError(
                f"PyhulaxAdapter({self.drone_id!r}): set_target_ip(ip) needs a "
                f"non-empty str like '192.168.1.50', got {ip!r}")
        if self._connected:
            raise FlightError(
                f"{self.drone_id}: set_target_ip refused — already connected to "
                f"{self._ip}; disconnect() before re-pointing (IP is a "
                f"pre-connect input)")
        self._ip = ip

    # ---------------- FlightAdapter contract ----------------
    async def connect(self, timeout_s: float = 10.0) -> None:
        # Idempotent: preflight (P4) connects and leaves the link up for the
        # mission, then the agent's run() connect() must NOT re-handshake a live
        # link (the S9-deferred connect-before-stream-start ordering). A degraded
        # adapter still reconnects (the _gate_not_degraded "re-connect() to
        # clear" path) — the guard deliberately excludes it.
        if self._connected and not self.degraded:
            self._log("connect: already connected — no-op")
            return
        detail = f"connect({self._ip})"
        if self._ip is None:
            raise FlightError(
                f"{self.drone_id}: connect() refused — no target IP. Resolve "
                f"plane_id -> ip via finals.flight.discovery (preflight, S10) "
                f"and construct PyhulaxAdapter(ip=...) — check the wiring order")
        deadline = time.monotonic() + timeout_s
        t0 = time.monotonic()

        def _rem() -> float:
            return max(0.0, deadline - time.monotonic())

        if self._pool is None:
            self._pool = ThreadPoolExecutor(
                max_workers=1, thread_name_prefix=f"pyhulax-{self.drone_id}")
        if self._api is None:
            factory = self._api_factory or _real_drone_api_factory
            self._api = factory()
        api = self._api

        # Reset poller + flight state for a fresh link (reconnect = operator
        # fix; a stale dead-flag/airborne belief must not leak across links).
        self._poll_stop.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2.0)
            self._poll_thread = None
        self._poll_stop = threading.Event()
        self._state = _TelemetryState()
        self._airborne = False

        # connect, with ONE robust_connect retry on a connection-class failure
        # (the audited single retry — hula_connection.py reconnect, bounded).
        try:
            await self._run_blocking(lambda: api.connect(self._ip), _rem())
        except asyncio.TimeoutError:
            raise self._timeout(detail, timeout_s, "connect") from None
        except self._sdk_errors as e:
            if type(e).__name__ == "DroneConnectionError" and _rem() > 0:
                self._log(f"connect failed ({type(e).__name__}: {e}) — one "
                          f"robust_connect retry")
                try:
                    ok = await self._run_blocking(
                        lambda: api.robust_connect(self._ip), _rem())
                except asyncio.TimeoutError:
                    raise self._timeout(
                        detail, timeout_s, "robust_connect retry") from None
                except self._sdk_errors as e2:
                    raise self._map_sdk_error(detail, e2) from e2
                if ok is False:
                    raise FlightError(
                        f"{self.drone_id}: {detail} failed — robust_connect "
                        f"returned False (drone unreachable after retry) — "
                        f"{self._check_hint()}")
            else:
                raise self._map_sdk_error(detail, e) from e

        # Battery failsafe ALWAYS (the onboard backstop the choke-point caveat
        # leans on — hula_connection.py never sets it).
        await self._call(lambda: api.enable_battery_failsafe(),
                         _rem(), detail, "enable_battery_failsafe")

        # Start the 2 Hz poller; require a first reading before connect succeeds
        # (everything downstream assumes live state).
        self._poll_thread = threading.Thread(
            target=self._poll_loop, name=f"pyhulax-poll:{self.drone_id}",
            daemon=True)
        self._poll_thread.start()
        while True:
            with self._state.lock:
                dead = self._state.dead
                ts = self._state.ts
            if ts is not None:
                break
            if dead is not None:
                raise FlightError(
                    f"{self.drone_id}: {detail} failed — {dead} — "
                    f"{self._check_hint()}")
            if time.monotonic() >= deadline:
                raise self._timeout(
                    detail, timeout_s, "waiting for first telemetry")
            await asyncio.sleep(0.05)

        self._connected = True
        self._ever_connected = True
        self.degraded = False
        self._final_snapshot = None
        self._log(f"connected to {self._ip} in {time.monotonic() - t0:.1f} s")

    async def disconnect(self) -> None:
        """Never raises. Captures the final telemetry snapshot (post-run
        forensics — mock/sitl parity), stops the poller, best-effort
        api.disconnect(), then shuts the pool down so no worker leaks."""
        try:
            with self._state.lock:
                have_state = self._state.ts is not None
            if self._connected and self._final_snapshot is None and have_state:
                self._final_snapshot = self._snapshot()
            self._connected = False
            self._poll_stop.set()
            t = self._poll_thread
            if t is not None:
                t.join(timeout=2.0)
                self._poll_thread = None
            api = self._api
            if api is not None and self._pool is not None:
                # Bounded best-effort: a wedged worker must not hang teardown.
                loop = asyncio.get_running_loop()
                await asyncio.wait_for(
                    loop.run_in_executor(self._pool, api.disconnect), 5.0)
        except Exception:  # whitelisted site 2 — disconnect must never raise
            self._log(f"disconnect teardown error (link abandoned):\n"
                      f"{traceback.format_exc()}")
        finally:
            if self._pool is not None:
                self._pool.shutdown(wait=False, cancel_futures=True)
                self._pool = None

    async def takeoff(self, height_cm: int = 80, timeout_s: float = 30.0) -> None:
        detail = f"takeoff({height_cm} cm)"
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        if not (isinstance(height_cm, (int, float))
                and not isinstance(height_cm, bool)
                and math.isfinite(height_cm) and height_cm > 0):
            raise FlightError(
                f"{self.drone_id}: {detail} refused — {height_cm!r} cm is not "
                f"a physically executable takeoff height (must be finite and "
                f"> 0) — check the phase math")
        if self._airborne:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — already flying — check "
                f"the phase logic (double takeoff is a mission bug)")
        self._check_alive_fresh(detail)
        await self._call(lambda: self._api.takeoff(height_cm=height_cm),
                         timeout_s, detail, "takeoff")
        self._airborne = True

    async def land(self, timeout_s: float = 30.0) -> None:
        detail = "land()"
        self._gate_connected(detail)
        # No degraded/flying gate: land is the safe-down path, "safe to call
        # repeatedly" per the ABC.
        await self._call(lambda: self._api.land(), timeout_s, detail, "land")
        self._airborne = False

    async def move(self, direction: Direction, distance_cm: int,
                   timeout_s: float = 15.0) -> None:
        detail = f"move({getattr(direction, 'name', direction)!s}, "\
                 f"{distance_cm} cm)"
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        if not isinstance(direction, Direction):
            raise FlightError(
                f"{self.drone_id}: {detail} refused — direction must be a "
                f"finals.types.Direction, got {direction!r}")
        if not (isinstance(distance_cm, (int, float))
                and not isinstance(distance_cm, bool)
                and math.isfinite(distance_cm) and distance_cm > 0):
            raise FlightError(
                f"{self.drone_id}: {detail} refused — {distance_cm!r} cm is "
                f"not a physically executable distance (must be finite and "
                f"> 0; direction encodes the sign)")
        if not self._airborne:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — not flying — takeoff() "
                f"first (check the phase ordering)")
        self._check_alive_fresh(detail)
        # UNIT CONTRACT: distance_cm (pyhulax docs) — hula_connection.py:45's
        # move(FORWARD, 0.5) is the onsite "unit hop" gate; do not silently
        # rescale here. Value-parity of Direction is pinned by test_types.py.
        sdk_dir = _to_sdk_direction(direction)
        await self._call(lambda: self._api.move(sdk_dir, distance_cm),
                         timeout_s, detail, "move")

    async def rotate(self, angle_deg: float, timeout_s: float = 15.0) -> None:
        detail = f"rotate({angle_deg:g} deg)"
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        if not (isinstance(angle_deg, (int, float))
                and not isinstance(angle_deg, bool)
                and math.isfinite(angle_deg)):
            raise FlightError(
                f"{self.drone_id}: {detail} refused — {angle_deg!r} deg is "
                f"not a physically executable rotation (must be finite)")
        if not self._airborne:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — not flying — takeoff() "
                f"first (check the phase ordering)")
        self._check_alive_fresh(detail)
        # CONTRACT: +deg == CCW (pyhulax convention); pass-through. Sign is an
        # onsite-verify preflight gate, not silently flipped here.
        await self._call(lambda: self._api.rotate(angle_deg),
                         timeout_s, detail, "rotate")

    async def hover(self, duration_s: float) -> None:
        detail = f"hover({duration_s:g} s)"
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        if not (isinstance(duration_s, (int, float))
                and not isinstance(duration_s, bool)
                and math.isfinite(duration_s) and duration_s >= 0):
            raise FlightError(
                f"{self.drone_id}: {detail} refused — {duration_s!r} s is not "
                f"a physically executable hover duration")
        if not self._airborne:
            raise FlightError(
                f"{self.drone_id}: {detail} refused — not flying — takeoff() "
                f"first (check the phase ordering)")
        self._check_alive_fresh(detail)
        # hover() carries no timeout_s in the ABC; the blocking call lasts
        # ~duration_s — bound it at duration_s + margin so a wedged call still
        # trips the deadline (convention 2).
        timeout_s = duration_s + self._hover_margin_s
        await self._call(lambda: self._api.hover(duration_s),
                         timeout_s, detail, "hover")

    async def set_led(self, r: int, g: int, b: int) -> None:
        detail = f"set_led({r}, {g}, {b})"
        self._gate_connected(detail)
        self._gate_not_degraded(detail)
        # No timeout_s in the ABC; a fixed 5 s bound keeps the single send
        # inside convention 2.
        await self._call(lambda: self._api.set_led(r, g, b),
                         5.0, detail, "set_led")

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
                    f"telemetry arrived; nothing honest to report — "
                    f"{self._check_hint()}")
            return self._final_snapshot
        with self._state.lock:
            ts = self._state.ts
        if ts is None:
            raise FlightError(
                f"{self.drone_id}: telemetry() — no telemetry received yet — "
                f"{self._check_hint()}")
        return self._snapshot()

    def _snapshot(self) -> Telemetry:
        """Build Telemetry from _TelemetryState under the lock. pyhulax has no
        closed-loop horizontal position -> position_m=None, quality=NONE
        (honest; mission logic already works at NONE). altitude cm -> m; yaw
        normalized + pass-through (CCW+ contract == pyhulax convention)."""
        with self._state.lock:
            ts = self._state.ts
            battery_pct = self._state.battery_pct
            altitude_cm = self._state.altitude_cm
            yaw_deg = self._state.yaw_deg
            is_flying = self._state.is_flying
            dead = self._state.dead
        return Telemetry(
            ts=ts if ts is not None else time.monotonic(),
            battery_pct=battery_pct,
            altitude_m=(altitude_cm / 100.0 if altitude_cm is not None
                        else None),
            yaw_deg=(normalize_yaw_deg(yaw_deg) if yaw_deg is not None
                     else None),
            is_flying=is_flying,
            position_m=None,
            position_quality=PositionQuality.NONE,
            raw={"altitude_cm": altitude_cm, "yaw_deg_raw": yaw_deg,
                 "airborne_cmd": self._airborne, "poller_dead": dead},
        )

    async def emergency_land(self) -> None:
        """Best-effort safe-down; NEVER raises (the one sanctioned swallow site
        in the flight stack — adapter.py ABC). Bounded: if a prior timeout
        wedged the pool's single worker, this wait_for trips and the drone's
        own failsafe (enable_battery_failsafe, set ALWAYS at connect) is the
        backstop — logged, never raised."""
        self._airborne = False
        if self._api is None or self._pool is None:
            self._log("emergency_land: never connected — nothing to command")
            return
        try:
            loop = asyncio.get_running_loop()
            await asyncio.wait_for(
                loop.run_in_executor(self._pool, self._api.land), 5.0)
        except Exception:  # whitelisted site 3 — emergency_land must never raise
            self._log(f"emergency_land land() failed (continuing; relying on "
                      f"the onboard battery failsafe):\n"
                      f"{traceback.format_exc()}")
        with self._state.lock:
            dead = self._state.dead
        self._log(f"emergency_land finished (airborne_cmd={self._airborne}, "
                  f"poller_dead={dead!r})")


# ============================================================
# FakeDroneAPI — pyhulax-surface test double (no pyhulax needed)
# ============================================================
class _FakeOrientation:
    def __init__(self, yaw_deg: float):
        self.yaw = float(yaw_deg)
        self.pitch = 0.0
        self.roll = 0.0


class _FakeDroneState:
    def __init__(self, is_flying: bool):
        self.is_flying = bool(is_flying)


class FakeDroneAPI:
    """Mirrors the pyhulax DroneAPI surface for unit tests WITHOUT pyhulax.
    Inject via PyhulaxAdapter(api=FakeDroneAPI(...)).

    Scriptable:
    - block_s={method: seconds}: a real time.sleep IN the executor thread, so
      the adapter's wait_for / FlightTimeout path is exercised for real.
    - fail_on={method: ExcClassOrInstance}: raises the name-matching fake SDK
      exception (CommandTimeout/CommandRejected/NotReady/LowBattery/
      DroneConnectionError/TelemetryUnavailable) so the mapper is tested by
      name. Applies to commands AND the telemetry getters (for the poller
      dead-flag path).
    - battery_pct/altitude_cm/yaw_deg/is_flying: what the getters report.
    Records command calls in .calls (telemetry getters are NOT recorded — they
    fire at 2 Hz and would swamp the log, mock parity)."""

    def __init__(self, *, battery_pct: float = 100.0, altitude_cm: float = 0.0,
                 yaw_deg: float = 0.0, is_flying: bool = False,
                 block_s: Optional[dict] = None,
                 fail_on: Optional[dict] = None,
                 video_stream=None):
        self.calls = []
        self._battery_pct = battery_pct
        self._altitude_cm = altitude_cm
        self._yaw_deg = yaw_deg
        self._is_flying = is_flying
        self._block_s = dict(block_s or {})
        self._fail_on = dict(fail_on or {})
        self._video_stream = video_stream
        self.battery_failsafe_enabled = False

    def _raise(self, name: str) -> None:
        exc = self._fail_on.get(name)
        if exc is not None:
            raise exc() if isinstance(exc, type) else exc

    def _do(self, name: str, **kw) -> None:
        self.calls.append((name, kw))
        block = self._block_s.get(name)
        if block:
            time.sleep(block)
        self._raise(name)

    # --- connection ---
    def connect(self, ip=None, timeout=5.0):
        self._do("connect", ip=ip)

    def robust_connect(self, ip=None, timeout=5.0, verbose=True):
        self._do("robust_connect", ip=ip)
        return True

    def disconnect(self):
        self._do("disconnect")

    def enable_battery_failsafe(self):
        self._do("enable_battery_failsafe")
        self.battery_failsafe_enabled = True

    # --- flight ---
    def takeoff(self, height_cm=100, **kw):
        self._do("takeoff", height_cm=height_cm)
        self._is_flying = True

    def land(self, **kw):
        self._do("land")
        self._is_flying = False

    def move(self, direction, distance_cm, **kw):
        self._do("move", direction=direction, distance_cm=distance_cm)

    def rotate(self, angle_degrees, **kw):
        self._do("rotate", angle_degrees=angle_degrees)

    def hover(self, duration_seconds, **kw):
        self._do("hover", duration_seconds=duration_seconds)

    def set_led(self, r, g=0, b=0, **kw):
        self._do("set_led", r=r, g=g, b=b)

    # --- telemetry getters (immediate; not recorded) ---
    def get_battery(self):
        self._raise("get_battery")
        return self._battery_pct

    def get_altitude(self):
        self._raise("get_altitude")
        return self._altitude_cm

    def get_orientation(self):
        self._raise("get_orientation")
        return _FakeOrientation(self._yaw_deg)

    def get_state(self):
        self._raise("get_state")
        return _FakeDroneState(self._is_flying)

    # --- video (shared with PyhulaxVideoSource, S9 Part 3) ---
    def create_video_stream(self):
        self._do("create_video_stream")
        return self._video_stream

    def set_video_stream(self, enabled):
        self._do("set_video_stream", enabled=enabled)
