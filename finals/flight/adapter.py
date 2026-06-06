"""FlightAdapter — THE critical seam of the finals package.

The contract is the intersection of what BOTH backends can honestly do:
- pyhulax (real HULA drones over Wi-Fi): blocking relative moves —
  takeoff(height_cm) / land() / move(Direction, distance_cm) / rotate(deg) /
  hover(s); immediate telemetry getters. Docs: https://pyhulax.xenops.ae
- MAVSDK SITL (qualifier PX4 VM): implements the SAME relative-move semantics
  on top of NED position setpoints (root drone_control.py + the proven
  _go_to_waypoint loop from qualifier_run.py:268-331).

Contract notes (binding):
- Every method either COMPLETES the physical action or RAISES
  (finals.errors.FlightError / FlightTimeout). No silent failure, no partial
  success without an exception.
- Methods are NOT re-entrant per drone: DroneAgent guarantees at most one
  in-flight command at a time.
- There is deliberately NO goto(x, y, z): pyhulax has no honest closed-loop
  position setpoint (its move_to() wraps relative moves with no feedback —
  verified against the SDK docs). Mission logic composes Move/Rotate; position
  feedback, if any, arrives via Telemetry.position_m + position_quality.
- distance_cm / height_cm units follow the pyhulax docs. The hula_connection.py
  example shows `move(FORWARD, 0.5)` which contradicts cm units — UNIT
  VERIFICATION IS AN ONSITE PREFLIGHT GATE ("unit hop"); a unit fix touches
  only the adapter boundary, never mission logic.
- Blocking SDK calls run in a per-drone single-thread executor under
  asyncio.wait_for so a hung drone cannot stall the orchestrator or the
  other drones (implemented per-backend, S6/S9).

Implementations: MockAdapter (S3, finals/flight/mock_adapter.py),
MavsdkSitlAdapter (S6, sitl_adapter.py), PyhulaxAdapter (S9,
pyhulax_adapter.py), BenchAdapter (S3, below).

Session: S1 (ABC); S3 (BenchAdapter implemented).
"""
from __future__ import annotations

import sys
from abc import ABC, abstractmethod

from finals.errors import FlightError
from finals.types import Direction, Telemetry


class FlightAdapter(ABC):
    """One instance per drone. Async outside, hard deadline on every command."""

    def __init__(self, drone_id: str):
        self.drone_id = drone_id

    @abstractmethod
    async def connect(self, timeout_s: float = 10.0) -> None:
        """Establish the link (and start the telemetry poller). Raises
        FlightError/FlightTimeout with drone_id + what to check."""

    @abstractmethod
    async def disconnect(self) -> None:
        """Release the link. Never raises — failures are logged."""

    @abstractmethod
    async def takeoff(self, height_cm: int = 80, timeout_s: float = 30.0) -> None:
        """Arm + climb to height_cm; returns when airborne at altitude."""

    @abstractmethod
    async def land(self, timeout_s: float = 30.0) -> None:
        """Descend + disarm; returns when on the ground. Safe to call repeatedly."""

    @abstractmethod
    async def move(self, direction: Direction, distance_cm: int,
                   timeout_s: float = 15.0) -> None:
        """Body-frame relative move; returns when the move completes."""

    @abstractmethod
    async def rotate(self, angle_deg: float, timeout_s: float = 15.0) -> None:
        """Relative yaw rotation, +ve = CCW (pyhulax convention)."""

    @abstractmethod
    async def hover(self, duration_s: float) -> None:
        """Hold position for duration_s."""

    @abstractmethod
    def telemetry(self) -> Telemetry:
        """Latest-known state. Non-blocking, may be stale — callers must check
        Telemetry.age_s() before trusting it."""

    @abstractmethod
    async def emergency_land(self) -> None:
        """Best-effort safe-down. The ONE place in the flight stack allowed to
        swallow exceptions — and every swallow is logged with traceback."""

    async def set_led(self, r: int, g: int, b: int) -> None:
        """Identity colour (pyhulax set_led). Default no-op for backends
        without LEDs (SITL/mock)."""
        return None


class BenchAdapter(FlightAdapter):
    """Bench profile: real drones on the ground, props-off validation only.

    Wraps the INNER adapter that actually talks to the drone (PyhulaxAdapter
    at S9; any FlightAdapter in tests — the seam is generic and is exercised
    over MockAdapter from S3). connect/disconnect/telemetry/set_led delegate
    to the inner adapter's non-flight surface; takeoff/land/move/rotate/hover
    are REFUSED with a FlightError naming the drone, the command, and the
    bench profile. emergency_land is a logged no-op — nothing is airborne,
    and it deliberately does NOT delegate (a bench session must never send
    any flight command, not even a safe-down, to a props-off airframe).

    Never-raise paths (disconnect/emergency_land) are kept safe by doing
    nothing risky — NOT by catching: this file is outside the
    blanket-exception-catching whitelist (tests/test_conventions.py).

    S4 wiring note: the generic `flight_cls(drone_id)` construction does not
    fit this class — bench needs a special case that builds the inner
    backend first and wraps it (BenchAdapter(inner)).

    Session: S3 (implemented; inner=PyhulaxAdapter arrives S9).
    """

    def __init__(self, inner: FlightAdapter):
        if not isinstance(inner, FlightAdapter):
            # The S4 generic flight_cls(drone_id) wiring WILL hit this if the
            # bench special case is forgotten — make the failure self-explain
            # instead of dying on inner.drone_id with a bare AttributeError.
            raise TypeError(
                f"BenchAdapter wraps an INNER FlightAdapter instance, got "
                f"{type(inner).__name__!r} ({inner!r}) — build the real "
                f"backend first, then BenchAdapter(inner); the generic "
                f"flight_cls(drone_id) wiring needs a bench special case "
                f"(see the class docstring)")
        super().__init__(inner.drone_id)
        self.inner = inner

    def _refuse(self, command: str) -> FlightError:
        return FlightError(
            f"{self.drone_id}: {command} refused — bench profile: flight "
            f"commands disabled (props-off validation only) — use "
            f"--profile sitl or --profile real to fly")

    async def connect(self, timeout_s: float = 10.0) -> None:
        await self.inner.connect(timeout_s=timeout_s)

    async def disconnect(self) -> None:
        await self.inner.disconnect()

    async def takeoff(self, height_cm: int = 80, timeout_s: float = 30.0) -> None:
        raise self._refuse(f"takeoff({height_cm} cm)")

    async def land(self, timeout_s: float = 30.0) -> None:
        raise self._refuse("land()")

    async def move(self, direction: Direction, distance_cm: int,
                   timeout_s: float = 15.0) -> None:
        raise self._refuse(f"move({direction.name}, {distance_cm} cm)")

    async def rotate(self, angle_deg: float, timeout_s: float = 15.0) -> None:
        raise self._refuse(f"rotate({angle_deg:g} deg)")

    async def hover(self, duration_s: float) -> None:
        raise self._refuse(f"hover({duration_s:g} s)")

    def telemetry(self) -> Telemetry:
        return self.inner.telemetry()

    async def set_led(self, r: int, g: int, b: int) -> None:
        await self.inner.set_led(r, g, b)

    async def emergency_land(self) -> None:
        try:
            print(
                f"[BenchAdapter] {self.drone_id}: emergency_land is a no-op "
                f"on the bench (props off, nothing airborne) — NOT delegated",
                file=sys.stderr, flush=True,
            )
        except OSError:
            # A closed/broken stderr must not turn the one must-never-raise
            # path into a raise; there is nowhere left to log to.
            pass
