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

Session: S1 (ABC implemented; BenchAdapter stub — session S3).
"""
from __future__ import annotations

from abc import ABC, abstractmethod

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

    connect()/telemetry()/set_led() delegate to a wrapped PyhulaxAdapter's
    non-flight surface; takeoff/move/rotate/land/hover raise FlightError
    ("bench profile: flight commands disabled"). emergency_land is a no-op
    (nothing is airborne).

    STUB — session S3 (after PyhulaxAdapter's telemetry poller shape exists in
    mock form). Derives from: finals/flight/pyhulax_adapter.py non-flight surface.
    """

    _STUB = "finals.flight.adapter.BenchAdapter: session S3 — see finals/docs/module_map.md"

    def __init__(self, drone_id: str, *args, **kwargs):
        raise NotImplementedError(self._STUB)

    async def connect(self, timeout_s: float = 10.0) -> None:
        raise NotImplementedError(self._STUB)

    async def disconnect(self) -> None:
        raise NotImplementedError(self._STUB)

    async def takeoff(self, height_cm: int = 80, timeout_s: float = 30.0) -> None:
        raise NotImplementedError(self._STUB)

    async def land(self, timeout_s: float = 30.0) -> None:
        raise NotImplementedError(self._STUB)

    async def move(self, direction: Direction, distance_cm: int,
                   timeout_s: float = 15.0) -> None:
        raise NotImplementedError(self._STUB)

    async def rotate(self, angle_deg: float, timeout_s: float = 15.0) -> None:
        raise NotImplementedError(self._STUB)

    async def hover(self, duration_s: float) -> None:
        raise NotImplementedError(self._STUB)

    def telemetry(self) -> Telemetry:
        raise NotImplementedError(self._STUB)

    async def emergency_land(self) -> None:
        raise NotImplementedError(self._STUB)
