"""Shared data shapes for the finals package. Pure stdlib — no SDK imports.

Derives from:
- pyhulax SDK docs (Direction values 0-5, cm units, blocking semantics):
  https://pyhulax.xenops.ae/reference/pyhulax/ + docs/finals/example_code/hula_connection.py
- qualifier_run.py MissionState / detection-context shapes (formalized, frozen)
- hula_connection.py lines 46-50 state-machine advice (the Action vocabulary is
  exactly what BOTH backends — pyhulax and MAVSDK-SITL — can honestly execute)

Design notes (binding):
- Direction mirrors pyhulax.core.Direction integer values WITHOUT importing
  pyhulax, so the SITL VM never needs the SDK. Parity is asserted against
  hardcoded ints in tests/test_types.py.
- There is deliberately NO "GotoPosition" action: pyhulax has no honest
  closed-loop position setpoint (its move_to() wraps relative moves with no
  feedback — verified research). Mission logic composes Move/Rotate.
- Telemetry.position_quality tags how much to trust position_m; mission logic
  MUST work at PositionQuality.NONE.

Session: S1 (implemented).
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import TYPE_CHECKING, Optional, Tuple, Union

if TYPE_CHECKING:  # numpy only needed for annotations; keeps this module stdlib-only
    import numpy as np


# ============================================================
# Enums
# ============================================================
class Direction(IntEnum):
    """Body-frame movement direction. Values mirror pyhulax.core.Direction."""

    FORWARD = 0
    BACK = 1
    LEFT = 2
    RIGHT = 3
    UP = 4
    DOWN = 5


class PositionQuality(IntEnum):
    """How much a position estimate may be trusted (ordered, comparable)."""

    NONE = 0            # no estimate at all — mission logic must still work
    DEAD_RECKONING = 1  # integrated from commanded moves; drifts, never trusted for control
    UNTRUSTED = 2       # backend-provided but uncharacterized (pyhulax get_position())
    MEASURED = 3        # externally measured (SITL telemetry; UWB if it ever materializes)


# ============================================================
# Sensor / telemetry snapshots
# ============================================================
@dataclass(frozen=True)
class Telemetry:
    """Latest-known vehicle state. Always check age_s() before acting on it."""

    ts: float                                            # time.monotonic() at capture
    battery_pct: Optional[float] = None
    altitude_m: Optional[float] = None                   # ToF (real, cm->m) / -down_m (SITL)
    yaw_deg: Optional[float] = None
    is_flying: Optional[bool] = None
    position_m: Optional[Tuple[float, float, float]] = None
    position_quality: PositionQuality = PositionQuality.NONE
    raw: dict = field(default_factory=dict)              # backend extras; mission logic must not require them

    def age_s(self, now: Optional[float] = None) -> float:
        return (time.monotonic() if now is None else now) - self.ts


@dataclass(frozen=True)
class FrameStamped:
    """One video frame, normalized by the VideoSource that produced it."""

    image: "np.ndarray"          # HxWx3 uint8, BGR ALWAYS (sources convert)
    ts: float                    # time.monotonic() at receipt
    frame_number: Optional[int]
    source_id: str               # drone id this frame came from


@dataclass(frozen=True)
class Sighting:
    """One detection event from one detector on one frame. Append-only record —
    the convoy MOVES, so there is no dedup (unlike the qualifier barrel_log)."""

    drone_id: str
    ts: float
    source: str                                          # "yolo" | "aruco" | "qr"
    class_name: str                                      # e.g. "robomaster", "aruco_17"
    marker_id: Optional[int]                             # ArUco only
    bbox_xyxy: Tuple[float, float, float, float]
    confidence: float                                    # ArUco => 1.0
    frame_shape: Tuple[int, int]                         # (h, w)
    frame_number: Optional[int] = None
    drone_yaw_deg: Optional[float] = None
    drone_alt_m: Optional[float] = None
    bearing_deg: Optional[float] = None                  # yaw - (cx - w/2)/w * HFOV (CCW+ yaw — see
                                                         # flight/dead_reckon.py + vision/perception.py
                                                         # bearing_from_bbox; sign fixed S7, test-pinned)
    pos_quality: PositionQuality = PositionQuality.NONE
    est_north_m: Optional[float] = None                  # filled only when pos_quality > NONE
    est_east_m: Optional[float] = None
    frame_path: Optional[str] = None                     # annotated JPEG on disk, if saved


# ============================================================
# Actions — the ONLY vocabulary mission logic speaks
# ============================================================
@dataclass(frozen=True)
class Takeoff:
    height_cm: int = 80          # pyhulax default takeoff height


@dataclass(frozen=True)
class Move:
    direction: Direction
    distance_cm: int


@dataclass(frozen=True)
class Rotate:
    angle_deg: float             # +ve = CCW (pyhulax convention)


@dataclass(frozen=True)
class Hover:
    duration_s: float


@dataclass(frozen=True)
class Land:
    pass


@dataclass(frozen=True)
class Wait:
    """No command sent; the phase just wants wall-clock time to pass."""

    duration_s: float


@dataclass(frozen=True)
class Done:
    """Phase finished successfully; agent advances to the next phase."""

    reason: str


@dataclass(frozen=True)
class Abort:
    """Phase demands this drone fail loudly; agent safes it down."""

    reason: str


Action = Union[Takeoff, Move, Rotate, Hover, Land, Wait, Done, Abort]
