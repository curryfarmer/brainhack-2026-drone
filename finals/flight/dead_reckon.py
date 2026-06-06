"""DeadReckoner — pure 2D pose + yaw integration of COMPLETED actions.

FRAME CONVENTION (binding — pinned here FIRST, enforced by
tests/test_dead_reckon.py before any consumer existed):

- World axes: north_m / east_m horizontal, alt_m UP-POSITIVE (a Telemetry
  position_m tuple built from a DRPose is (north_m, east_m, alt_m) — the
  third element is altitude, NOT NED down. types.py's altitude_m comment
  ("ToF (real, cm->m) / -down_m (SITL)") already commits to up-positive;
  backends converting from NED must negate down before populating either.)
- yaw_deg is CCW-POSITIVE viewed from above (the pyhulax Rotate convention:
  rotate(+90) turns the nose 90° counter-clockwise seen from above), with
  yaw 0 = the zero-yaw heading, defined as +north. Normalized to (-180, 180].
  The SAME duty as altitude applies to yaw: a backend whose source frame is
  NED (S6 SITL — detection_to_world.py's Pose.yaw_rad is CW-positive) must
  populate Telemetry.yaw_deg = -psi_NED, normalized, before publishing it.
- "north" here is each drone's OWN zero-yaw boot heading, so DRPose (and the
  est_north/east it stamps on Sightings) is a PER-DRONE frame. Cross-drone
  fusion of those estimates assumes all drones were booted facing a common
  heading — that alignment is an OPERATIONAL requirement (preflight duty,
  S10), not something this math can enforce.
- This is the NEGATION of detection_to_world.py's NED yaw_rad (0 = north,
  +CW from above): psi_NED = -yaw_deg. Substituting psi = -theta into
  detection_to_world.py:109-111 (north += cos(psi)*Xb - sin(psi)*Yb;
  east += sin(psi)*Xb + cos(psi)*Yb; Xb = body-forward, Yb = body-right):
      FORWARD d:  (dN, dE) = ( d*cos(theta), -d*sin(theta))
      RIGHT   d:  (dN, dE) = ( d*sin(theta),  d*cos(theta))
      BACK / LEFT = the negations.
  Spot check: at yaw +90 (rotated 90° CCW from north) the drone faces WEST;
  FORWARD moves it (0, -d) and its body-right points north (+d, 0).
- KNOWN UPSTREAM CONFLICT (do not copy blindly in S7): the bearing comment
  in types.py:102 reads "yaw + (cx - w/2)/w * HFOV". Under CCW-positive yaw
  a target right of frame-centre (cx > w/2) lies at DECREASING yaw, so the
  sign must be "yaw - offset" (or the yaw fed in must be compass/CW).
  Resolve when implementing vision/perception.py.

Semantics (binding):
- note_action_complete() is called ONLY for actions that actually COMPLETED
  (the FlightAdapter contract is complete-or-raise). A timed-out command is
  never noted — the airframe may still have moved, which is exactly why the
  quality is permanently PositionQuality.DEAD_RECKONING: drift compounds per
  move; the estimate annotates Sightings and feeds advisory geofencing but
  is NEVER used for closed-loop control decisions.
- Land sets alt_m = 0 and KEEPS north/east/yaw. Drones don't teleport on
  touchdown: a re-takeoff continues the same ground track, and wiping the
  horizontal estimate would silently re-zero every later Sighting's
  est_north/east at the landing spot. (Supersedes the S1 stub note "resets
  on Land" — decision recorded here, S3.)
- No clamping anywhere (e.g. a DOWN move integrating below alt 0 is recorded
  verbatim): this class records what the adapter REPORTED COMPLETE; refusing
  physically impossible sequences is the adapter's job, and clamping here
  would silently hide such adapter bugs.
- cm -> m conversion happens at THIS boundary (the FlightAdapter contract is
  cm, the world estimate is m — units live in the names on both sides).

Derives from: the body->NED yaw-rotation math of detection_to_world.py
(project_pixel_to_world), reduced to relative-move integration with the yaw
sign flipped to the pyhulax CCW convention (documented above). Shared with
MockAdapter's simulated pose so the math has a single source of truth.

Pure stdlib math, zero I/O, no asyncio, no module-level mutable state.

Session: S3 (implemented).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from finals.types import (Abort, Action, Direction, Done, Hover, Land, Move,
                          PositionQuality, Rotate, Takeoff, Wait)


def _require_finite(value: float, what: str) -> float:
    """NaN/Inf at this boundary would poison north/east on the next move
    with NO exception anywhere (nan % 360 == nan; cos(nan) == nan) — exactly
    the silent-Sighting-corruption class this module exists to prevent."""
    if not math.isfinite(value):
        raise ValueError(
            f"DeadReckoner: {what} must be finite, got {value!r} — check the "
            f"upstream computation that produced it (config divide-by-zero? "
            f"uninitialized field?)")
    return value


def normalize_yaw_deg(yaw_deg: float) -> float:
    """Normalize any FINITE angle to (-180, 180] (degrees, CCW-positive);
    NaN/Inf raise ValueError (they would propagate silently otherwise).

    -180 maps to +180 so every heading has exactly ONE representation.
    Python float % keeps tiny negative inputs non-negative (e.g.
    -1e-15 % 360 == 360.0 by rounding) — the > 180 branch folds that
    back to 0.0, so the range really is (-180, 180].
    """
    _require_finite(yaw_deg, "angle")
    r = yaw_deg % 360.0
    if r > 180.0:
        r -= 360.0
    return r


@dataclass(frozen=True)
class DRPose:
    """Dead-reckoned pose snapshot. Axes/conventions in the module docstring:
    north_m/east_m world-horizontal, alt_m up-positive, yaw_deg CCW-positive
    in (-180, 180] with 0 = +north."""

    north_m: float
    east_m: float
    alt_m: float
    yaw_deg: float


class DeadReckoner:
    """Integrates COMPLETED actions into a DRPose. One instance per drone
    (no locking: the FlightAdapter contract is non-reentrant per drone, so
    notes arrive strictly sequentially)."""

    #: Dead reckoning drifts per move and is NEVER closed-loop trustworthy.
    QUALITY = PositionQuality.DEAD_RECKONING

    def __init__(self, initial: Optional[DRPose] = None):
        p = initial if initial is not None else DRPose(0.0, 0.0, 0.0, 0.0)
        self._north_m = _require_finite(p.north_m, "initial north_m")
        self._east_m = _require_finite(p.east_m, "initial east_m")
        self._alt_m = _require_finite(p.alt_m, "initial alt_m")
        self._yaw_deg = normalize_yaw_deg(p.yaw_deg)   # finite-checked there

    @property
    def pose(self) -> DRPose:
        return DRPose(self._north_m, self._east_m, self._alt_m, self._yaw_deg)

    def note_action_complete(self, action: Action) -> None:
        """Integrate one COMPLETED action (see module docstring semantics).

        Takeoff sets alt; Move integrates through the current yaw (UP/DOWN
        alter alt only); Rotate updates yaw; Land zeroes alt keeping
        north/east/yaw; Hover/Wait/Done/Abort are explicit no-ops (the
        airframe does not translate). Anything else is a programming error
        and raises TypeError naming the offending type.
        """
        if isinstance(action, Takeoff):
            self._alt_m = _require_finite(
                action.height_cm, "Takeoff.height_cm") / 100.0  # cm -> m boundary
        elif isinstance(action, Move):
            self._integrate_move(
                action.direction,
                _require_finite(action.distance_cm, "Move.distance_cm") / 100.0)
        elif isinstance(action, Rotate):
            self._yaw_deg = normalize_yaw_deg(self._yaw_deg + action.angle_deg)
        elif isinstance(action, Land):
            self._alt_m = 0.0                            # keep north/east/yaw
        elif isinstance(action, (Hover, Wait, Done, Abort)):
            pass                                         # no translation
        else:
            raise TypeError(
                f"DeadReckoner.note_action_complete: unsupported action type "
                f"{type(action).__name__!r} ({action!r}) — expected one of "
                f"the finals.types Action vocabulary"
            )

    def _integrate_move(self, direction: Direction, distance_m: float) -> None:
        """Body-frame move -> world delta through the current yaw.
        See the module docstring for the derivation against
        detection_to_world.py (psi_NED = -yaw_deg)."""
        if direction is Direction.UP:
            self._alt_m += distance_m
            return
        if direction is Direction.DOWN:
            self._alt_m -= distance_m
            return
        theta = math.radians(self._yaw_deg)
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        if direction is Direction.FORWARD:
            dn, de = cos_t * distance_m, -sin_t * distance_m
        elif direction is Direction.BACK:
            dn, de = -cos_t * distance_m, sin_t * distance_m
        elif direction is Direction.RIGHT:
            dn, de = sin_t * distance_m, cos_t * distance_m
        elif direction is Direction.LEFT:
            dn, de = -sin_t * distance_m, -cos_t * distance_m
        else:  # IntEnum is closed today; fail loud if it ever grows a value
            raise TypeError(
                f"DeadReckoner: unsupported Direction {direction!r} — "
                f"FORWARD/BACK/LEFT/RIGHT/UP/DOWN are the known values"
            )
        self._north_m += dn
        self._east_m += de
