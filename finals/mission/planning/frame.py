"""Coordinate-frame plumbing for Challenge-2A landing navigation (NAV-2).

PURE stdlib `math` — no SDK, no numpy (the conventions scan bans a top-level
numpy import in pure modules). One job: turn an externally-posted landing
coordinate into the SHARED ARENA NED FRAME the planner + the dead reckoner
already speak, and answer the advisory "is this point inside my sector" question.

FRAME CONVENTION (binding — the SAME frame as flight/dead_reckon.py):
- Arena axes are 2-D metric (north_m, east_m): +north, +east. There is ONE
  shared arena frame; its origin + orientation are C2's boot pose, because
  every drone boots aligned at C2 (dead_reckon.py: "north here is each drone's
  OWN zero-yaw boot heading … all drones were booted facing a common heading —
  an OPERATIONAL requirement, S10"). That common heading is `c2_heading_deg`.
- Headings are degrees, CCW-POSITIVE viewed from above, 0 = +north — the
  pyhulax yaw sign (dead_reckon.py: psi_NED = -yaw_deg; Rotate(+90) turns the
  nose 90 deg CCW, facing WEST). discord_to_ned reuses dead_reckon's EXACT
  body->NED rotation so a hand-check against that file holds:
      at heading theta, FORWARD d -> (dN, dE) = ( d*cos theta, -d*sin theta)
      at heading theta, RIGHT   d -> (dN, dE) = ( d*sin theta,  d*cos theta)

ASSUMPTION A8 (the Discord landing-coordinate format is an ONSITE UNKNOWN):
We do NOT know the exact wire format the C2 operator will paste from Discord
(could be lat/lon, a grid ref, a "go 3 m forward 2 m left" instruction, …).
So this module commits to ONE documented, VALIDATED input contract and fails
loud on anything else, rather than guessing:

    coord = (forward_m, right_m)

a metric offset in C2's LOCAL body frame — forward_m along C2's boot heading,
right_m to C2's right (a person standing at C2 facing the heading: "x metres
ahead, y metres to my right"; left = negative right). This is the body frame
dead_reckon integrates, so the math has a single source of truth. The onsite
adapter that parses the real Discord message is responsible for reducing it to
this (forward_m, right_m) contract; keeping the unknown at the EDGE (one small
parser) instead of smearing it through the planner is the point.

Session: S11 (NAV-2). Derives from: flight/dead_reckon.py body->NED yaw math
(psi_NED = -yaw_deg, CCW+); the (north_m, east_m) frame of
mission/planning/types.py.
"""
from __future__ import annotations

import math
from typing import Any, Tuple

from finals.errors import ConfigError

Point = Tuple[float, float]  # (north_m, east_m) — same as types.Point


def _finite_pair(raw: Any, where: str) -> Tuple[float, float]:
    """Validate a 2-tuple of finite numbers; ConfigError otherwise. Shared by
    the coord + origin guards so a malformed input dies with the loader's
    actionable contract, never a raw TypeError deep in the trig (bool is
    rejected explicitly — True/False are ints in Python and would silently
    read as 1.0/0.0 metres)."""
    if (not isinstance(raw, (list, tuple)) or len(raw) != 2
            or any(not isinstance(c, (int, float)) or isinstance(c, bool)
                   or not math.isfinite(c) for c in raw)):
        raise ConfigError(
            f"{where}: expected a pair of finite numbers, got {raw!r}")
    return (float(raw[0]), float(raw[1]))


# ============================================================
# ORGANIZER FRAME BINDING (NAV-FIX, lever L1) — docs/field_markers.md
# ============================================================
# The organizers publish the 5 static beacons in their OWN (x, y) metres, where
# x is the SHORT cage axis (~5.3 m) and y is the LONG cage axis (~11.3 m). We
# ADOPT that frame as canonical to kill remap bugs (the marker coords arrive in
# this frame for free) and bind it ONCE to our arena (north_m, east_m):
#
#     north <- y   (the LONG ~11.3 m axis)
#     east  <- x   (the SHORT ~5.3 m axis)
#
# so an organizer point (x, y) is arena point_m = (north=y, east=x) — and the
# inverse, arena (north, east) -> organizer (x=east, y=north). This is an AXIS
# RELABEL (a swap), NOT a rotation: it is its own inverse, so the round-trip is
# exact. ⚠️ CONFIRM ONSITE (open Q #1, field_markers.md): which cage corner is
# the organizer (0,0) and which way +x/+y point sets whether this binding needs
# an additional offset/flip; the relabel itself (long<->north, short<->east) is
# the stable part. The step0 contract test already pins this binding
# (test_step0_contracts: "north = ... y, east = ... x"; id 11 -> point_m
# [4.40, 1.35] == [y, x]).
def organizer_xy_to_ne(xy: Any) -> Point:
    """Map an organizer (x, y) coord (x=short axis, y=long axis; metres) to the
    arena (north_m, east_m): north <- y, east <- x (see the binding block
    above). Pure relabel; ConfigError on a malformed pair. CONFIRM the origin /
    axis sense onsite."""
    x, y = _finite_pair(xy, "organizer_xy_to_ne: xy (x_short_m, y_long_m)")
    return (y, x)


def ne_to_organizer_xy(point_m: Any) -> Point:
    """Inverse of organizer_xy_to_ne: arena (north_m, east_m) -> organizer
    (x, y) = (east, north). Its own inverse with organizer_xy_to_ne, so the
    round-trip is EXACT (a swapped-axis mutant breaks the round-trip test)."""
    north, east = _finite_pair(point_m, "ne_to_organizer_xy: point_m (north_m, east_m)")
    return (east, north)


def discord_to_ned(coord: Any,
                   c2_origin_m: Any,
                   c2_heading_deg: float) -> Point:
    """Convert a Discord-posted landing coordinate into the shared arena NED
    frame (north_m, east_m).

    INPUT CONTRACT (assumption A8 — see the module docstring):
      coord        = (forward_m, right_m): a metric offset in C2's LOCAL body
                     frame (forward_m along C2's boot heading, right_m to C2's
                     right; left = negative right).
      c2_origin_m  = (north_m, east_m): C2's launch point in the arena frame
                     (ArenaMap.c2_origin_m).
      c2_heading_deg = the compass heading (deg, CCW-positive, 0 = +north) the
                     swarm booted aligned to (ArenaMap.c2_heading_deg).

    Returns (north_m, east_m) in the shared arena frame. PURE — same math as
    dead_reckon._integrate_move, so a hand-check against that file holds:
    a coord (forward, right) rotated by heading theta gives
        dN = cos(theta)*forward + sin(theta)*right
        dE = -sin(theta)*forward + cos(theta)*right
    then translated by the origin. At heading 0 (facing +north) the local
    frame == the arena frame: (forward, right) -> (+forward north, +right
    east). At heading +90 (facing WEST, CCW) forward goes -east and right goes
    +north — the dead_reckon spot check.

    Raises ConfigError (loud, actionable) on a malformed coord / origin /
    heading — the format is an onsite unknown, so the assumed shape is a
    VALIDATED contract, never a silent best-effort parse.
    """
    fwd, right = _finite_pair(coord, "discord_to_ned: coord (forward_m, right_m)")
    origin_n, origin_e = _finite_pair(
        c2_origin_m, "discord_to_ned: c2_origin_m (north_m, east_m)")
    if (not isinstance(c2_heading_deg, (int, float))
            or isinstance(c2_heading_deg, bool)
            or not math.isfinite(c2_heading_deg)):
        raise ConfigError(
            f"discord_to_ned: c2_heading_deg must be a finite number "
            f"(deg, CCW+), got {c2_heading_deg!r}")
    theta = math.radians(float(c2_heading_deg))
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    # Body (forward, right) -> NED delta, identical to dead_reckon FORWARD+RIGHT.
    d_north = cos_t * fwd + sin_t * right
    d_east = -sin_t * fwd + cos_t * right
    return (origin_n + d_north, origin_e + d_east)


def bearing_from_c2_deg(point_m: Point, c2_origin_m: Point) -> float:
    """The arena-frame compass heading (deg, CCW-positive, 0 = +north,
    normalized to (-180, 180]) from C2 toward point_m. Used by the sector
    predicate; exposed because the planner's leg headings are in the same
    convention. atan2(-east, north) gives the CCW-positive angle off +north
    (a point due EAST is at heading -90, matching FORWARD at yaw 0 = +north and
    yaw +90 = WEST). A point exactly AT C2 has no defined bearing -> 0.0."""
    d_north = point_m[0] - c2_origin_m[0]
    d_east = point_m[1] - c2_origin_m[1]
    if d_north == 0.0 and d_east == 0.0:
        return 0.0
    return _wrap180(math.degrees(math.atan2(-d_east, d_north)))


def in_sector(point_m: Point,
              c2_origin_m: Point,
              sector_center_deg: float,
              sector_half_width_deg: float) -> bool:
    """ADVISORY-ONLY keep-in test: is an estimated NED point inside this
    drone's assigned wedge, measured from C2?

    The sector is a wedge of headings centred on `sector_center_deg` (deg,
    CCW-positive, 0 = +north — the SAME convention as everything else here)
    with half-angle `sector_half_width_deg`. A point is IN the sector iff the
    (shortest, wrapped) angular difference between its bearing-from-C2 and the
    centre is <= the half-width. CLOSED on the boundary (a point exactly on the
    wedge edge counts as inside — the advisory geofence is generous, never a
    hard cut).

    PURE + ADVISORY: this is a SOFT geofence input (a hint for which drone owns
    which slice of the arena), NEVER a hard control gate — same status as
    ArenaMap.lanes and dead_reckon's DEAD_RECKONING quality. A half-width >= 180
    means "the whole circle" and always returns True; a negative or non-finite
    half-width is a config bug and raises ConfigError (a silent always-False
    sector would strand a drone). The point exactly AT C2 has no bearing and is
    treated as IN every sector (it is the shared origin).
    """
    if (not isinstance(sector_half_width_deg, (int, float))
            or isinstance(sector_half_width_deg, bool)
            or not math.isfinite(sector_half_width_deg)
            or sector_half_width_deg < 0):
        raise ConfigError(
            f"in_sector: sector_half_width_deg must be a finite number >= 0 "
            f"(degrees, the wedge half-angle), got {sector_half_width_deg!r}")
    if (not isinstance(sector_center_deg, (int, float))
            or isinstance(sector_center_deg, bool)
            or not math.isfinite(sector_center_deg)):
        raise ConfigError(
            f"in_sector: sector_center_deg must be a finite number "
            f"(deg, CCW+), got {sector_center_deg!r}")
    if sector_half_width_deg >= 180.0:
        return True
    # The shared origin has no defined bearing; it belongs to every drone's
    # wedge (it is where they all boot). Short-circuit so an at-C2 estimate is
    # never spuriously flagged outside a narrow sector (bearing_from_c2_deg
    # returns 0.0 there, which a non-zero-centred wedge would reject).
    if point_m[0] == c2_origin_m[0] and point_m[1] == c2_origin_m[1]:
        return True
    bearing = bearing_from_c2_deg(point_m, c2_origin_m)
    delta = abs(_wrap180(bearing - float(sector_center_deg)))
    return delta <= sector_half_width_deg


def _wrap180(angle_deg: float) -> float:
    """Normalize a finite angle to (-180, 180] (deg). Same rule + edge case as
    dead_reckon.normalize_yaw_deg (-180 -> +180); duplicated as a tiny private
    helper to keep this module dependency-free of the flight package (a pure
    planning leaf must not import flight/)."""
    r = angle_deg % 360.0
    if r > 180.0:
        r -= 360.0
    return r
