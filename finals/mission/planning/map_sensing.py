"""map_sensing.py — sense/assist the obstacle map instead of hand-measuring it.

STUB (session S11, NAV map-sensing). Contract + seam only; the ACTIVE map source
is the hand-authored arena.json (keep_out polygons), with the already-working
"omit keep_out -> straight-line transit" fallback. Kept a stub by explicit
choice ("stub first").

WHY there is no full SLAM here (the binding hardware verdict, researched
2026-06-09): the HULA drones are POSITION-BLIND (reliable altitude + heading +
battery; NO horizontal XY, no velocity), the RealSense is DOWN-LOOKING (sees the
floor for landing, not crates ahead), it has NO IMU, and its depth range is only
~1.5-3 m. Mapping needs to localize the sensor to place observations; with no XY
every observation lands in the DRIFTING dead-reckon frame (1-5%/leg) and the map
smears. So real-time autonomous SLAM is infeasible on this airframe.

The two levers that ARE real:
  - the down-cam photographs crate FOOTPRINTS from above (crates read as
    rectangles looking down);
  - ArUco markers sit at KNOWN world coords (Discord pad coords + the markers
    beside pads), so decoding one gives range+bearing to a known point = an
    ABSOLUTE position fix that RESETS drift.

Three fill paths (ranked by ROI), each a function below:
  A. keep_outs_from_overhead_corners — operator taps crate corners on ONE
     overhead image (drone hover-high once, or a phone over the cage), already
     rectified pixel->world; assemble validated KeepOut polygons. Highest ROI:
     "can't walk and measure" solved without pretending to do SLAM.
  B. (recon mosaic) — a low pre-flight lawnmower scan, detect footprints from
     above, georeference by dead-reckon ANCHORED with C's marker fixes. Not even
     a signature yet: needs the recon phase + cv2 footprint detection (vision/),
     so it is described here but lives across modules when built.
  C. position_fix_from_marker — a known-coord marker decode -> absolute XY,
     correcting dead-reckon. Foundational (also sharpens transit + landing) and
     the enabler for B.

Filling a stub (a future session): implement the pure geometry here; put any
cv2/image work in vision/ (this module stays PURE so the bare-venv suite keeps
passing); wire C into the DeadReckoner as a correction input and A into a
config/arena authoring step; test against hand-computed fixtures. See
finals/docs/module_map.md (S11 NAV map-sensing row). PURE: stdlib only.
"""
from __future__ import annotations

import math
from typing import Dict, Sequence, Tuple

from finals.mission.planning.types import KeepOut, Point

# Each call lists the function's onsite gate / data source in its docstring so a
# future session does not have to re-derive the interface.


def position_fix_from_marker(marker_world_m: Point, bearing_deg: float,
                             ground_range_m: float) -> Point:
    """Lever C — solve the drone's absolute (north_m, east_m) from a sighting of
    a marker at a KNOWN world coord. The position fix that RESETS dead-reckon
    drift on POSITION-BLIND HULA (field_markers: 5 static markers at fixed (x,y)).

    Inputs (all from the down-cam ArUco decode + telemetry):
      marker_world_m  : the marker's KNOWN (north_m, east_m).
      bearing_deg     : the ABSOLUTE compass bearing from the drone TO the marker
                        (deg, CCW+). This is exactly Sighting.bearing_deg, which
                        finals.vision.perception.bearing_from_bbox already builds
                        as `yaw_deg - pixel_offset_frac*hfov` — i.e. the drone's
                        heading is ALREADY folded in, so no separate yaw arg.
      ground_range_m  : HORIZONTAL distance drone->marker (from the pixel offset
                        + altitude by similar triangles), metres.

    Returns the drone's (north_m, east_m). Geometry uses the project heading
    convention (visibility_graph): a step of range r at compass bearing b
    advances (dN, dE) = (r*cos b, -r*sin b). The marker sits at that offset FROM
    the drone, so the drone is the marker MINUS it:
        drone = (M_n - r*cos b, M_e + r*sin b).

    Fail loud on non-finite / negative range (a degenerate fix would silently
    teleport the dead-reckoner). PURE — the cv2 decode + range estimate are the
    caller's (vision/) job; this is the closed-form solve only.
    """
    mn, me = _require_point(marker_world_m, "marker_world_m")
    if not isinstance(bearing_deg, (int, float)) or isinstance(bearing_deg, bool) \
            or not math.isfinite(bearing_deg):
        raise ValueError(
            f"map_sensing.position_fix_from_marker: bearing_deg must be a finite "
            f"number (deg, CCW+, absolute), got {bearing_deg!r}")
    if not isinstance(ground_range_m, (int, float)) \
            or isinstance(ground_range_m, bool) \
            or not math.isfinite(ground_range_m) or ground_range_m < 0:
        raise ValueError(
            f"map_sensing.position_fix_from_marker: ground_range_m must be a "
            f"finite distance >= 0 (m), got {ground_range_m!r} — a negative/NaN "
            f"range would teleport the dead-reckoner")
    b = math.radians(bearing_deg)
    return (mn - ground_range_m * math.cos(b),
            me + ground_range_m * math.sin(b))


def keep_outs_from_overhead_corners(
        corners_by_id: Dict[str, Sequence[Point]]) -> Tuple[KeepOut, ...]:
    """Lever A — assemble operator-tapped crate corners into validated KeepOut
    polygons. The "can't walk and measure" map source: the operator taps each
    crate's footprint on ONE rectified overhead image (drone hover-high once, or
    a phone over the cage); the pixel->world rectification is the caller's job,
    so this takes already-(north_m, east_m) corner rings.

    corners_by_id maps crate id -> its ring of (north_m, east_m) corners. Returns
    KeepOuts (sorted by id, deterministic) ready to feed ObstacleMap.add_keep_out
    or to merge into ArenaMap.keep_out.

    Validation REUSES KeepOut.from_dict (the arena loader's rule: >= 3 distinct
    finite vertices), so a mistapped point/edge fails LOUD with the SAME message
    an operator already knows from the arena JSON — never a silently-dropped
    obstacle. PURE: stdlib only.
    """
    if not isinstance(corners_by_id, dict):
        raise ValueError(
            f"map_sensing.keep_outs_from_overhead_corners: corners_by_id must be "
            f"a dict of id -> corner ring, got {type(corners_by_id).__name__}")
    out = []
    for cid in sorted(corners_by_id):
        ring = corners_by_id[cid]
        # Reuse the arena keep-out validator (>= 3 distinct finite vertices) so
        # the contract + error text match the hand-authored arena exactly.
        ko = KeepOut.from_dict({"id": cid, "polygon_m": list(ring)}, index=cid)
        out.append(ko)
    return tuple(out)


def _require_point(raw, where: str) -> Point:
    if (not isinstance(raw, (list, tuple)) or len(raw) != 2
            or any(not isinstance(c, (int, float)) or isinstance(c, bool)
                   or not math.isfinite(c) for c in raw)):
        raise ValueError(
            f"map_sensing: {where} must be a finite [north_m, east_m] pair, got "
            f"{raw!r}")
    return (float(raw[0]), float(raw[1]))
