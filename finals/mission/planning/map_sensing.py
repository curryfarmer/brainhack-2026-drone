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

from typing import Dict, Sequence, Tuple

from finals.mission.planning.types import KeepOut, Point

# Each call lists the function's onsite gate / data source in its docstring so a
# future session does not have to re-derive the interface.


def position_fix_from_marker(marker_world_m: Point, bearing_deg: float,
                             ground_range_m: float, drone_yaw_deg: float) -> Point:
    """STUB (C). Solve the drone's absolute (north_m, east_m) from a sighting of
    a marker at a KNOWN world coord.

    Inputs (all from the down-cam ArUco decode + telemetry): marker_world_m = the
    marker's known (north_m, east_m); bearing_deg = measured bearing to it
    (deg, CCW+, the perception convention); ground_range_m = horizontal distance
    (from pixel offset + altitude, similar triangles); drone_yaw_deg = compass
    heading. Returns the drone's (north_m, east_m) so the caller can RESET the
    dead-reckon XY (the position fix that defeats open-loop drift).

    Not implemented: kept a stub by design; the hand map + DeadReckoner are the
    active path. See finals/docs/module_map.md.
    """
    raise NotImplementedError(
        "finals.mission.planning.map_sensing.position_fix_from_marker: session "
        "S11 (NAV map-sensing, lever C) — kept a stub by design. See "
        "finals/docs/module_map.md")


def keep_outs_from_overhead_corners(
        corners_by_id: Dict[str, Sequence[Point]]) -> Tuple[KeepOut, ...]:
    """STUB (A). Assemble operator-tapped crate corners into validated KeepOut
    polygons for the arena.

    corners_by_id maps a crate id -> its ring of (north_m, east_m) corners,
    already rectified from one overhead image (the operator taps each crate's
    footprint on a tablet; the pixel->world rectification is the caller's job).
    Returns KeepOuts ready to merge into ArenaMap.keep_out — the "can't walk and
    measure" map source.

    Not implemented: kept a stub by design; hand-traced keep_out (or omit it for
    straight-line transit) is the active path. See finals/docs/module_map.md.
    """
    raise NotImplementedError(
        "finals.mission.planning.map_sensing.keep_outs_from_overhead_corners: "
        "session S11 (NAV map-sensing, lever A) — kept a stub by design. See "
        "finals/docs/module_map.md")
