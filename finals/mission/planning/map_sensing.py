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
from typing import Dict, Iterable, Optional, Sequence, Tuple

from finals.mission.planning.types import KeepOut, Marker, Point

Bounds = Tuple[float, float, float, float]  # (north_min, east_min, north_max, east_max)

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
    convention — SAME source of truth as flight/dead_reckon.py's FORWARD map:
    at heading theta, a FORWARD step of d advances (dN, dE) = (d*cos theta,
    -d*sin theta) (dead_reckon `_integrate_move`, psi_NED = -yaw_deg). A
    sighting at absolute compass bearing b is the drone "facing b and looking
    forward" at the marker, so the marker sits at that FORWARD offset FROM the
    drone:  M = drone + (r*cos b, -r*sin b).  Inverting:
        drone = M - (r*cos b, -r*sin b) = (M_n - r*cos b, M_e + r*sin b).
    SIGN CHECK (pinned by test_map_sensing 3-4-5 + due-north/east fixtures): a
    marker due NORTH (b=0) of a drone at origin -> M=(r,0) -> drone=(r-r, 0)=
    origin; a marker due EAST (b=-90, since dE=-r*sin b>0) -> drone=origin. A
    flipped sign on EITHER term moves the recovered pose to the wrong side and
    those fixtures go red (mutation kill-check (a)).

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


def bounds_from_markers_and_cage(
        markers: "Iterable",
        cage_bounds_m: Optional[Bounds] = None,
        *, margin_m: float = 0.0) -> Bounds:
    """Lever L2 — DERIVE arena bounds_m that PROVABLY enclose every marker (plus
    an optional surveyed cage rectangle), instead of a footlength guess.

    The 5 field beacons sit at SURVEYED interior coords (docs/field_markers.md
    L2), so the bounds MUST contain them — that is a ground-truth constraint, not
    an estimate. This returns the smallest axis-aligned rectangle (then grown by
    `margin_m`) that covers:
      * every marker point, AND
      * the cage rectangle, when a surveyed `cage_bounds_m`
        (north_min, east_min, north_max, east_max) is supplied.
    So bounds = union(marker extent, cage) +/- margin. With a cage given, the
    cage normally dominates (the markers are interior) and the result == the
    cage grown by margin; passing NO cage yields the tight marker hull (a useful
    lower bound before the cage tape is measured).

    `markers` is any iterable of Marker (use arena.markers directly) or of
    (north_m, east_m) pairs. At least one marker is required (an empty hull has
    no rectangle — refuse loudly rather than return a degenerate bound). margin_m
    must be finite >= 0. The result is ALWAYS a valid bounds_m (north_min <
    north_max, east_min < east_max) UNLESS every input collapses to a single
    point AND margin_m == 0 — that case raises (a zero-area arena), telling the
    operator to widen the cage or add a margin.

    PURE stdlib. The caller feeds the result into ArenaMap.from_dict, whose
    NAV-2 markers-within-bounds check then becomes a TAUTOLOGY by construction
    (the bound was built to contain them) — exactly the "bounds from the markers,
    not a guess" property. (The full cage rectangle still needs the tape; this
    pins scale/origin/containment, see field_markers.md L2.)
    """
    if (not isinstance(margin_m, (int, float)) or isinstance(margin_m, bool)
            or not math.isfinite(margin_m) or margin_m < 0):
        raise ValueError(
            f"map_sensing.bounds_from_markers_and_cage: margin_m must be a "
            f"finite number >= 0 (m), got {margin_m!r}")
    pts = [_marker_point(m) for m in markers]
    if not pts:
        raise ValueError(
            "map_sensing.bounds_from_markers_and_cage: need at least one marker "
            "to derive bounds (an empty marker set has no extent) — pass the "
            "field beacons (arena.markers) or a list of [north_m, east_m] pairs")
    norths = [p[0] for p in pts]
    easts = [p[1] for p in pts]
    n_min, n_max = min(norths), max(norths)
    e_min, e_max = min(easts), max(easts)
    if cage_bounds_m is not None:
        c_nmin, c_emin, c_nmax, c_emax = _require_bounds(
            cage_bounds_m, "cage_bounds_m")
        n_min, e_min = min(n_min, c_nmin), min(e_min, c_emin)
        n_max, e_max = max(n_max, c_nmax), max(e_max, c_emax)
    n_min, e_min = n_min - margin_m, e_min - margin_m
    n_max, e_max = n_max + margin_m, e_max + margin_m
    if not (n_min < n_max and e_min < e_max):
        # All inputs collapsed to a single point and no margin grew it — a
        # zero-area arena is not flyable. Fail loud (the NAV-2 from_dict would
        # reject this bound anyway; catch it HERE with an actionable message).
        raise ValueError(
            f"map_sensing.bounds_from_markers_and_cage: derived a degenerate "
            f"(zero-area) bound {(n_min, e_min, n_max, e_max)} — the markers "
            f"(and cage, if any) are collinear/coincident on an axis and "
            f"margin_m={margin_m} did not grow it. Supply the cage rectangle or "
            f"a margin_m > 0.")
    return (n_min, e_min, n_max, e_max)


def _marker_point(m) -> Point:
    """Accept either a Marker (use its point_m) or a raw (north_m, east_m) pair.
    Always re-validates finiteness via _require_point: a Marker is normally built
    by the validated from_dict, but a directly-constructed Marker(id, (nan, 0))
    would otherwise let a NaN slip into min/max and silently poison the derived
    bound (NaN < x is always False)."""
    raw = m.point_m if isinstance(m, Marker) else m
    return _require_point(raw, "marker point")


def _require_bounds(raw, where: str) -> Bounds:
    if (not isinstance(raw, (list, tuple)) or len(raw) != 4
            or any(not isinstance(c, (int, float)) or isinstance(c, bool)
                   or not math.isfinite(c) for c in raw)):
        raise ValueError(
            f"map_sensing: {where} must be a finite [north_min, east_min, "
            f"north_max, east_max] 4-tuple, got {raw!r}")
    n_min, e_min, n_max, e_max = (float(raw[0]), float(raw[1]),
                                  float(raw[2]), float(raw[3]))
    if not (n_min <= n_max and e_min <= e_max):
        raise ValueError(
            f"map_sensing: {where} is inverted ({raw!r}) — expected "
            f"north_min <= north_max and east_min <= east_max")
    return (n_min, e_min, n_max, e_max)


def _require_point(raw, where: str) -> Point:
    if (not isinstance(raw, (list, tuple)) or len(raw) != 2
            or any(not isinstance(c, (int, float)) or isinstance(c, bool)
                   or not math.isfinite(c) for c in raw)):
        raise ValueError(
            f"map_sensing: {where} must be a finite [north_m, east_m] pair, got "
            f"{raw!r}")
    return (float(raw[0]), float(raw[1]))
