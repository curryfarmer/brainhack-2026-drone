"""Frozen geometry + map contracts the S11 navigation sessions code against.

These are the SHAPES, not the behavior:
- NAV-1 (visibility_graph) consumes an ArenaMap and emits a list[Leg].
- NAV-2 (config) parses finals/configs/arenas/<name>.json into an ArenaMap and
  HARDENS the validation that from_dict only sketches here (see the NAV-2 notes
  on each from_dict).
- NAV-5 (navigate phase) executes the list[Leg]; NAV-6 (land_on_pad) reads the
  LandingPad it is targeting.

Frame convention (matches flight/dead_reckon.py): the arena is a flat 2-D metric
plane in (north_m, east_m); headings are degrees CCW-positive — the SAME sign as
pyhulax yaw (dead_reckon uses psi_NED = -yaw_deg). Distances are metres EXCEPT
Leg.distance_cm, which is centimetres to match the FlightAdapter Move contract.

Pure stdlib — no SDK, and no top-level numpy (the conventions scan bans a
top-level numpy import in pure modules; planners that need it import lazily).

Session: S11 (NAV-0 contracts; the SEMANTIC from_dict validation —
bounds ordering, pads-within-bounds, unique ids, >= 3-distinct-vertex
polygons, c2-origin-within-bounds — is HARDENED in NAV-2).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, FrozenSet, Optional, Sequence, Tuple

from finals.errors import ConfigError

Point = Tuple[float, float]  # (north_m, east_m)

#: The five STATIC field-beacon ArUco ids the organizers published (2026-06-10;
#: docs/field_markers.md). NAV-FIX exposes this as a SOFT, OPT-IN config rule —
#: `ArenaMap.from_dict(..., known_marker_ids=KNOWN_FIELD_MARKER_IDS)` rejects any
#: marker id outside the set, catching a fat-fingered beacon-coordinate paste at
#: gate D. DEFAULT (known_marker_ids=None) imposes NO restriction so the sim
#: arenas + the test fixtures (which use placeholder ids like 7) stay green; the
#: real landing arena opts in. (We OWN Marker — this never touches Gate.)
KNOWN_FIELD_MARKER_IDS: FrozenSet[int] = frozenset({11, 45, 51, 67, 101})


# ============================================================
# Thin parse helpers (NAV-2 hardens the SEMANTIC rules; these only shape-check
# so a malformed arena dies as a ConfigError, never a raw TypeError deep in the
# planner — the config.py weights-guard philosophy).
# ============================================================
def _arena_keys(raw: Any, required: Tuple[str, ...], optional: Tuple[str, ...],
                where: str) -> Dict[str, Any]:
    if not isinstance(raw, dict):
        raise ConfigError(
            f"{where}: expected a JSON object, got {type(raw).__name__}")
    data = {k: v for k, v in raw.items() if not k.startswith("_")}
    unknown = sorted(set(data) - set(required) - set(optional))
    if unknown:
        raise ConfigError(
            f"{where}: unknown key(s) {unknown} — valid keys: "
            f"{sorted(set(required) | set(optional))} (typo?)")
    missing = sorted(set(required) - set(data))
    if missing:
        raise ConfigError(f"{where}: missing required key(s) {missing}")
    return data


def _point(raw: Any, where: str) -> Point:
    if (not isinstance(raw, (list, tuple)) or len(raw) != 2
            or any(not isinstance(c, (int, float)) or isinstance(c, bool)
                   or not math.isfinite(c) for c in raw)):
        # NAV-2: a non-finite coordinate (NaN/Inf) would silently escape every
        # bounds comparison below (NaN < x is always False), so it dies HERE.
        raise ConfigError(
            f"{where}: expected [north_m, east_m] finite numbers, got {raw!r}")
    return (float(raw[0]), float(raw[1]))


def _point_in_bounds(pt: Point,
                     bounds: Tuple[float, float, float, float]) -> bool:
    """True iff pt = (north_m, east_m) lies in the CLOSED rectangle
    bounds = (north_min, east_min, north_max, east_max). Closed (<=) on
    purpose: a pad taped flush to the arena edge is legal — the keep-in
    geofence is the wall, and the wall is inclusive. Pure helper shared by
    the pad / c2-origin checks so 'inside' has one definition."""
    north, east = pt
    north_min, east_min, north_max, east_max = bounds
    return (north_min <= north <= north_max
            and east_min <= east <= east_max)


# ============================================================
# Contracts
# ============================================================
@dataclass(frozen=True)
class Leg:
    """One transit segment: rotate to the ABSOLUTE compass heading_deg (CCW+,
    pyhulax yaw sign), then Move(FORWARD, distance_cm). distance_cm is cm to
    match the FlightAdapter Move contract; heading is degrees."""

    heading_deg: float
    distance_cm: float


@dataclass(frozen=True)
class KeepOut:
    """A 2-D no-fly polygon (obstacle footprint). polygon_m is a ring of
    (north_m, east_m) vertices. NAV-1 inflates it by the safety margin before
    planning; NAV-2 enforces the >= 3-vertex / simple-polygon rules."""

    id: str
    polygon_m: Tuple[Point, ...]

    @classmethod
    def from_dict(cls, raw: Any, index: int) -> "KeepOut":
        where = f"arena.keep_out[{index}]"
        data = _arena_keys(raw, required=("id", "polygon_m"), optional=(),
                           where=where)
        poly = data["polygon_m"]
        if not isinstance(poly, (list, tuple)):
            raise ConfigError(
                f"{where}.polygon_m must be a list of [north_m, east_m] points, "
                f"got {poly!r}")
        points = tuple(_point(p, f"{where}.polygon_m[{i}]")
                       for i, p in enumerate(poly))
        # NAV-2 semantic rule: a keep-out must enclose AREA. A polygon with
        # fewer than 3 distinct vertices is degenerate (a point or a line) —
        # NAV-1's inflate_polygon / point_in_polygon would treat it as empty
        # and the obstacle would silently vanish from the plan. We require
        # >= 3 DISTINCT vertices (duplicates and collinear repeats don't add a
        # corner); full simple-polygon / self-intersection is NAV-1's geometry
        # concern, not the config loader's.
        distinct = []
        for p in points:
            if p not in distinct:
                distinct.append(p)
        if len(distinct) < 3:
            raise ConfigError(
                f"{where}.polygon_m must have >= 3 DISTINCT vertices to enclose "
                f"area (a keep-out is an obstacle footprint, not a point/line) — "
                f"got {len(points)} vertices, {len(distinct)} distinct: "
                f"{[list(p) for p in distinct]}")
        return cls(id=str(data["id"]), polygon_m=points)


@dataclass(frozen=True)
class LandingPad:
    """A hoop landing target. center_m = (north_m, east_m); radius_m = hoop
    radius (m); valid mirrors the green(valid)/red(invalid) ArUco beside the
    pad. NAV-2 enforces center-within-bounds + unique ids."""

    id: str
    center_m: Point
    radius_m: float
    valid: bool

    @classmethod
    def from_dict(cls, raw: Any, index: int) -> "LandingPad":
        where = f"arena.pads[{index}]"
        data = _arena_keys(
            raw, required=("id", "center_m", "radius_m", "valid"),
            optional=(), where=where)
        radius = data["radius_m"]
        if (not isinstance(radius, (int, float)) or isinstance(radius, bool)
                or not math.isfinite(radius) or not radius > 0):
            raise ConfigError(
                f"{where}.radius_m must be a finite number > 0 (m, the hoop "
                f"radius), got {radius!r}")
        valid = data["valid"]
        if not isinstance(valid, bool):
            raise ConfigError(
                f"{where}.valid must be a boolean (green pad = true), "
                f"got {valid!r}")
        return cls(id=str(data["id"]),
                   center_m=_point(data["center_m"], f"{where}.center_m"),
                   radius_m=float(radius), valid=valid)


@dataclass(frozen=True)
class Marker:
    """A fixed-coordinate ArUco beacon at a KNOWN arena point. Two roles:
    (1) NAV-FIX absolute position-fix anchor (reset open-loop DR drift); and
    (2) per the 2026-06-10 intel, the LANDING approach target itself — each of
    the 5 field beacons (ids 11/45/51/67/101) sits ~20-30 cm from its landing
    pad, so navigating to a beacon's known coordinate puts a drone over the pad
    within a single servo/descend step. That adjacency is the scope-reducer
    around blind colour pad-search. id = the ArUco marker id; point_m =
    (north_m, east_m) in the arena frame. NAV-FIX hardens further rules (e.g.
    id restricted to the known field set, the beacon->pad offset vector)."""

    id: int
    point_m: Point

    @classmethod
    def from_dict(cls, raw: Any, index: int,
                  known_ids: "Optional[FrozenSet[int]]" = None) -> "Marker":
        """Parse one marker dict. `known_ids` is the NAV-FIX SOFT, OPT-IN rule:
        when a frozenset is supplied (the real arena passes
        KNOWN_FIELD_MARKER_IDS), an id outside it is a loud ConfigError —
        catching a mistyped beacon coordinate before it anchors a wrong
        position-fix or a wrong landing region. DEFAULT None = no restriction
        (sim arenas / test fixtures with placeholder ids stay valid)."""
        where = f"arena.markers[{index}]"
        data = _arena_keys(raw, required=("id", "point_m"), optional=(),
                           where=where)
        mid = data["id"]
        if not isinstance(mid, int) or isinstance(mid, bool):
            raise ConfigError(
                f"{where}.id must be an int ArUco marker id (the beacon code, "
                f"e.g. 11/45/51/67/101), got {mid!r}")
        if known_ids is not None and mid not in known_ids:
            raise ConfigError(
                f"{where}.id {mid} is not one of the known field-beacon ids "
                f"{sorted(known_ids)} — this arena opted into the strict "
                f"known-marker rule (KNOWN_FIELD_MARKER_IDS). Fix the id (a "
                f"mistyped beacon would anchor a wrong position-fix / landing "
                f"region) or drop the strict rule for this arena.")
        return cls(id=mid, point_m=_point(data["point_m"], f"{where}.point_m"))


@dataclass(frozen=True)
class Gate:
    """A traversable opening — an arch GAP / doorway the planner may route a
    leg THROUGH even though it sits between obstacle legs (NAV-ARCH). The arch
    posts are ordinary keep_out polygons; this Gate marks the passable slot
    between them. span_m = the two (north_m, east_m) endpoints of the opening
    line the drone crosses; clearance_m = the usable RAW opening width (m, the
    distance between the two arch posts; 0 = unspecified). NAV-ARCH consumes
    this in visibility_graph: an edge that PROPERLY CROSSES this span line is
    excused from the inflated arch-post keep-outs the gate sits between, IFF the
    drone fits — clearance_m >= 2*inflation_m (each post's inflation margin eats
    into the opening from its own side). clearance_m == 0 is NOT verifiable, so
    the planner refuses to fly such a gate (a zero/unspecified clearance never
    excuses a keep-out — fail closed).

    ALTITUDE: a gate is a HORIZONTAL opening (north/east). The drone flies the
    gate at its single, fixed transit altitude (the ~1.1 m no-band ceiling) — it
    canNOT climb over an arch, so there is no per-gate height field; the operator
    sets the one transit height under the arch crossbar at gate D. See
    field_markers.md / navigate.py.

    NAV-ARCH hardens the geometry: from_dict rejects a DEGENERATE (zero-length)
    span (an opening with both endpoints at one point is not an opening), and
    ArenaMap.from_dict cross-checks that the span sits in a REAL keep-out gap
    (a gate with no arch posts around it is a config mistake) and within bounds.
    """

    id: str
    span_m: Tuple[Point, Point]
    clearance_m: float

    @classmethod
    def from_dict(cls, raw: Any, index: int) -> "Gate":
        where = f"arena.gates[{index}]"
        data = _arena_keys(raw, required=("id", "span_m"),
                           optional=("clearance_m",), where=where)
        span = data["span_m"]
        if not isinstance(span, (list, tuple)) or len(span) != 2:
            raise ConfigError(
                f"{where}.span_m must be [[north_m, east_m], [north_m, east_m]] "
                f"— the two endpoints of the passable opening, got {span!r}")
        endpoints = tuple(_point(p, f"{where}.span_m[{i}]")
                          for i, p in enumerate(span))
        # NAV-ARCH geometry rule: the two span endpoints must be DISTINCT. A
        # zero-length span is not an opening — it would give the planner a
        # degenerate gate line that no edge can "properly cross", so the gate
        # would silently never apply (an arch the drone can never pass). Fail
        # loud at load time naming the collapsed point.
        if endpoints[0] == endpoints[1]:
            raise ConfigError(
                f"{where}.span_m is DEGENERATE — both endpoints are the same "
                f"point {list(endpoints[0])}; a gate span must be the two "
                f"DISTINCT endpoints of the opening line the drone crosses. "
                f"Check the two arch-post inner corners that bound the gap.")
        clearance = data.get("clearance_m", 0.0)
        if (not isinstance(clearance, (int, float))
                or isinstance(clearance, bool)
                or not math.isfinite(clearance) or clearance < 0):
            raise ConfigError(
                f"{where}.clearance_m must be a finite number >= 0 (m, the "
                f"usable opening width; 0 = unspecified), got {clearance!r}")
        return cls(id=str(data["id"]), span_m=endpoints,
                   clearance_m=float(clearance))


@dataclass(frozen=True)
class ArenaMap:
    """The Challenge-2A world: metric bounds, obstacle keep-outs, candidate
    landing pads, taped floor lanes, and the C2 launch frame (origin + the
    compass heading the drones boot aligned to). Loaded from
    finals/configs/arenas/<name>.json by config.py.

    bounds_m = (north_min_m, east_min_m, north_max_m, east_max_m).
    lanes = tuple of polylines (each a tuple of (north_m, east_m) points),
    ADVISORY only (taped-floor reference; never a hard control input).
    NAV-2 hardens: bounds ordering, pads within bounds, unique pad/keep-out ids.
    """

    bounds_m: Tuple[float, float, float, float]
    keep_out: Tuple[KeepOut, ...]
    pads: Tuple[LandingPad, ...]
    lanes: Tuple[Tuple[Point, ...], ...]
    c2_origin_m: Point
    c2_heading_deg: float
    markers: Tuple[Marker, ...] = ()   # NAV-FIX: fixed-coord beacon anchors (Step 0)
    gates: Tuple[Gate, ...] = ()       # NAV-ARCH: traversable arch openings (Step 0)
    #: ORIGIN-CAL onsite knob: Δ = the misalignment between the compass-yaw frame
    #: the drones boot in and arena-north, measured at gate D
    #: (Δ = boot_yaw_reading − arena_heading_aimed). navigate bakes Δ into every
    #: leg's Rotate target so the open-loop transit points the nose along the
    #: ARENA heading even when HULA yaw is relative-to-boot or magnetically
    #: rotated. DISTINCT from c2_heading_deg (which rotates Discord coords in
    #: frame.discord_to_ned — do NOT overload it). Default 0.0 = no offset
    #: (today's behaviour verbatim).
    heading_offset_deg: float = 0.0

    @classmethod
    def from_dict(cls, raw: Any, *, name: str,
                  known_marker_ids: "Optional[FrozenSet[int]]" = None
                  ) -> "ArenaMap":
        """Parse + HARDEN an arena map (see the NAV-2 semantic rules below).

        NAV-FIX known-marker rule (SOFT, OPT-IN), resolved in priority order:
        (1) an explicit `known_marker_ids` arg from a caller wins; else
        (2) a top-level JSON key `"strict_marker_ids": true` opts the arena into
            KNOWN_FIELD_MARKER_IDS (the real field-arena config self-declares
            this so a mistyped beacon id fails at gate D).
        With NEITHER, there is NO restriction (sim arenas + fixtures, which use
        placeholder ids, stay valid)."""
        where = f"arena {name!r}"
        data = _arena_keys(
            raw,
            required=("bounds_m", "c2_origin_m", "c2_heading_deg"),
            optional=("keep_out", "pads", "lanes", "markers", "gates",
                      "strict_marker_ids", "heading_offset_deg"),
            where=where,
        )
        # Resolve the known-marker rule: explicit arg wins; else honour the
        # JSON opt-in flag. A non-bool flag is a config bug (loud).
        if known_marker_ids is None and "strict_marker_ids" in data:
            flag = data["strict_marker_ids"]
            if not isinstance(flag, bool):
                raise ConfigError(
                    f"{where}.strict_marker_ids must be a boolean (true opts "
                    f"into the known field-beacon id rule "
                    f"{sorted(KNOWN_FIELD_MARKER_IDS)}), got {flag!r}")
            if flag:
                known_marker_ids = KNOWN_FIELD_MARKER_IDS
        bounds = data["bounds_m"]
        if (not isinstance(bounds, (list, tuple)) or len(bounds) != 4
                or any(not isinstance(c, (int, float)) or isinstance(c, bool)
                       or not math.isfinite(c) for c in bounds)):
            raise ConfigError(
                f"{where}.bounds_m must be [north_min, east_min, north_max, "
                f"east_max] finite numbers, got {bounds!r}")
        bounds_t = (float(bounds[0]), float(bounds[1]),
                    float(bounds[2]), float(bounds[3]))
        # NAV-2 semantic rule: bounds must be a NON-EMPTY rectangle. min < max
        # on both axes (strict: a zero-width arena has no interior to fly in,
        # and an inverted min/max — a transposed copy-paste — would make every
        # point-in-bounds check below vacuously false).
        north_min, east_min, north_max, east_max = bounds_t
        if not north_min < north_max:
            raise ConfigError(
                f"{where}.bounds_m north_min ({north_min}) must be < north_max "
                f"({north_max}) — bounds_m is [north_min, east_min, north_max, "
                f"east_max]; an empty/inverted north span means no arena. Check "
                f"the ordering / a transposed copy-paste.")
        if not east_min < east_max:
            raise ConfigError(
                f"{where}.bounds_m east_min ({east_min}) must be < east_max "
                f"({east_max}) — bounds_m is [north_min, east_min, north_max, "
                f"east_max]; an empty/inverted east span means no arena. Check "
                f"the ordering / a transposed copy-paste.")
        keep_out = tuple(KeepOut.from_dict(k, i)
                         for i, k in enumerate(_as_list(data.get("keep_out", []),
                                                         f"{where}.keep_out")))
        pads = tuple(LandingPad.from_dict(p, i)
                     for i, p in enumerate(_as_list(data.get("pads", []),
                                                    f"{where}.pads")))
        lanes_raw = _as_list(data.get("lanes", []), f"{where}.lanes")
        lanes = tuple(
            tuple(_point(pt, f"{where}.lanes[{i}][{j}]")
                  for j, pt in enumerate(_as_list(line, f"{where}.lanes[{i}]")))
            for i, line in enumerate(lanes_raw)
        )
        markers = tuple(Marker.from_dict(m, i, known_ids=known_marker_ids)
                        for i, m in enumerate(_as_list(
                            data.get("markers", []), f"{where}.markers")))
        gates = tuple(Gate.from_dict(g, i)
                      for i, g in enumerate(_as_list(
                          data.get("gates", []), f"{where}.gates")))
        heading = data["c2_heading_deg"]
        if (not isinstance(heading, (int, float)) or isinstance(heading, bool)
                or not math.isfinite(heading)):
            raise ConfigError(
                f"{where}.c2_heading_deg must be a finite number (deg, CCW+), "
                f"got {heading!r}")
        c2_origin = _point(data["c2_origin_m"], f"{where}.c2_origin_m")
        # ORIGIN-CAL heading offset: optional, finite, default 0.0. Same guard
        # shape as c2_heading_deg above, but a SEPARATE field (it offsets the
        # navigate Rotate target; c2_heading_deg rotates Discord coords). A NaN/
        # string here would silently mis-aim every leg, so fail loud at load.
        heading_offset = data.get("heading_offset_deg", 0.0)
        if (not isinstance(heading_offset, (int, float))
                or isinstance(heading_offset, bool)
                or not math.isfinite(heading_offset)):
            raise ConfigError(
                f"{where}.heading_offset_deg must be a finite number (deg, CCW+; "
                f"the onsite compass-yaw-vs-arena-north misalignment Δ; omit or "
                f"0.0 = no offset), got {heading_offset!r}")

        # ---- NAV-2 cross-cutting semantic rules -------------------------
        # IDs must be unique: NAV-1/NAV-5/NAV-6 address pads & keep-outs BY id;
        # a duplicate would make one shadow the other (which one you get is dict
        # order — a silent wrong-target bug). Report the FIRST collision.
        _require_unique_ids((p.id for p in pads), kind="pad",
                            where=f"{where}.pads")
        _require_unique_ids((k.id for k in keep_out), kind="keep-out",
                            where=f"{where}.keep_out")
        # Every pad center must lie within bounds: a pad outside the keep-in
        # geofence is unreachable; planning a leg to it would aim the swarm at
        # the wall. (Closed bounds — a pad flush to the edge is legal.)
        for i, pad in enumerate(pads):
            if not _point_in_bounds(pad.center_m, bounds_t):
                raise ConfigError(
                    f"{where}.pads[{i}] (id {pad.id!r}) center_m "
                    f"{list(pad.center_m)} is OUTSIDE bounds_m {list(bounds_t)} "
                    f"= [north_min, east_min, north_max, east_max] — a pad "
                    f"outside the arena is unreachable. Fix the center or widen "
                    f"bounds_m.")
        # The C2 launch origin must lie within bounds too (it is where the
        # swarm boots; an out-of-bounds origin means every relative leg starts
        # outside the geofence).
        if not _point_in_bounds(c2_origin, bounds_t):
            raise ConfigError(
                f"{where}.c2_origin_m {list(c2_origin)} is OUTSIDE bounds_m "
                f"{list(bounds_t)} = [north_min, east_min, north_max, "
                f"east_max] — the C2 launch point must sit inside the arena. "
                f"Fix c2_origin_m or widen bounds_m.")
        # Markers (NAV-FIX anchors / beacon landing targets): unique ids + every
        # beacon coordinate INSIDE bounds (a beacon outside the arena can anchor
        # neither a position-fix nor a landing).
        _require_unique_ids((str(m.id) for m in markers), kind="marker",
                            where=f"{where}.markers")
        for i, m in enumerate(markers):
            if not _point_in_bounds(m.point_m, bounds_t):
                raise ConfigError(
                    f"{where}.markers[{i}] (id {m.id}) point_m "
                    f"{list(m.point_m)} is OUTSIDE bounds_m {list(bounds_t)} — "
                    f"a beacon outside the arena cannot anchor a position-fix "
                    f"or a landing. Fix the point or widen bounds_m.")
        # Gates (NAV-ARCH traversable openings): unique ids + each span within
        # bounds + each span sits in a REAL keep-out gap (an arch the planner
        # can fly through must have arch POSTS around it — a gate floating in
        # open airspace excuses nothing and is almost always a mis-typed coord).
        _require_unique_ids((g.id for g in gates), kind="gate",
                            where=f"{where}.gates")
        _validate_gate_geometry(gates, keep_out, bounds_t, where=where)

        return cls(
            bounds_m=bounds_t,
            keep_out=keep_out,
            pads=pads,
            lanes=lanes,
            c2_origin_m=c2_origin,
            c2_heading_deg=float(heading),
            markers=markers,
            gates=gates,
            heading_offset_deg=float(heading_offset),
        )


def _as_list(raw: Any, where: str) -> Sequence[Any]:
    if not isinstance(raw, (list, tuple)):
        raise ConfigError(f"{where} must be a list, got {raw!r}")
    return raw


def _validate_gate_geometry(
        gates: Tuple["Gate", ...],
        keep_out: Tuple["KeepOut", ...],
        bounds: Tuple[float, float, float, float], *, where: str) -> None:
    """NAV-ARCH cross-cutting gate geometry: every gate span must (1) lie within
    bounds (an opening outside the arena is unreachable) and (2) sit in a REAL
    keep-out gap — its span line must touch at least one keep-out polygon (the
    arch posts the gate threads between). A gate with no keep-out around it
    excuses nothing in the planner (the visibility edge through it was never
    blocked), so it is a silent no-op at best and a typo'd coordinate at worst —
    refuse it loudly.

    The polygon-intersection geometry lives in polygon_tools (the planner's
    geometry home); imported LAZILY here so the contracts module stays
    stdlib-only at import time (the conventions scan + numpy-less venv)."""
    if not gates:
        return
    # Lazy import: keep types.py import-time pure (polygon_tools is leaf stdlib;
    # no cycle, but the contracts module declares no geometry dependency).
    from finals.mission.planning.polygon_tools import segment_intersects_polygon

    for i, g in enumerate(gates):
        a, b = g.span_m
        if not (_point_in_bounds(a, bounds) and _point_in_bounds(b, bounds)):
            raise ConfigError(
                f"{where}.gates[{i}] (id {g.id!r}) span_m "
                f"[{list(a)}, {list(b)}] is OUTSIDE bounds_m {list(bounds)} — "
                f"a gate opening outside the arena is unreachable. Fix the span "
                f"or widen bounds_m.")
        # The span must touch a keep-out (the arch posts). segment_intersects_
        # polygon counts a touching/crossing span — the opening line of a real
        # arch runs right up to (and between) its two posts.
        if not any(segment_intersects_polygon(a, b, ko.polygon_m)
                   for ko in keep_out):
            raise ConfigError(
                f"{where}.gates[{i}] (id {g.id!r}) span_m "
                f"[{list(a)}, {list(b)}] does not touch ANY keep-out — a gate is "
                f"the GAP between arch posts, so its span line must run up to "
                f"the keep-out polygons it threads between. With no keep-out "
                f"around it the gate excuses nothing (the planner was never "
                f"blocked here). CHECK: the span endpoints (a typo?) or add the "
                f"arch-post keep-outs.")


def _require_unique_ids(ids: Any, *, kind: str, where: str) -> None:
    """Raise ConfigError naming the FIRST duplicate id (NAV-2). ids is any
    iterable of strings (consumed once). Reporting the first collision —
    rather than a set diff — keeps the message actionable (the exact value to
    rename) and order-stable for the tests."""
    seen = set()
    for _id in ids:
        if _id in seen:
            raise ConfigError(
                f"{where}: duplicate {kind} id {_id!r} — every {kind} id must "
                f"be unique (NAV-1/NAV-5/NAV-6 address {kind}s by id; a "
                f"duplicate silently shadows one). Rename the collision.")
        seen.add(_id)
