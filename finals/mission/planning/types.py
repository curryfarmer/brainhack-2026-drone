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
from typing import Any, Dict, Sequence, Tuple

from finals.errors import ConfigError

Point = Tuple[float, float]  # (north_m, east_m)


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

    @classmethod
    def from_dict(cls, raw: Any, *, name: str) -> "ArenaMap":
        where = f"arena {name!r}"
        data = _arena_keys(
            raw,
            required=("bounds_m", "c2_origin_m", "c2_heading_deg"),
            optional=("keep_out", "pads", "lanes"),
            where=where,
        )
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
        heading = data["c2_heading_deg"]
        if (not isinstance(heading, (int, float)) or isinstance(heading, bool)
                or not math.isfinite(heading)):
            raise ConfigError(
                f"{where}.c2_heading_deg must be a finite number (deg, CCW+), "
                f"got {heading!r}")
        c2_origin = _point(data["c2_origin_m"], f"{where}.c2_origin_m")

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

        return cls(
            bounds_m=bounds_t,
            keep_out=keep_out,
            pads=pads,
            lanes=lanes,
            c2_origin_m=c2_origin,
            c2_heading_deg=float(heading),
        )


def _as_list(raw: Any, where: str) -> Sequence[Any]:
    if not isinstance(raw, (list, tuple)):
        raise ConfigError(f"{where} must be a list, got {raw!r}")
    return raw


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
