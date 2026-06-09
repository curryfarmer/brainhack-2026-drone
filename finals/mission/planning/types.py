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

Session: S11 (NAV-0 contracts; from_dict validation is HARDENED in NAV-2).
"""
from __future__ import annotations

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
                   for c in raw)):
        raise ConfigError(
            f"{where}: expected [north_m, east_m] numbers, got {raw!r}")
    return (float(raw[0]), float(raw[1]))


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
        # NAV-2 enforces >= 3 vertices + non-self-intersecting; here we only
        # shape-check each vertex.
        points = tuple(_point(p, f"{where}.polygon_m[{i}]")
                       for i, p in enumerate(poly))
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
                or not radius > 0):
            raise ConfigError(
                f"{where}.radius_m must be a number > 0 (m, the hoop radius), "
                f"got {radius!r}")
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
                       for c in bounds)):
            raise ConfigError(
                f"{where}.bounds_m must be [north_min, east_min, north_max, "
                f"east_max] numbers, got {bounds!r}")
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
        if not isinstance(heading, (int, float)) or isinstance(heading, bool):
            raise ConfigError(
                f"{where}.c2_heading_deg must be a number (deg, CCW+), "
                f"got {heading!r}")
        return cls(
            bounds_m=(float(bounds[0]), float(bounds[1]),
                      float(bounds[2]), float(bounds[3])),
            keep_out=keep_out,
            pads=pads,
            lanes=lanes,
            c2_origin_m=_point(data["c2_origin_m"], f"{where}.c2_origin_m"),
            c2_heading_deg=float(heading),
        )


def _as_list(raw: Any, where: str) -> Sequence[Any]:
    if not isinstance(raw, (list, tuple)):
        raise ConfigError(f"{where} must be a list, got {raw!r}")
    return raw
