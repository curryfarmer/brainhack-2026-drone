"""polygon_tools — PURE 2-D polygon geometry for the visibility-graph planner.

inflate_polygon grows a keep-out by the safety margin (drone radius + open-loop
drift budget + heading-error band); segment_intersects_polygon and
point_in_polygon are the collision predicates the A* edge test stands on. All
coordinates are (north_m, east_m).

Pure stdlib (numpy imported lazily by NAV-1 if needed — a top-level numpy import
in a pure module fails the conventions scan).

STUB — session S11 (NAV-1). See finals/docs/module_map.md.
"""
from __future__ import annotations

from typing import Sequence, Tuple

_STUB = ("finals.mission.planning.polygon_tools: session S11 (NAV-1) — "
         "see finals/docs/module_map.md")

Point = Tuple[float, float]  # (north_m, east_m)


def inflate_polygon(polygon_m: Sequence[Point],
                    inflation_m: float) -> Tuple[Point, ...]:
    """Outward Minkowski-style offset of a simple polygon by inflation_m. (NAV-1)"""
    raise NotImplementedError(_STUB)


def segment_intersects_polygon(a_m: Point, b_m: Point,
                               polygon_m: Sequence[Point]) -> bool:
    """True if segment a_m->b_m crosses an edge of or enters the polygon. (NAV-1)"""
    raise NotImplementedError(_STUB)


def point_in_polygon(point_m: Point, polygon_m: Sequence[Point]) -> bool:
    """True if point_m lies inside the polygon (ray-cast). (NAV-1)"""
    raise NotImplementedError(_STUB)
