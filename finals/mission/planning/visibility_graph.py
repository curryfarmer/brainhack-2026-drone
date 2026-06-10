"""visibility_graph — PURE visibility-graph A* path planner (Challenge 2A).

plan(start_m, goal_m, arena, inflation_m, max_leg_cm) inflates the keep-out
polygons by inflation_m, builds a visibility graph over {start, goal, inflated
corners}, A*-searches the shortest collision-free polyline, then converts it to
a list[Leg] (absolute heading_deg CCW+, distance_cm) — subdividing any leg
longer than max_leg_cm so cumulative open-loop drift stays under the inflation
margin. Raises PlanningError when no collision-free path exists or the
start/goal sits inside an inflated keep-out.

HEADING CONVENTION (binding — pinned by the heading-consistency test against the
REAL DeadReckoner, NOT reimplemented here):
  flight/dead_reckon.py: at yaw theta (deg, CCW+, 0 = +north), a FORWARD move of
  d metres advances the world position by (dN, dE) = (d*cos(theta),
  -d*sin(theta)). NAV-5 executes each Leg as "Rotate-to-absolute(heading_deg);
  Move(FORWARD, distance_cm)". Inverting that forward map for a desired world
  delta (dN, dE): cos(theta) = dN/|v|, sin(theta) = -dE/|v|, hence
      heading_deg = degrees(atan2(-dE, dN))   (normalized to (-180, 180]).
  Spot check: a pure +east goal (dN=0, dE>0) => atan2(-dE, 0) = -90 deg; at yaw
  -90 the drone faces east and FORWARD moves it +east. Matches dead_reckon.

Algorithm (binding):
- Visibility graph: nodes = {start, goal, every inflated-polygon corner}; an
  edge exists between two nodes iff the straight segment between them does not
  ENTER any inflated polygon's interior (polygon_tools.segment_enters_polygon —
  the proper-crossing variant, so an edge is ALLOWED to touch the corner it
  connects; the conservative segment_intersects_polygon would forbid every
  corner edge). Corners that lie inside ANOTHER inflated polygon are dropped
  (they are not reachable / not useful).
- A*: euclidean edge cost, euclidean straight-line heuristic to the goal
  (admissible + consistent => optimal). Deterministic tie-break by an insertion
  counter so the same arena always yields the same plan (replayability).
- Leg conversion: consecutive polyline points -> heading_deg (above) +
  distance_cm = euclidean_metres * 100, with zero-length segments dropped. Any
  leg longer than max_leg_cm is split into ceil(distance/max) EQUAL sub-legs of
  the SAME heading (bounds per-move open-loop drift under the inflation margin).

Pure stdlib only (heapq + math). numpy is BANNED at top level in pure modules
(tests/test_conventions.py); none is needed. Consumes finals.mission.planning
.types + polygon_tools + finals.errors.PlanningError.

Implemented — session S11 (NAV-1). See finals/docs/module_map.md.
"""
from __future__ import annotations

import heapq
import math
from typing import Dict, List, Sequence, Tuple

from finals.errors import PlanningError
from finals.mission.planning.polygon_tools import (inflate_polygon,
                                                   point_in_polygon,
                                                   segment_enters_polygon)
from finals.mission.planning.types import ArenaMap, Leg

Point = Tuple[float, float]  # (north_m, east_m)


def _require_finite_point(p: Point, what: str) -> Point:
    """A NaN/Inf coordinate would poison every distance/orientation downstream
    with NO exception (math.isfinite is the only guard) — reject it loudly at
    the boundary, like dead_reckon._require_finite does for poses."""
    if (not isinstance(p, (list, tuple)) or len(p) != 2
            or not math.isfinite(p[0]) or not math.isfinite(p[1])):
        raise ValueError(
            f"visibility_graph.plan: {what} must be a finite (north_m, east_m) "
            f"point, got {p!r} — check the caller that computed it (an "
            f"uninitialized telemetry field or a config divide-by-zero?)")
    return (float(p[0]), float(p[1]))


def _heading_deg(frm: Point, to: Point) -> float:
    """Absolute yaw target (deg, CCW+, normalized (-180, 180]) that makes a
    DeadReckoner FORWARD move advance frm->to. Inverse of dead_reckon's forward
    map: heading = atan2(-dE, dN). See the module docstring derivation."""
    dn = to[0] - frm[0]
    de = to[1] - frm[1]
    deg = math.degrees(math.atan2(-de, dn))
    # Normalize to (-180, 180] (matches dead_reckon.normalize_yaw_deg, kept
    # local so this pure module needs no flight import). atan2 already returns
    # (-180, 180], but -180 must fold to +180 for a single representation.
    if deg <= -180.0:
        deg += 360.0
    return deg


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def plan(start_m: Point, goal_m: Point, arena: ArenaMap, inflation_m: float,
         max_leg_cm: float) -> List[Leg]:
    """Collision-free transit plan start_m->goal_m as an ordered list of Legs.

    Raises ValueError on non-finite / out-of-domain args (inflation_m < 0,
    max_leg_cm <= 0, non-finite start/goal) and PlanningError when the
    start/goal is trapped inside an inflated keep-out or no collision-free path
    exists. (NAV-1)
    """
    start = _require_finite_point(start_m, "start_m")
    goal = _require_finite_point(goal_m, "goal_m")
    if not math.isfinite(inflation_m):
        raise ValueError(
            f"visibility_graph.plan: inflation_m must be finite, got "
            f"{inflation_m!r} — check the safety-margin computation")
    if inflation_m < 0.0:
        raise ValueError(
            f"visibility_graph.plan: inflation_m must be >= 0 (the OUTWARD "
            f"keep-out margin), got {inflation_m!r} — a negative margin would "
            f"shrink obstacles and re-admit collisions")
    if not math.isfinite(max_leg_cm) or max_leg_cm <= 0.0:
        raise ValueError(
            f"visibility_graph.plan: max_leg_cm must be a finite number > 0 "
            f"(the open-loop sub-leg cap, cm), got {max_leg_cm!r} — a 0/neg cap "
            f"would make subdivision divide-by-zero or loop forever")

    inflated: List[Tuple[str, Tuple[Point, ...]]] = [
        (ko.id, inflate_polygon(ko.polygon_m, inflation_m))
        for ko in arena.keep_out
    ]

    # Fail loud (and SPECIFIC) when an endpoint is trapped: name the keep-out.
    for ko_id, poly in inflated:
        if point_in_polygon(start, poly):
            raise PlanningError(
                f"visibility_graph.plan: start {start} lies INSIDE inflated "
                f"keep-out {ko_id!r} (margin {inflation_m} m) — cannot plan a "
                f"transit out of an obstacle. CHECK: is the drone's takeoff "
                f"point really clear of that crate, or is inflation_m too "
                f"large for this arena?")
    for ko_id, poly in inflated:
        if point_in_polygon(goal, poly):
            raise PlanningError(
                f"visibility_graph.plan: goal {goal} lies INSIDE inflated "
                f"keep-out {ko_id!r} (margin {inflation_m} m) — cannot land in "
                f"an obstacle. CHECK: the goal/pad coordinates and whether "
                f"inflation_m ({inflation_m} m) has swallowed the pad.")

    # ---- Build the visibility-graph nodes. ----
    # node 0 = start, node 1 = goal, then the inflated corners that are not
    # buried inside SOME OTHER inflated polygon.
    nodes: List[Point] = [start, goal]
    for idx, (_ko_id, poly) in enumerate(inflated):
        for corner in poly:
            corner = (float(corner[0]), float(corner[1]))
            if any(j != idx and point_in_polygon(corner, other)
                   for j, (_oid, other) in enumerate(inflated)):
                continue
            nodes.append(corner)

    polyline = _astar(nodes, inflated)
    if polyline is None:
        raise PlanningError(
            f"visibility_graph.plan: NO collision-free path from start {start} "
            f"to goal {goal} around {len(inflated)} inflated keep-out(s) "
            f"(margin {inflation_m} m). The goal is reachable airspace but "
            f"every route is blocked. CHECK: is inflation_m ({inflation_m} m) "
            f"too large (corridors pinched shut), or do the keep-out polygons "
            f"box the goal in?")

    return _polyline_to_legs(polyline, max_leg_cm)


def _astar(nodes: Sequence[Point],
           inflated: Sequence[Tuple[str, Tuple[Point, ...]]]
           ) -> List[Point] | None:
    """A* over the visibility graph. nodes[0] = start, nodes[1] = goal. Edge
    (i, j) exists iff the segment is collision-free against every inflated
    polygon. Returns the shortest collision-free polyline (list of points) or
    None if the goal is unreachable. Euclidean cost + euclidean heuristic =>
    admissible/consistent => the polyline is shortest."""
    n = len(nodes)
    start_i, goal_i = 0, 1
    polys = [poly for _id, poly in inflated]

    def visible(i: int, j: int) -> bool:
        seg_a, seg_b = nodes[i], nodes[j]
        for poly in polys:
            if segment_enters_polygon(seg_a, seg_b, poly):
                return False
        return True

    def h(i: int) -> float:
        return _dist(nodes[i], nodes[goal_i])

    g: Dict[int, float] = {start_i: 0.0}
    came_from: Dict[int, int] = {}
    counter = 0  # deterministic tie-break (heap is otherwise order-undefined)
    # heap entries: (f, tie, node)
    open_heap: List[Tuple[float, int, int]] = [(h(start_i), counter, start_i)]
    closed = set()

    # Bounded (convention 3): every push records a STRICT g-improvement and each
    # node finalizes into `closed` at most once over n finite nodes, so the heap
    # drains in finite steps — no deadline needed (pure in-memory search).
    while open_heap:
        _f, _tie, current = heapq.heappop(open_heap)
        if current == goal_i:
            return _reconstruct(came_from, current, nodes)
        if current in closed:
            continue
        closed.add(current)
        for nxt in range(n):
            if nxt == current or nxt in closed:
                continue
            if not visible(current, nxt):
                continue
            tentative = g[current] + _dist(nodes[current], nodes[nxt])
            if nxt not in g or tentative < g[nxt]:
                g[nxt] = tentative
                came_from[nxt] = current
                counter += 1
                heapq.heappush(open_heap, (tentative + h(nxt), counter, nxt))
    return None


def _reconstruct(came_from: Dict[int, int], current: int,
                 nodes: Sequence[Point]) -> List[Point]:
    path_idx = [current]
    while current in came_from:
        current = came_from[current]
        path_idx.append(current)
    path_idx.reverse()
    return [nodes[i] for i in path_idx]


def _polyline_to_legs(polyline: Sequence[Point], max_leg_cm: float) -> List[Leg]:
    """Turn a point polyline into Legs: per segment a heading_deg + distance_cm
    (= euclidean metres * 100), splitting any leg longer than max_leg_cm into
    ceil(dist/max) EQUAL same-heading sub-legs. Zero-length segments are
    dropped (a degenerate start==goal yields an empty plan — NAV-5 treats that
    as 'already there')."""
    legs: List[Leg] = []
    for a, b in zip(polyline, polyline[1:]):
        dist_m = _dist(a, b)
        if dist_m == 0.0:
            continue
        heading = _heading_deg(a, b)
        total_cm = dist_m * 100.0
        # Number of equal sub-legs so each is <= max_leg_cm.
        n_sub = max(1, math.ceil(total_cm / max_leg_cm))
        sub_cm = total_cm / n_sub
        for _ in range(n_sub):
            legs.append(Leg(heading_deg=heading, distance_cm=sub_cm))
    return legs
