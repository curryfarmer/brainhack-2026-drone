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
- GATES (NAV-ARCH): an arch is keep_out post(s) (ordinary polygons) whose
  inflated footprint pinches shut — or, naively modelled, a single solid block
  spanning the opening — so every segment through the gap PROPERLY CROSSES an
  inflated post and the plain edge test rejects it, walling off a passage that
  is actually flyable. A declared ArenaMap.Gate marks that gap (span_m = the two
  opening-line endpoints, clearance_m = the raw opening width). The edge test is
  EXTENDED: an edge that enters an inflated post is EXCUSED for that post iff
  some FITTING gate — clearance_m >= 2*inflation_m (the inflation margins eat
  inflation_m off each side of the opening) AND whose span touches that post —
  is PROPERLY CROSSED by the edge. An edge is legal iff EVERY inflated post it
  enters is excused this way (a post with no fitting+crossed gate still blocks,
  so an unrelated obstacle on the same line is never opened). clearance_m == 0
  (unspecified) or < 2*inflation_m NEVER fits => the gate is ignored and the
  post keeps blocking (fail CLOSED — the planner routes around or raises, never
  threads a gap too narrow for the inflated drone). The route THROUGH a gate is
  carried by the ordinary graph edges (the direct start->goal edge and the
  inflated-corner edges) that the exemption now lets cross the opening — no
  special gate node is needed (an on-span midpoint would be collinear with the
  span, so no edge to it could "properly cross" it — a dead node). With NO gates
  the node set + edge test are byte-for-byte the pre-NAV-ARCH behaviour.
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
                                                   segment_enters_polygon,
                                                   segment_intersects_polygon)
from finals.mission.planning.types import ArenaMap, Gate, Leg

Point = Tuple[float, float]  # (north_m, east_m)


def _segments_properly_cross(a: Point, b: Point, c: Point, d: Point) -> bool:
    """True iff open segments a-b and c-d PROPERLY cross (each strictly
    separates the other's endpoints — a shared endpoint or a collinear touch
    does NOT count). The gate test: the transit edge a-b must genuinely pass
    THROUGH the opening line c-d, not merely graze its endpoint. Local
    orientation math (pure stdlib) so this module stays heapq+math only."""
    def orient(p: Point, q: Point, r: Point) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)
    return (o1 != 0.0 and o2 != 0.0 and o3 != 0.0 and o4 != 0.0
            and (o1 > 0.0) != (o2 > 0.0) and (o3 > 0.0) != (o4 > 0.0))


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

    # ---- NAV-ARCH: resolve the FITTING gates (clearance >= 2*inflation, span
    # touches >= 1 inflated post). A gate that does not fit the inflated drone,
    # or sits in no keep-out, is dropped HERE so the rest of the planner is
    # gate-aware only where a real, flyable opening exists. With no gates this
    # is an empty list and every downstream gate branch is skipped. ----
    fitting = _fitting_gates(arena.gates, inflated, inflation_m)

    # ---- Build the visibility-graph nodes. ----
    # node 0 = start, node 1 = goal, then the inflated corners that are not
    # buried inside SOME OTHER inflated polygon. NAV-ARCH adds NO gate-specific
    # nodes: the gate exemption (in _astar.visible) lets the ordinary start/goal
    # /corner edges cross the opening, which is what carries the route through.
    nodes: List[Point] = [start, goal]
    for idx, (_ko_id, poly) in enumerate(inflated):
        for corner in poly:
            corner = (float(corner[0]), float(corner[1]))
            if any(j != idx and point_in_polygon(corner, other)
                   for j, (_oid, other) in enumerate(inflated)):
                continue
            nodes.append(corner)

    polyline = _astar(nodes, inflated, fitting)
    if polyline is None:
        raise PlanningError(
            f"visibility_graph.plan: NO collision-free path from start {start} "
            f"to goal {goal} around {len(inflated)} inflated keep-out(s) "
            f"(margin {inflation_m} m). The goal is reachable airspace but "
            f"every route is blocked. CHECK: is inflation_m ({inflation_m} m) "
            f"too large (corridors pinched shut), or do the keep-out polygons "
            f"box the goal in?")

    return _polyline_to_legs(polyline, max_leg_cm)


class _GateInfo:
    """A FITTING gate (resolved by _fitting_gates): its id, span endpoints, and
    the set of inflated-keep-out indices its span touches (the arch posts it
    threads between). An edge that PROPERLY crosses `span` is excused from
    entering any post index in `post_idx`."""

    __slots__ = ("gid", "span_a", "span_b", "post_idx")

    def __init__(self, gid: str, span_a: Point, span_b: Point,
                 post_idx: frozenset):
        self.gid = gid
        self.span_a = span_a
        self.span_b = span_b
        self.post_idx = post_idx


def _fitting_gates(gates: Sequence[Gate],
                   inflated: Sequence[Tuple[str, Tuple[Point, ...]]],
                   inflation_m: float) -> List[_GateInfo]:
    """Resolve the gates that ACTUALLY apply: clearance fits the inflated drone
    AND the span touches >= 1 inflated post. Returns one _GateInfo per fitting
    gate (empty when there are no gates). A gate is dropped (NOT an error) when:
      * clearance_m <= 0 (unspecified) or < 2*inflation_m — the inflated posts'
        margins (inflation_m each, off opposite sides) leave < the drone's width
        in the opening, so threading it would clip a post. Fail CLOSED: the post
        keeps blocking and the planner routes around / raises.
      * its span touches no inflated post — there is nothing to excuse (the
        edge through it was never blocked); harmless, so just ignore it.
    ArenaMap.from_dict already rejected a degenerate span and a span in no RAW
    keep-out; this is the INFLATION-aware fit check the loader cannot do."""
    out: List[_GateInfo] = []
    if not gates:
        return out
    min_clear = 2.0 * inflation_m
    for g in gates:
        # clearance_m == 0 is "unspecified" => unverifiable => never fits.
        if g.clearance_m <= 0.0 or g.clearance_m < min_clear:
            continue
        a = (float(g.span_m[0][0]), float(g.span_m[0][1]))
        b = (float(g.span_m[1][0]), float(g.span_m[1][1]))
        touched = frozenset(
            idx for idx, (_id, poly) in enumerate(inflated)
            if segment_intersects_polygon(a, b, poly))
        if not touched:
            continue
        out.append(_GateInfo(g.id, a, b, touched))
    return out


def _astar(nodes: Sequence[Point],
           inflated: Sequence[Tuple[str, Tuple[Point, ...]]],
           fitting: Sequence[_GateInfo] = ()) -> List[Point] | None:
    """A* over the visibility graph. nodes[0] = start, nodes[1] = goal. Edge
    (i, j) exists iff the segment is collision-free against every inflated
    polygon, EXCEPT a post entry excused by a fitting gate the edge properly
    crosses (NAV-ARCH). Returns the shortest collision-free polyline (list of
    points) or None if the goal is unreachable. Euclidean cost + euclidean
    heuristic => admissible/consistent => the polyline is shortest."""
    n = len(nodes)
    start_i, goal_i = 0, 1
    polys = [poly for _id, poly in inflated]

    def visible(i: int, j: int) -> bool:
        seg_a, seg_b = nodes[i], nodes[j]
        # Which fitting gates does THIS edge properly cross? (Computed once per
        # edge, only when gates exist — the no-gate path is unchanged.)
        crossed = [gi for gi in fitting
                   if _segments_properly_cross(seg_a, seg_b, gi.span_a,
                                               gi.span_b)] if fitting else ()
        for idx, poly in enumerate(polys):
            if not segment_enters_polygon(seg_a, seg_b, poly):
                continue
            # The edge enters inflated post `idx`. Legal ONLY if a gate the edge
            # crosses excuses exactly THIS post (the gate threads between its
            # own posts; an unrelated obstacle on the same line is NOT excused).
            if any(idx in gi.post_idx for gi in crossed):
                continue
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
