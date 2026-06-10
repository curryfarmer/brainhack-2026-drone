"""polygon_tools — PURE 2-D polygon geometry for the visibility-graph planner.

inflate_polygon grows a keep-out by the safety margin (drone radius + open-loop
drift budget + heading-error band); segment_intersects_polygon and
point_in_polygon are the collision predicates the A* edge test stands on. All
coordinates are (north_m, east_m).

Geometry derivation / conventions (binding — pinned by tests/test_polygon_tools):
- inflate_polygon: per-vertex outward-normal offset. For each edge we compute
  the unit edge normal that points AWAY from the polygon interior (selected by
  the polygon's signed area / winding, so the result is OUTWARD for BOTH
  clockwise and counter-clockwise rings), then move each vertex along the
  miter of its two adjacent edge-normals by inflation_m / sin(half-angle). For
  the convex crate footprints we expect this is an exact Minkowski sum with a
  square-cornered offset (mitered corners, not rounded). NON-CONVEX vertices
  (reflex corners) over-shoot at sharp reentrants — acceptable here because (a)
  inflation only makes a keep-out LARGER (never smaller → never a missed
  collision) and (b) crate footprints are convex. Near-degenerate input (a
  zero-length edge / a spike whose miter blows up) is clamped: the miter scale
  is capped so a near-collinear corner cannot project to infinity. inflation_m
  == 0 returns the ring unchanged (as floats). A polygon with < 3 vertices is
  returned unchanged (there is no interior to grow — the planner never feeds
  one; NAV-2 enforces >= 3).
- point_in_polygon: even-odd ray cast (a horizontal ray in +east). Documented
  boundary behavior: a point exactly ON an edge or vertex is treated as
  OUTSIDE (returns False). The planner uses this only as a safety predicate on
  start/goal, where "exactly on the boundary" is not a flyable place anyway;
  the half-open edge test (north in [y0, y1) per edge) avoids the classic
  double-count at shared vertices.
- segment_intersects_polygon: True if the segment a->b crosses ANY polygon edge
  (proper or improper — shared endpoints/touching count as intersecting, so an
  edge that merely grazes a corner is rejected: conservative, never lets a
  grazing edge through) OR lies fully inside (tested via the segment midpoint).
  Built on an orientation (signed-area sign) test, the standard CLRS segment
  -intersection predicate, with collinear-overlap handled by on-segment checks.
- segment_enters_polygon: the LESS-conservative variant the visibility graph
  needs — True only if the segment PROPERLY crosses an edge (strict, so a
  touch at a shared corner does NOT block) or its midpoint is strictly inside.
  A graph edge to a polygon corner must be allowed to touch that corner, which
  the conservative predicate above forbids; see its docstring.

Pure stdlib math only (numpy is BANNED at top level in pure modules by
tests/test_conventions.py — the suite must stay green in a numpy-less venv).

Implemented — session S11 (NAV-1). See finals/docs/module_map.md.
"""
from __future__ import annotations

import math
from typing import Sequence, Tuple

Point = Tuple[float, float]  # (north_m, east_m)

# A corner sharper than this folds the miter scale toward infinity; cap it so a
# near-degenerate spike offsets by a bounded amount instead of NaN/inf. sin of
# ~0.057 deg => miter scale <= ~1000x inflation, which is already absurd for a
# crate footprint and only ever makes the keep-out (safely) larger.
_MIN_SIN_HALF_ANGLE = 1e-3
# Below this |signed area| (m^2) a ring is degenerate or self-intersecting
# (a bow-tie's lobes cancel to ~0): "outward" is undefined, so we fail loud
# rather than silently pick the inward normal and shrink the keep-out. Far
# below any real crate footprint (a 10 cm square is 1e-2 m^2).
_MIN_SIGNED_AREA_M2 = 1e-9


def _signed_area(polygon_m: Sequence[Point]) -> float:
    """Shoelace signed area in (north, east). Positive when the ring winds one
    way, negative the other — only its SIGN is used (to pick the outward
    normal), so the absolute orientation convention does not matter."""
    n = len(polygon_m)
    area = 0.0
    for i in range(n):
        n0, e0 = polygon_m[i]
        n1, e1 = polygon_m[(i + 1) % n]
        area += n0 * e1 - n1 * e0
    return area / 2.0


def inflate_polygon(polygon_m: Sequence[Point],
                    inflation_m: float) -> Tuple[Point, ...]:
    """Outward Minkowski-style offset of a simple polygon by inflation_m.

    Returns a new ring (same vertex count, same winding) every corner of which
    sits inflation_m farther OUT along its mitered outward normal. See the
    module docstring for the convexity assumption, the reflex-corner caveat,
    and the near-degenerate clamp. (NAV-1)
    """
    if not math.isfinite(inflation_m):
        raise ValueError(
            f"inflate_polygon: inflation_m must be finite, got {inflation_m!r} "
            f"— check the safety-margin computation that produced it")
    if inflation_m < 0.0:
        raise ValueError(
            f"inflate_polygon: inflation_m must be >= 0 (an OUTWARD offset), "
            f"got {inflation_m!r} — a negative offset would SHRINK the keep-out "
            f"and silently re-admit collisions; use 0 for no inflation")
    pts = [(float(n), float(e)) for n, e in polygon_m]
    n = len(pts)
    if n < 3 or inflation_m == 0.0:
        # No interior to grow (or nothing to do): return the points unchanged.
        return tuple(pts)

    # Sign of the signed area picks which perpendicular of each edge points OUT.
    # For a CCW ring (area > 0) the outward normal of edge (p_i -> p_{i+1}),
    # direction (dn, de), is (de, -dn) normalized; for a CW ring it is the
    # negation. We fold that into `s = +/-1`.
    area2 = _signed_area(pts)
    if abs(area2) <= _MIN_SIGNED_AREA_M2:
        raise ValueError(
            f"inflate_polygon: polygon signed area is ~0 ({area2!r} m^2) — the "
            f"ring is degenerate or self-intersecting (e.g. a bow-tie), so "
            f"'outward' is undefined and the offset would SHRINK it and silently "
            f"re-admit collisions; check the keep-out vertex order in the arena")
    s = 1.0 if area2 > 0.0 else -1.0

    # Per-edge OUTWARD unit normal, edge i = pts[i] -> pts[i+1].
    edge_normals = []
    for i in range(n):
        n0, e0 = pts[i]
        n1, e1 = pts[(i + 1) % n]
        dn, de = n1 - n0, e1 - e0
        length = math.hypot(dn, de)
        if length == 0.0:
            # Degenerate (duplicate) vertex: reuse a zero normal; the miter
            # below falls back to the neighbouring edge's normal.
            edge_normals.append((0.0, 0.0))
            continue
        # Right-perpendicular of (dn, de) is (de, -dn); `s` flips it outward.
        edge_normals.append((s * de / length, -s * dn / length))

    out = []
    for i in range(n):
        # Vertex i is shared by edge (i-1) and edge i. Offset along the miter
        # (sum of the two adjacent outward normals), scaled so the PERPENDICULAR
        # distance to each adjacent edge is exactly inflation_m.
        pn = edge_normals[(i - 1) % n]
        cn = edge_normals[i]
        mn, me = pn[0] + cn[0], pn[1] + cn[1]
        mlen = math.hypot(mn, me)
        if mlen == 0.0:
            # Opposed normals (a 180-degree spike) or both edges degenerate:
            # fall back to whichever single normal is non-zero, else no move.
            fb = cn if math.hypot(*cn) > 0.0 else pn
            mn, me = fb
            mlen = math.hypot(mn, me)
            if mlen == 0.0:
                out.append(pts[i])
                continue
        ux, uy = mn / mlen, me / mlen
        # sin(half-angle) = projection of a unit edge-normal onto the unit
        # miter direction = the dot of one adjacent normal with (ux, uy).
        sin_half = cn[0] * ux + cn[1] * uy
        if math.hypot(*cn) == 0.0:           # current edge degenerate, use prev
            sin_half = pn[0] * ux + pn[1] * uy
        sin_half = max(abs(sin_half), _MIN_SIN_HALF_ANGLE)
        scale = inflation_m / sin_half
        out.append((pts[i][0] + ux * scale, pts[i][1] + uy * scale))
    return tuple(out)


def _orient(a: Point, b: Point, c: Point) -> float:
    """Signed area sign of triangle a,b,c. > 0 / < 0 / == 0 for CCW / CW /
    collinear (north as x, east as y — a consistent right-handed convention,
    sign-only so the axis labelling is immaterial)."""
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: Point, b: Point, p: Point) -> bool:
    """True if collinear point p lies within the bounding box of segment a-b
    (the on-segment test used only when _orient says the three are collinear)."""
    return (min(a[0], b[0]) <= p[0] <= max(a[0], b[0])
            and min(a[1], b[1]) <= p[1] <= max(a[1], b[1]))


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    """True if segment a-b intersects segment c-d (CLRS predicate). Touching at
    an endpoint or collinear overlap COUNTS as intersecting (closed segments) —
    conservative for a collision test: a transit edge that merely grazes an
    obstacle corner is rejected rather than admitted."""
    o1 = _orient(a, b, c)
    o2 = _orient(a, b, d)
    o3 = _orient(c, d, a)
    o4 = _orient(c, d, b)
    # Proper crossing: c,d on opposite sides of ab AND a,b on opposite sides of cd.
    if ((o1 > 0) != (o2 > 0)) and ((o3 > 0) != (o4 > 0)) and o1 != 0 and o2 != 0 \
            and o3 != 0 and o4 != 0:
        return True
    # Collinear / touching endpoints.
    if o1 == 0 and _on_segment(a, b, c):
        return True
    if o2 == 0 and _on_segment(a, b, d):
        return True
    if o3 == 0 and _on_segment(c, d, a):
        return True
    if o4 == 0 and _on_segment(c, d, b):
        return True
    return False


def point_in_polygon(point_m: Point, polygon_m: Sequence[Point]) -> bool:
    """True if point_m lies strictly INSIDE the polygon (even-odd ray cast).

    A point exactly on an edge or vertex returns False (boundary == outside —
    see the module docstring). Uses a half-open per-edge test (north in
    [y0, y1)) so a ray grazing a shared vertex is counted once, not twice.
    Fewer than 3 vertices => no interior => False. (NAV-1)
    """
    n = len(polygon_m)
    if n < 3:
        return False
    px, py = float(point_m[0]), float(point_m[1])
    inside = False
    for i in range(n):
        x0, y0 = polygon_m[i]
        x1, y1 = polygon_m[(i + 1) % n]
        # An on-edge point is boundary => outside, per the documented contract.
        if _orient((x0, y0), (x1, y1), (px, py)) == 0.0 \
                and _on_segment((x0, y0), (x1, y1), (px, py)):
            return False
        # Half-open straddle on the north axis: does the +east ray from the
        # point cross this edge?
        if (y0 > py) != (y1 > py):
            x_at = x0 + (py - y0) * (x1 - x0) / (y1 - y0)
            if px < x_at:
                inside = not inside
    return inside


def segment_intersects_polygon(a_m: Point, b_m: Point,
                               polygon_m: Sequence[Point]) -> bool:
    """True if segment a_m->b_m crosses an edge of, or lies inside, the polygon.

    Two ways a segment "hits" a polygon: it crosses/touches some edge, or it is
    entirely interior (tested via the midpoint, since an edge-free interior
    segment never touches the boundary). Fewer than 3 vertices => no area =>
    only the edge test applies (always False for < 2 distinct points). (NAV-1)
    """
    n = len(polygon_m)
    if n < 3:
        return False
    a = (float(a_m[0]), float(a_m[1]))
    b = (float(b_m[0]), float(b_m[1]))
    for i in range(n):
        c = polygon_m[i]
        d = polygon_m[(i + 1) % n]
        if _segments_intersect(a, b, c, d):
            return True
    # No edge crossing: the segment is either fully outside or fully inside.
    mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    return point_in_polygon(mid, polygon_m)


def segment_enters_polygon(a_m: Point, b_m: Point,
                           polygon_m: Sequence[Point]) -> bool:
    """True if segment a_m->b_m enters the polygon INTERIOR (the visibility
    -graph edge test, distinct from the conservative segment_intersects_polygon).

    Why a second predicate: a visibility-graph edge legitimately TOUCHES the
    corners it connects (the polyline hugs the inflated obstacle), so the
    grazing-counts-as-hit rule of segment_intersects_polygon would reject every
    edge to a corner. Here an edge is blocked only if it PROPERLY crosses an
    edge (strict opposite-sides on both segments — collinear/endpoint touches
    do NOT block) OR its midpoint lies strictly inside. Running ALONG an
    inflated edge (collinear) is allowed: that is the optimal obstacle-hugging
    path and the inflation margin already buys the clearance. (NAV-1)
    """
    n = len(polygon_m)
    if n < 3:
        return False
    a = (float(a_m[0]), float(a_m[1]))
    b = (float(b_m[0]), float(b_m[1]))
    for i in range(n):
        c = polygon_m[i]
        d = polygon_m[(i + 1) % n]
        o1 = _orient(a, b, c)
        o2 = _orient(a, b, d)
        o3 = _orient(c, d, a)
        o4 = _orient(c, d, b)
        if (o1 != 0 and o2 != 0 and o3 != 0 and o4 != 0
                and (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)):
            return True
    mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    return point_in_polygon(mid, polygon_m)
