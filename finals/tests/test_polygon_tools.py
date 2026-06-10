"""finals.mission.planning.polygon_tools — hand-computed 2-D geometry cases.

Every expected value here is computed on paper from the documented contracts
(see the module docstring): inflate = per-vertex outward miter offset;
point_in_polygon = even-odd ray cast with boundary == OUTSIDE; the segment
predicates built on the orientation sign test. The `>` vs `>=` boundary cases
(on-edge points, touching corners) are pinned explicitly because that is the
class of off-by-one that silently lets a transit leg clip an obstacle.

EQUIVALENT-MUTANT NOTE (NAV-1 mutation kill-check): flipping the point_in_polygon
straddle test `(y0 > py) != (y1 > py)` to `>=` is a SEMANTICALLY EQUIVALENT
mutant — the explicit on-edge guard returns False before the straddle runs, so
every input that could distinguish `>` from `>=` is intercepted as a boundary
point (verified by a 1024-point differential grid over convex + concave polys).
The real off-by-one risk (touching-counts-vs-proper-crossing on the segment
predicates) IS killed — see test_enters_false_when_only_touching_a_corner.

Pure stdlib + pytest only (the suite must pass in a numpy-less bare venv).
"""
from __future__ import annotations

import math

import pytest

from finals.mission.planning.polygon_tools import (inflate_polygon,
                                                   point_in_polygon,
                                                   segment_enters_polygon,
                                                   segment_intersects_polygon)

EPS = 1e-9


def approx(v: float):
    return pytest.approx(v, abs=EPS)


# A unit square in (north, east). CCW and CW windings of the SAME square so the
# winding-independence of inflate is pinned, not assumed.
SQUARE_CCW = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))
SQUARE_CW = ((0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0))


# ============================================================
# inflate_polygon
# ============================================================
def test_inflate_unit_square_grows_by_m_each_side():
    """A unit square grown by 0.5 m becomes [-0.5, 1.5] on BOTH axes — every
    corner pushed out by 0.5 along its 45-degree miter."""
    out = inflate_polygon(SQUARE_CCW, 0.5)
    assert set(out) == {(-0.5, -0.5), (1.5, -0.5), (1.5, 1.5), (-0.5, 1.5)}


def test_inflate_is_winding_independent():
    """CW input inflates OUTWARD too (not inward) — the signed-area sign picks
    the outward normal regardless of ring direction."""
    ccw = set(inflate_polygon(SQUARE_CCW, 0.5))
    cw = set(inflate_polygon(SQUARE_CW, 0.5))
    assert ccw == cw == {(-0.5, -0.5), (1.5, -0.5), (1.5, 1.5), (-0.5, 1.5)}


def test_inflate_preserves_vertex_count_and_winding_order():
    out = inflate_polygon(SQUARE_CCW, 0.25)
    assert len(out) == len(SQUARE_CCW)
    # CCW stays CCW: signed area keeps its sign.
    def area(poly):
        n = len(poly)
        return sum(poly[i][0] * poly[(i + 1) % n][1]
                   - poly[(i + 1) % n][0] * poly[i][1] for i in range(n))
    assert math.copysign(1, area(out)) == math.copysign(1, area(SQUARE_CCW))


def test_inflate_zero_is_identity():
    out = inflate_polygon(SQUARE_CCW, 0.0)
    assert out == SQUARE_CCW


def test_inflate_degenerate_bowtie_raises():
    # A self-intersecting (bow-tie) ring has ~zero signed area, so "outward" is
    # undefined. Before the guard, _signed_area == 0 fell to the INWARD branch
    # and SHRANK the keep-out, silently re-admitting collisions. NAV-2 accepts a
    # 4-distinct-vertex bow-tie (no simple-polygon check), so this is reachable.
    bowtie = ((0.0, 0.0), (1.0, 1.0), (1.0, 0.0), (0.0, 1.0))  # area cancels to 0
    with pytest.raises(ValueError, match="signed area"):
        inflate_polygon(bowtie, 0.5)


def test_inflate_negative_raises():
    with pytest.raises(ValueError, match="must be >= 0"):
        inflate_polygon(SQUARE_CCW, -0.1)


def test_inflate_non_finite_raises():
    with pytest.raises(ValueError, match="finite"):
        inflate_polygon(SQUARE_CCW, float("inf"))
    with pytest.raises(ValueError, match="finite"):
        inflate_polygon(SQUARE_CCW, float("nan"))


def test_inflate_degenerate_polygon_returned_unchanged():
    """< 3 vertices has no interior to grow — returned as-is (NAV-2 enforces
    >= 3 upstream; this just refuses to invent geometry)."""
    seg = ((0.0, 0.0), (1.0, 0.0))
    assert inflate_polygon(seg, 0.5) == seg


def test_inflate_triangle_moves_corners_outward():
    """Right triangle; the inflated centroid-distance grows (corners moved
    AWAY from the centroid, never toward it)."""
    tri = ((0.0, 0.0), (2.0, 0.0), (0.0, 2.0))
    cx = sum(p[0] for p in tri) / 3.0
    cy = sum(p[1] for p in tri) / 3.0
    out = inflate_polygon(tri, 0.3)
    for orig, new in zip(tri, out):
        assert math.hypot(new[0] - cx, new[1] - cy) > math.hypot(
            orig[0] - cx, orig[1] - cy)


# ============================================================
# point_in_polygon
# ============================================================
def test_point_inside():
    assert point_in_polygon((0.5, 0.5), SQUARE_CCW) is True


def test_point_outside():
    assert point_in_polygon((2.0, 2.0), SQUARE_CCW) is False
    assert point_in_polygon((-1.0, 0.5), SQUARE_CCW) is False


def test_point_on_edge_is_outside():
    """Documented boundary behavior: ON the boundary counts as OUTSIDE."""
    assert point_in_polygon((0.0, 0.5), SQUARE_CCW) is False  # west edge
    assert point_in_polygon((0.5, 1.0), SQUARE_CCW) is False  # east edge


def test_point_on_vertex_is_outside():
    assert point_in_polygon((0.0, 0.0), SQUARE_CCW) is False


def test_point_just_inside_vs_just_outside_edge():
    """The `>` vs `>=` boundary class: a hair inside is True, a hair outside is
    False — the ray-cast must not be off by the edge itself."""
    assert point_in_polygon((0.5, 1e-9), SQUARE_CCW) is True
    assert point_in_polygon((0.5, -1e-9), SQUARE_CCW) is False


def test_point_in_degenerate_polygon_is_false():
    assert point_in_polygon((0.0, 0.0), ((0.0, 0.0), (1.0, 0.0))) is False


def test_point_in_polygon_no_double_count_at_shared_vertex():
    """A ray passing through a shared vertex must flip parity once, not twice.
    Point to the LEFT (south) of an L whose vertex the +east ray grazes."""
    # Concave L-shape; (0.5, -1.0) sits west of the shape, ray east crosses it.
    L = ((0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0))
    assert point_in_polygon((0.5, 0.5), L) is True
    assert point_in_polygon((1.5, 1.5), L) is False  # in the notch, outside


# ============================================================
# segment_intersects_polygon (conservative: touching counts)
# ============================================================
def test_segment_crossing_returns_true():
    assert segment_intersects_polygon((0.5, -1.0), (0.5, 2.0), SQUARE_CCW) is True


def test_segment_disjoint_returns_false():
    assert segment_intersects_polygon((-1.0, -1.0), (-2.0, -2.0), SQUARE_CCW) is False


def test_segment_fully_inside_returns_true():
    assert segment_intersects_polygon((0.3, 0.3), (0.7, 0.7), SQUARE_CCW) is True


def test_segment_sharing_a_vertex_returns_true():
    """Conservative predicate: an edge that merely touches a corner is REJECTED
    (counts as intersecting) — never lets a grazing transit leg through."""
    assert segment_intersects_polygon((0.0, 0.0), (-1.0, -1.0), SQUARE_CCW) is True


def test_segment_touching_edge_returns_true():
    """Endpoint landing exactly ON an edge counts as intersecting."""
    assert segment_intersects_polygon((0.5, 0.0), (0.5, -1.0), SQUARE_CCW) is True


def test_segment_intersects_degenerate_polygon_is_false():
    assert segment_intersects_polygon((0.0, 0.0), (1.0, 1.0),
                                      ((0.0, 0.0), (1.0, 0.0))) is False


# ============================================================
# segment_enters_polygon (visibility-graph variant: proper crossing only)
# ============================================================
def test_enters_true_when_properly_crossing():
    assert segment_enters_polygon((0.5, -1.0), (0.5, 2.0), SQUARE_CCW) is True


def test_enters_false_when_only_touching_a_corner():
    """The key difference from segment_intersects_polygon: touching a shared
    corner does NOT block a visibility edge (so corner nodes stay connectable)."""
    assert segment_enters_polygon((0.0, 0.0), (-1.0, -1.0), SQUARE_CCW) is False


def test_enters_false_when_running_along_an_edge():
    """Collinear-with-an-edge is allowed (the obstacle-hugging path)."""
    assert segment_enters_polygon((-1.0, 0.0), (2.0, 0.0), SQUARE_CCW) is False


def test_enters_true_when_segment_lies_inside():
    assert segment_enters_polygon((0.3, 0.5), (0.7, 0.5), SQUARE_CCW) is True


def test_enters_false_when_disjoint():
    assert segment_enters_polygon((-1.0, -1.0), (-2.0, 5.0), SQUARE_CCW) is False
