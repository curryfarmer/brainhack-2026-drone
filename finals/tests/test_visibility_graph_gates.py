"""NAV-ARCH — visibility-graph planning THROUGH arch gates.

An arch is two posts (or, naively, one solid block) the drone CANNOT overfly
(~1.1 m ceiling); the passable slot between the posts is a Step-0 Gate. NAV-ARCH
extends the A* edge test so an edge that PROPERLY CROSSES a fitting gate's span
is excused from the inflated arch-post keep-outs the gate threads between.

The headline contracts pinned here:
  * a plan THROUGH a gate SUCCEEDS where the SAME footprint with NO gate BLOCKS
    (routes around) or is impossible (walled in => PlanningError);
  * a genuinely blocked route still REFUSES (a gate elsewhere is NOT a free pass
    through an unrelated obstacle — no false gate);
  * a too-narrow gap (clearance_m < 2*inflation_m, or unspecified 0) does NOT
    fit the inflated drone => the gate is ignored, the post keeps blocking;
  * Gate.from_dict / ArenaMap.from_dict geometry validation is LOUD on malformed
    (degenerate span, span outside bounds, span in no keep-out gap).

Collision-freeness is asserted the physically-meaningful way (mirrors
test_visibility_graph): fly the returned Legs through the REAL DeadReckoner and
check the integrated track. The math is NEVER reimplemented here.

Pure stdlib + pytest only (numpy-less bare venv stays green).
"""
from __future__ import annotations

import pytest

from finals.errors import ConfigError, PlanningError
from finals.flight.dead_reckon import DeadReckoner, DRPose
from finals.mission.planning.polygon_tools import (inflate_polygon,
                                                   segment_enters_polygon)
from finals.mission.planning.types import ArenaMap, Gate, KeepOut
from finals.mission.planning.visibility_graph import plan
from finals.types import Direction, Move, Rotate

TOL_M = 1e-6


# ---------------- arena + DR helpers (mirror test_visibility_graph) ----------
def _arena(*keep_out, gates=(), bounds=(-100.0, -100.0, 100.0, 100.0)):
    return ArenaMap(bounds_m=bounds, keep_out=tuple(keep_out), pads=(),
                    lanes=(), c2_origin_m=(0.0, 0.0), c2_heading_deg=0.0,
                    gates=tuple(gates))


def fly(legs, start=(0.0, 0.0)):
    dr = DeadReckoner(DRPose(start[0], start[1], 0.0, 0.0))
    pts = [(dr.pose.north_m, dr.pose.east_m)]
    for leg in legs:
        cur = dr.pose.yaw_deg
        dr.note_action_complete(Rotate(angle_deg=leg.heading_deg - cur))
        dr.note_action_complete(
            Move(direction=Direction.FORWARD, distance_cm=leg.distance_cm))
        pts.append((dr.pose.north_m, dr.pose.east_m))
    return pts


def _hits(pts, polygon_m):
    return any(segment_enters_polygon(pts[i], pts[i + 1], polygon_m)
               for i in range(len(pts) - 1))


def _crosses_span(pts, span):
    """True iff some flown segment crosses the gate-span LINE (proved by the
    east-sign of the track flipping across the span's east at the span north).
    Here the gate span is the east line at north ~2; a north-bound transit at
    east 0 must pass between the post inner walls."""
    a, b = span
    # span endpoints share north; the opening is the east interval between them.
    e_lo, e_hi = sorted((a[1], b[1]))
    n_line = a[0]
    for i in range(len(pts) - 1):
        (n0, e0), (n1, e1) = pts[i], pts[i + 1]
        if (n0 - n_line) * (n1 - n_line) <= 0 and n0 != n1:
            # north crossed the span line; interpolate the east at the crossing
            t = (n_line - n0) / (n1 - n0)
            e_at = e0 + t * (e1 - e0)
            if e_lo - 1e-9 <= e_at <= e_hi + 1e-9:
                return True
    return False


# ============================================================
# FIXTURE A — a SOLID-BLOCK arch with a gate doorway carved through it.
# The intel's exact failure: a naive 2-D keep-out walls off a flyable passage.
# Block spans the whole width north 1..3, east -2..2; the doorway is the east
# [-0.5, 0.5] slot at the block, marked by a Gate. C2 (0,0) -> goal (4,0) is a
# straight north shot that the block BLOCKS and the gate REOPENS.
# ============================================================
def _solid_block():
    return KeepOut(id="arch_solid",
                   polygon_m=((1.0, -2.0), (1.0, 2.0), (3.0, 2.0), (3.0, -2.0)))


def _doorway_gate(clearance_m=1.0, gid="arch1"):
    # span = the opening line across the block at north 2, east -0.5..0.5.
    return Gate(id=gid, span_m=((2.0, -0.5), (2.0, 0.5)), clearance_m=clearance_m)


def test_no_gate_solid_block_forces_a_detour():
    """Baseline: the solid block with NO gate forces the planner AROUND it (the
    straight shot is illegal) — a multi-leg detour that clears the REAL block."""
    legs = plan((0.0, 0.0), (4.0, 0.0), _arena(_solid_block()),
                inflation_m=0.2, max_leg_cm=10_000.0)
    assert len(legs) >= 2                       # not a straight shot
    pts = fly(legs)
    assert pts[-1] == pytest.approx((4.0, 0.0), abs=TOL_M)
    assert not _hits(pts, _solid_block().polygon_m)
    # The detour did NOT go through the doorway slot (no gate => routed around).
    assert not _crosses_span(pts, _doorway_gate().span_m)


def test_through_gate_succeeds_where_solid_block_alone_detours():
    """THE headline: add the gate and the planner threads the doorway — a
    SHORTER, straighter route than the no-gate detour, and it crosses the gap."""
    block = _solid_block()
    gate = _doorway_gate(clearance_m=1.0)
    legs = plan((0.0, 0.0), (4.0, 0.0), _arena(block, gates=[gate]),
                inflation_m=0.2, max_leg_cm=10_000.0)
    pts = fly(legs)
    assert pts[-1] == pytest.approx((4.0, 0.0), abs=TOL_M)
    # The flown track passes THROUGH the doorway opening.
    assert _crosses_span(pts, gate.span_m)
    # And it is shorter than the forced no-gate detour around the block.
    around = plan((0.0, 0.0), (4.0, 0.0), _arena(block), inflation_m=0.2,
                  max_leg_cm=10_000.0)
    assert sum(l.distance_cm for l in legs) < sum(l.distance_cm for l in around)


def test_through_gate_is_a_straight_shot_for_a_centred_doorway():
    """A doorway centred on the straight line lets the planner fly the direct
    route (the gate excuses the block entirely for the centre edge)."""
    legs = plan((0.0, 0.0), (4.0, 0.0),
                _arena(_solid_block(), gates=[_doorway_gate(1.0)]),
                inflation_m=0.2, max_leg_cm=10_000.0)
    pts = fly(legs)
    # Direct shot: start -> goal in one straight line (subdivided or not, all
    # legs share the +north heading and the east never leaves the doorway).
    assert pts[-1] == pytest.approx((4.0, 0.0), abs=TOL_M)
    assert all(abs(e) < 1e-6 for _n, e in pts)
    assert len({round(l.heading_deg, 9) for l in legs}) == 1


# ============================================================
# FITS / TOO-NARROW — the clearance check.
# ============================================================
def test_too_narrow_gate_does_not_fit_and_still_detours():
    """clearance_m (0.3) < 2*inflation_m (0.4): the inflated drone does NOT fit
    the doorway, so the gate is IGNORED and the block forces the detour again —
    the planner never threads a gap too narrow for the safety radius."""
    block = _solid_block()
    narrow = _doorway_gate(clearance_m=0.3)
    legs = plan((0.0, 0.0), (4.0, 0.0), _arena(block, gates=[narrow]),
                inflation_m=0.2, max_leg_cm=10_000.0)
    pts = fly(legs)
    assert not _hits(pts, block.polygon_m)
    assert not _crosses_span(pts, narrow.span_m)   # did NOT use the narrow gate
    assert len(legs) >= 2                           # detoured around instead


def test_unspecified_clearance_zero_never_fits():
    """clearance_m == 0 (unspecified) can't be verified => never excuses the
    block. Same outcome as too-narrow: detour, not through."""
    block = _solid_block()
    gate = _doorway_gate(clearance_m=0.0)
    legs = plan((0.0, 0.0), (4.0, 0.0), _arena(block, gates=[gate]),
                inflation_m=0.2, max_leg_cm=10_000.0)
    pts = fly(legs)
    assert not _crosses_span(pts, gate.span_m)
    assert len(legs) >= 2


def test_gate_fits_exactly_at_the_boundary():
    """clearance_m == 2*inflation_m exactly fits (>=, inclusive) => threads."""
    block = _solid_block()
    gate = _doorway_gate(clearance_m=0.4)          # == 2 * 0.2
    legs = plan((0.0, 0.0), (4.0, 0.0), _arena(block, gates=[gate]),
                inflation_m=0.2, max_leg_cm=10_000.0)
    assert _crosses_span(fly(legs), gate.span_m)


# ============================================================
# NO FALSE GATE — a gate must not open an UNRELATED obstacle.
# ============================================================
def test_gate_does_not_excuse_an_unrelated_obstacle():
    """A solid block sits ON the straight line with NO gate of its own; a VALID
    gate is declared elsewhere (touching a second, off-path keep-out). The
    unrelated gate must NOT open the on-path block — the straight shot stays
    illegal, so the plan detours and clears the block. Proves a gate excuses
    only the posts its OWN span threads, never an arbitrary obstacle that a
    transit edge happens to cross (no false free pass)."""
    on_path = _solid_block()                       # north 1..3, east -2..2
    # A second keep-out far to the east, with its own (irrelevant) gate.
    side = KeepOut(id="side",
                   polygon_m=((6.0, 5.0), (6.0, 9.0), (8.0, 9.0), (8.0, 5.0)))
    elsewhere = Gate(id="g_side", span_m=((7.0, 6.5), (7.0, 7.5)),
                     clearance_m=1.0)
    legs = plan((0.0, 0.0), (4.0, 0.0), _arena(on_path, side, gates=[elsewhere]),
                inflation_m=0.2, max_leg_cm=10_000.0)
    pts = fly(legs)
    assert pts[-1] == pytest.approx((4.0, 0.0), abs=TOL_M)
    assert not _hits(pts, on_path.polygon_m)       # on-path block still avoided
    # The on-path block was NOT excused (no gate touches it) => a real detour,
    # not the straight shot through the doorway slot.
    assert len(legs) >= 2
    assert not _crosses_span(pts, _doorway_gate().span_m)
    assert any(abs(e) > 0.1 for _n, e in pts)      # had to leave the centreline


def test_walled_in_goal_with_no_fitting_gate_still_raises():
    """A goal boxed in by an arch wall with NO fitting gate => PlanningError
    (NOT a silent path through the wall). The gate's clearance is too small to
    fit, so it can't rescue the route."""
    # A C-shaped pocket: three walls + a front wall that fully blocks. The only
    # opening is the front, sealed by inflation, and the gate there is too narrow.
    front = KeepOut(id="front",
                    polygon_m=((1.0, -3.0), (1.0, 3.0), (1.5, 3.0), (1.5, -3.0)))
    left = KeepOut(id="left",
                   polygon_m=((1.0, -3.0), (5.0, -3.0), (5.0, -2.5), (1.0, -2.5)))
    right = KeepOut(id="right",
                    polygon_m=((1.0, 2.5), (5.0, 2.5), (5.0, 3.0), (1.0, 3.0)))
    back = KeepOut(id="back",
                   polygon_m=((5.0, -3.0), (5.0, 3.0), (5.5, 3.0), (5.5, -3.0)))
    too_narrow = Gate(id="g", span_m=((1.25, -0.2), (1.25, 0.2)),
                      clearance_m=0.3)             # < 2*0.4
    with pytest.raises(PlanningError, match="NO collision-free path"):
        plan((0.0, 0.0), (3.0, 0.0),
             _arena(front, left, right, back, gates=[too_narrow]),
             inflation_m=0.4, max_leg_cm=10_000.0)


def test_dog_leg_through_an_off_axis_gate():
    """The doorway is OFF the direct start->goal line (a two-post arch with the
    gap to the east). The straight shot misses the opening, so the planner must
    DOG-LEG: veer to the doorway, thread it, and continue. The wall spans the
    arena (no going around) so the gate is the only way through. Proves the gate
    routes a non-straight path through an off-centre opening, reaching the goal
    and crossing the actual doorway interval."""
    wall_w = KeepOut(id="ww", polygon_m=((1.9, -5.0), (1.9, 0.8),
                                         (2.1, 0.8), (2.1, -5.0)))
    wall_e = KeepOut(id="we", polygon_m=((1.9, 1.4), (1.9, 5.0),
                                         (2.1, 5.0), (2.1, 1.4)))
    gate = Gate(id="door", span_m=((2.0, 0.8), (2.0, 1.4)), clearance_m=0.6)
    legs = plan((0.0, 0.0), (4.0, 0.0),
                _arena(wall_w, wall_e, gates=[gate],
                       bounds=(-5.0, -5.0, 5.0, 5.0)),
                inflation_m=0.15, max_leg_cm=10_000.0)
    pts = fly(legs)
    assert pts[-1] == pytest.approx((4.0, 0.0), abs=TOL_M)
    assert not _hits(pts, wall_w.polygon_m)
    assert not _hits(pts, wall_e.polygon_m)
    assert _crosses_span(pts, gate.span_m)         # threaded the off-axis gap
    assert any(abs(e) > 0.1 for _n, e in pts)      # genuinely dog-legged east


def test_fitting_gate_opens_the_walled_in_goal():
    """The SAME walled-in pocket, but a fitting gate in the front wall reopens
    it => a plan exists and reaches the goal through the doorway."""
    front = KeepOut(id="front",
                    polygon_m=((1.0, -3.0), (1.0, 3.0), (1.5, 3.0), (1.5, -3.0)))
    left = KeepOut(id="left",
                   polygon_m=((1.0, -3.0), (5.0, -3.0), (5.0, -2.5), (1.0, -2.5)))
    right = KeepOut(id="right",
                    polygon_m=((1.0, 2.5), (5.0, 2.5), (5.0, 3.0), (1.0, 3.0)))
    back = KeepOut(id="back",
                   polygon_m=((5.0, -3.0), (5.0, 3.0), (5.5, 3.0), (5.5, -3.0)))
    gate = Gate(id="door", span_m=((1.25, -0.9), (1.25, 0.9)), clearance_m=1.8)
    legs = plan((0.0, 0.0), (3.0, 0.0),
                _arena(front, left, right, back, gates=[gate]),
                inflation_m=0.4, max_leg_cm=10_000.0)
    pts = fly(legs)
    assert pts[-1] == pytest.approx((3.0, 0.0), abs=TOL_M)
    assert _crosses_span(pts, gate.span_m)


# ============================================================
# MULTIPLE arches in series — each gate is independent.
# ============================================================
def test_two_arches_in_series_both_threaded():
    """Two solid-block arches on the path, each with its own doorway gate. The
    planner threads BOTH openings as a straight north shot — gates are
    independent and compose. (The straight line traverses the block FOOTPRINTS
    via their doorways, so we assert it crosses both SPANS, not that it avoids
    the solid blocks — the gate IS the legal hole through the block.)"""
    block1 = KeepOut(id="arch1",
                     polygon_m=((1.0, -2.0), (1.0, 2.0), (2.0, 2.0), (2.0, -2.0)))
    block2 = KeepOut(id="arch2",
                     polygon_m=((4.0, -2.0), (4.0, 2.0), (5.0, 2.0), (5.0, -2.0)))
    g1 = Gate(id="g1", span_m=((1.5, -0.5), (1.5, 0.5)), clearance_m=1.0)
    g2 = Gate(id="g2", span_m=((4.5, -0.5), (4.5, 0.5)), clearance_m=1.0)
    legs = plan((0.0, 0.0), (6.0, 0.0),
                _arena(block1, block2, gates=[g1, g2]),
                inflation_m=0.2, max_leg_cm=10_000.0)
    pts = fly(legs)
    assert pts[-1] == pytest.approx((6.0, 0.0), abs=TOL_M)
    assert _crosses_span(pts, g1.span_m)           # arch1 doorway threaded
    assert _crosses_span(pts, g2.span_m)           # arch2 doorway threaded
    # Straight shot: one heading, east stays in both doorways (never detoured).
    assert len({round(l.heading_deg, 9) for l in legs}) == 1
    assert all(abs(e) < 0.5 for _n, e in pts)


# ============================================================
# DETERMINISM + no-gate equivalence (the no-regression contract).
# ============================================================
def test_gates_do_not_change_a_gate_free_arena():
    """With NO gates declared, the plan is byte-for-byte the pre-NAV-ARCH plan
    (same arena, same legs) — the gate code is inert when there are no gates."""
    box = KeepOut(id="c", polygon_m=((1.0, -1.0), (1.0, 1.0),
                                     (3.0, 1.0), (3.0, -1.0)))
    a = plan((0.0, 0.0), (4.0, 0.0), _arena(box), 0.3, 10_000.0)
    # An IDENTICAL arena built with the gates=() default must plan identically.
    b = plan((0.0, 0.0), (4.0, 0.0), _arena(box, gates=()), 0.3, 10_000.0)
    assert [(l.heading_deg, l.distance_cm) for l in a] \
        == [(l.heading_deg, l.distance_cm) for l in b]


def test_through_gate_plan_is_deterministic():
    block = _solid_block()
    gate = _doorway_gate(1.0)
    a = plan((0.0, 0.0), (4.0, 0.0), _arena(block, gates=[gate]), 0.2, 10_000.0)
    b = plan((0.0, 0.0), (4.0, 0.0), _arena(block, gates=[gate]), 0.2, 10_000.0)
    assert [(l.heading_deg, l.distance_cm) for l in a] \
        == [(l.heading_deg, l.distance_cm) for l in b]


# ============================================================
# Gate.from_dict / ArenaMap.from_dict GEOMETRY VALIDATION (loud on malformed).
# ============================================================
def _raw_arena(**extra):
    raw = {"bounds_m": [-10.0, -10.0, 10.0, 10.0], "c2_origin_m": [0.0, 0.0],
           "c2_heading_deg": 0.0,
           "keep_out": [{"id": "post", "polygon_m": [[1.0, -1.0], [1.0, 1.0],
                                                     [3.0, 1.0], [3.0, -1.0]]}]}
    raw.update(extra)
    return raw


def test_from_dict_degenerate_span_is_loud():
    """A zero-length span (both endpoints identical) is not an opening — loud."""
    with pytest.raises(ConfigError, match="DEGENERATE"):
        ArenaMap.from_dict(
            _raw_arena(gates=[{"id": "g", "span_m": [[2.0, 0.5], [2.0, 0.5]],
                              "clearance_m": 1.0}]), name="t")


def test_from_dict_span_in_no_keepout_gap_is_loud():
    """A gate whose span touches NO keep-out excuses nothing — loud (a typo'd
    coord or a gate floating in open airspace)."""
    with pytest.raises(ConfigError, match="does not touch ANY keep-out"):
        ArenaMap.from_dict(
            _raw_arena(gates=[{"id": "g", "span_m": [[8.0, 8.0], [8.0, 9.0]],
                              "clearance_m": 1.0}]), name="t")


def test_from_dict_span_outside_bounds_is_loud():
    with pytest.raises(ConfigError, match="OUTSIDE bounds"):
        ArenaMap.from_dict(
            _raw_arena(gates=[{"id": "g", "span_m": [[2.0, 0.0], [2.0, 99.0]],
                              "clearance_m": 1.0}]), name="t")


def test_from_dict_valid_gate_in_a_keepout_gap_loads():
    """A well-formed gate whose span runs through the post keep-out parses."""
    a = ArenaMap.from_dict(
        _raw_arena(gates=[{"id": "arch", "span_m": [[2.0, -0.5], [2.0, 0.5]],
                          "clearance_m": 1.0}]), name="t")
    assert a.gates[0].id == "arch"
    assert a.gates[0].span_m == ((2.0, -0.5), (2.0, 0.5))


def test_from_dict_negative_clearance_is_loud():
    with pytest.raises(ConfigError, match="clearance_m"):
        ArenaMap.from_dict(
            _raw_arena(gates=[{"id": "g", "span_m": [[2.0, -0.5], [2.0, 0.5]],
                              "clearance_m": -0.5}]), name="t")
