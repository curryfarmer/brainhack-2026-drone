"""finals.mission.planning.visibility_graph — A* planner + Leg conversion.

The load-bearing test here is test_heading_consistency_with_dead_reckon: it
feeds plan()'s Legs through the REAL flight.dead_reckon.DeadReckoner (rotate to
each absolute heading, then Move FORWARD) and asserts the integrated NED
position lands on the goal. That pins NAV-1's heading derivation to the EXACT
forward model NAV-5 will execute — if either side flips a sign, this test goes
red. The math is NEVER reimplemented here; the DeadReckoner is the oracle.

Collision-freeness is asserted against the REAL (un-inflated) keep-out, flying
the returned Legs through the DeadReckoner: that is the physically meaningful
claim (the actual drone must not hit the actual crate). The inflation margin is
exactly the budget that absorbs the open-loop drift of hugging the boundary.

Pure stdlib + pytest only (numpy-less bare venv stays green).
"""
from __future__ import annotations

import math

import pytest

from finals.errors import PlanningError
from finals.flight.dead_reckon import DeadReckoner, DRPose
from finals.mission.planning.polygon_tools import (inflate_polygon,
                                                   segment_enters_polygon)
from finals.mission.planning.types import ArenaMap, KeepOut
from finals.mission.planning.visibility_graph import plan
from finals.types import Direction, Move, Rotate

TOL_M = 1e-6


def arena(*keep_out, bounds=(-100.0, -100.0, 100.0, 100.0)):
    return ArenaMap(bounds_m=bounds, keep_out=tuple(keep_out), pads=(),
                    lanes=(), c2_origin_m=(0.0, 0.0), c2_heading_deg=0.0)


def fly(legs, start=(0.0, 0.0)):
    """Execute the Legs through the REAL DeadReckoner (the NAV-5 forward model):
    rotate to each absolute heading, then Move FORWARD. Returns the ordered
    list of waypoints [start, ...]. Math lives in dead_reckon, never here."""
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


# ============================================================
# Straight shot — no obstacle
# ============================================================
def test_straight_shot_single_leg():
    legs = plan((0.0, 0.0), (3.0, 4.0), arena(), inflation_m=0.5,
                max_leg_cm=10_000.0)
    assert len(legs) == 1
    assert legs[0].distance_cm == pytest.approx(500.0)  # 5 m euclidean


def test_straight_shot_reaches_goal_under_dead_reckon():
    goal = (3.0, 4.0)
    legs = plan((0.0, 0.0), goal, arena(), inflation_m=0.5, max_leg_cm=10_000.0)
    end = fly(legs)[-1]
    assert end[0] == pytest.approx(goal[0], abs=TOL_M)
    assert end[1] == pytest.approx(goal[1], abs=TOL_M)


def test_total_distance_is_euclidean():
    start, goal = (1.0, -2.0), (4.0, 2.0)
    legs = plan(start, goal, arena(), inflation_m=0.2, max_leg_cm=10_000.0)
    total_cm = sum(l.distance_cm for l in legs)
    euclid_cm = math.hypot(goal[0] - start[0], goal[1] - start[1]) * 100.0
    assert total_cm == pytest.approx(euclid_cm, abs=1e-6)


# ============================================================
# Detour around one box
# ============================================================
def test_detour_around_one_box_reaches_goal_collision_free():
    box = KeepOut(id="crate1", polygon_m=((1.0, -1.0), (1.0, 1.0),
                                          (3.0, 1.0), (3.0, -1.0)))
    legs = plan((0.0, 0.0), (4.0, 0.0), arena(box), inflation_m=0.3,
                max_leg_cm=10_000.0)
    assert len(legs) >= 2          # straight line would cut the box
    pts = fly(legs)
    assert pts[-1][0] == pytest.approx(4.0, abs=TOL_M)
    assert pts[-1][1] == pytest.approx(0.0, abs=TOL_M)
    # Flying the legs must clear the REAL crate (margin absorbs DR drift).
    assert not _hits(pts, box.polygon_m)


def test_detour_path_goes_around_not_through_inflated_box():
    box = KeepOut(id="crate1", polygon_m=((1.0, -1.0), (1.0, 1.0),
                                          (3.0, 1.0), (3.0, -1.0)))
    legs = plan((0.0, 0.0), (4.0, 0.0), arena(box), inflation_m=0.3,
                max_leg_cm=10_000.0)
    # Every leg, evaluated at its ideal heading, must stay out of the inflated
    # box interior (the planner's contract, before any DR drift).
    pts = fly(legs)
    inflated = inflate_polygon(box.polygon_m, 0.3)
    # Allow boundary-hugging: assert no segment enters the slightly-shrunk
    # inflated box, proving the route really detoured (didn't cut through).
    shrunk = inflate_polygon(box.polygon_m, 0.29)
    assert not _hits(pts, shrunk)


def test_planner_returns_the_optimal_path_not_a_greedy_one():
    # OPTIMALITY pin (S-PLAN review HIGH: every other detour test asserts only
    # reachability + clearance, so a planner that returns a LONGER collision-free
    # path passes them all). Two asymmetric stacked boxes form a greedy TRAP:
    # B1 pokes east (so its near detour is WEST), B2 pokes west (near detour
    # EAST). A greedy-best-first search (heuristic only, no g) dives toward the
    # goal-ward corner each step and zig-zags ~16.7 m; the true shortest weaves
    # the other way for ~11.08 m. Asserting the EXACT optimal length KILLS the
    # greedy-best-first mutant (drop the g term in the priority key — verified it
    # returns ~16.7 m here and fails this assert). NOTE: an inadmissible-weight
    # (h*1.5) or a `<`->`<=` relaxation-retie mutant are EQUIVALENT mutants (both
    # still return the exact optimum on this unique-optimum graph), so no detour
    # fixture can kill them — they are not real defects. A* with an admissible
    # euclidean heuristic must return exactly the optimum.
    b1 = KeepOut(id="b1", polygon_m=((2.0, -0.5), (2.0, 4.0),
                                     (4.0, 4.0), (4.0, -0.5)))
    b2 = KeepOut(id="b2", polygon_m=((6.0, -4.0), (6.0, 0.5),
                                     (8.0, 0.5), (8.0, -4.0)))
    legs = plan((0.0, 0.0), (10.0, 0.0), arena(b1, b2), inflation_m=0.3,
                max_leg_cm=100_000.0)
    pts = fly(legs)
    assert pts[-1][0] == pytest.approx(10.0, abs=TOL_M)
    assert pts[-1][1] == pytest.approx(0.0, abs=TOL_M)
    assert not _hits(pts, b1.polygon_m)
    assert not _hits(pts, b2.polygon_m)
    total_cm = sum(l.distance_cm for l in legs)
    assert total_cm == pytest.approx(1108.369, abs=1.0)   # optimum; greedy ~1672


# ============================================================
# Corridor between two boxes
# ============================================================
def test_corridor_between_two_boxes():
    lower = KeepOut(id="lo", polygon_m=((1.0, -3.0), (1.0, -0.5),
                                        (3.0, -0.5), (3.0, -3.0)))
    upper = KeepOut(id="hi", polygon_m=((1.0, 0.5), (1.0, 3.0),
                                        (3.0, 3.0), (3.0, 0.5)))
    legs = plan((0.0, 0.0), (4.0, 0.0), arena(lower, upper), inflation_m=0.1,
                max_leg_cm=10_000.0)
    pts = fly(legs)
    assert pts[-1][0] == pytest.approx(4.0, abs=TOL_M)
    assert pts[-1][1] == pytest.approx(0.0, abs=TOL_M)
    assert not _hits(pts, lower.polygon_m)
    assert not _hits(pts, upper.polygon_m)


def test_goal_walled_in_raises_no_path():
    """Four thin walls leave a gap the drone can slip through at inflation 0,
    but a fat inflation seals every gap => no collision-free path => loud
    PlanningError (NOT a silent empty plan or a path THROUGH a wall)."""
    # A square pocket around goal (5,5) with a 0.6 m gap on the west wall.
    walls = (
        KeepOut(id="w_n", polygon_m=((6.0, 4.0), (6.5, 4.0), (6.5, 6.0), (6.0, 6.0))),
        KeepOut(id="w_s", polygon_m=((3.5, 4.0), (4.0, 4.0), (4.0, 6.0), (3.5, 6.0))),
        KeepOut(id="w_e", polygon_m=((4.0, 6.0), (6.0, 6.0), (6.0, 6.5), (4.0, 6.5))),
        # West wall split into two leaving a gap at north ~5 (4.7..5.3 open).
        KeepOut(id="w_w1", polygon_m=((4.0, 3.5), (4.7, 3.5), (4.7, 4.0), (4.0, 4.0))),
        KeepOut(id="w_w2", polygon_m=((5.3, 3.5), (6.0, 3.5), (6.0, 4.0), (5.3, 4.0))),
    )
    # Inflation 0.4 m seals the 0.6 m gap (each side grows 0.4 => 0.8 > 0.6).
    with pytest.raises(PlanningError, match="NO collision-free path"):
        plan((0.0, 0.0), (5.0, 5.0), arena(*walls), inflation_m=0.4,
             max_leg_cm=10_000.0)


# ============================================================
# Leg subdivision
# ============================================================
def test_subdivision_equal_sub_legs_summing_to_total():
    # 10 m straight = 1000 cm; cap 100 cm => 10 equal legs.
    legs = plan((0.0, 0.0), (0.0, 10.0), arena(), inflation_m=0.0,
                max_leg_cm=100.0)
    assert len(legs) == 10
    assert all(l.distance_cm <= 100.0 + 1e-9 for l in legs)
    assert {round(l.distance_cm, 9) for l in legs} == {100.0}
    assert sum(l.distance_cm for l in legs) == pytest.approx(1000.0)
    # All sub-legs share the SAME heading (a straight line was not bent).
    assert len({round(l.heading_deg, 9) for l in legs}) == 1


def test_subdivision_non_divisible_cap():
    # 5 m = 500 cm; cap 150 cm => ceil(500/150) = 4 legs of 125 cm each.
    legs = plan((0.0, 0.0), (5.0, 0.0), arena(), inflation_m=0.0,
                max_leg_cm=150.0)
    assert len(legs) == 4
    assert all(l.distance_cm <= 150.0 + 1e-9 for l in legs)
    assert sum(l.distance_cm for l in legs) == pytest.approx(500.0)


def test_subdivision_still_reaches_goal_under_dead_reckon():
    goal = (6.0, -2.0)
    legs = plan((0.0, 0.0), goal, arena(), inflation_m=0.0, max_leg_cm=50.0)
    assert len(legs) > 1
    end = fly(legs)[-1]
    assert end[0] == pytest.approx(goal[0], abs=TOL_M)
    assert end[1] == pytest.approx(goal[1], abs=TOL_M)


# ============================================================
# THE heading-consistency pin (load-bearing NAV-1 <-> NAV-5 contract)
# ============================================================
@pytest.mark.parametrize("goal", [
    (5.0, 0.0),     # due +north
    (0.0, 5.0),     # due +east
    (-5.0, 0.0),    # due -north
    (0.0, -5.0),    # due -east
    (3.0, 4.0),     # diagonal
    (-2.5, 7.1),    # arbitrary
])
def test_heading_consistency_with_dead_reckon(goal):
    """Feed plan()'s Legs through the REAL DeadReckoner and assert it lands on
    the goal. This pins NAV-1's heading_deg derivation to the EXACT NAV-5
    forward model — the single test that keeps the two sessions in sync."""
    legs = plan((0.0, 0.0), goal, arena(), inflation_m=0.0, max_leg_cm=10_000.0)
    end = fly(legs)[-1]
    assert end[0] == pytest.approx(goal[0], abs=TOL_M), f"north off for {goal}"
    assert end[1] == pytest.approx(goal[1], abs=TOL_M), f"east off for {goal}"


def test_heading_consistency_with_detour_and_subdivision():
    """The pin must hold for a MULTI-leg, subdivided, obstacle-avoiding plan —
    not just a single straight shot."""
    box = KeepOut(id="crate1", polygon_m=((1.0, -1.0), (1.0, 1.0),
                                          (3.0, 1.0), (3.0, -1.0)))
    goal = (4.0, 0.0)
    legs = plan((0.0, 0.0), goal, arena(box), inflation_m=0.4, max_leg_cm=60.0)
    end = fly(legs)[-1]
    assert end[0] == pytest.approx(goal[0], abs=TOL_M)
    assert end[1] == pytest.approx(goal[1], abs=TOL_M)


# ============================================================
# Fail-loud: trapped endpoints + bad args
# ============================================================
def test_goal_inside_keepout_raises_actionable():
    box = KeepOut(id="crateZ", polygon_m=((1.0, 1.0), (1.0, 5.0),
                                          (5.0, 5.0), (5.0, 1.0)))
    with pytest.raises(PlanningError) as ei:
        plan((0.0, 0.0), (3.0, 3.0), arena(box), inflation_m=0.2,
             max_leg_cm=10_000.0)
    msg = str(ei.value)
    # WHAT/WHICH/WHY/CHECK: names the goal, the keep-out id, the cause, and a check.
    assert "goal" in msg and "(3.0, 3.0)" in msg
    assert "crateZ" in msg
    assert "INSIDE" in msg
    assert "CHECK" in msg


def test_goal_inside_keepout_only_via_inflation_raises():
    """Goal just OUTSIDE the raw crate but swallowed by the inflation margin."""
    box = KeepOut(id="crateB", polygon_m=((1.0, 1.0), (1.0, 2.0),
                                          (2.0, 2.0), (2.0, 1.0)))
    with pytest.raises(PlanningError, match="crateB"):
        plan((0.0, 0.0), (0.9, 1.5), arena(box), inflation_m=0.3,
             max_leg_cm=10_000.0)


def test_start_inside_keepout_raises_actionable():
    box = KeepOut(id="crateS", polygon_m=((-1.0, -1.0), (-1.0, 1.0),
                                          (1.0, 1.0), (1.0, -1.0)))
    with pytest.raises(PlanningError) as ei:
        plan((0.0, 0.0), (5.0, 5.0), arena(box), inflation_m=0.1,
             max_leg_cm=10_000.0)
    msg = str(ei.value)
    assert "start" in msg and "crateS" in msg and "CHECK" in msg


def test_negative_inflation_raises_valueerror():
    with pytest.raises(ValueError, match="must be >= 0"):
        plan((0.0, 0.0), (1.0, 1.0), arena(), inflation_m=-0.1,
             max_leg_cm=100.0)


def test_zero_max_leg_raises_valueerror():
    with pytest.raises(ValueError, match="max_leg_cm"):
        plan((0.0, 0.0), (1.0, 1.0), arena(), inflation_m=0.0, max_leg_cm=0.0)


def test_negative_max_leg_raises_valueerror():
    with pytest.raises(ValueError, match="max_leg_cm"):
        plan((0.0, 0.0), (1.0, 1.0), arena(), inflation_m=0.0, max_leg_cm=-5.0)


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_non_finite_start_or_goal_raises(bad):
    with pytest.raises(ValueError, match="finite"):
        plan((bad, 0.0), (1.0, 1.0), arena(), inflation_m=0.0, max_leg_cm=100.0)
    with pytest.raises(ValueError, match="finite"):
        plan((0.0, 0.0), (1.0, bad), arena(), inflation_m=0.0, max_leg_cm=100.0)


def test_non_finite_inflation_raises():
    with pytest.raises(ValueError, match="finite"):
        plan((0.0, 0.0), (1.0, 1.0), arena(), inflation_m=float("inf"),
             max_leg_cm=100.0)


# ============================================================
# Edge cases
# ============================================================
def test_start_equals_goal_yields_empty_plan():
    """Degenerate start==goal is 'already there' — zero legs, not a crash."""
    legs = plan((2.0, 2.0), (2.0, 2.0), arena(), inflation_m=0.0,
                max_leg_cm=100.0)
    assert legs == []


def test_plan_is_deterministic():
    box = KeepOut(id="c", polygon_m=((1.0, -1.0), (1.0, 1.0),
                                     (3.0, 1.0), (3.0, -1.0)))
    a = plan((0.0, 0.0), (4.0, 0.0), arena(box), 0.3, 10_000.0)
    b = plan((0.0, 0.0), (4.0, 0.0), arena(box), 0.3, 10_000.0)
    assert [(l.heading_deg, l.distance_cm) for l in a] \
        == [(l.heading_deg, l.distance_cm) for l in b]
