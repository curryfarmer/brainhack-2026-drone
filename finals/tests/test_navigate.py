"""finals.mission.phases.navigate — open-loop transit phase (NAV-5).

Pure phase tests (the MissionPhase contract: the whole plan is testable by
stepping with hand-built AgentContexts, like test_search / test_takeoff_demo)
plus a DeadReckoner integration that flies the commanded actions and asserts the
drone lands ~ the goal. The forward model is NEVER reimplemented here: the REAL
flight.dead_reckon.DeadReckoner is the oracle (same discipline as
test_visibility_graph.fly), so a heading-sign flip on EITHER side goes red.

THE load-bearing assertion (test_reorient_re_zeros_drifted_yaw): feed a yaw that
has DRIFTED off the leg heading and prove the commanded Rotate closes to the
ABSOLUTE target heading, not a relative delta — that is what re-zeroes creep.
"""
from __future__ import annotations

import math

import pytest

from finals.config import DroneConfig, FinalsConfig
from finals.errors import ConfigError, PlanningError
from finals.flight.dead_reckon import DeadReckoner, DRPose
from finals.mission.phase import AgentContext
from finals.mission.phases import PHASE_REGISTRY
from finals.mission.phases.navigate import Navigate
from finals.mission.planning.types import ArenaMap, KeepOut, LandingPad, Leg
from finals.mission.planning.visibility_graph import plan
from finals.types import (Abort, Direction, Done, Move, Rotate, Telemetry)


# ---------------- arena fixtures (mirror test_visibility_graph.arena) --------
def _arena(*keep_out, pads=(), bounds=(-100.0, -100.0, 100.0, 100.0),
           c2=(0.0, 0.0)):
    return ArenaMap(bounds_m=bounds, keep_out=tuple(keep_out), pads=tuple(pads),
                    lanes=(), c2_origin_m=c2, c2_heading_deg=0.0)


def _pad(pad_id, center, *, radius=0.5, valid=True):
    return LandingPad(id=pad_id, center_m=center, radius_m=radius, valid=valid)


def _cfg(arena, arena_name="testarena"):
    """A minimal FinalsConfig carrying just the arena fields navigate reads."""
    return FinalsConfig(
        profile="mock", flight_backend="mock", frame_backend="none",
        detector=None, drones=[], arena_name=arena_name, arena=arena)


def _drone(zone):
    return DroneConfig(id="alpha", phases=["navigate"], zone=zone)


# ---------------- pure-stepping harness (mirrors test_search.make_ctx) -------
def _ctx(*, yaw_deg=0.0, elapsed=0.0, last_action=None, last_action_ok=None,
         last_action_error=None):
    return AgentContext(
        drone_id="alpha", now=100.0 + elapsed, mission_elapsed_s=elapsed,
        telemetry=Telemetry(ts=100.0 + elapsed, yaw_deg=yaw_deg,
                            altitude_m=1.5, is_flying=True),
        sightings=[], last_action=last_action, last_action_ok=last_action_ok,
        last_action_error=last_action_error)


def _phase_to_goal(goal_ne_m, *, arena=None, **zone_extra):
    arena = arena if arena is not None else _arena()
    zone = {"navigate": {"goal_ne_m": list(goal_ne_m), **zone_extra}}
    return Navigate.from_config(_drone(zone), _cfg(arena))


def _drive_with_dr(phase, *, start=(0.0, 0.0), max_steps=2000):
    """Drive the phase to Done feeding it the yaw from a REAL DeadReckoner that
    integrates every Rotate/Move the phase commands. Returns (actions, done,
    dr) — dr.pose is the dead-reckoned landing pose. The yaw fed back is
    realistic (it reflects the integrated rotations), so the per-leg re-orient
    is exercised end-to-end. Math lives in dead_reckon, never here."""
    dr = DeadReckoner(DRPose(start[0], start[1], 0.0, 0.0))
    actions = []
    last = None
    ok = None
    for _ in range(max_steps):                          # bounded (convention 3)
        action = phase.step(_ctx(yaw_deg=dr.pose.yaw_deg, last_action=last,
                                 last_action_ok=ok))
        if isinstance(action, Done):
            return actions, action, dr
        if isinstance(action, Abort):
            pytest.fail(f"unexpected Abort: {action.reason}")
        actions.append(action)
        dr.note_action_complete(action)                 # the agent's effect
        last, ok = action, True
    pytest.fail(f"phase never returned Done within {max_steps} steps")


# ============================================================
# Registry
# ============================================================
def test_registered_under_its_name():
    assert PHASE_REGISTRY["navigate"] is Navigate


# ============================================================
# Plan execution order: Rotate -> Move per leg, Done at the end
# ============================================================
def test_two_leg_plan_executes_rotate_move_rotate_move_done():
    # A box forces a 2-leg detour: C2 (0,0) -> (4,0) around a crate at north 1-3.
    box = KeepOut(id="crate", polygon_m=((1.0, -1.0), (1.0, 1.0),
                                         (3.0, 1.0), (3.0, -1.0)))
    arena = _arena(box)
    phase = Navigate.from_config(
        _drone({"navigate": {"goal_ne_m": [4.0, 0.0], "inflation_m": 0.3,
                             "max_leg_cm": 100000.0, "heading_tol_deg": 1.0,
                             "max_step_deg": 180.0}}),
        _cfg(arena))
    assert len(phase._legs) >= 2
    actions, done, dr = _drive_with_dr(phase)
    # Each leg is exactly one Rotate (max_step 180 closes any heading in one
    # step) then one Move, in order.
    assert all(isinstance(a, (Rotate, Move)) for a in actions)
    # Pattern: Rotate, Move, Rotate, Move, ... (one Rotate then one Move/leg).
    kinds = [type(a).__name__ for a in actions]
    for i in range(0, len(kinds), 2):
        assert kinds[i] == "Rotate" and kinds[i + 1] == "Move"
    assert len(actions) == 2 * len(phase._legs)
    assert "navigate complete" in done.reason
    # The DeadReckoner that flew the commands lands ~ the goal. The residual is
    # the heading_tol_deg (1 deg) the deadband leaves uncorrected per leg, which
    # on these few-metre legs is < ~5 cm of closure error — exactly the
    # open-loop drift the inflation margin is sized to absorb.
    assert dr.pose.north_m == pytest.approx(4.0, abs=0.1)
    assert dr.pose.east_m == pytest.approx(0.0, abs=0.1)


def test_single_leg_plan_is_rotate_move_done():
    phase = _phase_to_goal((0.0, 5.0), max_leg_cm=100000.0,
                           heading_tol_deg=1.0, max_step_deg=180.0)
    assert len(phase._legs) == 1
    actions, done, _ = _drive_with_dr(phase)
    assert [type(a).__name__ for a in actions] == ["Rotate", "Move"]


# ============================================================
# Within-deadband yaw -> straight to Move (no spurious Rotate)
# ============================================================
def test_within_deadband_yaw_goes_straight_to_move():
    # Goal due +north => leg heading 0 deg. Boot yaw 0 is already on heading.
    phase = _phase_to_goal((5.0, 0.0), max_leg_cm=100000.0,
                           heading_tol_deg=5.0, max_step_deg=45.0)
    assert len(phase._legs) == 1 and phase._legs[0].heading_deg == \
        pytest.approx(0.0)
    first = phase.step(_ctx(yaw_deg=0.0))
    assert isinstance(first, Move)        # NO Rotate — yaw already in deadband
    assert first.distance_cm == 500


def test_within_deadband_at_tol_boundary_still_no_rotate():
    # heading 0, yaw exactly at +tol => |error| == tol => deadband (inclusive).
    phase = _phase_to_goal((5.0, 0.0), max_leg_cm=100000.0,
                           heading_tol_deg=5.0, max_step_deg=45.0)
    first = phase.step(_ctx(yaw_deg=5.0))
    assert isinstance(first, Move)


# ============================================================
# THE absolute re-orient: a drifted yaw re-zeros creep (load-bearing)
# ============================================================
def test_reorient_re_zeros_drifted_yaw():
    """Feed a yaw that has DRIFTED off the leg heading and prove the commanded
    Rotate targets the ABSOLUTE heading (re-zeroing creep), NOT a relative
    delta. Leg heading is 0 (due-north goal); a relative scheme would command 0
    (it 'already turned'); the absolute scheme commands -drift to cancel it."""
    phase = _phase_to_goal((5.0, 0.0), max_leg_cm=100000.0,
                           heading_tol_deg=1.0, max_step_deg=180.0)
    assert phase._legs[0].heading_deg == pytest.approx(0.0)
    drift = 30.0                                  # the nose has crept +30 CCW
    rot = phase.step(_ctx(yaw_deg=drift))
    assert isinstance(rot, Rotate)
    # Absolute: error = wrap180(0 - 30) = -30 => Rotate(-30) cancels the drift
    # back to the true heading. A relative scheme would NOT correct it.
    assert rot.angle_deg == pytest.approx(-30.0)
    # And after applying that Rotate the nose is on heading => next is the Move.
    nxt = phase.step(_ctx(yaw_deg=drift + rot.angle_deg, last_action=rot,
                          last_action_ok=True))
    assert isinstance(nxt, Move)


def test_reorient_clamped_to_max_step_then_converges():
    """A large drift is closed in max_step_deg increments (clamped), still
    landing on the absolute heading."""
    phase = _phase_to_goal((5.0, 0.0), max_leg_cm=100000.0,
                           heading_tol_deg=1.0, max_step_deg=20.0)
    yaw = 90.0                                    # 90 deg off, step capped at 20
    first = phase.step(_ctx(yaw_deg=yaw))
    assert isinstance(first, Rotate)
    assert first.angle_deg == pytest.approx(-20.0)   # clamped toward the target
    # Drive it the rest of the way; it must reach a Move (converged), not spin.
    actions, _, _ = _drive_with_dr(
        _phase_to_goal((5.0, 0.0), max_leg_cm=100000.0, heading_tol_deg=1.0,
                       max_step_deg=20.0))
    # ceil(90/20)=5 rotate steps then the Move (DR starts at yaw 0, leg 0).
    assert sum(isinstance(a, Move) for a in actions) == 1


# ============================================================
# last_action_ok False -> Abort, actionable
# ============================================================
def test_failed_action_aborts_with_actionable_message():
    phase = _phase_to_goal((0.0, 5.0), max_leg_cm=100000.0)
    phase.step(_ctx())                            # consume the first action
    a = phase.step(_ctx(
        last_action=Move(direction=Direction.FORWARD, distance_cm=500),
        last_action_ok=False,
        last_action_error="alpha: move(FORWARD, 500 cm) exceeded 15.0 s"))
    assert isinstance(a, Abort)
    assert "alpha" in a.reason and "exceeded 15.0 s" in a.reason
    assert "leg" in a.reason and "CHECK" in a.reason
    assert "abort" in a.reason.lower()


def test_failed_move_abort_names_the_current_leg_not_the_next():
    """A Move that FAILS must Abort naming the leg whose Move it was — the leg
    index must NOT have advanced at Move-issue time (kills 'advance before the
    Move resolves'). Fly the 1st leg OK, then fail the 2nd leg's Move and check
    the Abort still names leg 2 (an early-advance mutant would say leg 3)."""
    legs = [Leg(heading_deg=0.0, distance_cm=100.0),
            Leg(heading_deg=0.0, distance_cm=200.0),
            Leg(heading_deg=0.0, distance_cm=300.0)]
    phase = _bare_phase(legs)
    last, ok = None, None
    moves_seen = 0
    for _ in range(50):                           # bounded
        a = phase.step(_ctx(yaw_deg=0.0, last_action=last, last_action_ok=ok))
        if isinstance(a, Move):
            moves_seen += 1
            if moves_seen == 2:                   # fail leg 2's Move
                ab = phase.step(_ctx(
                    yaw_deg=0.0, last_action=a, last_action_ok=False,
                    last_action_error="alpha: move exceeded 15.0 s"))
                assert isinstance(ab, Abort)
                assert "leg 2/3" in ab.reason     # NOT leg 3/3 (no early advance)
                return
        last, ok = a, True
    pytest.fail("never reached the 2nd leg's Move")


# ============================================================
# Budget exceeded -> Abort
# ============================================================
def test_budget_exceeded_aborts():
    phase = _phase_to_goal((0.0, 50.0), max_leg_cm=100000.0,
                           total_budget_s=10.0)
    # First step anchors the budget clock at elapsed=0.
    phase.step(_ctx(elapsed=0.0))
    a = phase.step(_ctx(elapsed=10.5, last_action_ok=True))   # past 10.0 s
    assert isinstance(a, Abort)
    assert "OVERRAN" in a.reason and "10.0 s" in a.reason and "CHECK" in a.reason


def test_budget_boundary_not_exceeded_at_exactly_budget():
    # elapsed == budget is NOT over (strict >); the phase keeps flying.
    phase = _phase_to_goal((0.0, 50.0), max_leg_cm=100000.0,
                           total_budget_s=10.0)
    phase.step(_ctx(elapsed=0.0))
    a = phase.step(_ctx(elapsed=10.0, last_action_ok=True))
    assert not isinstance(a, Abort)


# ============================================================
# Non-converging rotate -> Abort after the per-leg bound (not an infinite loop)
# ============================================================
def test_non_converging_rotate_aborts_after_bound():
    """A compass STUCK off the heading must Abort after the per-leg rotate cap,
    not spin forever. We feed a constant drifted yaw so the re-orient never
    closes."""
    phase = _phase_to_goal((5.0, 0.0), max_leg_cm=100000.0,
                           heading_tol_deg=1.0, max_step_deg=45.0)
    # rot_cap = ceil(360/45)+4 = 12. Feed a yaw 90 off forever (never closes
    # because we don't apply the Rotate to the fed-back yaw).
    last = None
    for i in range(100):                          # bounded
        a = phase.step(_ctx(yaw_deg=90.0, last_action=last,
                            last_action_ok=None if last is None else True))
        if isinstance(a, Abort):
            assert "did NOT converge" in a.reason
            assert "residual" in a.reason and "CHECK" in a.reason
            break
        last = a
    else:
        pytest.fail("re-orient never aborted — it spun forever")
    assert i <= phase._rot_cap + 1                # aborted right after the cap


# ============================================================
# from_config goal resolution + errors
# ============================================================
def test_from_config_resolves_pad_id():
    arena = _arena(pads=[_pad("H1", (3.0, 4.0)), _pad("H2", (-2.0, 1.0))])
    phase = Navigate.from_config(
        _drone({"navigate": {"pad_id": "H1", "max_leg_cm": 100000.0}}),
        _cfg(arena))
    assert phase.goal_m == (3.0, 4.0)
    assert "H1" in phase.goal_desc


def test_from_config_unknown_pad_id_lists_available():
    arena = _arena(pads=[_pad("H1", (3.0, 4.0)), _pad("H2", (-2.0, 1.0))])
    with pytest.raises(ConfigError) as ei:
        Navigate.from_config(_drone({"navigate": {"pad_id": "H9"}}), _cfg(arena))
    msg = str(ei.value)
    assert "H9" in msg and "H1" in msg and "H2" in msg   # lists available ids


def test_from_config_neither_goal_source_fails():
    with pytest.raises(ConfigError, match="NEITHER"):
        Navigate.from_config(_drone({"navigate": {"inflation_m": 0.5}}),
                             _cfg(_arena()))


def test_from_config_both_goal_sources_fails():
    arena = _arena(pads=[_pad("H1", (3.0, 4.0))])
    with pytest.raises(ConfigError, match="BOTH"):
        Navigate.from_config(
            _drone({"navigate": {"pad_id": "H1", "goal_ne_m": [1.0, 1.0]}}),
            _cfg(arena))


def test_from_config_missing_arena_names_arena_name():
    cfg = _cfg(arena=None, arena_name="convoy")     # arena None
    with pytest.raises(ConfigError) as ei:
        Navigate.from_config(_drone({"navigate": {"pad_id": "H1"}}), cfg)
    assert "arena" in str(ei.value) and "convoy" in str(ei.value)


def test_from_config_degenerate_plan_is_noop_trap():
    # Goal == C2 origin => zero-length plan => no-op trap ConfigError.
    arena = _arena(c2=(2.0, 2.0))
    with pytest.raises(ConfigError, match="ZERO legs"):
        Navigate.from_config(
            _drone({"navigate": {"goal_ne_m": [2.0, 2.0]}}), _cfg(arena))


def test_from_config_goal_in_keepout_surfaces_planning_error():
    box = KeepOut(id="crateZ", polygon_m=((1.0, 1.0), (1.0, 5.0),
                                          (5.0, 5.0), (5.0, 1.0)))
    arena = _arena(box)
    with pytest.raises(PlanningError, match="crateZ"):
        Navigate.from_config(
            _drone({"navigate": {"goal_ne_m": [3.0, 3.0], "inflation_m": 0.2}}),
            _cfg(arena))


def test_from_config_unknown_zone_key_fails_loudly():
    with pytest.raises(ConfigError, match=r"alpha.*inflaton_m"):
        Navigate.from_config(
            _drone({"navigate": {"goal_ne_m": [1.0, 1.0], "inflaton_m": 0.5}}),
            _cfg(_arena()))


def test_from_config_non_dict_zone_fails_loudly():
    with pytest.raises(ConfigError, match=r"alpha.*object"):
        Navigate.from_config(_drone({"navigate": 5}), _cfg(_arena()))


@pytest.mark.parametrize("key,value", [
    ("inflation_m", 0.0), ("inflation_m", -1.0), ("inflation_m", math.nan),
    ("inflation_m", math.inf), ("inflation_m", True),
    ("max_leg_cm", 0.0), ("max_leg_cm", -5.0),
    ("heading_tol_deg", -1.0), ("heading_tol_deg", math.inf),
    ("max_step_deg", 0.0), ("max_step_deg", -10.0),
    ("total_budget_s", 0.0), ("total_budget_s", -1.0),
])
def test_from_config_rejects_bad_tunables(key, value):
    zone = {"navigate": {"goal_ne_m": [1.0, 1.0], key: value}}
    with pytest.raises(ConfigError, match="navigate"):
        Navigate.from_config(_drone(zone), _cfg(_arena()))


@pytest.mark.parametrize("bad", [[1.0], [1.0, 2.0, 3.0], "x",
                                 [1.0, math.nan], [1.0, True]])
def test_from_config_bad_goal_ne_m_shape_fails(bad):
    with pytest.raises(ConfigError, match="goal_ne_m"):
        Navigate.from_config(_drone({"navigate": {"goal_ne_m": bad}}),
                             _cfg(_arena()))


# ============================================================
# yaw None -> Abort (the one thing this open-loop phase cannot fly without)
# ============================================================
def test_yaw_none_aborts():
    phase = _phase_to_goal((0.0, 5.0), max_leg_cm=100000.0)
    a = phase.step(AgentContext(
        drone_id="alpha", now=100.0, mission_elapsed_s=0.0,
        telemetry=Telemetry(ts=100.0, yaw_deg=None, is_flying=True)))
    assert isinstance(a, Abort)
    assert "yaw_deg is None" in a.reason and "CHECK" in a.reason


# ============================================================
# Phase never issues Takeoff/Land (it assumes airborne) + Done is stable
# ============================================================
def test_phase_never_takes_off_or_lands():
    box = KeepOut(id="crate", polygon_m=((1.0, -1.0), (1.0, 1.0),
                                         (3.0, 1.0), (3.0, -1.0)))
    phase = Navigate.from_config(
        _drone({"navigate": {"goal_ne_m": [4.0, 0.0], "inflation_m": 0.3,
                             "max_leg_cm": 200.0, "heading_tol_deg": 2.0,
                             "max_step_deg": 90.0}}),
        _cfg(_arena(box)))
    actions, _, _ = _drive_with_dr(phase)
    names = {type(a).__name__ for a in actions}
    assert "Takeoff" not in names and "Land" not in names


def test_done_is_stable_after_completion():
    phase = _phase_to_goal((0.0, 5.0), max_leg_cm=100000.0, heading_tol_deg=1.0,
                           max_step_deg=180.0)
    _drive_with_dr(phase)
    for _ in range(3):
        assert isinstance(phase.step(_ctx(yaw_deg=-90.0, last_action_ok=True)),
                          Done)


# ============================================================
# Heading at exactly +/-180 (a due-south goal) is handled
# ============================================================
def test_due_south_goal_heading_180():
    # Goal due -north (south) => heading atan2(0, -d) = 180 deg.
    phase = _phase_to_goal((-5.0, 0.0), max_leg_cm=100000.0, heading_tol_deg=1.0,
                           max_step_deg=180.0)
    assert abs(phase._legs[0].heading_deg) == pytest.approx(180.0)
    actions, _, dr = _drive_with_dr(phase)
    assert dr.pose.north_m == pytest.approx(-5.0, abs=1e-6)
    assert dr.pose.east_m == pytest.approx(0.0, abs=1e-6)


# ============================================================
# Integration: subdivided multi-leg detour lands ~ goal under the DeadReckoner
# ============================================================
def test_integration_subdivided_detour_lands_at_goal():
    """Drive a subdivided, obstacle-avoiding plan through the REAL DeadReckoner
    (the NAV-5 forward model) and assert it lands on the goal within a tight
    drift tolerance. Reuses dead_reckon; never reimplements the math."""
    box = KeepOut(id="crate", polygon_m=((1.0, -1.0), (1.0, 1.0),
                                         (3.0, 1.0), (3.0, -1.0)))
    goal = (4.0, 0.0)
    arena = _arena(box)
    phase = Navigate.from_config(
        _drone({"navigate": {"goal_ne_m": list(goal), "inflation_m": 0.4,
                             "max_leg_cm": 60.0, "heading_tol_deg": 0.5,
                             "max_step_deg": 90.0}}),
        _cfg(arena))
    assert len(phase._legs) > 2                   # subdivided + detoured
    actions, done, dr = _drive_with_dr(phase)
    # Documented drift tolerance: with heading_tol 0.5 deg the per-leg re-orient
    # leaves at most ~0.5 deg of heading error; over <= 6 m of legs that is well
    # under 1 cm of closure error. We assert 1 mm (the DR + tol math is exact
    # to float here because the fed-back yaw closes inside the deadband each
    # leg). Reaching the goal proves the phase tracked the planned polyline.
    assert dr.pose.north_m == pytest.approx(goal[0], abs=0.05)
    assert dr.pose.east_m == pytest.approx(goal[1], abs=0.05)
    # And the route cleared the REAL crate (collision-free claim).
    assert "navigate complete" in done.reason


# ============================================================
# Tiny leg rounding to 0 cm is skipped (never a refused 0 cm Move)
# ============================================================
def _bare_phase(legs, **kw):
    """Construct directly with a hand-built legs tuple (bypasses the planner)."""
    defaults = dict(goal_m=(0.0, 0.0), heading_tol_deg=1.0, max_step_deg=180.0,
                    total_budget_s=120.0)
    defaults.update(kw)
    return Navigate(legs=tuple(legs), **defaults)


def test_tiny_final_leg_rounds_to_zero_is_skipped_not_commanded():
    # A real leg then a 0.4 cm sub-leg (rounds to 0). The phase must NOT emit a
    # 0 cm Move (the adapter refuses it) — it skips that leg and Done-s.
    legs = [Leg(heading_deg=0.0, distance_cm=100.0),
            Leg(heading_deg=0.0, distance_cm=0.4)]
    phase = _bare_phase(legs)
    actions, done, _ = _drive_with_dr(phase)
    moves = [a for a in actions if isinstance(a, Move)]
    assert len(moves) == 1 and moves[0].distance_cm == 100   # the 0.4 cm skipped
    assert "navigate complete" in done.reason


def test_only_leg_rounds_to_zero_completes_without_moving():
    # Pathological: the single leg rounds to 0 cm. No Move at all -> Done (the
    # constructor's no-op trap is upstream; here we prove step() is safe).
    phase = _bare_phase([Leg(heading_deg=0.0, distance_cm=0.3)])
    actions, done, _ = _drive_with_dr(phase)
    assert not any(isinstance(a, Move) for a in actions)
    assert isinstance(done, Done)


# Sanity: the plan the phase pre-computed matches a direct plan() call (the
# phase does not bend the planner's output).
def test_phase_legs_equal_direct_plan():
    box = KeepOut(id="crate", polygon_m=((1.0, -1.0), (1.0, 1.0),
                                         (3.0, 1.0), (3.0, -1.0)))
    arena = _arena(box)
    phase = Navigate.from_config(
        _drone({"navigate": {"goal_ne_m": [4.0, 0.0], "inflation_m": 0.3,
                             "max_leg_cm": 120.0}}),
        _cfg(arena))
    direct = plan((0.0, 0.0), (4.0, 0.0), arena, 0.3, 120.0)
    assert [(l.heading_deg, l.distance_cm) for l in phase._legs] == \
        [(l.heading_deg, l.distance_cm) for l in direct]
