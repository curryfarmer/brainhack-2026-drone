"""NAV-FIX — absolute position fix from known field beacons + the wiring around
it (organizer-frame binding, bounds-from-markers derivation, marker_id navigate
goal, the soft known-marker rule, the optional DR-correction hook).

Pure (stdlib + pytest; no cv2/numpy). The position-fix CLOSED-FORM math itself
is pinned by test_map_sensing (hand-computed 3-4-5 / due-north / due-east
fixtures); THIS module pins the NAV-FIX consumers + the named mutation targets:
  (a) fix sign flipped              -> test_map_sensing fixtures + the e2e here
  (b) bounds drops contains-all-markers -> test_bounds_*_must_contain_markers
  (c) frame axes swapped (n<->e)    -> test_organizer_frame_round_trip

Source: mission/planning/map_sensing.py (position_fix_from_marker,
bounds_from_markers_and_cage), mission/planning/frame.py (organizer binding),
mission/planning/types.py (Marker.from_dict known-id rule), navigate.py
(marker_id goal), flight/dead_reckon.py (apply_position_fix).
"""
from __future__ import annotations

import math

import pytest

from finals.config import DroneConfig, FinalsConfig
from finals.errors import ConfigError
from finals.flight.dead_reckon import DeadReckoner, DRPose
from finals.mission.phases.navigate import Navigate
from finals.mission.planning.frame import (ne_to_organizer_xy,
                                          organizer_xy_to_ne)
from finals.mission.planning.map_sensing import (bounds_from_markers_and_cage,
                                                position_fix_from_marker)
from finals.mission.planning.types import (KNOWN_FIELD_MARKER_IDS, ArenaMap,
                                          Marker)
from finals.types import Direction, Move, PositionQuality

# The 5 published field beacons in the organizer (x, y) frame (field_markers.md).
_FIELD_XY = {11: (1.35, 4.40), 45: (1.30, 7.85), 51: (4.40, 4.40),
             67: (1.95, 8.70), 101: (4.40, 7.85)}


# ============================================================
# L1 — organizer-frame binding (north <- y, east <- x); ROUND-TRIP EXACT.
#   Mutation target (c): swapping the axes breaks this round-trip.
# ============================================================
def test_organizer_xy_maps_long_to_north_short_to_east():
    # organizer x = SHORT axis -> east; organizer y = LONG axis -> north.
    assert organizer_xy_to_ne((1.35, 4.40)) == (4.40, 1.35)   # (north=y, east=x)
    # a point far along y (long axis) must land far along NORTH, not east.
    n, e = organizer_xy_to_ne((1.30, 7.85))
    assert n == 7.85 and e == 1.30


def test_organizer_frame_round_trip_exact():
    # xy -> ne -> xy is identity for every published beacon (and the inverse).
    for xy in _FIELD_XY.values():
        ne = organizer_xy_to_ne(xy)
        assert ne_to_organizer_xy(ne) == xy
        # and the other direction
        assert organizer_xy_to_ne(ne_to_organizer_xy(ne)) == ne


def test_organizer_round_trip_is_a_swap_not_identity():
    # Guards against a "round-trip passes because both are identity" mutant: for
    # an ASYMMETRIC point the ne form must DIFFER from the xy form (a real swap).
    xy = (1.30, 7.85)
    assert organizer_xy_to_ne(xy) != xy
    assert organizer_xy_to_ne(xy) == (7.85, 1.30)


@pytest.mark.parametrize("bad", [[1.0], [1.0, 2.0, 3.0], [1.0, "x"],
                                 [1.0, True], [float("nan"), 1.0], None])
def test_organizer_bad_input_is_loud(bad):
    with pytest.raises(ConfigError):
        organizer_xy_to_ne(bad)
    with pytest.raises(ConfigError):
        ne_to_organizer_xy(bad)


# ============================================================
# L2 — bounds derived from markers (+ cage) CONTAIN all markers.
#   Mutation target (b): a derivation that drops the contains-all check would
#   let a marker fall outside the derived bound.
# ============================================================
def _field_markers():
    return [Marker(mid, organizer_xy_to_ne(xy)) for mid, xy in _FIELD_XY.items()]


def _contains(bounds, pt):
    n_min, e_min, n_max, e_max = bounds
    return n_min <= pt[0] <= n_max and e_min <= pt[1] <= e_max


def test_bounds_markers_only_contains_every_marker():
    ms = _field_markers()
    bounds = bounds_from_markers_and_cage(ms)
    for m in ms:
        assert _contains(bounds, m.point_m), f"{m.id} outside derived bounds"
    # tight hull: equals the marker extent exactly (no spurious slack)
    assert bounds == (4.40, 1.30, 8.70, 4.40)


def test_bounds_with_cage_unions_and_contains_markers_and_cage():
    ms = _field_markers()
    cage = (0.0, 0.0, 11.3, 5.3)
    bounds = bounds_from_markers_and_cage(ms, cage)
    # the cage dominates here (markers interior) -> bounds == cage
    assert bounds == cage
    for m in ms:
        assert _contains(bounds, m.point_m)
    # and the cage corners are inside too
    assert _contains(bounds, (0.0, 0.0)) and _contains(bounds, (11.3, 5.3))


def test_bounds_margin_grows_outward():
    ms = _field_markers()
    bounds = bounds_from_markers_and_cage(ms, (0.0, 0.0, 11.3, 5.3), margin_m=0.5)
    assert bounds == (-0.5, -0.5, 11.8, 5.8)


def test_bounds_cage_smaller_than_markers_still_contains_markers():
    # A cage that does NOT cover a marker: the union MUST still contain the
    # marker (this is the contains-all-markers property; a mutant that returns
    # the cage alone would FAIL here).
    ms = [Marker(11, (4.40, 1.35)), Marker(51, (8.70, 4.40))]
    tiny_cage = (0.0, 0.0, 1.0, 1.0)
    bounds = bounds_from_markers_and_cage(ms, tiny_cage)
    for m in ms:
        assert _contains(bounds, m.point_m)
    assert bounds == (0.0, 0.0, 8.70, 4.40)


def test_bounds_accepts_raw_pairs_and_marker_objects():
    assert bounds_from_markers_and_cage([(1.0, 2.0), (3.0, 5.0)]) == \
        (1.0, 2.0, 3.0, 5.0)


def test_bounds_empty_markers_is_loud():
    with pytest.raises(ValueError, match="at least one marker"):
        bounds_from_markers_and_cage([])


def test_bounds_single_point_no_margin_is_loud():
    with pytest.raises(ValueError, match="degenerate|zero-area"):
        bounds_from_markers_and_cage([(2.0, 2.0)])


def test_bounds_single_point_with_margin_ok():
    assert bounds_from_markers_and_cage([(2.0, 2.0)], margin_m=0.5) == \
        (1.5, 1.5, 2.5, 2.5)


@pytest.mark.parametrize("bad_margin", [-1.0, float("nan"), float("inf"), True])
def test_bounds_bad_margin_is_loud(bad_margin):
    with pytest.raises(ValueError, match="margin_m"):
        bounds_from_markers_and_cage([(0.0, 0.0), (1.0, 1.0)],
                                     margin_m=bad_margin)


def test_bounds_inverted_cage_is_loud():
    with pytest.raises(ValueError, match="inverted|cage_bounds_m"):
        bounds_from_markers_and_cage([(0.0, 0.0), (1.0, 1.0)],
                                     (5.0, 5.0, 0.0, 0.0))


def test_derived_bounds_make_from_dict_markers_in_bounds_a_tautology():
    # The whole point of L2: feed the DERIVED bound into ArenaMap.from_dict and
    # the NAV-2 markers-within-bounds check passes by construction.
    ms = _field_markers()
    bounds = bounds_from_markers_and_cage(ms, (0.0, 0.0, 11.3, 5.3))
    raw = {
        "bounds_m": list(bounds),
        "c2_origin_m": [0.5, 2.65],
        "c2_heading_deg": 0.0,
        "markers": [{"id": m.id, "point_m": list(m.point_m)} for m in ms],
    }
    a = ArenaMap.from_dict(raw, name="derived")
    assert sorted(m.id for m in a.markers) == [11, 45, 51, 67, 101]


# ============================================================
# Marker.from_dict — the SOFT, OPT-IN known-id rule.
# ============================================================
def _arena_raw(markers, **extra):
    raw = {"bounds_m": [0.0, 0.0, 11.3, 5.3], "c2_origin_m": [0.5, 2.65],
           "c2_heading_deg": 0.0, "markers": markers}
    raw.update(extra)
    return raw


def test_known_marker_set_is_the_published_field_ids():
    assert KNOWN_FIELD_MARKER_IDS == frozenset({11, 45, 51, 67, 101})


def test_unknown_marker_id_rejected_when_strict_arg_passed():
    raw = _arena_raw([{"id": 999, "point_m": [4.4, 1.35]}])
    with pytest.raises(ConfigError, match="not one of the known field-beacon"):
        ArenaMap.from_dict(raw, name="t",
                           known_marker_ids=KNOWN_FIELD_MARKER_IDS)


def test_unknown_marker_id_allowed_by_default_no_restriction():
    # DEFAULT (no arg, no flag): a placeholder id like 7 is fine (sim/fixtures).
    raw = _arena_raw([{"id": 7, "point_m": [4.4, 1.35]}])
    a = ArenaMap.from_dict(raw, name="t")
    assert a.markers[0].id == 7


def test_strict_marker_ids_json_flag_opts_in():
    raw = _arena_raw([{"id": 999, "point_m": [4.4, 1.35]}],
                     strict_marker_ids=True)
    with pytest.raises(ConfigError, match="not one of the known field-beacon"):
        ArenaMap.from_dict(raw, name="t")


def test_strict_marker_ids_flag_false_no_restriction():
    raw = _arena_raw([{"id": 7, "point_m": [4.4, 1.35]}],
                     strict_marker_ids=False)
    a = ArenaMap.from_dict(raw, name="t")
    assert a.markers[0].id == 7


def test_strict_marker_ids_flag_non_bool_is_loud():
    raw = _arena_raw([{"id": 11, "point_m": [4.4, 1.35]}],
                     strict_marker_ids="yes")
    with pytest.raises(ConfigError, match="strict_marker_ids must be a boolean"):
        ArenaMap.from_dict(raw, name="t")


def test_strict_flag_accepts_all_real_beacons():
    raw = _arena_raw(
        [{"id": mid, "point_m": list(organizer_xy_to_ne(xy))}
         for mid, xy in _FIELD_XY.items()],
        strict_marker_ids=True)
    a = ArenaMap.from_dict(raw, name="field")
    assert {m.id for m in a.markers} == set(_FIELD_XY)


def test_shipped_field_arena_loads_with_all_5_beacons_in_organizer_frame():
    """The shipped finals/configs/arenas/field.json (the real-arena artifact the
    operator overwrites at gate D) parses cleanly, opts into the strict rule,
    and carries the 5 published beacons at the ORGANIZER-FRAME coords
    (point_m = [north=y, east=x]). Locks the artifact so a future edit can't
    silently break the binding or the id set."""
    import json
    import os
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "configs", "arenas", "field.json")
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    a = ArenaMap.from_dict(raw, name="field")
    assert {m.id for m in a.markers} == {11, 45, 51, 67, 101}
    by_id = {m.id: m.point_m for m in a.markers}
    # organizer (x, y) -> point_m (north=y, east=x): beacon 11 = x1.35,y4.40.
    assert by_id[11] == organizer_xy_to_ne((1.35, 4.40)) == (4.40, 1.35)
    assert by_id[51] == organizer_xy_to_ne((4.40, 4.40)) == (4.40, 4.40)
    # every beacon inside the derived/declared bounds (NAV-2 already enforces it).
    for pt in by_id.values():
        assert _contains(a.bounds_m, pt)


# ============================================================
# navigate — marker_id goal source (consume ArenaMap.markers).
# ============================================================
def _cfg_with(arena):
    return FinalsConfig(profile="mock", flight_backend="mock",
                        frame_backend="none", detector=None, drones=[],
                        arena_name="field", arena=arena)


def _arena_obj(markers=(), pads=(), c2=(0.0, 0.0)):
    return ArenaMap(bounds_m=(-100.0, -100.0, 100.0, 100.0), keep_out=(),
                    pads=tuple(pads), lanes=(), c2_origin_m=c2,
                    c2_heading_deg=0.0, markers=tuple(markers))


def test_navigate_marker_id_targets_known_beacon_coord():
    arena = _arena_obj(markers=[Marker(51, (4.40, 4.40))])
    phase = Navigate.from_config(
        DroneConfig(id="alpha", phases=["navigate"],
                    zone={"navigate": {"marker_id": 51}}),
        _cfg_with(arena))
    assert phase.goal_m == (4.40, 4.40)
    assert "beacon 51" in phase.goal_desc


def test_navigate_marker_id_not_in_arena_is_loud():
    arena = _arena_obj(markers=[Marker(51, (4.40, 4.40))])
    with pytest.raises(ConfigError, match="not a beacon in this arena"):
        Navigate.from_config(
            DroneConfig(id="alpha", phases=["navigate"],
                        zone={"navigate": {"marker_id": 11}}),
            _cfg_with(arena))


def test_navigate_marker_id_must_be_int():
    arena = _arena_obj(markers=[Marker(51, (4.40, 4.40))])
    with pytest.raises(ConfigError, match="marker_id must be an int"):
        Navigate.from_config(
            DroneConfig(id="alpha", phases=["navigate"],
                        zone={"navigate": {"marker_id": "51"}}),
            _cfg_with(arena))


def test_navigate_marker_id_and_pad_id_both_is_loud():
    arena = _arena_obj(markers=[Marker(51, (4.40, 4.40))],
                       pads=[])
    with pytest.raises(ConfigError, match="MULTIPLE goal sources"):
        Navigate.from_config(
            DroneConfig(id="alpha", phases=["navigate"],
                        zone={"navigate": {"marker_id": 51,
                                           "goal_ne_m": [1.0, 1.0]}}),
            _cfg_with(arena))


# ============================================================
# DeadReckoner.apply_position_fix — the OPTIONAL drift-reset hook.
# ============================================================
def test_apply_position_fix_resets_horizontal_keeps_alt_yaw():
    dr = DeadReckoner(DRPose(north_m=5.0, east_m=3.0, alt_m=1.5, yaw_deg=30.0))
    dr.apply_position_fix(north_m=0.0, east_m=0.0)
    p = dr.pose
    assert p.north_m == 0.0 and p.east_m == 0.0      # horizontal reset
    assert p.alt_m == 1.5 and p.yaw_deg == 30.0      # alt + yaw untouched


def test_apply_position_fix_corrects_drift_from_a_known_beacon():
    # The drift-correction story end-to-end: a drone flew FORWARD 10 m at yaw 0
    # so its DR pose reads (10, 0) — but it actually drifted and the TRUE pose is
    # (9, 0). It sees beacon at known coord (12, 0) along bearing 0, range 3 (so
    # true drone = 12 - 3 = 9). position_fix_from_marker + apply_position_fix
    # snap the estimate to the truth.
    dr = DeadReckoner(DRPose(10.0, 0.0, 1.5, 0.0))
    fix = position_fix_from_marker((12.0, 0.0), 0.0, 3.0)
    dr.apply_position_fix(*fix)
    assert dr.pose.north_m == pytest.approx(9.0, abs=1e-9)
    assert dr.pose.east_m == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_apply_position_fix_rejects_non_finite(bad):
    dr = DeadReckoner(DRPose(0.0, 0.0, 1.0, 0.0))
    with pytest.raises(ValueError):
        dr.apply_position_fix(bad, 0.0)
    with pytest.raises(ValueError):
        dr.apply_position_fix(0.0, bad)


def test_apply_position_fix_keeps_quality_dead_reckoning():
    # A fix corrects drift but does NOT promote the quality (no live position
    # feed) — the position-blind contract holds.
    dr = DeadReckoner(DRPose(5.0, 5.0, 1.0, 0.0))
    dr.apply_position_fix(0.0, 0.0)
    assert dr.QUALITY == PositionQuality.DEAD_RECKONING


# ============================================================
# E2E — a mid-transit beacon fix SHRINKS the final landing error.
#   Mirrors test_nav_e2e's "navigate closed the transit" assertion, but with a
#   DRIFTING true pose: the open-loop estimate diverges, and a known-beacon fix
#   applied mid-transit snaps the estimate back so the final |estimate - truth|
#   (the handoff error to the visual servo / pad-detector) is SMALLER than with
#   no fix. The fix math + DR are the REAL ones (no reimplementation).
# ============================================================
def _fly_transit_with_drift(*, apply_fix_at_leg=None):
    """Fly a straight 8 m north transit (4 legs of 2 m) over a REAL DeadReckoner
    that is the drone's ESTIMATE, while a parallel REAL DeadReckoner with a tiny
    constant yaw bias is the TRUE pose (open-loop heading creep). Optionally, at
    `apply_fix_at_leg`, a beacon at a known coord is decoded and the estimate is
    corrected with position_fix_from_marker. Returns (estimate_pose, true_pose).

    The beacon sits at the TRUE drone position's known anchor: we model a
    perfect centred decode (range 0, bearing 0) so the fix recovers the true
    coord exactly — the degenerate, cleanest case from field_markers.md L3
    ('centre the marker under the drone -> world_pos = marker_xy')."""
    est = DeadReckoner(DRPose(0.0, 0.0, 1.0, 0.0))       # the open-loop estimate
    true = DeadReckoner(DRPose(0.0, 0.0, 1.0, 0.0))       # the real airframe
    yaw_bias_deg = 2.0                                    # constant heading creep
    leg_m = 2.0
    for leg in range(4):
        # Estimate flies a clean FORWARD 2 m at yaw 0; the TRUE airframe flies
        # the same command but its heading is biased, so it drifts east-of-north.
        est.note_action_complete(Move(direction=Direction.FORWARD,
                                      distance_cm=int(leg_m * 100)))
        # true pose: same FORWARD command, but executed under a yaw bias.
        true_biased = DeadReckoner(DRPose(true.pose.north_m, true.pose.east_m,
                                          true.pose.alt_m, yaw_bias_deg))
        true_biased.note_action_complete(
            Move(direction=Direction.FORWARD, distance_cm=int(leg_m * 100)))
        true = true_biased
        if apply_fix_at_leg is not None and leg == apply_fix_at_leg:
            # A beacon decoded dead-centre under the TRUE drone position -> the
            # fix recovers the true coord (range 0, bearing 0 => drone = marker).
            marker_at_true = (true.pose.north_m, true.pose.east_m)
            fix = position_fix_from_marker(marker_at_true, 0.0, 0.0)
            est.apply_position_fix(*fix)
    return est.pose, true.pose


def _err(a, b):
    return math.hypot(a.north_m - b.north_m, a.east_m - b.east_m)


def test_midtransit_beacon_fix_shrinks_final_estimate_error():
    est_nofix, true_nofix = _fly_transit_with_drift(apply_fix_at_leg=None)
    est_fix, true_fix = _fly_transit_with_drift(apply_fix_at_leg=2)
    # Same flight, same true pose either way.
    assert _err(true_nofix, true_fix) == pytest.approx(0.0, abs=1e-12)
    err_nofix = _err(est_nofix, true_nofix)
    err_fix = _err(est_fix, true_fix)
    # Without a fix the estimate has accumulated the heading-creep drift.
    assert err_nofix > 0.05, "expected meaningful open-loop drift to correct"
    # The mid-transit fix STRICTLY shrinks the residual estimate error (the
    # remaining error is only the post-fix legs' drift).
    assert err_fix < err_nofix
    # And the fix really helped: after the fix at leg 2 only ~1 leg of drift
    # remains, so the residual is a fraction of the un-fixed error.
    assert err_fix < 0.5 * err_nofix


def test_centred_beacon_fix_recovers_exact_truth():
    # The degenerate L3 case: a beacon centred under the drone (range 0) makes
    # the fix EXACT — the estimate equals the true pose right after applying it.
    est, true = _fly_transit_with_drift(apply_fix_at_leg=3)   # fix on last leg
    # last-leg fix: the estimate was snapped to the true pose AFTER the final
    # move, so they coincide exactly.
    assert _err(est, true) == pytest.approx(0.0, abs=1e-9)
