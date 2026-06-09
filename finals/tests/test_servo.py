"""Pins the shared PURE visual-servo + heading math (_servo, NAV-3).

The sign conventions here are LOAD-BEARING: NAV-5 (navigate) and NAV-6
(land_on_pad) wire to them, and they must stay consistent with
vision/perception.py (bearing = yaw MINUS pixel offset, CCW+). Each block
below is designed so the obvious mutant — a rotate sign flip, a LEFT<->RIGHT
swap, a `<=`->`<` deadband flip, a dropped clamp — fails at least one assert.

stdlib + pytest only (the module is PURE; the suite runs in a bare venv).
"""
from __future__ import annotations

import math

import pytest

from finals.mission.phases._servo import (bearing_error_to_rotate, clamp,
                                          pixel_offset_to_move, wrap180)
from finals.types import Direction, Move, Rotate


# ============================================================
# wrap180
# ============================================================
@pytest.mark.parametrize("deg, expected", [
    (0.0, 0.0),
    (179.0, 179.0),
    (180.0, 180.0),          # closed boundary stays +180
    (181.0, -179.0),
    (-179.0, -179.0),
    (-180.0, 180.0),         # open end maps across to the closed +180
    (360.0, 0.0),
    (540.0, 180.0),          # 540 % 360 = 180 -> stays +180
    (-540.0, 180.0),         # -540 -> -180 -> +180
    (90.0, 90.0),
    (-90.0, -90.0),
    (720.0, 0.0),
    (-1.0, -1.0),
    (359.0, -1.0),
])
def test_wrap180_values(deg, expected):
    assert wrap180(deg) == pytest.approx(expected)


def test_wrap180_always_in_half_open_interval():
    for deg in range(-1000, 1001):
        w = wrap180(float(deg))
        assert -180.0 < w <= 180.0


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_wrap180_rejects_nan_inf(bad):
    with pytest.raises(ValueError):
        wrap180(bad)


def test_wrap180_rejects_bool():
    # True is an int subclass = 1; a bool angle is always a wiring bug.
    with pytest.raises(ValueError):
        wrap180(True)


# ============================================================
# clamp
# ============================================================
def test_clamp_inside():
    assert clamp(5.0, 0.0, 10.0) == 5.0


def test_clamp_below():
    assert clamp(-3.0, 0.0, 10.0) == 0.0


def test_clamp_above():
    assert clamp(99.0, 0.0, 10.0) == 10.0


def test_clamp_at_exact_bounds():
    assert clamp(0.0, 0.0, 10.0) == 0.0
    assert clamp(10.0, 0.0, 10.0) == 10.0


def test_clamp_lo_equals_hi():
    assert clamp(5.0, 3.0, 3.0) == 3.0


def test_clamp_lo_greater_than_hi_fails_loud():
    with pytest.raises(ValueError, match="lo .* must be <= hi"):
        clamp(5.0, 10.0, 0.0)


@pytest.mark.parametrize("args", [
    (float("nan"), 0.0, 1.0),
    (0.5, float("inf"), 1.0),
    (0.5, 0.0, float("nan")),
])
def test_clamp_rejects_nonfinite(args):
    with pytest.raises(ValueError):
        clamp(*args)


# ============================================================
# bearing_error_to_rotate
# ============================================================
def test_bearing_rotate_on_target_returns_none():
    assert bearing_error_to_rotate(90.0, 90.0, tol_deg=2.0,
                                   max_step_deg=30.0) is None


def test_bearing_rotate_inside_deadband_returns_none():
    # error = +1.5, tol = 2 -> inside.
    assert bearing_error_to_rotate(91.5, 90.0, tol_deg=2.0,
                                   max_step_deg=30.0) is None


def test_bearing_rotate_exactly_at_deadband_is_none():
    # |error| == tol_deg must be INSIDE (boundary inclusive); a `<`->`<=`
    # mutant on the deadband flips this.
    assert bearing_error_to_rotate(92.0, 90.0, tol_deg=2.0,
                                   max_step_deg=30.0) is None
    assert bearing_error_to_rotate(88.0, 90.0, tol_deg=2.0,
                                   max_step_deg=30.0) is None


def test_bearing_rotate_just_outside_tol_turns_ccw_for_positive_error():
    # target CCW of nose (target > yaw) -> error +3 -> POSITIVE (CCW) Rotate.
    # A rotate-sign flip mutant emits -3 and fails this.
    r = bearing_error_to_rotate(93.0, 90.0, tol_deg=2.0, max_step_deg=30.0)
    assert r == Rotate(angle_deg=3.0)
    assert isinstance(r, Rotate) and r.angle_deg > 0


def test_bearing_rotate_just_outside_tol_turns_cw_for_negative_error():
    # target CW of nose (target < yaw) -> error -3 -> NEGATIVE (CW) Rotate.
    r = bearing_error_to_rotate(87.0, 90.0, tol_deg=2.0, max_step_deg=30.0)
    assert r == Rotate(angle_deg=-3.0)
    assert r.angle_deg < 0


def test_bearing_rotate_large_error_clamped_to_max_step_positive():
    # error +120 -> clamped to +max_step (30). Dropping the clamp -> 120.
    r = bearing_error_to_rotate(120.0, 0.0, tol_deg=2.0, max_step_deg=30.0)
    assert r == Rotate(angle_deg=30.0)


def test_bearing_rotate_large_error_clamped_to_max_step_negative():
    r = bearing_error_to_rotate(-120.0, 0.0, tol_deg=2.0, max_step_deg=30.0)
    assert r == Rotate(angle_deg=-30.0)


def test_bearing_rotate_wraparound_takes_shortest_turn():
    # target 170, yaw -170: raw diff = 340, wrap180 -> -20 -> shortest turn is
    # CW (negative) 20 deg, NOT a +340 spin. Exact value pinned.
    r = bearing_error_to_rotate(170.0, -170.0, tol_deg=2.0, max_step_deg=30.0)
    assert r == Rotate(angle_deg=-20.0)


def test_bearing_rotate_wraparound_other_direction():
    # target -170, yaw 170: raw diff = -340 -> wrap180 -> +20 -> CCW 20.
    r = bearing_error_to_rotate(-170.0, 170.0, tol_deg=2.0, max_step_deg=30.0)
    assert r == Rotate(angle_deg=20.0)


def test_bearing_rotate_matches_perception_bearing_sign():
    """Consistency with vision/perception.py: a bearing produced for a target
    LEFT of frame centre is POSITIVE (CCW of nose); steering to it from yaw 0
    must rotate CCW (positive). A sign-swap here would silently turn the drone
    AWAY from every detected target."""
    bearing_left_of_centre = 15.0   # perception emits +15 for a left target
    r = bearing_error_to_rotate(bearing_left_of_centre, 0.0, tol_deg=2.0,
                                max_step_deg=30.0)
    assert r.angle_deg > 0          # CCW, toward the target


@pytest.mark.parametrize("tol, step", [
    (-1.0, 30.0),       # negative deadband
    (2.0, 0.0),         # zero step
    (2.0, -5.0),        # negative step
])
def test_bearing_rotate_rejects_bad_params(tol, step):
    with pytest.raises(ValueError):
        bearing_error_to_rotate(45.0, 0.0, tol_deg=tol, max_step_deg=step)


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_bearing_rotate_rejects_nonfinite(bad):
    with pytest.raises(ValueError):
        bearing_error_to_rotate(bad, 0.0, tol_deg=2.0, max_step_deg=30.0)
    with pytest.raises(ValueError):
        bearing_error_to_rotate(45.0, bad, tol_deg=2.0, max_step_deg=30.0)


# ============================================================
# pixel_offset_to_move
# ============================================================
# Shared geometry: 640-wide frame, centre at px 320.
def _bbox_centred_at(cx, half=20.0):
    return (cx - half, 0.0, cx + half, 40.0)


def test_pixel_move_centered_returns_none():
    assert pixel_offset_to_move(_bbox_centred_at(320.0), frame_w=640,
                                altitude_m=2.0, k=1.0, min_cm=1.0,
                                max_cm=100.0, tol_px=5.0) is None


def test_pixel_move_inside_tol_returns_none():
    # cx 323 -> px +3, tol 5 -> inside.
    assert pixel_offset_to_move(_bbox_centred_at(323.0), frame_w=640,
                                altitude_m=2.0, k=1.0, min_cm=1.0,
                                max_cm=100.0, tol_px=5.0) is None


def test_pixel_move_exactly_at_tol_is_none():
    # |px| == tol_px must be INSIDE (boundary inclusive); `<`->`<=` mutant flips.
    assert pixel_offset_to_move(_bbox_centred_at(325.0), frame_w=640,
                                altitude_m=2.0, k=1.0, min_cm=1.0,
                                max_cm=100.0, tol_px=5.0) is None
    assert pixel_offset_to_move(_bbox_centred_at(315.0), frame_w=640,
                                altitude_m=2.0, k=1.0, min_cm=1.0,
                                max_cm=100.0, tol_px=5.0) is None


def test_pixel_move_target_right_of_centre_moves_right():
    # cx 480 -> px +160 (RIGHT of centre) -> Direction.RIGHT (chase the blob).
    # A LEFT<->RIGHT swap mutant fails here.
    m = pixel_offset_to_move(_bbox_centred_at(480.0), frame_w=640,
                             altitude_m=2.0, k=1.0, min_cm=1.0, max_cm=1000.0,
                             tol_px=5.0)
    assert isinstance(m, Move)
    assert m.direction == Direction.RIGHT


def test_pixel_move_target_left_of_centre_moves_left():
    m = pixel_offset_to_move(_bbox_centred_at(160.0), frame_w=640,
                             altitude_m=2.0, k=1.0, min_cm=1.0, max_cm=1000.0,
                             tol_px=5.0)
    assert m.direction == Direction.LEFT


def test_pixel_move_step_magnitude_is_positive_both_sides():
    # The clamp takes |...|; LEFT and RIGHT at mirror offsets give the same cm.
    right = pixel_offset_to_move(_bbox_centred_at(480.0), frame_w=640,
                                 altitude_m=2.0, k=1.0, min_cm=1.0,
                                 max_cm=1000.0, tol_px=5.0)
    left = pixel_offset_to_move(_bbox_centred_at(160.0), frame_w=640,
                                altitude_m=2.0, k=1.0, min_cm=1.0,
                                max_cm=1000.0, tol_px=5.0)
    assert right.distance_cm == left.distance_cm > 0


def test_pixel_move_altitude_doubling_doubles_step():
    # Same pixel error, 2x altitude -> ~2x step_cm (until clamp). A dropped
    # altitude factor makes both equal and fails this.
    # cx 480 -> px +160, error_norm = 160/640 = 0.25; k=1 -> step = 0.25*alt*100.
    low = pixel_offset_to_move(_bbox_centred_at(480.0), frame_w=640,
                               altitude_m=1.0, k=1.0, min_cm=0.0,
                               max_cm=1000.0, tol_px=5.0)
    high = pixel_offset_to_move(_bbox_centred_at(480.0), frame_w=640,
                                altitude_m=2.0, k=1.0, min_cm=0.0,
                                max_cm=1000.0, tol_px=5.0)
    assert low.distance_cm == 25      # 0.25 * 1.0 * 100
    assert high.distance_cm == 50     # 0.25 * 2.0 * 100
    assert high.distance_cm == pytest.approx(2 * low.distance_cm)


def test_pixel_move_clamped_to_max_cm():
    # Huge offset/altitude -> step pinned to max_cm. Dropping the clamp -> huge.
    m = pixel_offset_to_move(_bbox_centred_at(640.0), frame_w=640,
                             altitude_m=50.0, k=1.0, min_cm=1.0, max_cm=30.0,
                             tol_px=5.0)
    assert m.distance_cm == 30


def test_pixel_move_clamped_to_min_cm():
    # Tiny offset just outside tol -> raw step below min_cm -> floored to min_cm.
    # cx 326 -> px +6 (> tol 5), error_norm 6/640 ~= 0.009; raw step ~1.9 cm at
    # alt 2 -> below min 10 -> floored. Dropping the clamp -> 2 cm.
    m = pixel_offset_to_move(_bbox_centred_at(326.0), frame_w=640,
                             altitude_m=2.0, k=1.0, min_cm=10.0, max_cm=100.0,
                             tol_px=5.0)
    assert m.distance_cm == 10


def test_pixel_move_altitude_zero_floors_to_min():
    # altitude 0 (on the deck) -> raw step 0 -> clamped UP to min_cm. Allowed
    # (altitude_m >= 0), never negative.
    m = pixel_offset_to_move(_bbox_centred_at(480.0), frame_w=640,
                             altitude_m=0.0, k=1.0, min_cm=4.0, max_cm=100.0,
                             tol_px=5.0)
    assert m.distance_cm == 4
    assert m.direction == Direction.RIGHT


def test_pixel_move_even_vs_odd_frame_w_centroid():
    # Odd frame_w: centre is frame_w/2 = a .5 pixel. A bbox centred exactly on
    # that half-pixel has px 0 -> None.
    assert pixel_offset_to_move((319.5, 0.0, 321.5, 40.0), frame_w=641,
                                altitude_m=2.0, k=1.0, min_cm=1.0,
                                max_cm=100.0, tol_px=0.0) is None


@pytest.mark.parametrize("kwargs", [
    {"frame_w": 0},
    {"frame_w": 640.0},          # float w: real frame_shape is ints
    {"frame_w": True},
    {"altitude_m": -0.1},
    {"altitude_m": float("nan")},
    {"k": float("inf")},
    {"min_cm": -1.0},
    {"min_cm": 50.0, "max_cm": 10.0},   # swapped bounds
    {"tol_px": -1.0},
])
def test_pixel_move_rejects_bad_params(kwargs):
    args = dict(bbox_xyxy=_bbox_centred_at(480.0), frame_w=640, altitude_m=2.0,
                k=1.0, min_cm=1.0, max_cm=100.0, tol_px=5.0)
    args.update(kwargs)
    with pytest.raises(ValueError):
        pixel_offset_to_move(**args)


def test_pixel_move_rejects_nonfinite_bbox():
    with pytest.raises(ValueError):
        pixel_offset_to_move((float("nan"), 0.0, 10.0, 10.0), frame_w=640,
                             altitude_m=2.0, k=1.0, min_cm=1.0, max_cm=100.0,
                             tol_px=5.0)


def test_pixel_move_distance_is_int():
    m = pixel_offset_to_move(_bbox_centred_at(481.0), frame_w=640,
                             altitude_m=2.0, k=1.0, min_cm=1.0, max_cm=1000.0,
                             tol_px=5.0)
    assert isinstance(m.distance_cm, int)


def test_servo_module_is_pure_no_numpy_at_runtime():
    # Sanity: the module imported with only stdlib + finals.types; math is the
    # only third-party-ish dependency and it is stdlib.
    import finals.mission.phases._servo as servo
    assert servo.math is math


def test_pixel_move_step_scales_with_k():
    # k is the ONSITE-tuned lateral gain (Gate F). A mutant that drops/ignores
    # k would emit the SAME distance for k=1 and k=2 — every other servo test
    # uses k=1.0, so without this the drop-k mutant survives the whole suite.
    # cx 480 -> px +160 -> error_norm 0.25; alt 1.0 -> step_cm = 25*k, unclamped.
    common = dict(frame_w=640, altitude_m=1.0, min_cm=0.0, max_cm=1000.0,
                  tol_px=5.0)
    m1 = pixel_offset_to_move(_bbox_centred_at(480.0), k=1.0, **common)
    m2 = pixel_offset_to_move(_bbox_centred_at(480.0), k=2.0, **common)
    assert isinstance(m1, Move) and isinstance(m2, Move)
    assert m1.distance_cm == 25
    assert m2.distance_cm == pytest.approx(2 * m1.distance_cm, abs=1)


def test_pixel_move_short_bbox_raises_typed_not_indexerror():
    # A bbox missing element [2] must fail with the typed WHAT/WHICH/WHY/CHECK
    # ValueError, NOT a bare IndexError (the deadband loop indexes [0] and [2]).
    with pytest.raises(ValueError, match="bbox_xyxy"):
        pixel_offset_to_move((10.0, 20.0, 30.0), frame_w=640, altitude_m=1.0,
                             k=1.0, min_cm=0.0, max_cm=100.0, tol_px=5.0)
