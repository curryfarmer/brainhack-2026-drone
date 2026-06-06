"""finals.flight.dead_reckon — DR math vs HAND-COMPUTED vectors.

Every expected value below was computed on paper from the pinned frame
convention (yaw CCW-positive from above, 0 = +north, range (-180, 180];
FORWARD d -> (dN, dE) = (d*cos(theta), -d*sin(theta)); RIGHT d ->
(d*sin(theta), d*cos(theta))) BEFORE the implementation trig was trusted.
A sign error here silently corrupts every Sighting.est_north_m later —
this is the never-tested-by-accident path of S3.

All position assertions use pytest.approx(abs=1e-12): several hand vectors
are NOT float-exact (sin(30 deg) = 0.49999999999999994; the closed square
lands at ~2e-16, not 0.0) — exact equality would pass for SOME vectors and
fail for others, the worst kind of trap.
"""
from __future__ import annotations

import dataclasses
import math

import pytest

from finals.flight.dead_reckon import DeadReckoner, DRPose, normalize_yaw_deg
from finals.types import (Abort, Direction, Done, Hover, Land, Move,
                          PositionQuality, Rotate, Takeoff, Wait)

EPS = 1e-12
SQRT2_2 = math.sqrt(2.0) / 2.0      # 0.7071067811865476


def approx(value: float):
    return pytest.approx(value, abs=EPS)


def make_dr(*actions) -> DeadReckoner:
    dr = DeadReckoner()
    for a in actions:
        dr.note_action_complete(a)
    return dr


# ============================================================
# Basics
# ============================================================
def test_initial_pose_is_origin():
    assert make_dr().pose == DRPose(0.0, 0.0, 0.0, 0.0)


def test_takeoff_sets_alt_only():
    p = make_dr(Takeoff(height_cm=80)).pose
    assert p.alt_m == approx(0.8)            # cm -> m at this boundary
    assert (p.north_m, p.east_m, p.yaw_deg) == (0.0, 0.0, 0.0)


def test_quality_is_always_dead_reckoning():
    assert DeadReckoner.QUALITY is PositionQuality.DEAD_RECKONING


def test_pose_is_frozen_snapshot():
    dr = make_dr(Takeoff(80))
    snap = dr.pose
    with pytest.raises(dataclasses.FrozenInstanceError):
        snap.alt_m = 99.0                    # type: ignore[misc]
    dr.note_action_complete(Move(Direction.FORWARD, 100))
    assert snap.north_m == 0.0               # old snapshot untouched


# ============================================================
# Hand-computed move vectors (the heart of S3)
# ============================================================
def test_square_path_returns_to_origin():
    """4x (FORWARD 100 cm, rotate +90): CCW square going north, WEST,
    south, east. Hand-computed corners: (1,0) -> (1,-1) -> (0,-1) -> (0,0)."""
    dr = make_dr(Takeoff(80))
    expected_corners = [(1.0, 0.0), (1.0, -1.0), (0.0, -1.0), (0.0, 0.0)]
    expected_yaws = [90.0, 180.0, -90.0, 0.0]      # 270 normalizes to -90
    for corner, yaw in zip(expected_corners, expected_yaws):
        dr.note_action_complete(Move(Direction.FORWARD, 100))
        dr.note_action_complete(Rotate(90.0))
        p = dr.pose
        assert p.north_m == approx(corner[0])
        assert p.east_m == approx(corner[1])
        assert p.yaw_deg == approx(yaw)
    assert dr.pose.alt_m == approx(0.8)            # square never changed alt


def test_forward_at_yaw_45_goes_northwest():
    # Facing 45 deg CCW from north = northwest: dN = +cos45, dE = -sin45.
    p = make_dr(Rotate(45.0), Move(Direction.FORWARD, 100)).pose
    assert p.north_m == approx(SQRT2_2)
    assert p.east_m == approx(-SQRT2_2)


def test_right_at_yaw_45_goes_northeast():
    # Body-right of a northwest heading points northeast: (+sin45, +cos45).
    p = make_dr(Rotate(45.0), Move(Direction.RIGHT, 100)).pose
    assert p.north_m == approx(SQRT2_2)
    assert p.east_m == approx(SQRT2_2)


def test_left_at_yaw_30():
    # left = -right = (-sin30, -cos30) = (-0.5, -cos30).
    p = make_dr(Rotate(30.0), Move(Direction.LEFT, 100)).pose
    assert p.north_m == approx(-0.5)
    assert p.east_m == approx(-math.cos(math.radians(30.0)))


def test_back_at_yaw_120():
    # forward(120) = (cos120, -sin120) = (-0.5, -sin120); BACK negates it.
    p = make_dr(Rotate(120.0), Move(Direction.BACK, 100)).pose
    assert p.north_m == approx(0.5)
    assert p.east_m == approx(math.sin(math.radians(120.0)))


def test_up_down_at_nonzero_yaw_touch_alt_only():
    """Vertical moves must NEVER route through the horizontal trig."""
    dr = make_dr(Takeoff(80), Rotate(37.0))
    dr.note_action_complete(Move(Direction.UP, 50))
    p = dr.pose
    assert p.alt_m == approx(1.3)
    assert p.north_m == 0.0 and p.east_m == 0.0
    dr.note_action_complete(Move(Direction.DOWN, 120))
    p = dr.pose
    assert p.alt_m == approx(0.1)
    assert p.north_m == 0.0 and p.east_m == 0.0


def test_down_below_zero_is_not_clamped():
    """DR records completed actions VERBATIM — clamping would hide adapter
    bugs (the adapter is the one that must refuse impossible sequences)."""
    p = make_dr(Move(Direction.DOWN, 50)).pose
    assert p.alt_m == approx(-0.5)


def test_moves_compose_from_initial_pose():
    dr = DeadReckoner(DRPose(north_m=2.0, east_m=3.0, alt_m=1.5, yaw_deg=90.0))
    dr.note_action_complete(Move(Direction.FORWARD, 100))
    p = dr.pose
    assert p.north_m == approx(2.0)          # facing west: no north change
    assert p.east_m == approx(2.0)           # west = -east
    assert p.alt_m == approx(1.5)            # initial altitude honored
    assert p.yaw_deg == approx(90.0)


def test_initial_pose_yaw_is_normalized():
    assert DeadReckoner(DRPose(0, 0, 0, 270.0)).pose.yaw_deg == approx(-90.0)


# ============================================================
# Rotation + normalization
# ============================================================
@pytest.mark.parametrize("angle, expected", [
    (370.0, 10.0),      # the handover's canonical example
    (-90.0, -90.0),     # already in range
    (270.0, -90.0),
    (180.0, 180.0),     # boundary: +180 IS the representation
    (-180.0, 180.0),    # ...so -180 maps onto it (one representation each)
    (540.0, 180.0),
    (-540.0, 180.0),
    (360.0, 0.0),
    (-360.0, 0.0),
])
def test_rotate_normalization(angle, expected):
    assert make_dr(Rotate(angle)).pose.yaw_deg == approx(expected)


def test_rotation_accumulates_then_normalizes():
    # 3 x +100 = 300 -> -60; 8 x +270 = 2160 = 6 full turns -> 0.
    assert make_dr(*[Rotate(100.0)] * 3).pose.yaw_deg == approx(-60.0)
    assert make_dr(*[Rotate(270.0)] * 8).pose.yaw_deg == approx(0.0)


@pytest.mark.parametrize("raw, expected", [
    (0.0, 0.0), (180.0, 180.0), (-180.0, 180.0), (181.0, -179.0),
    (-181.0, 179.0), (720.0, 0.0), (-450.0, -90.0), (90.5, 90.5),
])
def test_normalize_yaw_deg_function(raw, expected):
    assert normalize_yaw_deg(raw) == approx(expected)


# ============================================================
# Land semantics (decision pinned: alt -> 0, KEEP north/east/yaw)
# ============================================================
def test_land_zeroes_alt_keeps_track_and_heading():
    dr = make_dr(Takeoff(80), Rotate(45.0), Move(Direction.FORWARD, 100),
                 Land())
    p = dr.pose
    assert p.alt_m == 0.0
    assert p.north_m == approx(SQRT2_2)      # drones don't teleport
    assert p.east_m == approx(-SQRT2_2)
    assert p.yaw_deg == approx(45.0)


def test_double_land_is_idempotent():
    dr = make_dr(Takeoff(80), Land())
    before = dr.pose
    dr.note_action_complete(Land())
    assert dr.pose == before


def test_retakeoff_continues_the_track():
    dr = make_dr(Takeoff(80), Move(Direction.FORWARD, 100), Land(),
                 Takeoff(120))
    p = dr.pose
    assert p.north_m == approx(1.0)          # track survived the landing
    assert p.alt_m == approx(1.2)


# ============================================================
# No-ops + fail-loud unknown types
# ============================================================
@pytest.mark.parametrize("noop", [
    Hover(duration_s=2.0),
    Wait(duration_s=1.0),
    Done(reason="phase complete"),
    Abort(reason="test"),
])
def test_non_translating_actions_are_noops(noop):
    dr = make_dr(Takeoff(80), Rotate(45.0), Move(Direction.FORWARD, 100))
    before = dr.pose
    dr.note_action_complete(noop)
    assert dr.pose == before


def test_unknown_action_type_raises_typeerror_naming_it():
    with pytest.raises(TypeError, match="str"):
        make_dr().note_action_complete("takeoff")   # type: ignore[arg-type]
    with pytest.raises(TypeError, match="NoneType"):
        make_dr().note_action_complete(None)        # type: ignore[arg-type]


def test_unknown_direction_raises_typeerror_naming_it():
    """The closed-IntEnum else-branch must stay fail-loud if Direction ever
    grows a value (or a raw int sneaks in through the frozen dataclass)."""
    with pytest.raises(TypeError, match="99"):
        make_dr().note_action_complete(Move(99, 100))  # type: ignore[arg-type]


# ============================================================
# Non-finite inputs — NaN would poison north/east SILENTLY otherwise
# ============================================================
@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_normalize_yaw_deg_rejects_non_finite(bad):
    with pytest.raises(ValueError, match="finite"):
        normalize_yaw_deg(bad)


def test_rotate_rejects_non_finite_angle():
    dr = make_dr(Takeoff(80))
    with pytest.raises(ValueError, match="finite"):
        dr.note_action_complete(Rotate(float("nan")))
    assert dr.pose.yaw_deg == 0.0               # yaw not poisoned


def test_move_and_takeoff_reject_non_finite_magnitudes():
    with pytest.raises(ValueError, match=r"Move\.distance_cm"):
        make_dr().note_action_complete(Move(Direction.FORWARD, float("inf")))
    with pytest.raises(ValueError, match=r"Takeoff\.height_cm"):
        make_dr().note_action_complete(Takeoff(float("nan")))


def test_constructor_rejects_non_finite_initial_pose():
    with pytest.raises(ValueError, match="north_m"):
        DeadReckoner(DRPose(float("nan"), 0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="finite"):
        DeadReckoner(DRPose(0.0, 0.0, 0.0, float("inf")))
