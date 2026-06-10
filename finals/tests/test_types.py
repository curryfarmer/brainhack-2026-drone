"""finals.types — enum parity, immutability, Action vocabulary."""
from __future__ import annotations

import dataclasses

import pytest

from finals.types import (
    Abort, Direction, Done, FrameStamped, Hover, Land, Move, PositionQuality,
    Rotate, Sighting, Takeoff, Telemetry, Wait,
)


def test_direction_mirrors_pyhulax_values():
    """Parity with pyhulax.core.Direction asserted against HARDCODED ints —
    never against a pyhulax import (SITL machines don't have the SDK)."""
    assert {d.name: d.value for d in Direction} == {
        "FORWARD": 0, "BACK": 1, "LEFT": 2, "RIGHT": 3, "UP": 4, "DOWN": 5,
    }


def test_position_quality_is_ordered():
    assert (PositionQuality.NONE < PositionQuality.DEAD_RECKONING
            < PositionQuality.UNTRUSTED < PositionQuality.MEASURED)


def test_telemetry_age_and_defaults():
    t = Telemetry(ts=100.0)
    assert t.age_s(now=102.5) == pytest.approx(2.5)
    assert t.battery_pct is None
    assert t.position_quality is PositionQuality.NONE
    assert t.raw == {}


def test_telemetry_is_frozen():
    t = Telemetry(ts=1.0)
    with pytest.raises(dataclasses.FrozenInstanceError):
        t.battery_pct = 50.0  # type: ignore[misc]


def test_sighting_defaults_nullable_world_fields():
    s = Sighting(
        drone_id="alpha", ts=1.0, source="aruco", class_name="aruco_17",
        marker_id=17, bbox_xyxy=(0, 0, 10, 10), confidence=1.0,
        frame_shape=(480, 640),
    )
    assert s.bearing_deg is None
    assert s.pos_quality is PositionQuality.NONE
    assert s.est_north_m is None and s.est_east_m is None
    with pytest.raises(dataclasses.FrozenInstanceError):
        s.confidence = 0.5  # type: ignore[misc]


def test_action_vocabulary():
    assert Takeoff().height_cm == 80
    assert Move(Direction.FORWARD, 100).distance_cm == 100
    assert Rotate(90.0).angle_deg == 90.0
    assert Hover(2.0).duration_s == 2.0
    assert Wait(0.5).duration_s == 0.5
    assert Done("finished").reason == "finished"
    assert Abort("battery").reason == "battery"
    Land()  # constructible, carries nothing
    # All actions are frozen — phases can't mutate what they emitted.
    with pytest.raises(dataclasses.FrozenInstanceError):
        Takeoff().height_cm = 120  # type: ignore[misc]


def test_frame_stamped_shape():
    fields = {f.name for f in dataclasses.fields(FrameStamped)}
    assert fields == {"image", "ts", "frame_number", "source_id", "depth"}


def test_frame_stamped_depth_defaults_none():
    """The depth seam (SENSE-IR) degrades absent: monocular frames omit it."""
    fr = FrameStamped(image=None, ts=1.0, frame_number=1, source_id="alpha")
    assert fr.depth is None
