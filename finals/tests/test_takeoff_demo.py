"""finals.mission.phases.takeoff_demo — pure phase tests (no asyncio, no
adapter: the MissionPhase contract means the whole plan is testable by
stepping with hand-built AgentContexts)."""
from __future__ import annotations

import math

import pytest

from finals.config import DroneConfig
from finals.errors import ConfigError
from finals.mission.phase import AgentContext
from finals.mission.phases import PHASE_REGISTRY
from finals.mission.phases.takeoff_demo import TakeoffDemo
from finals.types import (Abort, Direction, Done, Hover, Land, Move, Rotate,
                          Takeoff, Telemetry)


def make_ctx(last_action=None, last_action_ok=None, last_action_error=None):
    return AgentContext(
        drone_id="alpha", now=100.0, mission_elapsed_s=1.0,
        telemetry=Telemetry(ts=100.0),
        last_action=last_action, last_action_ok=last_action_ok,
        last_action_error=last_action_error)


def drive_to_done(phase, max_steps=100):
    """Step with ok=True contexts until Done; return the action list."""
    actions = []
    last = None
    for _ in range(max_steps):                      # bounded (convention 3)
        action = phase.step(make_ctx(
            last_action=last, last_action_ok=None if last is None else True))
        if isinstance(action, Done):
            return actions, action
        actions.append(action)
        last = action
    pytest.fail(f"phase never returned Done within {max_steps} steps")


# ============================================================
# Registry + plan
# ============================================================
def test_registered_under_its_name():
    assert PHASE_REGISTRY["takeoff_demo"] is TakeoffDemo


def test_default_plan_is_takeoff_hover_square_land():
    actions, done = drive_to_done(TakeoffDemo())
    assert actions == (
        [Takeoff(height_cm=80), Hover(duration_s=2.0)]
        + [Move(direction=Direction.FORWARD, distance_cm=100),
           Rotate(angle_deg=90.0)] * 4
        + [Land()])
    # Done reason carries the tunables — the run log should tell the story.
    assert "80 cm" in done.reason and "4 x" in done.reason
    assert "landed" in done.reason


def test_configured_pattern_exact():
    phase = TakeoffDemo(height_cm=120, hover_s=0.0, leg_cm=50, legs=2,
                        turn_deg=-90.0)
    actions, _ = drive_to_done(phase)
    assert actions == [
        Takeoff(height_cm=120), Hover(duration_s=0.0),
        Move(direction=Direction.FORWARD, distance_cm=50),
        Rotate(angle_deg=-90.0),
        Move(direction=Direction.FORWARD, distance_cm=50),
        Rotate(angle_deg=-90.0),
        Land(),
    ]


def test_zero_legs_is_takeoff_hover_land_only():
    actions, _ = drive_to_done(TakeoffDemo(legs=0))
    assert actions == [Takeoff(height_cm=80), Hover(duration_s=2.0), Land()]


def test_done_is_stable_after_completion():
    phase = TakeoffDemo(legs=0)
    drive_to_done(phase)
    for _ in range(3):                  # stepping past the end stays Done
        assert isinstance(phase.step(make_ctx(last_action_ok=True)), Done)


# ============================================================
# Failure branch (defensive — see the module docstring)
# ============================================================
def test_failed_action_aborts_with_underlying_error():
    phase = TakeoffDemo()
    first = phase.step(make_ctx())
    assert isinstance(first, Takeoff)
    action = phase.step(make_ctx(
        last_action=first, last_action_ok=False,
        last_action_error="alpha: takeoff(80 cm) exceeded 15.0 s — check x"))
    assert isinstance(action, Abort)
    assert "alpha" in action.reason
    assert "exceeded 15.0 s" in action.reason       # the underlying error
    assert "abort" in action.reason.lower()


def test_failure_branch_fires_mid_plan_too():
    phase = TakeoffDemo()
    last = None
    for _ in range(4):                              # takeoff, hover, move, rotate
        last = phase.step(make_ctx(last_action=last,
                                   last_action_ok=True if last else None))
    action = phase.step(make_ctx(last_action=last, last_action_ok=False,
                                 last_action_error="link drop"))
    assert isinstance(action, Abort) and "link drop" in action.reason


# ============================================================
# from_config (the main.py soft construction convention)
# ============================================================
def _drone(zone=None, band=None):
    return DroneConfig(id="alpha", phases=["takeoff_demo"],
                       altitude_band_m=band, zone=zone or {})


def test_from_config_defaults():
    phase = TakeoffDemo.from_config(_drone(), cfg=None)
    assert (phase.height_cm, phase.hover_s, phase.leg_cm, phase.legs,
            phase.turn_deg) == (80, 2.0, 100, 4, 90.0)


def test_from_config_altitude_band_sets_height():
    phase = TakeoffDemo.from_config(_drone(band=1.7), cfg=None)
    assert phase.height_cm == 170


def test_from_config_zone_tunables_and_explicit_height_beats_band():
    drone = _drone(zone={"takeoff_demo": {"height_cm": 90, "legs": 3,
                                          "leg_cm": 60, "_comment": "ok"}},
                   band=1.2)
    phase = TakeoffDemo.from_config(drone, cfg=None)
    assert phase.height_cm == 90                    # explicit beats band
    assert phase.legs == 3 and phase.leg_cm == 60


def test_from_config_unknown_key_fails_loudly():
    drone = _drone(zone={"takeoff_demo": {"leg_m": 1}})
    with pytest.raises(ConfigError, match=r"alpha.*leg_m") as ei:
        TakeoffDemo.from_config(drone, cfg=None)
    assert "height_cm" in str(ei.value)             # lists the valid keys


def test_from_config_non_dict_zone_entry_fails_loudly():
    drone = _drone(zone={"takeoff_demo": 5})
    with pytest.raises(ConfigError, match=r"alpha.*object"):
        TakeoffDemo.from_config(drone, cfg=None)


# ============================================================
# Constructor validation — config values die on the ground
# ============================================================
@pytest.mark.parametrize("kwargs", [
    {"height_cm": 0},
    {"height_cm": -80},
    {"height_cm": 80.5},                # int contract (cm are discrete)
    {"height_cm": True},                # bool is an int — reject it
    {"hover_s": -1.0},
    {"hover_s": math.nan},
    {"hover_s": math.inf},
    {"leg_cm": 0},
    {"leg_cm": -100},
    {"leg_cm": True},
    {"legs": -1},
    {"legs": 2.0},
    {"legs": True},
    {"turn_deg": math.nan},
    {"turn_deg": math.inf},
])
def test_constructor_rejects_bad_tunables(kwargs):
    with pytest.raises(ConfigError, match="takeoff_demo"):
        TakeoffDemo(**kwargs)
