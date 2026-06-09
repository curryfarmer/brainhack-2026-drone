"""finals.mission.phases.track_convoy — the reactive bearing-pursuit tracker.

Pure phase tests (the MissionPhase contract: drive step() with hand-built
AgentContexts) + one agent-over-MockAdapter smoke. No pytest-asyncio; coroutines
run via asyncio.run() (mirrors test_search)."""
from __future__ import annotations

import asyncio
import math
import time

import pytest

from finals.config import DroneConfig
from finals.errors import ConfigError
from finals.events import EventLog
from finals.flight.mock_adapter import MockAdapter
from finals.mission.agent import AgentState, DroneAgent
from finals.mission.phase import AgentContext
from finals.mission.phases import PHASE_REGISTRY
from finals.mission.phases.track_convoy import TrackConvoy
from finals.sightings import SightingBus
from finals.types import (Abort, Direction, Done, Hover, Move, Rotate, Sighting,
                          Takeoff, Telemetry)


# ---------------- pure-stepping harness ----------------
def ctx(now=100.0, yaw=0.0, is_flying=True, sightings=None,
        last_action=None, last_action_ok=None, last_action_error=None):
    return AgentContext(
        drone_id="alpha", now=now, mission_elapsed_s=now,
        telemetry=Telemetry(ts=now, yaw_deg=yaw, is_flying=is_flying),
        sightings=sightings or [], last_action=last_action,
        last_action_ok=last_action_ok, last_action_error=last_action_error)


def sight(marker_id=7, bearing=None, ts=100.0, bbox=(0.0, 0.0, 10.0, 10.0)):
    # Default bbox is the top-left CORNER of a 640x480 frame -> radial offset
    # ~1.39 (well past any sane center_px_frac), so the steer tests that expect
    # a Move are not silently swallowed by the deadband. Centre-frame tests pass
    # an explicit centred bbox.
    return Sighting(
        drone_id="alpha", ts=ts, source="aruco",
        class_name=f"aruco_{marker_id}", marker_id=marker_id,
        bbox_xyxy=bbox, confidence=1.0,
        frame_shape=(480, 640), bearing_deg=bearing)


# A marker box centred in a 640x480 frame (radial offset 0 -> inside the
# deadband): the car is directly under the drone.
_CENTRED_BBOX = (310.0, 230.0, 330.0, 250.0)


def _drone(zone=None, band=None):
    return DroneConfig(id="alpha", phases=["track_convoy"],
                       altitude_band_m=band, zone=zone or {})


# ============================================================
# Registry
# ============================================================
def test_registered_under_its_name():
    assert PHASE_REGISTRY["track_convoy"] is TrackConvoy


# ============================================================
# INIT — takeoff vs already-airborne
# ============================================================
def test_takeoff_when_landed_then_acquires():
    p = TrackConvoy()
    a = p.step(ctx(is_flying=False))
    assert isinstance(a, Takeoff) and a.height_cm == 80
    a2 = p.step(ctx(is_flying=True, last_action=a, last_action_ok=True))
    assert isinstance(a2, Hover)                 # straight into acquire dwell


def test_skips_takeoff_when_already_flying():
    a = TrackConvoy().step(ctx(is_flying=True))
    assert isinstance(a, Hover)                  # acquire dwell, no takeoff


# ============================================================
# ACQUIRE + steer
# ============================================================
def test_acquires_after_hits_then_rotates_ccw_toward_positive_bearing():
    p = TrackConvoy(acquire_hits=3, center_tol_deg=5.0, max_step_deg=30.0)
    s = [sight(7, bearing=40.0, ts=100.0) for _ in range(3)]
    a = p.step(ctx(yaw=0.0, sightings=s))
    assert p._target_id == 7
    assert isinstance(a, Rotate) and a.angle_deg == 30.0     # +err, clamped CCW


def test_rotates_clockwise_toward_negative_bearing():
    p = TrackConvoy(acquire_hits=1, center_tol_deg=5.0, max_step_deg=30.0)
    a = p.step(ctx(yaw=0.0, sightings=[sight(7, bearing=-40.0, ts=100.0)]))
    assert isinstance(a, Rotate) and a.angle_deg == -30.0


def test_small_error_is_not_clamped():
    p = TrackConvoy(acquire_hits=1, center_tol_deg=2.0, max_step_deg=30.0)
    a = p.step(ctx(yaw=10.0, sightings=[sight(7, bearing=25.0, ts=100.0)]))
    assert isinstance(a, Rotate) and a.angle_deg == 15.0     # 25 - 10


def test_centered_hovers_when_approach_disabled():
    p = TrackConvoy(acquire_hits=1, center_tol_deg=10.0, approach_enabled=False)
    a = p.step(ctx(yaw=0.0, sightings=[sight(7, bearing=3.0, ts=100.0)]))
    assert isinstance(a, Hover)                  # safe observer, no translation


def test_centered_moves_then_hovers_when_approach_enabled():
    p = TrackConvoy(acquire_hits=1, center_tol_deg=10.0, approach_enabled=True,
                    approach_cm=50, max_chase_cm=1000)
    a = p.step(ctx(yaw=0.0, sightings=[sight(7, bearing=0.0, ts=100.0)]))
    assert isinstance(a, Move) and a.direction is Direction.FORWARD
    assert a.distance_cm == 50 and p._chase_used_cm == 50
    a2 = p.step(ctx(yaw=0.0, sightings=[sight(7, bearing=0.0, ts=101.0)],
                    last_action_ok=True))
    assert isinstance(a2, Hover)                 # look after the move


def test_chase_cap_returns_done():
    p = TrackConvoy(acquire_hits=1, center_tol_deg=10.0, approach_enabled=True,
                    approach_cm=100, max_chase_cm=100)
    a1 = p.step(ctx(sightings=[sight(7, 0.0, 100.0)]))
    assert isinstance(a1, Move) and a1.distance_cm == 100
    a2 = p.step(ctx(sightings=[sight(7, 0.0, 101.0)], last_action_ok=True))
    assert isinstance(a2, Hover)                 # _just_moved
    a3 = p.step(ctx(sightings=[sight(7, 0.0, 102.0)], last_action_ok=True))
    assert isinstance(a3, Done) and "chase cap" in a3.reason


# ============================================================
# NADIR deadband — hold over a centred (under-drone) target, step when it drifts
# ============================================================
def test_centered_marker_holds_when_under_drone():
    # Bearing centred AND the marker box is in the frame centre = car under us:
    # HOLD (deadband). Without this the drone over-walks a near-stationary car
    # out of its footprint — the VM failure where the small-band drones lost
    # their cars while charlie's bigger footprint masked the over-walk.
    p = TrackConvoy(acquire_hits=1, center_tol_deg=10.0, approach_enabled=True,
                    approach_cm=50, center_px_frac=0.3)
    a = p.step(ctx(yaw=0.0, sightings=[
        sight(7, bearing=0.0, ts=100.0, bbox=_CENTRED_BBOX)]))
    assert isinstance(a, Hover) and p._chase_used_cm == 0   # held, no chase used


def test_offcenter_marker_steps_past_deadband():
    # Same bearing-centred, but the marker has drifted to the frame corner
    # (radial offset ~1.39 > the 0.3 deadband): the car left centre -> step.
    p = TrackConvoy(acquire_hits=1, center_tol_deg=10.0, approach_enabled=True,
                    approach_cm=50, center_px_frac=0.3)
    a = p.step(ctx(yaw=0.0, sightings=[sight(7, bearing=0.0, ts=100.0)]))  # corner
    assert isinstance(a, Move) and a.distance_cm == 50


def test_deadband_degrades_to_move_on_missing_frame_geometry():
    # off is None (no frame_shape) -> fall back to the prior always-step
    # behavior, never freeze on bad data.
    import dataclasses
    p = TrackConvoy(acquire_hits=1, center_tol_deg=10.0, approach_enabled=True,
                    approach_cm=50, center_px_frac=0.3)
    s = dataclasses.replace(sight(7, bearing=0.0, ts=100.0, bbox=_CENTRED_BBOX),
                            frame_shape=None)
    a = p.step(ctx(yaw=0.0, sightings=[s]))
    assert isinstance(a, Move) and a.distance_cm == 50


# ============================================================
# LOST / budgets / degrade
# ============================================================
def test_lost_timeout_drops_lock_and_reacquires():
    p = TrackConvoy(acquire_hits=1, lost_timeout_s=3.0, center_tol_deg=10.0)
    p.step(ctx(now=100.0, sightings=[sight(7, 0.0, 100.0)]))
    assert p._target_id == 7
    a = p.step(ctx(now=104.0, sightings=[], last_action_ok=True))   # >3 s gap
    assert isinstance(a, Hover)
    assert p._target_id is None and p._state == "acquire"


def test_brief_loss_keeps_lock():
    p = TrackConvoy(acquire_hits=1, lost_timeout_s=3.0)
    p.step(ctx(now=100.0, sightings=[sight(7, 0.0, 100.0)]))
    p.step(ctx(now=101.0, sightings=[], last_action_ok=True))       # 1 s gap
    assert p._target_id == 7 and p._state == "track"


def test_reacquire_gets_fresh_budget_after_late_loss():
    # Lose the lock LATE (phase-elapsed already past acquire_budget_s). The
    # re-acquire must get a FRESH budget measured from re-entry — not die
    # instantly because the PHASE clock already passed acquire_budget_s (the
    # vestigial-reacquire bug a phase-relative budget would cause).
    p = TrackConvoy(acquire_hits=1, acquire_budget_s=5.0,
                    investigate_budget_s=200.0, lost_timeout_s=3.0,
                    center_tol_deg=10.0)
    p.step(ctx(now=100.0, sightings=[sight(7, 0.0, 100.0)]))        # t_enter=100
    assert p._target_id == 7
    a = p.step(ctx(now=150.0, sightings=[], last_action_ok=True))   # lose @ +50 s
    assert isinstance(a, Hover) and p._state == "acquire"
    assert p._target_id is None
    a2 = p.step(ctx(now=151.0, sightings=[], last_action_ok=True))
    assert isinstance(a2, Hover)                 # fresh budget: still searching
    p.step(ctx(now=152.0, sightings=[sight(7, 0.0, 152.0)],
               last_action_ok=True))             # target reappears -> re-locks
    assert p._target_id == 7 and p._state == "track"


def test_investigate_budget_done():
    p = TrackConvoy(investigate_budget_s=10.0)
    p.step(ctx(now=100.0))                       # enter, t_enter = 100
    a = p.step(ctx(now=111.0, last_action_ok=True))
    assert isinstance(a, Done) and "investigate budget" in a.reason


def test_acquire_budget_done_when_no_target():
    p = TrackConvoy(acquire_budget_s=5.0, investigate_budget_s=100.0)
    p.step(ctx(now=100.0))
    a = p.step(ctx(now=106.0, last_action_ok=True))
    assert isinstance(a, Done) and "nothing to track" in a.reason


def test_bearing_none_degrades_to_hover():
    p = TrackConvoy(acquire_hits=1, approach_enabled=True)
    a = p.step(ctx(yaw=0.0, sightings=[sight(7, bearing=None, ts=100.0)]))
    assert isinstance(a, Hover)                  # seen but unsteerable


def test_yaw_none_degrades_to_hover():
    p = TrackConvoy(acquire_hits=1, approach_enabled=True)
    a = p.step(ctx(yaw=None, sightings=[sight(7, bearing=20.0, ts=100.0)]))
    assert isinstance(a, Hover)


def test_failed_action_aborts_with_underlying_error():
    p = TrackConvoy()
    a = p.step(ctx(last_action=Takeoff(height_cm=80), last_action_ok=False,
                   last_action_error="alpha: takeoff(80 cm) exceeded 30.0 s"))
    assert isinstance(a, Abort)
    assert "exceeded 30.0 s" in a.reason and "alpha" in a.reason


# ============================================================
# Target selection
# ============================================================
def test_track_marker_ids_filters_acquisition():
    p = TrackConvoy(acquire_hits=1, track_marker_ids=[7])
    a = p.step(ctx(sightings=[sight(11, 0.0, 100.0)]))   # wrong id ignored
    assert isinstance(a, Hover) and p._target_id is None
    p.step(ctx(sightings=[sight(7, 0.0, 101.0)], last_action_ok=True))
    assert p._target_id == 7


def test_locks_most_seen_id():
    p = TrackConvoy(acquire_hits=2, center_tol_deg=90.0)
    s = [sight(7, 0.0, 100.0), sight(11, 0.0, 100.0), sight(11, 0.0, 100.0)]
    p.step(ctx(sightings=s))
    assert p._target_id == 11                    # 2 hits beats 1


# ============================================================
# Constructor validation + from_config
# ============================================================
@pytest.mark.parametrize("kwargs", [
    {"height_cm": 0}, {"height_cm": 80.5}, {"height_cm": True},
    {"acquire_hits": 0}, {"acquire_hits": 2.0},
    {"acquire_window_s": 0}, {"acquire_window_s": math.inf},
    {"acquire_dwell_s": -1.0}, {"center_tol_deg": 0}, {"max_step_deg": -1},
    {"track_dwell_s": math.nan}, {"approach_cm": 0}, {"max_chase_cm": 0},
    {"center_px_frac": 0}, {"center_px_frac": -0.1}, {"center_px_frac": 2.0},
    {"center_px_frac": math.nan},
    {"approach_enabled": "yes"}, {"lead_gain": -1}, {"lead_gain": math.nan},
    {"track_marker_ids": 7}, {"track_marker_ids": [7, "x"]},
    {"track_marker_ids": [True]}, {"reacquire_dwell_s": 0},
    {"lost_timeout_s": math.nan}, {"investigate_budget_s": 0},
])
def test_constructor_rejects_bad_tunables(kwargs):
    with pytest.raises(ConfigError, match="track_convoy"):
        TrackConvoy(**kwargs)


def test_from_config_defaults():
    p = TrackConvoy.from_config(_drone(), cfg=None)
    assert p.height_cm == 80 and p.acquire_hits == 3
    assert p.approach_enabled is False and p.track_marker_ids is None


def test_from_config_altitude_band_sets_height():
    assert TrackConvoy.from_config(_drone(band=2.2), cfg=None).height_cm == 220


def test_from_config_zone_tunables():
    d = _drone(zone={"track_convoy": {"track_marker_ids": [7],
                                      "approach_enabled": True,
                                      "approach_cm": 30, "_comment": "ok"}})
    p = TrackConvoy.from_config(d, cfg=None)
    assert p.track_marker_ids == {7} and p.approach_enabled is True
    assert p.approach_cm == 30


def test_from_config_unknown_key_fails_loudly():
    d = _drone(zone={"track_convoy": {"approch_cm": 1}})
    with pytest.raises(ConfigError, match=r"alpha.*approch_cm"):
        TrackConvoy.from_config(d, cfg=None)


# ============================================================
# Integration — flies + completes over MockAdapter
# ============================================================
def test_takes_off_and_completes_over_mock_adapter(tmp_path):
    bus = SightingBus()
    adapter = MockAdapter("alpha")
    phase = TrackConvoy(acquire_budget_s=0.2, investigate_budget_s=0.5,
                        acquire_dwell_s=0.01)
    with EventLog(str(tmp_path)) as events:
        agent = DroneAgent("alpha", adapter, [phase], events, bus=bus)
        async def go():
            stop = asyncio.Event()
            await agent.run(deadline=time.monotonic() + 10.0, stop_event=stop)
        asyncio.run(go())
        asyncio.run(agent.shutdown())
    names = [c[0] for c in adapter.calls]
    assert "takeoff" in names and "land" in names
    assert agent.state is AgentState.DONE
    assert not adapter.is_flying
