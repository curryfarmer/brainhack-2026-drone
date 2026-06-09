"""finals.mission.phases.search — SentryScan + OpenLoopLawnmower.

Pure phase tests (the MissionPhase contract: the whole plan is testable by
stepping with hand-built AgentContexts) plus an agent-over-MockAdapter
integration — the search phase flies on a real adapter, and canned Sightings on
the bus surface to the phase (drone-id filtered), the path search relies on for
its sightings.csv output. No pytest-asyncio: coroutines run via asyncio.run()."""
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
from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases import PHASE_REGISTRY
from finals.mission.phases.search import OpenLoopLawnmower, SentryScan
from finals.sightings import SightingBus
from finals.types import (Abort, Action, Direction, Done, Hover, Land, Move,
                          Rotate, Sighting, Takeoff, Telemetry, Wait)


# ---------------- pure-stepping harness (mirrors test_takeoff_demo) ----------
def make_ctx(last_action=None, last_action_ok=None, last_action_error=None,
             sightings=None):
    return AgentContext(
        drone_id="alpha", now=100.0, mission_elapsed_s=1.0,
        telemetry=Telemetry(ts=100.0), sightings=sightings or [],
        last_action=last_action, last_action_ok=last_action_ok,
        last_action_error=last_action_error)


def drive_to_done(phase, max_steps=500):
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


def _drone(zone=None, band=None):
    return DroneConfig(id="alpha", phases=["sentry_scan"],
                       altitude_band_m=band, zone=zone or {})


# ============================================================
# Registry
# ============================================================
def test_registered_under_their_names():
    assert PHASE_REGISTRY["sentry_scan"] is SentryScan
    assert PHASE_REGISTRY["lawnmower"] is OpenLoopLawnmower


# ============================================================
# SentryScan — plan
# ============================================================
def test_sentry_default_plan_is_takeoff_hover_rotate_land():
    actions, done = drive_to_done(SentryScan(revolutions=1.0))
    assert actions == (
        [Takeoff(height_cm=80)]
        + [Hover(duration_s=2.0), Rotate(angle_deg=45.0)] * 8   # 360/45
        + [Land()])
    assert "sentry_scan complete" in done.reason and "1 rev" in done.reason
    assert "landed" in done.reason


def test_sentry_step_count_covers_revolutions():
    assert SentryScan(step_deg=90.0, revolutions=2.0).steps == 8     # 360*2/90
    assert SentryScan(step_deg=45.0, revolutions=3.0).steps == 24    # 360*3/45
    assert SentryScan(step_deg=-45.0, revolutions=1.0).steps == 8    # abs() math


def test_sentry_negative_step_rotates_clockwise():
    actions, _ = drive_to_done(SentryScan(step_deg=-90.0, revolutions=1.0))
    rotates = [a for a in actions if isinstance(a, Rotate)]
    assert rotates and all(r.angle_deg == -90.0 for r in rotates)


def test_sentry_done_is_stable_after_completion():
    phase = SentryScan(revolutions=1.0)
    drive_to_done(phase)
    for _ in range(3):
        assert isinstance(phase.step(make_ctx(last_action_ok=True)), Done)


def test_sentry_ignores_sightings_in_step():
    """Detection is the PerceptionLoop's job; the pure plan must not branch on
    ctx.sightings (the convoy re-crosses the static footprint)."""
    phase = SentryScan(revolutions=1.0)
    with_s = phase.step(make_ctx(sightings=[object()]))
    assert isinstance(with_s, Takeoff)           # same as without sightings


# ============================================================
# SentryScan — failure branch + from_config + validation
# ============================================================
def test_sentry_failed_action_aborts_with_underlying_error():
    phase = SentryScan(revolutions=1.0)
    first = phase.step(make_ctx())
    assert isinstance(first, Takeoff)
    action = phase.step(make_ctx(
        last_action=first, last_action_ok=False,
        last_action_error="alpha: takeoff(80 cm) exceeded 30.0 s"))
    assert isinstance(action, Abort)
    assert "alpha" in action.reason and "exceeded 30.0 s" in action.reason
    assert "abort" in action.reason.lower()


def test_sentry_from_config_defaults():
    p = SentryScan.from_config(_drone(), cfg=None)
    assert (p.height_cm, p.dwell_s, p.step_deg, p.revolutions) == \
        (80, 2.0, 45.0, 3.0)


def test_sentry_from_config_altitude_band_sets_height():
    assert SentryScan.from_config(_drone(band=1.7), cfg=None).height_cm == 170


def test_sentry_from_config_zone_tunables_and_explicit_height_beats_band():
    drone = _drone(zone={"sentry_scan": {"height_cm": 150, "dwell_s": 1.5,
                                         "step_deg": 90, "revolutions": 2,
                                         "_comment": "ok"}}, band=1.2)
    p = SentryScan.from_config(drone, cfg=None)
    assert p.height_cm == 150 and p.dwell_s == 1.5 and p.step_deg == 90.0
    assert p.revolutions == 2.0 and p.steps == 8


def test_sentry_from_config_unknown_key_fails_loudly():
    drone = _drone(zone={"sentry_scan": {"dwel_s": 1}})
    with pytest.raises(ConfigError, match=r"alpha.*dwel_s") as ei:
        SentryScan.from_config(drone, cfg=None)
    assert "dwell_s" in str(ei.value)            # lists the valid keys


def test_sentry_from_config_non_dict_zone_fails_loudly():
    drone = _drone(zone={"sentry_scan": 5})
    with pytest.raises(ConfigError, match=r"alpha.*object"):
        SentryScan.from_config(drone, cfg=None)


@pytest.mark.parametrize("kwargs", [
    {"height_cm": 0}, {"height_cm": -1}, {"height_cm": 80.5}, {"height_cm": True},
    {"dwell_s": 0}, {"dwell_s": -1.0}, {"dwell_s": math.nan}, {"dwell_s": math.inf},
    {"step_deg": 0}, {"step_deg": math.nan}, {"step_deg": math.inf},
    {"revolutions": 0}, {"revolutions": -1}, {"revolutions": math.nan},
])
def test_sentry_constructor_rejects_bad_tunables(kwargs):
    with pytest.raises(ConfigError, match="sentry_scan"):
        SentryScan(**kwargs)


# ============================================================
# OpenLoopLawnmower
# ============================================================
def test_lawnmower_default_plan_is_boustrophedon():
    actions, done = drive_to_done(OpenLoopLawnmower(lanes=2))
    assert actions == [
        Takeoff(height_cm=80),
        Move(direction=Direction.FORWARD, distance_cm=400),
        Hover(duration_s=1.0),
        Rotate(angle_deg=90.0),                          # U-turn out
        Move(direction=Direction.FORWARD, distance_cm=300),
        Rotate(angle_deg=90.0),                          # U-turn in
        Move(direction=Direction.FORWARD, distance_cm=400),
        Hover(duration_s=1.0),
        Land(),
    ]
    assert "lawnmower complete" in done.reason and "landed" in done.reason


def test_lawnmower_alternates_uturn_direction():
    actions, _ = drive_to_done(OpenLoopLawnmower(lanes=3, turn_deg=90.0))
    rotates = [a.angle_deg for a in actions if isinstance(a, Rotate)]
    assert rotates == [90.0, 90.0, -90.0, -90.0]         # two opposite U-turns


def test_lawnmower_from_config_band_and_typo_guard():
    p = OpenLoopLawnmower.from_config(_drone(band=1.2), cfg=None)
    assert p.height_cm == 120
    with pytest.raises(ConfigError, match=r"alpha.*lane_m"):
        OpenLoopLawnmower.from_config(
            _drone(zone={"lawnmower": {"lane_m": 1}}), cfg=None)


@pytest.mark.parametrize("kwargs", [
    {"height_cm": 0}, {"lanes": 0}, {"lanes": 2.0}, {"leg_cm": 0},
    {"lane_cm": -1}, {"turn_deg": 0}, {"turn_deg": math.inf},
    {"scan_pause_s": -1.0},
])
def test_lawnmower_constructor_rejects_bad_tunables(kwargs):
    with pytest.raises(ConfigError, match="lawnmower"):
        OpenLoopLawnmower(**kwargs)


# ============================================================
# Integration — search over MockAdapter + canned sightings
# ============================================================
def _run_agent(agent, *, budget_s: float = 30.0) -> None:
    async def go():
        stop = asyncio.Event()
        await agent.run(deadline=time.monotonic() + budget_s, stop_event=stop)
    asyncio.run(go())


def test_sentry_scan_flies_full_pattern_over_mock_adapter(tmp_path):
    adapter = MockAdapter("alpha")
    with EventLog(str(tmp_path)) as events:
        agent = DroneAgent("alpha", adapter,
                           [SentryScan(dwell_s=0.01, revolutions=1.0)], events)
        _run_agent(agent)
        asyncio.run(agent.shutdown())
    names = [c[0] for c in adapter.calls]
    assert names == (["connect", "takeoff"] + ["hover", "rotate"] * 8
                     + ["land", "disconnect"])
    assert agent.state is AgentState.DONE
    assert not adapter.is_flying


class _CaptureSightings(MissionPhase):
    """Takeoff, then accumulate every ctx.sightings until some arrive -> Done.
    NOT registered (test_conventions pins PHASE_REGISTRY exactly)."""

    name = "capture_sightings"

    def __init__(self):
        self._took_off = False
        self.seen = []

    def step(self, ctx: AgentContext) -> Action:
        self.seen.extend(ctx.sightings)          # drain may land on tick 1
        if not self._took_off:
            self._took_off = True
            return Takeoff(height_cm=80)
        if self.seen:
            return Done("captured a sighting")
        return Wait(duration_s=0.01)


def _sighting(drone_id: str, marker_id: int) -> Sighting:
    return Sighting(
        drone_id=drone_id, ts=time.monotonic(), source="aruco",
        class_name=f"aruco_{marker_id}", marker_id=marker_id,
        bbox_xyxy=(0.0, 0.0, 10.0, 10.0), confidence=1.0,
        frame_shape=(480, 640))


def test_agent_surfaces_canned_sightings_to_phase_drone_filtered(tmp_path):
    bus = SightingBus()
    bus.publish(_sighting("alpha", 7))
    bus.publish(_sighting("beta", 11))           # another drone — must NOT surface
    phase = _CaptureSightings()
    adapter = MockAdapter("alpha")
    with EventLog(str(tmp_path)) as events:
        agent = DroneAgent("alpha", adapter, [phase], events, bus=bus)
        _run_agent(agent, budget_s=10.0)
        asyncio.run(agent.shutdown())
    assert agent.state is AgentState.DONE
    assert sorted(s.marker_id for s in phase.seen) == [7]   # beta filtered out
