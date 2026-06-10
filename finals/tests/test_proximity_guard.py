"""finals.guards.ProximityGuard — the SENSE-IR 4-directional IR obstacle guard.

Covers the session gates: the guard TRIPS at the LAND threshold and CLEARS
below it; the advisory-vs-LAND ladder ORDERING (clear -> ADVISORY at warn ->
LAND_THIS at land, closest-of-four wins); the edge-latched advisory re-arms on
all-clear; a missing IR feed (GuardContext.proximity None) is a clean SKIP
(degrade-absent, never a guess); a bad reading fails SAFE (LAND_THIS); the
constructor rejects bad thresholds; the proximity_* config validation is loud;
and the END-TO-END path — an agent + ProximityGuard over MockAdapter, a
near-obstacle reading fires the guard -> the drone safes down clean while a
sibling drone continues (mirrors test_guards.py's deconfliction e2e).

No pytest-asyncio: coroutines are driven with asyncio.run() (suite convention).
"""
from __future__ import annotations

import asyncio
import os
import time

import pytest

from finals.config import load_config
from finals.errors import ConfigError
from finals.events import EventLog, read_events
from finals.flight.mock_adapter import MockAdapter
from finals.guards import (GuardContext, ProximityGuard, ProximityReading,
                           TripAction)
from finals.mission.agent import AgentState, DroneAgent
from finals.mission.orchestrator import Orchestrator
from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases.takeoff_demo import TakeoffDemo
from finals.types import Action, Takeoff, Wait


# ---------------- helpers ----------------
def names(calls):
    return [c[0] for c in calls]


def events_of(run_dir: str):
    return list(read_events(os.path.join(run_dir, "mission.jsonl")))


def guard_trips_of(run_dir: str):
    return [e for e in events_of(run_dir) if e["event"] == "guard_trip"]


def gctx(reading, drone_id="alpha", now=100.0, mission_elapsed_s=10.0):
    return GuardContext(drone_id=drone_id, now=now,
                        mission_elapsed_s=mission_elapsed_s, proximity=reading)


def reading(**kw) -> ProximityReading:
    kw.setdefault("ts", 100.0)
    return ProximityReading(**kw)


@pytest.fixture
def run_dir(tmp_path):
    return str(tmp_path)


def run_agent(agent, *, budget_s: float = 30.0) -> None:
    async def go():
        await agent.run(deadline=time.monotonic() + budget_s,
                        stop_event=asyncio.Event())

    asyncio.run(go())


class WaitForeverPhase(MissionPhase):
    name = "wait_forever"

    def __init__(self, wait_s: float = 10.0):
        self._took_off = False
        self._wait_s = wait_s

    def step(self, ctx: AgentContext) -> Action:
        if not self._took_off:
            self._took_off = True
            return Takeoff(height_cm=80)
        return Wait(duration_s=self._wait_s)


class ObstacleAfterTakeoff:
    """A proximity_fn that reports CLEAR until the drone is airborne, then the
    near obstacle — so the guard trips MID-FLIGHT and the agent exercises the
    clean airborne-land path (not a pre-takeoff refusal). Keyed off the
    adapter's is_flying (MockAdapter sets it on takeoff)."""

    def __init__(self, adapter, near: ProximityReading):
        self._adapter = adapter
        self._near = near

    def __call__(self):
        return self._near if self._adapter.is_flying else reading()


# ============================================================
# 1. The ladder — trip at threshold, clear below it, ordering
# ============================================================
def test_proximity_skips_when_no_reading():
    """No IR feed (proximity None) -> SKIP, never a guess (degrade-absent)."""
    guard = ProximityGuard(warn_cm=40.0, land_cm=25.0)
    assert guard.check(gctx(None)) is None


def test_proximity_clear_when_all_directions_clear_or_far():
    guard = ProximityGuard(warn_cm=40.0, land_cm=25.0)
    # All directions None = clear (no return in the sensing window).
    assert guard.check(gctx(reading())) is None
    # All four present but beyond the warn band -> quiet.
    assert guard.check(gctx(reading(front_cm=80.0, back_cm=90.0,
                                    left_cm=100.0, right_cm=70.0))) is None


def test_proximity_advisory_at_warn_band():
    """Between land and warn -> ADVISORY (flight unaffected)."""
    guard = ProximityGuard(warn_cm=40.0, land_cm=25.0)
    trip = guard.check(gctx(reading(front_cm=35.0)))
    assert trip is not None and trip.action is TripAction.ADVISORY
    assert trip.guard == "ProximityGuard"
    # message bar: WHAT (front), MEASURED vs LIMIT, CHECK.
    assert "front" in trip.reason and "35" in trip.reason
    assert "40" in trip.reason and "ADVISORY" in trip.reason
    assert "check" in trip.reason


def test_proximity_lands_at_hard_stop():
    """At/under the land hard-stop -> LAND_THIS."""
    guard = ProximityGuard(warn_cm=40.0, land_cm=25.0)
    trip = guard.check(gctx(reading(right_cm=25.0)))
    assert trip is not None and trip.action is TripAction.LAND_THIS
    assert "right" in trip.reason and "25" in trip.reason
    assert "alpha" in trip.reason


def test_proximity_exactly_at_thresholds_is_inclusive():
    """The boundary is INCLUSIVE (<=) on both rungs — at exactly warn_cm it
    advises, at exactly land_cm it lands (a reading sitting on the line must
    not slip through)."""
    guard_w = ProximityGuard(warn_cm=40.0, land_cm=25.0)
    at_warn = guard_w.check(gctx(reading(front_cm=40.0)))
    assert at_warn is not None and at_warn.action is TripAction.ADVISORY
    guard_l = ProximityGuard(warn_cm=40.0, land_cm=25.0)
    at_land = guard_l.check(gctx(reading(front_cm=25.0)))
    assert at_land is not None and at_land.action is TripAction.LAND_THIS


def test_proximity_ladder_ordering_land_beats_advisory():
    """The LADDER ORDERING gate: when one direction is in the warn band AND
    another is at the hard-stop, the guard returns the STRONGER LAND_THIS — the
    closest/critical reading wins, an advisory never masks a land."""
    guard = ProximityGuard(warn_cm=40.0, land_cm=25.0)
    # front 35 (advisory band) + left 20 (hard stop) -> LAND_THIS on left.
    trip = guard.check(gctx(reading(front_cm=35.0, left_cm=20.0)))
    assert trip is not None and trip.action is TripAction.LAND_THIS
    assert "left" in trip.reason and "20" in trip.reason


def test_proximity_reports_the_closest_direction():
    guard = ProximityGuard(warn_cm=40.0, land_cm=25.0)
    # Two advisory-band returns: the CLOSEST (back 30) is named, not front 38.
    trip = guard.check(gctx(reading(front_cm=38.0, back_cm=30.0)))
    assert trip is not None and trip.action is TripAction.ADVISORY
    assert "back" in trip.reason and "30" in trip.reason


def test_proximity_advisory_is_edge_latched_and_rearms():
    """ADVISORY is edge-triggered: one per approach episode; a return to
    all-clear re-arms it. (A slow drift toward a wall must not spam the log.)"""
    guard = ProximityGuard(warn_cm=40.0, land_cm=25.0)
    first = guard.check(gctx(reading(front_cm=35.0)))
    assert first is not None and first.action is TripAction.ADVISORY
    # Still in the warn band -> latched, no repeat.
    assert guard.check(gctx(reading(front_cm=34.0))) is None
    assert guard.check(gctx(reading(front_cm=36.0))) is None
    # Back clear (beyond warn) re-arms...
    assert guard.check(gctx(reading(front_cm=90.0))) is None
    # ...so the next approach advises again.
    second = guard.check(gctx(reading(front_cm=35.0)))
    assert second is not None and second.action is TripAction.ADVISORY


def test_proximity_all_clear_rearms_via_none():
    """A reading that goes fully None (all directions clear) also re-arms the
    advisory latch (not only a far numeric reading)."""
    guard = ProximityGuard(warn_cm=40.0, land_cm=25.0)
    assert guard.check(gctx(reading(front_cm=35.0))).action is TripAction.ADVISORY
    assert guard.check(gctx(reading())) is None        # all-None re-arms
    assert guard.check(gctx(reading(front_cm=35.0))).action is TripAction.ADVISORY


def test_proximity_land_does_not_latch_on_advisory_state():
    """A direct jump from clear to the hard-stop (no advisory first) still
    LANDS — the land rung is independent of the advisory latch."""
    guard = ProximityGuard(warn_cm=40.0, land_cm=25.0)
    trip = guard.check(gctx(reading(front_cm=20.0)))
    assert trip is not None and trip.action is TripAction.LAND_THIS


def test_proximity_bad_reading_fails_safe_to_land():
    """A negative / non-finite range is a sensor bug — treated as LAND_THIS
    (fail SAFE), never read as 'clear' or as 'right on the lens'."""
    guard = ProximityGuard(warn_cm=40.0, land_cm=25.0)
    for bad in (-5.0, float("nan"), float("inf")):
        g = ProximityGuard(warn_cm=40.0, land_cm=25.0)
        trip = g.check(gctx(reading(front_cm=bad)))
        assert trip is not None and trip.action is TripAction.LAND_THIS, bad
        assert "not a valid distance" in trip.reason


def test_proximity_disabled_never_trips():
    """enabled=False -> a no-op even with an obstacle inside the hard-stop."""
    guard = ProximityGuard(warn_cm=40.0, land_cm=25.0, enabled=False)
    assert guard.check(gctx(reading(front_cm=5.0))) is None


# ============================================================
# 2. Constructor validation — bad thresholds die loudly
# ============================================================
@pytest.mark.parametrize("factory", [
    lambda: ProximityGuard(warn_cm=0.0, land_cm=-1.0),       # warn must be > 0
    lambda: ProximityGuard(warn_cm=float("nan"), land_cm=10.0),
    lambda: ProximityGuard(warn_cm=40.0, land_cm=0.0),       # land must be > 0
    lambda: ProximityGuard(warn_cm=25.0, land_cm=40.0),      # land >= warn
    lambda: ProximityGuard(warn_cm=40.0, land_cm=40.0),      # land == warn
    lambda: ProximityGuard(warn_cm=40.0, land_cm=float("inf")),
])
def test_proximity_constructor_rejects_bad_thresholds(factory):
    with pytest.raises(ValueError):
        factory()


def test_proximity_constructor_land_ge_warn_message():
    with pytest.raises(ValueError, match="land_cm < warn_cm"):
        ProximityGuard(warn_cm=25.0, land_cm=40.0)


# ============================================================
# 3. Config validation — proximity_* loud
# ============================================================
def _cfg(**guards) -> dict:
    return {
        "profile": "mock",
        "flight_backend": "mock",
        "frame_backend": "none",
        "detector": {"backend": "none"},
        "drones": [{"id": "alpha", "phases": ["takeoff_demo"]}],
        "guards": guards,
    }


def test_config_proximity_defaults(write_config):
    cfg = load_config(write_config(_cfg()))
    assert cfg.guards.proximity_enable is False
    assert cfg.guards.proximity_warn_cm == 40.0
    assert cfg.guards.proximity_land_cm == 25.0


def test_config_proximity_accepts_valid(write_config):
    cfg = load_config(write_config(_cfg(
        proximity_enable=True, proximity_warn_cm=50.0, proximity_land_cm=30.0)))
    assert cfg.guards.proximity_enable is True
    assert cfg.guards.proximity_warn_cm == 50.0


def test_config_proximity_land_ge_warn_is_loud(write_config):
    with pytest.raises(ConfigError, match="proximity_land_cm"):
        load_config(write_config(_cfg(proximity_warn_cm=20.0,
                                      proximity_land_cm=30.0)))


def test_config_proximity_negative_is_loud(write_config):
    with pytest.raises(ConfigError, match="proximity_warn_cm"):
        load_config(write_config(_cfg(proximity_warn_cm=-1.0)))


def test_config_proximity_nonfinite_is_loud(write_config):
    with pytest.raises(ConfigError, match="proximity_land_cm"):
        load_config(write_config(_cfg(proximity_land_cm=0.0)))


def test_config_proximity_enable_must_be_bool(write_config):
    with pytest.raises(ConfigError, match="proximity_enable"):
        load_config(write_config(_cfg(proximity_enable="yes")))


def test_config_proximity_validated_even_when_disabled(write_config):
    """A bad threshold is caught on the ground even with the guard OFF — so a
    later enable can't surface a latent misconfig mid-flight."""
    with pytest.raises(ConfigError, match="proximity_land_cm"):
        load_config(write_config(_cfg(proximity_enable=False,
                                      proximity_warn_cm=20.0,
                                      proximity_land_cm=30.0)))


# ============================================================
# 4. End-to-end — near obstacle fires the guard, drone safes down clean,
#    the sibling continues (the deconfliction e2e shape)
# ============================================================
def test_proximity_lands_this_drone_clean_end_to_end(run_dir):
    """A near-obstacle IR reading -> ProximityGuard LAND_THIS -> the agent
    lands CLEAN (DONE, no emergency_land) through the normal break path."""
    adapter = MockAdapter("alpha")
    near = reading(front_cm=18.0)               # inside the 25 cm hard-stop
    with EventLog(run_dir) as events:
        agent = DroneAgent(
            "alpha", adapter, [TakeoffDemo()], events,
            guards=[ProximityGuard(warn_cm=40.0, land_cm=25.0)],
            proximity_fn=ObstacleAfterTakeoff(adapter, near))
        run_agent(agent)

    assert agent.state is AgentState.DONE                  # clean, not FAILED
    assert "emergency_land" not in names(adapter.calls)    # ZERO emergencies
    assert "takeoff" in names(adapter.calls)               # flew first
    assert names(adapter.calls)[-1] == "land"
    reason = agent.status()["stopped_reason"]
    assert "ProximityGuard" in reason and "front" in reason and "18" in reason
    trips = guard_trips_of(run_dir)
    assert [t["data"]["action"] for t in trips] == ["LAND_THIS"]
    assert agent.failure is None


def test_proximity_does_not_trip_when_lane_clear_end_to_end(run_dir):
    """Control: with the guard wired but the lane CLEAR, the mission flies to
    completion untouched (no proximity trips)."""
    adapter = MockAdapter("alpha")
    clear = reading()                           # all directions clear
    with EventLog(run_dir) as events:
        agent = DroneAgent(
            "alpha", adapter, [TakeoffDemo()], events,
            guards=[ProximityGuard(warn_cm=40.0, land_cm=25.0)],
            proximity_fn=lambda: clear)
        run_agent(agent)

    assert agent.state is AgentState.DONE
    assert guard_trips_of(run_dir) == []
    assert agent.failure is None


def test_proximity_lands_one_drone_others_continue(run_dir):
    """The swarm gate: one drone's IR sees an obstacle (LAND_THIS, a PER-DRONE
    trip) and safes itself down clean, while a sibling with a clear lane keeps
    flying to its own clean finish — a per-drone proximity land must NOT take
    the swarm down (only LAND_ALL does)."""
    alpha = MockAdapter("alpha")
    bravo = MockAdapter("bravo")
    near = reading(front_cm=15.0)
    with EventLog(run_dir) as events:
        agents = [
            # alpha trips AFTER takeoff (the obstacle appears once airborne);
            # bravo's lane is clear and it runs its WaitForever until the
            # mission budget winds it down clean.
            DroneAgent("alpha", alpha, [WaitForeverPhase(0.02)], events,
                       guards=[ProximityGuard(warn_cm=40.0, land_cm=25.0)],
                       proximity_fn=ObstacleAfterTakeoff(alpha, near)),
            DroneAgent("bravo", bravo, [WaitForeverPhase(0.02)], events,
                       guards=[ProximityGuard(warn_cm=40.0, land_cm=25.0)],
                       proximity_fn=lambda: reading()),
        ]
        orch = Orchestrator(agents, events, run_dir, budget_s=2.0,
                            heartbeat_period_s=0.02)
        code = asyncio.run(orch.run())

    assert code == 0                                       # clean overall
    for agent, adapter in ((agents[0], alpha), (agents[1], bravo)):
        assert agent.state is AgentState.DONE
        assert names(adapter.calls)[-2:] == ["land", "disconnect"]
        assert "emergency_land" not in names(adapter.calls)
    # alpha stopped on the proximity guard; bravo on the budget — NOT the guard.
    assert "ProximityGuard" in agents[0].status()["stopped_reason"]
    assert "ProximityGuard" not in (agents[1].status()["stopped_reason"] or "")
    alpha_trips = [t for t in guard_trips_of(run_dir)
                   if t["drone"] == "alpha"
                   and t["data"]["guard"] == "ProximityGuard"]
    assert alpha_trips and alpha_trips[0]["data"]["action"] == "LAND_THIS"
    bravo_trips = [t for t in guard_trips_of(run_dir)
                   if t["drone"] == "bravo"
                   and t["data"]["guard"] == "ProximityGuard"]
    assert bravo_trips == []                               # bravo never tripped


def test_proximity_obstacle_on_the_ground_refuses_takeoff(run_dir):
    """An obstacle inside the hard-stop BEFORE takeoff trips LAND_THIS on the
    first guard pass — the drone never leaves the ground (no takeoff, nothing
    to land), DONE clean. A guard that fires while grounded must not somehow
    arm the aircraft."""
    adapter = MockAdapter("alpha")
    near = reading(front_cm=10.0)
    with EventLog(run_dir) as events:
        agent = DroneAgent(
            "alpha", adapter, [TakeoffDemo()], events,
            guards=[ProximityGuard(warn_cm=40.0, land_cm=25.0)],
            proximity_fn=lambda: near)
        run_agent(agent)

    assert agent.state is AgentState.DONE
    assert "takeoff" not in names(adapter.calls)           # never flew
    assert "emergency_land" not in names(adapter.calls)
    assert agent.failure is None
    assert "ProximityGuard" in agent.status()["stopped_reason"]


def test_agent_rejects_non_callable_proximity_fn(run_dir):
    with EventLog(run_dir) as events:
        with pytest.raises(ValueError, match="proximity_fn"):
            DroneAgent("alpha", MockAdapter("alpha"), [TakeoffDemo()],
                       events, proximity_fn=42)


# ============================================================
# 5. main wiring — _build_guards adds ProximityGuard only when enabled;
#    _build_proximity_fn wires the synthetic feed
# ============================================================
def test_build_guards_omits_proximity_when_disabled(write_config):
    from finals.config import load_config
    from finals.main import _build_guards
    cfg = load_config(write_config(_cfg()))                # proximity off
    guards = _build_guards(cfg, cfg.drones[0])
    assert not any(isinstance(g, ProximityGuard) for g in guards)


def test_build_guards_adds_proximity_when_enabled(write_config):
    from finals.config import load_config
    from finals.main import _build_guards
    cfg = load_config(write_config(_cfg(
        proximity_enable=True, proximity_warn_cm=45.0, proximity_land_cm=30.0)))
    guards = _build_guards(cfg, cfg.drones[0])
    prox = [g for g in guards if isinstance(g, ProximityGuard)]
    assert len(prox) == 1
    # The wired guard carries the configured thresholds (a clear lane stays
    # quiet, a 30 cm obstacle lands, a 45 cm one advises).
    g = prox[0]
    assert g.check(gctx(reading(front_cm=50.0))) is None
    assert g.check(gctx(reading(front_cm=30.0))).action is TripAction.LAND_THIS


def test_build_proximity_fn_disabled_is_none(write_config):
    from finals.config import load_config
    from finals.main import _build_proximity_fn
    cfg = load_config(write_config(_cfg()))                # proximity off
    assert _build_proximity_fn(cfg, cfg.drones[0]) is None


def test_build_proximity_fn_enabled_wires_synthetic_feed(write_config):
    """When enabled, the per-drone proximity_fn is the synthetic sensor's read
    — and it returns None (no live IR), so the guard SKIPS (the live read is
    the onsite gate, never a fabricated clear)."""
    from finals.config import load_config
    from finals.main import _build_proximity_fn
    cfg = load_config(write_config(_cfg(proximity_enable=True)))
    fn = _build_proximity_fn(cfg, cfg.drones[0])
    assert callable(fn)
    assert fn() is None                                    # honest: no live IR yet
