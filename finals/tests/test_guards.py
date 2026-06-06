"""finals.guards — the S5 safety cluster, thoroughly smoked.

Covers the session gates: every concrete guard trips in a unit test; a
RAISING guard is itself a trip (never a silent disable); BatteryGuard ends
a mission with a CLEAN land (DONE, zero emergency_land); the
TelemetryWatchdog policy layer fires before the agent's 5 s mechanism
backstop; SafetyController trip idempotence + the bounded retry ladder +
the serialized landing slot (and that emergency_land NEVER waits for it);
the AbortListener with injected stdin lands everything orderly; the
MissionClockGuard puts drones DOWN before the budget, not at it.

No pytest-asyncio: coroutines are driven with asyncio.run() inside sync
tests (suite convention). EventLogs are real (tmp run dirs); ladder tests
assert ATTEMPT COUNTS, never wall time.
"""
from __future__ import annotations

import asyncio
import os
import threading
import time

import pytest

from finals.errors import FlightError, FlightTimeout
from finals.events import EventLog, read_events
from finals.flight.mock_adapter import MockAdapter
from finals.guards import (AbortListener, BatteryGuard, GeofenceLite, Guard,
                           GuardContext, LoopOverrunGuard, MissionClockGuard,
                           PhaseTimeout, SafetyController, TelemetryWatchdog,
                           Trip, TripAction, VideoWatchdog, evaluate_guards)
from finals.mission.agent import AgentState, DroneAgent
from finals.mission.orchestrator import Orchestrator
from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases.takeoff_demo import TakeoffDemo
from finals.types import Action, Done, Land, Takeoff, Telemetry, Wait

HAPPY_CALLS = (["connect", "takeoff", "hover"] + ["move", "rotate"] * 4
               + ["land"])


class FakeClock:
    """Deterministic monotonic source — tests set .t explicitly."""

    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


# ---------------- helpers ----------------
def names(calls):
    return [c[0] for c in calls]


def events_of(run_dir: str):
    return list(read_events(os.path.join(run_dir, "mission.jsonl")))


def guard_trips_of(run_dir: str):
    return [e for e in events_of(run_dir) if e["event"] == "guard_trip"]


def gctx(drone_id="alpha", now=100.0, mission_elapsed_s=10.0, **kw):
    return GuardContext(drone_id=drone_id, now=now,
                        mission_elapsed_s=mission_elapsed_s, **kw)


def telemetry(ts: float, **kw) -> Telemetry:
    return Telemetry(ts=ts, **kw)


def run_agent(agent, *, budget_s: float = 30.0,
              deadline: float = None) -> None:
    async def go():
        await agent.run(
            deadline=(time.monotonic() + budget_s
                      if deadline is None else deadline),
            stop_event=asyncio.Event())

    asyncio.run(go())


@pytest.fixture
def run_dir(tmp_path):
    return str(tmp_path)


# ---------------- test phases / guards (NOT registered) ----------------
class ScriptedPhase(MissionPhase):
    name = "scripted"

    def __init__(self, actions):
        self._actions = list(actions)
        self._idx = 0

    def step(self, ctx: AgentContext) -> Action:
        if self._idx >= len(self._actions):
            return Done("scripted phase exhausted")
        action = self._actions[self._idx]
        self._idx += 1
        return action


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


class TripOnNth(Guard):
    """Scripted guard: fires `action` on exactly the nth check()."""

    def __init__(self, n: int, action: TripAction, reason: str = "scripted"):
        self._n = n
        self._action = action
        self._reason = reason
        self.checks = 0

    def check(self, gctx_: GuardContext) -> Trip | None:
        self.checks += 1
        if self.checks == self._n:
            return self._trip(self._action,
                              f"{gctx_.drone_id}: {self._reason} "
                              f"(check #{self.checks})")
        return None


class HoldFirstN(Guard):
    """HOLD_THIS for the first n checks, then quiet."""

    def __init__(self, n: int):
        self._n = n
        self.checks = 0

    def check(self, gctx_: GuardContext) -> Trip | None:
        self.checks += 1
        if self.checks <= self._n:
            return self._trip(
                TripAction.HOLD_THIS,
                f"{gctx_.drone_id}: scripted hold {self.checks}/{self._n}")
        return None


class RaiseOnNth(Guard):
    """The buggy-guard class: raises on exactly the nth check()."""

    def __init__(self, n: int = 1):
        self._n = n
        self.checks = 0

    def check(self, gctx_: GuardContext) -> Trip | None:
        self.checks += 1
        if self.checks == self._n:
            raise RuntimeError("kaboom: guard bug")
        return None


# ============================================================
# 1. Unit trips — EVERY concrete guard (gate 1)
# ============================================================
def test_telemetry_watchdog_trips_on_stale():
    guard = TelemetryWatchdog(stale_s=2.0)
    assert guard.check(gctx(telemetry=None)) is None       # swarm ctx: skip
    fresh = gctx(now=11.0, telemetry=telemetry(ts=10.0))
    assert guard.check(fresh) is None
    trip = guard.check(gctx(now=13.5, telemetry=telemetry(ts=10.0)))
    assert trip is not None and trip.action is TripAction.LAND_THIS
    assert trip.guard == "TelemetryWatchdog"
    # The message bar: WHAT, WHICH drone, MEASURED vs LIMIT, CHECK.
    assert "alpha" in trip.reason and "3.5" in trip.reason
    assert "2.0" in trip.reason and "check" in trip.reason


def test_video_watchdog_trips_and_edge_latches():
    guard = VideoWatchdog(stale_s=2.0)
    assert guard.check(gctx(now=100.0, last_frame_ts=99.5)) is None
    trip = guard.check(gctx(now=103.0, last_frame_ts=99.5))
    assert trip is not None and trip.action is TripAction.DEGRADE_DETECTION
    assert "3.5" in trip.reason and "2.0" in trip.reason
    # Latched: same stale episode reports ONCE.
    assert guard.check(gctx(now=104.0, last_frame_ts=99.5)) is None
    # A fresh frame re-arms; the next stale episode fires again.
    assert guard.check(gctx(now=104.0, last_frame_ts=103.9)) is None
    trip2 = guard.check(gctx(now=110.0, last_frame_ts=103.9))
    assert trip2 is not None and trip2.action is TripAction.DEGRADE_DETECTION


def test_video_watchdog_no_frame_ever_anchors_on_first_check():
    guard = VideoWatchdog(stale_s=2.0)
    assert guard.check(gctx(now=100.0, last_frame_ts=None)) is None  # anchor
    trip = guard.check(gctx(now=103.0, last_frame_ts=None))
    assert trip is not None and trip.action is TripAction.DEGRADE_DETECTION
    assert "EVER" in trip.reason
    assert guard.check(gctx(now=104.0, last_frame_ts=None)) is None  # latched


def test_battery_guard_warn_floor_and_unknown():
    guard = BatteryGuard(floor_pct=20.0, warn_pct=30.0)
    assert guard.check(gctx(telemetry=None)) is None
    assert guard.check(gctx(telemetry=telemetry(ts=99.0, battery_pct=50.0))) is None
    warn = guard.check(gctx(telemetry=telemetry(ts=99.0, battery_pct=25.0)))
    assert warn is not None and warn.action is TripAction.ADVISORY
    assert "25" in warn.reason and "30" in warn.reason
    # Warn is one-shot (batteries do not recover mid-flight).
    assert guard.check(gctx(telemetry=telemetry(ts=99.0, battery_pct=24.0))) is None
    floor = guard.check(gctx(telemetry=telemetry(ts=99.0, battery_pct=20.0)))
    assert floor is not None and floor.action is TripAction.LAND_THIS
    assert "20" in floor.reason and "alpha" in floor.reason

    unknown = BatteryGuard(floor_pct=20.0, warn_pct=30.0)
    adv = unknown.check(gctx(telemetry=telemetry(ts=99.0)))   # battery None
    assert adv is not None and adv.action is TripAction.ADVISORY
    assert "UNKNOWN" in adv.reason
    assert unknown.check(gctx(telemetry=telemetry(ts=99.0))) is None  # once


def test_mission_clock_guard_trips_once_at_reserve():
    guard = MissionClockGuard(budget_s=30.0, landing_reserve_s=20.0)
    assert guard.check(gctx(drone_id="mission", mission_elapsed_s=9.9)) is None
    trip = guard.check(gctx(drone_id="mission", mission_elapsed_s=10.0))
    assert trip is not None and trip.action is TripAction.LAND_ALL
    assert "10.0" in trip.reason and "30" in trip.reason and "20" in trip.reason
    # One-shot: the stop it causes is latched anyway.
    assert guard.check(gctx(drone_id="mission", mission_elapsed_s=11.0)) is None


def test_loop_overrun_guard_two_stage_ladder_and_reset():
    guard = LoopOverrunGuard(period_s=1.0, factor=2.0, n_ticks=2)

    def check(latency):
        return guard.check(gctx(drone_id="mission", tick_latency_s=latency))

    assert check(None) is None                    # no measurement yet
    assert check(1.0) is None                     # healthy
    assert check(5.0) is None                     # overrun 1 < n_ticks
    stage1 = check(5.0)                           # overrun 2 == n_ticks
    assert stage1 is not None and stage1.action is TripAction.DEGRADE_DETECTION
    assert check(5.0) is None                     # 3: between stages
    stage2 = check(5.0)                           # 4 == 2 x n_ticks
    assert stage2 is not None and stage2.action is TripAction.LAND_ALL
    assert check(5.0) is None                     # latched
    assert check(1.0) is None                     # healthy tick resets...
    assert check(5.0) is None
    again = check(5.0)                            # ...so the ladder re-arms
    assert again is not None and again.action is TripAction.DEGRADE_DETECTION


def test_geofence_lite_is_advisory_and_edge_latched():
    guard = GeofenceLite(radius_m=10.0, alt_max_m=3.0)
    assert guard.check(gctx(telemetry=telemetry(ts=99.0))) is None  # no pos
    inside = telemetry(ts=99.0, position_m=(1.0, 1.0, 1.0))
    assert guard.check(gctx(telemetry=inside)) is None
    out_r = telemetry(ts=99.0, position_m=(20.0, 0.0, 1.0))
    trip = guard.check(gctx(telemetry=out_r))
    assert trip is not None and trip.action is TripAction.ADVISORY
    assert "20.0" in trip.reason and "10.0" in trip.reason
    assert "ADVISORY" in trip.reason              # never acted on
    assert guard.check(gctx(telemetry=out_r)) is None     # latched
    assert guard.check(gctx(telemetry=inside)) is None    # re-arms
    out_a = telemetry(ts=99.0, position_m=(0.0, 0.0, 5.0))
    trip2 = guard.check(gctx(telemetry=out_a))
    assert trip2 is not None and "altitude" in trip2.reason


def test_phase_timeout_trips_past_budget():
    guard = PhaseTimeout(timeout_s=10.0)
    assert guard.check(gctx(phase_elapsed_s=None)) is None   # not entered
    assert guard.check(gctx(phase_name="takeoff_demo",
                            phase_elapsed_s=5.0)) is None
    trip = guard.check(gctx(phase_name="takeoff_demo",
                            phase_elapsed_s=11.0))
    assert trip is not None and trip.action is TripAction.LAND_THIS
    assert "takeoff_demo" in trip.reason
    assert "11.0" in trip.reason and "10.0" in trip.reason


# ============================================================
# 2. evaluate_guards — a raising guard IS a trip (gate 2)
# ============================================================
def test_raising_guard_is_a_trip_and_others_still_run(capsys):
    raising = RaiseOnNth(1)
    quiet_then_advisory = TripOnNth(1, TripAction.ADVISORY)
    trips = evaluate_guards([raising, quiet_then_advisory], gctx())
    assert len(trips) == 2
    assert trips[0].guard == "RaiseOnNth"
    assert trips[0].action is TripAction.LAND_THIS        # default error_action
    assert "guard bug" in trips[0].reason and "check" in trips[0].reason
    assert trips[1].action is TripAction.ADVISORY         # still evaluated
    err = capsys.readouterr().err
    assert "RuntimeError" in err and "kaboom" in err      # full traceback


def test_raising_guard_uses_callers_error_action(capsys):
    trips = evaluate_guards([RaiseOnNth(1)], gctx(drone_id="mission"),
                            error_action=TripAction.LAND_ALL)
    assert trips[0].action is TripAction.LAND_ALL
    assert "kaboom" in capsys.readouterr().err


def test_guard_returning_garbage_is_a_trip(capsys):
    class Garbage(Guard):
        def check(self, gctx_):
            return "north"                                 # not a Trip

    trips = evaluate_guards([Garbage()], gctx())
    assert len(trips) == 1 and trips[0].action is TripAction.LAND_THIS
    assert "north" in trips[0].reason
    assert "Garbage" in capsys.readouterr().err


# ============================================================
# 3. Constructor validation — bad thresholds die loudly
# ============================================================
@pytest.mark.parametrize("factory", [
    lambda: TelemetryWatchdog(stale_s=0),
    lambda: TelemetryWatchdog(stale_s=float("nan")),
    lambda: VideoWatchdog(stale_s=-1.0),
    lambda: BatteryGuard(floor_pct=50.0, warn_pct=30.0),   # warn under floor
    lambda: BatteryGuard(floor_pct=-1.0, warn_pct=30.0),
    lambda: BatteryGuard(floor_pct=20.0, warn_pct=200.0),
    lambda: MissionClockGuard(budget_s=10.0, landing_reserve_s=10.0),
    lambda: MissionClockGuard(budget_s=0.0, landing_reserve_s=1.0),
    lambda: LoopOverrunGuard(period_s=1.0, factor=1.0),    # trips when healthy
    lambda: LoopOverrunGuard(period_s=1.0, factor=2.0, n_ticks=0),
    lambda: GeofenceLite(radius_m=0.0),
    lambda: GeofenceLite(radius_m=10.0, alt_max_m=float("inf")),
    lambda: PhaseTimeout(timeout_s=float("inf")),
])
def test_guard_constructors_reject_bad_thresholds(factory):
    with pytest.raises(ValueError):
        factory()


def test_safety_controller_rejects_bad_params(run_dir):
    with EventLog(run_dir) as events:
        with pytest.raises(ValueError, match="never retry"):
            SafetyController(events, land_retry_period_s=2.0,
                             land_retry_window_s=1.0)
        with pytest.raises(ValueError, match="land_retry_period_s"):
            SafetyController(events, land_retry_period_s=0.0)
        with pytest.raises(ValueError, match="slot_wait_s"):
            SafetyController(events, slot_wait_s=float("nan"))


def test_agent_and_orchestrator_reject_bad_guard_wiring(run_dir):
    with EventLog(run_dir) as events:
        with pytest.raises(ValueError, match="Guard instances"):
            DroneAgent("alpha", MockAdapter("alpha"), [TakeoffDemo()],
                       events, guards=[42])
        with pytest.raises(ValueError, match="SafetyController"):
            DroneAgent("alpha", MockAdapter("alpha"), [TakeoffDemo()],
                       events, safety="not a controller")
        with pytest.raises(ValueError, match="hold_poll_s"):
            DroneAgent("alpha", MockAdapter("alpha"), [TakeoffDemo()],
                       events, hold_poll_s=0.0)
        agent = DroneAgent("alpha", MockAdapter("alpha"), [TakeoffDemo()],
                           events)
        with pytest.raises(ValueError, match="Guard instances"):
            Orchestrator([agent], events, run_dir, budget_s=10.0,
                         swarm_guards=["not a guard"])
        with pytest.raises(ValueError, match="threading.Event"):
            Orchestrator([agent], events, run_dir, budget_s=10.0,
                         abort_event=42)


# ============================================================
# 4. BatteryGuard end-to-end — forced CLEAN land at the floor (gate 3)
# ============================================================
def test_battery_floor_forces_clean_land_not_emergency(run_dir):
    adapter = MockAdapter("alpha", battery_decay_pct_per_cmd=15.0)
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo()], events,
                           guards=[BatteryGuard(floor_pct=20.0,
                                                warn_pct=30.0)])
        run_agent(agent)

    # 100 -> takeoff 85 -> hover 70 -> move 55 -> rotate 40 -> move 25
    # (warn fires) -> rotate 10 -> floor trip -> CLEAN land -> DONE.
    assert agent.state is AgentState.DONE
    assert "emergency_land" not in names(adapter.calls)    # ZERO emergencies
    assert names(adapter.calls) == [
        "connect", "takeoff", "hover", "move", "rotate", "move", "rotate",
        "land"]
    reason = agent.status()["stopped_reason"]
    assert "BatteryGuard" in reason and "10" in reason and "20" in reason
    trips = guard_trips_of(run_dir)
    assert [t["data"]["action"] for t in trips] == ["ADVISORY", "LAND_THIS"]
    assert all(t["data"]["guard"] == "BatteryGuard" for t in trips)
    assert agent.failure is None                           # NOT a failure


# ============================================================
# 5. TelemetryWatchdog layering — guard fires BEFORE the backstop (gate 4)
# ============================================================
def test_telemetry_guard_fires_before_agent_backstop(run_dir):
    clock = FakeClock(100.0)

    class BumpOnTakeoff(MockAdapter):
        """Completing takeoff 'takes' 3 s on the shared fake clock — the
        frozen telemetry then ages to 2.5 s: inside the guard's 2 s limit
        but safely under the agent's 5 s SensorTimeout backstop."""

        async def takeoff(self, *a, **kw):
            await super().takeoff(*a, **kw)
            clock.t += 3.0

    adapter = BumpOnTakeoff("alpha", clock=clock,
                            freeze_telemetry_after_s=0.5)
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo()], events,
                           telemetry_stale_s=5.0, clock=clock,
                           guards=[TelemetryWatchdog(stale_s=2.0)])
        run_agent(agent, deadline=clock.t + 1000.0)

    # The guard's LAND_THIS is a CLEAN land — assert NO emergency anywhere.
    assert agent.state is AgentState.DONE
    assert names(adapter.calls) == ["connect", "takeoff", "land"]
    assert "emergency_land" not in names(adapter.calls)
    reason = agent.status()["stopped_reason"]
    assert "TelemetryWatchdog" in reason
    assert "2.5" in reason and "2.0" in reason             # measured vs limit
    assert agent.failure is None


def test_agent_backstop_still_wins_when_no_guard_wired(run_dir):
    """The layering contrast: same staleness WITHOUT the guard -> the 5 s
    backstop path (SensorTimeout -> FAILED + emergency) is unchanged."""
    clock = FakeClock(100.0)

    class BumpOnTakeoff(MockAdapter):
        async def takeoff(self, *a, **kw):
            await super().takeoff(*a, **kw)
            clock.t += 10.0                                # past the backstop

    adapter = BumpOnTakeoff("alpha", clock=clock,
                            freeze_telemetry_after_s=0.5)
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo()], events,
                           telemetry_stale_s=5.0, clock=clock)
        run_agent(agent, deadline=clock.t + 1000.0)

    assert agent.state is AgentState.FAILED
    assert names(adapter.calls).count("emergency_land") == 1


# ============================================================
# 6. Agent trip mapping — HOLD, LAND_ALL, raising guard (gate 10)
# ============================================================
def test_hold_this_skips_phase_steps_then_resumes(run_dir):
    adapter = MockAdapter("alpha")
    hold = HoldFirstN(3)
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo(legs=0,
                                                          hover_s=0.0)],
                           events, guards=[hold], hold_poll_s=0.01)
        run_agent(agent)

    # 3 held iterations stepped NO phase (no flight command before takeoff),
    # then the phase ran to completion untouched.
    assert agent.state is AgentState.DONE
    assert names(adapter.calls) == ["connect", "takeoff", "hover", "land"]
    holds = [t for t in guard_trips_of(run_dir)
             if t["data"]["action"] == "HOLD_THIS"]
    assert len(holds) == 3
    assert hold.checks > 3                                 # released + resumed


def test_agent_land_all_trip_stops_the_sibling(run_dir):
    alpha = MockAdapter("alpha")
    bravo = MockAdapter("bravo")
    with EventLog(run_dir) as events:
        agents = [
            # Short Waits: alpha must come back from its Wait to reach the
            # tripping 3rd check; bravo wakes on the stop event regardless.
            DroneAgent("alpha", alpha, [WaitForeverPhase(0.02)], events,
                       guards=[TripOnNth(3, TripAction.LAND_ALL,
                                         "scripted land-all")]),
            DroneAgent("bravo", bravo, [WaitForeverPhase(0.02)], events),
        ]
        orch = Orchestrator(agents, events, run_dir, budget_s=30.0,
                            heartbeat_period_s=0.02)
        code = asyncio.run(orch.run())

    assert code == 0                                       # clean stop
    for agent, adapter in ((agents[0], alpha), (agents[1], bravo)):
        assert agent.state is AgentState.DONE
        assert names(adapter.calls)[-2:] == ["land", "disconnect"]
        assert "emergency_land" not in names(adapter.calls)
    assert "LAND_ALL" in agents[0].status()["stopped_reason"]
    kinds = [e["event"] for e in events_of(run_dir)]
    assert "guard_trip" in kinds
    assert "stop_signalled" in kinds                       # orchestrator saw it
    assert "budget_expired" not in kinds


def test_raising_guard_lands_the_drone_clean(run_dir, capsys):
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo()], events,
                           guards=[RaiseOnNth(2)])
        run_agent(agent)

    assert agent.state is AgentState.DONE                  # clean, not FAILED
    assert names(adapter.calls) == ["connect", "takeoff", "land"]
    assert "RaiseOnNth" in agent.status()["stopped_reason"]
    assert "kaboom" in capsys.readouterr().err             # traceback logged


# ============================================================
# 7. SafetyController — idempotence, ladder, escalation (gate 5)
# ============================================================
def test_safety_trip_twice_runs_one_landing_sequence(run_dir):
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        safety = SafetyController(events, land_retry_period_s=0.02,
                                  land_retry_window_s=0.05)

        async def go():
            await adapter.connect()
            await adapter.takeoff(80)
            await safety.trip(adapter, "alpha", "guard X: battery floor")
            await safety.trip(adapter, "alpha", "guard Y: re-trip race")

        asyncio.run(go())

    assert names(adapter.calls).count("land") == 1         # ONE sequence
    kinds = [e["event"] for e in events_of(run_dir)]
    assert kinds.count("safety_trip") == 1
    assert kinds.count("safety_retrip_ignored") == 1


def test_landing_ladder_retries_then_escalates(run_dir):
    adapter = MockAdapter("alpha", fail_on={"land": FlightError(
        "alpha: land() failed — scripted link drop — check the scenario")})
    with EventLog(run_dir) as events:
        # FakeClock pins the COUNT bound: the wall-clock window (which also
        # bounds the ladder — see the hang-mode test below) never advances,
        # so exactly ceil(window/period) attempts run, deterministically.
        safety = SafetyController(events, land_retry_period_s=0.02,
                                  land_retry_window_s=0.05,
                                  command_timeout_s=1.0,
                                  clock=FakeClock(100.0))
        assert safety.land_attempts == 3                   # ceil(0.05/0.02)

        async def go():
            await adapter.connect()
            await adapter.takeoff(80)
            with pytest.raises(FlightError, match="OPERATOR ALARM"):
                await safety.land(adapter, "alpha")
            # Escalation is latched: re-entry raises IMMEDIATELY, no second
            # 30 s ladder burned on a dead link.
            with pytest.raises(FlightError, match="ESCALATED"):
                await safety.land(adapter, "alpha")

        asyncio.run(go())

    assert names(adapter.calls).count("land") == 3         # exactly the ladder
    escalations = [e for e in events_of(run_dir)
                   if e["event"] == "safety_escalation"]
    assert len(escalations) == 1
    assert escalations[0]["data"]["attempts"] == 3


def test_landing_ladder_wall_clock_bounds_the_hang_mode(run_dir):
    """The operator-alarm honesty bound: when each land attempt burns its
    full outer deadline (a hanging backend), the WALL CLOCK — not the
    attempt count — must end the ladder, or a '30 s' ladder alarms minutes
    late with the drone possibly still airborne."""
    clock = FakeClock(100.0)

    class SlowFailingLand(MockAdapter):
        """Every land attempt fails AND costs 20 fake-clock seconds."""

        async def land(self, timeout_s=30.0):
            clock.t += 20.0
            await super().land(timeout_s=timeout_s)        # raises (fail_on)

    adapter = SlowFailingLand("alpha", fail_on={"land": FlightError(
        "alpha: land() failed — scripted — check the scenario")})
    with EventLog(run_dir) as events:
        safety = SafetyController(events, land_retry_period_s=0.01,
                                  land_retry_window_s=30.0,
                                  command_timeout_s=1.0, clock=clock)
        assert safety.land_attempts == 3000                # count alone: 3000

        async def go():
            await adapter.connect()
            await adapter.takeoff(80)
            with pytest.raises(FlightError, match="OPERATOR ALARM"):
                await safety.land(adapter, "alpha")

        asyncio.run(go())

    # Attempt 1 ends at +20 s (< 30 window -> retry); attempt 2 ends at
    # +40 s (>= 30 -> ladder ends). NOT 30 attempts.
    assert names(adapter.calls).count("land") == 2
    esc = [e for e in events_of(run_dir)
           if e["event"] == "safety_escalation"][0]["data"]
    assert esc["attempts"] == 2
    assert esc["elapsed_s"] == pytest.approx(40.0)         # honest forensics


def test_landing_ladder_recovers_on_a_later_attempt(run_dir):
    adapter = MockAdapter("alpha", fail_at="land:1")       # only #1 fails
    with EventLog(run_dir) as events:
        safety = SafetyController(events, land_retry_period_s=0.02,
                                  land_retry_window_s=0.1,
                                  command_timeout_s=1.0)

        async def go():
            await adapter.connect()
            await adapter.takeoff(80)
            await safety.land(adapter, "alpha")            # must NOT raise

        asyncio.run(go())

    assert names(adapter.calls).count("land") == 2
    assert not adapter.is_flying
    kinds = [e["event"] for e in events_of(run_dir)]
    assert "safety_land_recovered" in kinds
    assert "safety_escalation" not in kinds


# ============================================================
# 8. The landing slot — serialized descent; emergency never waits (gate 6)
# ============================================================
def test_landing_slot_serializes_concurrent_landings(run_dir):
    record = []

    class RecordingAdapter(MockAdapter):
        async def land(self, timeout_s=30.0):
            record.append((self.drone_id, "enter", time.perf_counter()))
            await super().land(timeout_s=timeout_s)
            record.append((self.drone_id, "exit", time.perf_counter()))

    alpha = RecordingAdapter("alpha", latency_s=0.05)
    bravo = RecordingAdapter("bravo", latency_s=0.05)
    with EventLog(run_dir) as events:
        safety = SafetyController(events, land_retry_period_s=0.02,
                                  land_retry_window_s=0.05)
        agents = [
            DroneAgent("alpha", alpha,
                       [ScriptedPhase([Takeoff(80), Land()])], events,
                       safety=safety),
            DroneAgent("bravo", bravo,
                       [ScriptedPhase([Takeoff(80), Land()])], events,
                       safety=safety),
        ]

        async def go():
            stop = asyncio.Event()
            deadline = time.monotonic() + 30.0
            await asyncio.gather(*(a.run(deadline=deadline, stop_event=stop)
                                   for a in agents))

        asyncio.run(go())

    assert all(a.state is AgentState.DONE for a in agents)
    windows = {}
    for drone_id, kind, t in record:
        windows.setdefault(drone_id, {})[kind] = t
    a, b = windows["alpha"], windows["bravo"]
    assert len(a) == 2 and len(b) == 2                     # one landing each
    # Serialized: the two descent windows must NOT overlap.
    assert a["exit"] <= b["enter"] or b["exit"] <= a["enter"], (
        f"landings overlapped: alpha={a} bravo={b}")


def test_emergency_land_never_waits_for_the_slot(run_dir):
    alpha = MockAdapter("alpha")
    bravo = MockAdapter("bravo")
    with EventLog(run_dir) as events:
        safety = SafetyController(events, land_retry_period_s=0.02,
                                  land_retry_window_s=0.05)

        async def go():
            await alpha.connect()
            await alpha.takeoff(80)
            await bravo.connect()
            await bravo.takeoff(80)
            alpha.latency_s = 0.3                  # slow DESCENT holds the slot
            land_task = asyncio.create_task(safety.land(alpha, "alpha"))
            await asyncio.sleep(0.05)              # alpha is inside its landing
            t0 = time.perf_counter()
            # The agent's safe-down path calls the adapter DIRECTLY — by
            # construction it cannot touch the slot. Pin that it completes
            # while the slot is still held.
            await bravo.emergency_land()
            elapsed = time.perf_counter() - t0
            await land_task
            return elapsed

        elapsed = asyncio.run(go())

    assert elapsed < 0.2, (
        f"emergency_land took {elapsed:.3f} s — it must never queue behind "
        f"the landing slot")
    assert not bravo.is_flying and not alpha.is_flying


def test_slot_wait_is_bounded_with_actionable_timeout(run_dir):
    alpha = MockAdapter("alpha")
    bravo = MockAdapter("bravo")
    with EventLog(run_dir) as events:
        safety = SafetyController(events, land_retry_period_s=0.02,
                                  land_retry_window_s=0.05,
                                  slot_wait_s=0.05)

        async def go():
            await alpha.connect()
            await alpha.takeoff(80)
            await bravo.connect()
            await bravo.takeoff(80)
            alpha.latency_s = 0.5                          # hog the slot
            land_task = asyncio.create_task(safety.land(alpha, "alpha"))
            await asyncio.sleep(0.05)
            with pytest.raises(FlightTimeout, match="landing slot"):
                await safety.land(bravo, "bravo")
            await land_task

        asyncio.run(go())


def test_safety_wired_happy_path_routes_land_through_safety(run_dir):
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        safety = SafetyController(events, land_retry_period_s=0.02,
                                  land_retry_window_s=0.05)
        agent = DroneAgent("alpha", adapter, [TakeoffDemo()], events,
                           safety=safety)
        run_agent(agent)

    assert agent.state is AgentState.DONE
    assert names(adapter.calls) == HAPPY_CALLS             # unchanged behavior
    land_starts = [e for e in events_of(run_dir)
                   if e["event"] == "action_start"
                   and e["data"]["action"] == "Land"]
    assert land_starts and land_starts[0]["data"]["route"] == "safety"


def test_guard_trip_with_safety_goes_through_latched_trip(run_dir):
    adapter = MockAdapter("alpha", battery_decay_pct_per_cmd=15.0)
    with EventLog(run_dir) as events:
        safety = SafetyController(events, land_retry_period_s=0.02,
                                  land_retry_window_s=0.05)
        agent = DroneAgent("alpha", adapter, [TakeoffDemo()], events,
                           guards=[BatteryGuard(floor_pct=20.0,
                                                warn_pct=30.0)],
                           safety=safety)
        run_agent(agent)

    assert agent.state is AgentState.DONE
    assert names(adapter.calls).count("land") == 1
    assert "emergency_land" not in names(adapter.calls)
    kinds = [e["event"] for e in events_of(run_dir)]
    assert "safety_trip" in kinds                          # the latched entry


# ============================================================
# 9. AbortListener — 'q' lands everything orderly (gate 7)
# ============================================================
class FakeAbortSource:
    """Injectable stdin: readline blocks on a gate, then yields the
    scripted lines, then EOF. The wait is BOUNDED (test hang guard)."""

    def __init__(self, lines=("q\n",)):
        self.gate = threading.Event()
        self._lines = list(lines)

    def readline(self) -> str:
        if not self.gate.wait(timeout=10.0):
            return ""                                      # bail out: EOF
        return self._lines.pop(0) if self._lines else ""


def test_abort_key_lands_all_drones_orderly(run_dir, capsys):
    src = FakeAbortSource()
    abort_evt = threading.Event()
    alpha = MockAdapter("alpha")
    bravo = MockAdapter("bravo")
    with EventLog(run_dir) as events:
        agents = [DroneAgent("alpha", alpha, [WaitForeverPhase()], events),
                  DroneAgent("bravo", bravo, [WaitForeverPhase()], events)]
        orch = Orchestrator(agents, events, run_dir, budget_s=30.0,
                            heartbeat_period_s=0.02, abort_event=abort_evt)
        listener = AbortListener(abort_evt, source=src,
                                 on_abort=orch.request_stop_threadsafe)

        async def release_q_when_airborne():
            for _ in range(500):                           # bounded poll
                if alpha.is_flying and bravo.is_flying:
                    break
                await asyncio.sleep(0.01)
            src.gate.set()                                 # the operator types 'q'

        async def go():
            listener.start()
            releaser = asyncio.get_running_loop().create_task(
                release_q_when_airborne())
            try:
                return await orch.run()
            finally:
                await releaser
                listener.stop()

        t0 = time.perf_counter()
        code = asyncio.run(go())
        elapsed = time.perf_counter() - t0

    assert code == 0                                       # orderly, clean
    assert elapsed < 10.0                                  # woke mid-Wait
    for agent, adapter in ((agents[0], alpha), (agents[1], bravo)):
        assert agent.state is AgentState.DONE
        assert names(adapter.calls)[-2:] == ["land", "disconnect"]
        assert "emergency_land" not in names(adapter.calls)
    kinds = [e["event"] for e in events_of(run_dir)]
    assert "operator_abort" in kinds
    captured = capsys.readouterr()
    assert "MISSION SUMMARY" in captured.out               # orderly summary
    assert "OPERATOR ABORT" in captured.err
    # The thread fired its one shot and ended — no daemon leak.
    assert not listener.is_alive()
    assert listener.stop() is True


def test_abort_wakeup_beats_the_tick_poll(run_dir):
    """Pins request_stop_threadsafe (the call_soon_threadsafe hook): with a
    LONG heartbeat period the per-tick abort poll alone would take ~5 s to
    notice the key — the prompt wakeup must land everything well inside
    one beat."""
    src = FakeAbortSource()
    abort_evt = threading.Event()
    alpha = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agents = [DroneAgent("alpha", alpha, [WaitForeverPhase()], events)]
        orch = Orchestrator(agents, events, run_dir, budget_s=30.0,
                            heartbeat_period_s=5.0, abort_event=abort_evt)
        listener = AbortListener(abort_evt, source=src,
                                 on_abort=orch.request_stop_threadsafe)

        async def release_q_when_airborne():
            for _ in range(500):                           # bounded poll
                if alpha.is_flying:
                    break
                await asyncio.sleep(0.01)
            src.gate.set()

        async def go():
            listener.start()
            releaser = asyncio.get_running_loop().create_task(
                release_q_when_airborne())
            try:
                return await orch.run()
            finally:
                await releaser
                listener.stop()

        t0 = time.perf_counter()
        code = asyncio.run(go())
        elapsed = time.perf_counter() - t0

    assert code == 0
    assert elapsed < 3.0            # NOT the 5 s beat: the wakeup hook fired
    assert agents[0].state is AgentState.DONE
    assert "operator_abort" in [e["event"] for e in events_of(run_dir)]


def test_abort_poll_alone_lands_all_without_the_wakeup_hook(run_dir):
    """The per-tick abort_event poll is the RELIABLE channel: with NO
    on_abort wakeup hook wired at all, the poll alone must still land
    everything within about a beat."""
    src = FakeAbortSource()
    abort_evt = threading.Event()
    alpha = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agents = [DroneAgent("alpha", alpha, [WaitForeverPhase()], events)]
        orch = Orchestrator(agents, events, run_dir, budget_s=30.0,
                            heartbeat_period_s=0.02, abort_event=abort_evt)
        listener = AbortListener(abort_evt, source=src)    # no on_abort

        async def release_q_when_airborne():
            for _ in range(500):                           # bounded poll
                if alpha.is_flying:
                    break
                await asyncio.sleep(0.01)
            src.gate.set()

        async def go():
            listener.start()
            releaser = asyncio.get_running_loop().create_task(
                release_q_when_airborne())
            try:
                return await orch.run()
            finally:
                await releaser
                listener.stop()

        t0 = time.perf_counter()
        code = asyncio.run(go())
        elapsed = time.perf_counter() - t0

    assert code == 0
    assert elapsed < 5.0                # the poll caught it, not the budget
    assert agents[0].state is AgentState.DONE
    assert "operator_abort" in [e["event"] for e in events_of(run_dir)]


def test_abort_listener_ignores_other_keys(run_dir):
    src = FakeAbortSource(lines=("x\n", "quit\n", "q\n"))
    src.gate.set()                                         # no blocking needed
    abort_evt = threading.Event()
    listener = AbortListener(abort_evt, source=src)
    listener.start()
    assert abort_evt.wait(timeout=5.0)                     # only 'q' fired it
    assert listener.stop() is True


def test_abort_listener_eof_disables_quietly(capsys):
    class EOFSource:
        def readline(self) -> str:
            return ""

    abort_evt = threading.Event()
    listener = AbortListener(abort_evt, source=EOFSource())
    listener.start()
    for _ in range(500):                                   # bounded poll
        if not listener.is_alive():
            break
        time.sleep(0.01)
    assert not listener.is_alive()
    assert not abort_evt.is_set()
    assert "abort key disabled" in capsys.readouterr().err


def test_abort_listener_survives_pytest_style_stdin(capsys):
    """pytest's captured stdin raises OSError on read — the listener must
    disable itself loudly instead of dying as an unhandled thread crash."""

    class RaisingSource:
        def readline(self) -> str:
            raise OSError("pytest: reading from stdin while output is "
                          "captured!")

    abort_evt = threading.Event()
    listener = AbortListener(abort_evt, source=RaisingSource())
    listener.start()
    for _ in range(500):                                   # bounded poll
        if not listener.is_alive():
            break
        time.sleep(0.01)
    assert not listener.is_alive()
    assert not abort_evt.is_set()
    assert "abort key disabled" in capsys.readouterr().err


def test_abort_listener_start_twice_refused():
    src = FakeAbortSource(lines=())          # gate released -> instant EOF,
    src.gate.set()                           # so no thread lingers past stop
    listener = AbortListener(threading.Event(), source=src)
    listener.start()
    with pytest.raises(RuntimeError, match="twice"):
        listener.start()
    assert listener.stop() is True

    with pytest.raises(ValueError, match="threading.Event"):
        AbortListener("not an event")


# ============================================================
# 10. MissionClockGuard through the orchestrator — DOWN before budget (gate 8)
# ============================================================
def test_mission_clock_guard_lands_all_before_budget(run_dir):
    alpha = MockAdapter("alpha")
    bravo = MockAdapter("bravo")
    with EventLog(run_dir) as events:
        agents = [DroneAgent("alpha", alpha, [WaitForeverPhase()], events),
                  DroneAgent("bravo", bravo, [WaitForeverPhase()], events)]
        orch = Orchestrator(
            agents, events, run_dir, budget_s=30.0,
            heartbeat_period_s=0.02,
            swarm_guards=[MissionClockGuard(budget_s=30.0,
                                            landing_reserve_s=29.5)])
        t0 = time.perf_counter()
        code = asyncio.run(orch.run())
        elapsed = time.perf_counter() - t0

    assert code == 0
    assert elapsed < 10.0                       # DOWN long before the 30 s budget
    for agent, adapter in ((agents[0], alpha), (agents[1], bravo)):
        assert agent.state is AgentState.DONE
        assert names(adapter.calls)[-2:] == ["land", "disconnect"]
        assert not adapter.is_flying
    evs = events_of(run_dir)
    trips = [e for e in evs if e["event"] == "guard_trip"]
    assert any(t["data"]["guard"] == "MissionClockGuard"
               and t["data"]["action"] == "LAND_ALL" for t in trips)
    kinds = [e["event"] for e in evs]
    assert "budget_expired" not in kinds        # the guard won, not the budget


def test_loop_overrun_guard_trips_on_a_starved_loop(run_dir):
    """Integration pin for the BEAT-GAP measurement: a phase that BLOCKS
    the event loop (the bug class LoopOverrunGuard exists for) stretches
    the orchestrator's beat-to-beat gap and must walk the ladder to
    LAND_ALL — a drain-only duration would never see the starvation."""

    class BlockingPhase(MissionPhase):
        name = "blocking"

        def __init__(self):
            self._took_off = False

        def step(self, ctx: AgentContext) -> Action:
            if not self._took_off:
                self._took_off = True
                return Takeoff(height_cm=80)
            time.sleep(0.08)        # synchronous: starves the WHOLE loop
            return Wait(duration_s=0.001)

    alpha = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agents = [DroneAgent("alpha", alpha, [BlockingPhase()], events)]
        orch = Orchestrator(
            agents, events, run_dir, budget_s=10.0,
            heartbeat_period_s=0.02,
            swarm_guards=[LoopOverrunGuard(period_s=0.02, factor=2.0,
                                           n_ticks=1)])
        t0 = time.perf_counter()
        code = asyncio.run(orch.run())
        elapsed = time.perf_counter() - t0

    assert code == 0
    assert elapsed < 10.0           # the guard, not the budget, ended the run
    assert agents[0].state is AgentState.DONE
    actions = [t["data"]["action"] for t in guard_trips_of(run_dir)
               if t["data"]["guard"] == "LoopOverrunGuard"]
    assert "DEGRADE_DETECTION" in actions and "LAND_ALL" in actions


def test_raising_swarm_guard_is_a_land_all_trip(run_dir, capsys):
    alpha = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agents = [DroneAgent("alpha", alpha, [WaitForeverPhase()], events)]
        orch = Orchestrator(agents, events, run_dir, budget_s=30.0,
                            heartbeat_period_s=0.02,
                            swarm_guards=[RaiseOnNth(2)])
        code = asyncio.run(orch.run())

    assert code == 0
    assert agents[0].state is AgentState.DONE   # landed clean, not emergency
    trips = guard_trips_of(run_dir)
    assert any(t["data"]["guard"] == "RaiseOnNth"
               and t["data"]["action"] == "LAND_ALL" for t in trips)
    assert "kaboom" in capsys.readouterr().err  # traceback logged
