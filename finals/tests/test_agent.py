"""finals.mission.agent — DroneAgent over MockAdapter.

No pytest-asyncio: coroutines are driven with asyncio.run() inside sync
tests (suite convention). EventLogs are real (tmp run dirs) so the event
trail is asserted, not mocked.
"""
from __future__ import annotations

import asyncio
import os
import time

import pytest

from finals.errors import FlightError, FlightTimeout
from finals.events import EventLog, read_events
from finals.flight.mock_adapter import MockAdapter
from finals.mission.agent import AgentState, DroneAgent
from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases.takeoff_demo import TakeoffDemo
from finals.types import Abort, Action, Done, Takeoff, Wait

HAPPY_CALLS = (["connect", "takeoff", "hover"] + ["move", "rotate"] * 4
               + ["land"])


class FakeClock:
    """Deterministic monotonic source — tests set .t explicitly."""

    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


# ---------------- test phases (NOT registered: PHASE_REGISTRY is pinned
# exactly by test_conventions.py, so test phases subclass directly) --------
class ScriptedPhase(MissionPhase):
    """Returns a fixed action sequence, then Done forever."""

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
    """Takeoff, then Wait until the mission deadline/stop ends the run."""

    name = "wait_forever"

    def __init__(self, wait_s: float = 30.0):
        self._took_off = False
        self._wait_s = wait_s

    def step(self, ctx: AgentContext) -> Action:
        if not self._took_off:
            self._took_off = True
            return Takeoff(height_cm=80)
        return Wait(duration_s=self._wait_s)


class RaisingPhase(MissionPhase):
    """Takeoff, then raise — the phase-bug class the orchestrator nets."""

    name = "raising"

    def __init__(self):
        self._took_off = False

    def step(self, ctx: AgentContext) -> Action:
        if not self._took_off:
            self._took_off = True
            return Takeoff(height_cm=80)
        raise RuntimeError("kaboom: phase bug")


# ---------------- harness ----------------
def run_agent(agent, *, budget_s: float = 30.0,
              stop_delay_s: float = None) -> None:
    async def go():
        stop = asyncio.Event()
        if stop_delay_s is None:
            await agent.run(deadline=time.monotonic() + budget_s,
                            stop_event=stop)
        else:
            task = asyncio.ensure_future(
                agent.run(deadline=time.monotonic() + budget_s,
                          stop_event=stop))
            await asyncio.sleep(stop_delay_s)
            stop.set()
            await task

    asyncio.run(go())


def events_of(run_dir: str, drone_id: str = "alpha"):
    return list(read_events(os.path.join(run_dir, "mission.jsonl")))


def names(calls):
    return [c[0] for c in calls]


@pytest.fixture
def run_dir(tmp_path):
    return str(tmp_path)


# ============================================================
# 1. Happy path — .calls order exact; Done advances phases
# ============================================================
def test_happy_path_calls_exact_and_done(run_dir):
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo()], events)
        run_agent(agent)
        asyncio.run(agent.shutdown())

    assert names(adapter.calls) == HAPPY_CALLS + ["disconnect"]
    # Exact args on the critical commands (deadline plumbed from config).
    assert adapter.calls[0] == ("connect", {"timeout_s": 15.0})
    assert adapter.calls[1] == ("takeoff", {"height_cm": 80,
                                            "timeout_s": 15.0})
    assert agent.state is AgentState.DONE
    assert agent.phases_completed == 1
    assert agent.failure is None
    assert not adapter.is_flying
    # The DR square closes — the calibration property FLIGHT 1/2 relies on.
    assert adapter.dr.pose.north_m == pytest.approx(0.0, abs=1e-9)
    assert adapter.dr.pose.east_m == pytest.approx(0.0, abs=1e-9)
    assert adapter.dr.pose.alt_m == 0.0

    st = agent.status()
    assert st["state"] == "DONE" and st["last_action"] == "Land"
    assert st["last_action_ok"] is True and st["battery_pct"] == 100.0


def test_done_advances_through_multiple_phases(run_dir):
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agent = DroneAgent(
            "alpha", adapter,
            [TakeoffDemo(legs=0, hover_s=0.0), TakeoffDemo(legs=1)], events)
        run_agent(agent)

    assert agent.state is AgentState.DONE
    assert agent.phases_completed == 2
    # Phase 1 lands, phase 2 takes off again — two full cycles, no re-land
    # at the end (the agent only lands if airborne).
    assert names(adapter.calls) == [
        "connect", "takeoff", "hover", "land",            # phase 1 (legs=0)
        "takeoff", "hover", "move", "rotate", "land",     # phase 2 (legs=1)
    ]


def test_origin_and_action_events_written(run_dir):
    """The replay-plot prereq (simulation.md Tier 0): an initial-pose origin
    event plus one action_complete per executed command."""
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo()], events)
        run_agent(agent)

    evs = events_of(run_dir)
    origins = [e for e in evs if e["event"] == "origin"]
    assert len(origins) == 1
    od = origins[0]["data"]
    assert od["position_m"] == [0.0, 0.0, 0.0]
    assert od["position_quality"] == "DEAD_RECKONING"
    assert "dead_reckon" in od["frame"]            # points at the convention
    completes = [e["data"]["action"] for e in evs
                 if e["event"] == "action_complete"]
    assert completes == (["Takeoff", "Hover"] + ["Move", "Rotate"] * 4
                         + ["Land"])
    # Enums are logged by NAME — greppable at 2 a.m.
    move_events = [e["data"] for e in evs if e["event"] == "action_complete"
                   and e["data"]["action"] == "Move"]
    assert all(m["direction"] == "FORWARD" for m in move_events)


# ============================================================
# 2. Command deadline — adapter-enforced and agent-enforced (outer)
# ============================================================
def test_adapter_flighttimeout_fails_agent_safe_down_once(run_dir):
    class SlowAfterConnect(MockAdapter):
        """Connect at full speed, then every command outlasts its deadline
        (latency_s is public-mutable by design — see the mock docstring)."""
        async def connect(self, timeout_s=10.0):
            await super().connect(timeout_s=timeout_s)
            self.latency_s = 5.0                    # > timeout: immediate FT

    adapter = SlowAfterConnect("alpha")
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo()], events,
                           command_timeout_s=2.0)
        run_agent(agent)                            # returns normally

    assert agent.state is AgentState.FAILED
    assert names(adapter.calls).count("emergency_land") == 1   # EXACTLY once
    assert names(adapter.calls) == ["connect", "takeoff", "emergency_land"]
    assert "Takeoff failed" in agent.failure
    assert "2.0 s" in agent.failure and "check" in agent.failure
    # No auto-restart: terminal state, no further commands recorded.
    assert agent.status()["emergency_landed"] is True


def test_outer_deadline_catches_adapter_that_ignores_its_timeout(run_dir):
    """The mapping_drone.py watchdog-gap class: a backend that hangs PAST
    its own deadline must be caught by the agent's outer wait_for."""

    class HangingMoveAdapter(MockAdapter):
        async def move(self, direction, distance_cm, timeout_s=15.0):
            self._record("move", direction=direction,
                         distance_cm=distance_cm, timeout_s=timeout_s)
            await asyncio.sleep(3600.0)             # ignores timeout_s (bug)

    adapter = HangingMoveAdapter("alpha")
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo(hover_s=0.0)],
                           events, command_timeout_s=0.05,
                           command_grace_s=0.05)
        t0 = time.perf_counter()
        run_agent(agent)
        elapsed = time.perf_counter() - t0

    assert elapsed < 10.0                           # caught, not 3600 s
    assert agent.state is AgentState.FAILED
    assert names(adapter.calls).count("emergency_land") == 1
    # The message names the real culprit: the adapter's own deadline.
    assert "outer deadline" in agent.failure
    assert "timeout_s=0.1 never fired" in agent.failure \
        or "never fired" in agent.failure
    assert "alpha" in agent.failure and "check" in agent.failure
    failed = [e for e in events_of(run_dir) if e["event"] == "action_failed"]
    assert len(failed) == 1 and failed[0]["data"]["error_type"] == "FlightTimeout"


# ============================================================
# 3. Clean FlightError + degraded adapter — never crashes the loop
# ============================================================
def test_clean_flighterror_fails_agent_without_crash(run_dir):
    adapter = MockAdapter("alpha", fail_on={
        "move": FlightError("alpha: move(FORWARD, 100 cm) failed — "
                            "scripted link drop — check the scenario")})
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo()], events)
        run_agent(agent)                            # must not raise

    assert agent.state is AgentState.FAILED
    assert names(adapter.calls).count("emergency_land") == 1
    assert "link drop" in agent.failure


def test_post_timeout_degraded_safe_down_path_works(run_dir):
    """After a FlightTimeout the adapter refuses ordinary commands
    (degraded) but the safe-down surface must still work — the agent's
    emergency_land goes through, nothing crashes."""
    adapter = MockAdapter("alpha", fail_at="move:2")
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo()], events)
        run_agent(agent)

    assert adapter.degraded                         # timeout left it degraded
    assert agent.state is AgentState.FAILED
    assert names(adapter.calls) == [
        "connect", "takeoff", "hover", "move", "rotate", "move",
        "emergency_land"]                           # exactly one safe-down
    assert not adapter.is_flying


# ============================================================
# 4. Abort action
# ============================================================
def test_abort_action_safes_down_and_fails(run_dir):
    adapter = MockAdapter("alpha")
    phase = ScriptedPhase([Takeoff(height_cm=80),
                           Abort("test demanded failure")])
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [phase], events)
        run_agent(agent)

    assert agent.state is AgentState.FAILED
    assert "test demanded failure" in agent.failure
    assert names(adapter.calls) == ["connect", "takeoff", "emergency_land"]
    aborts = [e for e in events_of(run_dir) if e["event"] == "phase_abort"]
    assert len(aborts) == 1


# ============================================================
# 5. Telemetry staleness
# ============================================================
def test_stale_telemetry_fails_loud_before_acting(run_dir):
    clock = FakeClock(100.0)

    class BumpOnTakeoff(MockAdapter):
        """Completing takeoff 'takes' 10 s on the shared fake clock."""
        async def takeoff(self, *a, **kw):
            await super().takeoff(*a, **kw)
            clock.t += 10.0

    adapter = BumpOnTakeoff("alpha", clock=clock,
                            freeze_telemetry_after_s=5.0)
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo()], events,
                           telemetry_stale_s=3.0, clock=clock)
        run_agent(agent)

    assert agent.state is AgentState.FAILED
    assert names(adapter.calls).count("emergency_land") == 1
    msg = agent.failure
    # The errors.py bar: WHAT, WHICH drone, HOW LONG vs WHICH limit, CHECK.
    assert "alpha" in msg and "STALE" in msg
    assert "5.0 s" in msg and "3.0 s" in msg and "check" in msg


# ============================================================
# 6. Wait + stop event
# ============================================================
def test_wait_phase_stops_promptly_and_lands_clean(run_dir):
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [WaitForeverPhase(30.0)], events)
        t0 = time.perf_counter()
        run_agent(agent, stop_delay_s=0.05)         # stop mid-Wait(30)
        elapsed = time.perf_counter() - t0

    assert elapsed < 10.0                           # woke on stop, not 30 s
    assert agent.state is AgentState.DONE           # budget stop is CLEAN
    assert agent.status()["stopped_reason"] is not None
    assert names(adapter.calls) == ["connect", "takeoff", "land"]
    assert not adapter.is_flying


def test_deadline_expiry_lands_clean_without_stop_event(run_dir):
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [WaitForeverPhase(0.01)], events)
        run_agent(agent, budget_s=0.05)             # deadline does the job

    assert agent.state is AgentState.DONE
    assert "deadline" in agent.status()["stopped_reason"]
    assert names(adapter.calls)[-1] == "land"


# ============================================================
# 7. Unexpected exceptions propagate; shutdown still safes down
# ============================================================
def test_phase_exception_propagates_then_shutdown_safes_down(run_dir):
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [RaisingPhase()], events)
        with pytest.raises(RuntimeError, match="kaboom"):
            run_agent(agent)                        # bug escapes (by design)
        assert adapter.is_flying                    # still airborne!
        asyncio.run(agent.shutdown())               # the orchestrator's job

    assert names(adapter.calls) == [
        "connect", "takeoff", "emergency_land", "disconnect"]
    assert not adapter.is_flying


def test_fail_safe_is_noop_after_clean_done(run_dir):
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo(legs=0)], events)
        run_agent(agent)
        asyncio.run(agent.fail_safe("should be ignored"))

    assert agent.state is AgentState.DONE
    assert "emergency_land" not in names(adapter.calls)


def test_fail_safe_lands_exactly_once_even_when_called_twice(run_dir):
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [RaisingPhase()], events)
        with pytest.raises(RuntimeError):
            run_agent(agent)
        asyncio.run(agent.fail_safe("net 1"))
        asyncio.run(agent.fail_safe("net 2"))
        asyncio.run(agent.shutdown())

    assert names(adapter.calls).count("emergency_land") == 1
    assert agent.state is AgentState.FAILED


def test_shutdown_is_idempotent(run_dir):
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo(legs=0)], events)
        run_agent(agent)
        asyncio.run(agent.shutdown())
        asyncio.run(agent.shutdown())

    assert names(adapter.calls).count("disconnect") == 1


# ============================================================
# 8. Connect failure
# ============================================================
def test_connect_failure_fails_agent_cleanly(run_dir):
    adapter = MockAdapter("alpha", fail_on={"connect": FlightTimeout(
        "alpha: connect() exceeded 10.0 s — check Wi-Fi / drone power")})
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo()], events)
        run_agent(agent)

    assert agent.state is AgentState.FAILED
    assert "connect failed" in agent.failure
    # Safe-down is still commanded: after a connect timeout the true state
    # is unknown; emergency_land is the only safe answer (never-raise).
    assert names(adapter.calls) == ["connect", "emergency_land"]


# ============================================================
# 9. Constructor + single-shot guards
# ============================================================
@pytest.mark.parametrize("kwargs", [
    {"command_timeout_s": 0.0},
    {"command_timeout_s": -1.0},
    {"command_timeout_s": float("inf")},
    {"command_timeout_s": float("nan")},
    {"command_grace_s": -0.1},
    {"telemetry_stale_s": 0.0},
])
def test_constructor_rejects_bad_deadlines(run_dir, kwargs):
    with EventLog(run_dir) as events:
        with pytest.raises(ValueError, match="alpha"):
            DroneAgent("alpha", MockAdapter("alpha"), [TakeoffDemo()],
                       events, **kwargs)


def test_constructor_rejects_bad_wiring(run_dir):
    with EventLog(run_dir) as events:
        with pytest.raises(ValueError, match="adapter"):
            DroneAgent("alpha", "not an adapter", [TakeoffDemo()], events)
        with pytest.raises(ValueError, match="phases"):
            DroneAgent("alpha", MockAdapter("alpha"), [], events)
        with pytest.raises(ValueError, match="drone_id"):
            DroneAgent("", MockAdapter("alpha"), [TakeoffDemo()], events)


def test_run_twice_refused(run_dir):
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [TakeoffDemo(legs=0)], events)
        run_agent(agent)
        with pytest.raises(RuntimeError, match="single-shot"):
            run_agent(agent)


def test_garbage_action_from_phase_is_a_loud_typeerror(run_dir):
    adapter = MockAdapter("alpha")
    phase = ScriptedPhase([Takeoff(height_cm=80), "north"])
    with EventLog(run_dir) as events:
        agent = DroneAgent("alpha", adapter, [phase], events)
        with pytest.raises(TypeError, match="Action vocabulary"):
            run_agent(agent)
        asyncio.run(agent.shutdown())               # still safes down

    assert names(adapter.calls).count("emergency_land") == 1
