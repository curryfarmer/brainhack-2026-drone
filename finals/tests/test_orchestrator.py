"""finals.mission.orchestrator + the finals.main S4 wiring.

The S4 gate lives here: two agents, one scripted to fail mid-mission — the
other completes its full phase, the failed one emergency-lands EXACTLY once,
the orchestrator finishes, and the heartbeat tells the story.
"""
from __future__ import annotations

import asyncio
import json
import os
import time

import pytest

from finals.events import EventLog, read_events
from finals.flight.mock_adapter import MockAdapter
from finals.mission.agent import AgentState, DroneAgent
from finals.mission.orchestrator import Orchestrator
from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases.takeoff_demo import TakeoffDemo
from finals.sightings import SightingBus
from finals.types import (Abort, Action, Done, Hover, Land, Sighting,
                          Takeoff, Wait)

HAPPY_CALLS = (["connect", "takeoff", "hover"] + ["move", "rotate"] * 4
               + ["land"])


def names(calls):
    return [c[0] for c in calls]


def mission_events(run_dir):
    return list(read_events(os.path.join(run_dir, "mission.jsonl")))


def read_heartbeat(run_dir):
    with open(os.path.join(run_dir, "heartbeat.json"), encoding="utf-8") as f:
        return json.load(f)


def make_sighting(drone_id: str, marker_id: int) -> Sighting:
    return Sighting(drone_id=drone_id, ts=time.time(), source="aruco",
                    class_name=f"aruco_{marker_id}", marker_id=marker_id,
                    bbox_xyxy=(0.0, 0.0, 10.0, 10.0), confidence=1.0,
                    frame_shape=(480, 640))


# ---------------- test phases (NOT registered — registry is pinned) ------
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

    def __init__(self, wait_s: float = 0.02):
        self._took_off = False
        self._wait_s = wait_s

    def step(self, ctx: AgentContext) -> Action:
        if not self._took_off:
            self._took_off = True
            return Takeoff(height_cm=80)
        return Wait(duration_s=self._wait_s)


class RaisingPhase(MissionPhase):
    name = "raising"

    def __init__(self):
        self._took_off = False

    def step(self, ctx: AgentContext) -> Action:
        if not self._took_off:
            self._took_off = True
            return Takeoff(height_cm=80)
        raise RuntimeError("kaboom: phase bug")


class PublishingPhase(MissionPhase):
    """Publishes n sightings onto the bus, one per step, recording what it
    sees back through ctx.sightings (the agent's own bus cursor)."""

    name = "publishing"

    def __init__(self, bus: SightingBus, drone_id: str, n: int = 5):
        self._bus = bus
        self._drone_id = drone_id
        self._n = n
        self._published = 0
        self._took_off = False
        self.seen = []

    def step(self, ctx: AgentContext) -> Action:
        self.seen.extend(s.marker_id for s in ctx.sightings)
        if not self._took_off:
            self._took_off = True
            return Takeoff(height_cm=80)
        if self._published < self._n:
            self._published += 1
            self._bus.publish(make_sighting(self._drone_id, self._published))
            return Wait(duration_s=0.02)
        return Done(f"published {self._n} sightings")


# ============================================================
# 1. THE S4 GATE — failure isolation through the orchestrator
# ============================================================
def test_s4_gate_one_agent_fails_other_completes(tmp_path):
    run_dir = str(tmp_path)
    alpha = MockAdapter("alpha", latency_s=0.001)
    bravo = MockAdapter("bravo", latency_s=0.001, fail_at="move:2")
    with EventLog(run_dir) as events:
        agents = [DroneAgent("alpha", alpha, [TakeoffDemo()], events),
                  DroneAgent("bravo", bravo, [TakeoffDemo()], events)]
        orch = Orchestrator(agents, events, run_dir, budget_s=30.0,
                            heartbeat_period_s=0.02)
        code = asyncio.run(orch.run())

    assert code == 1                                # one drone FAILED

    # alpha: completes its FULL phase, untouched by bravo's failure.
    assert agents[0].state is AgentState.DONE
    assert names(alpha.calls) == HAPPY_CALLS + ["disconnect"]
    assert "emergency_land" not in names(alpha.calls)

    # bravo: fails on move #2, emergency-lands EXACTLY once, disconnects.
    assert agents[1].state is AgentState.FAILED
    assert names(bravo.calls) == [
        "connect", "takeoff", "hover", "move", "rotate", "move",
        "emergency_land", "disconnect"]
    assert names(bravo.calls).count("emergency_land") == 1
    assert "move:2" in agents[1].failure or "Move failed" in agents[1].failure

    # The final heartbeat reflects one failed + one done — and was written
    # AFTER bravo's death (heartbeat survives agent death).
    hb = read_heartbeat(run_dir)
    assert hb["final"] is True
    assert hb["drones"]["alpha"]["state"] == "DONE"
    assert hb["drones"]["bravo"]["state"] == "FAILED"
    assert hb["drones"]["bravo"]["emergency_landed"] is True
    assert hb["drones"]["bravo"]["failure"]

    kinds = [e["event"] for e in mission_events(run_dir)]
    assert "run_start" in kinds and "run_end" in kinds


# ============================================================
# 2. Budget expiry — lands all, exits cleanly
# ============================================================
def test_budget_expiry_lands_all_and_exits_clean(tmp_path):
    run_dir = str(tmp_path)
    alpha = MockAdapter("alpha")
    bravo = MockAdapter("bravo")
    with EventLog(run_dir) as events:
        agents = [DroneAgent("alpha", alpha, [WaitForeverPhase()], events),
                  DroneAgent("bravo", bravo, [WaitForeverPhase()], events)]
        orch = Orchestrator(agents, events, run_dir, budget_s=0.15,
                            heartbeat_period_s=0.02, settle_grace_s=10.0)
        t0 = time.perf_counter()
        code = asyncio.run(orch.run())
        elapsed = time.perf_counter() - t0

    assert code == 0                                # budget stop is CLEAN
    assert elapsed < 10.0                           # no settle-grace burn
    for agent, adapter in ((agents[0], alpha), (agents[1], bravo)):
        assert agent.state is AgentState.DONE
        assert agent.status()["stopped_reason"] is not None
        assert names(adapter.calls)[-2:] == ["land", "disconnect"]
        assert not adapter.is_flying
    kinds = [e["event"] for e in mission_events(run_dir)]
    assert "budget_expired" in kinds
    hb = read_heartbeat(run_dir)
    assert hb["stop_signalled"] is True


# ============================================================
# 3. Abort isolation
# ============================================================
def test_abort_in_one_phase_leaves_others_unaffected(tmp_path):
    run_dir = str(tmp_path)
    alpha = MockAdapter("alpha", latency_s=0.001)
    bravo = MockAdapter("bravo", latency_s=0.001)
    with EventLog(run_dir) as events:
        agents = [
            DroneAgent("alpha", alpha, [TakeoffDemo()], events),
            DroneAgent("bravo", bravo, [ScriptedPhase(
                [Takeoff(height_cm=80), Abort("pad occupied")])], events),
        ]
        orch = Orchestrator(agents, events, run_dir, budget_s=30.0,
                            heartbeat_period_s=0.02)
        code = asyncio.run(orch.run())

    assert code == 1
    assert agents[0].state is AgentState.DONE
    assert names(alpha.calls) == HAPPY_CALLS + ["disconnect"]
    assert agents[1].state is AgentState.FAILED
    assert "pad occupied" in agents[1].failure
    assert names(bravo.calls) == ["connect", "takeoff", "emergency_land",
                                  "disconnect"]


# ============================================================
# 4. Heartbeat — ~1 Hz (scaled), parseable, real data
# ============================================================
def test_heartbeat_cadence_and_content(tmp_path):
    run_dir = str(tmp_path)
    adapter = MockAdapter("alpha", battery_decay_pct_per_cmd=1.0)
    phase = ScriptedPhase([Takeoff(height_cm=80)] + [Wait(0.05)] * 6
                          + [Land()])
    with EventLog(run_dir) as events:
        agents = [DroneAgent("alpha", adapter, [phase], events)]
        orch = Orchestrator(agents, events, run_dir, budget_s=30.0,
                            heartbeat_period_s=0.05)
        code = asyncio.run(orch.run())

    assert code == 0
    hb = read_heartbeat(run_dir)                    # parseable JSON
    # ~0.3 s of Waits at a 0.05 s beat: at least a few in-loop ticks fired
    # (generous lower bound — only a stalled machine undercuts it).
    assert hb["tick"] >= 2
    assert hb["final"] is True
    assert hb["elapsed_s"] > 0
    assert hb["tick_latency_s"] >= 0
    drone = hb["drones"]["alpha"]
    assert drone["state"] == "DONE"
    assert drone["battery_pct"] < 100.0             # real telemetry flowed
    assert drone["last_action"] == "Land"
    assert drone["telemetry_age_s"] is not None


# ============================================================
# 5. SightingBus drain — seq cursor, exactly-once, both consumers
# ============================================================
def test_sighting_drain_no_misses_no_dupes(tmp_path):
    run_dir = str(tmp_path)
    bus = SightingBus()
    adapter = MockAdapter("alpha")
    with EventLog(run_dir) as events:
        phase = PublishingPhase(bus, "alpha", n=5)
        agents = [DroneAgent("alpha", adapter, [phase], events, bus=bus)]
        orch = Orchestrator(agents, events, run_dir, budget_s=30.0,
                            bus=bus, heartbeat_period_s=0.02)
        code = asyncio.run(orch.run())

    assert code == 0
    # Orchestrator cursor: every published sighting logged EXACTLY once
    # (the final post-shutdown drain catches stragglers).
    sightings = [e for e in mission_events(run_dir) if e["event"] == "sighting"]
    marker_ids = [e["data"]["marker_id"] for e in sightings]
    assert sorted(marker_ids) == [1, 2, 3, 4, 5]
    assert len(marker_ids) == len(set(marker_ids))  # no dupes across ticks
    # Agent cursor: the phase saw each of its sightings exactly once too.
    assert phase.seen == [1, 2, 3, 4, 5]


# ============================================================
# 6. Agent task crash — netted with traceback; others continue
# ============================================================
def test_agent_task_crash_is_netted_and_others_complete(tmp_path, capsys):
    run_dir = str(tmp_path)
    alpha = MockAdapter("alpha", latency_s=0.001)
    bravo = MockAdapter("bravo", latency_s=0.001)
    with EventLog(run_dir) as events:
        agents = [DroneAgent("alpha", alpha, [TakeoffDemo()], events),
                  DroneAgent("bravo", bravo, [RaisingPhase()], events)]
        orch = Orchestrator(agents, events, run_dir, budget_s=30.0,
                            heartbeat_period_s=0.02)
        code = asyncio.run(orch.run())

    assert code == 1
    assert agents[0].state is AgentState.DONE       # alpha untouched
    assert names(alpha.calls) == HAPPY_CALLS + ["disconnect"]
    assert agents[1].state is AgentState.FAILED     # bravo netted + safed
    assert names(bravo.calls) == ["connect", "takeoff", "emergency_land",
                                  "disconnect"]
    crashed = [e for e in mission_events(run_dir)
               if e["event"] == "agent_task_crashed"]
    assert len(crashed) == 1
    assert crashed[0]["drone"] == "bravo"
    assert "kaboom" in crashed[0]["data"]["traceback"]   # FULL traceback
    err = capsys.readouterr().err
    assert "kaboom" in err                          # also screamed to stderr


# ============================================================
# 7. Settle grace — a hung agent is cancelled and force-safed
# ============================================================
def test_settle_grace_cancels_hung_agent_and_force_lands(tmp_path):
    run_dir = str(tmp_path)

    class HangingHoverAdapter(MockAdapter):
        async def hover(self, duration_s):
            self._record("hover", duration_s=duration_s)
            await asyncio.sleep(3600.0)             # ignores everything (bug)

    adapter = HangingHoverAdapter("alpha")
    with EventLog(run_dir) as events:
        # Outer command deadline (15 + 2 + 60 s) far exceeds the settle
        # grace: only the orchestrator's hard deadline can end this.
        agents = [DroneAgent("alpha", adapter, [ScriptedPhase(
            [Takeoff(height_cm=80), Hover(60.0)])], events)]
        orch = Orchestrator(agents, events, run_dir, budget_s=0.05,
                            heartbeat_period_s=0.02, settle_grace_s=0.2)
        t0 = time.perf_counter()
        code = asyncio.run(orch.run())
        elapsed = time.perf_counter() - t0

    assert elapsed < 10.0                           # not 60, not 3600
    assert code == 1
    assert agents[0].state is AgentState.FAILED
    assert names(adapter.calls).count("emergency_land") == 1
    kinds = [e["event"] for e in mission_events(run_dir)]
    assert "settle_deadline_exceeded" in kinds
    assert "agent_task_cancelled" in kinds


# ============================================================
# 8. Constructor validation
# ============================================================
def test_constructor_rejects_bad_wiring(tmp_path):
    run_dir = str(tmp_path)
    with EventLog(run_dir) as events:
        def agent(drone_id="alpha"):
            return DroneAgent(drone_id, MockAdapter(drone_id),
                              [TakeoffDemo()], events)

        with pytest.raises(ValueError, match="non-empty"):
            Orchestrator([], events, run_dir, budget_s=10.0)
        with pytest.raises(ValueError, match="duplicate"):
            Orchestrator([agent(), agent()], events, run_dir, budget_s=10.0)
        with pytest.raises(ValueError, match="reserved"):
            Orchestrator([agent("mission")], events, run_dir, budget_s=10.0)
        for kwargs in ({"budget_s": 0.0}, {"budget_s": float("nan")},
                       {"heartbeat_period_s": 0.0},
                       {"settle_grace_s": -1.0}):
            with pytest.raises(ValueError, match="finite"):
                Orchestrator([agent()], events, run_dir,
                             **{"budget_s": 10.0, **kwargs})


# ============================================================
# 9. finals.main wiring — the composition root end to end
# ============================================================
def test_main_mock_runs_end_to_end(tmp_path, monkeypatch, capsys):
    from finals.main import main

    monkeypatch.chdir(tmp_path)
    code = main(["--profile", "mock", "--phases", "takeoff_demo",
                 "--budget", "30"])
    assert code == 0

    run_dirs = list((tmp_path / "runs_finals").iterdir())
    assert len(run_dirs) == 1
    rd = str(run_dirs[0])
    evs = mission_events(rd)
    kinds = [e["event"] for e in evs]
    for expected in ("preflight", "run_start", "origin", "phase_enter",
                     "action_complete", "phase_done", "agent_done",
                     "agent_disconnect", "run_end"):
        assert expected in kinds, f"missing {expected!r} in mission.jsonl"
    run_end = [e for e in evs if e["event"] == "run_end"][0]
    assert run_end["data"]["exit_code"] == 0
    hb = read_heartbeat(rd)
    assert hb["drones"]["alpha"]["state"] == "DONE"
    assert "MISSION SUMMARY" in capsys.readouterr().out


def test_main_exit_code_1_when_a_drone_fails(tmp_path, monkeypatch):
    """A drone failing mid-mission is NOT ok — exit 1 per the docstring.
    The mid-mission failure semantics live in the gate test; this only pins
    main's plumbing of the orchestrator exit code, by scripting the failure
    into the adapter the real wiring builds."""
    import finals.main as fmain

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(
        fmain, "_build_adapter",
        lambda cfg, drone, *, api=None: MockAdapter(drone.id, fail_at="move:2"))
    code = fmain.main(["--profile", "mock", "--phases", "takeoff_demo",
                       "--budget", "30"])
    assert code == 1


_FINALS_CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs")


def _inject_fake_fleet(monkeypatch):
    """Make a bench/real `--preflight-only` run exercise the WHOLE wiring with
    no pyhulax, no real Dola, and no cv2 on fake frames: one shared FakeDroneAPI
    + FakeVideoStream per drone, a fake Dola discovery returning an IP per
    plane_id, and a no-op marker detector (the real aruco one would call cv2 on
    the fake _ChannelArray frame in P7)."""
    import finals.main as fmain
    import finals.preflight as pf
    import finals.vision.aruco as aruco
    from finals.flight.pyhulax_adapter import FakeDroneAPI
    from finals.vision.pyhulax_video import FakeVideoStream

    monkeypatch.setattr(
        fmain, "_make_shared_pyhulax_api",
        lambda cfg: FakeDroneAPI(video_stream=FakeVideoStream()))
    monkeypatch.setattr(
        pf, "_default_discover",
        lambda plane_ids, timeout_s: {p: f"10.0.0.{p}" for p in plane_ids})
    monkeypatch.setattr(aruco, "make_marker_detector",
                        lambda backend, save_dir=None:
                        (lambda frame, source_id: []))


def test_main_bench_preflight_only_runs_the_gate(tmp_path, monkeypatch):
    """S10 (replaces the bench S10-stub pointer): `--preflight-only` builds the
    REAL bench fleet — BenchAdapter wrapping PyhulaxAdapter, inner-first (the
    special case generic flight_cls(drone_id) cannot build) — and runs P0-P9
    green with fakes injected. Exit 0 + a persisted preflight.json."""
    pytest.importorskip("cv2")
    import finals.main as fmain
    from finals.config import load_config
    from finals.flight.adapter import BenchAdapter
    from finals.flight.pyhulax_adapter import FakeDroneAPI, PyhulaxAdapter
    from finals.vision.pyhulax_video import FakeVideoStream

    # The inner-first wrap, asserted directly (no preflight needed).
    cfg = load_config(os.path.join(_FINALS_CONFIG_DIR, "bench.json"))
    adapter = fmain._build_adapter(
        cfg, cfg.drones[0],
        api=FakeDroneAPI(video_stream=FakeVideoStream()))
    assert isinstance(adapter, BenchAdapter)
    assert isinstance(adapter.inner, PyhulaxAdapter)

    monkeypatch.chdir(tmp_path)
    _inject_fake_fleet(monkeypatch)
    code = fmain.main(["--profile", "bench", "--preflight-only"])
    assert code == 0
    run_dirs = list((tmp_path / "runs_finals").iterdir())
    assert any((rd / "preflight.json").exists() for rd in run_dirs), (
        "preflight-only must persist preflight.json")


# (test_main_sitl_points_at_s6 was deleted in S6/SIM-1: MavsdkSitlAdapter is
# now real — wiring/endpoint coverage lives in tests/test_sitl_adapter.py and
# the flight path is the VM gate V1, sim_sessions.md SIM-1 evidence.
# test_main_real_points_at_s9 became _at_s10 in S9, then S10 made preflight
# real too — both bench and real now RUN the gate; full per-gate coverage lives
# in tests/test_preflight.py.)


def test_main_real_preflight_only_runs_the_gate(tmp_path, monkeypatch):
    """S10 (replaces the real S10-stub pointer): real `--preflight-only` runs
    P0-P9 green with fakes — still behind the --i-know-this-arms-real-drones
    gate (it makes hardware contact: connect + failsafe + LED), but P10
    operator-GO is skipped and nothing flies."""
    pytest.importorskip("cv2")
    import finals.main as fmain

    monkeypatch.chdir(tmp_path)
    _inject_fake_fleet(monkeypatch)
    code = fmain.main(["--profile", "real",
                       "--i-know-this-arms-real-drones", "--preflight-only"])
    assert code == 0
