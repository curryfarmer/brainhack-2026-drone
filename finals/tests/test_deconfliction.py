"""finals deconfliction layer (NAV-8): staggered launch + serialized landing
+ advisory sectors, all under the ~1.1 m-ceiling constraint that FORBIDS
altitude-band separation (so separation is TIME + SPACE, never bands).

Pins:
- STAGGERED LAUNCH: the SafetyController launch-corridor slot serializes
  takeoff — drive 3 agents through real takeoffs and assert no two takeoff
  windows overlap; the slot wait is bounded (FlightTimeout, never a hang).
- SERIALIZED LANDING: the landing slot serializes descent (reuses the S5
  machinery); launch and landing are SEPARATE slots and never deadlock.
- ADVISORY SECTORS: SectorGuard trips ADVISORY when the dead-reckoned
  estimate leaves the wedge — and ONLY ADVISORY (never a land / never a hard
  control input), edge-triggered, skips on no position.
- FAILURE INJECTION: with the launch slot wired, one drone failing mid-launch
  still lets the others finish and emergency-lands EXACTLY once (the S4
  property must survive the new slot).

No pytest-asyncio: coroutines run under asyncio.run inside sync tests (suite
convention). Slot tests assert ORDER/OVERLAP, not wall time.
"""
from __future__ import annotations

import asyncio
import time

import pytest

from finals.errors import FlightError, FlightTimeout
from finals.events import EventLog
from finals.flight.mock_adapter import MockAdapter
from finals.guards import (GuardContext, SafetyController, SectorGuard,
                           TripAction, evaluate_guards)
from finals.mission.agent import AgentState, DroneAgent
from finals.mission.orchestrator import Orchestrator
from finals.mission.phase import AgentContext, MissionPhase
from finals.types import (Abort, Action, Done, Land, PositionQuality, Takeoff,
                          Telemetry, Wait)


@pytest.fixture
def run_dir(tmp_path):
    return str(tmp_path)


def names(calls):
    return [c[0] for c in calls]


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


# ============================================================
# 1. STAGGERED LAUNCH — the launch corridor slot serializes takeoffs
# ============================================================
def test_launch_slot_serializes_concurrent_takeoffs(run_dir):
    record = []

    class RecordingAdapter(MockAdapter):
        async def takeoff(self, height_cm=80, timeout_s=30.0):
            record.append((self.drone_id, "enter", time.perf_counter()))
            await super().takeoff(height_cm=height_cm, timeout_s=timeout_s)
            record.append((self.drone_id, "exit", time.perf_counter()))

    ids = ["alpha", "bravo", "charlie"]
    adapters = {i: RecordingAdapter(i, latency_s=0.05) for i in ids}
    with EventLog(run_dir) as events:
        safety = SafetyController(events, land_retry_period_s=0.02,
                                  land_retry_window_s=0.05)
        agents = [
            DroneAgent(i, adapters[i],
                       [ScriptedPhase([Takeoff(80), Land()])], events,
                       safety=safety)
            for i in ids]

        async def go():
            stop = asyncio.Event()
            deadline = time.monotonic() + 30.0
            await asyncio.gather(*(a.run(deadline=deadline, stop_event=stop)
                                   for a in agents))

        asyncio.run(go())

    assert all(a.state is AgentState.DONE for a in agents)
    # Reconstruct each drone's takeoff window; NONE may overlap another's.
    windows = {}
    for drone_id, kind, t in record:
        windows.setdefault(drone_id, {})[kind] = t
    assert len(windows) == 3
    spans = sorted((w["enter"], w["exit"]) for w in windows.values())
    for (e0, x0), (e1, x1) in zip(spans, spans[1:]):
        assert x0 <= e1 + 1e-9, (
            f"two takeoffs overlapped: {spans} — the launch corridor was "
            f"shared by >1 drone at once")


def test_launch_slot_logs_acquire_release(run_dir):
    from finals.events import read_events
    import os
    adapter = MockAdapter("alpha", latency_s=0.001)
    with EventLog(run_dir) as events:
        safety = SafetyController(events, land_retry_period_s=0.02,
                                  land_retry_window_s=0.05)
        agent = DroneAgent("alpha", adapter,
                           [ScriptedPhase([Takeoff(80), Land()])], events,
                           safety=safety)

        async def go():
            await agent.run(deadline=time.monotonic() + 30.0,
                            stop_event=asyncio.Event())

        asyncio.run(go())
    kinds = [e["event"] for e in
             read_events(os.path.join(run_dir, "mission.jsonl"))]
    assert "launch_slot_acquired" in kinds
    assert "launch_slot_released" in kinds


# ============================================================
# 2. The launch slot wait is BOUNDED with an actionable timeout
# ============================================================
def test_launch_slot_wait_is_bounded(run_dir):
    alpha = MockAdapter("alpha")
    bravo = MockAdapter("bravo")
    with EventLog(run_dir) as events:
        safety = SafetyController(events, land_retry_period_s=0.02,
                                  land_retry_window_s=0.05,
                                  launch_slot_wait_s=0.05)

        async def go():
            await alpha.connect()
            await bravo.connect()
            alpha.latency_s = 0.5                       # hog the launch slot

            async def hold():
                async with safety.launch_slot("alpha"):
                    await alpha.takeoff(80)

            hold_task = asyncio.create_task(hold())
            await asyncio.sleep(0.05)
            with pytest.raises(FlightTimeout, match="launch corridor"):
                async with safety.launch_slot("bravo"):
                    await bravo.takeoff(80)
            await hold_task

        asyncio.run(go())


def test_launch_slot_released_on_exception(run_dir):
    """A takeoff that RAISES inside the slot must still release it (finally),
    so the next drone is never stranded behind a dead holder."""
    with EventLog(run_dir) as events:
        safety = SafetyController(events, launch_slot_wait_s=0.5)

        async def go():
            try:
                async with safety.launch_slot("alpha"):
                    raise FlightError("takeoff blew up")
            except FlightError:
                pass
            # The slot must be free now — a second acquire returns promptly.
            async with safety.launch_slot("bravo"):
                pass
            return True

        assert asyncio.run(go()) is True


# ============================================================
# 3. Launch + landing are SEPARATE slots — they never deadlock
# ============================================================
def test_launch_and_landing_slots_independent(run_dir):
    """A drone DESCENDING (landing slot held) must not block another drone
    LAUNCHING (launch slot) — different corridors-in-time. If they shared one
    semaphore this would serialize/deadlock; assert it completes promptly."""
    alpha = MockAdapter("alpha")
    bravo = MockAdapter("bravo")
    with EventLog(run_dir) as events:
        safety = SafetyController(events, land_retry_period_s=0.02,
                                  land_retry_window_s=0.05)

        async def go():
            await alpha.connect()
            await alpha.takeoff(80)
            await bravo.connect()
            alpha.latency_s = 0.3                  # slow descent holds LAND slot
            land_task = asyncio.create_task(safety.land(alpha, "alpha"))
            await asyncio.sleep(0.05)              # alpha is mid-landing
            t0 = time.perf_counter()
            async with safety.launch_slot("bravo"):   # different slot
                await bravo.takeoff(80)
            elapsed = time.perf_counter() - t0
            await land_task
            return elapsed

        elapsed = asyncio.run(go())
    assert elapsed < 0.2, (
        f"launch waited {elapsed:.3f} s behind a LANDING — the two slots must "
        f"be independent (no shared semaphore / deadlock)")


# ============================================================
# 4. ADVISORY SECTORS — SectorGuard is advisory only, never control
# ============================================================
def _telem_at(north_m, east_m):
    return Telemetry(ts=100.0, position_m=(north_m, east_m, 1.0),
                     position_quality=PositionQuality.DEAD_RECKONING)


def _gctx(north_m, east_m, drone_id="alpha"):
    return GuardContext(drone_id=drone_id, now=100.0, mission_elapsed_s=10.0,
                        telemetry=_telem_at(north_m, east_m))


def test_sector_guard_quiet_inside_wedge():
    # C2 at origin; wedge centred north (0 deg), +/-30 deg. A point due north
    # is dead centre -> inside -> no trip.
    g = SectorGuard(c2_origin_m=(0.0, 0.0), sector_center_deg=0.0,
                    sector_half_width_deg=30.0)
    assert g.check(_gctx(10.0, 0.0)) is None


def test_sector_guard_trips_advisory_outside_wedge():
    # A point due EAST is at bearing -90 deg from C2 — well outside a
    # north-centred +/-30 deg wedge.
    g = SectorGuard(c2_origin_m=(0.0, 0.0), sector_center_deg=0.0,
                    sector_half_width_deg=30.0)
    trip = g.check(_gctx(0.0, 10.0))
    assert trip is not None
    # The load-bearing invariant: ADVISORY ONLY. Never a land / hold / degrade.
    assert trip.action is TripAction.ADVISORY
    assert trip.action < TripAction.HOLD_THIS    # strictly below any control
    assert "ADVISORY ONLY" in trip.reason


def test_sector_guard_edge_triggered_one_per_episode():
    g = SectorGuard(c2_origin_m=(0.0, 0.0), sector_center_deg=0.0,
                    sector_half_width_deg=30.0)
    assert g.check(_gctx(0.0, 10.0)) is not None    # first excursion -> trip
    assert g.check(_gctx(0.0, 10.0)) is None        # still out -> no spam
    assert g.check(_gctx(10.0, 0.0)) is None         # back inside -> re-arm
    assert g.check(_gctx(0.0, 10.0)) is not None    # new excursion -> trip


def test_sector_guard_skips_on_no_position():
    g = SectorGuard(c2_origin_m=(0.0, 0.0), sector_center_deg=0.0,
                    sector_half_width_deg=30.0)
    # No position_m -> skip (never guess).
    gctx = GuardContext(drone_id="alpha", now=100.0, mission_elapsed_s=10.0,
                        telemetry=Telemetry(ts=100.0))
    assert g.check(gctx) is None
    # No telemetry at all -> skip.
    gctx2 = GuardContext(drone_id="alpha", now=100.0, mission_elapsed_s=10.0)
    assert g.check(gctx2) is None


def test_sector_guard_at_c2_is_inside_every_sector():
    # The shared origin belongs to every drone's wedge (where they all boot).
    g = SectorGuard(c2_origin_m=(1.0, 5.0), sector_center_deg=120.0,
                    sector_half_width_deg=10.0)
    assert g.check(_gctx(1.0, 5.0)) is None


def test_sector_guard_rejects_bad_wedge_on_construction():
    with pytest.raises(Exception):     # ConfigError from in_sector
        SectorGuard(c2_origin_m=(0.0, 0.0), sector_center_deg=0.0,
                    sector_half_width_deg=-1.0)
    with pytest.raises(ValueError, match="c2_origin_m"):
        SectorGuard(c2_origin_m=(0.0,), sector_center_deg=0.0,
                    sector_half_width_deg=30.0)


def test_sector_guard_through_evaluate_guards_is_advisory():
    """Even through the evaluate_guards wrapper an outside-sector trip is
    ADVISORY — the agent maps ADVISORY to a logged event, never a land."""
    g = SectorGuard(c2_origin_m=(0.0, 0.0), sector_center_deg=0.0,
                    sector_half_width_deg=30.0)
    trips = evaluate_guards([g], _gctx(0.0, 10.0),
                            error_action=TripAction.LAND_THIS)
    assert len(trips) == 1
    assert trips[0].action is TripAction.ADVISORY


# ============================================================
# 5. Advisory sector NEVER lands the drone (end-to-end through the agent)
# ============================================================
def test_sector_advisory_does_not_force_land(run_dir):
    """Drive a full mission with a SectorGuard whose wedge the drone's
    dead-reckoned estimate leaves: the drone must complete its phase normally
    (sector is advisory; the agent logs guard_trip but never lands on it)."""
    from finals.events import read_events
    import os
    # A FORWARD move at yaw 0 takes the drone due NORTH; put the wedge to the
    # SOUTH so the estimate is guaranteed outside it the whole flight.
    adapter = MockAdapter("alpha", latency_s=0.001)
    sector = SectorGuard(c2_origin_m=(0.0, 0.0), sector_center_deg=180.0,
                         sector_half_width_deg=10.0)
    with EventLog(run_dir) as events:
        from finals.types import Move, Direction
        phase = ScriptedPhase([Takeoff(80),
                               Move(Direction.FORWARD, 100),  # drift north
                               Move(Direction.FORWARD, 100),
                               Land()])
        agent = DroneAgent("alpha", adapter, [phase], events,
                           guards=[sector])

        async def go():
            await agent.run(deadline=time.monotonic() + 30.0,
                            stop_event=asyncio.Event())

        asyncio.run(go())

    assert agent.state is AgentState.DONE        # NOT failed, NOT force-landed
    assert names(adapter.calls).count("emergency_land") == 0
    trips = [e for e in read_events(os.path.join(run_dir, "mission.jsonl"))
             if e["event"] == "guard_trip"]
    assert any(t["data"]["guard"] == "SectorGuard" for t in trips)
    assert all(t["data"]["action"] == "ADVISORY"
               for t in trips if t["data"]["guard"] == "SectorGuard")


# ============================================================
# 6. FAILURE INJECTION — the S4 property survives the launch slot
# ============================================================
def test_one_drone_fails_at_launch_others_finish_exactly_once_emergency(run_dir):
    """With the launch slot wired through SafetyController, one drone failing
    its takeoff must NOT block the others (independent slots, released in
    finally) and must emergency-land EXACTLY once (the S4 latch)."""
    ids = ["alpha", "bravo", "charlie"]
    adapters = {
        "alpha": MockAdapter("alpha", latency_s=0.001),
        "bravo": MockAdapter("bravo", latency_s=0.001, fail_at="takeoff:1"),
        "charlie": MockAdapter("charlie", latency_s=0.001),
    }
    with EventLog(run_dir) as events:
        safety = SafetyController(events, land_retry_period_s=0.02,
                                  land_retry_window_s=0.05)
        agents = {
            i: DroneAgent(i, adapters[i],
                          [ScriptedPhase([Takeoff(80), Wait(0.01), Land()])],
                          events, safety=safety)
            for i in ids}
        orch = Orchestrator(list(agents.values()), events, run_dir,
                            budget_s=30.0, heartbeat_period_s=0.02)
        code = asyncio.run(orch.run())

    assert code == 1                                       # one FAILED
    assert agents["bravo"].state is AgentState.FAILED
    assert names(adapters["bravo"].calls).count("emergency_land") == 1
    # The others, behind bravo in the launch corridor, still complete:
    for i in ("alpha", "charlie"):
        assert agents[i].state is AgentState.DONE, (
            f"{i} did not finish — bravo's launch failure stranded the "
            f"corridor (slot not released on failure)")
        assert names(adapters[i].calls).count("emergency_land") == 0
