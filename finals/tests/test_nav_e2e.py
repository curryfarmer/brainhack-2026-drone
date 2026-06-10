"""End-to-end Challenge-2A LANDING-mission integration over the MockAdapter.

The layer ABOVE the per-phase unit tests (test_takeoff / test_navigate /
test_land_on_pad) and the orchestrator gate (test_orchestrator): it drives the
WHOLE [takeoff, navigate, land_on_pad] pipeline — real phase objects built from
real config via main._build_phases — over the scriptable MockAdapter (NO
hardware, NO SDK, NO gz, NO cv2). The single headline proof: the three landing
phases CHAIN into a completed, LANDED mission, and multi-drone deconfliction
(staggered launch + serialized descent) holds.

Why this is honest and not a per-phase re-test:
- The MockAdapter integrates every commanded Move/Rotate/Takeoff/Land through
  the SHARED DeadReckoner (flight/dead_reckon.py — the same oracle the navigate
  unit test flies), so the open-loop transit really closes the gap to the pad
  and the DOWN-steps really drop the reported altitude past commit_alt_m. The
  pipeline lands because the math closes, not because a stub said so.
- Phases are built by main._build_phases(drone_cfg, cfg) from JSON config — the
  exact wiring path the real mission uses — so a from_config / planner / arena
  regression goes red here too.

Position stays position-blind throughout: the navigate transit re-orients to
absolute compass headings (yaw only) and NEVER trusts position_m for control;
land_on_pad servos on camera Sightings + the ToF altitude. We assert the
telemetry quality that actually flowed is never better than DEAD_RECKONING
(i.e. no MEASURED position was ever needed) — the position-blind contract.

stdlib + pytest only (the phases are PURE; the suite runs in a bare venv with
no cv2/numpy). Mirrors the canned-Sighting-bus harness of test_land_on_pad and
the failure-injection harness of test_orchestrator — it does not reinvent them.

Run: python -m pytest finals/tests/test_nav_e2e.py -q -p no:randomly
"""
from __future__ import annotations

import asyncio
import json
import os
import time

import pytest

from finals.config import load_config
from finals.errors import PlanningError
from finals.events import EventLog, read_events
from finals.flight.mock_adapter import MockAdapter
from finals.main import _build_guards, _build_phases, _build_safety
from finals.mission.agent import AgentState, DroneAgent
from finals.mission.orchestrator import Orchestrator
from finals.sightings import SightingBus
from finals.types import Direction, PositionQuality, Sighting

_CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs")
_LANDING_REAL = os.path.join(_CONFIG_DIR, "landing_real.json")


# ============================================================
# Shared canned-perception helpers (TEST-ONLY; no production code touched).
# Mirrors test_land_on_pad._sighting / _CentredPadBus.
# ============================================================
def _centred_sighting(marker_id: int, *, drone_id: str, cx: float = 320.0,
                      cy: float = 240.0, half: float = 20.0,
                      frame=(480, 640)) -> Sighting:
    """A valid, dead-centre ArUco Sighting of `marker_id` for `drone_id`."""
    return Sighting(
        drone_id=drone_id, ts=time.monotonic(), source="aruco",
        class_name=f"aruco_{marker_id}", marker_id=marker_id,
        bbox_xyxy=(cx - half, cy - half, cx + half, cy + half),
        confidence=1.0, frame_shape=frame)


class _PadInRangeBus(SightingBus):
    """Surfaces ONE fresh CENTRED valid sighting per drain ONLY once the drone
    is within `near_m` (horizontal dead-reckon distance) of the pad — i.e. the
    pad swims into the onboard FOV after navigate has flown the transit. Before
    then the bus is empty, so land_on_pad runs its real PAD_ACQUIRE scan.

    Each drone gets its OWN marker id + adapter (so the DR pose is per-drone).
    Synthesizing per-drain (rather than pre-seeding the real bus, which would
    drain everything at once) is the test_land_on_pad._CentredPadBus pattern,
    extended with a position gate so navigate is genuinely exercised first.
    """

    def __init__(self, *, marker_by_drone, adapter_by_drone, pad_ne_by_drone,
                 near_m: float = 0.6, maxlen: int = 500):
        super().__init__(maxlen=maxlen)
        self._marker_by_drone = dict(marker_by_drone)
        self._adapter_by_drone = dict(adapter_by_drone)
        self._pad_ne_by_drone = dict(pad_ne_by_drone)
        self._near_m = float(near_m)

    def _within_range(self, drone_id: str) -> bool:
        adapter = self._adapter_by_drone.get(drone_id)
        pad = self._pad_ne_by_drone.get(drone_id)
        if adapter is None or pad is None:
            return False
        pose = adapter.dr.pose
        dn, de = pose.north_m - pad[0], pose.east_m - pad[1]
        return (dn * dn + de * de) ** 0.5 <= self._near_m

    def drain_after(self, seq, drone_id=None):
        if drone_id is None or drone_id not in self._marker_by_drone:
            return seq + 1, []          # orchestrator-wide drain: log nothing
        if not self._within_range(drone_id):
            return seq + 1, []          # pad not yet in FOV — real scan runs
        marker = self._marker_by_drone[drone_id]
        return seq + 1, [_centred_sighting(marker, drone_id=drone_id)]


# ============================================================
# Config helpers — a CONTROLLED single-drone landing config whose C2 origin is
# (0, 0), so the per-drone DeadReckoner pose (which boots at (0,0)) maps 1:1
# onto arena north/east and we can assert "navigate closed the transit to the
# pad centre" directly. Built + loaded the REAL way (load_config + _build_*).
# ============================================================
def _write_controlled_config(tmp_path, *, pad_ne=(0.0, 4.0), marker_id=7,
                             keep_out=None, nav_budget_s=120.0,
                             goal_override=None, land_zone_extra=None,
                             gates=None, nav_inflation_m=0.5):
    """Write a temp arena + landing config (profile mock) and return its path.
    pad 'pad_t' sits at pad_ne; C2 origin at (0,0) facing north so DR == arena.
    A crate may be injected via keep_out to force a detour. goal_override
    replaces the navigate goal (e.g. an unreachable coord). gates injects
    NAV-ARCH arch openings (the arena loader validates them against keep_out).
    nav_inflation_m sizes the planner safety margin (gate fit = clearance >=
    2*inflation)."""
    arena = {
        "bounds_m": [-10.0, -10.0, 10.0, 10.0],
        "c2_origin_m": [0.0, 0.0],
        "c2_heading_deg": 0.0,
        "keep_out": list(keep_out or []),
        "pads": [{"id": "pad_t", "center_m": list(pad_ne), "radius_m": 0.3,
                  "valid": True}],
        "lanes": [],
        "gates": list(gates or []),
    }
    arenas_dir = tmp_path / "arenas"
    arenas_dir.mkdir(exist_ok=True)
    (arenas_dir / "ctrl.json").write_text(json.dumps(arena), encoding="utf-8")

    nav_zone = dict(goal_override) if goal_override is not None \
        else {"pad_id": "pad_t"}
    nav_zone.update({"inflation_m": nav_inflation_m, "max_leg_cm": 100.0,
                     "heading_tol_deg": 1.0, "max_step_deg": 180.0,
                     "total_budget_s": nav_budget_s})
    land_zone = {"valid_marker_ids": [marker_id], "commit_alt_m": 0.5,
                 "descend_step_cm": 50, "center_persist_frames": 2,
                 "descend_persist_frames": 2, "acquire_min_hits": 2,
                 "acquire_window_frames": 3, "scan_dwell_s": 0.001,
                 "acquire_timeout_s": 30.0, "total_budget_s": 90.0}
    land_zone.update(land_zone_extra or {})
    cfg = {
        "profile": "mock", "flight_backend": "mock", "frame_backend": "none",
        "command_timeout_s": 15, "mission_budget_s": 600,
        "marker_backend": "aruco", "arena_name": "ctrl",
        "detector": {"backend": "none"},
        "guards": {"landing_reserve_s": 0, "geofence_radius_m": 14.0},
        "drones": [{
            "id": "alpha", "phases": ["takeoff", "navigate", "land_on_pad"],
            "sector_deg": [0.0, 60.0],
            "zone": {"takeoff": {"height_cm": 200}, "navigate": nav_zone,
                     "land_on_pad": land_zone},
        }],
    }
    path = tmp_path / "ctrl_landing.json"
    path.write_text(json.dumps(cfg), encoding="utf-8")
    return str(path)


def _run_agent_to_completion(agent, *, budget_s=30.0, clock=time.monotonic):
    """Drive one agent's whole life synchronously, then shut it down. No sleeps,
    no wallclock races — the agent loop steps pure phases over the mock. The
    deadline is in the agent's OWN clock domain (`clock`), so a fake clock and
    the mission deadline stay consistent."""
    async def go():
        stop = asyncio.Event()
        await agent.run(deadline=clock() + budget_s, stop_event=stop)
        await agent.shutdown()
    asyncio.run(go())


def _names(calls):
    return [c[0] for c in calls]


def _mission_events(run_dir):
    return list(read_events(os.path.join(run_dir, "mission.jsonl")))


def _drone_events(run_dir, drone_id):
    """All events for one drone (read from mission.jsonl, filtered by id — the
    per-drone files are named drone_<id>.jsonl, but mission.jsonl carries the
    full ordered story and is the single source of truth here)."""
    return [e for e in _mission_events(run_dir) if e["drone"] == drone_id]


# ============================================================
# 1. SINGLE-DRONE FULL MISSION — the headline: the pipeline LANDS.
# ============================================================
def test_full_landing_mission_chains_three_phases_to_a_verified_landing(tmp_path):
    """[takeoff, navigate, land_on_pad] over the MockAdapter, end to end:
      * the three phases execute IN ORDER (phase_enter events in sequence);
      * navigate flies the planned legs so the dead-reckon pose ends within
        tolerance of the pad centre (open-loop transit closes);
      * land_on_pad ACQUIRES the valid marker, CENTERS, DESCENDS, and COMMITS a
        Land (the canned bus surfaces the pad only once the drone is in range);
      * the drone ends is_flying False at alt ~ 0 and the agent reaches DONE.
    The pad sits at arena (0, 4); C2 at (0, 0) facing north, so the DR pose maps
    1:1 onto arena north/east and the landing-spot assertion is exact."""
    cfg_path = _write_controlled_config(tmp_path, pad_ne=(0.0, 4.0), marker_id=7)
    cfg = load_config(cfg_path)
    drone = cfg.drones[0]
    phases = _build_phases(drone, cfg)
    assert [p.name for p in phases] == ["takeoff", "navigate", "land_on_pad"]

    adapter = MockAdapter("alpha")
    bus = _PadInRangeBus(marker_by_drone={"alpha": 7},
                         adapter_by_drone={"alpha": adapter},
                         pad_ne_by_drone={"alpha": (0.0, 4.0)})
    with EventLog(str(tmp_path)) as events:
        agent = DroneAgent("alpha", adapter, phases, events, bus=bus,
                           safety=_build_safety(cfg, events))
        _run_agent_to_completion(agent, budget_s=60.0)

    # Phases ran IN ORDER (one phase_enter per phase, in sequence).
    evs = _drone_events(str(tmp_path), "alpha")
    entered = [e["data"]["phase"] for e in evs if e["event"] == "phase_enter"]
    assert entered == ["takeoff", "navigate", "land_on_pad"]

    # The pipeline LANDED: terminal success + grounded at alt ~ 0.
    assert agent.state is AgentState.DONE
    assert not adapter.is_flying
    assert adapter.dr.pose.alt_m == pytest.approx(0.0, abs=1e-9)

    # Command order proves the chain: takeoff -> transit moves -> a real Land.
    names = _names(adapter.calls)
    assert names[0] == "connect"
    assert "takeoff" in names and "land" in names
    assert names.index("takeoff") < names.index("land")
    assert any(n == "move" for n in names)      # transit + descend moves flew

    # navigate CLOSED the open-loop transit: DR horizontal pose ~ the pad centre
    # (land_on_pad's only horizontal commands are lateral centering, and the
    # canned sighting is dead-centre, so the horizontal pose stays at the
    # navigate endpoint). Tolerance is the inflation + heading-tol drift budget.
    assert adapter.dr.pose.north_m == pytest.approx(0.0, abs=0.2)
    assert adapter.dr.pose.east_m == pytest.approx(4.0, abs=0.2)

    # land_on_pad reached a VERIFIED landing (the success funnel, not Fallback).
    land_phase = phases[2]
    assert land_phase._fallback_reason is None
    done = [e for e in evs if e["event"] == "agent_done"]
    assert done and done[0]["data"]["phases_completed"] == 3

    # Position-blind: nothing better than DEAD_RECKONING was ever required.
    assert adapter.telemetry().position_quality <= PositionQuality.DEAD_RECKONING


def test_full_landing_mission_descends_in_steps_then_commits(tmp_path):
    """The descent is REAL, not a teleport: land_on_pad commands a sequence of
    Move(DOWN) steps that drop the reported altitude from the takeoff height
    (2.0 m) down across commit_alt_m (0.5 m), then a final blind Land. Proves
    the LAND_COMMIT funnel (alt <= commit) is what ends it, before any budget
    Fallback — kills a 'lands by timeout' false positive."""
    cfg_path = _write_controlled_config(tmp_path, pad_ne=(0.0, 3.0), marker_id=9)
    cfg = load_config(cfg_path)
    cfg.drones[0].zone["land_on_pad"]["valid_marker_ids"] = [9]
    phases = _build_phases(cfg.drones[0], cfg)

    adapter = MockAdapter("alpha")
    bus = _PadInRangeBus(marker_by_drone={"alpha": 9},
                         adapter_by_drone={"alpha": adapter},
                         pad_ne_by_drone={"alpha": (0.0, 3.0)})
    with EventLog(str(tmp_path)) as events:
        agent = DroneAgent("alpha", adapter, phases, events, bus=bus,
                           safety=_build_safety(cfg, events))
        _run_agent_to_completion(agent, budget_s=60.0)

    assert agent.state is AgentState.DONE
    # At least one DOWN move flew (the genuine descent), then a Land committed.
    down_moves = [c for c in adapter.calls
                  if c[0] == "move" and c[1].get("direction") is Direction.DOWN]
    assert down_moves, "expected real Move(DOWN) descent steps"
    assert "land" in _names(adapter.calls)
    assert not adapter.is_flying


# ============================================================
# 1b. NAV-ARCH E2E — the FULL landing mission FLIES THROUGH AN ARCH GATE.
#     Loads an arena JSON WITH a gate via the real load_config (so the
#     from_dict gate validation + the planner gate exemption both run on the
#     mission path) and chains [takeoff, navigate, land_on_pad] to a landing.
# ============================================================
def test_full_mission_through_an_arch_gate_lands_on_the_pad(tmp_path):
    """The arch is a SOLID block (north 1..3, east -2..2) the drone canNOT
    overfly; a Gate carves the doorway (east -0.5..0.5). The pad sits BEYOND the
    arch at (4, 0). The mission must takeoff, navigate THROUGH the gate to the
    pad vicinity (a straight north shot the gate reopens; without it the block
    would force a detour), then land. Proves the whole from_dict-validated,
    gate-aware planner -> phase -> mock-flight chain closes into a landing."""
    arch = {"id": "arch_solid",
            "polygon_m": [[1.0, -2.0], [1.0, 2.0], [3.0, 2.0], [3.0, -2.0]]}
    gate = {"id": "arch1", "span_m": [[2.0, -0.5], [2.0, 0.5]],
            "clearance_m": 1.0}
    cfg_path = _write_controlled_config(
        tmp_path, pad_ne=(4.0, 0.0), marker_id=7, keep_out=[arch],
        gates=[gate], nav_inflation_m=0.2)
    cfg = load_config(cfg_path)
    assert [g.id for g in cfg.arena.gates] == ["arch1"]   # gate survived load
    phases = _build_phases(cfg.drones[0], cfg)

    # navigate threaded the gate: a straight north shot (all legs share heading),
    # NOT a detour — the gate exemption opened the block on the mission path.
    nav = phases[1]
    assert len({round(l.heading_deg, 6) for l in nav._legs}) == 1

    adapter = MockAdapter("alpha")
    bus = _PadInRangeBus(marker_by_drone={"alpha": 7},
                         adapter_by_drone={"alpha": adapter},
                         pad_ne_by_drone={"alpha": (4.0, 0.0)}, near_m=0.8)
    with EventLog(str(tmp_path)) as events:
        agent = DroneAgent("alpha", adapter, phases, events, bus=bus,
                           safety=_build_safety(cfg, events))
        _run_agent_to_completion(agent, budget_s=60.0)

    assert agent.state is AgentState.DONE
    assert not adapter.is_flying
    # navigate closed the transit to the pad BEYOND the arch (DR == arena here).
    assert adapter.dr.pose.north_m == pytest.approx(4.0, abs=0.2)
    assert adapter.dr.pose.east_m == pytest.approx(0.0, abs=0.2)
    names = _names(adapter.calls)
    assert names.index("takeoff") < names.index("land")
    assert adapter.telemetry().position_quality <= PositionQuality.DEAD_RECKONING


# ============================================================
# 2a. NEGATIVE / SAFETY E2E — an unreachable navigate goal FAILS LOUD.
# ============================================================
def test_navigate_goal_in_keepout_fails_loud_at_build_not_silent_done(tmp_path):
    """A navigate goal trapped inside a keep-out must FAIL LOUD (PlanningError
    at phase-build), NOT silently Done a transit that never flew. This is the
    config-time fail-loud bar: a goal in a crate is an operator error to fix,
    never something the mission swallows."""
    box = {"id": "crateZ",
           "polygon_m": [[-2.0, 2.0], [-2.0, 6.0], [2.0, 6.0], [2.0, 2.0]]}
    cfg_path = _write_controlled_config(
        tmp_path, keep_out=[box],
        goal_override={"goal_ne_m": [0.0, 4.0], "inflation_m": 0.2})
    cfg = load_config(cfg_path)
    with pytest.raises(PlanningError, match="crateZ"):
        _build_phases(cfg.drones[0], cfg)


class _FakeClock:
    """Deterministic monotonic source (mirrors test_agent.FakeClock): tests
    advance .t explicitly so elapsed-time logic is exercised with ZERO
    wall-clock dependence."""

    def __init__(self, t: float = 100.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


def test_navigate_budget_too_small_aborts_mid_transit_not_silent_done(tmp_path):
    """A transit budget too small for the route makes navigate Abort (the drone
    goes FAILED + safe-down), never a silent Done. Driven by a SHARED FAKE CLOCK
    that a Move bumps +1.0 s on completion: navigate's 0.0001 s phase budget
    overruns deterministically on the 2nd step (no wall-clock race). The mission
    deadline (30 s of fake time) is far away, so it is the PHASE budget that
    Aborts, not the mission deadline (which would be a clean DONE). Asserts the
    agent FAILED with exactly ONE emergency_land — fail-loud, not silent Done."""
    cfg_path = _write_controlled_config(
        tmp_path, pad_ne=(8.0, 0.0), nav_budget_s=0.0001)
    cfg = load_config(cfg_path)
    phases = _build_phases(cfg.drones[0], cfg)

    clock = _FakeClock(100.0)

    class _BumpOnMove(MockAdapter):
        """Each completed move 'takes' 1.0 s on the shared fake clock — enough
        to overrun navigate's 0.0001 s budget, far under the 30 s mission one.
        Telemetry is re-stamped at the same clock each tick, so the staleness
        guard never trips first (it is the navigate budget under test)."""
        async def move(self, *a, **kw):
            await super().move(*a, **kw)
            clock.t += 1.0

    adapter = _BumpOnMove("alpha", clock=clock)
    with EventLog(str(tmp_path)) as events:
        agent = DroneAgent("alpha", adapter, phases, events, clock=clock,
                           safety=_build_safety(cfg, events))
        _run_agent_to_completion(agent, budget_s=30.0, clock=clock)

    assert agent.state is AgentState.FAILED
    assert agent.failure and "OVERRAN" in agent.failure
    names = _names(adapter.calls)
    assert names.count("emergency_land") == 1       # exactly-once safe-down
    assert not adapter.is_flying


# ============================================================
# 2b. NEGATIVE / SAFETY E2E — land_on_pad that NEVER acquires Fallback-LANDS.
# ============================================================
def test_land_on_pad_never_acquires_fallback_lands_never_hovers_to_death(tmp_path):
    """No sightings ever reach land_on_pad (empty bus): it must run its bounded
    PAD_ACQUIRE scan, hit acquire_timeout_s, and Fallback blind-LAND in place —
    NEVER a battery-dead hover. The drone ends on the ground, DONE, with an
    UNVERIFIED_LANDING reason (an orderly clean end, not a FAILED crash)."""
    cfg_path = _write_controlled_config(
        tmp_path, pad_ne=(0.0, 2.0), marker_id=7,
        land_zone_extra={"acquire_timeout_s": 0.05, "total_budget_s": 0.1,
                         "scan_dwell_s": 0.001})
    cfg = load_config(cfg_path)
    phases = _build_phases(cfg.drones[0], cfg)

    adapter = MockAdapter("alpha")
    empty_bus = SightingBus()                       # nothing is ever published
    with EventLog(str(tmp_path)) as events:
        agent = DroneAgent("alpha", adapter, phases, events, bus=empty_bus,
                           safety=_build_safety(cfg, events))
        _run_agent_to_completion(agent, budget_s=60.0)

    assert agent.state is AgentState.DONE           # orderly clean end
    assert not adapter.is_flying                    # on the ground
    assert adapter.dr.pose.alt_m == pytest.approx(0.0, abs=1e-9)
    assert "land" in _names(adapter.calls)          # a real Land committed
    land_phase = phases[2]
    assert land_phase._fallback_reason is not None
    assert "UNVERIFIED_LANDING" in land_phase._fallback_reason
    # NEVER an emergency_land (this is a clean fallback, not a failure path).
    assert "emergency_land" not in _names(adapter.calls)


# ============================================================
# 3. MULTI-DRONE DECONFLICTION over the orchestrator (landing_real.json, 3
#    drones, real phases + guards + the SafetyController slots; MockAdapters).
# ============================================================
def _build_landing_fleet(events, *, fail=None):
    """Build the 3-drone landing_real.json fleet over MockAdapters, with the
    REAL phases / per-drone guards / shared SafetyController + a per-drone
    in-range canned bus. `fail` = {drone_id: fail_at_string} injects a scripted
    mid-mission adapter failure. Returns (cfg, agents, adapters, bus)."""
    cfg = load_config(_LANDING_REAL)
    safety = _build_safety(cfg, events)
    pads_by_id = {p.id: p for p in cfg.arena.pads}
    fail = fail or {}

    adapters = {}
    marker_by_drone = {}
    pad_ne_by_drone = {}
    for d in cfg.drones:
        adapters[d.id] = MockAdapter(d.id, fail_at=fail.get(d.id))
        marker_by_drone[d.id] = d.zone["land_on_pad"]["valid_marker_ids"][0]
        pad_id = d.zone["navigate"]["pad_id"]
        # The arena pad centre, shifted into the per-drone DR frame (which boots
        # at (0,0)) by subtracting the C2 origin — so the in-range gate fires
        # when the dead-reckoned transit reaches the pad.
        c2 = cfg.arena.c2_origin_m
        center = pads_by_id[pad_id].center_m
        pad_ne_by_drone[d.id] = (center[0] - c2[0], center[1] - c2[1])

    bus = _PadInRangeBus(marker_by_drone=marker_by_drone,
                         adapter_by_drone=adapters, pad_ne_by_drone=pad_ne_by_drone,
                         near_m=0.8)
    agents = [DroneAgent(d.id, adapters[d.id], _build_phases(d, cfg), events,
                         bus=bus, guards=_build_guards(cfg, d), safety=safety)
              for d in cfg.drones]
    return cfg, agents, adapters, bus


def test_three_drone_landing_all_reach_done_with_serialized_corridors(tmp_path):
    """The full 3-drone Challenge-2A landing over the orchestrator: every drone
    flies takeoff -> navigate -> land_on_pad to its OWN valid pad and reaches
    terminal DONE. The SafetyController serializes the shared corridors:
      * launch — at most ONE drone in the C2 takeoff zone at a time
        (launch_slot_acquired / launch_slot_released never overlap);
      * landing — descent routes through the landing slot (route='safety').
    Exit code 0 (all DONE)."""
    run_dir = str(tmp_path)
    with EventLog(run_dir) as events:
        cfg, agents, adapters, bus = _build_landing_fleet(events)
        orch = Orchestrator(agents, events, run_dir,
                            budget_s=cfg.mission_budget_s, bus=bus,
                            heartbeat_period_s=0.02,
                            swarm_guards=[])
        code = asyncio.run(orch.run())

    assert code == 0
    for d_id, adapter in adapters.items():
        assert "land" in _names(adapter.calls)
        assert not adapter.is_flying
    assert all(a.state is AgentState.DONE for a in agents)

    # -- Staggered launch: the launch-corridor slot is held by AT MOST one
    # drone at a time. Replay the acquire/release events in monotonic order
    # (the within-run sequencing clock); the held count must never exceed 1. --
    all_evs = _mission_events(run_dir)
    launch_pairs = []
    for e in all_evs:
        if e["event"] == "launch_slot_acquired":
            launch_pairs.append((e["mono"], +1))
        elif e["event"] == "launch_slot_released":
            launch_pairs.append((e["mono"], -1))
    assert sum(1 for _, d in launch_pairs if d > 0) == 3   # all 3 launched
    launch_pairs.sort()
    held = 0
    for _mono, delta in launch_pairs:
        held += delta
        assert held <= 1, "two drones held the launch corridor at once"

    # -- Serialized descent: every drone's final Land routed through the
    # SafetyController landing slot (route='safety'), the one-descent-at-a-time
    # mechanism. --
    for d_id in adapters:
        evs = _drone_events(run_dir, d_id)
        safety_lands = [e for e in evs if e["event"] == "action_start"
                        and e["data"].get("action") == "Land"
                        and e["data"].get("route") == "safety"]
        assert safety_lands, f"{d_id} did not route its Land through the slot"


# ============================================================
# 4. FAILURE INJECTION — one drone fails mid-mission; the others complete.
#    Mirrors test_orchestrator.test_s4_gate_one_agent_fails_other_completes,
#    but over the FULL landing fleet (3 real-phase drones).
# ============================================================
def test_one_drone_fails_mid_mission_others_land_run_exits_clean(tmp_path):
    """In the 3-drone landing run, bravo's adapter is scripted to fail on its
    2nd move (fail_at='move:2', tripped during navigate's transit). bravo goes
    FAILED with EXACTLY ONE emergency_land; alpha + charlie complete their full
    landing pipeline and end DONE; the orchestrator exits cleanly with code 1
    (a drone failed). Failure isolation through the whole supervisor."""
    run_dir = str(tmp_path)
    with EventLog(run_dir) as events:
        cfg, agents, adapters, bus = _build_landing_fleet(
            events, fail={"bravo": "move:2"})
        orch = Orchestrator(agents, events, run_dir,
                            budget_s=cfg.mission_budget_s, bus=bus,
                            heartbeat_period_s=0.02)
        code = asyncio.run(orch.run())

    assert code == 1                                # one drone FAILED -> exit 1
    by_id = {a.drone_id: a for a in agents}

    # bravo: FAILED, emergency-landed EXACTLY once, on the ground.
    assert by_id["bravo"].state is AgentState.FAILED
    assert _names(adapters["bravo"].calls).count("emergency_land") == 1
    assert not adapters["bravo"].is_flying
    assert by_id["bravo"].failure

    # alpha + charlie: untouched by bravo's death — full pipeline, DONE,
    # a real Land, and NO emergency_land.
    for ok_id in ("alpha", "charlie"):
        assert by_id[ok_id].state is AgentState.DONE
        names = _names(adapters[ok_id].calls)
        assert "land" in names
        assert "emergency_land" not in names
        assert not adapters[ok_id].is_flying

    # The supervisor told the story: a run_start + run_end bracket the run.
    kinds = [e["event"] for e in _mission_events(run_dir)]
    assert "run_start" in kinds and "run_end" in kinds
