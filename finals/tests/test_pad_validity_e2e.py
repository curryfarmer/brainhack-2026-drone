"""PAD-VALID end-to-end — 3 agents over MockAdapter through the REAL
Orchestrator + the REAL LandOnPad phase, sharing ONE PadValidityMap.

Proves the two cross-drone landing properties on the live wiring (not just the
unit-level map):

  1. INVALID broadcast: when drone A reads an INVALID (red) beacon, that verdict
     is published to the shared map (heartbeat pad_validity.invalid_ids) and NO
     drone ever CLAIMS / lands on that pad.

  2. NO DOUBLE-CLAIM: when all three drones see the SAME single valid pad, the
     map grants the claim to EXACTLY ONE drone — two drones never own one pad.

Mirrors test_orchestrator.py (the 3-agent orchestrator harness) and
test_land_on_pad.py (the scripted-sighting bus + takeoff-then-land fleet). Pure:
stdlib + pytest, no cv2/numpy (LandOnPad + PadValidityMap are pure).
"""
from __future__ import annotations

import asyncio
import json
import os
import time

from finals.events import EventLog
from finals.flight.mock_adapter import MockAdapter
from finals.mission.agent import AgentState, DroneAgent
from finals.mission.orchestrator import Orchestrator
from finals.mission.pad_validity import PadValidityMap
from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases.land_on_pad import LandOnPad
from finals.sightings import SightingBus
from finals.types import Action, Done, Sighting, Takeoff


# ---------------- harness ----------------------------------------------------
def _sighting(marker_id, *, drone_id, cx=320.0, cy=240.0, half=20.0,
              frame=(480, 640)) -> Sighting:
    return Sighting(
        drone_id=drone_id, ts=time.monotonic(), source="aruco",
        class_name=f"aruco_{marker_id}", marker_id=marker_id,
        bbox_xyxy=(cx - half, cy - half, cx + half, cy + half),
        confidence=1.0, frame_shape=frame)


class _ScriptedPadBus(SightingBus):
    """Surfaces a FIXED set of CENTRED sightings to each drone every tick (models
    a perception loop that frames the pad(s) dead-centre on every frame). The
    real per-tick drain shape — each agent calls drain_after(cursor, drone_id)
    and we hand back that drone's scripted frame, advancing its cursor by one."""

    def __init__(self, by_drone):
        super().__init__()
        # drone_id -> list[marker_id] visible to it every frame.
        self._by_drone = by_drone

    def drain_after(self, seq, drone_id=None):
        ids = self._by_drone.get(drone_id, [])
        frame = [_sighting(mid, drone_id=drone_id or "?") for mid in ids]
        return seq + 1, frame


class _Takeoff(MissionPhase):
    """Lift off once -> Done (NOT registered; test_conventions pins the registry
    exactly). land_on_pad assumes the drone is already airborne."""

    name = "_e2e_takeoff"

    def __init__(self, height_cm=200):
        self._done = False
        self._h = height_cm

    def step(self, ctx: AgentContext) -> Action:
        if not self._done:
            self._done = True
            return Takeoff(height_cm=self._h)
        return Done("airborne over the pad vicinity")


def _land(validity_map, *, valid_ids):
    """A LandOnPad tuned for a fast deterministic E2E: small acquire window,
    big descend step so the winner reaches the commit floor quickly, short
    timeouts so the losers Fallback-land promptly."""
    return LandOnPad(
        valid_marker_ids=list(valid_ids), k_lateral=1.0, tol_px=30.0,
        min_step_cm=5, max_step_cm=50, descend_step_cm=50,
        descend_persist_frames=2, center_persist_frames=2,
        acquire_window_frames=3, acquire_min_hits=2, commit_alt_m=0.5,
        acquire_timeout_s=3.0, total_budget_s=6.0, max_loss_retries=2,
        acquire_scan_step_deg=30.0, scan_dwell_s=0.001,
        validity_map=validity_map)


def _read_heartbeat(run_dir):
    with open(os.path.join(run_dir, "heartbeat.json"), encoding="utf-8") as f:
        return json.load(f)


def _run(agents, run_dir, events, validity_map, *, budget_s=15.0):
    orch = Orchestrator(agents, events, run_dir, budget_s=budget_s,
                        heartbeat_period_s=0.01, settle_grace_s=10.0,
                        validity_map=validity_map)
    return asyncio.run(orch.run())


# ============================================================
# 1. INVALID broadcast — A reads a red beacon; nobody lands on it
# ============================================================
def test_invalid_beacon_broadcast_and_never_claimed(tmp_path):
    """Drone A sees ONLY an invalid beacon (99, not in the valid set); B and C
    see distinct valid pads (11, 51). The shared map must (a) record 99 as
    broadcast-INVALID (heartbeat invalid_ids), and (b) NEVER claim 99 for any
    drone — the red pad is excluded from every drone's centering."""
    run_dir = str(tmp_path)
    vmap = PadValidityMap()
    valid_ids = [11, 51, 67, 101]            # 99 is NOT here -> invalid/red
    bus = _ScriptedPadBus({"alpha": [99], "bravo": [11], "charlie": [51]})

    with EventLog(run_dir) as events:
        agents = [
            DroneAgent(d, MockAdapter(d), [_Takeoff(200),
                                           _land(vmap, valid_ids=valid_ids)],
                       events, bus=bus)
            for d in ("alpha", "bravo", "charlie")]
        _run(agents, run_dir, events, vmap)

    # The red beacon was BROADCAST invalid by alpha and skipped by all.
    assert vmap.is_valid(99) is False
    assert vmap.claimed_by(99) is None, "no drone may claim a red pad"

    hb = _read_heartbeat(run_dir)
    pv = hb["pad_validity"]
    assert 99 in pv["invalid_ids"]                       # the broadcast is live
    assert "99" not in pv["claimed_by"]                  # never claimed
    # The two valid pads were each claimed by their (only) seer — distinct.
    assert vmap.claimed_by(11) == "bravo"
    assert vmap.claimed_by(51) == "charlie"


# ============================================================
# 2. NO DOUBLE-CLAIM — three drones, ONE valid pad, one winner
# ============================================================
def test_three_drones_one_valid_pad_exactly_one_claims(tmp_path):
    """All three drones see the SAME single valid pad (11). The shared map's
    single-winner CAS grants the claim to EXACTLY ONE drone — the other two
    never own it (the loser re-targets / Fallback-lands)."""
    run_dir = str(tmp_path)
    vmap = PadValidityMap()
    bus = _ScriptedPadBus({"alpha": [11], "bravo": [11], "charlie": [11]})

    with EventLog(run_dir) as events:
        agents = [
            DroneAgent(d, MockAdapter(d), [_Takeoff(200),
                                           _land(vmap, valid_ids=[11])],
                       events, bus=bus)
            for d in ("alpha", "bravo", "charlie")]
        code = _run(agents, run_dir, events, vmap)

    # Exactly one owner of pad 11 — two drones NEVER own one pad.
    owner = vmap.claimed_by(11)
    assert owner in ("alpha", "bravo", "charlie")

    hb = _read_heartbeat(run_dir)
    claimed = hb["pad_validity"]["claimed_by"]
    assert claimed == {"11": owner}, f"exactly one claimant, got {claimed}"
    assert list(claimed.values()).count(owner) == 1
    # The run is well-formed (every agent reached a terminal state, no hang).
    assert code in (0, 1)
    assert all(a.state in (AgentState.DONE, AgentState.FAILED) for a in agents)


# ============================================================
# 3. Two valid pads, contended pick -> the two seers split them
# ============================================================
def test_two_drones_two_valid_pads_no_shared_claim(tmp_path):
    """Two drones BOTH see BOTH valid pads (11, 51). Each commits to its
    deterministic _pick_target; the claim CAS guarantees they cannot both own
    the same pad — the two claims are DISJOINT."""
    run_dir = str(tmp_path)
    vmap = PadValidityMap()
    bus = _ScriptedPadBus({"alpha": [11, 51], "bravo": [11, 51]})

    with EventLog(run_dir) as events:
        agents = [
            DroneAgent(d, MockAdapter(d), [_Takeoff(200),
                                           _land(vmap, valid_ids=[11, 51])],
                       events, bus=bus)
            for d in ("alpha", "bravo")]
        _run(agents, run_dir, events, vmap)

    owner_11 = vmap.claimed_by(11)
    owner_51 = vmap.claimed_by(51)
    # No pad is owned by two drones (the core property): each pad has at most one
    # owner, and the same drone never owns BOTH while the other owns NEITHER...
    owners = [o for o in (owner_11, owner_51) if o is not None]
    # ...and crucially the two owners differ when both pads are claimed.
    if owner_11 is not None and owner_51 is not None:
        assert owner_11 != owner_51, "two drones split two pads, never share one"
    # Whatever the outcome, no pad maps to more than one drone (by construction
    # of claim(), re-asserted via the heartbeat).
    hb = _read_heartbeat(run_dir)
    claimed = hb["pad_validity"]["claimed_by"]
    assert len(set(claimed.values())) == len(claimed), \
        f"a drone appears twice in claimed_by — double claim: {claimed}"


# ============================================================
# 4. CROSS-DRONE is_valid propagation — one drone's INVALID broadcast steers
#    ANOTHER drone off a pad its OWN static set would have accepted
# ============================================================
def _ctx(drone_id, sightings):
    from finals.types import Telemetry
    return AgentContext(
        drone_id=drone_id, now=100.0, mission_elapsed_s=0.0,
        telemetry=Telemetry(ts=100.0, altitude_m=2.0, is_flying=True),
        sightings=sightings, last_action=None, last_action_ok=None,
        last_action_error=None)


def test_invalid_broadcast_drops_a_statically_valid_pad_for_another_drone():
    """The cross-drone propagation that ONLY is_valid provides: drone A has read
    beacon 51 and broadcast it INVALID into the shared map (e.g. its own valid
    set, or a mid-flight re-classification, marked it red). Drone B's land_on_pad
    has 51 in its STATIC valid_marker_ids — yet B must STILL drop 51, because the
    shared map says it is invalid. This is the exact path the 'is_valid ignores
    record' mutant breaks (with that mutant is_valid->None, B would happily
    centre on the red pad). Driven at the _valid_sightings seam (the method
    PAD-VALID owns) so the kill is unambiguous."""
    vmap = PadValidityMap()
    # Drone A broadcasts beacon 51 INVALID.
    vmap.record(51, valid=False, drone_id="alpha", ts=100.0)

    # Drone B: 51 IS in its static set, but the shared invalid broadcast wins.
    land_b = LandOnPad(valid_marker_ids=[51], k_lateral=1.0, tol_px=30.0,
                       min_step_cm=5, max_step_cm=50, descend_step_cm=30,
                       descend_persist_frames=2, center_persist_frames=2,
                       acquire_window_frames=3, acquire_min_hits=2,
                       commit_alt_m=0.5, acquire_timeout_s=5.0,
                       total_budget_s=10.0, max_loss_retries=2,
                       acquire_scan_step_deg=30.0, scan_dwell_s=0.01,
                       validity_map=vmap)
    kept = land_b._valid_sightings(
        _ctx("bravo", [_sighting(51, drone_id="bravo")]))
    assert kept == [], (
        "B must drop the pad A broadcast invalid, even though 51 is in B's "
        "static valid_marker_ids — the cross-drone is_valid broadcast governs")
    assert vmap.claimed_by(51) is None, "a red pad is never claimed"


def test_unread_valid_pad_still_landable_via_static_set():
    """The paired guard: a beacon NO drone has broadcast (is_valid -> None) must
    NOT be dropped — it stays landable via the static set. This pins that the
    invalid-drop logic keys on `is False`, not on falsy/None (so the cross-drone
    test above is a real invalid-propagation kill, not an over-eager drop)."""
    vmap = PadValidityMap()                     # nothing recorded
    land = LandOnPad(valid_marker_ids=[51], k_lateral=1.0, tol_px=30.0,
                     min_step_cm=5, max_step_cm=50, descend_step_cm=30,
                     descend_persist_frames=2, center_persist_frames=2,
                     acquire_window_frames=3, acquire_min_hits=2,
                     commit_alt_m=0.5, acquire_timeout_s=5.0,
                     total_budget_s=10.0, max_loss_retries=2,
                     acquire_scan_step_deg=30.0, scan_dwell_s=0.01,
                     validity_map=vmap)
    s = _sighting(51, drone_id="bravo")
    kept = land._valid_sightings(_ctx("bravo", [s]))
    assert kept == [s], "an unread (None) valid pad must remain landable"
    assert vmap.claimed_by(51) == "bravo", "and it gets claimed by the seer"
