"""finals.mission.phases.land_on_pad — LandOnPad visual-servo landing.

Pure phase tests (the MissionPhase contract: step() with hand-built
AgentContexts driving canned Sighting streams + scripted telemetry), plus an
agent-over-MockAdapter integration where a takeoff phase lifts the drone and a
canned good approach centres-then-descends to a verified landing.

stdlib + pytest only (the phase is PURE; the suite runs in a bare venv —
no cv2/numpy). Mirrors the test_search.py MockAdapter / canned-stream harness.
"""
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
from finals.mission.phases.land_on_pad import LandOnPad, _SubState
from finals.sightings import SightingBus
from finals.types import (Abort, Action, Direction, Done, Hover, Land, Move,
                          Rotate, Sighting, Takeoff, Telemetry)


# ---------------- pure-stepping harness --------------------------------------
def _sighting(marker_id, *, cx=320.0, cy=240.0, half=20.0, frame=(480, 640),
              drone_id="alpha"):
    """A minimal valid Sighting centred at (cx, cy) with a square bbox."""
    return Sighting(
        drone_id=drone_id, ts=time.monotonic(), source="aruco",
        class_name=f"aruco_{marker_id}", marker_id=marker_id,
        bbox_xyxy=(cx - half, cy - half, cx + half, cy + half),
        confidence=1.0, frame_shape=frame)


def make_ctx(*, sightings=None, altitude_m=2.0, is_flying=True,
             elapsed_s=0.0, last_action=None, last_action_ok=None,
             last_action_error=None, drone_id="alpha"):
    return AgentContext(
        drone_id=drone_id, now=100.0 + elapsed_s, mission_elapsed_s=elapsed_s,
        telemetry=Telemetry(ts=100.0 + elapsed_s, altitude_m=altitude_m,
                            is_flying=is_flying),
        sightings=sightings or [], last_action=last_action,
        last_action_ok=last_action_ok, last_action_error=last_action_error)


def _phase(**kw):
    """A LandOnPad with small, easy-to-reason-about defaults."""
    base = dict(valid_marker_ids=[7], k_lateral=1.0, tol_px=30.0,
                min_step_cm=5, max_step_cm=50, descend_step_cm=30,
                descend_persist_frames=2, center_persist_frames=3,
                acquire_window_frames=5, acquire_min_hits=3,
                commit_alt_m=0.5, acquire_timeout_s=20.0, total_budget_s=90.0,
                max_loss_retries=3, acquire_scan_step_deg=30.0,
                scan_dwell_s=0.5)
    base.update(kw)
    return LandOnPad(**base)


def _drone(zone=None):
    return DroneConfig(id="alpha", phases=["land_on_pad"], zone=zone or {})


# ============================================================
# Registry
# ============================================================
def test_registered_under_its_name():
    assert PHASE_REGISTRY["land_on_pad"] is LandOnPad
    assert LandOnPad.name == "land_on_pad"


# ============================================================
# PAD_ACQUIRE
# ============================================================
def test_acquire_on_3_of_5_marker_stream_then_centers():
    """3 of the last 5 frames with a CENTRED valid marker -> acquired ->
    PAD_CENTER, and (since centred) it begins counting the centered streak."""
    p = _phase()
    # frame 1: seen (centred bbox). Not yet 3 hits -> still acquiring (scan).
    a1 = p.step(make_ctx(sightings=[_sighting(7)]))
    assert p._sub is _SubState.PAD_ACQUIRE
    assert isinstance(a1, (Rotate, Hover))          # bounded scan
    # frame 2: not seen.
    p.step(make_ctx(sightings=[]))
    # frame 3: seen.
    p.step(make_ctx(sightings=[_sighting(7)]))
    assert p._sub is _SubState.PAD_ACQUIRE
    # frame 4: seen -> now 3 of last <=5 -> acquire -> center; centred bbox ->
    # streak starts, returns a Hover (hold for the next centering frame).
    a4 = p.step(make_ctx(sightings=[_sighting(7)]))
    assert p._sub is _SubState.PAD_CENTER
    assert p._target_marker_id == 7
    assert isinstance(a4, Hover)                    # centred this frame, holding


def test_acquire_ignores_non_valid_marker_id():
    """A marker whose id is NOT in valid_marker_ids must never count as a hit
    (an INVALID/red-pad marker drives nothing)."""
    p = _phase()
    for _ in range(5):
        p.step(make_ctx(sightings=[_sighting(99)]))     # 99 not valid
    assert p._sub is _SubState.PAD_ACQUIRE
    assert p._recent_hits() == 0


def test_acquire_ignores_flicker_below_n_of_m():
    """Only 2 of the last 5 frames hit (< acquire_min_hits=3) -> never
    acquires."""
    p = _phase()
    seen = [True, False, True, False, False]
    for s in seen:
        p.step(make_ctx(sightings=[_sighting(7)] if s else []))
    assert p._sub is _SubState.PAD_ACQUIRE
    assert p._recent_hits() == 2


def test_acquire_scan_alternates_rotate_and_hover():
    """While acquiring, the bounded scan sweeps the FOV: Rotate then a dwell
    Hover so a frame can be observed."""
    p = _phase()
    a1 = p.step(make_ctx(sightings=[]))
    a2 = p.step(make_ctx(sightings=[]))
    a3 = p.step(make_ctx(sightings=[]))
    assert isinstance(a1, Rotate) and a1.angle_deg == 30.0
    assert isinstance(a2, Hover)
    assert isinstance(a3, Rotate)


# ============================================================
# PAD_CENTER
# ============================================================
def _force_center(p, *, altitude_m=2.0):
    """Drive the phase into PAD_CENTER with the marker centred (no streak yet
    consumed beyond entry)."""
    for _ in range(p.acquire_min_hits):
        p.step(make_ctx(sightings=[_sighting(7)], altitude_m=altitude_m))
    assert p._sub is _SubState.PAD_CENTER


def test_center_off_centre_marker_moves_toward_it():
    """A marker to the RIGHT of frame centre -> Move(RIGHT) (chase the blob);
    a clamped step within [min, max]."""
    p = _phase()
    _force_center(p)
    p._center_streak = 0
    a = p.step(make_ctx(sightings=[_sighting(7, cx=600.0)]))   # right of 320
    assert isinstance(a, Move)
    assert a.direction == Direction.RIGHT
    assert p.min_step_cm <= a.distance_cm <= p.max_step_cm
    assert p._center_streak == 0          # a Move resets the centered streak


def test_center_off_centre_left_moves_left():
    p = _phase()
    _force_center(p)
    a = p.step(make_ctx(sightings=[_sighting(7, cx=40.0)]))    # left of 320
    assert isinstance(a, Move) and a.direction == Direction.LEFT


def test_center_centred_marker_no_lateral_move():
    """A centred bbox (px within tol) -> no lateral Move (a Hover hold)."""
    p = _phase()
    _force_center(p)
    p._center_streak = 0
    a = p.step(make_ctx(sightings=[_sighting(7, cx=320.0)]))
    assert not isinstance(a, Move)
    assert p._center_streak == 1


def test_center_persist_frames_then_descends():
    """Centred for center_persist_frames consecutive frames -> PAD_DESCEND."""
    p = _phase(center_persist_frames=3)
    _force_center(p)
    p._center_streak = 0
    p.step(make_ctx(sightings=[_sighting(7, cx=320.0)]))   # streak 1
    p.step(make_ctx(sightings=[_sighting(7, cx=320.0)]))   # streak 2
    a = p.step(make_ctx(sightings=[_sighting(7, cx=320.0)]))  # streak 3 -> descend
    assert p._sub is _SubState.PAD_DESCEND
    # entering descend it re-gates: still centred + persistently seen -> DOWN
    assert isinstance(a, Move) and a.direction == Direction.DOWN


def test_center_marker_lost_returns_to_acquire():
    """Lost mid-centre -> back to PAD_ACQUIRE (the streak resets)."""
    p = _phase()
    _force_center(p)
    p._center_streak = 2
    a = p.step(make_ctx(sightings=[]))      # lost
    assert p._sub is _SubState.PAD_ACQUIRE
    assert p._center_streak == 0
    assert isinstance(a, (Rotate, Hover))   # acquire scan resumes


# ============================================================
# PAD_DESCEND
# ============================================================
def _force_descend(p, *, altitude_m=2.0):
    _force_center(p, altitude_m=altitude_m)
    p._center_streak = 0
    for _ in range(p.center_persist_frames):
        p.step(make_ctx(sightings=[_sighting(7, cx=320.0)], altitude_m=altitude_m))
    assert p._sub is _SubState.PAD_DESCEND


def test_descend_only_when_centered_and_persistently_seen():
    """DOWN step requires centred AND >= descend_persist_frames recent hits."""
    p = _phase(descend_persist_frames=2)
    _force_descend(p)
    # recent window is full of hits (we centred over several seen frames).
    a = p.step(make_ctx(sightings=[_sighting(7, cx=320.0)]))
    assert isinstance(a, Move) and a.direction == Direction.DOWN
    assert a.distance_cm == p.descend_step_cm


def test_descend_re_gates_centering_after_a_step():
    """If the marker drifts off-centre during descend, drop back to
    PAD_CENTER and correct laterally instead of descending blindly."""
    p = _phase()
    _force_descend(p)
    a = p.step(make_ctx(sightings=[_sighting(7, cx=600.0)]))   # drifted right
    assert p._sub is _SubState.PAD_CENTER
    assert isinstance(a, Move) and a.direction == Direction.RIGHT


def test_descend_holds_when_centered_but_not_persistently_seen():
    """Centred but the recent window has < descend_persist_frames hits ->
    hold (do NOT descend on a single fragile sighting)."""
    p = _phase(descend_persist_frames=3, acquire_min_hits=1,
               acquire_window_frames=5, center_persist_frames=1)
    # acquire on a single hit, center immediately, then ensure the window has
    # only 1 recent hit by interleaving misses is impossible (lost -> acquire);
    # instead enter descend with a thin window via a fresh build:
    p._sub = _SubState.PAD_DESCEND
    p._recent.clear()
    p._recent.append(True)        # only 1 recent hit < descend_persist 3
    a = p.step(make_ctx(sightings=[_sighting(7, cx=320.0)]))
    # the new centred frame appends another hit -> 2 hits < 3 -> still holds
    assert not isinstance(a, Move)
    assert p._sub is _SubState.PAD_DESCEND


def test_descend_lost_marker_ascends_and_reacquires():
    """Marker lost mid-descend -> Move(UP) one step + back to PAD_ACQUIRE."""
    p = _phase(max_loss_retries=3)
    _force_descend(p)
    a = p.step(make_ctx(sightings=[]))      # lost
    assert isinstance(a, Move) and a.direction == Direction.UP
    assert a.distance_cm == p.descend_step_cm
    assert p._sub is _SubState.PAD_ACQUIRE
    assert p._loss_retries == 1


def test_descend_loss_retry_limit_then_fallback_lands():
    """After max_loss_retries lost-marker events -> Fallback blind Land."""
    p = _phase(max_loss_retries=2)
    _force_descend(p)
    # 1st loss -> UP + acquire
    p.step(make_ctx(sightings=[]))
    assert p._loss_retries == 1
    # re-enter descend, 2nd loss -> UP + acquire
    p._sub = _SubState.PAD_DESCEND
    p._recent.extend([True, True, True])
    p.step(make_ctx(sightings=[]))
    assert p._loss_retries == 2
    # re-enter descend, 3rd loss exceeds the limit -> Fallback Land
    p._sub = _SubState.PAD_DESCEND
    p._recent.extend([True, True, True])
    a = p.step(make_ctx(sightings=[]))
    assert isinstance(a, Land)
    assert p._sub is _SubState.FALLBACK
    assert "UNVERIFIED_LANDING" in (p._fallback_reason or "")


# ============================================================
# Budget / Fallback
# ============================================================
def test_budget_exceeded_in_acquire_fallback_blind_land():
    p = _phase(total_budget_s=10.0, acquire_timeout_s=10.0)
    p.step(make_ctx(sightings=[], elapsed_s=5.0))    # prime the phase clock t0
    a = p.step(make_ctx(sightings=[], elapsed_s=16.0))   # +11 s > budget
    assert isinstance(a, Land)
    assert p._sub is _SubState.FALLBACK
    assert "total landing budget" in (p._fallback_reason or "")


def test_budget_measured_from_phase_entry_not_mission_start():
    """The budget is a per-PHASE wall clock (the phase may start mid-mission
    after navigate) — a large mission_elapsed_s at entry must NOT instantly
    trip the budget."""
    p = _phase(total_budget_s=10.0, acquire_timeout_s=10.0)
    # First step at a big mission elapsed (e.g. navigate took 100 s): captures
    # t0; phase elapsed is 0 -> NOT a budget trip.
    a = p.step(make_ctx(sightings=[], elapsed_s=100.0))
    assert p._sub is not _SubState.FALLBACK
    assert not isinstance(a, Land)


def test_budget_exceeded_in_center_fallback_blind_land():
    p = _phase(total_budget_s=10.0, acquire_timeout_s=10.0)
    _force_center(p)                                  # several steps at t=0
    a = p.step(make_ctx(sightings=[_sighting(7, cx=600.0)], elapsed_s=11.0))
    assert isinstance(a, Land)
    assert p._sub is _SubState.FALLBACK


def test_acquire_timeout_fallback_blind_land():
    """No acquire by acquire_timeout_s -> Fallback (distinct from total
    budget; the timeout fires first)."""
    p = _phase(acquire_timeout_s=5.0, total_budget_s=90.0)
    p.step(make_ctx(sightings=[], elapsed_s=0.0))     # prime t0
    a = p.step(make_ctx(sightings=[], elapsed_s=6.0))
    assert isinstance(a, Land)
    assert p._sub is _SubState.FALLBACK
    assert "no valid pad acquired" in (p._fallback_reason or "")


def test_fallback_keeps_landing_until_grounded_then_done():
    """Fallback emits Land repeatedly until is_flying flips, then a Done
    carrying the UNVERIFIED reason — never an infinite hover."""
    p = _phase(total_budget_s=10.0, acquire_timeout_s=10.0)
    p.step(make_ctx(sightings=[], elapsed_s=0.0))     # prime t0
    a1 = p.step(make_ctx(sightings=[], elapsed_s=11.0, is_flying=True))
    assert isinstance(a1, Land)
    a2 = p.step(make_ctx(sightings=[], elapsed_s=12.0, is_flying=True))
    assert isinstance(a2, Land)             # idempotent re-land
    done = p.step(make_ctx(sightings=[], elapsed_s=13.0, is_flying=False))
    assert isinstance(done, Done)
    assert "UNVERIFIED_LANDING" in done.reason


# ============================================================
# LAND_COMMIT
# ============================================================
def test_commit_below_alt_lands():
    """altitude <= commit_alt_m -> Land (the marker leaves the FOV anyway)."""
    p = _phase(commit_alt_m=0.5)
    _force_descend(p)
    a = p.step(make_ctx(sightings=[_sighting(7)], altitude_m=0.4))
    assert isinstance(a, Land)


def test_commit_is_flying_false_is_verified_done():
    p = _phase()
    _force_descend(p)
    a = p.step(make_ctx(sightings=[_sighting(7)], altitude_m=0.3,
                        is_flying=False))
    assert isinstance(a, Done)
    assert "VERIFIED_LANDING" in a.reason


def test_commit_not_triggered_above_floor():
    p = _phase(commit_alt_m=0.5)
    _force_descend(p)
    a = p.step(make_ctx(sightings=[_sighting(7, cx=320.0)], altitude_m=1.0))
    assert not isinstance(a, (Land, Done))


# ============================================================
# Adversarial / boundary
# ============================================================
def test_two_valid_markers_picks_largest_bbox_deterministically():
    """Two valid pads in frame -> the larger bbox (closest) wins, tie-broken
    by lowest id; the choice must be deterministic (no flapping)."""
    p = _phase(valid_marker_ids=[7, 11])
    _force_center_multi = None
    # acquire on frames carrying both ids; the picked target is stable.
    big = _sighting(11, cx=320.0, half=40.0)      # bigger bbox
    small = _sighting(7, cx=320.0, half=10.0)     # smaller bbox
    for _ in range(p.acquire_min_hits):
        p.step(make_ctx(sightings=[small, big]))
    assert p._sub is _SubState.PAD_CENTER
    assert p._target_marker_id == 11              # the larger bbox


def test_two_valid_markers_equal_area_tie_break_lowest_id():
    p = _phase(valid_marker_ids=[7, 11])
    a = _sighting(11, cx=320.0, half=20.0)
    b = _sighting(7, cx=320.0, half=20.0)         # equal area
    picked = p._pick_target([a, b])
    assert picked.marker_id == 7                  # lowest id wins the tie


def test_altitude_already_below_commit_on_entry_lands_immediately():
    """If we enter the phase already below the depth floor (navigate handed
    off low), commit immediately — don't try to acquire from the deck."""
    p = _phase(commit_alt_m=0.5)
    a = p.step(make_ctx(sightings=[_sighting(7)], altitude_m=0.3))
    assert isinstance(a, Land)


def test_nan_altitude_does_not_commit_and_aborts_actionably():
    """A NaN altitude must NOT silently blind-land (it is not <= commit), and
    when centering would scale by it the phase Aborts ACTIONABLY rather than
    letting a raw servo ValueError escape (fail-loud bar)."""
    p = _phase()
    # acquire over finite-altitude frames first
    _force_center(p)
    a = p.step(make_ctx(sightings=[_sighting(7, cx=600.0)],
                        altitude_m=float("nan")))
    assert isinstance(a, Abort)
    assert "altitude_m" in a.reason and "ToF" in a.reason
    assert "abort" in a.reason.lower()


def test_none_altitude_aborts_actionably_in_center():
    """A None altitude (telemetry not reporting it) during centering -> Abort,
    not a crash."""
    p = _phase()
    _force_center(p)
    a = p.step(make_ctx(sightings=[_sighting(7, cx=600.0)], altitude_m=None))
    assert isinstance(a, Abort)
    assert "altitude_m" in a.reason


def test_marker_vanishes_exactly_at_persist_boundary():
    """A marker present through center_persist-1 frames then vanishing on the
    boundary frame must NOT descend — it drops back to acquire instead."""
    p = _phase(center_persist_frames=3)
    _force_center(p)
    p._center_streak = 0
    p.step(make_ctx(sightings=[_sighting(7, cx=320.0)]))   # streak 1
    p.step(make_ctx(sightings=[_sighting(7, cx=320.0)]))   # streak 2
    a = p.step(make_ctx(sightings=[]))                     # vanishes at boundary
    assert p._sub is _SubState.PAD_ACQUIRE
    assert not isinstance(a, Move) or a.direction not in (Direction.DOWN,)


def test_tol_px_boundary_inclusive_no_move():
    """A bbox exactly tol_px off centre is INSIDE the deadband (the locked
    _servo convention) -> no lateral Move."""
    p = _phase(tol_px=30.0)
    _force_center(p)
    p._center_streak = 0
    # frame_w 640, centre 320; cx 350 -> px +30 == tol -> None (inside).
    a = p.step(make_ctx(sightings=[_sighting(7, cx=350.0)]))
    assert not isinstance(a, Move)
    assert p._center_streak == 1


# ============================================================
# last_action_ok False -> Abort
# ============================================================
def test_failed_action_aborts_actionable():
    p = _phase()
    a = p.step(make_ctx(
        sightings=[_sighting(7)], last_action=Move(Direction.DOWN, 30),
        last_action_ok=False,
        last_action_error="alpha: move(DOWN, 30 cm) exceeded 15.0 s"))
    assert isinstance(a, Abort)
    assert "alpha" in a.reason and "exceeded 15.0 s" in a.reason
    assert "abort" in a.reason.lower()
    assert "unknown attitude" in a.reason.lower()


# ============================================================
# from_config + validation
# ============================================================
def test_from_config_defaults_with_minimal_zone():
    p = LandOnPad.from_config(
        _drone(zone={"land_on_pad": {"valid_marker_ids": [7, 11]}}), cfg=None)
    assert p.valid_marker_ids == frozenset({7, 11})
    assert p.k_lateral == 1.0 and p.commit_alt_m == 0.5


def test_from_config_full_zone_tunables():
    zone = {"land_on_pad": {
        "valid_marker_ids": [3], "k_lateral": 2.5, "tol_px": 25,
        "min_step_cm": 10, "max_step_cm": 60, "descend_step_cm": 20,
        "descend_persist_frames": 3, "center_persist_frames": 4,
        "acquire_window_frames": 6, "acquire_min_hits": 4, "commit_alt_m": 0.6,
        "acquire_timeout_s": 15, "total_budget_s": 60, "max_loss_retries": 2,
        "acquire_scan_step_deg": 45, "scan_dwell_s": 0.3,
        "_comment": "ignored"}}
    p = LandOnPad.from_config(_drone(zone=zone), cfg=None)
    assert p.k_lateral == 2.5 and p.max_step_cm == 60
    assert p.center_persist_frames == 4 and p.acquire_min_hits == 4


def test_from_config_empty_valid_marker_ids_fails():
    with pytest.raises(ConfigError, match="valid_marker_ids"):
        LandOnPad.from_config(
            _drone(zone={"land_on_pad": {"valid_marker_ids": []}}), cfg=None)


def test_from_config_missing_valid_marker_ids_fails():
    # A missing valid_marker_ids defaults to None -> the same actionable
    # ConfigError as an empty list (a lander that could never acquire).
    with pytest.raises(ConfigError, match="valid_marker_ids"):
        LandOnPad.from_config(_drone(zone={"land_on_pad": {}}), cfg=None)


def test_from_config_unknown_key_fails_loudly():
    with pytest.raises(ConfigError, match=r"alpha.*k_latteral"):
        LandOnPad.from_config(
            _drone(zone={"land_on_pad": {"valid_marker_ids": [7],
                                         "k_latteral": 1}}), cfg=None)


@pytest.mark.parametrize("kwargs", [
    {"valid_marker_ids": []},
    {"valid_marker_ids": [7, "x"]},
    {"valid_marker_ids": [True]},          # bool is not a real marker id
    {"valid_marker_ids": 7},               # not a list
    {"k_lateral": 0},
    {"k_lateral": -1.0},
    {"k_lateral": math.nan},
    {"tol_px": -1.0},
    {"min_step_cm": 0},
    {"max_step_cm": 0},
    {"min_step_cm": 60, "max_step_cm": 10},   # swapped
    {"descend_step_cm": 0},                   # never descends
    {"descend_persist_frames": 0},
    {"center_persist_frames": 0},
    {"acquire_window_frames": 0},
    {"acquire_min_hits": 0},
    {"acquire_min_hits": 6, "acquire_window_frames": 5},   # N > M
    {"descend_persist_frames": 6, "acquire_window_frames": 5},  # > window
    {"commit_alt_m": 0},
    {"commit_alt_m": -0.1},
    {"acquire_timeout_s": 0},
    {"total_budget_s": 0},
    {"acquire_timeout_s": 100, "total_budget_s": 50},   # timeout > budget
    {"max_loss_retries": 0},
    {"acquire_scan_step_deg": 0},
    {"scan_dwell_s": 0},
])
def test_constructor_rejects_bad_tunables(kwargs):
    base = dict(valid_marker_ids=[7])
    base.update(kwargs)
    with pytest.raises(ConfigError, match="land_on_pad"):
        LandOnPad(**base)


# ============================================================
# Integration — agent + phase over MockAdapter, scripted good approach
# ============================================================
class _Takeoff(MissionPhase):
    """Tiny helper phase: takeoff once -> Done. NOT registered (test_conventions
    pins PHASE_REGISTRY exactly)."""

    name = "_takeoff_helper"

    def __init__(self, height_cm=200):
        self._done = False
        self._h = height_cm

    def step(self, ctx: AgentContext) -> Action:
        if not self._done:
            self._done = True
            return Takeoff(height_cm=self._h)
        return Done("airborne over the pad vicinity")


class _CentredPadBus(SightingBus):
    """A SightingBus that surfaces ONE fresh CENTRED valid sighting every tick
    (models a perception loop that sees the pad dead-centre on every frame —
    the scripted GOOD approach). Real drains are per-tick; pre-seeding the real
    bus would drain everything at once, so we synthesize a fresh frame here."""

    def drain_after(self, seq, drone_id=None):
        return seq + 1, [_sighting(7, cx=320.0, drone_id=drone_id or "alpha")]


def _run_agent(agent, *, budget_s=30.0):
    async def go():
        stop = asyncio.Event()
        await agent.run(deadline=time.monotonic() + budget_s, stop_event=stop)
    asyncio.run(go())


def test_agent_lands_done_on_a_scripted_good_approach(tmp_path):
    """Full path: takeoff to 2 m, then a stream of CENTRED valid sightings
    (one per tick) centres the drone; each descend step drops the DR altitude
    until it crosses commit_alt_m -> Land -> is_flying False -> Done (verified).

    The DR altitude is driven by the Move(DOWN) steps the phase commands, so
    the descent really reaches the floor and the agent reaches DONE without the
    budget Fallback firing."""
    bus = _CentredPadBus()
    land = _phase(commit_alt_m=0.5, descend_step_cm=50, center_persist_frames=2,
                  descend_persist_frames=2, acquire_min_hits=2,
                  acquire_window_frames=3, scan_dwell_s=0.001)

    adapter = MockAdapter("alpha")
    with EventLog(str(tmp_path)) as events:
        agent = DroneAgent("alpha", adapter, [_Takeoff(200), land], events,
                           bus=bus)
        _run_agent(agent, budget_s=20.0)
        asyncio.run(agent.shutdown())

    assert agent.state is AgentState.DONE
    assert not adapter.is_flying
    names = [c[0] for c in adapter.calls]
    assert "takeoff" in names
    assert "land" in names                 # committed a real landing
    assert any(n == "move" for n in names)  # at least one descend/center move
    # The descent reached the commit floor BEFORE the budget Fallback — the
    # phase_done reason is the VERIFIED landing, not UNVERIFIED.
    phase = land
    assert phase._fallback_reason is None
