"""finals/configs/sitl3_dyn{3,5}_vision.json (WS-5) — DYNAMIC self-assignment.

These are the gz-integration configs for the user's sim #3: 3 drones, NO drone
told which car to chase. The gz RUN happens on the VM; what is pinned HERE
(pure) is that the CONFIG wires the dynamic coordinator correctly AND that the
REAL track_convoy phases + a REAL ConvoyRegistry, built straight from the config
by finals.main, resolve worst-case contention (every drone sees every car) into
DISTINCT claims with a correct serviced/remaining tally. The registry's logic is
proven in isolation by test_convoy_registry / test_track_convoy; this proves the
*config-built wiring* of it end-to-end, so a typo (a non-null track_marker_ids,
a missing convoy_ids seed) is caught here, not on the VM.
"""
from __future__ import annotations

import pytest

from finals.config import load_config
from finals.main import _build_convoy_registry, _build_phases
from finals.mission.phase import AgentContext
from finals.types import Done, Sighting, Telemetry

_DYN3 = "finals/configs/sitl3_dyn3_vision.json"
_DYN5 = "finals/configs/sitl3_dyn5_vision.json"


def _ctx(drone_id, now, ids):
    """A context for `drone_id` at `now` seeing EVERY id in `ids` (worst-case
    contention: the dedup must come from the registry, not from who sees what)."""
    s = [Sighting(drone_id=drone_id, ts=now, source="aruco",
                  class_name=f"aruco_{m}", marker_id=m,
                  bbox_xyxy=(310.0, 230.0, 330.0, 250.0), confidence=1.0,
                  frame_shape=(480, 640), bearing_deg=0.0) for m in ids]
    return AgentContext(
        drone_id=drone_id, now=now, mission_elapsed_s=now,
        telemetry=Telemetry(ts=now, yaw_deg=0.0, is_flying=True), sightings=s)


def _build(path):
    cfg = load_config(path)
    reg = _build_convoy_registry(cfg)
    phases = {d.id: _build_phases(d, cfg, reg)[0] for d in cfg.drones}
    return cfg, reg, phases


def _drive_to_distinct_locks(reg, phases, ids, t0=100.0, max_ticks=10):
    """Step every phase each tick (sequentially, sharing the registry) feeding
    all ids, until each phase has locked a target. Returns (locked, t)."""
    t = t0
    locked = {}
    for _ in range(max_ticks):
        for did, ph in phases.items():
            ph.step(_ctx(did, t, ids))
            if ph._target_id is not None:
                locked[did] = ph._target_id
        if len(locked) == len(phases):
            return locked, t
        t += 0.6
    raise AssertionError(f"not all drones locked after {max_ticks} ticks: {locked}")


# ============================================================
# Config contract
# ============================================================
@pytest.mark.parametrize("path,ids", [(_DYN3, [7, 23, 88]),
                                      (_DYN5, [7, 11, 23, 42, 88])])
def test_dyn_config_is_fully_dynamic_and_seeds_the_registry(path, ids):
    cfg, reg, phases = _build(path)
    assert cfg.convoy_ids == ids                       # known set seeded
    assert cfg.convoy_lock_ttl_s > 0
    assert reg is not None                             # a real registry is built
    assert len(phases) == 3
    for did, ph in phases.items():
        assert ph.name == "track_convoy"
        assert ph.track_marker_ids is None, f"{did} is NOT dynamic (has an id list)"
        assert ph.registry is reg                      # all share ONE registry


# ============================================================
# dyn3 — clean: 3 drones, 3 cars -> 3 distinct claims, serviced 3/3
# ============================================================
def test_dyn3_three_drones_self_assign_distinct_then_service_all():
    cfg, reg, phases = _build(_DYN3)
    ids = cfg.convoy_ids

    locked, t = _drive_to_distinct_locks(reg, phases, ids)
    assert sorted(locked.values()) == sorted(ids)      # 3 DISTINCT, the whole set
    for did, mid in locked.items():
        assert reg.owner_of(mid, t) == did             # registry agrees
    assert all(reg.owner_of(m, t) is not None for m in ids)   # every car owned
    assert reg.remaining_ids(ids) == sorted(ids)       # claimed != serviced yet

    # Run each past its investigate budget -> clean Done that SERVICES the car.
    budget = cfg.drones[0].zone["track_convoy"]["investigate_budget_s"]
    t_done = 100.0 + budget + 1.0                       # _t_enter was the first step (t0=100)
    for did, ph in phases.items():
        a = ph.step(_ctx(did, t_done, ids))
        assert isinstance(a, Done), f"{did} did not finish: {a!r}"
    assert reg.serviced_ids() == sorted(ids)
    assert reg.all_serviced(ids) is True
    assert reg.remaining_ids(ids) == []                # NOW all serviced


# ============================================================
# dyn5 — contention: 3 drones, 5 cars -> 3 distinct claims, 2 remain
# ============================================================
def test_dyn5_three_drones_claim_distinct_and_two_cars_remain():
    cfg, reg, phases = _build(_DYN5)
    ids = cfg.convoy_ids                               # 5 ids

    locked, t = _drive_to_distinct_locks(reg, phases, ids)
    assert len(set(locked.values())) == 3              # 3 DISTINCT claims
    assert set(locked.values()).issubset(set(ids))
    # 5 cars, 3 drones -> exactly 2 cars sit UNCLAIMED (no owner).
    unclaimed = [m for m in ids if reg.owner_of(m, t) is None]
    assert len(unclaimed) == 2
    assert set(unclaimed).isdisjoint(set(locked.values()))
    assert reg.all_serviced(ids) is False              # nobody serviced yet
    assert reg.remaining_ids(ids) == sorted(ids)       # remaining = unserviced = all
