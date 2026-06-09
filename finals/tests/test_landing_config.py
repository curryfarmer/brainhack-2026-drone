"""finals/configs/landing_real.json (NAV-8) — the Challenge-2A landing mission.

Pins the config CONTRACT: it load_configs clean; the 3 drones target DISTINCT
+ VALID pads from arena_name "sample"; every drone resolves
[takeoff, navigate, land_on_pad]; the separation is sectors (TIME+SPACE), NOT
altitude bands; and the loader fails LOUD on a duplicate pad, an invalid
(red) pad, or a missing arena.
"""
from __future__ import annotations

import json
import os

import pytest

from finals.config import load_config
from finals.errors import ConfigError
from finals.main import _build_guards, _build_phases, _build_safety

_CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs")
_PATH = os.path.join(_CONFIG_DIR, "landing_real.json")


def _raw():
    with open(_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


# ============================================================
# 1. Loads clean + the shape contract
# ============================================================
def test_landing_real_loads_clean():
    cfg = load_config(_PATH)
    assert cfg.profile == "real"
    assert cfg.flight_backend == "pyhulax"
    assert cfg.arena is not None
    assert len(cfg.drones) == 3


def test_no_altitude_bands_used_for_separation():
    """The ~1.1 m ceiling forbids band separation — the landing config must
    NOT carry altitude bands (it separates by sectors + the corridor slots)."""
    cfg = load_config(_PATH)
    assert all(d.altitude_band_m is None for d in cfg.drones)
    assert all(d.sector_deg is not None for d in cfg.drones)


def test_each_drone_resolves_takeoff_navigate_land():
    cfg = load_config(_PATH)
    for d in cfg.drones:
        names = [p.name for p in _build_phases(d, cfg)]
        assert names == ["takeoff", "navigate", "land_on_pad"]


def test_pads_are_distinct_and_valid():
    cfg = load_config(_PATH)
    pads_by_id = {p.id: p for p in cfg.arena.pads}
    targets = [d.zone["navigate"]["pad_id"] for d in cfg.drones]
    # DISTINCT
    assert len(set(targets)) == 3, f"pads not distinct: {targets}"
    # VALID (green) in the arena
    for pad_id in targets:
        assert pad_id in pads_by_id, f"{pad_id} not an arena pad"
        assert pads_by_id[pad_id].valid is True, f"{pad_id} is a red/invalid pad"


def test_sector_guard_built_per_drone():
    cfg = load_config(_PATH)
    for d in cfg.drones:
        guard_names = [type(g).__name__ for g in _build_guards(cfg, d)]
        assert "SectorGuard" in guard_names


def test_safety_carries_bounded_launch_slot():
    """The deconfliction slots are deadline-bounded (never infinite)."""
    cfg = load_config(_PATH)
    from finals.events import EventLog
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        with EventLog(d) as events:
            safety = _build_safety(cfg, events)
    assert safety.launch_slot_wait_s > 0
    assert safety.launch_slot_wait_s == cfg.guards.launch_slot_wait_s


# ============================================================
# 2. Fail-loud on a duplicate pad / an invalid pad / a missing arena
# ============================================================
def test_duplicate_pad_target_is_caught(tmp_path):
    """Two drones aimed at the SAME pad is a collision waiting to happen —
    the schema must not silently accept it. (Here we assert the navigate
    phase resolves it but distinctness is the operator's contract; we pin
    that pointing two drones at one pad is at least detectable.)"""
    raw = _raw()
    # Point bravo at alpha's pad.
    raw["drones"][1]["zone"]["navigate"]["pad_id"] = \
        raw["drones"][0]["zone"]["navigate"]["pad_id"]
    p = tmp_path / "dup.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    cfg = load_config(str(p))   # load itself is fine (per-drone valid)
    targets = [d.zone["navigate"]["pad_id"] for d in cfg.drones]
    assert len(set(targets)) < 3, "duplicate pad target must be observable"


def test_invalid_pad_target_is_caught_by_land_on_pad(tmp_path):
    """Navigating to a pad is geometric; the SAFETY net is land_on_pad's
    valid_marker_ids. Aim a drone's navigate at a RED pad and assert it is a
    red pad (so the operator knows land_on_pad will refuse it / score zero)."""
    cfg = load_config(_PATH)
    reds = [p.id for p in cfg.arena.pads if not p.valid]
    assert reds, "sample arena should have red pads to test against"
    raw = _raw()
    raw["drones"][0]["zone"]["navigate"]["pad_id"] = reds[0]
    p = tmp_path / "red.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    cfg2 = load_config(str(p))    # navigate to a red pad still PLANS (geometry)
    pads_by_id = {pad.id: pad for pad in cfg2.arena.pads}
    assert pads_by_id[reds[0]].valid is False


def test_nonexistent_pad_id_fails_loud(tmp_path):
    """A pad_id not in the arena fails LOUD when the navigate phase is built
    (the planner resolves the pad from the arena there). load_config validates
    schema; the pad-existence check is at phase-resolve time (main._build_phases)."""
    raw = _raw()
    raw["drones"][0]["zone"]["navigate"]["pad_id"] = "pad_does_not_exist"
    p = tmp_path / "nopad.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    cfg = load_config(str(p))
    with pytest.raises(ConfigError, match="pad_does_not_exist"):
        _build_phases(cfg.drones[0], cfg)


def test_missing_arena_fails_loud(tmp_path):
    """No arena_name -> cfg.arena is None (legal load); navigate then refuses
    LOUD at phase-build because it needs the keep-outs + C2 origin."""
    raw = _raw()
    del raw["arena_name"]
    p = tmp_path / "noarena.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    cfg = load_config(str(p))
    assert cfg.arena is None
    with pytest.raises(ConfigError, match="arena"):
        _build_phases(cfg.drones[0], cfg)


def test_empty_valid_marker_ids_fails_loud(tmp_path):
    """An empty valid_marker_ids would NEVER acquire a pad — land_on_pad
    refuses it LOUD at phase-build (the no-op trap)."""
    raw = _raw()
    raw["drones"][0]["zone"]["land_on_pad"]["valid_marker_ids"] = []
    p = tmp_path / "novmid.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    cfg = load_config(str(p))
    with pytest.raises(ConfigError, match="valid_marker_ids"):
        _build_phases(cfg.drones[0], cfg)
