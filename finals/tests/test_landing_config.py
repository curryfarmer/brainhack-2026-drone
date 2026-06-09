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
def test_duplicate_pad_target_is_refused(tmp_path):
    """Two drones aimed at the SAME pad cannot both score it (they fight for
    one physical pad / one scores zero) — the loader must REFUSE it LOUD, not
    silently accept it. (Was test_duplicate_pad_target_is_caught, which only
    asserted observability and passed with or without a guard — batch-2 NAV
    review R1-MED-1/2.)"""
    raw = _raw()
    dup = raw["drones"][0]["zone"]["navigate"]["pad_id"]
    # Point bravo at alpha's pad.
    raw["drones"][1]["zone"]["navigate"]["pad_id"] = dup
    p = tmp_path / "dup.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ConfigError) as ei:
        load_config(str(p))
    msg = str(ei.value)
    # Names BOTH colliding drones + the pad (actionable WHICH).
    assert "alpha" in msg and "bravo" in msg and dup in msg
    assert "duplicate" in msg.lower()


def test_invalid_red_pad_target_is_refused(tmp_path):
    """A drone whose navigate target is a RED (valid=false) decoy pad would
    never acquire a valid marker (land_on_pad refuses it / scores zero) — the
    loader must REFUSE it LOUD at load time, not plan blindly to it. (Was
    test_invalid_pad_target_is_caught_by_land_on_pad, which only asserted the
    pad was red — batch-2 NAV review R2 LOW BUG.)"""
    cfg = load_config(_PATH)
    reds = [p.id for p in cfg.arena.pads if not p.valid]
    assert reds, "sample arena should have red pads to test against"
    raw = _raw()
    raw["drones"][0]["zone"]["navigate"]["pad_id"] = reds[0]
    p = tmp_path / "red.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ConfigError) as ei:
        load_config(str(p))
    msg = str(ei.value)
    assert "alpha" in msg and reds[0] in msg
    assert "red" in msg.lower() or "valid=false" in msg.lower()


def test_too_few_valid_pads_for_drones_is_refused(tmp_path):
    """Case (c): if the arena holds FEWER valid (green) pads than the number of
    drones navigating to a pad, distinct valid targets cannot be satisfied —
    refuse LOUD. We mutate two of the three GREEN sample pads to red so the
    arena holds only 1 valid pad for 3 navigating drones (and give the 3 drones
    distinct, valid-at-write-time targets so it is the CAPACITY guard, not the
    dup/red guard, that fires)."""
    # Copy the real config + a mutated arena into tmp so arena_name resolves to
    # the mutated map (config.py looks for <arena_name>.json beside the config).
    raw = _raw()
    arena_path = os.path.join(_CONFIG_DIR, "arenas", "sample.json")
    with open(arena_path, "r", encoding="utf-8") as f:
        arena_raw = json.load(f)
    greens = [pad for pad in arena_raw["pads"] if pad.get("valid")]
    assert len(greens) >= 3, "sample arena should ship >= 3 green pads"
    # Flip all but ONE green pad to red -> only 1 valid pad remains.
    for pad in greens[1:]:
        pad["valid"] = False
    only_valid = greens[0]["id"]
    # Isolate the (c) CAPACITY guard from (a) dup + (b) red: drone 0 keeps the
    # one surviving green pad (valid, distinct), drones 1 + 2 navigate to an
    # explicit goal_ne_m coord (EXEMPT from (a)/(b) but still COUNTED toward
    # (c)). So: 1 valid pad < 3 navigating drones -> (c) fires.
    raw["drones"][0]["zone"]["navigate"]["pad_id"] = only_valid
    for i in (1, 2):
        del raw["drones"][i]["zone"]["navigate"]["pad_id"]
        raw["drones"][i]["zone"]["navigate"]["goal_ne_m"] = [3.0, 3.0]
    arena_dir = tmp_path / "arenas"
    arena_dir.mkdir()
    (arena_dir / "sample.json").write_text(json.dumps(arena_raw),
                                           encoding="utf-8")
    p = tmp_path / "fewpads.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ConfigError) as ei:
        load_config(str(p))
    msg = str(ei.value)
    assert "valid" in msg.lower() and "pad" in msg.lower()
    assert "3" in msg          # 3 navigating drones named


def test_shipped_config_pad_targets_load_clean():
    """Regression guard: landing_real.json's 3 pad targets are DISTINCT +
    VALID, so the new cross-drone pad-target guard does NOT reject the shipped
    config."""
    cfg = load_config(_PATH)
    targets = [d.zone["navigate"]["pad_id"] for d in cfg.drones]
    assert len(set(targets)) == 3
    valid_ids = {pad.id for pad in cfg.arena.pads if pad.valid}
    for t in targets:
        assert t in valid_ids


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
