"""NAV-0 contract tests for the arena map skeleton.

These pin the FROZEN shapes + the thin parse/resolution that the parallel S11
navigation sessions code against. NAV-2 adds the SEMANTIC-validation suite
(bounds ordering, pads-within-bounds, unique ids, >= 3-vertex polygons) in
test_arena_config.py — this file only proves the contract exists and that the
parse fails loudly on a malformed map (never silently drops a key).
"""
from __future__ import annotations

import json

import pytest

from finals.config import DetectorConfig, FinalsConfig, _resolve_arena
from finals.errors import ConfigError
from finals.mission.planning.types import ArenaMap, KeepOut, LandingPad, Leg

_MINIMAL_ARENA = {
    "_comment": "comment keys are ignored everywhere",
    "bounds_m": [0.0, 0.0, 10.0, 8.0],
    "c2_origin_m": [0.5, 4.0],
    "c2_heading_deg": 90.0,
    "keep_out": [
        {"id": "crate_a", "polygon_m": [[3, 3], [4, 3], [4, 4], [3, 4]]},
    ],
    "pads": [
        {"id": "pad_1", "center_m": [8, 2], "radius_m": 0.25, "valid": True},
        {"id": "pad_2", "center_m": [8, 6], "radius_m": 0.25, "valid": False},
    ],
    "lanes": [[[0, 0], [10, 0]]],
}


def _bare_cfg(**kw) -> FinalsConfig:
    """A FinalsConfig carrying only what _resolve_arena reads (arena_name)."""
    return FinalsConfig(
        profile="mock", flight_backend="mock", frame_backend="none",
        detector=DetectorConfig(), drones=[], **kw)


# ---- contract shapes -------------------------------------------------------
def test_leg_is_frozen_cm_and_deg():
    leg = Leg(heading_deg=90.0, distance_cm=150.0)
    assert (leg.heading_deg, leg.distance_cm) == (90.0, 150.0)
    with pytest.raises(Exception):  # frozen dataclass
        leg.heading_deg = 0.0  # type: ignore[misc]


def test_arenamap_from_dict_minimal():
    arena = ArenaMap.from_dict(_MINIMAL_ARENA, name="sample")
    assert arena.bounds_m == (0.0, 0.0, 10.0, 8.0)
    assert arena.c2_origin_m == (0.5, 4.0)
    assert arena.c2_heading_deg == 90.0
    assert len(arena.keep_out) == 1 and isinstance(arena.keep_out[0], KeepOut)
    assert arena.keep_out[0].id == "crate_a"
    assert arena.keep_out[0].polygon_m == ((3, 3), (4, 3), (4, 4), (3, 4))
    assert [p.id for p in arena.pads] == ["pad_1", "pad_2"]
    assert isinstance(arena.pads[0], LandingPad)
    assert arena.pads[0].valid is True and arena.pads[1].valid is False
    assert arena.lanes == (((0.0, 0.0), (10.0, 0.0)),)


def test_arena_missing_required_key_raises():
    bad = {k: v for k, v in _MINIMAL_ARENA.items() if k != "bounds_m"}
    with pytest.raises(ConfigError, match="missing required key"):
        ArenaMap.from_dict(bad, name="sample")


def test_arena_unknown_key_is_loud_not_dropped():
    bad = dict(_MINIMAL_ARENA, typo_field=1)
    with pytest.raises(ConfigError, match="unknown key"):
        ArenaMap.from_dict(bad, name="sample")


def test_pad_unknown_key_is_loud():
    bad = json.loads(json.dumps(_MINIMAL_ARENA))
    bad["pads"][0]["colour"] = "green"
    with pytest.raises(ConfigError, match=r"pads\[0\]"):
        ArenaMap.from_dict(bad, name="sample")


def test_malformed_point_raises_configerror_not_typeerror():
    bad = json.loads(json.dumps(_MINIMAL_ARENA))
    bad["c2_origin_m"] = [1.0, "east"]
    with pytest.raises(ConfigError, match="north_m, east_m"):
        ArenaMap.from_dict(bad, name="sample")


def test_pad_zero_radius_rejected():
    bad = json.loads(json.dumps(_MINIMAL_ARENA))
    bad["pads"][0]["radius_m"] = 0
    with pytest.raises(ConfigError, match="radius_m"):
        ArenaMap.from_dict(bad, name="sample")


# ---- config resolution -----------------------------------------------------
def test_no_arena_name_leaves_arena_none():
    cfg = _bare_cfg()
    _resolve_arena(cfg, config_dir=".")
    assert cfg.arena is None


def test_resolve_arena_loads_file(tmp_path):
    arenas = tmp_path / "arenas"
    arenas.mkdir()
    (arenas / "sample.json").write_text(json.dumps(_MINIMAL_ARENA),
                                        encoding="utf-8")
    cfg = _bare_cfg(arena_name="sample")
    _resolve_arena(cfg, config_dir=str(tmp_path))
    assert isinstance(cfg.arena, ArenaMap)
    assert [p.id for p in cfg.arena.pads] == ["pad_1", "pad_2"]


def test_resolve_arena_missing_file_raises(tmp_path):
    cfg = _bare_cfg(arena_name="does_not_exist")
    with pytest.raises(ConfigError, match="map file not found"):
        _resolve_arena(cfg, config_dir=str(tmp_path))
