"""NAV-2 SEMANTIC-validation suite for the arena map.

NAV-0's test_arena_skeleton.py pins the frozen shapes + thin parse; THIS file
pins the hardened semantic rules NAV-2 added to ArenaMap.from_dict (bounds
ordering, pads within bounds, unique pad/keep-out ids, >= 3-distinct-vertex
keep-outs, c2-origin within bounds) plus the real shipped sample.json + the
config-level arena resolution. Every malformed case must raise ConfigError with
an ACTIONABLE message (which pad/polygon, what rule, the offending value).

Source: finals/mission/planning/types.py (ArenaMap.from_dict), config.py
(_resolve_arena / load_config). Pure — no SDK, no cv2/numpy.
"""
from __future__ import annotations

import copy
import json
import os

import pytest

from finals.config import load_config
from finals.errors import ConfigError
from finals.mission.planning.types import ArenaMap

# A semantically-valid arena: 1 keep-out (square), 2 pads (both in bounds),
# c2 origin in bounds. Mutated per-test to trip exactly one rule at a time.
_VALID = {
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

_SAMPLE_PATH = os.path.join(
    os.path.dirname(__file__), "..", "configs", "arenas", "sample.json")


def _mut(**deep):
    """Deep-copy _VALID then apply a flat set of top-level replacements."""
    raw = copy.deepcopy(_VALID)
    raw.update(deep)
    return raw


# ---- the valid baseline still loads ---------------------------------------
def test_valid_arena_loads():
    arena = ArenaMap.from_dict(_VALID, name="t")
    assert arena.bounds_m == (0.0, 0.0, 10.0, 8.0)
    assert [p.id for p in arena.pads] == ["pad_1", "pad_2"]
    assert [k.id for k in arena.keep_out] == ["crate_a"]


# ---- bounds ordering -------------------------------------------------------
def test_bounds_north_min_not_less_than_max_raises():
    # Zero-WIDTH north span (min == max). Place every pad + the origin ON that
    # degenerate line so they stay in-bounds (closed compare) — this isolates
    # the bounds-ORDERING rule as the ONLY thing that can raise, which kills a
    # `<` -> `<=` mutant (a `<=` would wave a zero-area arena through).
    bad = _mut(bounds_m=[5.0, 0.0, 5.0, 8.0], c2_origin_m=[5.0, 4.0])
    bad["pads"][0]["center_m"] = [5.0, 2.0]
    bad["pads"][1]["center_m"] = [5.0, 6.0]
    bad["keep_out"][0]["polygon_m"] = [[5.0, 3.0], [5.0, 4.0], [5.0, 5.0]]
    with pytest.raises(ConfigError, match=r"north_min.*must be <.*north_max"):
        ArenaMap.from_dict(bad, name="t")


def test_bounds_north_inverted_raises():
    with pytest.raises(ConfigError, match=r"north_min.*north_max"):
        ArenaMap.from_dict(_mut(bounds_m=[10.0, 0.0, 0.0, 8.0]), name="t")


def test_bounds_east_min_not_less_than_max_raises():
    # Zero-WIDTH east span, everything on the degenerate line — isolates the
    # east ordering rule (kills a `<` -> `<=` mutant on the east axis).
    bad = _mut(bounds_m=[0.0, 4.0, 10.0, 4.0], c2_origin_m=[0.5, 4.0])
    bad["pads"][0]["center_m"] = [3.0, 4.0]
    bad["pads"][1]["center_m"] = [7.0, 4.0]
    bad["keep_out"][0]["polygon_m"] = [[3.0, 4.0], [4.0, 4.0], [5.0, 4.0]]
    with pytest.raises(ConfigError, match=r"east_min.*must be <.*east_max"):
        ArenaMap.from_dict(bad, name="t")


def test_bounds_nonfinite_raises():
    with pytest.raises(ConfigError, match="finite"):
        ArenaMap.from_dict(_mut(bounds_m=[0.0, 0.0, float("inf"), 8.0]),
                           name="t")


# ---- pad center within bounds ---------------------------------------------
def test_pad_out_of_bounds_north_raises():
    bad = _mut()
    bad["pads"][1]["center_m"] = [11.0, 4.0]   # north 11 > north_max 10
    with pytest.raises(ConfigError, match=r"pads\[1\].*pad_2.*OUTSIDE bounds"):
        ArenaMap.from_dict(bad, name="t")


def test_pad_out_of_bounds_negative_east_raises():
    bad = _mut()
    bad["pads"][0]["center_m"] = [5.0, -0.1]   # east < east_min 0
    with pytest.raises(ConfigError, match=r"pads\[0\].*OUTSIDE bounds"):
        ArenaMap.from_dict(bad, name="t")


def test_pad_exactly_on_bounds_edge_is_legal():
    # Closed bounds: a pad flush to the wall is valid (the keep-in geofence
    # wall is inclusive). north 10 == north_max, east 8 == east_max.
    ok = _mut()
    ok["pads"][0]["center_m"] = [10.0, 8.0]
    arena = ArenaMap.from_dict(ok, name="t")
    assert arena.pads[0].center_m == (10.0, 8.0)


# ---- unique ids ------------------------------------------------------------
def test_duplicate_pad_id_raises():
    bad = _mut()
    bad["pads"][1]["id"] = "pad_1"
    with pytest.raises(ConfigError, match=r"duplicate pad id 'pad_1'"):
        ArenaMap.from_dict(bad, name="t")


def test_duplicate_keepout_id_raises():
    bad = _mut()
    bad["keep_out"].append(
        {"id": "crate_a", "polygon_m": [[6, 6], [7, 6], [7, 7]]})
    with pytest.raises(ConfigError, match=r"duplicate keep-out id 'crate_a'"):
        ArenaMap.from_dict(bad, name="t")


# ---- keep-out polygon >= 3 distinct vertices ------------------------------
def test_keepout_two_vertices_raises():
    bad = _mut()
    bad["keep_out"][0]["polygon_m"] = [[3, 3], [4, 4]]
    with pytest.raises(ConfigError, match=r"keep_out\[0\].*>= 3 DISTINCT"):
        ArenaMap.from_dict(bad, name="t")


def test_keepout_three_collinear_but_distinct_is_allowed():
    # 3 DISTINCT vertices (even if collinear) pass the loader; full
    # simple-polygon geometry is NAV-1's concern, not the config's.
    ok = _mut()
    ok["keep_out"][0]["polygon_m"] = [[3, 3], [3, 4], [3, 5]]
    arena = ArenaMap.from_dict(ok, name="t")
    assert len(arena.keep_out[0].polygon_m) == 3


def test_keepout_three_vertices_two_duplicate_raises():
    # 3 listed vertices but only 2 distinct -> degenerate (a line segment).
    bad = _mut()
    bad["keep_out"][0]["polygon_m"] = [[3, 3], [4, 4], [3, 3]]
    with pytest.raises(ConfigError, match=r"keep_out\[0\].*>= 3 DISTINCT"):
        ArenaMap.from_dict(bad, name="t")


def test_keepout_four_listed_three_distinct_is_allowed():
    ok = _mut()
    ok["keep_out"][0]["polygon_m"] = [[3, 3], [4, 3], [4, 4], [3, 3]]
    arena = ArenaMap.from_dict(ok, name="t")
    assert len(arena.keep_out[0].polygon_m) == 4   # raw kept; distinctness ok


# ---- c2 origin within bounds ----------------------------------------------
def test_c2_origin_out_of_bounds_raises():
    with pytest.raises(ConfigError, match=r"c2_origin_m.*OUTSIDE bounds"):
        ArenaMap.from_dict(_mut(c2_origin_m=[-1.0, 4.0]), name="t")


def test_c2_origin_on_edge_is_legal():
    arena = ArenaMap.from_dict(_mut(c2_origin_m=[0.0, 0.0]), name="t")
    assert arena.c2_origin_m == (0.0, 0.0)


def test_c2_heading_nonfinite_raises():
    with pytest.raises(ConfigError, match=r"c2_heading_deg.*finite"):
        ArenaMap.from_dict(_mut(c2_heading_deg=float("nan")), name="t")


def test_c2_heading_zero_and_360_load():
    for h in (0.0, 180.0, 360.0, -180.0):
        arena = ArenaMap.from_dict(_mut(c2_heading_deg=h), name="t")
        assert arena.c2_heading_deg == h


# ---- the real shipped sample.json -----------------------------------------
def test_sample_json_parses_with_expected_counts():
    with open(_SAMPLE_PATH, "r", encoding="utf-8") as f:
        raw = json.load(f)
    arena = ArenaMap.from_dict(raw, name="sample")
    assert len(arena.pads) == 5
    assert len(arena.keep_out) == 4
    assert len(arena.lanes) == 2
    assert sum(p.valid for p in arena.pads) == 3       # 3 green
    assert sum(not p.valid for p in arena.pads) == 2   # 2 red decoys
    # every pad + the origin sit inside bounds (the validator would have
    # raised otherwise — assert the invariant explicitly).
    n0, e0, n1, e1 = arena.bounds_m
    for pad in arena.pads:
        assert n0 <= pad.center_m[0] <= n1 and e0 <= pad.center_m[1] <= e1
    assert n0 <= arena.c2_origin_m[0] <= n1


# ---- config-level resolution ----------------------------------------------
def test_load_config_mock_arena_resolves_arena():
    cfg = load_config("finals/configs/mock_arena.json")
    assert isinstance(cfg.arena, ArenaMap)
    assert cfg.arena_name == "sample"
    assert len(cfg.arena.pads) == 5


def test_load_config_missing_arena_raises(tmp_path):
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({
        "profile": "mock", "flight_backend": "mock", "frame_backend": "none",
        "detector": {"backend": "none"}, "arena_name": "no_such_arena",
        "drones": [{"id": "alpha", "phases": ["takeoff_demo"]}],
    }), encoding="utf-8")
    with pytest.raises(ConfigError, match="map file not found"):
        load_config(str(bad))


def test_pad_radius_infinite_raises():
    # radius_m = inf passes `radius > 0` (inf > 0 is True) but is a config bug
    # that would create an infinite NAV-6 hoop. Every other numeric guard checks
    # isfinite; this pins that radius_m does too.
    bad = copy.deepcopy(_VALID)
    bad["pads"][0]["radius_m"] = float("inf")
    with pytest.raises(ConfigError, match=r"radius_m must be a finite"):
        ArenaMap.from_dict(bad, name="t")


def test_load_config_propagates_semantically_malformed_arena(tmp_path):
    # A SEMANTIC arena error (pad out of bounds) must surface THROUGH
    # load_config, not only through ArenaMap.from_dict — guards a _resolve_arena
    # regression that swallows the ConfigError (the whole suite would otherwise
    # stay green while shipping a broken arena).
    arena_dir = tmp_path / "arenas"
    arena_dir.mkdir()
    bad_arena = copy.deepcopy(_VALID)
    bad_arena["pads"][0]["center_m"] = [999.0, 2.0]   # north 999 >> north_max 10
    (arena_dir / "bad_arena.json").write_text(
        json.dumps(bad_arena), encoding="utf-8")
    cfg_path = tmp_path / "cfg.json"
    cfg_path.write_text(json.dumps({
        "profile": "mock", "flight_backend": "mock", "frame_backend": "none",
        "detector": {"backend": "none"}, "arena_name": "bad_arena",
        "drones": [{"id": "alpha", "phases": ["takeoff_demo"]}],
    }), encoding="utf-8")
    with pytest.raises(ConfigError, match="OUTSIDE bounds"):
        load_config(str(cfg_path))
