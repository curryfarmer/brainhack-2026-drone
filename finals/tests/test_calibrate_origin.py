"""finals.tools.calibrate_origin — ORIGIN-CAL gate-D calibration helper.

Pins the helper that turns a measured cage arena into a checkable artifact:
- the arena is VALIDATED through the REAL loader (a bad measurement raises);
- each drone's card carries the bearing/distance from the REAL planner +
  frame.bearing_from_c2_deg (never reimplemented here);
- the heading_offset_deg (Delta) shows in the card's "compass reads" column but
  does NOT bend the planned world geometry (it only shifts the Rotate target —
  same split as navigate);
- an unreachable goal records a reason instead of crashing the whole run;
- the ASCII map always renders (matplotlib-free).

Pure stdlib — calibrate_origin imports the pure planner/frame/loader; matplotlib
is lazy (the one PNG test is importorskip-gated).
"""
from __future__ import annotations

import math
import os

import pytest

from finals.errors import FinalsError
from finals.mission.planning.types import ArenaMap
from finals.tools.calibrate_origin import (CalibrateError, _assignments_all_pads,
                                           _route_points, ascii_map, build_cards,
                                           format_cards, run)

# A tiny corner-frame cage: C2 at (1,3); one pad due +north at (9,3) -> bearing
# 0, distance 8 m. Mutated per test.
_ARENA = {
    "bounds_m": [0.0, 0.0, 10.0, 6.0],
    "c2_origin_m": [1.0, 3.0],
    "c2_heading_deg": 0.0,
    "heading_offset_deg": 0.0,
    "keep_out": [],
    "pads": [{"id": "p_n", "center_m": [9.0, 3.0], "radius_m": 0.3,
              "valid": True}],
}

_CAGE_PATH = os.path.join(os.path.dirname(__file__), "..", "configs",
                          "arenas", "cage.json")


def _cards(arena_raw):
    arena = ArenaMap.from_dict(arena_raw, name="t")
    cards = build_cards(arena, _assignments_all_pads(arena, 0.5, 100.0))
    return arena, cards


# ---- card geometry comes from the REAL planner ----------------------------
def test_card_bearing_and_distance_match_hand_calc():
    arena, cards = _cards(_ARENA)
    assert len(cards) == 1
    card = cards[0]
    assert card.error is None and card.legs
    # straight line C2(1,3)->p_n(9,3): dN=8, dE=0 -> bearing 0, distance 8 m.
    total_m = sum(l.distance_cm for l in card.legs) / 100.0
    assert total_m == pytest.approx(8.0, abs=0.01)
    assert all(l.heading_deg == pytest.approx(0.0) for l in card.legs)
    report = format_cards(arena, cards, arena_name="t")
    assert "bearing +0.0" in report
    assert "8.00 m" in report


# ---- heading_offset shifts the COMPASS target, not the world geometry ------
def test_offset_shifts_compass_column_only():
    raw = dict(_ARENA, heading_offset_deg=30.0)
    arena, cards = _cards(raw)
    card = cards[0]
    # The planned legs stay arena-frame (heading 0); the offset is a DISPLAY/
    # navigate-target shift, so the world route is unchanged.
    assert all(l.heading_deg == pytest.approx(0.0) for l in card.legs)
    pts = _route_points(arena.c2_origin_m, card.legs, arena.heading_offset_deg)
    assert pts[-1][0] == pytest.approx(9.0, abs=0.01)   # ends due north
    assert pts[-1][1] == pytest.approx(3.0, abs=0.01)
    report = format_cards(arena, cards, arena_name="t")
    # arena heading 0 + Delta 30 -> compass reads +30.0.
    assert "compass reads   +30.0" in report
    assert "+30.00 deg" in report           # the Delta header line


# ---- a trapped goal records a reason, never crashes -----------------------
def test_unreachable_goal_recorded_not_raised():
    # A keep-out box AROUND the pad; with inflation the goal sits inside the
    # inflated polygon -> plan() raises PlanningError -> recorded as an error
    # card (the other cards in a real run would still render).
    raw = dict(_ARENA, keep_out=[
        {"id": "box", "polygon_m": [[8.0, 2.0], [10.0, 2.0], [10.0, 4.0],
                                    [8.0, 4.0]]}])
    arena = ArenaMap.from_dict(raw, name="t")
    cards = build_cards(arena, _assignments_all_pads(arena, 0.5, 100.0))
    assert len(cards) == 1
    assert cards[0].legs is None
    assert cards[0].error                       # a non-empty reason string
    report = format_cards(arena, cards, arena_name="t")
    assert "UNREACHABLE" in report


# ---- ASCII map always renders (matplotlib-free) ---------------------------
def test_ascii_map_marks_c2_and_pad():
    arena, cards = _cards(_ARENA)
    art = ascii_map(arena, cards)
    assert "*" in art                           # C2
    assert "o" in art                           # the pad
    assert "C2" in art and "pad" in art         # the legend


# ---- the REAL loader is the validation: bad numbers raise -----------------
def test_run_rejects_out_of_bounds_pad(tmp_path):
    import json
    bad = dict(_ARENA, pads=[{"id": "p", "center_m": [99.0, 3.0],
                              "radius_m": 0.3, "valid": True}])
    p = tmp_path / "bad.json"
    p.write_text(json.dumps(bad), encoding="utf-8")
    # ConfigError is a FinalsError; run() lets it propagate (main turns it into
    # exit 2). The point: a fat-fingered measurement dies at load, not mid-air.
    with pytest.raises(FinalsError):
        run(str(p), config_path=None, all_pads=True, inflation_m=0.5,
            max_leg_cm=100.0, save=None, no_plot=True)


def test_run_missing_arena_file_raises_calibrate_error():
    with pytest.raises(CalibrateError, match=r"does not exist"):
        run("does_not_exist_arena.json", config_path=None, all_pads=True,
            inflation_m=0.5, max_leg_cm=100.0, save=None, no_plot=True)


# ---- the SHIPPED cage.json loads + cards (regression on the real config) ----
def test_shipped_cage_json_cards_all_pads():
    report = run(_CAGE_PATH, config_path=None, all_pads=True, inflation_m=0.5,
                 max_leg_cm=100.0, save=None, no_plot=True)
    for pad_id in ("pad_north", "pad_se", "pad_mid"):
        assert pad_id in report
    assert "UNREACHABLE" not in report          # keep_out=[] -> all reachable


# ---- the PNG path (skip when matplotlib absent) ---------------------------
def test_png_written_when_matplotlib_present(tmp_path):
    pytest.importorskip("matplotlib")
    out = tmp_path / "cage.png"
    run(_CAGE_PATH, config_path=None, all_pads=True, inflation_m=0.5,
        max_leg_cm=100.0, save=str(out), no_plot=False)
    assert out.is_file() and out.stat().st_size > 0
