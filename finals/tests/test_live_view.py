"""Tests for live_view.py — the real-time CV feed visualiser.

Two tiers, so the suite stays green on the BARE venv (no cv2/numpy):
  * PURE-LOGIC tests (vote aggregation, dominant-id tie-break, ghost
    classification, HUD text formatting, YOLO box adapter) run with NO SDK.
  * cv2/numpy-DEPENDENT tests (the annotate_* draw fns return a real ndarray;
    the --fake --headless e2e path) are gated with pytest.importorskip.

The cv2.imshow window loop (_run_window) is NEVER called here — CI must never
open a window. Mutation kill-check notes are inline on the boundary asserts
(ghost-vs-field classification + the dominant-id tie-break)."""
from __future__ import annotations

from collections import Counter

import pytest

from finals.tools import live_view as lv


# ============================================================
# PURE: ghost-vs-field classification  (NO SDK)
# ============================================================
def test_field_ids_are_not_ghosts():
    # All five fixed field markers classify as NOT ghost.
    for fid in (11, 45, 51, 67, 101):
        assert lv.classify_ghost(fid) is False


def test_non_field_ids_are_ghosts():
    # KILL-CHECK: the boundary is `id NOT in {11,45,51,67,101}`. If the source
    # flipped the comparison (returned `id IN allowlist`), these would fail.
    for gid in (0, 1, 12, 44, 100, 102, 999):
        assert lv.classify_ghost(gid) is True


def test_classify_ghost_honors_custom_allowlist():
    # A custom allowlist re-bases the boundary — 7 is field here, 11 is ghost.
    assert lv.classify_ghost(7, field_ids={7, 8}) is False
    assert lv.classify_ghost(11, field_ids={7, 8}) is True


def test_classify_ghost_boundary_is_exact():
    # KILL-CHECK on the exact set membership: 51 is in, 50 and 52 are out.
    assert lv.classify_ghost(51) is False
    assert lv.classify_ghost(50) is True
    assert lv.classify_ghost(52) is True


# ============================================================
# PURE: vote aggregation + dominant-id tie-break  (NO SDK)
# ============================================================
def test_update_votes_counts_per_id():
    c = Counter()
    lv.update_votes(c, [11, 11, 45])
    lv.update_votes(c, [11])
    assert c[11] == 3
    assert c[45] == 1


def test_update_votes_none_and_empty_are_noops():
    c = Counter()
    lv.update_votes(c, None)
    lv.update_votes(c, [])
    assert sum(c.values()) == 0


def test_dominant_id_picks_most_voted():
    c = Counter({11: 5, 45: 2, 51: 1})
    assert lv.dominant_id(c) == 11


def test_dominant_id_empty_is_none():
    assert lv.dominant_id(Counter()) is None


def test_dominant_id_tie_break_is_smallest_id():
    # KILL-CHECK: on an EXACT vote tie the SMALLEST id wins (deterministic, so
    # the operator's HUD does not flicker). If the source used max()/dict-order
    # instead, 67 (or an arbitrary key) could win — this pins 45.
    c = Counter({67: 4, 45: 4, 101: 4})
    assert lv.dominant_id(c) == 45
    # and the winner must be one of the tied ids, never an outvoted one
    c2 = Counter({11: 1, 45: 9, 101: 9})
    assert lv.dominant_id(c2) == 45        # 45 and 101 tie at 9; 45 is smaller


def test_dominant_id_count_beats_id_order():
    # A higher count must win even when its id is larger than a low-count id —
    # guards against accidentally sorting by id instead of by votes.
    c = Counter({1: 1, 999: 5})
    assert lv.dominant_id(c) == 999


def test_vote_summary_splits_field_and_ghosts():
    c = Counter({11: 3, 45: 1, 7: 2, 999: 1})
    s = lv.vote_summary(c)
    assert s["dominant"] == 11
    assert s["field_ids"] == [11, 45]
    assert s["ghost_ids"] == [7, 999]
    assert s["total_votes"] == 7
    # pairs sorted by descending votes then ascending id
    assert s["pairs"][0] == (11, 3)


# ============================================================
# PURE: HUD top-line formatting + YOLO box adapter  (NO SDK)
# ============================================================
def test_hud_top_lines_formats_numbers_and_na():
    lines = lv._hud_top_lines({
        "fps": 12.0, "battery_pct": 87.0, "yaw_deg": 10.0, "frames": 5,
        "channel_order": "bgr", "aruco_on": True, "yolo_on": False})
    text = "\n".join(lines)
    assert "fps 12.0" in text
    assert "batt 87.0%" in text
    assert "yaw 10.0deg" in text
    assert "frames 5" in text
    assert "ArUco [on]" in text and "YOLO [OFF]" in text


def test_hud_top_lines_missing_values_show_na():
    lines = lv._hud_top_lines({})
    text = "\n".join(lines)
    assert "n/a" in text                       # battery/yaw/fps unknown
    assert "frames n/a" in text


class _FakeBox:
    def __init__(self, cls, conf, xyxy):
        self.cls = [cls]
        self.conf = [conf]
        self.xyxy = [xyxy]


class _FakeResult:
    def __init__(self, boxes):
        self.boxes = boxes


def test_yolo_boxes_from_result_keeps_every_box():
    # No edge rejection: BOTH boxes (incl. one touching the frame border) are
    # returned. KILL-CHECK against any reintroduced border/edge filtering.
    names = {0: "landing_pad"}
    res = _FakeResult([
        _FakeBox(0, 0.9, (10, 10, 100, 100)),
        _FakeBox(0, 0.3, (0, 0, 50, 50)),          # touches the border, kept
    ])
    out = lv.yolo_boxes_from_result(res, names)
    assert len(out) == 2
    assert out[0] == ("landing_pad", pytest.approx(0.9), (10.0, 10.0, 100.0, 100.0))


def test_yolo_boxes_from_result_empty():
    assert lv.yolo_boxes_from_result(_FakeResult(None), {}) == []
    assert lv.yolo_boxes_from_result(_FakeResult([]), {}) == []


# ============================================================
# cv2/numpy: annotate_* return a real annotated ndarray, headless
# ============================================================
def _blank(h=480, w=640):
    import numpy as np
    return np.full((h, w, 3), 128, np.uint8)


def test_annotate_aruco_returns_copy_and_draws():
    pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    img = _blank()
    # one square marker corner set + an id
    corners = [np.array([[[200, 140], [420, 140], [420, 340], [200, 340]]],
                        dtype=np.float32)]
    ids = np.array([[11]])
    out = lv.annotate_aruco(img, corners, ids)
    assert out.shape == img.shape
    assert out is not img                       # never mutates the input
    assert not np.array_equal(out, img)         # something WAS drawn


def test_annotate_aruco_no_detection_returns_unchanged_copy():
    pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    img = _blank()
    out = lv.annotate_aruco(img, None, None)
    assert out is not img
    assert np.array_equal(out, img)             # nothing decoded -> unchanged


def test_annotate_aruco_ghost_draws_differently_than_field():
    pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    corners = [np.array([[[200, 140], [420, 140], [420, 340], [200, 340]]],
                        dtype=np.float32)]
    field = lv.annotate_aruco(_blank(), corners, np.array([[11]]))
    ghost = lv.annotate_aruco(_blank(), corners, np.array([[999]]))
    # KILL-CHECK: a ghost id (999, outside the allowlist) must render
    # DIFFERENTLY from a field id (11) — the warning color/outline. If
    # classify_ghost were inverted, these two frames would match.
    assert not np.array_equal(field, ghost)


def test_annotate_yolo_draws_every_box():
    pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    img = _blank()
    boxes = [("landing_pad", 0.9, (10, 10, 100, 100)),
             ("landing_pad", 0.2, (0, 0, 40, 40))]   # low-conf, still drawn
    out = lv.annotate_yolo(img, boxes)
    assert out is not img
    assert not np.array_equal(out, img)


def test_annotate_yolo_empty_returns_unchanged_copy():
    pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    img = _blank()
    out = lv.annotate_yolo(img, [])
    assert out is not img
    assert np.array_equal(out, img)


def test_draw_hud_renders_and_does_not_mutate():
    pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    img = _blank()
    stats = {"fps": 10.0, "battery_pct": 87.0, "yaw_deg": 10.0, "frames": 3,
             "channel_order": "bgr", "votes": Counter({11: 5, 999: 1})}
    out = lv.draw_hud(img, stats, show_votes=True)
    assert out is not img
    assert not np.array_equal(out, img)         # HUD text was drawn


def test_draw_hud_votes_toggle_changes_output():
    pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    img = _blank()
    stats = {"votes": Counter({11: 5, 45: 2})}
    with_votes = lv.draw_hud(img, stats, show_votes=True)
    without = lv.draw_hud(img, stats, show_votes=False)
    # the 'd' dedup view actually adds the vote block -> different pixels.
    assert not np.array_equal(with_votes, without)


# ============================================================
# arg parsing (NO SDK)
# ============================================================
def test_args_defaults():
    a = lv._parse_args([])
    assert a.aruco_dict == "DICT_7X7_1000"
    assert a.all_dicts is False
    assert a.yolo_conf == 0.25
    assert a.no_yolo is False
    assert a.headless is False
    assert a.fake is False


def test_args_no_edge_margin_knob_exists():
    # The --edge-margin / box post-processing concept was REMOVED this week —
    # it must NOT exist on live_view (the corrected spec). KILL-CHECK against a
    # reintroduction.
    a = lv._parse_args([])
    assert not hasattr(a, "edge_margin")
    for dead in ("--edge-margin", "--yolo-preproc"):
        with pytest.raises(SystemExit):
            lv._parse_args([dead, "x"])


def test_args_headless_alias():
    a = lv._parse_args(["--no-window"])
    assert a.headless is True
    b = lv._parse_args(["--headless", "--frames", "10"])
    assert b.headless is True and b.frames == 10


# ============================================================
# E2E: --fake --headless runs the WHOLE path with no SDK-backend, no window
# ============================================================
def test_fake_headless_e2e_runs_and_votes(tmp_path):
    # The full tool path: connect a FakeDroneAPI, open the synthetic-marker
    # stream, run detect + the full annotate pipeline headless, exit clean.
    # Needs cv2 + numpy (the synthetic frame + ArUco decode + draw). The window
    # loop is NEVER touched.
    pytest.importorskip("cv2")
    pytest.importorskip("numpy")
    rc = lv.main(["--fake", "--headless", "--frames", "8",
                  "--no-yolo", "--out", str(tmp_path)])
    assert rc == 0                              # teardown never-raises
    log = (tmp_path / "live_view.log").read_text(encoding="utf-8")
    # the synthetic frame carries field id 11 -> it must dominate, no ghosts.
    assert "dominant ArUco id 11" in log
    assert "ghosts []" in log


def test_fake_headless_snapshot_on_exit_writes_both(tmp_path):
    pytest.importorskip("cv2")
    pytest.importorskip("numpy")
    rc = lv.main(["--fake", "--headless", "--frames", "4", "--no-yolo",
                  "--snapshot-on-exit", "--out", str(tmp_path)])
    assert rc == 0
    raws = list(tmp_path.rglob("*_raw.jpg"))
    annots = list(tmp_path.rglob("*_annot.jpg"))
    assert raws, "snapshot did not save a raw frame"
    assert annots, "snapshot did not save an annotated frame"


def test_fake_headless_ghost_marker_flags_ghost(tmp_path):
    # KILL-CHECK end-to-end: bake a NON-field id (999) into the fake frame; the
    # vote summary must report it as a ghost and NOT as a field id.
    pytest.importorskip("cv2")
    pytest.importorskip("numpy")
    rc = lv.main(["--fake", "--fake-marker-id", "999", "--headless",
                  "--frames", "6", "--no-yolo", "--out", str(tmp_path)])
    assert rc == 0
    log = (tmp_path / "live_view.log").read_text(encoding="utf-8")
    assert "ghosts [999]" in log
    assert "field []" in log                    # 999 is NOT a field id
