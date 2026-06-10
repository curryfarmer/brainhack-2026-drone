"""Tests for the hula_smoke.py ArUco vision fixes (dict-lock + field-id allowlist
+ per-id frame voting). The pure helpers run on the bare venv; the cv2-dependent
ones (detector build, synthetic-marker decode) are gated with importorskip so the
suite stays green without the SDK."""
from __future__ import annotations

import json
from collections import Counter

import pytest

from finals.tools import hula_smoke as hs


class _StubLog:
    """Minimal _Log double — collects warn/error so detector-build can report."""

    def __init__(self):
        self.warnings = []
        self.errors = []

    def warn(self, msg):
        self.warnings.append(msg)

    def error(self, msg):
        self.errors.append(msg)

    def line(self, msg=""):
        pass


# ---------------- field-marker allowlist ----------------
def test_field_aruco_ids_are_the_five_fixed_markers():
    assert hs._FIELD_ARUCO_IDS == frozenset({11, 45, 51, 67, 101})


# ---------------- _build_aruco_detectors (dict-lock) ----------------
def test_build_detectors_only_builds_one():
    pytest.importorskip("cv2")
    dets = hs._build_aruco_detectors(_StubLog(), only="DICT_7X7_1000")
    assert list(dets) == ["DICT_7X7_1000"]


def test_build_detectors_all_builds_the_full_set():
    pytest.importorskip("cv2")
    dets = hs._build_aruco_detectors(_StubLog(), only=None)
    # every available dict (APRILTAG may be absent in some cv2 builds)
    assert set(dets).issubset(set(hs._ARUCO_DICTS))
    assert "DICT_7X7_1000" in dets and "DICT_6X6_250" in dets


def test_locked_detector_decodes_the_synthetic_field_marker():
    cv2 = pytest.importorskip("cv2")
    pytest.importorskip("numpy")
    frame = hs._synthetic_marker_frame(11)          # DICT_7X7_1000 id 11
    det = hs._build_aruco_detectors(_StubLog(), only="DICT_7X7_1000")["DICT_7X7_1000"]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, ids, _ = det.detectMarkers(gray)
    assert ids is not None
    assert 11 in [int(x) for x in ids.flatten()]


# ---------------- arg parsing (no SDK) ----------------
def test_args_default_to_locked_field_dict():
    a = hs._parse_args([])
    assert a.aruco_dict == "DICT_7X7_1000"
    assert a.all_dicts is False
    assert a.yolo_conf == 0.25


def test_args_all_dicts_and_aruco_dict_choice():
    a = hs._parse_args(["--all-dicts", "--aruco-dict", "DICT_6X6_250"])
    assert a.all_dicts is True
    assert a.aruco_dict == "DICT_6X6_250"


def test_removed_yolo_postproc_knobs_are_gone():
    # --edge-margin / --yolo-preproc / --channel-order were retired: the model
    # (retrain) is the FP/quality fix now, not box post-processing.
    a = hs._parse_args([])
    assert not hasattr(a, "edge_margin")
    assert not hasattr(a, "yolo_preproc")
    assert not hasattr(a, "channel_order")
    for dead in ("--edge-margin", "--yolo-preproc", "--channel-order"):
        with pytest.raises(SystemExit):
            hs._parse_args([dead, "x"])


# ---------------- capture mode (W1: raw frames for YOLO retraining) ----------
def test_args_capture_mode_defaults():
    a = hs._parse_args([])
    assert a.capture is False          # scan is the default, not capture
    assert a.capture_secs == 180.0     # ~3 min photographer window
    assert a.capture_period == 0.5     # ~2 fps
    assert a.capture_max == 1200       # per-drone frame cap


def test_args_capture_flag_and_overrides():
    a = hs._parse_args(["--capture", "--capture-secs", "30",
                        "--capture-period", "0.25", "--capture-max", "200"])
    assert a.capture is True
    assert a.capture_secs == 30.0
    assert a.capture_period == 0.25
    assert a.capture_max == 200


def test_capture_fake_writes_raw_frames_and_manifest(tmp_path):
    # End-to-end --fake --capture: the photographer saves REAL frames (synthetic
    # numpy) + a per-drone manifest, with NO ArUco/YOLO inference. Needs cv2 +
    # numpy (cv2.imwrite + the synthetic frame); skipped on the bare venv.
    pytest.importorskip("cv2")
    pytest.importorskip("numpy")
    rc = hs.main(["--fake", "--capture", "--capture-secs", "1",
                  "--capture-period", "0.05", "--out", str(tmp_path)])
    assert rc == 0                                          # teardown never-raises
    manifests = list(tmp_path.rglob("capture_manifest.json"))
    frames = list(tmp_path.rglob("cap_*.jpg"))
    assert manifests, "no capture_manifest.json written"
    assert frames, "no raw capture frames written"


def test_capture_period_bounds_the_frame_count(tmp_path):
    # Cadence MUST throttle saves: a 1 s window @ 0.5 s period can save at most
    # ~2-3 frames per drone (kill-check — drop the period gate and this blows up).
    pytest.importorskip("cv2")
    pytest.importorskip("numpy")
    rc = hs.main(["--fake", "--capture", "--capture-secs", "1",
                  "--capture-period", "0.5", "--out", str(tmp_path)])
    assert rc == 0
    frames = list(tmp_path.rglob("cap_*.jpg"))
    assert 1 <= len(frames) <= 4, f"period not throttling saves: {len(frames)}"


# ---------------- W3: --dedup-report (per-id stability + ghosts) -------------
def test_args_dedup_report_default_and_flag():
    a = hs._parse_args([])
    assert a.dedup_report is False                     # off by default
    a = hs._parse_args(["--dedup-report"])
    assert a.dedup_report is True


def test_dedup_stats_stability_dominant_and_ghosts():
    # PURE helper, NO SDK: feed it plain Counters and assert the stability metric,
    # the dominant id, the field-valid/ghost split, and first-seen ordering.
    # frames_seen = max single-id votes; stability = votes / frames_seen.
    aruco = {"DICT_7X7_1000": Counter({11: 50, 45: 30, 999: 2})}
    first_seen = {"DICT_7X7_1000": {45: 0, 11: 1, 999: 2}}   # 45 decoded first
    rep = hs._dedup_stats(aruco, first_seen=first_seen)
    d = rep["by_dict"]["DICT_7X7_1000"]
    assert d["frames_seen"] == 50
    assert d["dominant_id"] == 11                       # most votes wins
    assert d["dominant_votes"] == 50
    # stability is votes / frames_seen — pin the exact ratios (kill-check: flip
    # the divisor and these all move).
    assert d["dominant_stability"] == pytest.approx(1.0)
    assert d["ids"]["45"]["stability"] == pytest.approx(0.6)   # 30/50
    assert d["ids"]["999"]["stability"] == pytest.approx(0.04)  # 2/50
    # field-valid {11,45,51,67,101} vs ghosts — 999 is the lone ghost.
    assert d["field_valid_ids"] == [11, 45]
    assert d["ghost_ids"] == [999]
    assert d["ids"]["11"]["field_valid"] is True
    assert d["ids"]["999"]["field_valid"] is False
    # first-seen transitions: 45 appeared first, not the dominant id.
    assert d["first_id"] == 45
    assert d["ids"]["45"]["first_seen_order"] == 0


def test_dedup_stats_skips_empty_dicts_and_no_ghosts_when_all_field():
    aruco = {"DICT_7X7_1000": Counter({51: 10, 67: 4}),
             "DICT_6X6_250": Counter()}            # empty -> skipped
    rep = hs._dedup_stats(aruco)
    assert list(rep["by_dict"]) == ["DICT_7X7_1000"]   # empty dict dropped
    d = rep["by_dict"]["DICT_7X7_1000"]
    assert d["ghost_ids"] == []                        # both ids are field-valid
    assert d["dominant_id"] == 51
    assert rep["field_ids"] == [11, 45, 51, 67, 101]


def test_dedup_report_fake_writes_artifact_with_dominant_11_no_ghosts(tmp_path):
    # End-to-end --fake --dedup-report: the locked DICT_7X7_1000 decodes the
    # injected field id 11; the persisted dedup_report.json must name 11 dominant
    # with ZERO ghosts (kill-check: a ghost classification regression trips here).
    pytest.importorskip("cv2")
    pytest.importorskip("numpy")
    rc = hs.main(["--fake", "--dedup-report", "--scan-secs", "1",
                  "--no-yolo", "--out", str(tmp_path)])
    assert rc == 0                                     # teardown never-raises
    reports = list(tmp_path.rglob("dedup_report.json"))
    assert reports, "no dedup_report.json written"
    rep = json.loads(reports[0].read_text(encoding="utf-8"))
    d = rep["by_dict"]["DICT_7X7_1000"]
    assert d["dominant_id"] == 11                       # the injected field id
    assert d["ghost_ids"] == []                         # under the locked dict
    assert d["dominant_stability"] > 0.0


# ---------------- W5: --video-only (P6 bring-up diagnostic) ------------------
def test_args_video_only_default_and_flag():
    a = hs._parse_args([])
    assert a.video_only is False                        # full scan is the default
    assert a.video_only_secs == 10.0
    a = hs._parse_args(["--video-only", "--video-only-secs", "3"])
    assert a.video_only is True
    assert a.video_only_secs == 3.0


def test_video_only_distinct_from_capture_and_scan():
    # The three stage branches are mutually-sensible flags (not the same switch).
    a = hs._parse_args(["--video-only"])
    assert a.video_only is True and a.capture is False
    a = hs._parse_args(["--capture"])
    assert a.capture is True and a.video_only is False


def test_p6_hints_embed_the_operator_fixes():
    # The known P6 fixes must be in the embedded hint string (kill-check: drop a
    # fix from _P6_VIDEO_HINTS and this catches it).
    hints = hs._P6_VIDEO_HINTS.lower()
    assert "firewall" in hints
    assert "power-cycle" in hints
    assert "video-timeout" in hints
    assert "one stream per drone" in hints


def test_video_only_fake_records_time_to_first_frame(tmp_path):
    # End-to-end --fake --video-only: first-frame time recorded, exit 0,
    # teardown never-raises, NO ArUco/YOLO artifacts. Needs cv2/numpy for the
    # synthetic frame + PyhulaxVideoSource path.
    pytest.importorskip("cv2")
    pytest.importorskip("numpy")
    rc = hs.main(["--fake", "--video-only", "--video-only-secs", "1",
                  "--out", str(tmp_path)])
    assert rc == 0                                      # teardown never-raises
    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert "video_only" in summary
    rec = next(iter(summary["video_only"].values()))
    assert rec["ok"] is True
    assert rec["time_to_first_frame_s"] >= 0.0          # first frame was timed
    assert "fps" in rec
    # video-only does NOT run the scan/aruco/yolo stages.
    assert "aruco" not in summary and "yolo" not in summary
