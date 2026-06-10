"""Tests for the hula_smoke.py ArUco vision fixes (dict-lock + field-id allowlist
+ per-id frame voting). The pure helpers run on the bare venv; the cv2-dependent
ones (detector build, synthetic-marker decode) are gated with importorskip so the
suite stays green without the SDK."""
from __future__ import annotations

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
