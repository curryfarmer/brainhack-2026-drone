"""Tests for the hula_smoke.py vision fixes (ArUco dict-lock + allowlist, YOLO
border-reject + preproc). The pure helpers run on the bare venv; the
cv2-dependent ones (detector build, normalization, synthetic-marker decode) are
gated with importorskip so the suite stays green without the SDK."""
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


# ---------------- _touches_border (the hand-in-corner reject) ----------------
def test_touches_border_centered_box_is_kept():
    # 640x480 frame, a box well inside -> not a border touch
    assert hs._touches_border((100, 100, 300, 300), 640, 480, margin=8) is False


@pytest.mark.parametrize("box", [
    (0, 100, 50, 300),        # touches the left edge
    (100, 0, 300, 50),        # touches the top edge
    (600, 100, 640, 300),     # touches the right edge
    (100, 440, 300, 480),     # touches the bottom edge
])
def test_touches_border_edge_boxes_are_rejected(box):
    assert hs._touches_border(box, 640, 480, margin=8) is True


def test_touches_border_margin_zero_only_rejects_exact_edge():
    assert hs._touches_border((1, 1, 300, 300), 640, 480, margin=0) is False
    assert hs._touches_border((0, 1, 300, 300), 640, 480, margin=0) is True


# ---------------- _normalize_for_yolo ----------------
def test_normalize_none_is_identity_no_copy():
    sentinel = object()
    assert hs._normalize_for_yolo(sentinel, "none") is sentinel


def test_normalize_gray_world_returns_fresh_uint8_same_shape():
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    img = np.zeros((8, 8, 3), np.uint8)
    img[:, :, 0] = 200          # blue-heavy (oversaturated channel)
    img[:, :, 1] = 100
    img[:, :, 2] = 50
    out = hs._normalize_for_yolo(img, "gray-world")
    assert out is not img and out.shape == img.shape and out.dtype == np.uint8
    # gray-world pulls the channel means together (toward the global mean)
    spread_in = np.ptp(img.reshape(-1, 3).mean(0))
    spread_out = np.ptp(out.reshape(-1, 3).mean(0))
    assert spread_out < spread_in


def test_normalize_clahe_runs_and_preserves_shape():
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    img = np.full((16, 16, 3), 128, np.uint8)
    out = hs._normalize_for_yolo(img, "clahe")
    assert out.shape == img.shape and out.dtype == np.uint8


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
    assert a.edge_margin == 8
    assert a.yolo_preproc == "none"


def test_args_all_dicts_and_preproc_choices():
    a = hs._parse_args(["--all-dicts", "--yolo-preproc", "clahe",
                        "--aruco-dict", "DICT_6X6_250"])
    assert a.all_dicts is True
    assert a.yolo_preproc == "clahe"
    assert a.aruco_dict == "DICT_6X6_250"
