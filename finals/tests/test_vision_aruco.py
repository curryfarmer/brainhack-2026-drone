"""finals.vision.aruco against the COMMITTED fixtures (generated once by
finals/tests/fixtures/gen_fixtures.py — the position table below mirrors its
MARKER_LAYOUT; change one, regenerate + change the other).

The committed ArUco fixtures are DICT_6X6_250 (gen_fixtures.py), so every legacy
decode call below pins marker_dict=SIM_DICT — the detector's NEW default is the
real-field DICT_7X7_1000 (PAD-DICT). The dictionary-resolver, params-whitelist
and 7x7-vs-6x6 e2e tests live in the PAD-DICT section at the bottom."""
from __future__ import annotations

import os

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from finals.errors import ConfigError                            # noqa: E402
from finals.sightings import SightingLog                         # noqa: E402
from finals.types import FrameStamped                            # noqa: E402
from finals.vision.aruco import (detect_aruco, detect_qr,        # noqa: E402
                                 make_marker_detector)

# The committed ArUco fixtures are 6x6; pin every legacy decode to that dict.
SIM_DICT = "DICT_6X6_250"


def _aruco6(frame, drone_id="alpha"):
    """detect_aruco over the 6x6 fixture dictionary (the legacy default)."""
    return detect_aruco(frame, drone_id, marker_dict=SIM_DICT)

FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "fixtures")
MARKER_PX = 120
# frame file -> {marker_id: (x, y) top-left}; bbox = x, y, x+120, y+120.
# Mirrors gen_fixtures.MARKER_LAYOUT.
LAYOUT = {
    "000.png": {17: (40, 60), 23: (260, 180), 42: (480, 300)},
    "001.png": {17: (300, 40), 23: (60, 280), 42: (400, 200)},
    "002.png": {},
    "003.png": {17: (500, 40), 23: (40, 40), 42: (240, 300)},
}
BBOX_TOL_PX = 3.0


def load_frame(name: str, frame_number: int = 1, ts: float = 100.0,
               source_id: str = "replay") -> FrameStamped:
    path = os.path.join(FIXTURES, "frames", name)
    image = cv2.imread(path)
    assert image is not None, f"committed fixture missing/corrupt: {path}"
    return FrameStamped(image=image, ts=ts, frame_number=frame_number,
                        source_id=source_id)


@pytest.mark.parametrize("name", ["000.png", "001.png", "003.png"])
def test_detect_known_markers_ids_and_bboxes(name):
    sightings = _aruco6(load_frame(name))
    expected = LAYOUT[name]
    assert sorted(s.marker_id for s in sightings) == sorted(expected)
    for s in sightings:
        x, y = expected[s.marker_id]
        want = (x, y, x + MARKER_PX, y + MARKER_PX)
        for got_v, want_v in zip(s.bbox_xyxy, want):
            assert abs(got_v - want_v) <= BBOX_TOL_PX, (
                f"id {s.marker_id}: bbox {s.bbox_xyxy} vs pasted {want} "
                f"(tolerance {BBOX_TOL_PX} px)")
        # The exact f-string pins the int() cast: a leaked numpy scalar
        # formats as "aruco_[17]" / "aruco_np.int32(17)".
        assert s.class_name == f"aruco_{s.marker_id}"
        assert isinstance(s.marker_id, int)
        assert s.source == "aruco"
        assert s.confidence == 1.0
        assert s.frame_shape == (480, 640)
        assert s.drone_id == "alpha"
        assert s.ts == 100.0 and s.frame_number == 1
        assert s.bearing_deg is None and s.drone_yaw_deg is None, (
            "detectors emit MINIMAL sightings; enrichment is perception's job")


def test_blank_frame_returns_empty():
    assert _aruco6(load_frame("002.png")) == []


def test_sightings_roundtrip_through_csv(tmp_path):
    """End-to-end np-type leak detector: the CSV codec dispatches on the
    DECLARED field types, so any numpy scalar that survived the casts dies
    here — and reload must reproduce the rows exactly."""
    sightings = _aruco6(load_frame("000.png"))
    csv_path = str(tmp_path / "sightings.csv")
    with SightingLog(csv_path) as log:
        for s in sightings:
            log.append(s)
    with SightingLog(csv_path) as reloaded:
        assert reloaded.snapshot() == sightings


def test_qr_fixture_decodes_payload():
    path = os.path.join(FIXTURES, "frames_qr", "000.png")
    image = cv2.imread(path)
    assert image is not None, f"committed fixture missing: {path}"
    frame = FrameStamped(image=image, ts=5.0, frame_number=7,
                         source_id="replay")
    sightings = detect_qr(frame, "bravo")
    assert len(sightings) == 1
    s = sightings[0]
    assert s.source == "qr"
    assert s.class_name == "qr_7"
    assert s.marker_id == 7                     # numeric payload -> int id
    assert s.confidence == 1.0
    assert s.frame_shape == (480, 640)
    # The QR was pasted centered (gen_fixtures): the bbox must cover it.
    x1, y1, x2, y2 = s.bbox_xyxy
    assert x1 < 320 < x2 and y1 < 240 < y2


def test_qr_on_blank_frame_returns_empty():
    assert detect_qr(load_frame("002.png"), "alpha") == []


def test_qr_weird_payloads_never_crash_and_never_alias(tmp_path):
    """str.isdigit() is true for characters int() REFUSES ("²") — one
    garbled/hostile sticker must yield marker_id None, not a perception-
    killing ValueError; non-ASCII decimals ("٣") must not silently alias
    marker id 3; newlines are sanitized before the CSV codec sees them."""
    from finals.vision.aruco import _detect_qr_with

    quad = np.array([[[10.0, 10.0], [60.0, 10.0],
                      [60.0, 60.0], [10.0, 60.0]]], dtype=np.float32)
    payloads = ["²", "٣", "7", "line1\r\nline2", ""]

    class FakeQrDetector:
        def detectAndDecodeMulti(self, image):
            return (True, payloads,
                    np.repeat(quad, len(payloads), axis=0), None)

    frame = load_frame("002.png", frame_number=9)
    sightings = _detect_qr_with(FakeQrDetector(), frame, "alpha")
    assert len(sightings) == 4                  # the empty payload is skipped
    by_id = {s.class_name: s.marker_id for s in sightings}
    assert by_id["qr_²"] is None           # superscript two: no crash
    assert by_id["qr_٣"] is None           # Arabic-Indic 3: no aliasing
    assert by_id["qr_7"] == 7
    for s in sightings:
        assert "\n" not in s.class_name and "\r" not in s.class_name
    # Every one of them survives the CSV codec end to end.
    with SightingLog(str(tmp_path / "qr.csv")) as log:
        for s in sightings:
            log.append(s)
    with SightingLog(str(tmp_path / "qr.csv")) as reloaded:
        assert reloaded.snapshot() == sightings


def test_make_marker_detector_seam():
    aruco = make_marker_detector("aruco", marker_dict=SIM_DICT)
    qr = make_marker_detector("qr")
    frame = load_frame("000.png")
    assert sorted(s.marker_id for s in aruco(frame, "a")) == [17, 23, 42]
    assert qr(frame, "a") == []                 # no QR on an aruco frame
    with pytest.raises(ConfigError, match="apriltag"):
        make_marker_detector("apriltag")


# ============================================================
# S11 — save_marker_frames (the photo demo)
# ============================================================
def test_no_save_dir_leaves_frame_path_none(tmp_path):
    """The default (save_marker_frames off): minimal Sightings, no files."""
    detector = make_marker_detector("aruco", marker_dict=SIM_DICT)  # save_dir omitted
    sightings = detector(load_frame("000.png"), "alpha")
    assert sightings and all(s.frame_path is None for s in sightings)
    assert list(tmp_path.iterdir()) == []               # nothing written


def test_save_marker_frames_writes_annotated_and_stamps_path(tmp_path):
    """save_dir set -> one annotated JPEG per frame-with-markers, and EVERY
    Sighting on that frame carries its path (drawDetectedMarkers draws all)."""
    save_dir = str(tmp_path / "marker_frames" / "alpha")
    detector = make_marker_detector("aruco", marker_dict=SIM_DICT,
                                    save_dir=save_dir)
    assert os.path.isdir(save_dir), "save_dir is created at build time"
    frame = load_frame("000.png", frame_number=5, ts=12.0)
    sightings = detector(frame, "alpha")
    assert len(sightings) == 3
    paths = {s.frame_path for s in sightings}
    assert len(paths) == 1                              # all share the one frame
    path = paths.pop()
    assert path is not None and os.path.isfile(path)
    assert os.path.basename(path) == "aruco_alpha_5_12000.jpg"   # id_fnum_tsms
    saved = cv2.imread(path)
    assert saved is not None and saved.shape == frame.image.shape


def test_save_marker_frames_blank_frame_writes_nothing(tmp_path):
    """No markers -> no Sightings AND no file (we only save frames with reads)."""
    save_dir = str(tmp_path / "frames")
    detector = make_marker_detector("aruco", marker_dict=SIM_DICT,
                                    save_dir=save_dir)
    assert detector(load_frame("002.png"), "alpha") == []
    assert list(os.scandir(save_dir)) == []


def test_save_marker_frames_bad_dir_fails_loudly(tmp_path):
    """A save_dir that cannot be created dies on the ground (fail-loud), not
    silently mid-flight — mirrors DetectorPool's save_dir guard."""
    clash = tmp_path / "afile"
    clash.write_text("not a dir")
    with pytest.raises(ConfigError, match="save_dir"):
        make_marker_detector("aruco", save_dir=str(clash / "frames"))


# ============================================================
# PAD-DICT — DICT_7X7_1000 dictionary fix + DetectorParameters whitelist
# ============================================================
# The real field is DICT_7X7_1000 (beacons 11/45/51/67/101); the detector USED
# to hardcode DICT_6X6_250 and read NOTHING off the real field. These tests pin
# the resolver, the params whitelist, and the bug-existed proof (a 6x6 frame must
# NOT decode under the 7x7 detector).
from finals.config import VALID_MARKER_DICTS                     # noqa: E402
from finals.vision.aruco import (_resolve_detector_params,       # noqa: E402
                                 _resolve_marker_dict)

REAL_DICT = "DICT_7X7_1000"
FIELD_IDS = [11, 45, 51, 67, 101]
_MARKER7_PX = 140                       # 9 modules (7x7+border) -> ~15 px/module


def _frame_with_markers(dict_name: str, ids, px: int = _MARKER7_PX,
                        ts: float = 7.0, frame_number: int = 3) -> FrameStamped:
    """A 640x480 white canvas with `ids` drawn from `dict_name`, laid out on a
    grid so none overlap. Generated IN-test (no new committed fixture)."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    canvas = np.full((480, 640, 3), 255, dtype=np.uint8)
    # up to 5 markers on a simple grid: cols of `px`+gap, two rows.
    for i, mid in enumerate(ids):
        col, row = i % 3, i // 3
        x = 20 + col * (px + 20)
        y = 20 + row * (px + 20)
        marker = cv2.aruco.generateImageMarker(dictionary, mid, px)
        canvas[y:y + px, x:x + px] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return FrameStamped(image=canvas, ts=ts, frame_number=frame_number,
                        source_id="replay")


# --- dict-name resolver --------------------------------------------------
def test_resolve_marker_dict_valid_name_returns_constant():
    assert _resolve_marker_dict("DICT_7X7_1000") == cv2.aruco.DICT_7X7_1000
    assert _resolve_marker_dict("DICT_6X6_250") == cv2.aruco.DICT_6X6_250


def test_resolve_marker_dict_unknown_name_is_loud():
    # a typo cv2 would otherwise turn into an AttributeError deep in the thread.
    with pytest.raises(ConfigError, match="unknown marker_dict"):
        _resolve_marker_dict("DICT_7x7_1000")          # lowercase x
    with pytest.raises(ConfigError, match="DICT_7X7_1000"):
        _resolve_marker_dict("DICT_8X8_250")           # not a real dict


def test_valid_marker_dicts_all_resolve_on_this_cv2():
    """Every name config promises must be a real cv2.aruco constant (the
    aruco import-time guard asserts this; pinned here too for visibility)."""
    for name in VALID_MARKER_DICTS:
        assert _resolve_marker_dict(name) == getattr(cv2.aruco, name)


# --- DetectorParameters whitelist ----------------------------------------
def test_resolve_detector_params_none_is_library_defaults():
    dp = _resolve_detector_params(None)
    ref = cv2.aruco.DetectorParameters()
    assert dp.adaptiveThreshWinSizeMin == ref.adaptiveThreshWinSizeMin
    assert dp.minMarkerPerimeterRate == ref.minMarkerPerimeterRate


def test_resolve_detector_params_applies_whitelisted_overrides():
    dp = _resolve_detector_params({"adaptiveThreshWinSizeMin": 5,
                                   "minMarkerPerimeterRate": 0.01})
    assert dp.adaptiveThreshWinSizeMin == 5
    assert dp.minMarkerPerimeterRate == pytest.approx(0.01)


def test_resolve_detector_params_typo_key_is_loud():
    # a typo'd field name is a SILENT no-op on the real DetectorParameters —
    # it must be a loud ConfigError instead (the faint-beacon tuning trap).
    with pytest.raises(ConfigError, match="minMarkerPerimterRate"):
        _resolve_detector_params({"minMarkerPerimterRate": 0.01})   # typo'd


def test_make_marker_detector_threads_params_typo_is_loud():
    with pytest.raises(ConfigError, match="unknown key"):
        make_marker_detector("aruco", marker_dict=REAL_DICT,
                             aruco_detector_params={"notARealField": 1})


def test_make_marker_detector_bad_dict_is_loud():
    with pytest.raises(ConfigError, match="unknown marker_dict"):
        make_marker_detector("aruco", marker_dict="DICT_NOPE")


# --- the bug-existed proof: 7x7 decodes, 6x6 does NOT under the 7x7 detector --
def test_seven_by_seven_field_markers_decode_under_7x7():
    """The campaign-critical fix: the real DICT_7X7_1000 beacons
    (11/45/51/67/101) decode under the 7x7 detector. (Mutant (a) — default
    flipped back to 6X6 — fails HERE.)"""
    detector = make_marker_detector("aruco", marker_dict=REAL_DICT)
    frame = _frame_with_markers(REAL_DICT, FIELD_IDS)
    sightings = detector(frame, "alpha")
    assert sorted(s.marker_id for s in sightings) == sorted(FIELD_IDS)
    assert all(s.source == "aruco" for s in sightings)


def test_six_by_six_frame_does_not_decode_under_7x7():
    """Proves the bug EXISTED: the committed 6x6 fixtures read NOTHING under the
    7x7 detector — exactly why a 6x6-hardcoded detector saw nothing on the real
    7x7 field. (Mutant (b) — resolver ignores config + returns a fixed dict —
    fails HERE.)"""
    detector = make_marker_detector("aruco", marker_dict=REAL_DICT)
    assert detector(load_frame("000.png"), "alpha") == []      # 000.png is 6x6


def test_default_marker_dict_is_the_real_field():
    """make_marker_detector + detect_aruco default to the REAL field (7x7), so a
    caller that forgets the knob still reads the real beacons, not nothing."""
    detector = make_marker_detector("aruco")                   # no marker_dict
    frame = _frame_with_markers(REAL_DICT, FIELD_IDS)
    assert sorted(s.marker_id for s in detector(frame, "a")) == sorted(FIELD_IDS)
    # detect_aruco shares the default.
    assert sorted(s.marker_id for s in detect_aruco(frame, "a")) == \
        sorted(FIELD_IDS)
