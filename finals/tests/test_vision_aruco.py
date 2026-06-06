"""finals.vision.aruco against the COMMITTED fixtures (generated once by
finals/tests/fixtures/gen_fixtures.py — the position table below mirrors its
MARKER_LAYOUT; change one, regenerate + change the other)."""
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
    sightings = detect_aruco(load_frame(name), "alpha")
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
    assert detect_aruco(load_frame("002.png"), "alpha") == []


def test_sightings_roundtrip_through_csv(tmp_path):
    """End-to-end np-type leak detector: the CSV codec dispatches on the
    DECLARED field types, so any numpy scalar that survived the casts dies
    here — and reload must reproduce the rows exactly."""
    sightings = detect_aruco(load_frame("000.png"), "alpha")
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
    aruco = make_marker_detector("aruco")
    qr = make_marker_detector("qr")
    frame = load_frame("000.png")
    assert sorted(s.marker_id for s in aruco(frame, "a")) == [17, 23, 42]
    assert qr(frame, "a") == []                 # no QR on an aruco frame
    with pytest.raises(ConfigError, match="apriltag"):
        make_marker_detector("apriltag")
