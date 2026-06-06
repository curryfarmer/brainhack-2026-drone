"""Marker detection — pure functions, frame in, Sightings out.

THE PRIMARY DETECTOR (user-confirmed 2026-06-06): the convoy robots carry
markers, and the mission is to detect and READ them. detectMarkers returns
the marker ID directly — no training, no weights, no sim-to-real gap.
YOLO (finals/vision/detector.py) is the optional, config-gated extra.

Two pluggable paths feed the SAME Sighting stream (the QR-vs-ArUco intel
question — "QR codes, 20x20 cm" — is still unconfirmed, so the seam is a
config knob, marker_backend, not a code edit):
- detect_aruco: cv2.aruco DICT_6X6_250 (the default);
- detect_qr:    cv2.QRCodeDetector.detectAndDecodeMulti (the alternate).

Design (binding):
- Cheap (~2-5 ms/frame) and deterministic -> runs SYNCHRONOUSLY inside each
  drone's perception loop on EVERY sampled frame (YOLO goes through the
  worker pool; marker detection does not need to). Cost permits high sample
  rates on all 3 streams — important for MOVING targets, where motion blur
  and short sight windows are the enemies of marker reads.
- Detectors emit MINIMAL Sightings (ts/source/class_name/marker_id/bbox/
  confidence/frame_shape/frame_number); yaw/alt/bearing enrichment is the
  perception loop's job (dataclasses.replace — Sighting is frozen).
- Every numeric leaving cv2 is explicitly cast (int()/float()): numpy
  scalars otherwise leak into class_name (f"aruco_[17]") and into the
  SightingLog CSV codec, which dispatches on declared types.
- frame_shape is image.shape[:2] as an int 2-tuple — the CSV codec REFUSES
  a 3-tuple at append time (finals/sightings.py arity guard).
- QR payloads are sanitized (CR/LF stripped, capped) BEFORE entering a
  Sighting: an embedded newline raises at CSV-append time and a failed
  append poisons the SightingLog (finals/sightings.py) — sanitize at the
  source, not in the log.
- Marker-ID semantics (which IDs = convoy robots, which = landing pads) come
  from config, never hardcoded here — briefing-day edits stay config edits.
- NO depth: the laptop video stream has none. The RealSense depth/
  deprojection half of the example is intentionally dropped.

Derives from: docs/finals/example_code/potential_detection_targets.py lines
5-30 (audited, not copied) — NOTE that file is pseudocode with a SYNTAX
ERROR: `corners, ids,  = detector.detectMarkers(...)` — detectMarkers
returns THREE values (corners, ids, rejected); fixed here. Its depth/
deprojection half (lines 31-37) is deliberately dropped (no depth source).

cv2 is imported at module top level (SDK_ALLOWED) — this module is only
ever imported lazily, by make_marker_detector's callers (main.py wiring)
and the cv2-gated tests; pure modules never import it.

Session: S7 (implemented). Tested against committed
cv2.aruco.generateImageMarker fixtures (finals/tests/fixtures/frames).
"""
from __future__ import annotations

from typing import Callable, List

import cv2

from finals.config import VALID_MARKER_BACKENDS
from finals.errors import ConfigError
from finals.types import FrameStamped, Sighting

#: A marker detector: (frame, drone_id) -> minimal Sightings. The perception
#: loop holds ONE closure from make_marker_detector (per-loop detector state,
#: never module globals — convention 4).
MarkerDetector = Callable[[FrameStamped, str], List[Sighting]]

#: Decoded QR payloads are capped at this length in class_name — a hostile/
#: garbage decode must not bloat every CSV row and log line it touches.
_QR_PAYLOAD_MAX = 64


def _frame_shape(image) -> "tuple[int, int]":
    h, w = image.shape[:2]
    return (int(h), int(w))


# ============================================================
# ArUco (primary)
# ============================================================
def detect_aruco(frame: FrameStamped, drone_id: str) -> List[Sighting]:
    """One-shot convenience: builds a detector per call. Hot paths hold the
    closure from make_marker_detector("aruco") instead."""
    return make_marker_detector("aruco")(frame, drone_id)


def _detect_aruco_with(detector: "cv2.aruco.ArucoDetector",
                       frame: FrameStamped, drone_id: str) -> List[Sighting]:
    gray = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY)
    # THREE return values — the official example's syntax error dropped
    # `rejected` and would not even parse (see module docstring).
    corners, ids, _rejected = detector.detectMarkers(gray)
    if ids is None:
        return []
    shape = _frame_shape(frame.image)
    sightings: List[Sighting] = []
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        pts = marker_corners.reshape(4, 2)
        mid = int(marker_id)                     # numpy scalar -> int
        sightings.append(Sighting(
            drone_id=drone_id,
            ts=frame.ts,
            source="aruco",
            class_name=f"aruco_{mid}",
            marker_id=mid,
            bbox_xyxy=(float(pts[:, 0].min()), float(pts[:, 1].min()),
                       float(pts[:, 0].max()), float(pts[:, 1].max())),
            confidence=1.0,
            frame_shape=shape,
            frame_number=frame.frame_number,
        ))
    return sightings


# ============================================================
# QR (alternate)
# ============================================================
def detect_qr(frame: FrameStamped, drone_id: str) -> List[Sighting]:
    """One-shot convenience — hot paths hold make_marker_detector("qr")."""
    return make_marker_detector("qr")(frame, drone_id)


def _detect_qr_with(detector: "cv2.QRCodeDetector",
                    frame: FrameStamped, drone_id: str) -> List[Sighting]:
    ok, payloads, points, _codes = detector.detectAndDecodeMulti(frame.image)
    if not ok or points is None:
        return []
    shape = _frame_shape(frame.image)
    sightings: List[Sighting] = []
    for payload, quad in zip(payloads, points):
        if not payload:
            continue        # detected but NOT decoded: an unread marker
                            # scores nothing — emit only actual reads
        clean = payload.replace("\r", " ").replace("\n", " ").strip()
        clean = clean[:_QR_PAYLOAD_MAX]
        if not clean:
            continue        # whitespace-only decode: nothing to report
        pts = quad.reshape(4, 2)
        sightings.append(Sighting(
            drone_id=drone_id,
            ts=frame.ts,
            source="qr",
            class_name=f"qr_{clean}",
            # ASCII digits ONLY: str.isdigit() is true for characters int()
            # REFUSES ("²".isdigit() -> True, int("²") -> ValueError — one
            # hostile/garbled sticker would kill the perception task), and
            # non-ASCII decimals ("٣") would silently alias marker ids.
            marker_id=(int(clean) if clean.isascii() and clean.isdigit()
                       else None),
            bbox_xyxy=(float(pts[:, 0].min()), float(pts[:, 1].min()),
                       float(pts[:, 0].max()), float(pts[:, 1].max())),
            confidence=1.0,
            frame_shape=shape,
            frame_number=frame.frame_number,
        ))
    return sightings


# ============================================================
# The pluggable seam
# ============================================================
def make_marker_detector(backend: str) -> MarkerDetector:
    """marker_backend name -> a detector closure holding ONE cv2 detector
    instance (built once, reused every frame). ConfigError on unknown names
    — same loud-loader convention as the other backend resolvers."""
    if backend == "aruco":
        detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250),
            cv2.aruco.DetectorParameters())

        def _aruco(frame: FrameStamped, drone_id: str) -> List[Sighting]:
            return _detect_aruco_with(detector, frame, drone_id)
        return _aruco
    if backend == "qr":
        detector = cv2.QRCodeDetector()

        def _qr(frame: FrameStamped, drone_id: str) -> List[Sighting]:
            return _detect_qr_with(detector, frame, drone_id)
        return _qr
    raise ConfigError(
        f"unknown marker_backend {backend!r} — one of "
        f"{VALID_MARKER_BACKENDS} (finals/configs/*.json marker_backend)")
