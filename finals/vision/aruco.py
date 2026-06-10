"""Marker detection — pure functions, frame in, Sightings out.

THE PRIMARY DETECTOR (user-confirmed 2026-06-06): the convoy robots carry
markers, and the mission is to detect and READ them. detectMarkers returns
the marker ID directly — no training, no weights, no sim-to-real gap.
YOLO (finals/vision/detector.py) is the optional, config-gated extra.

Two pluggable paths feed the SAME Sighting stream (the QR-vs-ArUco intel
question — "QR codes, 20x20 cm" — is still unconfirmed, so the seam is a
config knob, marker_backend, not a code edit):
- detect_aruco: cv2.aruco over a CONFIGURABLE dictionary (cfg.marker_dict;
  real-field default DICT_7X7_1000, the 6x6 sim/fixture configs pin
  DICT_6X6_250) + optional whitelisted DetectorParameters overrides;
- detect_qr:    cv2.QRCodeDetector.detectAndDecodeMulti (the alternate).

PAD-DICT (campaign-critical): the ArUco detector USED to hardcode DICT_6X6_250 —
a 6x6 detector reads NOTHING off the real DICT_7X7_1000 beacons
(11/45/51/67/101), so a finals run would have detected nothing on the real field.
The dictionary name is now resolved from config (VALID_MARKER_DICTS, loud
ConfigError on an unknown name) and threaded through make_marker_detector +
every caller; the default stays the config value. The low-contrast gray-on-white
7x7 beacons (field_markers.md) need cv2 DetectorParameters tuning, so
aruco_detector_params is a WHITELIST of safe DetectorParameters field names
(VALID_ARUCO_PARAM_KEYS; loud ConfigError on an unknown key) applied over the
library defaults — onsite gate F calibrates the actual values on real beacons.

Design (binding):
- Cheap (~2-5 ms/frame) and deterministic -> runs SYNCHRONOUSLY inside each
  drone's perception loop on EVERY sampled frame (YOLO goes through the
  worker pool; marker detection does not need to). Cost permits high sample
  rates on all 3 streams — important for MOVING targets, where motion blur
  and short sight windows are the enemies of marker reads.
- Detectors emit MINIMAL Sightings (ts/source/class_name/marker_id/bbox/
  confidence/frame_shape/frame_number); yaw/alt/bearing enrichment is the
  perception loop's job (dataclasses.replace — Sighting is frozen).
- frame_path is the ONE exception (S11, config-gated by save_marker_frames):
  the ArUco detector has the frame AND the corners in hand, so when main wires
  a save_dir it draws cv2.aruco.drawDetectedMarkers + imwrites an annotated
  JPEG and stamps Sighting.frame_path — mirroring the YOLO path, where the
  DETECTOR (not perception) computes the saved path. Off by default: no
  save_dir -> frame_path None, perception stays pure (no cv2).
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

import os
import sys
from typing import Any, Callable, Dict, List, Optional

import cv2

from finals.config import (DEFAULT_MARKER_DICT, VALID_ARUCO_PARAM_KEYS,
                           VALID_MARKER_BACKENDS, VALID_MARKER_DICTS)
from finals.errors import ConfigError
from finals.types import FrameStamped, Sighting

# Import-time guards: the PURE name tuples live in config.py (which never
# imports cv2); here, where cv2 IS present, prove every listed name actually
# resolves on this cv2 build. A name that config promises but cv2 lacks would
# otherwise surface as an AttributeError deep in the detector thread — catch it
# the instant a cv2-bearing machine imports this module. RAISED (not assert) so
# `python -O` cannot strip the invariant.
_missing_dicts = [n for n in VALID_MARKER_DICTS if not hasattr(cv2.aruco, n)]
if _missing_dicts:
    raise ConfigError(
        f"finals.vision.aruco: VALID_MARKER_DICTS names not on this cv2 "
        f"({cv2.__version__}): {_missing_dicts} — fix config.VALID_MARKER_DICTS")
_missing_params = [n for n in VALID_ARUCO_PARAM_KEYS
                   if not hasattr(cv2.aruco.DetectorParameters(), n)]
if _missing_params:
    raise ConfigError(
        f"finals.vision.aruco: VALID_ARUCO_PARAM_KEYS not on this cv2's "
        f"DetectorParameters: {_missing_params} — fix config.VALID_ARUCO_PARAM_KEYS")

#: A marker detector: (frame, drone_id) -> minimal Sightings. The perception
#: loop holds ONE closure from make_marker_detector (per-loop detector state,
#: never module globals — convention 4).
MarkerDetector = Callable[[FrameStamped, str], List[Sighting]]

#: Decoded QR payloads are capped at this length in class_name — a hostile/
#: garbage decode must not bloat every CSV row and log line it touches.
_QR_PAYLOAD_MAX = 64


def _resolve_marker_dict(marker_dict: str) -> int:
    """marker_dict NAME (e.g. 'DICT_7X7_1000') -> the cv2.aruco.DICT_* int
    constant. Loud ConfigError on an unknown name, listing the valid set — the
    same loud-loader convention as make_marker_detector's backend resolver. The
    membership is ALSO enforced at config load (config._validate); this is the
    detector-side backstop so detect_aruco / a hand-built call never silently
    falls through to a wrong (or no) dictionary."""
    if marker_dict not in VALID_MARKER_DICTS:
        raise ConfigError(
            f"unknown marker_dict {marker_dict!r} — one of "
            f"{list(VALID_MARKER_DICTS)} (the real field is 'DICT_7X7_1000'; "
            f"the 6x6 sim/fixture configs pin 'DICT_6X6_250'). A 6x6 detector "
            f"reads NOTHING off 7x7 markers (finals/configs/*.json marker_dict)")
    return getattr(cv2.aruco, marker_dict)


def _resolve_detector_params(
        params: Optional[Dict[str, Any]]) -> "cv2.aruco.DetectorParameters":
    """Build a cv2.aruco.DetectorParameters, applying any WHITELISTED overrides
    over the library defaults. None -> library defaults. Each key MUST be in
    VALID_ARUCO_PARAM_KEYS (loud ConfigError otherwise — a typo'd field name is a
    silent no-op that would leave the detector mis-tuned on the faint 7x7
    beacons). cv2 owns the value-type contract: an int field given a string
    raises cv2.error, surfaced here as a ConfigError naming the key. config
    validation already gates the KEYS purely; this is the cv2-side application."""
    dp = cv2.aruco.DetectorParameters()
    if not params:
        return dp
    for key, value in params.items():
        if key not in VALID_ARUCO_PARAM_KEYS:
            raise ConfigError(
                f"aruco_detector_params: unknown key {key!r} — not a "
                f"whitelisted cv2.aruco.DetectorParameters field. Valid keys: "
                f"{sorted(VALID_ARUCO_PARAM_KEYS)}")
        try:
            setattr(dp, key, value)
        except (cv2.error, TypeError, OverflowError) as e:
            raise ConfigError(
                f"aruco_detector_params[{key!r}] = {value!r} rejected by "
                f"cv2.aruco.DetectorParameters — check the value type/range "
                f"(e.g. an int field needs an int): {e}") from e
    return dp


def _frame_shape(image) -> "tuple[int, int]":
    h, w = image.shape[:2]
    return (int(h), int(w))


# ============================================================
# ArUco (primary)
# ============================================================
def detect_aruco(frame: FrameStamped, drone_id: str, *,
                 marker_dict: str = DEFAULT_MARKER_DICT,
                 aruco_detector_params: Optional[Dict[str, Any]] = None
                 ) -> List[Sighting]:
    """One-shot convenience: builds a detector per call. Hot paths hold the
    closure from make_marker_detector("aruco") instead. Default dictionary is
    the real-field DICT_7X7_1000 (mirrors config); callers reading the 6x6
    sim/fixture assets pass marker_dict='DICT_6X6_250'."""
    return make_marker_detector(
        "aruco", marker_dict=marker_dict,
        aruco_detector_params=aruco_detector_params)(frame, drone_id)


def _save_marker_frame(frame: FrameStamped, corners, ids, drone_id: str,
                       save_dir: str) -> Optional[str]:
    """Draw the detected markers on a copy of the frame and write it as an
    annotated JPEG; return the path, or None on any write failure. Frame-save
    is best-effort OBSERVABILITY: a failed write NEVER kills a detection — the
    Sightings still flow, just with frame_path None. Mirrors
    finals/vision/detector.py::_save_annotated (the YOLO save path)."""
    annotated = frame.image.copy()
    cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
    # frame_number + ts make the name unique without a counter (no module
    # global — convention 4); frame_number may be None (some sources).
    path = os.path.join(
        save_dir,
        f"aruco_{drone_id}_{frame.frame_number}_{int(frame.ts * 1000)}.jpg")
    try:
        ok = cv2.imwrite(path, annotated)
    except cv2.error as e:
        print(f"[aruco:{drone_id}] WARNING: imwrite({path!r}) raised: {e} — "
              f"annotated frame not saved; detection unaffected",
              file=sys.stderr, flush=True)
        return None
    if not ok:
        print(f"[aruco:{drone_id}] WARNING: imwrite({path!r}) returned False — "
              f"annotated frame not saved; check disk space / the .jpg "
              f"extension; detection unaffected", file=sys.stderr, flush=True)
        return None
    return path


def _detect_aruco_with(detector: "cv2.aruco.ArucoDetector",
                       frame: FrameStamped, drone_id: str,
                       save_dir: Optional[str] = None) -> List[Sighting]:
    gray = cv2.cvtColor(frame.image, cv2.COLOR_BGR2GRAY)
    # THREE return values — the official example's syntax error dropped
    # `rejected` and would not even parse (see module docstring).
    corners, ids, _rejected = detector.detectMarkers(gray)
    if ids is None:
        return []
    shape = _frame_shape(frame.image)
    # One annotated frame per detection pass (drawDetectedMarkers draws ALL
    # markers); every Sighting on this frame shares the path. save_dir None
    # (default) -> frame_path None, the minimal-Sighting behavior.
    saved_path = (_save_marker_frame(frame, corners, ids, drone_id, save_dir)
                  if save_dir is not None else None)
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
            frame_path=saved_path,
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
def make_marker_detector(backend: str, *,
                         marker_dict: str = DEFAULT_MARKER_DICT,
                         aruco_detector_params: Optional[Dict[str, Any]] = None,
                         save_dir: Optional[str] = None) -> MarkerDetector:
    """marker_backend name -> a detector closure holding ONE cv2 detector
    instance (built once, reused every frame). ConfigError on unknown names
    — same loud-loader convention as the other backend resolvers.

    marker_dict (PAD-DICT, ArUco only): the cv2.aruco dictionary NAME
    (VALID_MARKER_DICTS; default DICT_7X7_1000 = the real beacons). The 6x6
    sim/fixture configs pass DICT_6X6_250 so their 6x6 assets still decode. The
    default mirrors config.FinalsConfig.marker_dict — callers pass cfg.marker_dict.
    aruco_detector_params (PAD-DICT, ArUco only): WHITELISTED
    cv2.aruco.DetectorParameters overrides for the faint 7x7 beacons; None =
    library defaults. Both are resolved fail-loud (ConfigError) BEFORE any flight.

    save_dir (S11 save_marker_frames, ArUco only): when set, the ArUco detector
    writes an annotated JPEG per frame-with-markers and stamps frame_path. The
    directory is created HERE, fail-loud, before any flight (mirrors
    DetectorPool). None (default) keeps the minimal-Sighting behavior."""
    if backend == "aruco":
        # Resolve the dict + params FIRST: a bad marker_dict / unknown param key
        # must fail before we touch the filesystem (save_dir) — the loud-on-the-
        # ground contract, no half-built detector + a created dir left behind.
        dictionary = cv2.aruco.getPredefinedDictionary(
            _resolve_marker_dict(marker_dict))
        det_params = _resolve_detector_params(aruco_detector_params)
        if save_dir is not None:
            try:
                os.makedirs(save_dir, exist_ok=True)
            except OSError as e:
                raise ConfigError(
                    f"make_marker_detector('aruco'): cannot create save_dir "
                    f"{save_dir!r} — errno {e.errno} ({e.strerror}) — check "
                    f"the path / permissions (save_marker_frames)") from e
        detector = cv2.aruco.ArucoDetector(dictionary, det_params)

        def _aruco(frame: FrameStamped, drone_id: str) -> List[Sighting]:
            return _detect_aruco_with(detector, frame, drone_id, save_dir)
        return _aruco
    if backend == "qr":
        detector = cv2.QRCodeDetector()

        def _qr(frame: FrameStamped, drone_id: str) -> List[Sighting]:
            return _detect_qr_with(detector, frame, drone_id)
        return _qr
    raise ConfigError(
        f"unknown marker_backend {backend!r} — one of "
        f"{VALID_MARKER_BACKENDS} (finals/configs/*.json marker_backend)")
