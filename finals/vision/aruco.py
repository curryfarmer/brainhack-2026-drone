"""ArUco DICT_6X6_250 detection — pure function, frame in, Sightings out.

THE PRIMARY DETECTOR (user-confirmed 2026-06-06): the convoy robots carry
ArUco markers, and the mission is to detect and READ them. detectMarkers
returns the marker ID directly — no training, no weights, no sim-to-real gap.
YOLO (finals/vision/detector.py) is the optional, config-gated extra.

Planned surface (S7):
- detect_aruco(frame: FrameStamped, drone_id: str) -> list[Sighting]:
  grayscale -> cv2.aruco.ArucoDetector(DICT_6X6_250).detectMarkers ->
  Sighting(source="aruco", class_name=f"aruco_{id}", marker_id=id, bbox from
  corner extremes, confidence=1.0).
- Cheap (~2-5 ms/frame) and deterministic -> runs SYNCHRONOUSLY inside each
  drone's perception loop on EVERY sampled frame (YOLO goes through the
  worker pool; ArUco does not need to). Cost permits high sample rates on
  all 3 streams — important for MOVING targets, where motion blur and short
  sight windows are the enemies of marker reads (the sentry-scan's sharp
  hover frames help here too).
- Marker-ID semantics (which IDs = convoy robots, which = landing pads) come
  from config, never hardcoded — briefing-day edits stay config edits.
- NO depth: the laptop video stream has none. The RealSense depth/deprojection
  half of the example is intentionally dropped (mapping-challenge material).

Derives from: docs/finals/example_code/potential_detection_targets.py lines
5-30 — NOTE that file is pseudocode with a syntax error
(`corners, ids,  = detector.detectMarkers(...)` — detectMarkers returns
THREE values: corners, ids, rejected). Audited, not copied.

STUB — session S7. Tested with cv2.aruco.generateImageMarker synthetic frames.
"""
from __future__ import annotations

_STUB = "finals.vision.aruco: session S7 — see finals/docs/module_map.md"


def detect_aruco(*args, **kwargs):
    raise NotImplementedError(_STUB)
