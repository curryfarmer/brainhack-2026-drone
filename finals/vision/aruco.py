"""ArUco DICT_6X6_250 detection — pure function, frame in, Sightings out.

Planned surface (S7):
- detect_aruco(frame: FrameStamped, drone_id: str) -> list[Sighting]:
  grayscale -> cv2.aruco.ArucoDetector(DICT_6X6_250).detectMarkers ->
  Sighting(source="aruco", class_name=f"aruco_{id}", marker_id=id, bbox from
  corner extremes, confidence=1.0).
- Cheap (~2-5 ms/frame) and deterministic -> runs SYNCHRONOUSLY inside each
  drone's perception loop on every sampled frame (YOLO goes through the
  worker pool; ArUco does not need to).
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
