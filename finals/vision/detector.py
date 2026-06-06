"""Laptop YOLO detection — vendored FIXED Detector + CannedDetector mock.

OPTIONAL FALLBACK (user-confirmed 2026-06-06): the convoy robots carry ArUco
markers, so finals/vision/aruco.py is the primary detector and YOLO is OFF by
default in every shipped config (detector.backend "none"). Enable it (one
config edit: backend "ultralytics" + weights) only if the briefing scores
spotting robots whose marker isn't readable — which also reactivates the
retraining question (best.pt is barrel-trained).

Planned surface (S7):
- A vendored copy of root Detector.py (threaded ultralytics worker pool;
  callback(detections, annotated_image, context)) with the verified bugs
  FIXED — the root file stays untouched for the qualifier stack:
  1. the `finally: del results` worker-killer (root Detector.py:143-150 —
     when self.model() raises before `results` binds, the finally block
     raises NameError, which silently kills the worker thread: detection
     stops forever with no crash);
  2. the silent COCO fallback (root Detector.py:28-29 model_path=None ->
     "yolov8n.pt") — here weights are REQUIRED and come validated from
     finals.config (which already rejects placeholder COCO names);
  3. the unbounded submit queue -> bounded, drop-oldest, with a loud counter.
- Callback contract kept IDENTICAL to root Detector.py:135-139 (each det:
  bbox, confidence, class_id, class_name). Note (verified): the root only
  fires the callback when there ARE detections — perception must not treat
  callback silence as "frame processed, nothing found".
- CannedDetector: same DetectorLike surface, driven by a JSON script
  ([{"after_n_submits": 5, "detections": [...]}]) and invoking the callback
  on a WORKER THREAD like the real one, so threading bugs surface in tests.
- One shared detector instance serves all drones; frames carry
  context={"drone_id", "ts", "yaw", "alt"}.

Derives from: root Detector.py (verified line-by-line; bugs listed above).

STUB — session S7.
"""
from __future__ import annotations

_STUB = "finals.vision.detector: session S7 — see finals/docs/module_map.md"


def make_ultralytics_detector(*args, **kwargs):
    raise NotImplementedError(_STUB)


class CannedDetector:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)
