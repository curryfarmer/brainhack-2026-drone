"""PerceptionLoop — per-drone sampler: video -> YOLO + ArUco -> bus + log.

Planned surface (S7):
- One async loop per drone with a VideoSource, rate-adjusted per agent state
  (search vs landing vs idle); ArUco is the PRIMARY detector and is cheap, so
  sample as fast as the stream delivers (~10 Hz target; the qualifier ran
  5 Hz, qualifier_run.py:236-252) — high rates matter for reading markers on
  MOVING robots:
    frame = source.get_frame(); if None -> count + warn-once; continue
    detect_aruco(frame) -> bus.publish(...)          # synchronous, ~ms, EVERY frame
    detector.submit_image(frame.image, context=...)  # OPTIONAL YOLO via worker pool, config-gated
- The detector callback (fires on a WORKER thread) maps class names via
  cfg.detector.class_map, builds Sighting(source="yolo") with bearing_deg
  from yaw + bbox center + camera_hfov_deg (bearing-only — no depth, no
  ground-projection module by design), then bus.publish() + log.append().
- Detector callbacks NEVER touch agents/FSM directly — the orchestrator polls
  the SightingBus each tick (thread -> asyncio handoff happens only there).
- Health: source.healthy False for > 5 s -> loud log every 5 s; adaptive
  degrade if the detector queue backs up (drop to 1 Hz, logged, never silent).

Derives from: detection_loop + make_detection_callback
(qualifier_run.py:192-252, proven in sim).

STUB — session S7.
"""
from __future__ import annotations

_STUB = "finals.vision.perception: session S7 — see finals/docs/module_map.md"


class PerceptionLoop:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)
