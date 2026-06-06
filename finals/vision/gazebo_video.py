"""GazeboRgbSource — Gazebo camera frames behind the VideoSource seam (SITL VM).

Planned surface (S8):
- Direct port of the PROVEN RgbReceiver from qualifier_run.py:163-186
  (gz-transport subscriber, thread-safe latest frame, reshape guard) behind
  finals.vision.video.VideoSource.
- Lazy `gz.transport13` import INSIDE this module with the existing actionable
  error message ("install Gazebo Harmonic") — a missing SDK fails loudly at
  wiring time but only when this backend is selected.
- Converts to BGR if needed; healthy = frame age < 2 s.

Derives from: qualifier_run.py RgbReceiver (proven in sim).

STUB — session S8.
"""
from __future__ import annotations

_STUB = "finals.vision.gazebo_video: session S8 — see finals/docs/module_map.md"


class GazeboRgbSource:  # implements finals.vision.video.VideoSource in S8
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)
