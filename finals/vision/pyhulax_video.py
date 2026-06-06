"""PyhulaxVideoSource — HULA drone video stream behind the VideoSource seam.

Planned surface (S9):
- Wraps an EXISTING DroneAPI's stream (one DroneAPI per drone, shared between
  the flight adapter and this source — created in main.py wiring):
  create_video_stream() -> set_video_stream(True) -> stream.start() ->
  stream.latest_frame.to_rgb() (hula_connection.py:33-37, 58-62).
- Handles the verified gotchas: latest_frame is None during the ~1-2 s startup
  window; stream.state == ERROR + last_error means dead stream with NO
  auto-reconnect -> bounded stop()/start() restart attempts, then
  healthy=False; channel order normalized to BGR per the video_channel_order
  config flag (what .to_rgb() ACTUALLY returns is bench-verified — open item).
- FakeVideoStream test double (None -> frames -> ERROR sequences) so the
  restart logic is unit-testable without pyhulax.

Derives from: hula_connection.py + pyhulax video docs
https://pyhulax.xenops.ae/reference/video/ (StreamState, fps, last_error).

STUB — session S9.
"""
from __future__ import annotations

_STUB = "finals.vision.pyhulax_video: session S9 — see finals/docs/module_map.md"


class PyhulaxVideoSource:  # implements finals.vision.video.VideoSource in S9
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)
