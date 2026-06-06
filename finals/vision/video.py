"""VideoSource — the frame-source seam, plus the disk-replay implementation.

Contract derives from the latest-frame-copy pattern proven by RgbReceiver in
qualifier_run.py:163-186 (thread-safe latest frame, None until first frame),
generalized over three backends:
- PyhulaxVideoSource (S9, pyhulax_video.py): stream.latest_frame.to_rgb()
- GazeboRgbSource    (S8, gazebo_video.py):  gz-transport subscriber (SITL VM)
- ReplaySource       (S7, below):            frames from disk (laptop-only dev)

Contract notes (binding):
- get_frame() is non-blocking and returns the LATEST frame (a copy) or None.
- Frames are normalized to BGR uint8 HxWx3 by the source (cv2 convention; the
  pyhulax .to_rgb() channel order is bench-verified via the
  video_channel_order config flag — see plan open questions).
- start() raises finals.errors.SensorTimeout if no first frame arrives within
  timeout_s; never spins silently.
- healthy turns False when frames go stale (> 2 s) or the stream errors —
  the orchestrator's VideoWatchdog polls it. Video loss alone never lands a
  drone (blind flight is safe; blind mission isn't).

Session: S1 (ABC implemented; ReplaySource stub — session S7).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional

from finals.types import FrameStamped


class VideoSource(ABC):
    @abstractmethod
    def start(self, timeout_s: float = 10.0) -> None:
        """Begin streaming; raises SensorTimeout if no first frame in time."""

    @abstractmethod
    def stop(self) -> None:
        """Stop streaming and release resources. Never raises — logs failures."""

    @abstractmethod
    def get_frame(self) -> Optional[FrameStamped]:
        """Latest frame (copied, BGR-normalized) or None if nothing yet."""

    @property
    @abstractmethod
    def healthy(self) -> bool:
        """False when frames are stale (> 2 s) or the stream is in error."""

    @property
    @abstractmethod
    def source_id(self) -> str:
        """The drone id (or replay id) frames are attributed to."""


class ReplaySource(VideoSource):
    """Frames from disk — a directory of jpg/png (sorted) or a video file,
    paced by a background thread at the configured fps. Powers the `replay`
    profile (0 drones, detector-only dev on the laptop) and detector tests.

    STUB — session S7. Derives from: cv2.imread/VideoCapture (new, trivial)
    behind the RgbReceiver-style latest-frame contract.
    """

    _STUB = "finals.vision.video.ReplaySource: session S7 — see finals/docs/module_map.md"

    def __init__(self, source_id: str, path: str, fps: float = 10.0, loop: bool = True):
        raise NotImplementedError(self._STUB)

    def start(self, timeout_s: float = 10.0) -> None:
        raise NotImplementedError(self._STUB)

    def stop(self) -> None:
        raise NotImplementedError(self._STUB)

    def get_frame(self) -> Optional[FrameStamped]:
        raise NotImplementedError(self._STUB)

    @property
    def healthy(self) -> bool:
        raise NotImplementedError(self._STUB)

    @property
    def source_id(self) -> str:
        raise NotImplementedError(self._STUB)
