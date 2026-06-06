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

cv2 NOTE (deliberate, S7): although SDK_ALLOWED permits a top-level cv2
import here, cv2 is imported LAZILY inside ReplaySource — main.py's backend
resolution imports this module for EVERY profile (including --dry-run on
machines without cv2), and the bare test suite must stay green without the
SDK. The `loader` seam exists for the same reason: the ReplaySource contract
tests run dependency-free with an injected fake decoder.

Session: S1 (ABC); S7 (ReplaySource implemented — derives from cv2.imread/
VideoCapture, new and trivial, behind the RgbReceiver-style latest-frame
contract; no upstream bugs to fix, the pacing/exhaustion/staleness semantics
are new here and pinned by tests/test_vision_video.py).
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time
from abc import ABC, abstractmethod
from typing import Any, Callable, List, Optional, Tuple

from finals.errors import SensorError, SensorTimeout
from finals.types import FrameStamped

_FRAME_EXTENSIONS = (".jpg", ".jpeg", ".png")


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
    """Frames from disk — a directory of jpg/jpeg/png (sorted by filename)
    or a video file — paced by a background thread at `fps`. Powers the
    `replay` profile (0 drones, detector-only dev on the laptop), the
    mock-flight-with-frames vision smoke, and the detector tests.

    - The pacing thread sleeps via stop_event.wait(period): REAL time, so
      pacing is promptly interruptible; the injectable `clock` is used only
      for FrameStamped.ts stamps and the staleness math (deterministic
      tests without cross-thread fake-time machinery).
    - frame_number is a MONOTONIC delivery counter, not the file index —
      loop=True wraps must not break perception's seen-this-frame dedupe.
    - loop=False: after the last frame, `exhausted` flips True (and healthy
      False); get_frame() keeps returning the final frame (latest-copy
      contract) so a sampler can still drain it.
    - A decode failure marks the source errored (loud stderr) and ends the
      stream: a broken replay set is a fixture/codec problem to fix, not to
      skip silently.
    """

    def __init__(self, source_id: str, path: str, fps: float = 10.0,
                 loop: bool = True, *,
                 stale_s: float = 2.0,
                 clock: Callable[[], float] = time.monotonic,
                 loader: Optional[Callable[[str], Any]] = None):
        if not isinstance(source_id, str) or not source_id:
            raise ValueError(
                f"ReplaySource: source_id must be a non-empty str, got "
                f"{source_id!r} — check the wiring")
        for name, value in (("fps", fps), ("stale_s", stale_s)):
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(
                    f"ReplaySource({source_id!r}): {name} must be finite and "
                    f"> 0, got {value!r} — check replay_fps in the config")
        self._source_id = source_id
        self._path = os.path.abspath(path)
        self._period_s = 1.0 / float(fps)
        self._loop = bool(loop)
        self._stale_s = float(stale_s)
        self._clock = clock
        self._loader = loader

        self._files: Optional[List[str]] = None      # dir mode
        if os.path.isdir(self._path):
            listing = sorted(os.listdir(self._path))
            self._files = [os.path.join(self._path, f) for f in listing
                           if f.lower().endswith(_FRAME_EXTENSIONS)]
            if not self._files:
                raise SensorError(
                    f"{source_id}: replay dir {self._path!r} contains no "
                    f"{'/'.join(_FRAME_EXTENSIONS)} frames — found "
                    f"{listing[:10] or '(empty)'} — check replay_dir points "
                    f"at the frames (dev fixtures: finals/tests/fixtures/"
                    f"frames)")
        elif not os.path.isfile(self._path):
            raise SensorError(
                f"{source_id}: replay path {self._path!r} is neither a "
                f"directory of frames nor a video file — check replay_dir "
                f"in the config (CWD: {os.getcwd()})")

        self._lock = threading.Lock()
        self._latest: Optional[Tuple[Any, float, int]] = None  # img, ts, n
        self._stop_event = threading.Event()
        self._first_or_error = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._started = False
        self._stopped = False
        self._errored = False
        self._error_msg: Optional[str] = None
        self._exhausted = False
        self._delivered = 0

    # ---------------- lifecycle ----------------
    def start(self, timeout_s: float = 10.0) -> None:
        if self._started or self._stopped:
            raise RuntimeError(
                f"ReplaySource({self._source_id!r}).start() called twice / "
                f"after stop() — one source, one thread, one stream; build "
                f"a fresh instance")
        self._started = True
        self._thread = threading.Thread(
            target=self._run, name=f"replay-{self._source_id}", daemon=False)
        self._thread.start()
        if not self._first_or_error.wait(timeout_s):
            self.stop()
            raise SensorTimeout(
                f"{self._source_id}: no first frame from {self._path!r} "
                f"within {timeout_s:.1f} s — the decode thread is stuck; "
                f"check the replay dir/codec and disk health")
        if self._errored:
            self.stop()
            raise SensorTimeout(
                f"{self._source_id}: replay stream errored before the first "
                f"frame: {self._error_msg} — check the replay files")

    def stop(self) -> None:
        """Idempotent; never raises (logs instead)."""
        self._stopped = True
        self._stop_event.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(5.0)
            if t.is_alive():
                print(f"[ReplaySource:{self._source_id}] WARNING: pacing "
                      f"thread still alive 5 s after stop() — a stuck "
                      f"decoder; the thread is non-daemon and WILL block "
                      f"process exit until it returns — check the file/disk",
                      file=sys.stderr, flush=True)

    # ---------------- the frame contract ----------------
    def get_frame(self) -> Optional[FrameStamped]:
        with self._lock:
            if self._latest is None:
                return None
            img, ts, n = self._latest
        return FrameStamped(image=img.copy(), ts=ts, frame_number=n,
                            source_id=self._source_id)

    @property
    def healthy(self) -> bool:
        if not self._started or self._errored or self._exhausted:
            return False
        with self._lock:
            latest = self._latest
        if latest is None:
            return False
        return (self._clock() - latest[1]) <= self._stale_s

    @property
    def source_id(self) -> str:
        return self._source_id

    @property
    def exhausted(self) -> bool:
        """loop=False and every frame delivered (replay-runner exit signal)."""
        return self._exhausted

    @property
    def errored(self) -> bool:
        """The stream died on a decode/I/O error (already screamed to
        stderr) — the replay runner exits 1 on this instead of burning the
        whole mission budget sampling a dead source."""
        return self._errored

    @property
    def delivered_count(self) -> int:
        with self._lock:
            return self._delivered

    # ---------------- the pacing thread ----------------
    def _fail(self, msg: str) -> None:
        self._error_msg = msg
        self._errored = True
        print(f"[ReplaySource:{self._source_id}] ERROR: {msg}",
              file=sys.stderr, flush=True)
        self._first_or_error.set()

    def _publish(self, img: Any) -> None:
        shape = getattr(img, "shape", None)
        if shape is None or len(shape) != 3 or shape[2] != 3:
            self._fail(
                f"decoded frame has shape {shape!r} — expected HxWx3 BGR "
                f"(a non-color or corrupt file in {self._path!r}?)")
            return
        with self._lock:
            self._delivered += 1
            self._latest = (img, self._clock(), self._delivered)
        self._first_or_error.set()

    def _run(self) -> None:
        # Decode failures and I/O errors are typed-caught and end the stream
        # loudly. An UNEXPECTED exception from an injected loader propagates
        # to threading's default excepthook (traceback on stderr) and the
        # source decays to healthy=False via staleness — loud either way.
        if self._files is not None:
            self._run_dir()
        else:
            self._run_video()

    def _run_dir(self) -> None:
        idx = 0
        # Bounded (convention 3): the stop event ends the loop; loop=False
        # additionally ends it at the last file.
        while not self._stop_event.is_set():
            if idx >= len(self._files):
                if not self._loop:
                    self._exhausted = True
                    return
                idx = 0
            path = self._files[idx]
            idx += 1
            try:
                img = self._load(path)
            except (OSError, ValueError) as e:
                self._fail(f"loader raised on {path!r}: "
                           f"{type(e).__name__}: {e}")
                return
            if img is None:
                self._fail(f"could not decode {path!r} (loader returned "
                           f"None) — corrupt/unsupported image")
                return
            self._publish(img)
            if self._errored:
                return
            if self._stop_event.wait(self._period_s):
                return

    def _load(self, path: str) -> Any:
        if self._loader is None:
            import cv2   # lazy: see the module-docstring cv2 NOTE
            self._loader = cv2.imread   # IMREAD_COLOR default: BGR uint8 HxWx3
        return self._loader(path)

    def _run_video(self) -> None:
        try:
            import cv2   # lazy: video-file mode is cv2-only
        except ImportError as e:
            # Without this, the thread dies via threading.excepthook and
            # start() times out with a misleading "decode thread is stuck".
            self._fail(f"video-file replay needs cv2 and it is not "
                       f"importable ({e}) — pip install opencv-python")
            return
        cap = cv2.VideoCapture(self._path)
        try:
            if not cap.isOpened():
                self._fail(f"cv2.VideoCapture could not open {self._path!r} "
                           f"— unsupported codec/container?")
                return
            delivered_at_rewind = -1     # -1 = no rewind yet
            # Bounded (convention 3): stop event; loop=False ends at EOF;
            # a rewind that recovers nothing fails loudly (no busy-spin).
            while not self._stop_event.is_set():
                ok, img = cap.read()
                if not ok:
                    if self._delivered == 0:
                        # BEFORE the loop check: a zero-frame video must
                        # fail loud in both modes, not sit "exhausted" while
                        # start() burns its timeout on a wrong diagnosis.
                        self._fail(f"{self._path!r} opened but yielded no "
                                   f"frames — empty/corrupt video")
                        return
                    if not self._loop:
                        self._exhausted = True
                        return
                    if self._delivered == delivered_at_rewind:
                        self._fail(
                            f"{self._path!r}: rewind delivered no frames "
                            f"(decoder error mid-stream / truncated "
                            f"container?) — stopping instead of spinning")
                        return
                    delivered_at_rewind = self._delivered
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                self._publish(img)
                if self._errored:
                    return
                if self._stop_event.wait(self._period_s):
                    return
        finally:
            cap.release()
