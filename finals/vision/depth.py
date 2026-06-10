"""DepthSource — the OPTIONAL aligned-depth seam (SENSE-IR), mirroring
VideoSource, plus a dependency-free FakeDepthSource.

WHY optional / degrade-absent (binding): the HULA swarm path is MONOCULAR
(720p RGB, 71 deg FOV, NO depth camera — see finals/docs/field_markers.md).
The RealSense depth stack in docs/finals/example_code/getDepth.py is the
MAPPING drone's onboard hardware, NOT this swarm. So depth must be a seam that
the mission NEVER requires: `depth_backend: "none"` (the default) wires NO
DepthSource at all and the perception pipeline is byte-for-byte unchanged. A
future real RealSense backend fills FrameStamped.depth (the Step-0 optional
field on finals/types.py) with an HxW float32 metres array aligned to `image`.

Contract (mirrors finals/vision/video.py VideoSource — same lifecycle so the
wiring/teardown is identical):
- start(timeout_s): begin streaming; raises finals.errors.SensorTimeout if no
  first depth frame arrives in time. Never spins silently.
- stop(): release resources; idempotent; NEVER raises (logs instead).
- read(): the LATEST depth frame as a DepthFrame (a copy) or None if nothing
  yet. Non-blocking. (Named read(), not get_frame(): a depth frame is not a
  FrameStamped — it carries the float32 metres map + the registration the
  RGB source's image does not.)
- healthy: False when depth goes stale (> stale_s) or the stream errors — the
  same VideoWatchdog-style staleness the orchestrator already polls for RGB.
- source_id: the drone (or rig) id depth is attributed to.
- distance_at(cx, cy): the metres reading at one pixel — the
  `depth_frame.get_distance(cx, cy)` primitive the mission would servo on; out
  of bounds / a 0 (no-return) reading -> None, never a fabricated range.

cv2 / pyrealsense2 NOTE (deliberate): this module is PURE (NOT in
tests/test_conventions.py SDK_ALLOWED) — main.py resolves every backend for
--dry-run on SDK-less machines, and the bare suite stays green WITHOUT numpy.
So numpy is type-only (TYPE_CHECKING) and any RealSense/cv2 contact in a real
backend MUST be a method-local lazy import (the ReplaySource pattern). The real
RealSense backend is NOT in scope here — its API is documented inline as a
reference only (rs.pipeline / rs.align / depth_frame.get_distance), never
imported.

Session: SENSE-IR.
"""
from __future__ import annotations

import math
import sys
import threading
import time
from abc import ABC, abstractmethod
from typing import (TYPE_CHECKING, Any, Callable, List, Optional, Sequence,
                    Tuple)

from finals.errors import SensorTimeout

if TYPE_CHECKING:  # numpy is type-only — keeps this module stdlib-only (the
    import numpy as np    # bare dev venv has no numpy; the suite must pass)  # noqa: F401


class DepthFrame:
    """One aligned depth reading. `data_m` is the per-pixel metres map (HxW,
    float32 in a real backend); `distance_at` reads one pixel.

    Kept a plain class (not a frozen dataclass) so `data_m` may be a numpy
    array (unhashable, and frozen dataclasses try to be hashable) without
    importing numpy here. Carries its own monotonic `ts` for staleness and the
    `source_id` it belongs to."""

    __slots__ = ("data_m", "ts", "source_id", "width", "height")

    def __init__(self, data_m: "Any", ts: float, source_id: str, *,
                 width: int, height: int):
        self.data_m = data_m
        self.ts = ts
        self.source_id = source_id
        self.width = int(width)
        self.height = int(height)

    def distance_at(self, cx: int, cy: int) -> Optional[float]:
        """Metres at pixel (cx, cy) — the `depth_frame.get_distance(cx, cy)`
        primitive the mission servos on. Out-of-bounds, a 0 / non-finite / a
        no-return reading -> None (never a fabricated range; a blind 0.0 would
        read as 'obstacle at the lens' and force a needless reaction)."""
        if not (0 <= cx < self.width and 0 <= cy < self.height):
            return None
        try:
            value = float(self.data_m[cy][cx])
        except (IndexError, TypeError, ValueError):
            return None
        if not math.isfinite(value) or value <= 0.0:
            return None
        return value


class DepthSource(ABC):
    """The depth-source seam. Mirror of VideoSource (start/read/healthy/stop)
    so main.py wires and tears it down with the SAME lifecycle as the RGB
    source. Selected by cfg.depth_backend; "none" wires nothing at all."""

    @abstractmethod
    def start(self, timeout_s: float = 10.0) -> None:
        """Begin streaming; raises SensorTimeout if no first depth in time."""

    @abstractmethod
    def stop(self) -> None:
        """Stop streaming and release resources. Never raises — logs failures."""

    @abstractmethod
    def read(self) -> Optional[DepthFrame]:
        """Latest depth frame (copied) or None if nothing yet."""

    @property
    @abstractmethod
    def healthy(self) -> bool:
        """False when depth is stale (> stale_s) or the stream is in error."""

    @property
    @abstractmethod
    def source_id(self) -> str:
        """The drone (or rig) id depth is attributed to."""


class FakeDepthSource(DepthSource):
    """A dependency-free DepthSource: replays a fixed list of depth maps (or a
    single constant map) on a paced background thread. Mirrors ReplaySource's
    thread/staleness/lifecycle, but stdlib-only — a depth map here is just a
    list-of-lists of metres (no numpy needed), so the contract tests run on the
    bare venv.

    - frames: a list of HxW row-major depth maps (each a Sequence[Sequence[
      float]]); or pass a single map for a constant feed. None/empty -> a
      degenerate 1x1 zero map (a source that produces frames but no usable
      range — exercises distance_at's no-return path).
    - The pacing thread sleeps via stop_event.wait(period): REAL time, promptly
      interruptible; the injectable `clock` stamps frames + drives staleness
      (deterministic tests without cross-thread fake-time machinery).
    - loop: wrap the frame list (default True) so a constant feed never goes
      stale; loop=False ends the stream after the last frame (healthy False).
    """

    def __init__(self, source_id: str,
                 frames: "Optional[Sequence[Sequence[Sequence[float]]]]" = None,
                 *, fps: float = 10.0, loop: bool = True,
                 stale_s: float = 2.0,
                 clock: Callable[[], float] = time.monotonic):
        if not isinstance(source_id, str) or not source_id:
            raise ValueError(
                f"FakeDepthSource: source_id must be a non-empty str, got "
                f"{source_id!r} — check the wiring")
        for name, value in (("fps", fps), ("stale_s", stale_s)):
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(
                    f"FakeDepthSource({source_id!r}): {name} must be finite and "
                    f"> 0, got {value!r}")
        self._source_id = source_id
        self._frames = self._normalize(frames)
        self._period_s = 1.0 / float(fps)
        self._loop = bool(loop)
        self._stale_s = float(stale_s)
        self._clock = clock

        self._lock = threading.Lock()
        self._latest: Optional[Tuple[Any, float]] = None   # (map, ts)
        self._stop_event = threading.Event()
        self._first = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._started = False
        self._stopped = False
        self._exhausted = False

    @staticmethod
    def _normalize(frames) -> List[List[List[float]]]:
        """Accept a single HxW map or a list of them; default to one 1x1 zero
        map (frames-but-no-range)."""
        if not frames:
            return [[[0.0]]]
        # A single map is a Sequence[Sequence[number]] (its first element is a
        # row = a sequence of numbers). A list of maps has a Sequence of maps
        # (first element is itself a 2-D map = a sequence of rows).
        first = frames[0]
        is_single_map = (len(first) > 0
                         and isinstance(first[0], (int, float))
                         and not isinstance(first[0], bool))
        maps = [frames] if is_single_map else list(frames)
        out: List[List[List[float]]] = []
        for m in maps:
            out.append([[float(v) for v in row] for row in m])
        return out

    # ---------------- lifecycle ----------------
    def start(self, timeout_s: float = 10.0) -> None:
        if self._started or self._stopped:
            raise RuntimeError(
                f"FakeDepthSource({self._source_id!r}).start() called twice / "
                f"after stop() — one source, one thread; build a fresh instance")
        self._started = True
        self._thread = threading.Thread(
            target=self._run, name=f"depth-{self._source_id}", daemon=False)
        self._thread.start()
        if not self._first.wait(timeout_s):
            self.stop()
            raise SensorTimeout(
                f"{self._source_id}: no first depth frame within {timeout_s:.1f} "
                f"s — the depth thread is stuck; check the depth source")

    def stop(self) -> None:
        """Idempotent; never raises (logs instead)."""
        self._stopped = True
        self._stop_event.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(5.0)
            if t.is_alive():
                print(f"[FakeDepthSource:{self._source_id}] WARNING: pacing "
                      f"thread still alive 5 s after stop()",
                      file=sys.stderr, flush=True)

    # ---------------- the contract ----------------
    def read(self) -> Optional[DepthFrame]:
        with self._lock:
            if self._latest is None:
                return None
            data, ts = self._latest
        # Copy so a consumer can never mutate the source's live map.
        copy = [list(row) for row in data]
        height = len(copy)
        width = len(copy[0]) if height else 0
        return DepthFrame(copy, ts, self._source_id,
                          width=width, height=height)

    @property
    def healthy(self) -> bool:
        if not self._started or self._exhausted:
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
        """loop=False and every frame delivered."""
        return self._exhausted

    # ---------------- the pacing thread ----------------
    def _run(self) -> None:
        idx = 0
        # Bounded (convention 3): the stop event ends the loop; loop=False ends
        # it at the last frame.
        while not self._stop_event.is_set():
            if idx >= len(self._frames):
                if not self._loop:
                    self._exhausted = True
                    return
                idx = 0
            data = self._frames[idx]
            idx += 1
            with self._lock:
                self._latest = (data, self._clock())
            self._first.set()
            if self._stop_event.wait(self._period_s):
                return


# ============================================================
# Real RealSense backend — REFERENCE ONLY (NOT in scope, NOT wired)
# ============================================================
# A real DepthSource over an Intel RealSense (the MAPPING-drone stack, kept
# here only so the seam's intended shape is documented). It would be a new
# `RealSenseDepthSource(DepthSource)` whose start() does, with cv2/pyrealsense2
# imported LAZILY inside the method (this module stays pure):
#
#     import pyrealsense2 as rs              # method-local, gated
#     pipeline = rs.pipeline(); config = rs.config()
#     config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#     profile = pipeline.start(config)
#     align = rs.align(rs.stream.color)      # register depth to the RGB frame
#     depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
#
# and whose pacing thread reads:
#
#     frames = align.process(pipeline.wait_for_frames())
#     depth_frame = frames.get_depth_frame()
#     metres = depth_frame.get_distance(cx, cy)   # the distance_at() primitive
#
# (see docs/finals/example_code/getDepth.py — that is the mapping drone's
# onboard code, audited not copied). It is OUT OF SCOPE for SENSE-IR: the
# swarm path is monocular, so shipping it would add a real SDK dependency for a
# sensor this challenge does not use. VALID_DEPTH_BACKENDS gains "realsense"
# (and main._build_depth grows a branch) the day the swarm actually carries a
# depth camera — not before.
