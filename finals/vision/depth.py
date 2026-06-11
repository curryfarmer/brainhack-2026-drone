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
backend MUST be a method-local lazy import (the ReplaySource pattern).
RealSenseDepthSource below IS that real (opt-in) backend — pyrealsense2 is
lazy-imported inside _open(), never at module top — used only by a rig that
physically carries an Intel RealSense (e.g. the props-off flight_monitor's
forward obstacle poll). The default swarm path stays monocular: depth_backend
"none" wires NO DepthSource at all.

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


class RealSenseDepthSource(DepthSource):
    """A real DepthSource over an Intel RealSense (D4xx) — the forward-facing
    USB depth sensor the props-off flight_monitor polls for "object in front of
    me?". OPT-IN ONLY (depth_backend "realsense" / flight_monitor
    --depth-backend realsense): the HULA swarm path is monocular, so the default
    backend stays "none" and the scored mission is byte-for-byte unchanged.

    Lifecycle/threading mirror FakeDepthSource (start/stop/read/healthy +
    pacing thread). pyrealsense2 is imported LAZILY inside _open() so this module
    stays pure (no top-level SDK import → test_conventions + the bare venv stay
    green). The pacing thread reads aligned frames and DOWNSAMPLES the HxW depth
    image to a coarse `grid_h x grid_w` metres map (a few thousand get_distance()
    calls/frame, not ~300k), published as a DepthFrame whose distance_at(cx, cy)
    reads the true metres at that grid cell. depth_frame.get_distance() already
    returns METRES, so no depth_scale arithmetic is needed.

    The SDK contact is isolated in three overridable hooks — _open() / _read_grid
    / _close() — so tests inject a fake pipeline and exercise the pacing /
    staleness / grid logic WITHOUT hardware or pyrealsense2 (the importorskip is
    only for a true on-camera run). See docs/finals/example_code/getDepth.py for
    the audited reference (the mapping drone's onboard code, not copied).
    """

    def __init__(self, source_id: str, *, width: int = 640, height: int = 480,
                 fps: int = 30, grid_w: int = 64, grid_h: int = 48,
                 stale_s: float = 2.0, frame_timeout_ms: int = 2000,
                 clock: Callable[[], float] = time.monotonic):
        if not isinstance(source_id, str) or not source_id:
            raise ValueError(
                f"RealSenseDepthSource: source_id must be a non-empty str, got "
                f"{source_id!r} — check the wiring")
        for name, value in (("width", width), ("height", height), ("fps", fps),
                            ("grid_w", grid_w), ("grid_h", grid_h),
                            ("stale_s", stale_s),
                            ("frame_timeout_ms", frame_timeout_ms)):
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value) or value <= 0):
                raise ValueError(
                    f"RealSenseDepthSource({source_id!r}): {name} must be finite "
                    f"and > 0, got {value!r}")
        if grid_w > width or grid_h > height:
            raise ValueError(
                f"RealSenseDepthSource({source_id!r}): grid {grid_w}x{grid_h} "
                f"must not exceed the sensor frame {width}x{height}")
        self._source_id = source_id
        self._width = int(width)
        self._height = int(height)
        self._fps = int(fps)
        self._grid_w = int(grid_w)
        self._grid_h = int(grid_h)
        self._stale_s = float(stale_s)
        self._frame_timeout_ms = int(frame_timeout_ms)
        self._clock = clock

        self._lock = threading.Lock()
        self._latest: Optional[Tuple[Any, float]] = None   # (grid, ts)
        self._stop_event = threading.Event()
        self._first = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._started = False
        self._stopped = False
        self._error: Optional[str] = None

    # ---------------- lifecycle ----------------
    def start(self, timeout_s: float = 10.0) -> None:
        if self._started or self._stopped:
            raise RuntimeError(
                f"RealSenseDepthSource({self._source_id!r}).start() called twice "
                f"/ after stop() — one source, one thread; build a fresh instance")
        self._started = True
        self._thread = threading.Thread(
            target=self._run, name=f"depth-rs-{self._source_id}", daemon=False)
        self._thread.start()
        if not self._first.wait(timeout_s):
            self.stop()
            detail = self._error or (
                "no frame and no error — is the RealSense connected? check the "
                "USB3 cable, or run with --depth-backend fake")
            raise SensorTimeout(
                f"{self._source_id}: no first RealSense depth frame within "
                f"{timeout_s:.1f} s — {detail}")

    def stop(self) -> None:
        """Idempotent; never raises (logs instead)."""
        self._stopped = True
        self._stop_event.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(6.0)
            if t.is_alive():
                print(f"[RealSenseDepthSource:{self._source_id}] WARNING: pacing "
                      f"thread still alive 6 s after stop()",
                      file=sys.stderr, flush=True)

    # ---------------- the contract ----------------
    def read(self) -> Optional[DepthFrame]:
        with self._lock:
            if self._latest is None:
                return None
            data, ts = self._latest
        copy = [list(row) for row in data]
        height = len(copy)
        width = len(copy[0]) if height else 0
        return DepthFrame(copy, ts, self._source_id, width=width, height=height)

    @property
    def healthy(self) -> bool:
        if not self._started or self._stopped:
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
    def last_error(self) -> Optional[str]:
        """The fatal error that ended the pacing thread, or None — surfaced so
        the caller can report WHY depth went unhealthy instead of guessing."""
        return self._error

    # ---------------- SDK seam (overridden by tests with a fake pipeline) ------
    def _open(self) -> Any:
        """Build + start the RealSense pipeline; return an opaque ctx
        (pipeline, align). pyrealsense2 is lazy-imported here so the module stays
        pure. Raises on failure (recorded by _run as last_error)."""
        import pyrealsense2 as rs   # lazy by design — keeps this module pure
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self._width, self._height,
                             rs.format.z16, self._fps)
        config.enable_stream(rs.stream.color, self._width, self._height,
                             rs.format.bgr8, self._fps)
        pipeline.start(config)
        align = rs.align(rs.stream.color)
        return (pipeline, align)

    def _read_grid(self, ctx: Any) -> "Optional[List[List[float]]]":
        """One coarse metres map (grid_h x grid_w) from the aligned depth frame,
        or None if no depth arrived this cycle. get_distance() returns METRES."""
        pipeline, align = ctx
        frames = pipeline.wait_for_frames(self._frame_timeout_ms)
        depth = align.process(frames).get_depth_frame()
        if not depth:
            return None
        xs = [min(self._width - 1, int((i + 0.5) * self._width / self._grid_w))
              for i in range(self._grid_w)]
        ys = [min(self._height - 1, int((j + 0.5) * self._height / self._grid_h))
              for j in range(self._grid_h)]
        return [[float(depth.get_distance(x, y)) for x in xs] for y in ys]

    def _close(self, ctx: Any) -> None:
        if ctx is None:
            return
        pipeline = ctx[0]
        try:
            pipeline.stop()
        except (RuntimeError, OSError) as e:   # teardown best-effort, never raise
            print(f"[RealSenseDepthSource:{self._source_id}] pipeline.stop(): "
                  f"{type(e).__name__}: {e}", file=sys.stderr, flush=True)

    # ---------------- the pacing thread ----------------
    def _run(self) -> None:
        ctx = None
        try:
            ctx = self._open()
        except (ImportError, RuntimeError, OSError, ValueError) as e:
            # ImportError = pyrealsense2 absent; RuntimeError = no device / busy.
            self._error = f"{type(e).__name__}: {e}"
            return
        try:
            # Bounded (convention 3): the stop event ends the loop.
            while not self._stop_event.is_set():
                try:
                    grid = self._read_grid(ctx)
                except (RuntimeError, OSError) as e:
                    # A persistent frame-wait timeout / cable yank ends the
                    # thread loudly → healthy goes False, last_error explains it.
                    self._error = f"frame read failed: {type(e).__name__}: {e}"
                    return
                if grid is None:
                    if self._stop_event.wait(0.01):
                        return
                    continue
                with self._lock:
                    self._latest = (grid, self._clock())
                self._first.set()
        finally:
            self._close(ctx)
