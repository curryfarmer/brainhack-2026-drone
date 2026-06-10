"""PyhulaxVideoSource — HULA drone video behind the VideoSource seam + FakeVideoStream.

Wraps an EXISTING pyhulax DroneAPI's video stream (ONE DroneAPI per drone,
SHARED with PyhulaxAdapter — injected here, NOT constructed, so flight and
video speak to the same link). The connect-before-start ordering and the
discovery->ip wiring live in preflight (S10); this module is the leaf that
turns stream.latest_frame into the BGR FrameStamped contract.

Sequence (hula_connection.py:33-37, 58-62, audited): create_video_stream() ->
set_video_stream(True) -> stream.start(); then read stream.latest_frame and
.to_rgb(). Verified gotchas handled:
- latest_frame is None during the ~1-2 s startup window -> start() TOLERATES
  None and polls (bounded by timeout_s), raising SensorTimeout only if no first
  frame ever arrives — never spins silently.
- stream.state == ERROR (StreamState 4) + last_error means a DEAD stream with
  NO auto-reconnect (pyhulax research) -> a bounded stop()/start() restart
  ladder (max_restarts), then healthy stays False (surfaced to the
  VideoWatchdog — video loss never lands a drone, but it must be VISIBLE). The
  counter resets on a fresh frame.
- channel order normalized to BGR per the video_channel_order config flag —
  what .to_rgb() ACTUALLY returns is bench-verified (onsite open item): "rgb"
  (default, matching the method name) reverses to BGR; "bgr" passes through.
  The flag is the seam; the order is never hardcoded.

No background thread here (unlike ReplaySource): pyhulax's VideoStream owns its
own decode thread, so this source just reads the latest frame and runs the
restart ladder lazily from get_frame()/healthy.

SDK imports are METHOD-LOCAL (this module is in tests/test_conventions.py
SDK_ALLOWED) and the whole thing is unit-tested with pyhulax ABSENT via
FakeVideoStream + FakeDroneAPI. This module is NOT on the except-Exception
whitelist (the brief): catches are TYPED — the SDK surface raises pyhulax
errors / OSError / RuntimeError / ValueError, all caught and logged, so
stop()/get_frame()/healthy never raise in practice; a genuinely unexpected
exception type surfaces a real bug instead of being silently swallowed.

Derives from: hula_connection.py + the pyhulax video docs
(https://pyhulax.xenops.ae). Session: S9.
"""
from __future__ import annotations

import math
import sys
import time
from typing import Callable, Optional, Tuple

from finals.errors import SensorError, SensorTimeout
from finals.flight.pyhulax_adapter import _pyhulax_sdk_error_types
from finals.types import FrameStamped
from finals.vision.video import VideoSource

#: StreamState integer values this module reasons about (pyhulax docs:
#: DISCONNECTED 0 ... ERROR 4, STOPPED 5). Only ERROR is special to the source
#: logic; the rest are informational / used by the fake.
_STREAM_STATE_ERROR = 4
_STREAM_STATE_STOPPED = 5
_STREAM_STATE_RUNNING = 3   # any non-ERROR value; the source only special-cases ERROR

_CHANNEL_ORDERS = ("rgb", "bgr")


def _state_int(state) -> Optional[int]:
    """StreamState (real IntEnum or the fake's plain int) -> int, or None."""
    if state is None:
        return None
    val = getattr(state, "value", state)
    try:
        return int(val)
    except (TypeError, ValueError):
        return None


class PyhulaxVideoSource(VideoSource):
    """VideoSource over a shared pyhulax DroneAPI stream. See the module
    docstring for the sequence, the None-window tolerance, the bounded restart
    ladder, and the channel-order seam."""

    def __init__(self, source_id: str, api, *,
                 video_channel_order: str = "rgb",
                 stale_s: float = 2.0,
                 max_restarts: int = 3,
                 restart_stop_timeout_s: float = 2.0,
                 clock: Callable[[], float] = time.monotonic):
        if not isinstance(source_id, str) or not source_id:
            raise ValueError(
                f"PyhulaxVideoSource: source_id must be a non-empty str, got "
                f"{source_id!r} — check the wiring")
        if api is None or not callable(getattr(api, "create_video_stream", None)):
            raise ValueError(
                f"PyhulaxVideoSource({source_id!r}): api must be a pyhulax "
                f"DroneAPI (with create_video_stream) shared with the flight "
                f"adapter, got {api!r} — preflight (S10) injects it")
        if video_channel_order not in _CHANNEL_ORDERS:
            raise ValueError(
                f"PyhulaxVideoSource({source_id!r}): video_channel_order must "
                f"be one of {_CHANNEL_ORDERS}, got {video_channel_order!r} — "
                f"this declares what .to_rgb() really returns (bench-verified)")
        if (not isinstance(stale_s, (int, float)) or isinstance(stale_s, bool)
                or not math.isfinite(stale_s) or stale_s <= 0):
            raise ValueError(
                f"PyhulaxVideoSource({source_id!r}): stale_s must be finite "
                f"and > 0, got {stale_s!r}")
        if (not isinstance(max_restarts, int) or isinstance(max_restarts, bool)
                or max_restarts < 0):
            raise ValueError(
                f"PyhulaxVideoSource({source_id!r}): max_restarts must be an "
                f"int >= 0, got {max_restarts!r}")
        if (not isinstance(restart_stop_timeout_s, (int, float))
                or isinstance(restart_stop_timeout_s, bool)
                or not math.isfinite(restart_stop_timeout_s)
                or restart_stop_timeout_s <= 0):
            raise ValueError(
                f"PyhulaxVideoSource({source_id!r}): restart_stop_timeout_s "
                f"must be finite and > 0, got {restart_stop_timeout_s!r}")

        self._source_id = source_id
        self._api = api
        self._channel_order = video_channel_order
        self._stale_s = float(stale_s)
        self._max_restarts = int(max_restarts)
        self._restart_stop_timeout_s = float(restart_stop_timeout_s)
        self._clock = clock

        self._stream = None
        self._started = False
        self._stopped = False
        self._restarts = 0
        self._last_count: Optional[int] = None
        self._last_progress_ts: Optional[float] = None
        #: typed catch tuple for the SDK surface (empty real-types tuple when
        #: pyhulax is absent — then only OSError/RuntimeError/ValueError apply).
        self._sdk_errors: Tuple[type, ...] = _pyhulax_sdk_error_types()

    # ---------------- helpers ----------------
    def _log(self, msg: str) -> None:
        print(f"[PyhulaxVideoSource] {self._source_id}: {msg}",
              file=sys.stderr, flush=True)

    @property
    def _typed_sdk(self) -> Tuple[type, ...]:
        return (OSError, RuntimeError, ValueError, *self._sdk_errors)

    def _reverse_channels(self, arr):
        """RGB -> BGR (reverse the last axis). Works on a numpy array (returns
        a contiguous copy) and on the fake's observable channel array."""
        rev = arr[:, :, ::-1]
        copy = getattr(rev, "copy", None)
        return copy() if callable(copy) else rev

    # ---------------- lifecycle ----------------
    def start(self, timeout_s: float = 10.0) -> None:
        if self._started or self._stopped:
            raise SensorError(
                f"{self._source_id}: PyhulaxVideoSource.start() called twice / "
                f"after stop() — one source, one stream; build a fresh instance")
        if (not isinstance(timeout_s, (int, float))
                or isinstance(timeout_s, bool)
                or not math.isfinite(timeout_s) or timeout_s <= 0):
            raise ValueError(
                f"{self._source_id}: start() timeout_s must be finite and > 0, "
                f"got {timeout_s!r}")
        self._started = True
        try:
            self._stream = self._api.create_video_stream()
            if self._stream is None:
                raise SensorError(
                    f"{self._source_id}: create_video_stream() returned None — "
                    f"the shared DroneAPI has no stream (connect() first? "
                    f"check the preflight ordering)")
            self._api.set_video_stream(True)
            self._stream.start()
        except self._typed_sdk as e:
            raise SensorError(
                f"{self._source_id}: could not start the video stream "
                f"({type(e).__name__}: {e}) — check the drone link / camera "
                f"and that connect() ran first") from e

        # Tolerate the None startup window; bounded by REAL wall-clock so an
        # injected (possibly frozen) test clock cannot hang this loop.
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if _state_int(getattr(self._stream, "state", None)) \
                    == _STREAM_STATE_ERROR:
                last_error = getattr(self._stream, "last_error", None)
                raise SensorTimeout(
                    f"{self._source_id}: video stream errored before the first "
                    f"frame (last_error={last_error!r}) — check the camera / "
                    f"drone link")
            if self.get_frame() is not None:
                return
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        raise SensorTimeout(
            f"{self._source_id}: no first video frame within {timeout_s:.1f} s "
            f"— stream stuck in the startup None-window; check the camera / "
            f"drone link / decode. FIXES: raise the video timeout; turn the "
            f"WINDOWS FIREWALL OFF for inbound UDP (blocked UDP looks exactly "
            f"like this); POWER-CYCLE the drone to clear a stale bind_client; "
            f"ONE stream per drone, no auto-reconnect (close the HulaGo app / "
            f"any other client)")

    def stop(self) -> None:
        """Idempotent; never raises (logs). Typed catches only (see module
        docstring)."""
        self._stopped = True
        stream = self._stream
        if stream is not None:
            try:
                stream.stop(self._restart_stop_timeout_s)
            except self._typed_sdk as e:
                self._log(f"stop: stream.stop failed "
                          f"({type(e).__name__}: {e})")
        if self._api is not None:
            try:
                self._api.set_video_stream(False)
            except self._typed_sdk as e:
                self._log(f"stop: set_video_stream(False) failed "
                          f"({type(e).__name__}: {e})")

    # ---------------- the frame contract ----------------
    def get_frame(self) -> Optional[FrameStamped]:
        if not self._started or self._stopped or self._stream is None:
            return None
        self._poll()
        return self._build_frame()

    @property
    def healthy(self) -> bool:
        if not self._started or self._stopped or self._stream is None:
            return False
        self._poll()
        if _state_int(getattr(self._stream, "state", None)) \
                == _STREAM_STATE_ERROR:
            return False
        if self._last_progress_ts is None:
            return False
        return (self._clock() - self._last_progress_ts) <= self._stale_s

    @property
    def source_id(self) -> str:
        return self._source_id

    # ---------------- internals ----------------
    def _poll(self) -> None:
        """Read stream state + frame_count: stamp progress (and reset the
        restart counter) on a fresh frame; run the bounded restart ladder on
        ERROR. Typed-caught — a flaky SDK read must not crash the watchdog."""
        stream = self._stream
        try:
            state_int = _state_int(getattr(stream, "state", None))
            frame_count = getattr(stream, "frame_count", None)
        except self._typed_sdk as e:
            self._log(f"poll: reading stream state failed "
                      f"({type(e).__name__}: {e})")
            return
        if state_int == _STREAM_STATE_ERROR:
            self._maybe_restart()
            return
        if frame_count is not None and frame_count != self._last_count:
            self._last_count = frame_count
            self._last_progress_ts = self._clock()
            self._restarts = 0       # a fresh frame clears the restart ladder

    def _maybe_restart(self) -> None:
        """Bounded stop()/start() restart on a dead stream (no auto-reconnect).
        Once max_restarts is spent, healthy stays False until a fresh frame."""
        if self._restarts >= self._max_restarts:
            return
        self._restarts += 1
        last_error = getattr(self._stream, "last_error", None)
        self._log(f"video stream ERROR (last_error={last_error!r}) — restart "
                  f"attempt {self._restarts}/{self._max_restarts}")
        try:
            self._stream.stop(self._restart_stop_timeout_s)
        except self._typed_sdk as e:
            self._log(f"restart: stream.stop failed "
                      f"({type(e).__name__}: {e})")
        try:
            self._api.set_video_stream(True)
            self._stream.start()
        except self._typed_sdk as e:
            self._log(f"restart: stream.start failed "
                      f"({type(e).__name__}: {e})")

    def _build_frame(self) -> Optional[FrameStamped]:
        stream = self._stream
        try:
            frame = getattr(stream, "latest_frame", None)
            if frame is None:
                return None
            rgb = frame.to_rgb()
            image = (self._reverse_channels(rgb)
                     if self._channel_order == "rgb" else rgb)
        except self._typed_sdk as e:
            # A transient decode error: logged (never silent), surfaced as "no
            # frame"; the ERROR-state path drives the restart ladder.
            self._log(f"get_frame: decode failed ({type(e).__name__}: {e})")
            return None
        return FrameStamped(
            image=image, ts=self._clock(),
            frame_number=getattr(stream, "frame_count", None),
            source_id=self._source_id)


# ============================================================
# FakeVideoStream — pyhulax VideoStream double (no pyhulax needed)
# ============================================================
class _ChannelArray:
    """Minimal stand-in for a HxWx3 image whose channel ORDER is observable
    without numpy. Supports the `[:, :, ::-1]` last-axis reverse the source
    uses, plus .shape/.copy() for the BGR-normalized FrameStamped contract."""

    def __init__(self, order):
        self.order: Tuple = tuple(order)

    @property
    def shape(self):
        return (1, 1, len(self.order))

    def copy(self):
        return _ChannelArray(self.order)

    def __getitem__(self, key):
        if (isinstance(key, tuple) and len(key) == 3
                and isinstance(key[2], slice)
                and key[2] == slice(None, None, -1)):
            return _ChannelArray(tuple(reversed(self.order)))
        return self

    def __eq__(self, other):
        return isinstance(other, _ChannelArray) and self.order == other.order

    def __repr__(self):
        return f"_ChannelArray(order={self.order!r})"


class _FakeFrame:
    def __init__(self, channels):
        self._channels = tuple(channels)

    def to_rgb(self):
        return _ChannelArray(self._channels)


class FakeVideoStream:
    """Scriptable pyhulax VideoStream double: None startup window -> frames ->
    ERROR, so the None-window tolerance, channel-order flip, bounded restart
    ladder, and staleness are all unit-tested without pyhulax.

    - channels: the PHYSICAL channel order to_rgb() reports (e.g. ('R','G','B')
      to mimic a stream whose .to_rgb() is genuinely RGB).
    - none_reads: how many latest_frame reads return None before the first
      frame appears (the startup window).
    - go_error(stuck=True): the stream stays ERROR across stop()/start()
      (exhausts the restart ladder); stuck=False lets a restart recover it.
    Counters (started/stopped/frame_count) are assertable in tests."""

    def __init__(self, *, channels=("R", "G", "B"), none_reads: int = 0):
        self.channels = tuple(channels)
        self.state = _STREAM_STATE_RUNNING
        self.frame_count = 0
        self.last_error = None
        self.started = 0
        self.stopped = 0
        self._none_reads = int(none_reads)
        self._frame: Optional[_FakeFrame] = None
        self._stuck = False

    # --- VideoStream surface ---
    def start(self, blocking: bool = False):
        self.started += 1
        if not self._stuck:
            self.state = _STREAM_STATE_RUNNING
            if self._none_reads == 0 and self._frame is None:
                self._produce()

    def stop(self, timeout=None):
        self.stopped += 1
        if not self._stuck:
            self.state = _STREAM_STATE_STOPPED

    @property
    def latest_frame(self):
        if self._none_reads > 0:
            self._none_reads -= 1
            if self._none_reads == 0 and self._frame is None and not self._stuck:
                self._produce()
            return None
        return self._frame

    # --- test controls ---
    def _produce(self):
        self.frame_count += 1
        self._frame = _FakeFrame(self.channels)

    def push_frame(self):
        self._produce()

    def go_error(self, msg="decode error", *, stuck: bool = True):
        self.state = _STREAM_STATE_ERROR
        self.last_error = msg
        self._stuck = stuck

    def recover(self):
        self._stuck = False
        self.last_error = None
        self.state = _STREAM_STATE_RUNNING
