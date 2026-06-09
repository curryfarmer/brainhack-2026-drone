"""GazeboRgbSource — Gazebo camera frames behind the VideoSource seam (SITL VM).

THE 3.11-VENV CONSTRAINT (the SIM-4 problem, solved here): finals/ runs on a
Python 3.11 venv (asyncio.timeout), but the apt `gz.transport13` Python bindings
are compiled for SYSTEM python 3.10 and will NOT import inside the venv (sim/
README "gz bindings wrinkle"). So this module does NOT `import gz.transport13` —
instead a sidecar, `sim/gz_camera_bridge.py`, runs the PROVEN check_detection.py
gz subscriber under system 3.10 and forwards raw RGB frames over a localhost TCP
socket. This module is the venv-side CLIENT: stdlib socket + numpy only, no gz,
no cv2. (The RTP/H.264 alternative was rejected: PX4 gz Harmonic does not
auto-stream RTP, and the venv's pip opencv is built without GStreamer.)

Contract (finals.vision.video.VideoSource): get_frame() returns the LATEST frame
(a copy), BGR-normalized; start() raises SensorTimeout if no first frame within
timeout_s (never spins silently); healthy goes False when frames go stale (> 2 s)
or the transport errors. NO silent auto-reconnect — a dead bridge/camera stales
out and the VideoWatchdog DEGRADEs detection (a blind drone still flies home).

Structure mirrors finals/vision/pyhulax_video.py: an INJECTABLE `receiver` seam
(pyhulax's injected `api`) so the contract is unit-tested with FakeFrameReceiver
and the module imports on a bare venv (numpy is imported LAZILY inside the real
_TcpFrameReceiver). gz delivers R8G8B8 → with video_channel_order "rgb" (default)
the source reverses the last axis to BGR; "bgr" passes through.

This module is on tests/test_conventions.py SDK_ALLOWED (for the historical
in-process gz plan) but no longer imports an SDK at module scope. It is NOT on
the except-Exception whitelist: every catch here is TYPED (socket/struct/value),
so stop()/get_frame()/healthy never raise on a flaky transport, while a genuinely
unexpected exception type surfaces a real bug.

Derives from: sim/check_detection.py CameraReceiver (the gz latest-frame pattern,
now living in the bridge) + pyhulax_video.py (the VideoSource shape). Session: S8.
"""
from __future__ import annotations

import math
import socket
import struct
import sys
import threading
import time
from typing import Any, Callable, Optional

from finals.errors import SensorError, SensorTimeout
from finals.types import FrameStamped
from finals.vision.video import VideoSource

_CHANNEL_ORDERS = ("rgb", "bgr")

#: Wire framing shared with sim/gz_camera_bridge.py. Each message is a
#: length-prefixed frame, big-endian:
#:   [u32 total_len][u64 frame_no][u32 width][u32 height][u8 channels][raw bytes]
#: total_len counts everything AFTER itself (header + payload). The payload is
#: width*height*channels raw bytes in the gz-native channel order (RGB).
_LEN_FMT = ">I"
_LEN_SIZE = struct.calcsize(_LEN_FMT)          # 4
_HDR_FMT = ">QIIB"
_HDR_SIZE = struct.calcsize(_HDR_FMT)          # 17


class GazeboRgbSource(VideoSource):
    """VideoSource over the gz_camera_bridge TCP feed. See the module docstring
    for the transport, the BGR normalization, and the no-auto-reconnect policy.

    `receiver` is the injectable transport seam (default None -> the real
    _TcpFrameReceiver is built lazily in start()); tests inject FakeFrameReceiver
    so the contract runs without gz/numpy/sockets."""

    def __init__(self, source_id: str, *,
                 host: str = "127.0.0.1",
                 port: int = 5600,
                 video_channel_order: str = "rgb",
                 stale_s: float = 2.0,
                 clock: Callable[[], float] = time.monotonic,
                 receiver: Optional[Any] = None):
        if not isinstance(source_id, str) or not source_id:
            raise ValueError(
                f"GazeboRgbSource: source_id must be a non-empty str, got "
                f"{source_id!r} — check the wiring")
        if video_channel_order not in _CHANNEL_ORDERS:
            raise ValueError(
                f"GazeboRgbSource({source_id!r}): video_channel_order must be "
                f"one of {_CHANNEL_ORDERS}, got {video_channel_order!r} — gz is "
                f"R8G8B8 so the default 'rgb' reverses to BGR")
        if (not isinstance(stale_s, (int, float)) or isinstance(stale_s, bool)
                or not math.isfinite(stale_s) or stale_s <= 0):
            raise ValueError(
                f"GazeboRgbSource({source_id!r}): stale_s must be finite and "
                f"> 0, got {stale_s!r}")
        # host/port only build the real receiver; validate anyway so a bad value
        # dies at construction (the weights-guard philosophy), not on connect.
        if not isinstance(host, str) or not host:
            raise ValueError(
                f"GazeboRgbSource({source_id!r}): host must be a non-empty str, "
                f"got {host!r}")
        if (not isinstance(port, int) or isinstance(port, bool)
                or not 1 <= port <= 65535):
            raise ValueError(
                f"GazeboRgbSource({source_id!r}): port must be an int in "
                f"[1, 65535], got {port!r}")

        self._source_id = source_id
        self._host = host
        self._port = int(port)
        self._channel_order = video_channel_order
        self._stale_s = float(stale_s)
        self._clock = clock

        self._receiver = receiver
        self._started = False
        self._stopped = False
        self._last_count = 0
        self._last_progress_ts: Optional[float] = None

    # ---------------- helpers ----------------
    def _log(self, msg: str) -> None:
        print(f"[GazeboRgbSource] {self._source_id}: {msg}",
              file=sys.stderr, flush=True)

    def _reverse_channels(self, arr):
        """RGB -> BGR (reverse the last axis), returning a contiguous copy.
        Works on a numpy array and on the fake's observable channel array."""
        rev = arr[:, :, ::-1]
        copy = getattr(rev, "copy", None)
        return copy() if callable(copy) else rev

    def _copy(self, arr):
        copy = getattr(arr, "copy", None)
        return copy() if callable(copy) else arr

    # ---------------- lifecycle ----------------
    def start(self, timeout_s: float = 10.0) -> None:
        if self._started or self._stopped:
            raise SensorError(
                f"{self._source_id}: GazeboRgbSource.start() called twice / "
                f"after stop() — one source, one receiver; build a fresh instance")
        if (not isinstance(timeout_s, (int, float)) or isinstance(timeout_s, bool)
                or not math.isfinite(timeout_s) or timeout_s <= 0):
            raise ValueError(
                f"{self._source_id}: start() timeout_s must be finite and > 0, "
                f"got {timeout_s!r}")
        self._started = True
        if self._receiver is None:
            self._receiver = _TcpFrameReceiver(
                self._host, self._port, clock=self._clock)
        try:
            self._receiver.start(timeout_s)
        except (ImportError, OSError, RuntimeError, ValueError) as e:
            raise SensorError(
                f"{self._source_id}: could not start the gz frame receiver "
                f"({type(e).__name__}: {e}) — is sim/gz_camera_bridge running "
                f"on {self._host}:{self._port}?") from e

        # Bound the first-frame wait by REAL wall-clock so an injected (possibly
        # frozen) test clock cannot hang this loop (pyhulax_video.py:180).
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if self._receiver_errored():
                raise SensorTimeout(
                    f"{self._source_id}: gz frame receiver errored before the "
                    f"first frame (last_error="
                    f"{getattr(self._receiver, 'last_error', None)!r}) — check "
                    f"sim/gz_camera_bridge and the gz camera topic")
            if self.get_frame() is not None:
                return
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        raise SensorTimeout(
            f"{self._source_id}: no first gz frame within {timeout_s:.1f} s on "
            f"{self._host}:{self._port} — is sim/gz_camera_bridge streaming? It "
            f"must be up BEFORE finals (the run-script gates on its first frame); "
            f"the camera renders BLANK under ogre2 — use llvmpipe/ogre1")

    def stop(self) -> None:
        """Idempotent; never raises (logs). Typed catches only."""
        self._stopped = True
        r = self._receiver
        if r is not None:
            try:
                r.stop()
            except (OSError, RuntimeError, ValueError) as e:
                self._log(f"stop: receiver.stop failed "
                          f"({type(e).__name__}: {e})")

    # ---------------- the frame contract ----------------
    def get_frame(self) -> Optional[FrameStamped]:
        if not self._started or self._stopped or self._receiver is None:
            return None
        self._poll()
        return self._build_frame()

    @property
    def healthy(self) -> bool:
        if not self._started or self._stopped or self._receiver is None:
            return False
        self._poll()
        if self._receiver_errored():
            return False
        if self._last_progress_ts is None:
            return False
        return (self._clock() - self._last_progress_ts) <= self._stale_s

    @property
    def source_id(self) -> str:
        return self._source_id

    # ---------------- internals ----------------
    def _receiver_errored(self) -> bool:
        try:
            return bool(getattr(self._receiver, "errored", False))
        except (OSError, RuntimeError, ValueError):
            # Can't read the flag — let staleness be the judge, don't crash.
            return False

    def _poll(self) -> None:
        """Stamp a progress timestamp when the receiver's frame COUNT advances
        (the staleness clock the VideoWatchdog reads). count == 0 means no frame
        yet — never stamps (else healthy would be true with nothing received)."""
        try:
            count = self._receiver.count
        except (OSError, RuntimeError, ValueError) as e:
            self._log(f"poll: reading receiver count failed "
                      f"({type(e).__name__}: {e})")
            return
        if count and count != self._last_count:
            self._last_count = count
            self._last_progress_ts = self._clock()

    def _build_frame(self) -> Optional[FrameStamped]:
        try:
            rgb = self._receiver.get()
            if rgb is None:
                return None
            image = (self._reverse_channels(rgb)
                     if self._channel_order == "rgb" else self._copy(rgb))
            count = self._receiver.count
        except (OSError, RuntimeError, ValueError) as e:
            self._log(f"get_frame: building frame failed "
                      f"({type(e).__name__}: {e})")
            return None
        return FrameStamped(
            image=image, ts=self._clock(),
            frame_number=count, source_id=self._source_id)


# ============================================================
# _TcpFrameReceiver — the real transport (TCP client + reader thread)
# ============================================================
class _TcpFrameReceiver:
    """Connects to sim/gz_camera_bridge (TCP server) and reads length-prefixed
    RGB frames on a background thread, keeping only the LATEST (a slow consumer
    must never back-pressure the bridge's gz callback). numpy is imported LAZILY
    here so finals/vision/gazebo_video imports on a venv without numpy."""

    def __init__(self, host: str, port: int, *,
                 clock: Callable[[], float] = time.monotonic,
                 connect_retry_s: float = 0.2,
                 read_timeout_s: float = 0.5):
        self._host = host
        self._port = int(port)
        self._clock = clock
        self._connect_retry_s = float(connect_retry_s)
        self._read_timeout_s = float(read_timeout_s)

        self._lock = threading.Lock()
        self._latest = None
        self._count = 0
        self._errored = False
        self._last_error: Optional[str] = None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._sock: Optional[socket.socket] = None
        self._np = None

    # --- VideoSource-receiver surface (mirrored by FakeFrameReceiver) ---
    def start(self, timeout_s: float) -> None:
        import numpy as np    # lazy: keeps the module importable without numpy
        self._np = np
        deadline = time.monotonic() + float(timeout_s)
        self._thread = threading.Thread(
            target=self._run, args=(deadline,),
            name=f"gz-rx-{self._host}:{self._port}", daemon=True)
        self._thread.start()

    @property
    def count(self) -> int:
        with self._lock:
            return self._count

    @property
    def errored(self) -> bool:
        return self._errored

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    def get(self):
        with self._lock:
            return self._latest

    def stop(self) -> None:
        self._stop_event.set()
        sock = self._sock
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        t = self._thread
        if t is not None and t.is_alive():
            t.join(2.0)

    # --- reader thread ---
    def _fail(self, msg: str) -> None:
        self._last_error = msg
        self._errored = True
        print(f"[_TcpFrameReceiver] {self._host}:{self._port}: {msg}",
              file=sys.stderr, flush=True)

    def _run(self, deadline: float) -> None:
        sock = self._connect(deadline)
        if sock is None:
            return                          # errored or stopped (logged)
        self._sock = sock
        try:
            self._read_loop(sock)
        except (OSError, ConnectionError, struct.error, ValueError) as e:
            if not self._stop_event.is_set():
                # NO auto-reconnect by design: the VideoWatchdog DEGRADEs on the
                # now-stale frame (a blind drone still flies home).
                self._fail(f"frame stream ended ({type(e).__name__}: {e}) — the "
                           f"bridge/gz camera stopped; no auto-reconnect")
        finally:
            try:
                sock.close()
            except OSError:
                pass

    def _connect(self, deadline: float) -> Optional[socket.socket]:
        while not self._stop_event.is_set():
            try:
                sock = socket.create_connection(
                    (self._host, self._port), timeout=1.0)
                sock.settimeout(self._read_timeout_s)
                return sock
            except OSError as e:
                if time.monotonic() >= deadline:
                    self._fail(
                        f"could not connect within the start window "
                        f"({type(e).__name__}: {e}) — is sim/gz_camera_bridge "
                        f"running on {self._host}:{self._port}?")
                    return None
                if self._stop_event.wait(self._connect_retry_s):
                    return None
        return None

    def _read_loop(self, sock: socket.socket) -> None:
        np = self._np
        while not self._stop_event.is_set():
            hdr = self._recv_exactly(sock, _LEN_SIZE)
            (total_len,) = struct.unpack(_LEN_FMT, hdr)
            if total_len < _HDR_SIZE:
                raise ValueError(
                    f"frame length {total_len} < header {_HDR_SIZE} — wire "
                    f"format mismatch with sim/gz_camera_bridge")
            body = self._recv_exactly(sock, total_len)
            frame_no, w, h, ch = struct.unpack(_HDR_FMT, body[:_HDR_SIZE])
            payload = body[_HDR_SIZE:]
            expected = w * h * ch
            if len(payload) != expected:
                raise ValueError(
                    f"frame {frame_no}: payload {len(payload)} != w*h*ch "
                    f"{w}x{h}x{ch}={expected}")
            arr = np.frombuffer(payload, dtype=np.uint8).reshape((h, w, ch))
            with self._lock:
                self._latest = arr          # RGB; the source normalizes to BGR
                self._count += 1

    def _recv_exactly(self, sock: socket.socket, n: int) -> bytes:
        chunks = []
        got = 0
        while got < n:
            if self._stop_event.is_set():
                raise ConnectionError("receiver stopped")
            try:
                chunk = sock.recv(n - got)
            except socket.timeout:
                continue                    # gz between frames — re-check stop
            if not chunk:
                raise ConnectionError("bridge closed the connection")
            chunks.append(chunk)
            got += len(chunk)
        return b"".join(chunks)


# ============================================================
# FakeFrameReceiver — a gz_camera_bridge double (no gz / no numpy / no sockets)
# ============================================================
class _ChannelArray:
    """Minimal HxWx3 stand-in whose channel ORDER is observable without numpy.
    Supports the `[:, :, ::-1]` last-axis reverse the source uses, plus
    .shape/.copy() (mirrors pyhulax_video.py:_ChannelArray)."""

    def __init__(self, order):
        self.order = tuple(order)

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


class FakeFrameReceiver:
    """Scriptable _TcpFrameReceiver double: None startup window -> frames ->
    optional error, so the None-window tolerance, channel-order flip, frame
    counting, staleness, and the error->unhealthy path are unit-tested with no
    gz/numpy/sockets. Mirrors pyhulax_video.py:FakeVideoStream.

    - channels: the channel order get() reports (e.g. ('R','G','B') to mimic the
      gz-native RGB the bridge forwards).
    - none_reads: how many get() calls return None before the first frame.
    - never: get() always returns None and never produces (the SensorTimeout
      path). Counters (started/stopped/count) are assertable."""

    def __init__(self, *, channels=("R", "G", "B"), none_reads: int = 0,
                 never: bool = False):
        self.channels = tuple(channels)
        self._none_reads = int(none_reads)
        self._never = bool(never)
        self._latest = None
        self._count = 0
        self._errored = False
        self._last_error: Optional[str] = None
        self.started = 0
        self.stopped = 0

    # --- receiver surface ---
    def start(self, timeout_s: float) -> None:
        self.started += 1
        if not self._never and self._none_reads == 0 and self._latest is None:
            self._produce()

    @property
    def count(self) -> int:
        return self._count

    @property
    def errored(self) -> bool:
        return self._errored

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    def get(self):
        if self._never:
            return None
        if self._none_reads > 0:
            self._none_reads -= 1
            if self._none_reads == 0 and self._latest is None:
                self._produce()
            return None
        return self._latest

    def stop(self) -> None:
        self.stopped += 1

    # --- test controls ---
    def _produce(self) -> None:
        self._count += 1
        self._latest = _ChannelArray(self.channels)

    def push_frame(self) -> None:
        self._produce()

    def go_error(self, msg: str = "bridge died") -> None:
        self._errored = True
        self._last_error = msg
