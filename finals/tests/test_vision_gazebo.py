"""finals.vision.gazebo_video — GazeboRgbSource contract tests.

The transport is behind an injectable receiver seam, so the VideoSource contract
(start/SensorTimeout, BGR-normalize, channel order, frame_number, staleness,
stop) is tested with FakeFrameReceiver — NO gz, NO sockets, NO numpy, so the
bare suite stays green on Windows. One extra test exercises the REAL
_TcpFrameReceiver against an in-process loopback TCP server (the actual wire
format), guarded by importorskip("numpy")."""
from __future__ import annotations

import math
import socket
import struct
import threading
import time

import pytest

from finals.errors import SensorError, SensorTimeout
from finals.types import FrameStamped
from finals.vision.gazebo_video import (_HDR_FMT, _LEN_FMT, FakeFrameReceiver,
                                        GazeboRgbSource, _ChannelArray)


class FakeClock:
    """Deterministic monotonic source — tests set .t explicitly."""

    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


# ============================================================
# Constructor validation
# ============================================================
@pytest.mark.parametrize("source_id,kwargs,match", [
    ("", {}, "source_id"),
    ("alpha", {"video_channel_order": "rbg"}, "video_channel_order"),
    ("alpha", {"stale_s": 0}, "stale_s"),
    ("alpha", {"stale_s": -1.0}, "stale_s"),
    ("alpha", {"stale_s": math.nan}, "stale_s"),
    ("alpha", {"host": ""}, "host"),
    ("alpha", {"port": 0}, "port"),
    ("alpha", {"port": 70000}, "port"),
    ("alpha", {"port": True}, "port"),
])
def test_constructor_rejects_bad_args(source_id, kwargs, match):
    with pytest.raises(ValueError, match=match):
        GazeboRgbSource(source_id, **kwargs)


# ============================================================
# start() — SensorTimeout, None-window tolerance, error surfacing
# ============================================================
def test_start_times_out_without_a_frame():
    src = GazeboRgbSource("alpha", receiver=FakeFrameReceiver(never=True),
                          clock=FakeClock())
    with pytest.raises(SensorTimeout, match="no first gz frame"):
        src.start(timeout_s=0.05)


def test_start_tolerates_none_window_then_returns():
    rx = FakeFrameReceiver(none_reads=2)
    src = GazeboRgbSource("alpha", receiver=rx, clock=FakeClock())
    src.start(timeout_s=1.0)             # returns (no raise)
    assert rx.started == 1
    assert src.get_frame() is not None


def test_start_surfaces_receiver_error():
    rx = FakeFrameReceiver(never=True)
    rx.go_error("connect refused")
    src = GazeboRgbSource("alpha", receiver=rx, clock=FakeClock())
    with pytest.raises(SensorTimeout, match="errored before the first frame"):
        src.start(timeout_s=1.0)


def test_double_start_raises():
    src = GazeboRgbSource("alpha", receiver=FakeFrameReceiver(),
                          clock=FakeClock())
    src.start(timeout_s=1.0)
    with pytest.raises(SensorError, match="called twice"):
        src.start(timeout_s=1.0)


def test_start_rejects_bad_timeout():
    src = GazeboRgbSource("alpha", receiver=FakeFrameReceiver(),
                          clock=FakeClock())
    with pytest.raises(ValueError, match="timeout_s"):
        src.start(timeout_s=0)


# ============================================================
# Frame contract — BGR normalization, frame_number, pre-start
# ============================================================
def test_get_frame_and_healthy_none_before_start():
    src = GazeboRgbSource("alpha", receiver=FakeFrameReceiver())
    assert src.get_frame() is None
    assert src.healthy is False


def test_rgb_is_reversed_to_bgr():
    rx = FakeFrameReceiver(channels=("R", "G", "B"))
    src = GazeboRgbSource("alpha", video_channel_order="rgb", receiver=rx,
                          clock=FakeClock())
    src.start(timeout_s=1.0)
    f = src.get_frame()
    assert isinstance(f, FrameStamped)
    assert f.image == _ChannelArray(("B", "G", "R"))
    assert f.source_id == "alpha"


def test_bgr_passes_through_as_a_copy():
    rx = FakeFrameReceiver(channels=("B", "G", "R"))
    src = GazeboRgbSource("alpha", video_channel_order="bgr", receiver=rx,
                          clock=FakeClock())
    src.start(timeout_s=1.0)
    f = src.get_frame()
    assert f.image == _ChannelArray(("B", "G", "R"))
    assert f.image is not rx.get()       # the source returns a COPY


def test_frame_number_tracks_receiver_count():
    rx = FakeFrameReceiver()
    src = GazeboRgbSource("alpha", receiver=rx, clock=FakeClock())
    src.start(timeout_s=1.0)
    assert src.get_frame().frame_number == 1
    rx.push_frame()
    assert src.get_frame().frame_number == 2


# ============================================================
# healthy — staleness + error
# ============================================================
def test_healthy_goes_stale_then_re_arms():
    clk = FakeClock(0.0)
    rx = FakeFrameReceiver()
    src = GazeboRgbSource("alpha", stale_s=2.0, receiver=rx, clock=clk)
    src.start(timeout_s=1.0)             # first frame -> progress stamped at t=0
    assert src.healthy is True
    clk.t = 1.9
    assert src.healthy is True
    clk.t = 2.5
    assert src.healthy is False          # > stale_s with no new frame
    rx.push_frame()                      # a fresh frame re-arms (count advances)
    assert src.healthy is True


def test_healthy_false_when_receiver_errored():
    rx = FakeFrameReceiver()
    src = GazeboRgbSource("alpha", receiver=rx, clock=FakeClock())
    src.start(timeout_s=1.0)
    assert src.healthy is True
    rx.go_error("bridge died")
    assert src.healthy is False


# ============================================================
# stop() — idempotent, never raises
# ============================================================
def test_stop_is_idempotent_and_safes_the_source():
    rx = FakeFrameReceiver()
    src = GazeboRgbSource("alpha", receiver=rx, clock=FakeClock())
    src.start(timeout_s=1.0)
    src.stop()
    src.stop()                           # no raise
    assert rx.stopped >= 1
    assert src.get_frame() is None and src.healthy is False


# ============================================================
# REAL transport — _TcpFrameReceiver over a loopback socket (needs numpy)
# ============================================================
def test_tcp_receiver_reads_framed_rgb_over_loopback():
    np = pytest.importorskip("numpy")
    h, w = 3, 4
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    rgb[..., 0] = 10        # R plane
    rgb[..., 1] = 20        # G plane
    rgb[..., 2] = 30        # B plane
    body = struct.pack(_HDR_FMT, 1, w, h, 3) + rgb.tobytes()
    msg = struct.pack(_LEN_FMT, len(body)) + body

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))          # ephemeral port — never hardcode 5600
    srv.listen(1)
    port = srv.getsockname()[1]

    def serve():
        conn, _ = srv.accept()
        try:
            conn.sendall(msg)
            time.sleep(0.5)             # keep open so the client can read
        finally:
            conn.close()

    th = threading.Thread(target=serve, daemon=True)
    th.start()
    src = GazeboRgbSource("alpha", host="127.0.0.1", port=port,
                          video_channel_order="rgb")
    try:
        src.start(timeout_s=3.0)
        f = src.get_frame()
        assert isinstance(f.image, np.ndarray)
        assert f.image.shape == (h, w, 3)
        # rgb -> bgr: channel 0 now holds the old B (30), channel 2 the old R (10)
        assert int(f.image[0, 0, 0]) == 30 and int(f.image[0, 0, 2]) == 10
        assert f.frame_number == 1
    finally:
        src.stop()
        srv.close()
        th.join(2.0)
