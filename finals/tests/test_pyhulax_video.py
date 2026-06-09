"""finals.vision.pyhulax_video — the video source, tested WITHOUT pyhulax.

FakeVideoStream + FakeDroneAPI drive PyhulaxVideoSource on the bare dev venv:
the None startup window, the BGR channel-order flag, the bounded ERROR->restart
ladder, and the staleness gate are all exercised with the SDK absent. The fake
frame's channel order is observable WITHOUT numpy (a tiny _ChannelArray), so
the RGB->BGR flip is asserted directly.
"""
from __future__ import annotations

import pytest

from finals.errors import SensorError, SensorTimeout
from finals.flight.pyhulax_adapter import FakeDroneAPI
from finals.types import FrameStamped
from finals.vision.pyhulax_video import FakeVideoStream, PyhulaxVideoSource


class FakeClock:
    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def make_source(stream, *, order="rgb", clock=None, **kw):
    api = FakeDroneAPI(video_stream=stream)
    kw["video_channel_order"] = order
    if clock is not None:
        kw["clock"] = clock
    return PyhulaxVideoSource("alpha", api, **kw), api


# ============================================================
# start() + channel order
# ============================================================
def test_start_and_flip_rgb_to_bgr():
    stream = FakeVideoStream(channels=("R", "G", "B"))
    src, api = make_source(stream, order="rgb")
    src.start(timeout_s=1.0)
    assert stream.started == 1
    assert ("set_video_stream", {"enabled": True}) in api.calls
    f = src.get_frame()
    assert isinstance(f, FrameStamped)
    assert f.source_id == "alpha"
    assert f.image.order == ("B", "G", "R")          # RGB flipped to BGR
    src.stop()


def test_channel_order_bgr_passthrough():
    stream = FakeVideoStream(channels=("B", "G", "R"))
    src, _ = make_source(stream, order="bgr")
    src.start(timeout_s=1.0)
    assert src.get_frame().image.order == ("B", "G", "R")   # already BGR, no flip
    src.stop()


def test_start_tolerates_none_window():
    stream = FakeVideoStream(channels=("R", "G", "B"), none_reads=3)
    src, _ = make_source(stream)
    src.start(timeout_s=2.0)                          # 3 Nones, then a frame
    assert src.get_frame() is not None
    src.stop()


def test_start_times_out_without_first_frame():
    stream = FakeVideoStream(none_reads=10_000)       # never produces in time
    src, _ = make_source(stream)
    with pytest.raises(SensorTimeout, match="None-window"):
        src.start(timeout_s=0.2)


def test_start_raises_when_stream_errors_before_first_frame():
    stream = FakeVideoStream()
    stream.go_error("camera fault")
    src, _ = make_source(stream)
    with pytest.raises(SensorTimeout, match="errored before the first frame"):
        src.start(timeout_s=1.0)


def test_start_twice_refused():
    stream = FakeVideoStream(channels=("R", "G", "B"))
    src, _ = make_source(stream)
    src.start(timeout_s=1.0)
    with pytest.raises(SensorError, match="twice"):
        src.start(timeout_s=1.0)
    src.stop()


# ============================================================
# healthy / staleness
# ============================================================
def test_healthy_then_goes_stale():
    clk = FakeClock()
    stream = FakeVideoStream(channels=("R", "G", "B"))
    src, _ = make_source(stream, clock=clk)
    src.start(timeout_s=1.0)
    assert src.healthy is True
    clk.advance(5.0)                                  # > stale_s, no new frame
    assert src.healthy is False
    src.stop()


# ============================================================
# the bounded ERROR -> restart ladder
# ============================================================
def test_error_triggers_bounded_restarts_then_unhealthy():
    stream = FakeVideoStream(channels=("R", "G", "B"))
    src, _ = make_source(stream, max_restarts=3)
    src.start(timeout_s=1.0)
    assert src.healthy is True
    stream.go_error("decode dead", stuck=True)        # survives stop()/start()
    for _ in range(10):                               # each poll => <=1 restart
        assert src.healthy is False
    assert stream.started == 1 + 3                    # initial + exactly 3 restarts
    assert stream.stopped == 3
    src.stop()


def test_restart_recovers_on_fresh_frame():
    clk = FakeClock()
    stream = FakeVideoStream(channels=("R", "G", "B"))
    src, _ = make_source(stream, clock=clk, max_restarts=3)
    src.start(timeout_s=1.0)
    clk.advance(5.0)                                  # last frame now stale
    stream.go_error(stuck=False)                      # a restart will clear it
    assert src.healthy is False                       # ERROR -> restart, still stale
    stream.push_frame()                               # a fresh frame arrives
    assert src.healthy is True
    assert src._restarts == 0                         # the ladder reset
    src.stop()


# ============================================================
# stop() — idempotent, never raises
# ============================================================
def test_stop_idempotent_and_quiets_source():
    stream = FakeVideoStream(channels=("R", "G", "B"))
    src, _ = make_source(stream)
    src.start(timeout_s=1.0)
    src.stop()
    src.stop()                                        # idempotent, never raises
    assert stream.stopped >= 1
    assert src.get_frame() is None
    assert src.healthy is False


# ============================================================
# constructor guards
# ============================================================
@pytest.mark.parametrize("kw, match", [
    (dict(video_channel_order="bgra"), "video_channel_order"),
    (dict(stale_s=0), "stale_s"),
    (dict(max_restarts=-1), "max_restarts"),
    (dict(restart_stop_timeout_s=0), "restart_stop_timeout_s"),
])
def test_constructor_guards(kw, match):
    api = FakeDroneAPI(video_stream=FakeVideoStream())
    with pytest.raises(ValueError, match=match):
        PyhulaxVideoSource("alpha", api, **kw)


def test_constructor_requires_real_api():
    with pytest.raises(ValueError, match="create_video_stream"):
        PyhulaxVideoSource("alpha", object())


def test_constructor_rejects_empty_source_id():
    with pytest.raises(ValueError, match="source_id"):
        PyhulaxVideoSource("", FakeDroneAPI(video_stream=FakeVideoStream()))
