"""finals.vision.video.ReplaySource — the latest-frame contract. Core tests
run WITHOUT cv2 via the injected `loader` seam + a FakeImage; only the
committed-PNG / video-file tests need cv2 (importorskip inside those tests).

NO wall-clock pacing assertions anywhere: Windows timer granularity is
~15.6 ms — the tests assert ordering / delivery / exhaustion / staleness
(via the injected stamp clock), never elapsed real time.
"""
from __future__ import annotations

import os
import threading
import time

import pytest

from finals.errors import SensorError, SensorTimeout
from finals.types import FrameStamped
from finals.vision.video import ReplaySource


class FakeClock:
    """Deterministic monotonic source — tests set .t explicitly."""

    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


class FakeImage:
    """The minimal surface ReplaySource touches: .shape + .copy()."""

    def __init__(self, tag: str, shape=(480, 640, 3)):
        self.tag = tag
        self.shape = shape

    def copy(self) -> "FakeImage":
        return FakeImage(self.tag, self.shape)


def make_frames_dir(tmp_path, names=("000.png", "001.png", "002.png")) -> str:
    d = tmp_path / "frames"
    d.mkdir()
    for n in names:
        (d / n).write_bytes(b"fake")        # decoded by the injected loader
    return str(d)


def fake_loader(path: str) -> FakeImage:
    return FakeImage(os.path.basename(path))


def wait_until(pred, timeout_s: float = 5.0, what: str = "condition") -> None:
    deadline = time.monotonic() + timeout_s
    while not pred():
        if time.monotonic() > deadline:
            pytest.fail(f"timed out after {timeout_s} s waiting for {what}")
        time.sleep(0.005)


@pytest.fixture
def sources():
    built = []
    yield built.append
    for s in built:
        s.stop()


# ============================================================
# Construction
# ============================================================
def test_missing_path_raises_sensor_error(tmp_path):
    with pytest.raises(SensorError, match="neither a directory"):
        ReplaySource("replay", str(tmp_path / "nope"))


def test_empty_dir_raises_and_lists_contents(tmp_path):
    d = tmp_path / "frames"
    d.mkdir()
    (d / "notes.txt").write_text("not a frame", encoding="utf-8")
    with pytest.raises(SensorError, match="notes.txt"):
        ReplaySource("replay", str(d))


@pytest.mark.parametrize("bad", [0, -1.0, float("inf"), float("nan"), True])
def test_bad_fps_rejected(tmp_path, bad):
    with pytest.raises(ValueError, match="fps"):
        ReplaySource("replay", make_frames_dir(tmp_path), fps=bad)


def test_empty_source_id_rejected(tmp_path):
    with pytest.raises(ValueError, match="source_id"):
        ReplaySource("", make_frames_dir(tmp_path))


# ============================================================
# The latest-frame contract
# ============================================================
def test_get_frame_none_before_start(tmp_path, sources):
    src = ReplaySource("replay", make_frames_dir(tmp_path),
                       loader=fake_loader)
    sources(src)
    assert src.get_frame() is None
    assert src.healthy is False


def test_frames_advance_exhaust_and_stay_drainable(tmp_path, sources):
    src = ReplaySource("replay", make_frames_dir(tmp_path), fps=100.0,
                       loop=False, loader=fake_loader)
    sources(src)
    src.start(timeout_s=5.0)
    wait_until(lambda: src.exhausted, what="exhaustion (loop=False)")
    # Sample after exhaustion: the LAST frame must still be drainable.
    f = src.get_frame()
    assert isinstance(f, FrameStamped)
    assert f.frame_number == 3 and f.image.tag == "002.png"
    assert f.source_id == "replay"
    assert src.delivered_count == 3
    assert src.healthy is False, "exhausted source must report unhealthy"


def test_frame_numbers_strictly_increase_and_files_sorted(tmp_path, sources):
    delivered = []
    lock = threading.Lock()

    def recording_loader(path):
        img = fake_loader(path)
        with lock:
            delivered.append(img.tag)
        return img

    # Names deliberately written out of order — delivery must be sorted.
    src = ReplaySource("replay",
                       make_frames_dir(tmp_path,
                                       ("b.png", "a.jpg", "c.jpeg")),
                       fps=100.0, loop=False, loader=recording_loader)
    sources(src)
    src.start(timeout_s=5.0)
    wait_until(lambda: src.exhausted, what="exhaustion")
    assert delivered == ["a.jpg", "b.png", "c.jpeg"]


def test_loop_wraps_with_monotonic_frame_numbers(tmp_path, sources):
    src = ReplaySource("replay", make_frames_dir(tmp_path), fps=200.0,
                       loop=True, loader=fake_loader)
    sources(src)
    src.start(timeout_s=5.0)
    # 3 files; > 3 deliveries proves the wrap; frame_number keeps counting
    # (perception dedupes on it — a wrapped file index would alias).
    wait_until(lambda: src.delivered_count > 4, what="a loop wrap")
    f = src.get_frame()
    assert f.frame_number == src.delivered_count or \
        f.frame_number == src.delivered_count - 1   # racing one delivery is fine
    assert f.frame_number > 3
    assert not src.exhausted


def test_latest_frame_is_a_copy(tmp_path, sources):
    src = ReplaySource("replay", make_frames_dir(tmp_path, ("a.png",)),
                       fps=100.0, loop=True, loader=fake_loader)
    sources(src)
    src.start(timeout_s=5.0)
    f1, f2 = src.get_frame(), src.get_frame()
    assert f1.image is not f2.image, (
        "get_frame must hand out COPIES — a shared buffer would let a "
        "detector see a frame mutate mid-inference")


# ============================================================
# Health / staleness
# ============================================================
def test_healthy_goes_stale_with_fake_clock(tmp_path, sources):
    clock = FakeClock(100.0)
    src = ReplaySource("replay", make_frames_dir(tmp_path), fps=100.0,
                       loop=False, stale_s=2.0, clock=clock,
                       loader=fake_loader)
    sources(src)
    src.start(timeout_s=5.0)
    wait_until(lambda: src.get_frame() is not None, what="first frame")
    assert src.healthy is True          # stamped at t=100, age 0
    clock.t += 3.0                      # all stamps now > stale_s old
    wait_until(lambda: src.exhausted, what="exhaustion")
    assert src.healthy is False


def test_start_timeout_raises_sensor_timeout_and_cleans_up(tmp_path):
    gate = threading.Event()

    def blocked_loader(path):
        gate.wait(10.0)
        return fake_loader(path)

    src = ReplaySource("replay", make_frames_dir(tmp_path),
                       loader=blocked_loader)
    try:
        with pytest.raises(SensorTimeout, match="no first frame"):
            src.start(timeout_s=0.1)
    finally:
        gate.set()
        src.stop()
    wait_until(lambda: not src._thread.is_alive(),
               what="pacing thread to end after the gate opened")


def test_loader_error_marks_errored_and_start_raises(tmp_path, capsys):
    def broken_loader(path):
        raise OSError(5, "disk on fire (scripted)")

    src = ReplaySource("replay", make_frames_dir(tmp_path),
                       loader=broken_loader)
    try:
        with pytest.raises(SensorTimeout, match="disk on fire"):
            src.start(timeout_s=5.0)
    finally:
        src.stop()
    assert src.healthy is False
    assert "disk on fire" in capsys.readouterr().err


def test_decode_failure_none_is_loud(tmp_path, capsys):
    src = ReplaySource("replay", make_frames_dir(tmp_path),
                       loader=lambda path: None)
    try:
        with pytest.raises(SensorTimeout, match="could not decode"):
            src.start(timeout_s=5.0)
    finally:
        src.stop()
    assert "could not decode" in capsys.readouterr().err


def test_bad_shape_from_loader_is_loud(tmp_path, sources):
    src = ReplaySource("replay", make_frames_dir(tmp_path),
                       loader=lambda path: FakeImage("bad", shape=(480, 640)))
    sources(src)
    with pytest.raises(SensorTimeout, match="HxWx3"):
        src.start(timeout_s=5.0)


# ============================================================
# Lifecycle
# ============================================================
def test_stop_idempotent_and_before_start(tmp_path):
    src = ReplaySource("replay", make_frames_dir(tmp_path),
                       loader=fake_loader)
    src.stop()                              # before start: fine
    src.stop()                              # idempotent: fine
    with pytest.raises(RuntimeError, match="after stop"):
        src.start()


def test_double_start_rejected(tmp_path, sources):
    src = ReplaySource("replay", make_frames_dir(tmp_path), fps=100.0,
                       loop=True, loader=fake_loader)
    sources(src)
    src.start(timeout_s=5.0)
    with pytest.raises(RuntimeError, match="twice"):
        src.start()


# ============================================================
# cv2-gated: the committed fixtures + video-file mode
# ============================================================
def test_reads_committed_png_fixtures(repo_root, sources):
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    frames_dir = os.path.join(repo_root, "finals", "tests", "fixtures",
                              "frames")
    src = ReplaySource("replay", frames_dir, fps=100.0, loop=False)
    sources(src)
    src.start(timeout_s=10.0)
    f = src.get_frame()
    assert isinstance(f.image, np.ndarray)
    assert f.image.dtype == np.uint8
    assert f.image.shape == (480, 640, 3), "fixtures are 640x480 BGR"
    wait_until(lambda: src.exhausted, what="fixture exhaustion")
    assert src.delivered_count == 4         # 000..003.png


def test_video_file_mode(tmp_path, sources):
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    path = str(tmp_path / "clip.avi")
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), 10.0,
                             (64, 48))
    assert writer.isOpened(), "cv2 build cannot write MJPG — environment issue"
    for shade in (0, 128, 255):
        writer.write(np.full((48, 64, 3), shade, dtype=np.uint8))
    writer.release()

    src = ReplaySource("replay", path, fps=100.0, loop=False)
    sources(src)
    src.start(timeout_s=10.0)
    wait_until(lambda: src.exhausted, what="video exhaustion")
    assert src.delivered_count == 3
    f = src.get_frame()
    assert f.image.shape == (48, 64, 3)


def test_zero_frame_video_fails_fast_with_the_right_diagnosis(tmp_path,
                                                              sources):
    """loop=False (the replay profile) + a video that opens but yields no
    frames must fail LOUD and FAST with 'yielded no frames' — not sit
    silently 'exhausted' while start() burns its whole timeout and then
    misdiagnoses a stuck decode thread."""
    cv2 = pytest.importorskip("cv2")
    path = str(tmp_path / "empty.avi")
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), 10.0,
                             (64, 48))
    writer.release()                            # header only, zero frames
    src = ReplaySource("replay", path, fps=100.0, loop=False)
    sources(src)
    t0 = time.monotonic()
    # Backend-dependent but equally honest: either the container is refused
    # at open ("could not open") or it opens and yields no frames.
    with pytest.raises(SensorTimeout, match="no frames|could not open"):
        src.start(timeout_s=8.0)
    assert time.monotonic() - t0 < 5.0, (
        "the zero-frame diagnosis must arrive from the decode thread, not "
        "from start() exhausting its timeout")
