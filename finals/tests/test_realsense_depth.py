"""RealSenseDepthSource — the opt-in real depth backend (props-off
flight_monitor's forward obstacle poll).

The SDK contact (pyrealsense2) is isolated in three overridable hooks
(_open / _read_grid / _close), so this whole suite runs on the BARE venv with NO
pyrealsense2 and NO camera: a fake subclass feeds scripted coarse depth grids and
we exercise the start/read/healthy/staleness/error contract + the config/main
wiring. A true on-camera run is exercised by `flight_monitor --depth-backend
realsense` (and the importorskip smoke at the bottom, skipped when the SDK is
absent), not here — the logic under test is the pacing/DepthFrame plumbing.
"""
import threading

import pytest

from finals.vision.depth import RealSenseDepthSource, DepthFrame


class _Clock:
    """A hand-cranked monotonic clock so staleness is deterministic (no sleeps)."""
    def __init__(self, t: float = 1000.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


class _FakeRS(RealSenseDepthSource):
    """RealSenseDepthSource with the pyrealsense2 hooks replaced by a scripted
    grid feed — proves the pacing/contract logic without hardware."""
    def __init__(self, *args, grids, **kwargs):
        super().__init__(*args, **kwargs)
        self._grids = list(grids)
        self._idx = 0
        self.opened = False
        self.closed = False

    def _open(self):
        self.opened = True
        return ("fake-pipeline",)

    def _read_grid(self, ctx):
        if self._idx >= len(self._grids):
            return None            # stream "drains" — latest freezes, can go stale
        g = self._grids[self._idx]
        self._idx += 1
        return g

    def _close(self, ctx):
        self.closed = True         # fake pipeline has no .stop(); just record it


# A 3-row x 4-col metres grid; the 0.0 cell is a no-return (distance_at -> None).
_GRID = [[1.5, 2.0, 0.0, 3.0],
         [1.0, 1.0, 1.0, 1.0],
         [5.0, 5.0, 5.0, 5.0]]


def _src(grids, clock, **kw):
    return _FakeRS("rs-test", width=8, height=6, grid_w=4, grid_h=3,
                   stale_s=2.0, grids=grids, clock=clock, **kw)


def test_start_publishes_first_frame_and_reads_back():
    clk = _Clock()
    src = _src([_GRID], clk)
    src.start(timeout_s=2.0)
    try:
        assert src.opened
        assert src.healthy
        frame = src.read()
        assert isinstance(frame, DepthFrame)
        assert (frame.width, frame.height) == (4, 3)
        assert frame.distance_at(0, 0) == pytest.approx(1.5)
        assert frame.distance_at(3, 2) == pytest.approx(5.0)
        assert frame.distance_at(2, 0) is None      # the 0.0 no-return cell
        assert frame.distance_at(99, 99) is None     # out of bounds
    finally:
        src.stop()
    assert src.closed


def test_read_is_a_copy_consumer_cannot_mutate_source():
    clk = _Clock()
    src = _src([_GRID], clk)
    src.start(timeout_s=2.0)
    try:
        f1 = src.read()
        f1.data_m[0][0] = -999.0
        f2 = src.read()
        assert f2.distance_at(0, 0) == pytest.approx(1.5)   # untouched
    finally:
        src.stop()


def test_healthy_goes_false_when_stale():
    clk = _Clock(1000.0)
    src = _src([_GRID], clk)              # one frame, then the stream drains
    src.start(timeout_s=2.0)
    try:
        assert src.healthy
        clk.t = 1000.0 + 5.0             # 5 s later, > stale_s=2.0
        assert not src.healthy
    finally:
        src.stop()


def test_no_first_frame_raises_sensor_timeout_with_open_error():
    from finals.errors import SensorTimeout

    class _Boom(RealSenseDepthSource):
        def _open(self):
            raise RuntimeError("no device connected")

    src = _Boom("rs-test", width=8, height=6, grid_w=4, grid_h=3)
    with pytest.raises(SensorTimeout) as ei:
        src.start(timeout_s=1.0)
    assert "no device connected" in str(ei.value)
    assert src.last_error is not None
    assert "RuntimeError" in src.last_error


def test_missing_sdk_surfaces_as_importerror_in_last_error():
    from finals.errors import SensorTimeout

    class _NoSdk(RealSenseDepthSource):
        def _open(self):
            raise ImportError("No module named 'pyrealsense2'")

    src = _NoSdk("rs-test")
    with pytest.raises(SensorTimeout):
        src.start(timeout_s=1.0)
    assert "ImportError" in (src.last_error or "")


def test_constructor_rejects_bad_args():
    with pytest.raises(ValueError):
        RealSenseDepthSource("")                       # empty source_id
    with pytest.raises(ValueError):
        RealSenseDepthSource("rs", fps=0)              # non-positive
    with pytest.raises(ValueError):
        RealSenseDepthSource("rs", width=8, grid_w=64)  # grid exceeds frame


def test_start_twice_is_rejected():
    clk = _Clock()
    src = _src([_GRID], clk)
    src.start(timeout_s=2.0)
    try:
        with pytest.raises(RuntimeError):
            src.start()
    finally:
        src.stop()


def test_no_dangling_pacing_threads_after_stop():
    clk = _Clock()
    src = _src([_GRID, _GRID, _GRID], clk)
    src.start(timeout_s=2.0)
    src.stop()
    rs_threads = [t for t in threading.enumerate()
                  if t.name.startswith("depth-rs-")]
    assert not rs_threads, f"leaked pacing thread(s): {rs_threads}"


# ---- config / main wiring -------------------------------------------------
def test_realsense_is_a_valid_backend_and_resolves():
    from finals.config import VALID_DEPTH_BACKENDS
    from finals.main import resolve_depth_source_cls

    assert "realsense" in VALID_DEPTH_BACKENDS
    assert resolve_depth_source_cls("realsense") is RealSenseDepthSource
    assert resolve_depth_source_cls("none") is None


def test_unknown_backend_still_raises():
    from finals.errors import ConfigError
    from finals.main import resolve_depth_source_cls

    with pytest.raises(ConfigError):
        resolve_depth_source_cls("lidar")


# ---- real-SDK smoke (skipped on the bare venv / no camera) ----------------
def test_pyrealsense2_import_smoke():
    pytest.importorskip("pyrealsense2")
    # SDK present: the class still imports/constructs without touching hardware
    # (the pipeline is only built in start() -> _open()). A true on-camera run is
    # left to flight_monitor --depth-backend realsense.
    src = RealSenseDepthSource("rs-smoke")
    assert src.source_id == "rs-smoke"
    assert src.healthy is False        # not started yet
