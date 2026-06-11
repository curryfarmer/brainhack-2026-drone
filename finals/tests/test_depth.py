"""finals.vision.depth — the OPTIONAL DepthSource seam (SENSE-IR).

Covers the session gates: the DepthSource contract over FakeDepthSource
(start/read/healthy/stop, the latest-copy + staleness + lifecycle semantics
that mirror VideoSource); DepthFrame.distance_at (the get_distance primitive,
out-of-bounds / no-return -> None); and the ABSENT-DEPTH path is a CLEAN no-op
(depth_backend "none" wires nothing, no exception, the mission is unaffected) —
proven through main's resolver + config + a dry-run.

Dependency-free: FakeDepthSource is stdlib-only (depth maps are lists of
lists), so these run on the bare venv WITHOUT numpy.
"""
from __future__ import annotations

import threading

import pytest

from finals.errors import ConfigError, SensorTimeout
from finals.vision.depth import DepthFrame, DepthSource, FakeDepthSource


class FakeClock:
    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


# ============================================================
# 1. DepthFrame.distance_at — the get_distance primitive
# ============================================================
def test_depth_frame_distance_at_reads_metres():
    fr = DepthFrame([[1.0, 2.0, 3.0],
                     [4.0, 5.0, 6.0]], ts=1.0, source_id="alpha",
                    width=3, height=2)
    assert fr.distance_at(0, 0) == 1.0
    assert fr.distance_at(2, 1) == 6.0          # (cx=2, cy=1)
    assert fr.distance_at(1, 0) == 2.0


def test_depth_frame_distance_out_of_bounds_is_none():
    fr = DepthFrame([[1.0, 2.0]], ts=1.0, source_id="a", width=2, height=1)
    assert fr.distance_at(2, 0) is None         # cx past width
    assert fr.distance_at(0, 1) is None         # cy past height
    assert fr.distance_at(-1, 0) is None


def test_depth_frame_no_return_is_none():
    """A 0 / non-finite reading = no return (the RealSense sentinel) -> None,
    never a fabricated 0.0 m 'obstacle at the lens'."""
    fr = DepthFrame([[0.0, float("nan"), float("inf"), -1.0]],
                    ts=1.0, source_id="a", width=4, height=1)
    for cx in range(4):
        assert fr.distance_at(cx, 0) is None


# ============================================================
# 2. FakeDepthSource — the VideoSource-mirror contract
# ============================================================
def test_fake_depth_source_is_a_depthsource():
    assert issubclass(FakeDepthSource, DepthSource)
    src = FakeDepthSource("alpha")
    assert src.source_id == "alpha"


def test_fake_depth_start_read_stop_lifecycle():
    clock = FakeClock(100.0)
    src = FakeDepthSource("alpha", [[1.5, 2.5], [3.5, 4.5]], clock=clock)
    assert src.read() is None                   # nothing before start
    assert not src.healthy
    src.start(timeout_s=2.0)
    fr = src.read()
    assert fr is not None and isinstance(fr, DepthFrame)
    assert fr.source_id == "alpha"
    assert fr.width == 2 and fr.height == 2
    assert fr.distance_at(0, 0) == 1.5
    assert src.healthy
    src.stop()
    src.stop()                                  # idempotent


def test_fake_depth_read_returns_a_copy():
    """The latest-copy contract: mutating a read frame must not corrupt the
    source's live map (the RgbReceiver discipline)."""
    src = FakeDepthSource("alpha", [[1.0, 2.0]])
    src.start(timeout_s=2.0)
    try:
        fr = src.read()
        fr.data_m[0][0] = 999.0
        again = src.read()
        assert again.distance_at(0, 0) == 1.0   # source unchanged
    finally:
        src.stop()


def test_fake_depth_healthy_false_when_stale():
    clock = FakeClock(100.0)
    src = FakeDepthSource("alpha", [[1.0]], stale_s=2.0, clock=clock)
    src.start(timeout_s=2.0)
    try:
        assert src.healthy
        clock.t = 103.0                         # 3 s later, no new frame stamp
        assert not src.healthy                  # stale
    finally:
        src.stop()


def test_fake_depth_default_map_is_a_no_return_frame():
    """No frames -> a 1x1 zero map: a source that DELIVERS frames but no usable
    range (exercises distance_at's no-return path end to end)."""
    src = FakeDepthSource("alpha")
    src.start(timeout_s=2.0)
    try:
        fr = src.read()
        assert fr is not None
        assert fr.distance_at(0, 0) is None     # the 0.0 reads as no-return
    finally:
        src.stop()


def test_fake_depth_loop_false_exhausts():
    clock = FakeClock(0.0)
    src = FakeDepthSource("alpha", [[[1.0]], [[2.0]]],   # two distinct maps
                          fps=1000.0, loop=False, clock=clock)
    src.start(timeout_s=2.0)
    # Give the pacing thread time to deliver both and end.
    for _ in range(500):
        if src.exhausted:
            break
        threading.Event().wait(0.005)
    assert src.exhausted
    assert not src.healthy                       # exhausted -> unhealthy
    assert src.read() is not None                # last frame still readable
    src.stop()


def test_fake_depth_rejects_bad_params():
    with pytest.raises(ValueError, match="source_id"):
        FakeDepthSource("")
    with pytest.raises(ValueError, match="fps"):
        FakeDepthSource("a", fps=0.0)
    with pytest.raises(ValueError, match="stale_s"):
        FakeDepthSource("a", stale_s=-1.0)


def test_fake_depth_start_twice_refused():
    src = FakeDepthSource("a", [[1.0]])
    src.start(timeout_s=2.0)
    try:
        with pytest.raises(RuntimeError, match="twice"):
            src.start()
    finally:
        src.stop()


def test_fake_depth_single_map_vs_list_of_maps():
    """The _normalize seam: a single HxW map and a [map, map] list both work
    (a 1-row single map must not be mistaken for a list of 1-D maps)."""
    single = FakeDepthSource("a", [[1.0, 2.0, 3.0]])     # one 1x3 map
    single.start(timeout_s=2.0)
    try:
        assert single.read().width == 3
    finally:
        single.stop()
    multi = FakeDepthSource("b", [[[1.0]], [[2.0]]])      # two 1x1 maps
    multi.start(timeout_s=2.0)
    try:
        assert multi.read().width == 1
    finally:
        multi.stop()


# ============================================================
# 3. Absent-depth = clean no-op (the degrade-absent gate)
# ============================================================
def test_resolve_depth_none_is_none():
    """depth_backend 'none' -> NO DepthSource class (wires nothing)."""
    from finals.main import resolve_depth_source_cls
    assert resolve_depth_source_cls("none") is None


def test_resolve_depth_fake_is_fakedepthsource():
    from finals.main import resolve_depth_source_cls
    assert resolve_depth_source_cls("fake") is FakeDepthSource


def test_resolve_depth_unknown_is_loud():
    from finals.main import resolve_depth_source_cls
    with pytest.raises(ConfigError, match="depth_backend"):
        resolve_depth_source_cls("lidar")        # genuinely unwired backend


def test_build_depth_none_is_a_clean_noop(write_config):
    """_build_depth with depth_backend 'none' returns None and NEVER raises —
    the monocular mission path is unaffected."""
    from finals.config import load_config
    from finals.main import _build_depth
    cfg = load_config(write_config({
        "profile": "mock", "flight_backend": "mock", "frame_backend": "none",
        "detector": {"backend": "none"},
        "drones": [{"id": "alpha", "phases": ["takeoff_demo"]}],
    }))
    assert cfg.depth_backend == "none"
    assert _build_depth(cfg, "alpha") is None    # clean no-op, no exception


def test_build_depth_fake_builds_a_source(write_config):
    from finals.config import load_config
    from finals.main import _build_depth
    cfg = load_config(write_config({
        "profile": "mock", "flight_backend": "mock", "frame_backend": "none",
        "detector": {"backend": "none"}, "depth_backend": "fake",
        "drones": [{"id": "alpha", "phases": ["takeoff_demo"]}],
    }))
    ds = _build_depth(cfg, "alpha")
    assert isinstance(ds, FakeDepthSource) and ds.source_id == "alpha"


def test_config_depth_backend_default_none(write_config):
    from finals.config import load_config
    cfg = load_config(write_config({
        "profile": "mock", "flight_backend": "mock", "frame_backend": "none",
        "detector": {"backend": "none"},
        "drones": [{"id": "alpha", "phases": ["takeoff_demo"]}],
    }))
    assert cfg.depth_backend == "none"           # degrade-absent default


def test_config_depth_fake_accepted(write_config):
    from finals.config import load_config
    cfg = load_config(write_config({
        "profile": "mock", "flight_backend": "mock", "frame_backend": "none",
        "detector": {"backend": "none"}, "depth_backend": "fake",
        "drones": [{"id": "alpha", "phases": ["takeoff_demo"]}],
    }))
    assert cfg.depth_backend == "fake"


def test_config_depth_unknown_is_loud(write_config):
    from finals.config import load_config
    with pytest.raises(ConfigError, match="depth_backend"):
        load_config(write_config({
            "profile": "mock", "flight_backend": "mock", "frame_backend": "none",
            "detector": {"backend": "none"}, "depth_backend": "lidar",
            "drones": [{"id": "alpha", "phases": ["takeoff_demo"]}],
        }))
