"""finals.vision.perception — lockstep determinism pins, the publish-site
CSV+bus rule, enrichment, the corrected bearing sign, shed/degrade, health.
Runs WITHOUT cv2/numpy: fake sources + fake images + injected detectors —
exactly the seams the module ships. The one real-threading test drives a
real CannedDetector (worker thread -> bus handoff)."""
from __future__ import annotations

import asyncio
import os
import time

import pytest

from finals.events import EventLog, read_events
from finals.sightings import SightingBus, SightingLog, SightingLogError
from finals.types import FrameStamped, PositionQuality, Sighting, Telemetry
from finals.vision.detector import CannedDetector
from finals.vision.perception import (PerceptionLoop, bearing_from_bbox,
                                      make_detection_callback)
from finals.vision.video import VideoSource

DET = {"bbox": [100.0, 100.0, 200.0, 200.0], "confidence": 0.9,
       "class_id": 0, "class_name": "car"}


class FakeClock:
    """Deterministic monotonic source — tests set .t explicitly."""

    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


class FakeImage:
    def __init__(self, shape=(480, 640, 3)):
        self.shape = shape

    def copy(self):
        return FakeImage(self.shape)


class FakeVideoSource(VideoSource):
    """Lockstep-steppable latest-frame source."""

    def __init__(self, source_id: str = "replay"):
        self._id = source_id
        self.frame = None
        self.healthy_flag = True

    def start(self, timeout_s: float = 10.0) -> None:
        pass

    def stop(self) -> None:
        pass

    def get_frame(self):
        return self.frame

    @property
    def healthy(self) -> bool:
        return self.healthy_flag

    @property
    def source_id(self) -> str:
        return self._id

    def step(self, n: int, ts: float = None) -> None:
        self.frame = FrameStamped(image=FakeImage(),
                                  ts=float(n) if ts is None else ts,
                                  frame_number=n, source_id=self._id)


def marker_per_frame(frame: FrameStamped, drone_id: str):
    """Fake detector: exactly one minimal sighting per frame, tagged with
    the frame number so duplicates are visible."""
    return [Sighting(
        drone_id=drone_id, ts=frame.ts, source="aruco",
        class_name="aruco_17", marker_id=17,
        bbox_xyxy=(100.0, 100.0, 200.0, 200.0), confidence=1.0,
        frame_shape=(480, 640), frame_number=frame.frame_number)]


def wait_until(pred, timeout_s: float = 5.0, what: str = "condition") -> None:
    deadline = time.monotonic() + timeout_s
    while not pred():
        if time.monotonic() > deadline:
            pytest.fail(f"timed out after {timeout_s} s waiting for {what}")
        time.sleep(0.01)


def events_of(run_dir: str):
    return list(read_events(os.path.join(run_dir, "mission.jsonl")))


@pytest.fixture
def run_dir(tmp_path):
    return str(tmp_path)


def make_loop(run_dir, **kw):
    source = kw.pop("source", FakeVideoSource())
    bus = kw.pop("bus", SightingBus())
    events = EventLog(run_dir)
    kw.setdefault("detect_marker", marker_per_frame)
    loop = PerceptionLoop("alpha", source, bus, events, **kw)
    return loop, source, bus, events


# ============================================================
# Bearing math (smoke requirement 6 — the corrected sign)
# ============================================================
def test_bearing_sign_marker_right_of_center_is_negative():
    # cx = 480 = 3w/4 on a 640-wide frame; offset = +60/4 = +15 deg.
    bearing = bearing_from_bbox(0.0, (440.0, 0.0, 520.0, 80.0), 640, 60.0)
    assert bearing == pytest.approx(-15.0), (
        "Under the BINDING CCW+ yaw convention (finals/flight/"
        "dead_reckon.py: rotate(+90) = nose 90 deg counter-clockwise from "
        "above), a target RIGHT of frame centre lies CLOCKWISE of the nose "
        "— at DECREASING yaw. bearing must be yaw MINUS the pixel offset "
        "(the original types.py comment had it as PLUS — the known "
        "upstream conflict, resolved S7).")


def test_bearing_left_of_center_is_positive_and_yaw_shifts():
    assert bearing_from_bbox(0.0, (120.0, 0.0, 200.0, 80.0), 640,
                             60.0) == pytest.approx(15.0)
    assert bearing_from_bbox(45.0, (280.0, 0.0, 360.0, 80.0), 640,
                             60.0) == pytest.approx(45.0)   # dead centre


def test_bearing_normalized_to_half_open_interval():
    # yaw 170, target ~20 deg left -> 190 -> normalized to -170.
    cx = 320.0 - (20.0 / 60.0) * 640.0
    bearing = bearing_from_bbox(170.0, (cx - 10, 0.0, cx + 10, 20.0), 640,
                                60.0)
    assert bearing == pytest.approx(-170.0)
    assert -180.0 < bearing <= 180.0


@pytest.mark.parametrize("kwargs", [
    {"yaw_deg": float("nan")},
    {"hfov_deg": float("inf")},
    {"frame_w": 0},
    {"frame_w": 640.0},      # float w: a real frame_shape is ints
])
def test_bearing_rejects_poison_inputs(kwargs):
    args = {"yaw_deg": 0.0, "bbox_xyxy": (0.0, 0.0, 10.0, 10.0),
            "frame_w": 640, "hfov_deg": 60.0, **kwargs}
    with pytest.raises(ValueError):
        bearing_from_bbox(**args)


# ============================================================
# Lockstep determinism (the dedupe pin behind the e2e set-assertions)
# ============================================================
def test_every_distinct_frame_processed_exactly_once(run_dir, tmp_path):
    slog = SightingLog(str(tmp_path / "s.csv"))
    loop, source, bus, events = make_loop(run_dir, slog=slog)
    try:
        loop.sample_once()                   # no frame yet -> nothing
        for n, samples in ((1, 3), (2, 1), (3, 2)):
            source.step(n)
            for _ in range(samples):         # over-sampling must not dup
                loop.sample_once()
        _, sightings = bus.drain_after(0)
        assert [s.frame_number for s in sightings] == [1, 2, 3], (
            "a latest-frame source sampled faster than it delivers must "
            "yield each frame EXACTLY once (dedupe on frame_number)")
        assert [s.frame_number for s in slog.snapshot()] == [1, 2, 3], (
            "the CSV is appended at the PUBLISH site — same rows as the bus")
        assert loop.stats()["frames_sampled"] == 3
    finally:
        slog.close()
        events.close()


def test_csv_and_bus_rows_match_with_enrichment(run_dir, tmp_path):
    slog = SightingLog(str(tmp_path / "s.csv"))
    loop, source, bus, events = make_loop(run_dir, slog=slog,
                                          camera_hfov_deg=60.0)
    loop.set_telemetry_source(lambda: Telemetry(
        ts=1.0, yaw_deg=0.0, altitude_m=1.5,
        position_m=(2.0, -3.0, 1.5),
        position_quality=PositionQuality.DEAD_RECKONING))
    try:
        source.step(1)
        loop.sample_once()
        s = bus.latest("alpha")
        assert s.drone_yaw_deg == 0.0
        assert s.drone_alt_m == 1.5
        # bbox centre cx=150 on w=640: offset = (150-320)/640*60 = -15.9375
        assert s.bearing_deg == pytest.approx(15.9375)
        assert s.pos_quality is PositionQuality.DEAD_RECKONING
        assert s.est_north_m == 2.0 and s.est_east_m == -3.0
        assert slog.snapshot() == [s], "CSV row == published sighting"
    finally:
        slog.close()
        events.close()


def test_no_telemetry_source_means_unenriched_never_dropped(run_dir):
    loop, source, bus, events = make_loop(run_dir, camera_hfov_deg=60.0)
    try:
        source.step(1)
        loop.sample_once()
        s = bus.latest("alpha")
        assert s is not None, (
            "missing telemetry must NOT drop sightings (the qualifier "
            "detection_loop silently skipped frames without pose — the bug "
            "class fixed here)")
        assert s.bearing_deg is None and s.drone_yaw_deg is None
    finally:
        events.close()


def test_poisoned_csv_latches_after_consecutive_failures(run_dir, capsys):
    """A log that fails EVERY append (disk full / poisoned) latches dead
    after 3 consecutive failures; the bus keeps flowing throughout."""
    class PoisonedLog:
        def append(self, s):
            raise SightingLogError("disk full (scripted)")

    loop, source, bus, events = make_loop(run_dir, slog=PoisonedLog())
    try:
        for n in (1, 2, 3, 4):
            source.step(n)
            loop.sample_once()
        _, sightings = bus.drain_after(0)
        assert [s.frame_number for s in sightings] == [1, 2, 3, 4], (
            "a dead CSV must not stop the bus — intel flow survives "
            "forensics death")
        assert loop.stats()["csv_dead"] is True
        assert loop.stats()["csv_append_failures"] == 3, (
            "after the dead-latch (3 consecutive) no further appends are "
            "attempted")
        err = capsys.readouterr().err
        assert err.count("append FAILED") == 3 and "disk full" in err
        assert "judged DEAD" in err and "mission.jsonl" in err, (
            "the latch must scream AND name the recovery source")
    finally:
        events.close()


def test_one_bad_row_does_not_kill_csv_recording(run_dir, tmp_path):
    """The F2b pin: a codec refusal of ONE malformed row leaves the
    SightingLog healthy — recording must continue for valid rows (the
    publish-site charter: never lose score rows to a single bad sighting)."""
    real = SightingLog(str(tmp_path / "s.csv"))

    class FlakyOnce:
        """Raises for exactly one append, passes the rest through."""

        def __init__(self):
            self.n = 0

        def append(self, s):
            self.n += 1
            if self.n == 2:
                raise SightingLogError("one malformed row (scripted)")
            return real.append(s)

    loop, source, bus, events = make_loop(run_dir, slog=FlakyOnce())
    try:
        for n in (1, 2, 3, 4):
            source.step(n)
            loop.sample_once()
        assert loop.stats()["csv_dead"] is False
        assert loop.stats()["csv_append_failures"] == 1
        assert [s.frame_number for s in real.snapshot()] == [1, 3, 4], (
            "only the ONE refused row is lost; recording continues")
    finally:
        real.close()
        events.close()


# ============================================================
# shed / degrade
# ============================================================
def test_shed_latches_rate_and_logs_event(run_dir, capsys):
    loop, source, bus, events = make_loop(run_dir, sample_hz=10.0,
                                          degraded_hz=1.0)
    try:
        assert loop.current_period_s == pytest.approx(0.1)
        loop.shed("test trip")
        assert loop.degraded is True
        assert loop.current_period_s == pytest.approx(1.0)
        loop.shed("second trip is a no-op")     # idempotent latch
        events.close()
        degr = [e for e in events_of(run_dir)
                if e["event"] == "perception_degraded"]
        assert len(degr) == 1 and degr[0]["data"]["reason"] == "test trip"
        assert "DEGRADE" in capsys.readouterr().err
    finally:
        events.close()


def test_auto_shed_when_detector_drops(run_dir):
    class FakeDetector:
        dropped_total = 0
        submits = []

        def submit_image(self, image, context=None):
            self.submits.append(context)

    det = FakeDetector()
    loop, source, bus, events = make_loop(run_dir, detector=det)
    try:
        source.step(1)
        loop.sample_once()
        assert loop.degraded is False
        assert len(det.submits) == 1
        det.dropped_total = 2                   # the pool started dropping
        source.step(2)
        loop.sample_once()
        assert loop.degraded is True, (
            "a backing-up detector queue must shed the sample rate "
            "(perception.py spec: drop to 1 Hz, logged, never silent)")
    finally:
        events.close()


def test_detector_context_is_fresh_and_complete(run_dir):
    class FakeDetector:
        dropped_total = 0

        def __init__(self):
            self.contexts = []

        def submit_image(self, image, context=None):
            self.contexts.append(context)

    det = FakeDetector()
    loop, source, bus, events = make_loop(run_dir, detector=det)
    loop.set_telemetry_source(lambda: Telemetry(ts=1.0, yaw_deg=30.0,
                                                altitude_m=1.2))
    try:
        source.step(1, ts=42.0)
        loop.sample_once()
        source.step(2, ts=43.0)
        loop.sample_once()
        c0, c1 = det.contexts
        assert c0 is not c1, "the pool MUTATES context — fresh dict per submit"
        assert c0 == {"drone_id": "alpha", "ts": 42.0, "yaw": 30.0,
                      "alt": 1.2, "position_m": None,
                      "position_quality": PositionQuality.NONE,
                      "frame_shape": (480, 640), "frame_number": 1}
    finally:
        events.close()


# ============================================================
# Health logging
# ============================================================
def test_unhealthy_logged_after_period_not_spammed(run_dir, capsys):
    clock = FakeClock(100.0)
    loop, source, bus, events = make_loop(run_dir, clock=clock,
                                          unhealthy_log_period_s=5.0)
    try:
        source.healthy_flag = False
        loop.sample_once()                      # t=100: starts the clock
        clock.t = 103.0
        loop.sample_once()                      # < 5 s unhealthy: silent
        assert "unhealthy" not in capsys.readouterr().err
        clock.t = 106.0
        loop.sample_once()                      # > 5 s: first scream
        clock.t = 107.0
        loop.sample_once()                      # within the period: silent
        assert capsys.readouterr().err.count("unhealthy") == 1
        clock.t = 112.0
        loop.sample_once()                      # next period: screams again
        assert capsys.readouterr().err.count("unhealthy") == 1
        source.healthy_flag = True
        loop.sample_once()                      # recovery resets the clock
        clock.t = 113.0
        source.healthy_flag = False
        loop.sample_once()
        assert "unhealthy" not in capsys.readouterr().err
        events.close()
        assert len([e for e in events_of(run_dir)
                    if e["event"] == "perception_unhealthy"]) == 2
    finally:
        events.close()


# ============================================================
# run() bounds + agents-see-it-via-the-bus (the PublishingPhase pattern
# in reverse: perception publishes, the consumer drains ctx-style)
# ============================================================
def test_run_returns_on_stop_event_and_deadline(run_dir):
    async def go():
        loop_, source, bus, events = make_loop(run_dir)
        stop = asyncio.Event()
        stop.set()
        await loop_.run(deadline=time.monotonic() + 60.0, stop_event=stop)
        await loop_.run(deadline=time.monotonic() - 1.0,
                        stop_event=asyncio.Event())   # already past: returns
        events.close()

    asyncio.run(go())


def test_run_publishes_while_stepping(run_dir):
    """Drive run() for real beats while the source steps — the consumer
    (an agent, in production) sees the sightings via its own bus cursor."""
    loop_, source, bus, events = make_loop(run_dir, sample_hz=100.0)

    async def go():
        stop = asyncio.Event()
        task = asyncio.get_running_loop().create_task(
            loop_.run(deadline=time.monotonic() + 10.0, stop_event=stop))
        cursor, seen = 0, []
        for n in (1, 2, 3):
            source.step(n)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                cursor, fresh = bus.drain_after(cursor, drone_id="alpha")
                seen.extend(s.frame_number for s in fresh)
                if n in seen:
                    break
                await asyncio.sleep(0.005)
        stop.set()
        await task
        assert seen == [1, 2, 3]

    try:
        asyncio.run(go())
    finally:
        events.close()


# ============================================================
# The worker-thread callback (real CannedDetector -> bus handoff)
# ============================================================
def test_yolo_callback_to_bus_via_canned_detector(run_dir, tmp_path):
    bus = SightingBus()
    slog = SightingLog(str(tmp_path / "s.csv"))
    cb = make_detection_callback(bus, slog,
                                 class_map={"car": "robomaster"},
                                 camera_hfov_deg=60.0)
    det = CannedDetector([{"after_n_submits": 1, "detections": [dict(DET)]}],
                         cb)
    loop_, source, _bus_unused, events = make_loop(run_dir, detector=det,
                                                   bus=bus)
    loop_.set_telemetry_source(lambda: Telemetry(ts=1.0, yaw_deg=0.0,
                                                 altitude_m=1.5))
    try:
        source.step(1, ts=42.0)
        loop_.sample_once()                     # submits to the worker pool
        wait_until(lambda: bus.latest("alpha", source="yolo") is not None,
                   what="the worker-thread callback to publish")
        s = bus.latest("alpha", source="yolo")
        assert s.class_name == "robomaster", "class_map must rename"
        assert s.confidence == 0.9
        assert s.ts == 42.0 and s.frame_number == 1
        assert s.drone_yaw_deg == 0.0 and s.drone_alt_m == 1.5
        # bbox centre cx=150: same enrichment math as the marker path.
        assert s.bearing_deg == pytest.approx(15.9375)
        assert slog.snapshot() == [s], "publish-site CSV on the worker thread"
    finally:
        det.stop()
        slog.close()
        events.close()


def test_unmapped_class_skipped_with_one_warning(capsys):
    bus = SightingBus()
    cb = make_detection_callback(bus, None, class_map={"truck": "x"},
                                 camera_hfov_deg=None)
    ctx = {"drone_id": "alpha", "ts": 1.0, "frame_shape": (480, 640)}
    cb([dict(DET)], None, dict(ctx))
    cb([dict(DET)], None, dict(ctx))
    assert bus.latest("alpha") is None
    assert capsys.readouterr().err.count("no detector.class_map entry") == 1


def test_callback_position_parity_with_marker_path():
    """A YOLO sighting must get the SAME position enrichment the marker
    sighting on the same frame gets — the fallback channel must not
    systematically under-report est_north/east."""
    bus = SightingBus()
    cb = make_detection_callback(bus, None, class_map={},
                                 camera_hfov_deg=None)
    cb([dict(DET)], None,
       {"drone_id": "alpha", "ts": 1.0, "frame_shape": (480, 640),
        "yaw": 0.0, "alt": 1.5, "position_m": (2.0, -3.0, 1.5),
        "position_quality": PositionQuality.DEAD_RECKONING})
    s = bus.latest("alpha")
    assert s.est_north_m == 2.0 and s.est_east_m == -3.0
    assert s.pos_quality is PositionQuality.DEAD_RECKONING


def test_empty_class_map_is_identity_passthrough():
    bus = SightingBus()
    cb = make_detection_callback(bus, None, class_map={},
                                 camera_hfov_deg=None)
    cb([dict(DET)], None,
       {"drone_id": "alpha", "ts": 1.0, "frame_shape": (480, 640)})
    s = bus.latest("alpha")
    assert s.class_name == "car"
    assert s.bearing_deg is None               # no yaw, no hfov -> None


def test_callback_missing_context_drops_loudly_once(capsys):
    bus = SightingBus()
    cb = make_detection_callback(bus, None, class_map={},
                                 camera_hfov_deg=None)
    cb([dict(DET)], None, {})                  # wiring bug: empty context
    cb([dict(DET)], None, {})
    assert bus.latest("alpha") is None
    assert capsys.readouterr().err.count("wiring bug") == 1


# ============================================================
# Constructor gates
# ============================================================
def test_constructor_rejects_bad_args(run_dir):
    events = EventLog(run_dir)
    bus = SightingBus()
    src = FakeVideoSource()
    try:
        with pytest.raises(ValueError, match="drone_id"):
            PerceptionLoop("", src, bus, events,
                           detect_marker=marker_per_frame)
        with pytest.raises(ValueError, match="VideoSource"):
            PerceptionLoop("a", object(), bus, events,
                           detect_marker=marker_per_frame)
        with pytest.raises(ValueError, match="detect_marker"):
            PerceptionLoop("a", src, bus, events, detect_marker=None)
        with pytest.raises(ValueError, match="sample_hz"):
            PerceptionLoop("a", src, bus, events,
                           detect_marker=marker_per_frame, sample_hz=0)
        with pytest.raises(ValueError, match="LOWER"):
            PerceptionLoop("a", src, bus, events,
                           detect_marker=marker_per_frame,
                           sample_hz=1.0, degraded_hz=5.0)
        loop_ = PerceptionLoop("a", src, bus, events,
                               detect_marker=marker_per_frame)
        with pytest.raises(ValueError, match="telemetry"):
            loop_.set_telemetry_source(None)
    finally:
        events.close()
