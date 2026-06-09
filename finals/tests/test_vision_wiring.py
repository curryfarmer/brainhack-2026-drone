"""main.py's vision wiring: the _WIRED_FRAME_BACKENDS gate (VideoWatchdog
present ONLY where a frame source actually feeds it — gazebo is WIRED as of
S8/SIM-4; pyhulax stays clean until S9/S10), the mock-flight-with-replay-frames
e2e, and the perception-crash screamer. Only the flight e2e needs cv2."""
from __future__ import annotations

import asyncio
import json
import os
import sys

import pytest

import finals.main as fmain
from finals.config import (DetectorConfig, DroneConfig, FinalsConfig,
                           GuardsConfig)
from finals.events import EventLog, read_events
from finals.guards import VideoWatchdog
from finals.sightings import SightingBus, SightingLog
from finals.types import FrameStamped
from finals.vision.perception import PerceptionLoop
from finals.vision.video import VideoSource

FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "fixtures")


def cfg_with(frame_backend: str, profile: str = "mock",
             flight_backend: str = "mock") -> FinalsConfig:
    return FinalsConfig(profile=profile, flight_backend=flight_backend,
                        frame_backend=frame_backend,
                        detector=DetectorConfig(), drones=[],
                        guards=GuardsConfig())


# ============================================================
# The wired-backends gate (the C2 pin)
# ============================================================
def test_build_guards_video_watchdog_only_for_wired_backends():
    drone = DroneConfig(id="alpha", phases=["takeoff_demo"])

    def has_watchdog(cfg: FinalsConfig) -> bool:
        return any(isinstance(g, VideoWatchdog)
                   for g in fmain._build_guards(cfg, drone))

    assert has_watchdog(cfg_with("replay")) is True
    assert has_watchdog(cfg_with("none")) is False
    # THE SIM-TRACK PIN: gazebo is WIRED as of S8/SIM-4 (GazeboRgbSource feeds
    # a real frame source over the sim/gz_camera_bridge TCP seam), so sitl.json's
    # frame_backend "gazebo" now BUILDS the VideoWatchdog. pyhulax stays OUT of
    # the wired set until S9/S10 — a watchdog built for an UNWIRED backend would
    # log a guaranteed-false "no frame EVER" DEGRADE every run (guards.py
    # reconciliation 5); the gate is the WIRED set, not frame_backend != "none".
    assert has_watchdog(cfg_with("gazebo", profile="sitl",
                                 flight_backend="mavsdk_sitl")) is True
    assert has_watchdog(cfg_with("pyhulax", profile="real",
                                 flight_backend="pyhulax")) is False
    assert fmain._frames_wired(cfg_with("replay")) is True
    assert fmain._frames_wired(cfg_with("gazebo")) is True
    assert fmain._frames_wired(cfg_with("pyhulax")) is False


# ============================================================
# Mock flight + replay frames: the full-wiring e2e (S8 rehearsal)
# ============================================================
def test_mock_flight_with_replay_frames_end_to_end(tmp_path, monkeypatch):
    pytest.importorskip("cv2")
    config = {
        "profile": "mock",
        "flight_backend": "mock",
        "frame_backend": "replay",
        "replay_dir": os.path.join(FIXTURES, "frames"),
        "replay_fps": 5.0,
        "run_dir": str(tmp_path / "runs"),
        "detector": {"backend": "none"},
        "drones": [
            {"id": "alpha", "phases": ["takeoff_demo"]},
            {"id": "bravo", "phases": ["takeoff_demo"]},
        ],
    }
    path = tmp_path / "mock_frames.json"
    path.write_text(json.dumps(config), encoding="utf-8")

    code = fmain.main(["--profile", "mock", "--config", str(path),
                       "--budget", "30"])
    assert code == 0

    run_dirs = list((tmp_path / "runs").iterdir())
    assert len(run_dirs) == 1
    run_dir = str(run_dirs[0])

    with SightingLog(os.path.join(run_dir, "sightings.csv")) as log:
        rows = log.snapshot()
    by_drone = {d: [s for s in rows if s.drone_id == d]
                for d in ("alpha", "bravo")}
    for drone_id, drone_rows in by_drone.items():
        assert drone_rows, (
            f"{drone_id}: a flying profile with wired frames must produce "
            f"PER-DRONE sightings (own ReplaySource, own PerceptionLoop)")
        assert {s.marker_id for s in drone_rows} == {17, 23, 42}

    evs = list(read_events(os.path.join(run_dir, "mission.jsonl")))
    # Live frames the whole mission: the VideoWatchdog is BUILT (wired
    # backend) but must never trip.
    video_trips = [e for e in evs if e["event"] == "guard_trip"
                   and e["data"]["guard"] == "VideoWatchdog"]
    assert video_trips == [], (
        f"VideoWatchdog tripped on a live stream: {video_trips}")
    assert "perception_crashed" not in [e["event"] for e in evs]
    # The flight itself is untouched by perception: both drones end DONE.
    run_end = [e for e in evs if e["event"] == "run_end"][0]
    assert run_end["data"]["states"] == {"alpha": "DONE", "bravo": "DONE"}


# ============================================================
# The perception-crash screamer (no cv2 needed)
# ============================================================
class CrashFakeSource(VideoSource):
    """Delivers one frame, never errors/exhausts — the crash comes from the
    marker detector, not the source."""

    def __init__(self):
        self.frame = FrameStamped(image=None, ts=1.0, frame_number=1,
                                  source_id="replay")

    def start(self, timeout_s: float = 10.0) -> None:
        pass

    def stop(self) -> None:
        pass

    def get_frame(self):
        return self.frame

    @property
    def healthy(self) -> bool:
        return True

    @property
    def source_id(self) -> str:
        return "replay"

    # the _areplay beat probes these (ReplaySource extras):
    exhausted = False
    errored = False
    delivered_count = 1


class SteppingSource(VideoSource):
    """Delivers a fresh frame per get_frame call up to n_frames, then
    reports exhausted — drives _areplay's normal end-of-frames path."""

    def __init__(self, n_frames: int = 4):
        self._n = n_frames
        self._served = 0
        self.exhausted = False
        self.errored = False

    def start(self, timeout_s: float = 10.0) -> None:
        pass

    def stop(self) -> None:
        pass

    def get_frame(self):
        if self._served < self._n:
            self._served += 1
        if self._served >= self._n:
            self.exhausted = True
        return FrameStamped(image=None, ts=float(self._served),
                            frame_number=self._served, source_id="replay")

    @property
    def healthy(self) -> bool:
        return not self.exhausted

    @property
    def source_id(self) -> str:
        return "replay"

    @property
    def delivered_count(self) -> int:
        return self._served


def test_areplay_exits_1_when_the_sighting_csv_dies(tmp_path, capsys):
    """The CRITICAL review pin: sightings.csv is THE score artifact — a run
    whose CSV died mid-way must NOT exit 0 with a silently truncated file."""
    from finals.sightings import SightingLogError
    from finals.vision.perception import CsvRecordingHealth
    from finals.types import Sighting

    def one_sighting_per_frame(frame, drone_id):
        return [Sighting(drone_id=drone_id, ts=frame.ts, source="aruco",
                         class_name="aruco_17", marker_id=17,
                         bbox_xyxy=(1.0, 1.0, 2.0, 2.0), confidence=1.0,
                         frame_shape=(480, 640),
                         frame_number=frame.frame_number)]

    class PoisonedLog:
        def append(self, s):
            raise SightingLogError("disk full (scripted)")

    cfg = cfg_with("replay", profile="replay", flight_backend="none")
    cfg.mission_budget_s = 10.0
    bus = SightingBus()
    source = SteppingSource(n_frames=4)        # > the 3-failure dead latch
    health = CsvRecordingHealth()
    run_dir = str(tmp_path)
    with EventLog(run_dir) as events:
        perception = PerceptionLoop(
            "replay", source, bus, events,
            detect_marker=one_sighting_per_frame,
            slog=PoisonedLog(), csv_health=health, sample_hz=50.0)
        code = asyncio.run(fmain._areplay(cfg, events, bus, source,
                                          perception, csv_health=health))
    assert health.dead is True
    assert code == 1, "exit 0 + truncated CSV is the forbidden outcome"
    err = capsys.readouterr().err
    assert "RECOVER" in err and "mission.jsonl" in err
    evs = list(read_events(os.path.join(run_dir, "mission.jsonl")))
    dead_events = [e for e in evs if e["event"] == "csv_recording_dead"]
    assert len(dead_events) == 1
    # The mirror really does hold the lost rows — the recovery claim is true.
    assert len([e for e in evs if e["event"] == "sighting"]) == 4


def test_areplay_exits_1_and_screams_when_perception_crashes(tmp_path,
                                                             capsys):
    def exploding_marker_detector(frame, drone_id):
        raise RuntimeError("marker detector bug (scripted)")

    cfg = cfg_with("replay", profile="replay", flight_backend="none")
    cfg.mission_budget_s = 10.0
    run_dir = str(tmp_path)
    bus = SightingBus()
    source = CrashFakeSource()
    with EventLog(run_dir) as events:
        perception = PerceptionLoop(
            "replay", source, bus, events,
            detect_marker=exploding_marker_detector)
        code = asyncio.run(fmain._areplay(cfg, events, bus, source,
                                          perception))
    assert code == 1, "a crashed perception task is NOT a clean run"
    err = capsys.readouterr().err
    assert "CRASHED" in err and "marker detector bug" in err, (
        "the screamer must print the full story — a dead perception task "
        "must never be a silent zero-sighting run")
    evs = list(read_events(os.path.join(run_dir, "mission.jsonl")))
    crashed = [e for e in evs if e["event"] == "perception_crashed"]
    assert len(crashed) == 1
    assert "marker detector bug" in crashed[0]["data"]["error"]
