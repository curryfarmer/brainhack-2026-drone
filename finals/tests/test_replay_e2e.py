"""The replay profile end to end (smoke requirement 7): python -m
finals.main --profile replay -> exit 0, sightings.csv rows match the
committed fixture markers, mission.jsonl mirrors them, loud summary.
cv2-gated: the real ReplaySource decodes the committed PNGs."""
from __future__ import annotations

import json
import os

import pytest

cv2 = pytest.importorskip("cv2")

from finals.events import read_events                            # noqa: E402
from finals.main import main                                     # noqa: E402
from finals.sightings import SightingLog                         # noqa: E402

FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "fixtures")
N_MARKER_FRAMES = 3                  # 000/001/003.png (002 is blank)
MARKERS_PER_FRAME = 3                # every marker frame carries 17+23+42


def write_replay_config(tmp_path, **extra) -> str:
    cfg = {
        "profile": "replay",
        "flight_backend": "none",
        "frame_backend": "replay",
        "replay_dir": os.path.join(FIXTURES, "frames"),
        "replay_fps": 5.0,
        "mission_budget_s": 20.0,
        "run_dir": str(tmp_path / "runs"),
        "detector": {"backend": "none"},
        "drones": [],
        **extra,
    }
    path = tmp_path / "replay_test.json"
    path.write_text(json.dumps(cfg), encoding="utf-8")
    return str(path)


def only_run_dir(tmp_path) -> str:
    dirs = list((tmp_path / "runs").iterdir())
    assert len(dirs) == 1, f"expected exactly one run dir, got {dirs}"
    return str(dirs[0])


def csv_rows(run_dir: str):
    with SightingLog(os.path.join(run_dir, "sightings.csv")) as log:
        return log.snapshot()


def test_replay_profile_end_to_end(tmp_path, capsys):
    code = main(["--profile", "replay",
                 "--config", write_replay_config(tmp_path)])
    assert code == 0

    run_dir = only_run_dir(tmp_path)
    rows = csv_rows(run_dir)
    assert rows, "the fixture markers must land in sightings.csv"
    # Set semantics, not exact rows: a latest-frame sampler may miss a frame
    # under CI scheduling — but every marker frame carries ALL THREE ids
    # (fixture design), so the id set is schedule-proof.
    assert {s.marker_id for s in rows} == {17, 23, 42}
    assert len(rows) <= N_MARKER_FRAMES * MARKERS_PER_FRAME
    for s in rows:
        assert s.source == "aruco"
        assert s.confidence == 1.0
        assert s.frame_shape == (480, 640)
        assert s.drone_id == "replay"
        assert s.frame_number is not None
        assert s.bearing_deg is None       # no telemetry on the no-drone path

    # The event log mirrors the bus exactly once per sighting.
    evs = list(read_events(os.path.join(run_dir, "mission.jsonl")))
    kinds = [e["event"] for e in evs]
    assert "run_start" in kinds and "run_end" in kinds
    assert "replay_exhausted" in kinds, "exhaustion (not budget) ends the run"
    sighting_events = [e for e in evs if e["event"] == "sighting"]
    assert len(sighting_events) == len(rows), (
        "every CSV row goes through the bus and is mirrored as exactly one "
        "'sighting' event")
    run_end = [e for e in evs if e["event"] == "run_end"][0]
    assert run_end["data"]["exit_code"] == 0
    assert run_end["data"]["sightings"] == len(rows)

    out = capsys.readouterr().out
    assert "REPLAY SUMMARY" in out and "aruco_17" in out


def test_replay_with_canned_detector_adds_yolo_rows(tmp_path):
    code = main(["--profile", "replay", "--config", write_replay_config(
        tmp_path,
        detector={"backend": "canned",
                  "canned_script": os.path.join(FIXTURES,
                                                "canned_script.json"),
                  "class_map": {"car": "robomaster"}})])
    assert code == 0
    rows = csv_rows(only_run_dir(tmp_path))
    yolo = [s for s in rows if s.source == "yolo"]
    assert len(yolo) == 1, (
        "the canned script fires once (after_n_submits=2) — its detection "
        "must reach the CSV through the worker-thread callback")
    assert yolo[0].class_name == "robomaster"          # class_map applied
    assert yolo[0].confidence == 0.9
    assert {s.marker_id for s in rows if s.source == "aruco"} == {17, 23, 42}


def test_replay_qr_backend(tmp_path):
    code = main(["--profile", "replay", "--config", write_replay_config(
        tmp_path,
        marker_backend="qr",
        replay_dir=os.path.join(FIXTURES, "frames_qr"))])
    assert code == 0
    rows = csv_rows(only_run_dir(tmp_path))
    assert rows, "the QR fixture must decode"
    assert all(s.source == "qr" for s in rows)
    assert {s.class_name for s in rows} == {"qr_7"}
    assert {s.marker_id for s in rows} == {7}


def test_replay_missing_replay_dir_is_config_error(tmp_path, capsys):
    code = main(["--profile", "replay", "--config", write_replay_config(
        tmp_path, replay_dir=str(tmp_path / "no_such_frames"))])
    assert code == 2                       # ConfigError at load, not a crash
    assert "not found on disk" in capsys.readouterr().err


def test_replay_no_detector_flag_still_runs_markers(tmp_path):
    """--no-detector kills YOLO only; the marker path is ALWAYS on."""
    code = main(["--profile", "replay", "--no-detector",
                 "--config", write_replay_config(
                     tmp_path,
                     detector={"backend": "canned",
                               "canned_script": os.path.join(
                                   FIXTURES, "canned_script.json")})])
    assert code == 0
    rows = csv_rows(only_run_dir(tmp_path))
    assert {s.source for s in rows} == {"aruco"}
