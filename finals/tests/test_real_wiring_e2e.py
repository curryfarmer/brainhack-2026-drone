"""End-to-end REAL-profile MISSION wiring — the composition root flown, with no
pyhulax, no Dola, no torch, no real cv2 detector.

This sits ABOVE test_main_real_preflight_only (test_orchestrator.py), which
proves the real fleet composes and passes the P0-P10 GATE but then DISCONNECTS
without flying. The seam it does NOT cover is the one that actually arms:

    main(--profile real)
      -> _run_mission -> _build_fleet_support -> _build_agents(frame_backend=
         pyhulax: ONE shared DroneAPI per drone feeds BOTH PyhulaxAdapter and
         PyhulaxVideoSource)
      -> _amain -> run_preflight(P0-P10, including the P10 operator-GO) leaves
         the link up + video started
      -> the PerceptionLoop tasks run live over the SAME link
      -> Orchestrator.run() flies every drone to DONE
      -> run_end exit 0.

Nothing else in the suite runs the real backend's MISSION loop (the preflight
tests stop at the gate; the LANDED-mission e2e, test_nav_e2e, runs over the
MockAdapter, not the pyhulax composition). This is that missing proof.

How it stays deterministic + dependency-free:
- _make_shared_pyhulax_api -> FakeDroneAPI(video_stream=FakeVideoStream()), one
  per drone, CAPTURED so the same-link invariant is assertable.
- preflight._default_discover -> fake {plane_id: ip} (no Dola UDP).
- preflight._stdin_go_reader -> "GO" (P10 authorizes without a TTY; the mission
  path, unlike --preflight-only, really runs P10).
- aruco.make_marker_detector -> no-op (no cv2 on the fake frame).
- main._build_detector -> None: the YOLO pool is an orthogonal, already-tested
  concern (test_vision_detector / test_pad_weights_e2e). Stubbing it keeps this
  test torch-free; the PerceptionLoop still RUNS (marker-only) and still pulls
  frames over the shared link, which is the property under test.
- --phases takeoff_demo: a position-blind takeoff/land that completes without
  needing any sighting, so DONE depends on the WIRING, not on perception output.

Run: python -m pytest finals/tests/test_real_wiring_e2e.py -q -p no:randomly
"""
from __future__ import annotations

import json
import os

import pytest

from finals.events import read_events

_CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs")


def _mission_events(run_dir):
    return list(read_events(os.path.join(run_dir, "mission.jsonl")))


def _heartbeat(run_dir):
    with open(os.path.join(run_dir, "heartbeat.json"), encoding="utf-8") as f:
        return json.load(f)


def _only_run_dir(tmp_path):
    run_dirs = list((tmp_path / "runs_finals").iterdir())
    assert len(run_dirs) == 1, f"expected one run dir, got {run_dirs}"
    return str(run_dirs[0])


def _wire_fake_real_fleet(monkeypatch):
    """The real-mission composition with every SDK/IO seam faked. Returns the
    list that captures one FakeDroneAPI per drone (in cfg.drones order), so a
    test can read back the calls the shared link actually received."""
    import finals.main as fmain
    import finals.preflight as pf
    import finals.vision.aruco as aruco
    from finals.flight.pyhulax_adapter import FakeDroneAPI
    from finals.vision.pyhulax_video import FakeVideoStream

    apis = []

    def _fake_api(cfg):
        api = FakeDroneAPI(video_stream=FakeVideoStream())
        apis.append(api)
        return api

    monkeypatch.setattr(fmain, "_make_shared_pyhulax_api", _fake_api)
    monkeypatch.setattr(fmain, "_build_detector",
                        lambda cfg, bus, slog, run_dir, csv_health=None: None)
    monkeypatch.setattr(
        pf, "_default_discover",
        lambda plane_ids, timeout_s: {p: f"10.0.0.{p}" for p in plane_ids})
    monkeypatch.setattr(pf, "_stdin_go_reader", lambda: "GO")
    # **kw absorbs marker_dict / aruco_detector_params / save_dir.
    monkeypatch.setattr(aruco, "make_marker_detector",
                        lambda backend, **kw: (lambda frame, source_id: []))
    return apis


def _run_real_mission(tmp_path, monkeypatch):
    import finals.main as fmain

    apis = _wire_fake_real_fleet(monkeypatch)
    monkeypatch.chdir(tmp_path)
    code = fmain.main(["--profile", "real", "--i-know-this-arms-real-drones",
                       "--phases", "takeoff_demo", "--budget", "30"])
    return code, apis


def test_real_profile_mission_runs_to_done(tmp_path, monkeypatch):
    """main(--profile real) flies the WHOLE composition to a clean finish: P0-P10
    (operator GO included) passes, the orchestrator runs every landing_real.json
    drone to DONE, and run_end carries exit 0 — over faked pyhulax/Dola, no torch."""
    pytest.importorskip("cv2")
    code, apis = _run_real_mission(tmp_path, monkeypatch)

    assert code == 0
    # landing_real.json is the 3-drone fleet; one shared api was built per drone.
    assert len(apis) == 3

    rd = _only_run_dir(tmp_path)
    evs = _mission_events(rd)
    kinds = [e["event"] for e in evs]
    for expected in ("preflight", "run_start", "phase_enter", "phase_done",
                     "agent_done", "run_end"):
        assert expected in kinds, f"missing {expected!r} in mission.jsonl"
    run_end = [e for e in evs if e["event"] == "run_end"][-1]
    assert run_end["data"]["exit_code"] == 0

    hb = _heartbeat(rd)
    assert hb["drones"], "heartbeat carried no drones"
    assert all(d["state"] == "DONE" for d in hb["drones"].values()), (
        f"not all drones DONE: {hb['drones']}")


def test_real_mission_runs_full_p0_p10_gate(tmp_path, monkeypatch):
    """Every preflight gate P0..P10 emitted a result and none FAILED — the
    mission path (unlike --preflight-only) really executes P10 operator-GO, here
    authorized by the injected reader. Guards against a gate being skipped or
    silently demoted when the real composition arms."""
    pytest.importorskip("cv2")
    code, _ = _run_real_mission(tmp_path, monkeypatch)
    assert code == 0

    rd = _only_run_dir(tmp_path)
    evs = _mission_events(rd)
    gate_ids = {e["data"].get("id") for e in evs
                if e["event"] == "preflight_gate"}
    for gid in ("P0", "P1", "P2", "P3", "P4", "P5", "P6", "P8", "P9", "P10"):
        assert gid in gate_ids, f"preflight gate {gid} never ran (have {gate_ids})"
    failed = [e["data"].get("id") for e in evs
              if e["event"] == "preflight_gate"
              and e["data"].get("ok") is False
              and e["data"].get("critical") is True]
    assert not failed, f"critical preflight gates FAILED in a real mission: {failed}"


def test_real_mission_shares_one_link_for_flight_and_video(tmp_path, monkeypatch):
    """The same-link invariant (main._build_agents: ONE pyhulax DroneAPI per
    drone feeds BOTH the flight adapter and the video source) holds through a
    flown mission, not just structurally: each captured FakeDroneAPI recorded
    BOTH flight commands (takeoff + land) AND create_video_stream on the SAME
    object."""
    pytest.importorskip("cv2")
    code, apis = _run_real_mission(tmp_path, monkeypatch)
    assert code == 0
    assert apis, "no shared api was ever built"

    for api in apis:
        names = [c[0] for c in api.calls]
        assert "create_video_stream" in names, (
            f"video never opened on this link: {names}")
        assert "takeoff" in names and "land" in names, (
            f"flight commands did not flow on the SAME link as video: {names}")
        assert "connect" in names, f"preflight P4 never connected: {names}"
