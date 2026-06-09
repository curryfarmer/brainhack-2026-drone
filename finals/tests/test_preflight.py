"""finals.preflight — the P0-P10 gate, tested WITHOUT pyhulax or cv2.

The fleet is doubles all the way down: one FakeDroneAPI (+FakeVideoStream) per
drone, SHARED by a real PyhulaxAdapter and a real PyhulaxVideoSource (the
same-link invariant), wrapped in a tiny FakeAgent that exposes the exact surface
preflight reaches (`._adapter`, `.drone_id`). Discovery is injected (no Dola
UDP) and the marker detector is injected (no cv2). Each gate is tripped by
scripting one drone's fake; the happy path proves the mission-vs-bench teardown
contract and the persisted preflight.json. Session: S10.
"""
from __future__ import annotations

import asyncio
import json
import os
import time

import pytest

from finals.config import (DetectorConfig, DroneConfig, FinalsConfig,
                           GuardsConfig)
from finals.errors import PreflightError
from finals.events import EventLog, read_events
from finals.flight.pyhulax_adapter import (CommandRejected,
                                           DroneConnectionError, FakeDroneAPI,
                                           PyhulaxAdapter)
from finals.preflight import run_preflight
from finals.vision.pyhulax_video import FakeVideoStream, PyhulaxVideoSource

_GATE_IDS = ["P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7", "P8", "P9", "P10"]


# ============================================================
# Doubles + builders
# ============================================================
class FakeAgent:
    """The surface preflight reaches: a flight adapter + a drone_id (the real
    DroneAgent has both; nothing else is touched here)."""

    def __init__(self, drone_id: str, adapter) -> None:
        self.drone_id = drone_id
        self._adapter = adapter


def _no_marker(frame, source_id):
    return []


def fake_discover(plane_ids, timeout_s):
    return {p: f"10.0.0.{p}" for p in plane_ids}


def two_drones():
    return [
        DroneConfig(id="alpha", plane_id=1, led_rgb=(255, 0, 0),
                    altitude_band_m=1.2, phases=["takeoff_demo"]),
        DroneConfig(id="bravo", plane_id=2, led_rgb=(0, 255, 0),
                    altitude_band_m=1.7, phases=["takeoff_demo"]),
    ]


def one_drone():
    return [DroneConfig(id="alpha", plane_id=1, led_rgb=(255, 0, 0),
                        altitude_band_m=1.2, phases=["takeoff_demo"])]


def make_cfg(*, drones=None, profile="bench", min_battery_pct=20.0,
             use_uwb=False) -> FinalsConfig:
    drones = two_drones() if drones is None else drones
    return FinalsConfig(
        profile=profile,
        flight_backend=("bench" if profile == "bench" else "pyhulax"),
        frame_backend="pyhulax", video_channel_order="rgb",
        detector=DetectorConfig(), drones=drones, guards=GuardsConfig(),
        min_battery_pct=min_battery_pct, use_uwb=use_uwb)


def make_fleet(cfg, *, apis=None):
    """One shared FakeDroneAPI per drone -> a PyhulaxAdapter (ip unset; P3
    applies it) wrapped in a FakeAgent + a PyhulaxVideoSource on the SAME api.
    apis lets a test script a specific drone's fake (low battery, connect
    failure, error stream, ...)."""
    agents, sources = [], []
    for i, d in enumerate(cfg.drones):
        api = apis[i] if apis else FakeDroneAPI(video_stream=FakeVideoStream())
        adapter = PyhulaxAdapter(d.id, api=api)
        agents.append(FakeAgent(d.id, adapter))
        sources.append(PyhulaxVideoSource(
            d.id, api, video_channel_order=cfg.video_channel_order,
            stale_s=cfg.guards.video_stale_s))
    return agents, sources


def run(cfg, agents, sources, **kw):
    kw.setdefault("discover_fn", fake_discover)
    kw.setdefault("marker_detector", _no_marker)
    return asyncio.run(run_preflight(cfg.profile, agents, cfg,
                                     sources=sources, **kw))


def teardown(agents, sources):
    for s in sources:
        s.stop()
    for a in agents:
        asyncio.run(a._adapter.disconnect())


# ============================================================
# Happy path (preflight-only): every gate runs, then teardown
# ============================================================
def test_all_gates_pass_and_preflight_only_tears_down():
    cfg = make_cfg()
    agents, sources = make_fleet(cfg)
    results = run(cfg, agents, sources, preflight_only=True)

    assert [r.id for r in results] == _GATE_IDS
    assert all(r.ok or not r.critical for r in results), \
        [(r.id, r.detail) for r in results if not r.ok and r.critical]
    # P7 is the ONE advisory gate (a slow laptop WARNs, never blocks).
    assert next(r for r in results if r.id == "P7").critical is False
    # P10 skipped under --preflight-only (it authorizes flight; this never flies)
    p10 = next(r for r in results if r.id == "P10")
    assert p10.ok and p10.data.get("skipped") is True
    # bench tool leaves the fleet IDLE: disconnected + video stopped.
    for a in agents:
        assert a._adapter._connected is False
    for s in sources:
        assert s._stopped is True


def test_p8_skipped_when_uwb_unused():
    cfg = make_cfg()
    agents, sources = make_fleet(cfg)
    results = run(cfg, agents, sources, preflight_only=True)
    p8 = next(r for r in results if r.id == "P8")
    assert p8.ok and p8.data.get("skipped") is True


# ============================================================
# Mission path (P10 GO): link LEFT UP + artifacts persisted
# ============================================================
def test_mission_path_leaves_link_up_and_persists(tmp_path):
    cfg = make_cfg()
    agents, sources = make_fleet(cfg)
    run_dir = str(tmp_path)
    try:
        with EventLog(run_dir) as events:
            results = run(cfg, agents, sources, events=events, run_dir=run_dir,
                          confirm_fn=lambda: "GO", preflight_only=False)
        assert all(r.ok or not r.critical for r in results)
        # The mission path keeps adapters CONNECTED + sources STARTED for _amain.
        for a in agents:
            assert a._adapter._connected is True
        for s in sources:
            assert s.healthy is True
        with open(os.path.join(run_dir, "preflight.json"), encoding="utf-8") as f:
            saved = json.load(f)
        assert [r["id"] for r in saved] == _GATE_IDS
        evs = list(read_events(os.path.join(run_dir, "mission.jsonl")))
        assert sum(1 for e in evs if e["event"] == "preflight_gate") == 11
        summary = [e for e in evs if e["event"] == "preflight"]
        assert len(summary) == 1 and summary[0]["data"]["status"] == "passed"
    finally:
        teardown(agents, sources)


def test_preflight_connect_then_agent_connect_is_single_handshake():
    """The S9-deferred ordering proof: preflight P4 connects and LEAVES the link
    up; the agent's later run() connect() (idempotent) must NOT re-handshake."""
    cfg = make_cfg(drones=one_drone())
    api = FakeDroneAPI(video_stream=FakeVideoStream())
    agents, sources = make_fleet(cfg, apis=[api])
    try:
        run(cfg, agents, sources, confirm_fn=lambda: "GO", preflight_only=False)
        asyncio.run(agents[0]._adapter.connect())     # the agent's connect()
        assert [c[0] for c in api.calls].count("connect") == 1
    finally:
        teardown(agents, sources)


# ============================================================
# Per-gate trips (first CRITICAL failure -> teardown + PreflightError)
# ============================================================
def test_run_preflight_rejects_non_bench_real():
    cfg = make_cfg(profile="mock")
    agents, sources = make_fleet(cfg)
    with pytest.raises(PreflightError, match="bench/real"):
        asyncio.run(run_preflight("mock", agents, cfg, sources=sources))


def test_p0_trips_on_duplicate_plane_ids():
    cfg = make_cfg(drones=[
        DroneConfig(id="alpha", plane_id=1, altitude_band_m=1.2,
                    phases=["takeoff_demo"]),
        DroneConfig(id="bravo", plane_id=1, altitude_band_m=1.7,
                    phases=["takeoff_demo"]),
    ])
    agents, sources = make_fleet(cfg)
    with pytest.raises(PreflightError, match="P0"):
        run(cfg, agents, sources, preflight_only=True)


def test_p1_trips_on_unwritable_run_dir(tmp_path):
    cfg = make_cfg(drones=one_drone())
    agents, sources = make_fleet(cfg)
    missing = str(tmp_path / "does_not_exist")      # open(...) -> FileNotFound
    with pytest.raises(PreflightError, match="P1"):
        run(cfg, agents, sources, run_dir=missing, preflight_only=True)


def test_p3_trips_when_discovery_misses_a_plane():
    cfg = make_cfg()
    agents, sources = make_fleet(cfg)

    def bad_discover(plane_ids, timeout_s):
        raise PreflightError("plane_id 2 not found on the network")

    with pytest.raises(PreflightError, match="P3"):
        run(cfg, agents, sources, discover_fn=bad_discover, preflight_only=True)


def test_p4_trips_when_a_drone_will_not_connect():
    cfg = make_cfg()
    bad = FakeDroneAPI(video_stream=FakeVideoStream(),
                       fail_on={"connect": DroneConnectionError,
                                "robust_connect": DroneConnectionError})
    good = FakeDroneAPI(video_stream=FakeVideoStream())
    agents, sources = make_fleet(cfg, apis=[bad, good])
    with pytest.raises(PreflightError, match="P4"):
        run(cfg, agents, sources, preflight_only=True)
    assert agents[0]._adapter._connected is False    # safed down


def test_p5_trips_on_low_battery():
    cfg = make_cfg(min_battery_pct=50.0)
    low = FakeDroneAPI(video_stream=FakeVideoStream(), battery_pct=10.0)
    good = FakeDroneAPI(video_stream=FakeVideoStream(), battery_pct=90.0)
    agents, sources = make_fleet(cfg, apis=[low, good])
    with pytest.raises(PreflightError, match="P5"):
        run(cfg, agents, sources, preflight_only=True)


def test_p5_trips_when_not_on_ground():
    cfg = make_cfg(drones=one_drone())
    api = FakeDroneAPI(video_stream=FakeVideoStream(), altitude_cm=300.0)
    agents, sources = make_fleet(cfg, apis=[api])
    with pytest.raises(PreflightError, match="P5"):
        run(cfg, agents, sources, preflight_only=True)


def test_p6_trips_when_video_errors():
    cfg = make_cfg(drones=one_drone())
    stream = FakeVideoStream()
    stream.go_error()                               # ERROR before first frame
    api = FakeDroneAPI(video_stream=stream)
    agents, sources = make_fleet(cfg, apis=[api])
    with pytest.raises(PreflightError, match="P6"):
        run(cfg, agents, sources, preflight_only=True)


def test_p8_trips_when_uwb_enabled_without_port():
    cfg = make_cfg(drones=one_drone(), use_uwb=True)   # no uwb_serial_port
    agents, sources = make_fleet(cfg)
    with pytest.raises(PreflightError, match="P8"):
        run(cfg, agents, sources, preflight_only=True)


def test_p9_trips_when_led_set_fails():
    cfg = make_cfg(drones=one_drone())
    api = FakeDroneAPI(video_stream=FakeVideoStream(),
                       fail_on={"set_led": CommandRejected})
    agents, sources = make_fleet(cfg, apis=[api])
    with pytest.raises(PreflightError, match="P9"):
        run(cfg, agents, sources, preflight_only=True)


def test_p10_denies_on_wrong_answer():
    cfg = make_cfg(drones=one_drone())
    agents, sources = make_fleet(cfg)
    with pytest.raises(PreflightError, match="P10"):
        run(cfg, agents, sources, confirm_fn=lambda: "yes", preflight_only=False)
    assert agents[0]._adapter._connected is False    # default-deny safed down


def test_p10_denies_on_timeout():
    cfg = make_cfg(drones=one_drone())
    agents, sources = make_fleet(cfg)

    def slow_operator():
        time.sleep(1.0)
        return "GO"

    with pytest.raises(PreflightError, match="P10"):
        run(cfg, agents, sources, confirm_fn=slow_operator, go_timeout_s=0.2,
            preflight_only=False)
