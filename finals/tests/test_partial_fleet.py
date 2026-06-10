"""Degraded-fleet fallback (cfg.allow_partial_fleet) — the "default to fewer
drones if the full swarm can't be brought up" path.

Three layers:
1. config — the new knobs validate (min_drones floor, profile gate).
2. preflight partial gates — a failing drone is DROPPED (not a swarm abort);
   the gate fails critically ONLY when survivors fall below min_drones.
3. end-to-end via main(--profile real --config landing_real.json) — one drone
   never discovered -> the OTHER TWO fly the mission to DONE, exit 0, and the
   casualty is logged (drone_dropped / fleet_degraded / flying_degraded_fleet).

Deterministic: faked pyhulax/Dola/cv2/torch seams, injected operator GO, no
wall-clock races. Mirrors the seam style of test_real_wiring_e2e.

Run: python -m pytest finals/tests/test_partial_fleet.py -q -p no:randomly
"""
from __future__ import annotations

import asyncio
import json
import os
from types import SimpleNamespace

import pytest

from finals.config import load_config
from finals.errors import ConfigError, FlightError, PreflightError
from finals.events import read_events

_CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs")
_LANDING_REAL = os.path.join(_CONFIG_DIR, "landing_real.json")
_CONVOY_REAL = os.path.join(_CONFIG_DIR, "convoy_real.json")


# ============================================================
# 1. Config validation
# ============================================================
def test_landing_real_ships_degraded_fallback():
    cfg = load_config(_LANDING_REAL)
    assert cfg.allow_partial_fleet is True
    assert cfg.min_drones == 1


def test_convoy_real_ships_degraded_fallback():
    cfg = load_config(os.path.join(_CONFIG_DIR, "convoy_real.json"))
    assert cfg.allow_partial_fleet is True and cfg.min_drones == 1


def test_strict_configs_default_off():
    for name in ("bench.json", "real.json", "mock.json"):
        cfg = load_config(os.path.join(_CONFIG_DIR, name))
        assert cfg.allow_partial_fleet is False


def _write_cfg(tmp_path, **patch):
    base = json.load(open(_LANDING_REAL, encoding="utf-8"))
    base.update(patch)
    p = tmp_path / "cfg.json"
    p.write_text(json.dumps(base), encoding="utf-8")
    return str(p)


def test_min_drones_below_one_rejected(tmp_path):
    with pytest.raises(ConfigError, match="min_drones"):
        load_config(_write_cfg(tmp_path, min_drones=0))


def test_min_drones_above_fleet_rejected(tmp_path):
    # landing_real has 3 drones; a floor of 4 can never be met.
    with pytest.raises(ConfigError, match="min_drones"):
        load_config(_write_cfg(tmp_path, min_drones=4))


def test_allow_partial_rejected_on_software_profile(tmp_path):
    # mock has no preflight gate to degrade.
    base = json.load(open(os.path.join(_CONFIG_DIR, "mock.json"), encoding="utf-8"))
    base["allow_partial_fleet"] = True
    p = tmp_path / "m.json"
    p.write_text(json.dumps(base), encoding="utf-8")
    with pytest.raises(ConfigError, match="allow_partial_fleet"):
        load_config(str(p))


# ============================================================
# 1b. Preflight P0 separation — the bands-OR-sectors contract
# ============================================================
def test_p0_accepts_sector_separation_no_bands():
    """Regression: landing_real deconflicts by sector_deg (NO altitude bands —
    illegal under the ~1.1 m ceiling). P0 must accept sectors-on-every-drone the
    same way config.py does at load, not hard-require distinct bands."""
    from finals.preflight import _p0_config
    cfg = load_config(_LANDING_REAL)
    assert all(d.altitude_band_m is None for d in cfg.drones)   # no bands
    assert all(d.sector_deg is not None for d in cfg.drones)    # sectors instead
    ok, detail, _data = asyncio.run(_p0_config(cfg))
    assert ok is True, detail


def test_p0_still_rejects_no_separation(tmp_path):
    """A multi-drone real config with NEITHER distinct bands NOR sectors on every
    drone still fails P0 (the silent-no-separation bug class stays guarded). This
    can't pass config.py either, so build the cfg then strip separation."""
    from finals.preflight import _p0_config
    cfg = load_config(_CONVOY_REAL)                 # has distinct bands
    for d in cfg.drones:
        d.altitude_band_m = None                    # remove the only separation
        d.sector_deg = None
    ok, detail, _data = asyncio.run(_p0_config(cfg))
    assert ok is False and "separation" in detail


# ============================================================
# 2. Preflight partial gates (unit) — stubs, no SDK
# ============================================================
class _StubAgent:
    def __init__(self, drone_id, adapter):
        self.drone_id = drone_id
        self._adapter = adapter


class _StubTelem:
    def __init__(self, battery_pct, altitude_m=0.0, age=0.0):
        self.battery_pct = battery_pct
        self.altitude_m = altitude_m
        self._age = age

    def age_s(self, now):
        return self._age


class _StubAdapter:
    """Minimal FlightAdapter surface the partial gates touch."""
    def __init__(self, *, telem=None, telem_exc=None):
        self._telem = telem
        self._telem_exc = telem_exc
        self.disconnected = 0

    def telemetry(self):
        if self._telem_exc is not None:
            raise self._telem_exc
        return self._telem

    async def disconnect(self):
        self.disconnected += 1


class _StubSource:
    def __init__(self, source_id, *, healthy=True, start_exc=None):
        self.source_id = source_id
        self._healthy = healthy
        self._start_exc = start_exc
        self.started = 0
        self.stopped = 0

    def start(self, timeout_s=None):
        self.started += 1
        if self._start_exc is not None:
            raise self._start_exc

    @property
    def healthy(self):
        return self._healthy

    def stop(self):
        self.stopped += 1


def test_p4_partial_drops_connect_failure_keeps_survivors():
    """A drone that fails BOTH connect + robust_connect is dropped; the survivors
    connect and (min_drones=1) the gate passes."""
    from finals.preflight import _p4_connect_partial
    from finals.flight.pyhulax_adapter import (FakeDroneAPI, PyhulaxAdapter,
                                               DroneConnectionError, NotReady)
    good1 = PyhulaxAdapter("alpha", ip="1.2.3.4", api=FakeDroneAPI())
    good2 = PyhulaxAdapter("bravo", ip="1.2.3.5", api=FakeDroneAPI())
    bad = PyhulaxAdapter("charlie", ip="1.2.3.6", api=FakeDroneAPI(
        fail_on={"connect": DroneConnectionError, "robust_connect": NotReady}))
    agents = [_StubAgent("alpha", good1), _StubAgent("bravo", good2),
              _StubAgent("charlie", bad)]
    cfg = SimpleNamespace(command_timeout_s=5.0)
    dropped: set = set()
    ok, detail, data = asyncio.run(
        _p4_connect_partial(cfg, agents, [], dropped, 1, None))
    assert ok is True
    assert dropped == {"charlie"}
    assert set(data["connected"]) == {"alpha", "bravo"}


def test_p4_partial_fails_critically_below_floor():
    """min_drones=3 but one drone can't connect -> the gate fails (survivors < floor)."""
    from finals.preflight import _p4_connect_partial
    from finals.flight.pyhulax_adapter import (FakeDroneAPI, PyhulaxAdapter,
                                               DroneConnectionError, NotReady)
    good = PyhulaxAdapter("alpha", ip="1.2.3.4", api=FakeDroneAPI())
    good2 = PyhulaxAdapter("bravo", ip="1.2.3.5", api=FakeDroneAPI())
    bad = PyhulaxAdapter("charlie", ip="1.2.3.6", api=FakeDroneAPI(
        fail_on={"connect": DroneConnectionError, "robust_connect": NotReady}))
    agents = [_StubAgent("alpha", good), _StubAgent("bravo", good2),
              _StubAgent("charlie", bad)]
    dropped: set = set()
    ok, _detail, _data = asyncio.run(_p4_connect_partial(
        SimpleNamespace(command_timeout_s=5.0), agents, [], dropped, 3, None))
    assert ok is False                      # _gate maps a critical False -> abort
    assert dropped == {"charlie"}


def test_p5_partial_drops_low_battery_drone():
    from finals.preflight import _p5_telemetry_partial
    cfg = SimpleNamespace(min_battery_pct=20.0,
                          guards=SimpleNamespace(telemetry_stale_s=5.0))
    healthy = _StubAgent("alpha", _StubAdapter(telem=_StubTelem(80.0)))
    flat = _StubAgent("bravo", _StubAdapter(telem=_StubTelem(5.0)))
    dropped: set = set()
    ok, _detail, data = asyncio.run(
        _p5_telemetry_partial(cfg, [healthy, flat], [], dropped, 1, None))
    assert ok is True
    assert dropped == {"bravo"}
    assert flat._adapter.disconnected == 1   # the casualty was safed
    # The low reading is recorded (forensics) before the drop; the DROP is the
    # outcome under test, asserted via `dropped` above.
    assert data["bravo"]["battery_pct"] == 5.0


def test_p5_partial_drops_telemetry_exception_drone():
    from finals.preflight import _p5_telemetry_partial
    cfg = SimpleNamespace(min_battery_pct=20.0,
                          guards=SimpleNamespace(telemetry_stale_s=5.0))
    healthy = _StubAgent("alpha", _StubAdapter(telem=_StubTelem(90.0)))
    dead = _StubAgent("bravo", _StubAdapter(
        telem_exc=FlightError("link dropped")))
    dropped: set = set()
    ok, _detail, _data = asyncio.run(
        _p5_telemetry_partial(cfg, [healthy, dead], [], dropped, 1, None))
    assert ok is True and dropped == {"bravo"}


def test_p6_partial_drops_unhealthy_and_failed_video():
    from finals.preflight import _p6_video_partial
    s_ok = _StubSource("alpha")
    s_unhealthy = _StubSource("bravo", healthy=False)
    from finals.errors import SensorTimeout
    s_fail = _StubSource("charlie", start_exc=SensorTimeout("no frames"))
    agents = [_StubAgent("alpha", _StubAdapter()),
              _StubAgent("bravo", _StubAdapter()),
              _StubAgent("charlie", _StubAdapter())]
    started: list = []
    dropped: set = set()
    ok, _detail, data = asyncio.run(_p6_video_partial(
        [s_ok, s_unhealthy, s_fail], started, agents, dropped, 1, None))
    assert ok is True
    assert dropped == {"bravo", "charlie"}
    assert data["healthy"] == ["alpha"]
    assert s_unhealthy.stopped >= 1 and s_fail.started == 1


def test_p6_partial_skips_already_dropped():
    """A drone dropped at P3/P4/P5 is not re-touched at P6."""
    from finals.preflight import _p6_video_partial
    s_alpha = _StubSource("alpha")
    s_bravo = _StubSource("bravo")
    agents = [_StubAgent("alpha", _StubAdapter()),
              _StubAgent("bravo", _StubAdapter())]
    dropped = {"bravo"}
    ok, _detail, _data = asyncio.run(_p6_video_partial(
        [s_alpha, s_bravo], [], agents, dropped, 1, None))
    assert ok is True
    assert s_bravo.started == 0               # skipped — already dropped
    assert s_alpha.started == 1


# ============================================================
# 3. End-to-end through main(--profile real)
# ============================================================
def _mission_events(run_dir):
    return list(read_events(os.path.join(run_dir, "mission.jsonl")))


def _heartbeat(run_dir):
    with open(os.path.join(run_dir, "heartbeat.json"), encoding="utf-8") as f:
        return json.load(f)


def _only_run_dir(tmp_path):
    run_dirs = list((tmp_path / "runs_finals").iterdir())
    assert len(run_dirs) == 1, run_dirs
    return str(run_dirs[0])


def _wire_common(monkeypatch):
    """Fake every SDK seam EXCEPT discovery (each test scripts its own drops)."""
    import finals.main as fmain
    import finals.preflight as pf
    import finals.vision.aruco as aruco
    from finals.flight.pyhulax_adapter import FakeDroneAPI
    from finals.vision.pyhulax_video import FakeVideoStream

    monkeypatch.setattr(
        fmain, "_make_shared_pyhulax_api",
        lambda cfg: FakeDroneAPI(video_stream=FakeVideoStream()))
    monkeypatch.setattr(fmain, "_build_detector",
                        lambda cfg, bus, slog, run_dir, csv_health=None: None)
    monkeypatch.setattr(pf, "_stdin_go_reader", lambda: "GO")
    monkeypatch.setattr(aruco, "make_marker_detector",
                        lambda backend, **kw: (lambda frame, source_id: []))


def test_one_drone_undiscovered_flies_the_other_two(tmp_path, monkeypatch):
    """The headline: 1 of 3 drones never answers discovery -> the OTHER TWO fly
    the mission to DONE, exit 0, and the casualty is logged + absent from the
    flying fleet."""
    pytest.importorskip("cv2")
    import finals.main as fmain
    import finals.preflight as pf

    # The ACTUAL Challenge-2A landing mission (sector_deg deconfliction, no
    # bands) — proves degraded-fleet on the primary use case AND that P0 now
    # accepts sector separation.
    cfg = load_config(_LANDING_REAL)
    pids = [d.plane_id for d in cfg.drones]
    found = set(pids[:2])                      # plane_ids[2] never discovered
    dropped_id = cfg.drones[2].id

    _wire_common(monkeypatch)
    monkeypatch.setattr(
        pf, "_default_discover_partial",
        lambda plane_ids, timeout_s, min_count: {
            p: f"10.0.0.{p}" for p in plane_ids if p in found})

    monkeypatch.chdir(tmp_path)
    code = fmain.main(["--profile", "real", "--config", _LANDING_REAL,
                       "--i-know-this-arms-real-drones", "--no-detector",
                       "--phases", "takeoff_demo", "--budget", "120"])
    assert code == 0

    rd = _only_run_dir(tmp_path)
    hb = _heartbeat(rd)
    assert set(hb["drones"]) == {cfg.drones[0].id, cfg.drones[1].id}
    assert all(d["state"] == "DONE" for d in hb["drones"].values())
    assert dropped_id not in hb["drones"]      # the casualty never flew

    evs = _mission_events(rd)
    drops = [e for e in evs if e["event"] == "drone_dropped"]
    assert [e["data"]["drone"] for e in drops] == [dropped_id]
    degraded = [e for e in evs if e["event"] == "fleet_degraded"]
    assert degraded and degraded[-1]["data"]["dropped"] == [dropped_id]
    flying = [e for e in evs if e["event"] == "flying_degraded_fleet"]
    assert flying and dropped_id in flying[-1]["data"]["dropped"]


def test_all_drones_missing_aborts_below_floor(tmp_path, monkeypatch):
    """Zero discovered < min_drones=1 -> a hard abort (exit 3), NOT a 0-drone
    flight."""
    pytest.importorskip("cv2")
    import finals.main as fmain
    import finals.preflight as pf

    _wire_common(monkeypatch)
    monkeypatch.setattr(pf, "_default_discover_partial",
                        lambda plane_ids, timeout_s, min_count: {})
    monkeypatch.chdir(tmp_path)
    code = fmain.main(["--profile", "real", "--config", _CONVOY_REAL,
                       "--i-know-this-arms-real-drones", "--no-detector",
                       "--phases", "takeoff_demo", "--budget", "120"])
    assert code == 3                           # PreflightError -> exit 3


def test_strict_config_still_aborts_on_a_missing_drone(tmp_path, monkeypatch):
    """Regression: a config WITHOUT allow_partial_fleet (real.json) keeps the
    all-or-nothing contract — a missing drone aborts the whole swarm (exit 3),
    no drone_dropped / fleet_degraded events."""
    pytest.importorskip("cv2")
    import finals.main as fmain
    import finals.preflight as pf

    real_cfg = os.path.join(_CONFIG_DIR, "real.json")
    assert load_config(real_cfg).allow_partial_fleet is False

    _wire_common(monkeypatch)

    def _strict_missing(plane_ids, timeout_s):
        raise PreflightError("planes not found: [3] — found [1, 2]")
    monkeypatch.setattr(pf, "_default_discover", _strict_missing)

    monkeypatch.chdir(tmp_path)
    code = fmain.main(["--profile", "real", "--config", real_cfg,
                       "--i-know-this-arms-real-drones", "--no-detector",
                       "--phases", "takeoff_demo", "--budget", "30"])
    assert code == 3
    evs = _mission_events(_only_run_dir(tmp_path))
    kinds = {e["event"] for e in evs}
    assert "drone_dropped" not in kinds and "fleet_degraded" not in kinds
