"""flight_monitor.py tests.

Two tiers:
- PURE (bare venv, no cv2/numpy/SDK): the 2 Hz formatting, the depth reduction,
  the phase assembly (incl. the lawnmower trailing-Land drop), and the async
  flight runner against a stub adapter (completion, abort, budget wall, telemetry
  failure, action-failure -> phase abort, and the always-on emergency_land
  safe-down).
- cv2/numpy-GATED e2e: `main(--mock --headless)` drives BOTH test profiles end to
  end on FakeDroneAPI + a synthetic DICT_7X7_1000 marker frame + fake depth, and
  we assert the mission.jsonl carries the 2 Hz readback, the ArUco broadcast, the
  camera-tilt probe verdict, and that scan_land actually reaches the land_on_pad
  phase. Mirrors the live_view --fake e2e (no hardware).
"""
import asyncio
import json
import os
import threading

import pytest

import finals.tools.flight_monitor as fm
from finals.errors import FlightError
from finals.types import Hover, Land, Move, Rotate, Takeoff
from finals.vision.depth import DepthFrame


# ===========================================================================
# PURE: 2 Hz formatting
# ===========================================================================
def test_format_line_object_ahead():
    line = fm.format_monitor_line(alt_m=0.0, yaw_deg=12.3, is_flying=True,
                                  depth_m=0.6)
    assert "OBJECT AHEAD 0.60m" in line
    assert "FLYING" in line


def test_format_line_clear_and_na():
    assert "clear 2.40m" in fm.format_monitor_line(
        alt_m=1.2, yaw_deg=-5.0, is_flying=True, depth_m=2.4)
    na = fm.format_monitor_line(alt_m=None, yaw_deg=None, is_flying=False,
                                depth_m=None)
    assert "depth n/a" in na and "ground" in na and "n/a" in na


def test_format_line_threshold_is_inclusive():
    # exactly at the threshold counts as an object ahead (<=).
    assert "OBJECT AHEAD" in fm.format_monitor_line(
        alt_m=0.0, yaw_deg=0.0, is_flying=True, depth_m=1.0, threshold_m=1.0)
    assert "clear" in fm.format_monitor_line(
        alt_m=0.0, yaw_deg=0.0, is_flying=True, depth_m=1.01, threshold_m=1.0)


# ===========================================================================
# PURE: depth reduction
# ===========================================================================
def test_nearest_object_m_center_box():
    grid = [[5.0, 5.0, 5.0, 5.0],
            [5.0, 0.6, 0.6, 5.0],
            [5.0, 5.0, 5.0, 5.0]]
    df = DepthFrame(grid, ts=0.0, source_id="t", width=4, height=3)
    assert fm.nearest_object_m(df) == pytest.approx(0.6)


def test_nearest_object_m_ignores_zero_and_none_frame():
    # a 0.0 cell is a no-return (distance_at -> None) and must not read as 0 m.
    grid = [[0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]]
    df = DepthFrame(grid, ts=0.0, source_id="t", width=4, height=3)
    assert fm.nearest_object_m(df) is None
    assert fm.nearest_object_m(None) is None


# ===========================================================================
# PURE: phase assembly
# ===========================================================================
def test_build_takeoff_land():
    phases = fm.build_test_phases("takeoff_land")
    assert len(phases) == 1
    plan = [type(a).__name__ for a in phases[0]._plan]
    assert plan == ["Takeoff", "Hover", "Land"]


def test_build_scan_land_drops_trailing_land_and_sets_marker_ids():
    phases = fm.build_test_phases("scan_land", marker_ids=[11, 45])
    assert [type(p).__name__ for p in phases] == ["OpenLoopLawnmower", "LandOnPad"]
    lawnmower = phases[0]
    # the chain-into-land_on_pad invariant: NO trailing Land (stays airborne).
    assert not isinstance(lawnmower._plan[-1], Land)
    assert not any(isinstance(a, Land) for a in lawnmower._plan)


def test_build_unknown_test_raises():
    with pytest.raises(fm.FlightMonitorError):
        fm.build_test_phases("orbit")


def test_lawnmower_no_land_pops_only_the_trailing_land():
    lm = fm._lawnmower_no_land(lanes=2, leg_cm=200, lane_cm=100)
    assert not any(isinstance(a, Land) for a in lm._plan)
    assert isinstance(lm._plan[0], Takeoff)


def test_action_fields():
    assert fm._action_fields(Takeoff(height_cm=90)) == {"height_cm": 90}
    assert fm._action_fields(Move(direction=__import__(
        "finals.types", fromlist=["Direction"]).Direction.FORWARD,
        distance_cm=300))["distance_cm"] == 300
    assert fm._action_fields(Rotate(angle_deg=90.0)) == {"angle_deg": 90.0}
    assert fm._action_fields(Hover(duration_s=2.0)) == {"duration_s": 2.0}
    assert fm._action_fields(Land()) == {}


# ===========================================================================
# PURE: the async flight runner (stub adapter — no SDK)
# ===========================================================================
class _StubAdapter:
    """Minimal FlightAdapter surface for run_phases: async commands + a sync
    telemetry(). is_flying flips on takeoff/land like the real link."""

    def __init__(self):
        self.calls = []
        self._flying = False
        self.emergency_called = False

    async def takeoff(self, height_cm=80, timeout_s=30.0):
        self.calls.append(("takeoff", height_cm))
        self._flying = True

    async def land(self, timeout_s=30.0):
        self.calls.append(("land",))
        self._flying = False

    async def move(self, direction, distance_cm, timeout_s=15.0):
        self.calls.append(("move", int(direction), distance_cm))

    async def rotate(self, angle_deg, timeout_s=15.0):
        self.calls.append(("rotate", angle_deg))

    async def hover(self, duration_s):
        self.calls.append(("hover", duration_s))

    async def emergency_land(self):
        self.emergency_called = True

    def telemetry(self):
        from finals.types import Telemetry
        return Telemetry(ts=0.0, is_flying=self._flying, altitude_m=0.0,
                         yaw_deg=0.0)


def test_run_phases_takeoff_land_completes_and_safes_down():
    ad = _StubAdapter()
    summary = asyncio.run(fm.run_phases(
        ad, fm.build_test_phases("takeoff_land"), "t"))
    assert summary["completed"] is True
    assert summary["aborted"] is False
    assert ("takeoff", 80) in ad.calls
    assert ("hover", 2.0) in ad.calls
    assert ("land",) in ad.calls
    assert ad.emergency_called          # finally safe-down ran


def test_run_phases_aborts_on_event_before_takeoff():
    ad = _StubAdapter()
    ev = threading.Event()
    ev.set()
    summary = asyncio.run(fm.run_phases(
        ad, fm.build_test_phases("takeoff_land"), "t", abort_event=ev))
    assert summary["aborted"] is True
    assert ("takeoff", 80) not in ad.calls   # never started
    assert ad.emergency_called


def test_run_phases_budget_wall_trips():
    ad = _StubAdapter()
    summary = asyncio.run(fm.run_phases(
        ad, fm.build_test_phases("takeoff_land"), "t", mission_budget_s=-1.0))
    assert summary["completed"] is False
    assert "budget" in (summary["error"] or "")
    assert ad.emergency_called


def test_run_phases_telemetry_failure_returns_loud():
    class _Dead(_StubAdapter):
        def telemetry(self):
            raise FlightError("link dead")

    ad = _Dead()
    summary = asyncio.run(fm.run_phases(
        ad, fm.build_test_phases("takeoff_land"), "t"))
    assert summary["completed"] is False
    assert "telemetry" in (summary["error"] or "")
    assert ad.emergency_called


def test_run_phases_action_failure_aborts_the_phase():
    class _BadTakeoff(_StubAdapter):
        async def takeoff(self, height_cm=80, timeout_s=30.0):
            raise FlightError("takeoff rejected")

    ad = _BadTakeoff()
    summary = asyncio.run(fm.run_phases(
        ad, fm.build_test_phases("takeoff_land"), "t"))
    # the runner feeds the failure back; _FixedPlanPhase Aborts next step.
    assert summary["aborted"] is True
    assert ad.emergency_called


# ===========================================================================
# cv2/numpy-GATED: --mock end-to-end (FakeDroneAPI + synthetic marker + fake depth)
# ===========================================================================
def _events(run_dir):
    path = os.path.join(run_dir, "mission.jsonl")
    out = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def _event_names(run_dir):
    return {e["event"] for e in _events(run_dir)}


def test_mock_e2e_takeoff_land(tmp_path):
    pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    rc = fm.main(["--test", "takeoff_land", "--mock", "--headless",
                  "--no-yolo", "--no-abort-key", "--duration-s", "4",
                  "--camera-tilt-deg", "20", "--out", str(tmp_path)])
    assert rc == 0
    names = _event_names(str(tmp_path))
    assert "monitor_2hz" in names          # 2 Hz readback fired
    assert "broadcast" in names            # synthetic marker decoded + published
    assert "camera_tilt_probe" in names    # the tilt question was answered
    assert "phase_done" in names
    # the tilt probe must report NEGATIVE on FakeDroneAPI (no set_camera_angle).
    probe = [e for e in _events(str(tmp_path)) if e["event"] == "camera_tilt_probe"]
    assert probe and probe[0]["data"]["supported"] is False


def test_mock_e2e_scan_land_reaches_landing(tmp_path):
    pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    rc = fm.main(["--test", "scan_land", "--mock", "--headless",
                  "--no-yolo", "--no-abort-key", "--mission-budget-s", "30",
                  "--duration-s", "0", "--fake-marker-id", "11",
                  "--out", str(tmp_path)])
    assert rc in (0, 1)                     # bounded: completed, or budget-walled
    evs = _events(str(tmp_path))
    phases_entered = {e["data"].get("phase") for e in evs
                      if e["event"] == "phase_enter"}
    assert "lawnmower" in phases_entered
    assert "land_on_pad" in phases_entered  # the full chain was stepped
    assert "broadcast" in {e["event"] for e in evs}
