"""Mechanical enforcement of the package conventions (see the plan / module_map):

1. No bare `except:` anywhere in finals/.
2. `except Exception` only in the whitelisted safety sites.
3. Any module that raises NotImplementedError must point at module_map.md
   (the stub convention: "session N — see finals/docs/module_map.md").
4. Pure modules never import an SDK at the top level (the seam discipline).
5. Every registered phase resolves; unknown names fail with the available list.

The remaining conventions (every while-loop bounded, no module-level mutable
globals, timeout_s on every blocking op) are enforced by review — they are
listed in finals/docs/module_map.md so every session re-reads them.
"""
from __future__ import annotations

import os
import re

import pytest

from finals.errors import ConfigError

FINALS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The ONLY files allowed to contain `except Exception` (always logged with
# traceback): SafetyController safe-down + guard-evaluation wrapper live in
# guards.py; the orchestrator top loop in orchestrator.py. Widening this list
# is a deliberate, reviewable act.
# S7 widening (reviewed): vision/detector.py — the vendored worker pool must
# survive ARBITRARY model/callback exceptions (ultralytics/torch/cv2 raise an
# open set; root Detector.py's worker died SILENTLY on them — the bug class
# this file fixes). Its two catch sites each log the full traceback and
# increment a loud counter; threading.excepthook is not covered by
# install_crash_hooks, so these guards are the only thing keeping worker
# death observable.
# S6/SIM-1 widening (reviewed, user-approved): flight/sitl_adapter.py — its
# never-raise contract paths cannot be honored by a typed tuple (a RuntimeError
# from a closing loop or a non-RpcError grpc surprise would escape mid-safe-
# down), and MAVSDK telemetry streams END SILENTLY when PX4 dies, so the
# stream-task wrapper must catch-anything to convert death into a loud
# dead-flag. Exactly three sites, each logging the full traceback:
# emergency_land per-step (the ABC names emergency_land the one sanctioned
# swallow in the flight stack), disconnect teardown, the stream wrapper.
# S9 widening (reviewed, user-approved): flight/pyhulax_adapter.py — same
# justification as sitl_adapter on the real backend. The blocking pyhulax SDK
# raises an OPEN set (CommandTimeout/Rejected/NotReady/LowBattery/
# DroneConnectionError + undocumented Wi-Fi-dropout surprises), so a typed
# tuple cannot honor the never-raise/never-silent paths. Exactly three sites,
# each logging the full traceback: the 2 Hz telemetry-poller tick (a poller
# dying silently is the mapping_drone.py bug class — it sets a loud dead-flag
# instead), emergency_land (the one sanctioned swallow in the flight stack),
# and disconnect teardown. flight/discovery.py and vision/pyhulax_video.py stay
# OFF this list by design — both are typed-only.
# flight-test widening (reviewed): tools/hula_smoke.py — the OFFLINE no-flight
# bring-up smoke must survive ARBITRARY per-drone/per-frame failures from the
# open error sets it touches (cv2 decode, ultralytics inference, pyhulax connect/
# telemetry, socket/Dola discovery) and keep diagnosing the rest of the fleet.
# Every catch logs the full traceback and issues NO flight command — it is a
# read-only probe, never in the flight path. Same justification as detector.py.
# dev-bench widening (reviewed): tools/live_view.py — the real-time CV feed
# visualiser is the SAME class as hula_smoke: a read-only, no-flight diagnostic
# that must survive the open pyhulax/cv2/ultralytics error set (connect, video
# start, per-frame decode/inference) plus the never-raise teardown (stream stop +
# disconnect). Five sites, each logging a full traceback via log.exc(); it issues
# NO flight command.
# flight-bench widening (reviewed): tools/flight_monitor.py — the props-off
# instrumented runner. THREE sanctioned sites: the flight WORKER thread (must not
# die silently — logs the full traceback), the camera-tilt PROBE (an open SDK
# error set on an optional method), and the never-raise emergency_land safe-down
# in run_phases' finally. Unlike hula_smoke/live_view it DOES issue flight
# commands, but only props-off behind the --props-off-confirmed gate.
EXCEPT_EXCEPTION_WHITELIST = {
    os.path.join("finals", "guards.py"),
    os.path.join("finals", "mission", "orchestrator.py"),
    os.path.join("finals", "vision", "detector.py"),
    os.path.join("finals", "flight", "sitl_adapter.py"),
    os.path.join("finals", "flight", "pyhulax_adapter.py"),
    os.path.join("finals", "tools", "hula_smoke.py"),
    os.path.join("finals", "tools", "live_view.py"),
    os.path.join("finals", "tools", "flight_monitor.py"),
}

# Modules that may import SDK/heavy-I/O packages at module top level.
# S7 notes: perception.py is REMOVED (it is deliberately pure — detectors
# arrive as injected callables; this scan now enforces that). video.py and
# detector.py keep their entries but in fact lazy-import cv2/ultralytics
# (main.py resolves every backend for --dry-run on SDK-less machines).
SDK_ALLOWED = {
    os.path.join("finals", "flight", "sitl_adapter.py"),     # mavsdk (via drone_control)
    os.path.join("finals", "flight", "pyhulax_adapter.py"),  # pyhulax
    os.path.join("finals", "vision", "video.py"),            # cv2 (ReplaySource)
    os.path.join("finals", "vision", "gazebo_video.py"),     # gz.transport13
    os.path.join("finals", "vision", "pyhulax_video.py"),    # pyhulax
    os.path.join("finals", "vision", "detector.py"),         # ultralytics, cv2
    os.path.join("finals", "vision", "aruco.py"),            # cv2
}

FORBIDDEN_SDK_ROOTS = {
    "pyhulax", "mavsdk", "gz", "ultralytics", "cv2", "serial", "rclpy",
    "pyrealsense2", "torch",
    # numpy is NOT installed in the bare dev venv (the suite must pass
    # without it) — a top-level numpy import in a pure module would break
    # that invariant silently. Whitelisted vision files keep their freedom;
    # pure modules use the TYPE_CHECKING pattern (types.py:29).
    "numpy",
}


def _python_files():
    for root, dirs, files in os.walk(FINALS_DIR):
        dirs[:] = [d for d in dirs if d not in ("__pycache__", "tests")]
        for name in files:
            if name.endswith(".py"):
                full = os.path.join(root, name)
                rel = os.path.relpath(full, os.path.dirname(FINALS_DIR))
                yield full, rel


def _source(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def test_no_bare_except():
    offenders = [rel for full, rel in _python_files()
                 if re.search(r"\bexcept\s*:", _source(full))]
    assert not offenders, f"bare `except:` found in: {offenders}"


def test_except_exception_only_in_whitelist():
    offenders = [
        rel for full, rel in _python_files()
        if re.search(r"\bexcept\s+(Exception|BaseException)\b", _source(full))
        and rel not in EXCEPT_EXCEPTION_WHITELIST
    ]
    assert not offenders, (
        f"`except Exception` outside the whitelist in: {offenders} — the only "
        f"sanctioned sites are {sorted(EXCEPT_EXCEPTION_WHITELIST)}"
    )


def test_stubs_point_at_module_map():
    offenders = []
    for full, rel in _python_files():
        src = _source(full)
        if "NotImplementedError" in src and "module_map.md" not in src:
            offenders.append(rel)
    assert not offenders, (
        f"NotImplementedError without a module_map.md session pointer in: {offenders}"
    )


def test_pure_modules_do_not_import_sdks():
    import_re = re.compile(r"^(?:import|from)\s+([A-Za-z_][A-Za-z0-9_]*)", re.MULTILINE)
    offenders = {}
    for full, rel in _python_files():
        if rel in SDK_ALLOWED:
            continue
        roots = set(import_re.findall(_source(full)))
        bad = roots & FORBIDDEN_SDK_ROOTS
        if bad:
            offenders[rel] = sorted(bad)
    assert not offenders, (
        f"SDK imports leaked into pure modules: {offenders} — backends import "
        f"SDKs inside their own module only (see SDK_ALLOWED)"
    )


def test_phase_registry_names():
    from finals.mission.phases import PHASE_REGISTRY, resolve_phase

    assert set(PHASE_REGISTRY) == {
        "takeoff_demo", "takeoff", "sentry_scan", "lawnmower", "track_convoy",
        "land_on_pad", "navigate",
    }
    with pytest.raises(ConfigError, match="takeoff_demo"):  # lists available names
        resolve_phase("no_such_phase")


def test_main_dry_run_all_profiles(repo_root, monkeypatch, capsys):
    """The S1 acceptance test: every shipped profile resolves end-to-end."""
    from finals.main import main

    monkeypatch.chdir(repo_root)
    for profile in ("mock", "sitl", "replay", "bench", "real"):
        argv = ["--profile", profile, "--dry-run"]
        if profile == "real":
            argv.append("--i-know-this-arms-real-drones")
        assert main(argv) == 0, f"--dry-run failed for profile {profile}"
        out = capsys.readouterr().out
        assert "RESOLVED PLAN" in out and profile in out


def test_main_real_profile_refuses_without_gate(repo_root, monkeypatch, capsys):
    from finals.main import main

    monkeypatch.chdir(repo_root)
    assert main(["--profile", "real", "--dry-run"]) == 2  # ConfigError exit code
    assert "i-know-this-arms-real-drones" in capsys.readouterr().err


# (test_main_no_drone_execution_points_at_s7 was deleted in S7: the no-drone
# replay runner is now real — its end-to-end coverage lives in
# tests/test_replay_e2e.py.)
