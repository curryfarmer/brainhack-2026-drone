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
EXCEPT_EXCEPTION_WHITELIST = {
    os.path.join("finals", "guards.py"),
    os.path.join("finals", "mission", "orchestrator.py"),
}

# Modules that may import SDK/heavy-I/O packages at module top level.
SDK_ALLOWED = {
    os.path.join("finals", "flight", "sitl_adapter.py"),     # mavsdk (via drone_control)
    os.path.join("finals", "flight", "pyhulax_adapter.py"),  # pyhulax
    os.path.join("finals", "vision", "video.py"),            # cv2 (ReplaySource)
    os.path.join("finals", "vision", "gazebo_video.py"),     # gz.transport13
    os.path.join("finals", "vision", "pyhulax_video.py"),    # pyhulax
    os.path.join("finals", "vision", "detector.py"),         # ultralytics, cv2
    os.path.join("finals", "vision", "aruco.py"),            # cv2
    os.path.join("finals", "vision", "perception.py"),       # cv2 (annotations)
}

FORBIDDEN_SDK_ROOTS = {
    "pyhulax", "mavsdk", "gz", "ultralytics", "cv2", "serial", "rclpy",
    "pyrealsense2", "torch",
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
        "takeoff_demo", "sentry_scan", "lawnmower", "track_convoy", "land_on_pad",
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


def test_main_no_drone_execution_points_at_s7(repo_root, monkeypatch):
    """S4 wired the flight path (mock now RUNS — see test_orchestrator.py);
    the no-drone replay/vision path stays a loud S7 pointer until then."""
    from finals.main import main

    monkeypatch.chdir(repo_root)
    with pytest.raises(NotImplementedError, match="S7"):
        main(["--profile", "replay"])
