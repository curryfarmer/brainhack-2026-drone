"""Shared fixtures for the finals test suite. Zero hardware, zero SDKs."""
from __future__ import annotations

import json
import os
import sys

import pytest

# Repo root = two levels above this file (finals/tests/conftest.py).
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


@pytest.fixture
def repo_root() -> str:
    return REPO_ROOT


@pytest.fixture
def write_config(tmp_path):
    """Factory: dict -> JSON config file path inside tmp_path."""
    counter = {"n": 0}

    def _write(data: dict) -> str:
        counter["n"] += 1
        path = tmp_path / f"cfg_{counter['n']}.json"
        path.write_text(json.dumps(data), encoding="utf-8")
        return str(path)

    return _write


@pytest.fixture
def minimal_mock_config() -> dict:
    """The smallest valid config — profile mock, one drone, no detection."""
    return {
        "profile": "mock",
        "flight_backend": "mock",
        "frame_backend": "none",
        "detector": {"backend": "none"},
        "drones": [{"id": "alpha", "phases": ["takeoff_demo"]}],
    }
