"""finals/configs/convoy_real.json (NAV-8) — the proven convoy search mission,
real profile. Pins: loads clean; frame_backend pyhulax; 3 drones resolve
[takeoff, sentry_scan]; distinct altitude bands (the proven SIM-5 separation,
legal for the search task — not under the 1.1 m ceiling); fail-loud on a
band collision; the optional track_convoy phase is registry-resolvable.
"""
from __future__ import annotations

import json
import os

import pytest

from finals.config import load_config
from finals.errors import ConfigError
from finals.main import _build_phases

_CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs")
_PATH = os.path.join(_CONFIG_DIR, "convoy_real.json")


def _raw():
    with open(_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def test_convoy_real_loads_clean():
    cfg = load_config(_PATH)
    assert cfg.profile == "real"
    assert cfg.flight_backend == "pyhulax"
    assert cfg.frame_backend == "pyhulax"
    assert cfg.marker_backend == "aruco"
    assert len(cfg.drones) == 3


def test_each_drone_resolves_takeoff_then_sentry_scan():
    cfg = load_config(_PATH)
    for d in cfg.drones:
        names = [p.name for p in _build_phases(d, cfg)]
        assert names == ["takeoff", "sentry_scan"]


def test_distinct_altitude_bands():
    cfg = load_config(_PATH)
    bands = [d.altitude_band_m for d in cfg.drones]
    assert None not in bands
    assert len(set(bands)) == 3, f"bands not distinct: {bands}"


def test_takeoff_height_follows_band():
    """The band IS the takeoff height (takeoff.from_config / _height_from_band)."""
    cfg = load_config(_PATH)
    for d in cfg.drones:
        tk = _build_phases(d, cfg)[0]
        assert tk.height_cm == int(round(d.altitude_band_m * 100))


def test_duplicate_band_fails_loud(tmp_path):
    raw = _raw()
    raw["drones"][1]["altitude_band_m"] = raw["drones"][0]["altitude_band_m"]
    raw["drones"][2]["altitude_band_m"] = raw["drones"][0]["altitude_band_m"]
    p = tmp_path / "dupband.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ConfigError, match="DISTINCT altitude_band_m|separation"):
        load_config(str(p))


def test_no_separation_at_all_fails_loud(tmp_path):
    """A multi-drone real config with NEITHER distinct bands NOR per-drone
    sectors is refused — silent no-separation is the bug class the guard
    prevents."""
    raw = _raw()
    for d in raw["drones"]:
        d.pop("altitude_band_m", None)
    p = tmp_path / "nosep.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ConfigError, match="separation|sector_deg"):
        load_config(str(p))


def test_track_convoy_is_resolvable_optional_upgrade():
    """The commented track_convoy upgrade must actually resolve when added."""
    from finals.mission.phases import resolve_phase
    assert resolve_phase("track_convoy") is not None
