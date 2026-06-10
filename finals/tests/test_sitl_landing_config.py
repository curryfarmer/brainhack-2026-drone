"""finals/configs/sitl{1,3}_landing.json (NAV-9) — the SITL LANDING rehearsal.

Pins the SITL landing-config CONTRACT: both load clean; the 3-drone config targets
DISTINCT + VALID pads from arena_name "sitl_landing"; every drone resolves
[takeoff, navigate, land_on_pad]; and — the NAV-9 config.py fix under test — the
'sitl' profile accepts the NAV-8 TIME+SPACE separation (a sector_deg on every
drone) INSTEAD of distinct altitude bands (the ~1.1 m-ceiling landing mission has
no bands), while still REFUSING a multi-drone SITL flight that declares neither.

These are the configs sim/run_landing.sh flies on the VM; the arena mirrors
sim/worlds/landing_px4.sdf (north=gz+Y, east=gz+X). Pure: stdlib + pytest.
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
_P1 = os.path.join(_CONFIG_DIR, "sitl1_landing.json")
_P3 = os.path.join(_CONFIG_DIR, "sitl3_landing.json")


def _raw(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


# ============================================================
# Load + shape
# ============================================================
def test_sitl1_landing_loads_clean():
    cfg = load_config(_P1)
    assert cfg.profile == "sitl"
    assert cfg.flight_backend == "mavsdk_sitl"
    assert cfg.frame_backend == "gazebo"
    assert cfg.arena is not None
    assert len(cfg.drones) == 1
    assert cfg.drones[0].zone["navigate"]["pad_id"] == "pad_101"


def test_sitl3_landing_loads_clean_distinct_valid_pads():
    cfg = load_config(_P3)
    assert cfg.profile == "sitl"
    assert len(cfg.drones) == 3
    targets = [d.zone["navigate"]["pad_id"] for d in cfg.drones]
    assert targets == ["pad_101", "pad_100", "pad_102"]
    assert len(set(targets)) == 3                      # distinct
    valid_ids = {p.id for p in cfg.arena.pads if p.valid}
    for t in targets:
        assert t in valid_ids                          # each target is a VALID pad


def test_sitl3_landing_separation_is_sectors_not_bands():
    """The landing mission separates by sector (+ time), NOT altitude bands —
    so every drone carries sector_deg and NONE carries an altitude_band_m."""
    cfg = load_config(_P3)
    assert all(d.sector_deg is not None for d in cfg.drones)
    assert all(d.altitude_band_m is None for d in cfg.drones)


def test_sitl3_landing_transport_endpoints_distinct():
    cfg = load_config(_P3)
    assert len({d.sitl_address for d in cfg.drones}) == 3
    assert len({d.mavsdk_grpc_port for d in cfg.drones}) == 3
    assert len({d.gazebo_video_port for d in cfg.drones}) == 3


def test_sitl3_landing_every_drone_builds_landing_phases():
    cfg = load_config(_P3)
    for drone in cfg.drones:
        phases = _build_phases(drone, cfg)
        assert [p.name for p in phases] == ["takeoff", "navigate", "land_on_pad"]


# ============================================================
# The config.py fix under test: 'sitl' multi-drone accepts sectors-OR-bands.
# ============================================================
def test_sitl_multidrone_sectors_accepted_without_bands():
    """Loading sitl3_landing.json AT ALL proves it: 3 drones, sector_deg on each,
    NO altitude_band_m. Before the NAV-9 fix the 'sitl' branch demanded distinct
    bands and this raised — so a clean load kills that mutant."""
    cfg = load_config(_P3)              # must not raise
    assert len(cfg.drones) == 3


def test_sitl_multidrone_without_any_separation_is_refused(tmp_path):
    """Strip sector_deg from one drone (and there are no bands) -> the SITL guard
    must still REFUSE (silent no-separation is the bug class it prevents)."""
    raw = _raw(_P3)
    del raw["drones"][1]["sector_deg"]
    p = tmp_path / "nosep.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ConfigError, match="sector_deg"):
        load_config(str(p))


def test_sitl_multidrone_distinct_bands_still_accepted(tmp_path):
    """The OTHER separation mechanism must still work: distinct altitude_band_m
    on every drone, no sectors, loads clean (proves the EITHER/OR, not sectors-
    only)."""
    raw = _raw(_P3)
    for i, drone in enumerate(raw["drones"]):
        drone.pop("sector_deg", None)
        drone["altitude_band_m"] = 1.0 + 0.1 * i        # 1.0 / 1.1 / 1.2, distinct
    p = tmp_path / "bands.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    cfg = load_config(str(p))                            # must not raise
    assert [d.altitude_band_m for d in cfg.drones] == [1.0, 1.1, 1.2]
