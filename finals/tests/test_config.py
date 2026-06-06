"""finals.config — the loud loader. Every failure mode must name its cause."""
from __future__ import annotations

import os

import pytest

from finals.config import load_config
from finals.errors import ConfigError


def test_minimal_mock_config_loads(write_config, minimal_mock_config):
    cfg = load_config(write_config(minimal_mock_config))
    assert cfg.profile == "mock"
    assert cfg.tick_hz == 10.0                     # default applied
    assert cfg.mission_budget_s == 600.0
    assert cfg.drones[0].id == "alpha"
    assert cfg.drones[0].phases == ["takeoff_demo"]


def test_missing_file_names_path():
    with pytest.raises(ConfigError, match="no_such_config.json"):
        load_config("no_such_config.json")


def test_invalid_json_is_config_error(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text("{ not json", encoding="utf-8")
    with pytest.raises(ConfigError, match="invalid JSON"):
        load_config(str(p))


def test_unknown_top_level_key_is_named(write_config, minimal_mock_config):
    minimal_mock_config["tick_hzz"] = 5  # typo
    with pytest.raises(ConfigError, match="tick_hzz"):
        load_config(write_config(minimal_mock_config))


def test_comment_keys_are_ignored(write_config, minimal_mock_config):
    minimal_mock_config["_comment"] = "ignored"
    minimal_mock_config["detector"]["_why"] = "also ignored"
    assert load_config(write_config(minimal_mock_config)).profile == "mock"


def test_missing_required_key_is_named(write_config, minimal_mock_config):
    del minimal_mock_config["drones"]
    with pytest.raises(ConfigError, match="drones"):
        load_config(write_config(minimal_mock_config))


def test_profile_pins_flight_backend(write_config, minimal_mock_config):
    minimal_mock_config["flight_backend"] = "pyhulax"  # copy-paste accident
    with pytest.raises(ConfigError, match="pins flight_backend"):
        load_config(write_config(minimal_mock_config))


def test_mock_profile_requires_a_drone(write_config, minimal_mock_config):
    minimal_mock_config["drones"] = []
    with pytest.raises(ConfigError, match="at least one drone"):
        load_config(write_config(minimal_mock_config))


def test_duplicate_drone_ids_rejected(write_config, minimal_mock_config):
    minimal_mock_config["drones"] = [
        {"id": "alpha", "phases": ["takeoff_demo"]},
        {"id": "alpha", "phases": ["takeoff_demo"]},
    ]
    with pytest.raises(ConfigError, match="duplicate drone ids"):
        load_config(write_config(minimal_mock_config))


def test_empty_phases_rejected(write_config, minimal_mock_config):
    minimal_mock_config["drones"][0]["phases"] = []
    with pytest.raises(ConfigError, match="phases"):
        load_config(write_config(minimal_mock_config))


def test_bad_led_rgb_rejected(write_config, minimal_mock_config):
    minimal_mock_config["drones"][0]["led_rgb"] = [300, 0, 0]
    with pytest.raises(ConfigError, match="led_rgb"):
        load_config(write_config(minimal_mock_config))


# ---------------- detector weights guards (the qualifier trap) ----------------
def _ultra_config(base: dict, weights: str) -> dict:
    base["detector"] = {"backend": "ultralytics", "weights": weights}
    return base


def test_ultralytics_requires_weights(write_config, minimal_mock_config):
    minimal_mock_config["detector"] = {"backend": "ultralytics"}
    with pytest.raises(ConfigError, match="weights"):
        load_config(write_config(minimal_mock_config))


def test_missing_weights_file_lists_candidates(write_config, minimal_mock_config):
    cfg = _ultra_config(minimal_mock_config, "definitely_not_here.pt")
    with pytest.raises(ConfigError, match="definitely_not_here.pt"):
        load_config(write_config(cfg))


def test_coco_placeholder_rejected_without_flag(write_config, tmp_path, minimal_mock_config):
    coco = tmp_path / "yolov8n.pt"
    coco.write_bytes(b"fake")
    cfg = _ultra_config(minimal_mock_config, str(coco))
    with pytest.raises(ConfigError, match="allow_coco_weights"):
        load_config(write_config(cfg))


def test_coco_placeholder_allowed_with_flag(write_config, tmp_path, minimal_mock_config):
    coco = tmp_path / "yolov8n.pt"
    coco.write_bytes(b"fake")
    cfg = _ultra_config(minimal_mock_config, str(coco))
    cfg["detector"]["allow_coco_weights"] = True
    loaded = load_config(write_config(cfg))
    assert loaded.detector.weights == str(coco)


def test_custom_weights_resolved_relative_to_config_dir(write_config, tmp_path, minimal_mock_config):
    (tmp_path / "robomaster.pt").write_bytes(b"fake")
    cfg = _ultra_config(minimal_mock_config, "robomaster.pt")
    loaded = load_config(write_config(cfg))
    assert os.path.isfile(loaded.detector.weights)


# ---------------- replay / bench profile rules ----------------
def test_replay_requires_no_drones_and_replay_dir(write_config):
    base = {
        "profile": "replay", "flight_backend": "none", "frame_backend": "replay",
        "detector": {"backend": "none"}, "drones": [],
    }
    with pytest.raises(ConfigError, match="replay_dir"):
        load_config(write_config(dict(base)))
    base["replay_dir"] = "frames/"
    assert load_config(write_config(dict(base))).replay_dir == "frames/"
    base["drones"] = [{"id": "alpha", "phases": ["takeoff_demo"]}]
    with pytest.raises(ConfigError, match="laptop-only"):
        load_config(write_config(base))


def _bench_config(drones: list) -> dict:
    return {
        "profile": "bench", "flight_backend": "bench", "frame_backend": "pyhulax",
        "detector": {"backend": "none"}, "drones": drones,
    }


def test_bench_requires_plane_ids(write_config):
    cfg = _bench_config([{"id": "alpha", "phases": ["takeoff_demo"]}])
    with pytest.raises(ConfigError, match="plane_id"):
        load_config(write_config(cfg))


def test_multi_drone_requires_distinct_altitude_bands(write_config):
    cfg = _bench_config([
        {"id": "alpha", "plane_id": 1, "altitude_band_m": 1.2, "phases": ["takeoff_demo"]},
        {"id": "bravo", "plane_id": 2, "altitude_band_m": 1.2, "phases": ["takeoff_demo"]},
    ])
    with pytest.raises(ConfigError, match="altitude_band_m"):
        load_config(write_config(cfg))


# ---------------- overrides ----------------
def test_overrides_apply_before_validation(write_config, minimal_mock_config):
    cfg = load_config(
        write_config(minimal_mock_config),
        overrides={"budget_s": 120.0, "no_detector": True, "phases": ["sentry_scan"]},
    )
    assert cfg.mission_budget_s == 120.0
    assert cfg.detector.backend == "none"
    assert cfg.drones[0].phases == ["sentry_scan"]


def test_unknown_override_key_rejected(write_config, minimal_mock_config):
    with pytest.raises(ConfigError, match="bogus"):
        load_config(write_config(minimal_mock_config), overrides={"bogus": 1})


def test_use_uwb_requires_port(write_config, minimal_mock_config):
    minimal_mock_config["use_uwb"] = True
    with pytest.raises(ConfigError, match="uwb_serial_port"):
        load_config(write_config(minimal_mock_config))


# ---------------- the five shipped profiles ----------------
@pytest.mark.parametrize("profile", ["mock", "sitl", "replay", "bench", "real"])
def test_shipped_config_loads(profile, repo_root, monkeypatch):
    monkeypatch.chdir(repo_root)  # weights ("best.pt") resolve from repo root
    cfg = load_config(os.path.join(repo_root, "finals", "configs", f"{profile}.json"))
    assert cfg.profile == profile
