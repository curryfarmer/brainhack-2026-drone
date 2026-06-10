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
def test_replay_requires_no_drones_and_replay_dir(write_config, tmp_path):
    base = {
        "profile": "replay", "flight_backend": "none", "frame_backend": "replay",
        "detector": {"backend": "none"}, "drones": [],
    }
    with pytest.raises(ConfigError, match="replay_dir"):
        load_config(write_config(dict(base)))
    # S7: replay_dir must EXIST at load time (the weights-guard philosophy —
    # a missing frame source dies at load, not in a perception thread) and
    # is stored resolved-absolute.
    frames = tmp_path / "frames"
    frames.mkdir()
    base["replay_dir"] = str(frames)
    assert load_config(write_config(dict(base))).replay_dir == str(frames)
    base["drones"] = [{"id": "alpha", "phases": ["takeoff_demo"]}]
    with pytest.raises(ConfigError, match="laptop-only"):
        load_config(write_config(base))


def test_replay_dir_must_be_a_string(write_config):
    base = {
        "profile": "replay", "flight_backend": "none", "frame_backend": "replay",
        "replay_dir": 123,
        "detector": {"backend": "none"}, "drones": [],
    }
    with pytest.raises(ConfigError, match="replay_dir"):   # not a TypeError
        load_config(write_config(base))


def test_replay_dir_must_exist_on_disk(write_config):
    base = {
        "profile": "replay", "flight_backend": "none", "frame_backend": "replay",
        "replay_dir": "no_such_frames_dir/",
        "detector": {"backend": "none"}, "drones": [],
    }
    with pytest.raises(ConfigError, match="not found on disk"):
        load_config(write_config(base))


def test_replay_frames_on_mock_profile_require_replay_dir(
        write_config, minimal_mock_config, tmp_path):
    """frame_backend 'replay' is profile-independent (a mock flight over
    disk frames is the S7 vision-wiring smoke) — replay_dir is required and
    resolved on ANY profile, not just profile=replay."""
    minimal_mock_config["frame_backend"] = "replay"
    with pytest.raises(ConfigError, match="replay_dir"):
        load_config(write_config(dict(minimal_mock_config)))
    frames = tmp_path / "frames"
    frames.mkdir()
    minimal_mock_config["replay_dir"] = str(frames)
    assert load_config(write_config(minimal_mock_config)).replay_dir == str(frames)


# ---------------- marker / replay knobs (S7) ----------------
def test_marker_backend_default_and_membership(write_config, minimal_mock_config):
    assert load_config(
        write_config(dict(minimal_mock_config))).marker_backend == "aruco"
    minimal_mock_config["marker_backend"] = "apriltag"
    with pytest.raises(ConfigError, match="marker_backend"):
        load_config(write_config(minimal_mock_config))


# --- PAD-DICT: marker_dict strict membership + params whitelist ---------------
def test_marker_dict_default_is_the_real_field(write_config, minimal_mock_config):
    # The real default is DICT_7X7_1000 (the field beacons), NOT the 6x6 the
    # detector used to hardcode.
    cfg = load_config(write_config(dict(minimal_mock_config)))
    assert cfg.marker_dict == "DICT_7X7_1000"


def test_marker_dict_sim_pin_accepted(write_config, minimal_mock_config):
    minimal_mock_config["marker_dict"] = "DICT_6X6_250"     # the sim/fixture pin
    assert load_config(
        write_config(minimal_mock_config)).marker_dict == "DICT_6X6_250"


@pytest.mark.parametrize("bad", ["DICT_7x7_1000", "DICT_8X8_250", "", 7, None])
def test_marker_dict_unknown_is_loud(write_config, minimal_mock_config, bad):
    # a typo or a non-cv2 name dies on the ground (strict VALID_MARKER_DICTS),
    # not in the detector thread reading nothing off the real field.
    minimal_mock_config["marker_dict"] = bad
    with pytest.raises(ConfigError, match="marker_dict"):
        load_config(write_config(minimal_mock_config))


def test_aruco_detector_params_none_default(write_config, minimal_mock_config):
    assert load_config(
        write_config(dict(minimal_mock_config))).aruco_detector_params is None


def test_aruco_detector_params_whitelisted_key_accepted(
        write_config, minimal_mock_config):
    minimal_mock_config["aruco_detector_params"] = {
        "minMarkerPerimeterRate": 0.01, "adaptiveThreshWinSizeMin": 5}
    cfg = load_config(write_config(minimal_mock_config))
    assert cfg.aruco_detector_params == {
        "minMarkerPerimeterRate": 0.01, "adaptiveThreshWinSizeMin": 5}


def test_aruco_detector_params_typo_key_is_loud(write_config, minimal_mock_config):
    # a typo'd DetectorParameters field is a silent no-op on the real object —
    # config rejects the KEY on the ground (the faint-beacon tuning trap).
    minimal_mock_config["aruco_detector_params"] = {"minMarkerPerimterRate": 0.01}
    with pytest.raises(ConfigError, match="aruco_detector_params"):
        load_config(write_config(minimal_mock_config))


def test_aruco_detector_params_non_object_is_loud(write_config, minimal_mock_config):
    minimal_mock_config["aruco_detector_params"] = [1, 2, 3]      # not an object
    with pytest.raises(ConfigError, match="aruco_detector_params"):
        load_config(write_config(minimal_mock_config))


@pytest.mark.parametrize("bad", [0, -1, float("inf"), True])
def test_replay_fps_validated(write_config, minimal_mock_config, bad):
    minimal_mock_config["replay_fps"] = bad
    with pytest.raises(ConfigError, match="replay_fps"):
        load_config(write_config(minimal_mock_config))


def test_detector_workers_validated(write_config, minimal_mock_config):
    minimal_mock_config["detector"] = {"backend": "none", "workers": 0}
    with pytest.raises(ConfigError, match="workers"):
        load_config(write_config(minimal_mock_config))


def test_discovery_timeout_s_default_and_roundtrip(write_config,
                                                   minimal_mock_config):
    # S10: the preflight P3 Dola listen window — defaults, onsite-tunable.
    assert load_config(
        write_config(dict(minimal_mock_config))).discovery_timeout_s == 10.0
    minimal_mock_config["discovery_timeout_s"] = 4.5
    assert load_config(
        write_config(minimal_mock_config)).discovery_timeout_s == 4.5


@pytest.mark.parametrize("bad", [0, -1, float("inf"), True])
def test_discovery_timeout_s_validated(write_config, minimal_mock_config, bad):
    minimal_mock_config["discovery_timeout_s"] = bad
    with pytest.raises(ConfigError, match="discovery_timeout_s"):
        load_config(write_config(minimal_mock_config))


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


# ---------------- SIM-5: per-drone gazebo_video_port ----------------
def _sitl3_gazebo() -> dict:
    """The SIM-5 shape: 3 PX4 camera-drones, gazebo frames, a distinct bridge
    port per drone."""
    def drone(id_, addr, grpc, band, port):
        return {"id": id_, "phases": ["sentry_scan"], "altitude_band_m": band,
                "sitl_address": addr, "mavsdk_grpc_port": grpc,
                "gazebo_video_port": port}
    return {
        "profile": "sitl", "flight_backend": "mavsdk_sitl",
        "frame_backend": "gazebo", "camera_hfov_deg": 99.69,
        "detector": {"backend": "none"},
        "drones": [
            drone("alpha", "udpin://0.0.0.0:14540", 50051, 1.2, 5600),
            drone("bravo", "udpin://0.0.0.0:14541", 50052, 1.7, 5601),
            drone("charlie", "udpin://0.0.0.0:14542", 50053, 2.2, 5602),
        ],
    }


def test_resolve_gazebo_video_port_per_drone_then_fallback(write_config):
    from finals.config import resolve_gazebo_video_port
    cfg = load_config(write_config(_sitl3_gazebo()))
    assert resolve_gazebo_video_port(cfg, "alpha") == 5600
    assert resolve_gazebo_video_port(cfg, "bravo") == 5601
    assert resolve_gazebo_video_port(cfg, "charlie") == 5602
    # an id not in the fleet (the replay runner never hits the gazebo branch)
    # falls back to the top-level default port, never crashes.
    assert resolve_gazebo_video_port(cfg, "nobody") == cfg.gazebo_video_port


def test_sitl3_gazebo_ports_distinct_and_parsed(write_config):
    cfg = load_config(write_config(_sitl3_gazebo()))
    ports = [d.gazebo_video_port for d in cfg.drones]
    assert ports == [5600, 5601, 5602]
    assert len(set(ports)) == 3


def test_multidrone_gazebo_missing_port_rejected(write_config):
    cfg = _sitl3_gazebo()
    del cfg["drones"][2]["gazebo_video_port"]      # charlie has no bridge port
    with pytest.raises(ConfigError, match="gazebo_video_port on EVERY drone"):
        load_config(write_config(cfg))


def test_multidrone_gazebo_duplicate_port_rejected(write_config):
    cfg = _sitl3_gazebo()
    cfg["drones"][1]["gazebo_video_port"] = 5600    # bravo collides with alpha
    with pytest.raises(ConfigError, match="DISTINCT"):
        load_config(write_config(cfg))


def test_gazebo_video_port_range_checked(write_config):
    cfg = _sitl3_gazebo()
    cfg["drones"][0]["gazebo_video_port"] = 80       # privileged, out of range
    with pytest.raises(ConfigError, match="gazebo_video_port"):
        load_config(write_config(cfg))


def test_single_drone_gazebo_falls_back_to_top_level(write_config):
    """sitl_vision.json's shape: one drone, no per-drone port — the top-level
    fallback resolves and the multi-drone distinctness guard does NOT fire."""
    from finals.config import resolve_gazebo_video_port
    cfg_dict = _sitl3_gazebo()
    cfg_dict["drones"] = cfg_dict["drones"][:1]
    del cfg_dict["drones"][0]["gazebo_video_port"]
    cfg_dict["gazebo_video_port"] = 5600
    cfg = load_config(write_config(cfg_dict))
    assert resolve_gazebo_video_port(cfg, "alpha") == 5600


def test_shipped_sitl3_vision_loads(repo_root):
    cfg = load_config(os.path.join(
        repo_root, "finals", "configs", "sitl3_vision.json"))
    assert cfg.frame_backend == "gazebo"
    assert [d.id for d in cfg.drones] == ["alpha", "bravo", "charlie"]
    ports = [d.gazebo_video_port for d in cfg.drones]
    assert ports == [5600, 5601, 5602]


# ---------------- guards (S5) ----------------
def test_guards_defaults_applied(write_config, minimal_mock_config):
    cfg = load_config(write_config(minimal_mock_config))
    assert cfg.guards.telemetry_stale_s == 2.0
    assert cfg.guards.battery_warn_pct == 30.0
    assert cfg.guards.landing_reserve_s == 0.0      # MissionClockGuard OFF
    assert cfg.guards.phase_timeout_s is None       # PhaseTimeout OFF
    assert cfg.guards.geofence_radius_m is None     # GeofenceLite OFF
    assert cfg.guards.land_retry_period_s == 1.0
    assert cfg.guards.land_retry_window_s == 30.0


def test_guards_keys_roundtrip_and_comments_ignored(write_config,
                                                    minimal_mock_config):
    minimal_mock_config["guards"] = {
        "_comment": "ignored",
        "battery_warn_pct": 50.0,
        "landing_reserve_s": 60.0,
        "phase_timeout_s": 90.0,
        "geofence_radius_m": 25.0,
        "geofence_alt_m": 4.0,
    }
    cfg = load_config(write_config(minimal_mock_config))
    assert cfg.guards.battery_warn_pct == 50.0
    assert cfg.guards.landing_reserve_s == 60.0
    assert cfg.guards.phase_timeout_s == 90.0
    assert cfg.guards.geofence_radius_m == 25.0
    assert cfg.guards.geofence_alt_m == 4.0


def test_guards_unknown_key_named(write_config, minimal_mock_config):
    minimal_mock_config["guards"] = {"battery_warn_pctt": 50.0}   # typo
    with pytest.raises(ConfigError, match="battery_warn_pctt"):
        load_config(write_config(minimal_mock_config))


def test_guards_warn_under_floor_rejected(write_config, minimal_mock_config):
    minimal_mock_config["guards"] = {"battery_warn_pct": 10.0}    # floor is 20
    with pytest.raises(ConfigError, match="battery_warn_pct"):
        load_config(write_config(minimal_mock_config))


def test_guards_reserve_over_budget_rejected(write_config,
                                             minimal_mock_config):
    minimal_mock_config["guards"] = {"landing_reserve_s": 700.0}  # budget 600
    with pytest.raises(ConfigError, match="landing_reserve_s"):
        load_config(write_config(minimal_mock_config))


def test_guards_reserve_checked_against_budget_override(write_config,
                                                        minimal_mock_config):
    """The instant-trip trap: a --budget override shrinking the budget under
    the configured reserve must die at load time, not at t=0 in the air."""
    minimal_mock_config["guards"] = {"landing_reserve_s": 60.0}
    with pytest.raises(ConfigError, match="landing_reserve_s"):
        load_config(write_config(minimal_mock_config),
                    overrides={"budget_s": 30.0})


def test_guards_geofence_alt_requires_radius(write_config,
                                             minimal_mock_config):
    minimal_mock_config["guards"] = {"geofence_alt_m": 4.0}
    with pytest.raises(ConfigError, match="geofence_radius_m"):
        load_config(write_config(minimal_mock_config))


def test_guards_ladder_window_under_period_rejected(write_config,
                                                    minimal_mock_config):
    minimal_mock_config["guards"] = {"land_retry_period_s": 5.0,
                                     "land_retry_window_s": 1.0}
    with pytest.raises(ConfigError, match="land_retry_window_s"):
        load_config(write_config(minimal_mock_config))


def test_guards_telemetry_stale_over_backstop_rejected(write_config,
                                                       minimal_mock_config):
    """The layering-inversion trap: the TelemetryWatchdog policy threshold
    at/above the agent's 5 s SensorTimeout backstop would turn every stale-
    telemetry event into an emergency FAILED instead of a clean landing."""
    minimal_mock_config["guards"] = {"telemetry_stale_s": 5.0}
    with pytest.raises(ConfigError, match="backstop"):
        load_config(write_config(minimal_mock_config))
