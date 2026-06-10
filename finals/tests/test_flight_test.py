"""Hardware-free tests for finals/tools/flight_test.py — the single-drone
flight-test launcher. Proves the SAFETY gating (a real flight is forced to
--dry-run without --live, and always carries the arms-real-drones flag) and the
config override plumbing, WITHOUT ever arming anything: finals.main is
monkeypatched to capture the argv the launcher would hand it."""
from __future__ import annotations

import json

import pytest

from finals.config import load_config
from finals.tools import flight_test as ft


# ---------------- base_config_path ----------------
def test_base_config_path_defaults_to_real():
    args = ft.parse_args([])
    assert ft.base_config_path(args) == ft.REAL_CONFIG


def test_base_config_path_sitl():
    args = ft.parse_args(["--sitl"])
    assert ft.base_config_path(args) == ft.SITL_CONFIG


def test_base_config_path_explicit_override():
    args = ft.parse_args(["--config", "/tmp/my.json"])
    assert ft.base_config_path(args) == "/tmp/my.json"


def test_base_config_path_3x_real():
    args = ft.parse_args(["--drones", "3"])
    assert ft.base_config_path(args) == ft.REAL_3X_CONFIG


def test_base_config_path_3x_sitl():
    args = ft.parse_args(["--sitl", "--drones", "3"])
    assert ft.base_config_path(args) == ft.SITL_3X_CONFIG


def test_base_config_path_explicit_override_wins_over_drones():
    # An explicit --config beats the --drones default in BOTH directions.
    args = ft.parse_args(["--drones", "3", "--config", "/tmp/my.json"])
    assert ft.base_config_path(args) == "/tmp/my.json"


# ---------------- needs_patch ----------------
def test_needs_patch_false_without_overrides():
    assert ft.needs_patch(ft.parse_args([])) is False
    assert ft.needs_patch(ft.parse_args(["--budget", "120"])) is False


@pytest.mark.parametrize("flag", ["--plane-id", "--marker-id", "--height-cm"])
def test_needs_patch_true_with_any_override(flag):
    assert ft.needs_patch(ft.parse_args([flag, "7"])) is True


def test_needs_patch_true_with_plane_ids():
    assert ft.needs_patch(ft.parse_args(["--plane-ids", "7", "10", "12"])) is True


# ---------------- _parse_plane_ids (runtime drone-code input) ----------------
@pytest.mark.parametrize("argv,expected", [
    (["--plane-ids", "7", "10", "12"], [7, 10, 12]),    # space-separated
    (["--plane-ids", "7,10,12"], [7, 10, 12]),          # comma-separated
    (["--plane-ids", "7,", "10", "12"], [7, 10, 12]),   # mixed comma/space
    (["--plane-ids", "9"], [9]),                         # single drone code
])
def test_parse_plane_ids_accepts_space_and_comma(argv, expected):
    assert ft._parse_plane_ids(ft.parse_args(argv).plane_ids) == expected


def test_parse_plane_ids_none_when_absent():
    assert ft._parse_plane_ids(ft.parse_args([]).plane_ids) is None


def test_parse_plane_ids_non_integer_is_loud():
    with pytest.raises(ValueError):
        ft._parse_plane_ids(["7", "x"])


# ---------------- patch_config ----------------
def _base():
    return {"drones": [{"id": "test", "plane_id": 1,
                        "zone": {"takeoff": {"height_cm": 100},
                                 "land_on_pad": {"valid_marker_ids": [0]}}}]}


def test_patch_config_overrides_land_in_the_right_keys():
    out = ft.patch_config(_base(), plane_id=6, marker_id=7, height_cm=120)
    d = out["drones"][0]
    assert d["plane_id"] == 6
    assert d["zone"]["takeoff"]["height_cm"] == 120
    assert d["zone"]["land_on_pad"]["valid_marker_ids"] == [7]


def test_patch_config_does_not_mutate_base():
    base = _base()
    ft.patch_config(base, plane_id=99, marker_id=42, height_cm=200)
    d = base["drones"][0]
    assert d["plane_id"] == 1
    assert d["zone"]["takeoff"]["height_cm"] == 100
    assert d["zone"]["land_on_pad"]["valid_marker_ids"] == [0]


def test_patch_config_creates_missing_zone_subdicts():
    out = ft.patch_config({"drones": [{"id": "t"}]},
                          marker_id=11, height_cm=90)
    z = out["drones"][0]["zone"]
    assert z["land_on_pad"]["valid_marker_ids"] == [11]
    assert z["takeoff"]["height_cm"] == 90


def test_patch_config_refuses_a_droneless_config():
    with pytest.raises(ValueError, match="no drone to patch"):
        ft.patch_config({"drones": []}, plane_id=1)


def test_patch_config_plane_ids_sets_every_drone_in_fleet_order():
    base = {"drones": [{"id": "a", "plane_id": 1}, {"id": "b", "plane_id": 2},
                       {"id": "c", "plane_id": 3}]}
    out = ft.patch_config(base, plane_ids=[7, 10, 12])
    assert [d["plane_id"] for d in out["drones"]] == [7, 10, 12]
    assert [d["plane_id"] for d in base["drones"]] == [1, 2, 3]   # base untouched


def test_patch_config_plane_ids_count_mismatch_is_loud():
    base = {"drones": [{"id": "a"}, {"id": "b"}, {"id": "c"}]}
    with pytest.raises(ValueError, match="plane-ids has 2 id"):
        ft.patch_config(base, plane_ids=[7, 10])


# ---------------- build_finals_argv (THE safety gate) ----------------
def test_real_without_live_is_forced_to_dry_run():
    argv = ft.build_finals_argv(ft.parse_args([]), "cfg.json")
    assert "--profile" in argv and argv[argv.index("--profile") + 1] == "real"
    assert "--i-know-this-arms-real-drones" in argv
    assert "--dry-run" in argv          # no --live -> never arms


def test_real_with_live_arms_no_dry_run():
    argv = ft.build_finals_argv(ft.parse_args(["--live"]), "cfg.json")
    assert "--i-know-this-arms-real-drones" in argv
    assert "--dry-run" not in argv      # --live removes the dry-run safety


def test_real_live_but_explicit_dry_run_stays_dry():
    argv = ft.build_finals_argv(ft.parse_args(["--live", "--dry-run"]), "cfg.json")
    assert "--dry-run" in argv          # explicit --dry-run wins over --live


def test_sitl_profile_has_no_real_gate_and_no_forced_dry():
    argv = ft.build_finals_argv(ft.parse_args(["--sitl"]), "cfg.json")
    assert argv[argv.index("--profile") + 1] == "sitl"
    assert "--i-know-this-arms-real-drones" not in argv
    assert "--dry-run" not in argv      # sitl never arms a real aircraft


def test_sitl_respects_explicit_dry_run():
    argv = ft.build_finals_argv(ft.parse_args(["--sitl", "--dry-run"]), "cfg.json")
    assert "--dry-run" in argv


def test_budget_passthrough():
    argv = ft.build_finals_argv(ft.parse_args(["--budget", "150"]), "cfg.json")
    assert "--budget" in argv and argv[argv.index("--budget") + 1] == "150.0"


def test_3x_real_without_live_is_forced_to_dry_run():
    # The SAME safety double-gate applies to the 3-drone real path.
    argv = ft.build_finals_argv(ft.parse_args(["--drones", "3"]), "cfg.json")
    assert argv[argv.index("--profile") + 1] == "real"
    assert "--i-know-this-arms-real-drones" in argv
    assert "--dry-run" in argv          # no --live -> never arms (3x too)


def test_3x_sitl_has_no_real_gate_and_no_forced_dry():
    argv = ft.build_finals_argv(
        ft.parse_args(["--sitl", "--drones", "3"]), "cfg.json")
    assert argv[argv.index("--profile") + 1] == "sitl"
    assert "--i-know-this-arms-real-drones" not in argv
    assert "--dry-run" not in argv      # sitl never arms a real aircraft


# ---------------- main() integration (finals.main monkeypatched) ----------------
def test_main_dry_run_calls_finals_with_expected_argv(monkeypatch):
    captured = {}

    def fake_finals_main(argv):
        captured["argv"] = argv
        return 0

    monkeypatch.setattr("finals.main.main", fake_finals_main)
    rc = ft.main(["--dry-run"])
    assert rc == 0
    assert "--dry-run" in captured["argv"]
    assert captured["argv"][captured["argv"].index("--profile") + 1] == "real"


def test_main_live_arms_after_a_pause(monkeypatch):
    """--live reaches finals.main WITHOUT --dry-run, after the read-the-brief
    pause — proven without flying or sleeping for real."""
    captured = {}
    slept = {"n": 0}

    def fake_finals_main(argv):
        captured["argv"] = argv
        return 0

    monkeypatch.setattr("finals.main.main", fake_finals_main)
    monkeypatch.setattr(ft.time, "sleep", lambda s: slept.__setitem__("n", slept["n"] + 1))
    rc = ft.main(["--live"])
    assert rc == 0
    assert "--dry-run" not in captured["argv"]
    assert "--i-know-this-arms-real-drones" in captured["argv"]
    assert slept["n"] == 1              # the operator-read pause ran


def test_main_missing_config_is_config_error(monkeypatch):
    monkeypatch.setattr("finals.main.main",
                        lambda argv: pytest.fail("must not reach finals.main"))
    rc = ft.main(["--config", "/no/such/file.json"])
    assert rc == 2


def test_main_3x_dry_run_calls_finals_with_3x_config(monkeypatch):
    captured = {}
    monkeypatch.setattr("finals.main.main",
                        lambda argv: captured.__setitem__("argv", argv) or 0)
    rc = ft.main(["--drones", "3", "--dry-run"])
    assert rc == 0
    assert "--dry-run" in captured["argv"]
    assert captured["argv"][captured["argv"].index("--profile") + 1] == "real"
    assert "--i-know-this-arms-real-drones" in captured["argv"]
    cfg_path = captured["argv"][captured["argv"].index("--config") + 1]
    assert cfg_path == ft.REAL_3X_CONFIG


@pytest.mark.parametrize("flag,value", [("--plane-id", "5"),
                                        ("--marker-id", "7"),
                                        ("--height-cm", "120")])
def test_main_3x_with_single_drone_override_refuses_loud(monkeypatch, capsys,
                                                         flag, value):
    """--drones 3 + a single-value override is a LOUD exit-2 refusal — the
    overrides patch ONE drone, so they are meaningless for a 3-drone config."""
    monkeypatch.setattr(
        "finals.main.main",
        lambda argv: pytest.fail("must not reach finals.main on a refusal"))
    rc = ft.main(["--drones", "3", flag, value])
    assert rc == 2
    err = capsys.readouterr().err
    # The message must NAME all three overrides AND point at the fleet-wide fix.
    assert "--plane-id/--marker-id/--height-cm" in err
    assert "flight_test_3x_real.json" in err
    assert "--plane-ids" in err          # the actionable multi-drone path


def test_main_3x_plane_ids_patches_fleet_and_reaches_finals(monkeypatch):
    """--drones 3 --plane-ids 7 10 12 is ALLOWED: it patches every drone's
    plane_id in fleet order and the resolved config reaches finals.main."""
    captured = {}
    monkeypatch.setattr("finals.main.main",
                        lambda argv: captured.__setitem__("argv", argv) or 0)
    rc = ft.main(["--drones", "3", "--plane-ids", "7", "10", "12", "--dry-run"])
    assert rc == 0
    cfg_path = captured["argv"][captured["argv"].index("--config") + 1]
    with open(cfg_path, encoding="utf-8") as f:
        cfg = json.load(f)
    assert [d["plane_id"] for d in cfg["drones"]] == [7, 10, 12]


def test_main_3x_plane_ids_wrong_count_is_config_error(monkeypatch):
    monkeypatch.setattr(
        "finals.main.main",
        lambda argv: pytest.fail("must not reach finals.main on a bad count"))
    rc = ft.main(["--drones", "3", "--plane-ids", "7", "10"])
    assert rc == 2


# ---------------- the shipped real config is valid + minimal ----------------
def test_real_config_loads_and_is_single_drone_landing():
    cfg = load_config(ft.REAL_CONFIG)
    assert cfg.profile == "real"
    assert cfg.flight_backend == "pyhulax" and cfg.frame_backend == "pyhulax"
    assert cfg.marker_backend == "aruco"
    assert len(cfg.drones) == 1
    assert cfg.drones[0].phases == ["takeoff", "land_on_pad"]
    # No navigate -> no arena needed (the minimal in-place-scan invariant).
    assert cfg.arena is None


def test_marker_override_reaches_valid_marker_ids(tmp_path):
    with open(ft.REAL_CONFIG, "r", encoding="utf-8") as f:
        base = json.load(f)
    patched = ft.patch_config(base, marker_id=7)
    p = tmp_path / "patched.json"
    p.write_text(json.dumps(patched), encoding="utf-8")
    cfg = load_config(str(p))
    assert cfg.drones[0].zone["land_on_pad"]["valid_marker_ids"] == [7]


# ---------------- the shipped 3x real config is valid + minimal ----------------
def test_3x_real_config_loads_and_is_three_drone_landing():
    cfg = load_config(ft.REAL_3X_CONFIG)
    assert cfg.profile == "real"
    assert cfg.flight_backend == "pyhulax" and cfg.frame_backend == "pyhulax"
    assert cfg.marker_backend == "aruco"
    assert len(cfg.drones) == 3
    # Every drone runs the SAME minimal in-place scan chain (no navigate).
    for d in cfg.drones:
        assert d.phases == ["takeoff", "land_on_pad"]
    # No navigate -> no arena needed (the minimal in-place-scan invariant).
    assert cfg.arena is None
    # Distinct plane_ids (distinct discovery keys) ...
    plane_ids = [d.plane_id for d in cfg.drones]
    assert len(set(plane_ids)) == 3
    # ... and a DISTINCT valid pad marker per drone (each lands on its OWN pad).
    marker_ids = [tuple(d.zone["land_on_pad"]["valid_marker_ids"])
                  for d in cfg.drones]
    assert len(set(marker_ids)) == 3
