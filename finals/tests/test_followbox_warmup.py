"""finals/configs/sitl1_followbox{1,_multi}.json (WS-4) — the two warm-up
'follow a convoy' sims A & B, pinned PURELY (stdlib + pytest, no gz/ROS/cv2).

These configs fly ONE PX4 camera-drone [takeoff -> navigate around crate(s) ->
track_convoy] in sim/worlds/followbox{1,_multi}_px4.sdf. The SITL run on the VM
is an integration check; what is asserted HERE is the part that must be right
BEFORE any gz time and that a typo would silently break:

  * both configs load clean, profile sitl / mavsdk_sitl / gazebo frames, ONE
    drone whose phases END in track_convoy (follow convoy_robot_7), not a pad
    landing — so config.py's pad-capacity guard must NOT demand a pad (the WS-4
    _validate_pad_targets fix: count land_on_pad drones, not navigate drones);
  * the navigate goal really sits BEHIND the crate(s) so the visibility-graph
    planner is FORCED to detour/weave — proven by flying the planned Legs through
    the REAL DeadReckoner (the NAV-5 forward model, the oracle) and asserting the
    flown path (a) reaches the goal and (b) clears every REAL crate, while the
    straight C2->goal shot WOULD cut an inflated keep-out (so the detour is
    necessary, not incidental);
  * sim/convoy_driver.py parse_route (the SIM-B irregular-path source) parses a
    good spec and fails LOUD on each malformed field.

The arena keep-outs mirror the SDF crate footprints (WS-7B turns that into a CI
assertion); here we just trust the loaded arena and check the geometry is flyable.
"""
from __future__ import annotations

import importlib.util
import json
import os

import pytest

from finals.config import load_config
from finals.flight.dead_reckon import DeadReckoner, DRPose
from finals.main import _build_phases
from finals.mission.planning.polygon_tools import (inflate_polygon,
                                                   segment_enters_polygon)
from finals.types import Direction, Move, Rotate

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(os.path.dirname(_HERE))
_CONFIG_DIR = os.path.join(os.path.dirname(_HERE), "configs")
_P_A = os.path.join(_CONFIG_DIR, "sitl1_followbox1.json")
_P_B = os.path.join(_CONFIG_DIR, "sitl1_followbox_multi.json")


# --- load sim/convoy_driver.py without ROS: its module top imports only
#     argparse/sys/time (rclpy is imported inside main), so this is safe. ---
def _load_convoy_driver():
    path = os.path.join(_ROOT, "sim", "convoy_driver.py")
    spec = importlib.util.spec_from_file_location("_ws4_convoy_driver", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# --- fly Legs through the REAL DeadReckoner (math never reimplemented here). ---
def _fly(legs, start):
    dr = DeadReckoner(DRPose(start[0], start[1], 0.0, 0.0))
    pts = [(dr.pose.north_m, dr.pose.east_m)]
    for leg in legs:
        dr.note_action_complete(Rotate(angle_deg=leg.heading_deg - dr.pose.yaw_deg))
        dr.note_action_complete(
            Move(direction=Direction.FORWARD, distance_cm=leg.distance_cm))
        pts.append((dr.pose.north_m, dr.pose.east_m))
    return pts


def _hits(pts, polygon_m):
    return any(segment_enters_polygon(pts[i], pts[i + 1], polygon_m)
               for i in range(len(pts) - 1))


def _plan_navigate(cfg):
    """Build the navigate Legs for a 1-drone followbox cfg exactly as the phase
    will: c2_origin -> goal_ne_m, planning over the loaded arena keep-outs."""
    from finals.mission.planning.visibility_graph import plan
    drone = cfg.drones[0]
    nav = drone.zone["navigate"]
    start = tuple(cfg.arena.c2_origin_m)
    goal = tuple(nav["goal_ne_m"])
    legs = plan(start, goal, cfg.arena,
                inflation_m=float(nav["inflation_m"]),
                max_leg_cm=float(nav["max_leg_cm"]))
    return start, goal, legs


# ============================================================
# Load + shape (both configs)
# ============================================================
@pytest.mark.parametrize("path", [_P_A, _P_B])
def test_followbox_config_loads_clean_one_tracking_drone(path):
    cfg = load_config(path)
    assert cfg.profile == "sitl"
    assert cfg.flight_backend == "mavsdk_sitl"
    assert cfg.frame_backend == "gazebo"
    assert cfg.arena is not None
    assert len(cfg.drones) == 1
    drone = cfg.drones[0]
    phases = [p.name for p in _build_phases(drone, cfg)]
    assert phases == ["takeoff", "navigate", "track_convoy"]
    # Ends in track_convoy, NOT land_on_pad -> consumes NO pad, and the arena
    # ships none. Loading AT ALL proves config.py's pad-capacity guard counts
    # land_on_pad drones (the WS-4 fix), not navigate drones.
    assert cfg.arena.pads == ()
    assert drone.zone["track_convoy"]["track_marker_ids"] == [7]


def test_followbox1_navigate_goal_sits_behind_the_crate():
    """SIM-A: the straight C2->goal shot must cut the inflated crate (so a detour
    is REQUIRED) and the flown detour must clear the REAL crate + reach goal."""
    cfg = load_config(_P_A)
    start, goal, legs = _plan_navigate(cfg)
    crate = cfg.arena.keep_out[0]
    infl = float(cfg.drones[0].zone["navigate"]["inflation_m"])

    # The straight line is blocked -> detour is necessary, not incidental.
    assert _hits([start, goal], inflate_polygon(crate.polygon_m, infl))
    assert len(legs) >= 2

    pts = _fly(legs, start)
    assert pts[-1][0] == pytest.approx(goal[0], abs=1e-6)
    assert pts[-1][1] == pytest.approx(goal[1], abs=1e-6)
    assert not _hits(pts, crate.polygon_m)          # clears the REAL crate
    # It went AROUND: some waypoint bulges east past the crate's east face.
    assert max(abs(e) for _, e in pts) > 0.5


def test_followboxmulti_navigate_weaves_past_all_three_crates():
    """SIM-B: the straight shot cuts >=1 crate; the flown weave clears ALL three
    REAL crates and reaches the goal past them."""
    cfg = load_config(_P_B)
    start, goal, legs = _plan_navigate(cfg)
    infl = float(cfg.drones[0].zone["navigate"]["inflation_m"])
    crates = cfg.arena.keep_out
    assert len(crates) == 3

    # Straight C2->goal would hit at least one inflated crate -> weave required.
    assert any(_hits([start, goal], inflate_polygon(c.polygon_m, infl))
               for c in crates)
    assert len(legs) >= 2

    pts = _fly(legs, start)
    assert pts[-1][0] == pytest.approx(goal[0], abs=1e-6)
    assert pts[-1][1] == pytest.approx(goal[1], abs=1e-6)
    for c in crates:
        assert not _hits(pts, c.polygon_m), f"flown path clips real crate {c.id}"


# ============================================================
# parse_route — the SIM-B irregular-path source (fail-loud)
# ============================================================
def test_parse_route_parses_a_good_snaking_spec():
    drv = _load_convoy_driver()
    segs = drv.parse_route("40,0.07,0.0; 20,0.05,0.30; 40,0.07,0.0; 20,0.05,-0.30")
    assert segs == [(40.0, 0.07, 0.0), (20.0, 0.05, 0.30),
                    (40.0, 0.07, 0.0), (20.0, 0.05, -0.30)]


def test_parse_route_tolerates_whitespace_and_trailing_semicolon():
    drv = _load_convoy_driver()
    assert drv.parse_route("  10 , 0.1 , 0.2 ;  ") == [(10.0, 0.1, 0.2)]


@pytest.mark.parametrize("bad,needle", [
    ("", "non-empty"),
    ("   ", "non-empty"),
    ("10,0.1", "exactly 3 fields"),
    ("10,0.1,0.2,0.3", "exactly 3 fields"),
    ("10,fast,0.2", "non-numeric"),
    ("0,0.1,0.2", "must be > 0"),
    ("-5,0.1,0.2", "must be > 0"),
])
def test_parse_route_fails_loud_on_malformed_spec(bad, needle):
    drv = _load_convoy_driver()
    with pytest.raises(ValueError, match=needle):
        drv.parse_route(bad)
