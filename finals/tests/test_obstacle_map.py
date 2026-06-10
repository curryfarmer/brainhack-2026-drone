"""finals.mission.obstacle_map + the navigate merge — WS-6 shared collective map.

ObstacleMap is the in-process shared store of FIXED obstacles every drone's
navigate MERGES with the static arena (the user's "build a collective map and
share it" extension). Tests cover: the store + the ADDS-ONLY merge policy (static
authoritative), fail-loud on a degenerate contribution, the end-to-end navigate
merge (an observed-only keep-out forces a detour a static-arena plan would cut),
and the full config plumbing (observed_keep_out -> _build_obstacle_map ->
navigate). Pure — stdlib + pytest (no cv2/numpy)."""
from __future__ import annotations

import json
import os
import types

import pytest

from finals.config import DroneConfig, load_config
from finals.errors import ConfigError
from finals.flight.dead_reckon import DeadReckoner, DRPose
from finals.main import _build_obstacle_map, _build_phases
from finals.mission.obstacle_map import MapError, ObstacleMap
from finals.mission.phases.navigate import Navigate
from finals.mission.planning.polygon_tools import (inflate_polygon,
                                                   segment_enters_polygon)
from finals.mission.planning.types import ArenaMap, KeepOut
from finals.types import Direction, Move, Rotate

_CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "configs")


def _ko(kid, poly):
    # Canonical KeepOut (float-tuple vertices), as KeepOut.from_dict would build.
    return KeepOut(id=kid,
                   polygon_m=tuple((float(p[0]), float(p[1])) for p in poly))


def _fly(legs, start=(0.0, 0.0)):
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


# ============================================================
# ObstacleMap store + merge
# ============================================================
def test_add_and_snapshot_keep_outs():
    m = ObstacleMap()
    assert m.add_keep_out("alpha", _ko("c1", [[0, 0], [1, 0], [1, 1]]), 10.0) is True
    assert len(m) == 1
    assert [k.id for k in m.keep_outs()] == ["c1"]
    assert m.contributors() == {"c1": "alpha"}


def test_reseen_id_updates_and_records_provenance():
    m = ObstacleMap()
    m.add_keep_out("alpha", _ko("c1", [[0, 0], [1, 0], [1, 1]]), 10.0)
    # A second drone re-maps the SAME crate id -> update (not new), new provenance.
    assert m.add_keep_out("bravo", _ko("c1", [[0, 0], [2, 0], [2, 2]]), 20.0) is False
    assert len(m) == 1
    assert m.contributors() == {"c1": "bravo"}
    snap = m.snapshot(now=25.0)
    assert snap["c1"]["drone"] == "bravo"
    assert snap["c1"]["age_s"] == pytest.approx(5.0)


def test_merge_is_adds_only_static_authoritative():
    m = ObstacleMap()
    # An observed keep-out that COLLIDES on id with a static one must NOT override.
    m.add_keep_out("alpha", _ko("static1", [[9, 9], [9, 10], [10, 10]]), 0.0)
    m.add_keep_out("alpha", _ko("obs1", [[0, 0], [1, 0], [1, 1]]), 0.0)
    static = (_ko("static1", [[0, 0], [2, 0], [2, 2]]),)
    merged = m.merge(static)
    by_id = {k.id: k for k in merged}
    assert set(by_id) == {"static1", "obs1"}
    # static1 keeps the STATIC geometry (observation of that id dropped).
    assert by_id["static1"].polygon_m == ((0.0, 0.0), (2.0, 0.0), (2.0, 2.0))


def test_merge_empty_map_returns_static_unchanged():
    m = ObstacleMap()
    static = (_ko("s", [[0, 0], [1, 0], [1, 1]]),)
    assert m.merge(static) == static


@pytest.mark.parametrize("bad", [
    "not-a-keepout",
    None,
])
def test_add_non_keepout_fails_loud(bad):
    with pytest.raises(MapError):
        ObstacleMap().add_keep_out("alpha", bad, 0.0)


def test_add_degenerate_polygon_fails_loud():
    with pytest.raises(MapError, match="enclose AREA|distinct"):
        ObstacleMap().add_keep_out("alpha", _ko("line", [[0, 0], [1, 1]]), 0.0)


def test_add_bad_drone_or_ts_fails_loud():
    m = ObstacleMap()
    good = _ko("c", [[0, 0], [1, 0], [1, 1]])
    with pytest.raises(MapError):
        m.add_keep_out("", good, 0.0)
    with pytest.raises(MapError):
        m.add_keep_out("alpha", good, float("nan"))


# ============================================================
# navigate merge — the money property: an observed-only keep-out detours
# ============================================================
def _navigate(drone_cfg, arena, obstacle_map=None):
    cfg = types.SimpleNamespace(arena=arena, arena_name="t")
    return Navigate.from_config(drone_cfg, cfg, obstacle_map=obstacle_map)


def test_observed_keep_out_forces_a_detour_no_static_keep_out():
    # Empty static arena -> the baseline plan is a straight shot (0,0)->(6,0).
    arena = ArenaMap(bounds_m=(-5.0, -5.0, 12.0, 5.0), keep_out=(), pads=(),
                     lanes=(), c2_origin_m=(0.0, 0.0), c2_heading_deg=0.0)
    drone = DroneConfig(id="alpha", phases=["navigate"],
                        zone={"navigate": {"goal_ne_m": [6.0, 0.0],
                                           "inflation_m": 0.3,
                                           "max_leg_cm": 10000.0}})
    base = _navigate(drone, arena)._legs
    assert len(base) == 1                             # straight, no obstacle
    base_pts = _fly(base)

    # An OBSERVED-only crate straddling the straight line.
    obs = _ko("obs1", [[2.0, -1.0], [2.0, 1.0], [4.0, 1.0], [4.0, -1.0]])
    omap = ObstacleMap()
    omap.add_keep_out("bravo", obs, 0.0)              # a DIFFERENT drone mapped it
    merged = _navigate(drone, arena, obstacle_map=omap)._legs

    assert len(merged) >= 2                           # now detours
    merged_pts = _fly(merged)
    assert merged_pts[-1] == pytest.approx((6.0, 0.0), abs=1e-6)  # still reaches goal
    assert not _hits(merged_pts, obs.polygon_m)       # clears the observed crate
    assert _hits(base_pts, inflate_polygon(obs.polygon_m, 0.3))   # straight WOULD cut


def test_empty_map_leaves_navigate_plan_unchanged():
    arena = ArenaMap(bounds_m=(-5.0, -5.0, 12.0, 5.0), keep_out=(), pads=(),
                     lanes=(), c2_origin_m=(0.0, 0.0), c2_heading_deg=0.0)
    drone = DroneConfig(id="alpha", phases=["navigate"],
                        zone={"navigate": {"goal_ne_m": [6.0, 0.0],
                                           "inflation_m": 0.3,
                                           "max_leg_cm": 10000.0}})
    a = _navigate(drone, arena)._legs
    b = _navigate(drone, arena, obstacle_map=ObstacleMap())._legs   # empty map
    assert a == b


# ============================================================
# Full config plumbing: observed_keep_out -> _build_obstacle_map -> navigate
# ============================================================
def _followbox1_raw():
    with open(os.path.join(_CONFIG_DIR, "sitl1_followbox1.json"),
              encoding="utf-8") as f:
        return json.load(f)


def test_config_observed_keep_out_builds_a_shared_map(tmp_path):
    raw = _followbox1_raw()
    raw["observed_keep_out"] = [
        {"id": "observed_west", "polygon_m": [[1.0, -1.8], [1.0, -0.8],
                                              [3.0, -0.8], [3.0, -1.8]]}]
    p = tmp_path / "obs.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    cfg = load_config(str(p))
    assert cfg.observed_keep_out is not None
    omap = _build_obstacle_map(cfg)
    assert omap is not None
    assert "observed_west" in {k.id for k in omap.keep_outs()}
    assert omap.contributors()["observed_west"] == "operator"

    # The navigate phase built by main now sees the merged keep-out set.
    phases = _build_phases(cfg.drones[0], cfg, None, omap)
    nav = [p for p in phases if p.name == "navigate"][0]
    # Fly its legs: must clear BOTH the static crate AND the observed one.
    pts = _fly(nav._legs, start=tuple(cfg.arena.c2_origin_m))
    observed = [[1.0, -1.8], [1.0, -0.8], [3.0, -0.8], [3.0, -1.8]]
    assert not _hits(pts, tuple(map(tuple, observed)))


def test_config_duplicate_observed_id_fails_loud(tmp_path):
    raw = _followbox1_raw()
    ring = [[1.0, -1.8], [1.0, -0.8], [3.0, -0.8], [3.0, -1.8]]
    raw["observed_keep_out"] = [{"id": "dup", "polygon_m": ring},
                                {"id": "dup", "polygon_m": ring}]
    p = tmp_path / "dup.json"
    p.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(ConfigError, match="duplicate id"):
        load_config(str(p))
