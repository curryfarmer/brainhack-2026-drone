"""Pins the NAV map-sensing CONTRACT + stubs (mission/planning/map_sensing.py).

Kept a stub by design ("stub first"): full SLAM is infeasible on a position-blind
+ down-looking-cam airframe; the hand-authored arena.json (with the omit-keep_out
straight-line fallback) is the active map source. These tests fix the two fill
paths' signatures and assert each stub fails loud with the module_map pointer.
Pure — stdlib + pytest only.
"""
from __future__ import annotations

import pytest

from finals.mission.planning.map_sensing import (keep_outs_from_overhead_corners,
                                                 position_fix_from_marker)


def test_position_fix_from_marker_is_a_documented_stub():
    with pytest.raises(NotImplementedError, match="module_map.md"):
        position_fix_from_marker(marker_world_m=(5.0, 5.0), bearing_deg=10.0,
                                 ground_range_m=1.2, drone_yaw_deg=90.0)


def test_keep_outs_from_overhead_corners_is_a_documented_stub():
    corners = {"crate_a": ((4.0, 2.0), (5.0, 2.0), (5.0, 3.0), (4.0, 3.0))}
    with pytest.raises(NotImplementedError, match="module_map.md"):
        keep_outs_from_overhead_corners(corners)
