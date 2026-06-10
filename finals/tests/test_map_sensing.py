"""finals.mission.planning.map_sensing — WS-6 map-sensing levers (pure geometry).

Lever A (keep_outs_from_overhead_corners): operator-tapped corner rings -> validated
KeepOuts, reusing the arena keep-out rule (>= 3 distinct vertices) so a mistap fails
loud. Lever C (position_fix_from_marker): a known-coord marker decode -> the drone's
absolute (north,east), the drift-reset fix. Hand-computed fixtures pin the geometry to
the project heading convention (visibility_graph: dN=r*cos b, dE=-r*sin b). PURE.
"""
from __future__ import annotations

import math

import pytest

from finals.errors import ConfigError
from finals.mission.planning.map_sensing import (keep_outs_from_overhead_corners,
                                                 position_fix_from_marker)


# ============================================================
# Lever C — position_fix_from_marker (absolute bearing convention)
# ============================================================
def test_fix_marker_due_north_recovers_origin():
    # Marker 10 m due NORTH of a drone at origin -> absolute bearing 0, range 10.
    assert position_fix_from_marker((10.0, 0.0), 0.0, 10.0) == \
        pytest.approx((0.0, 0.0), abs=1e-9)


def test_fix_marker_due_east_recovers_origin():
    # Marker 5 m due EAST (north 0, east 5). East is absolute bearing -90 in the
    # CCW+ compass convention (dE = -r*sin b > 0 -> sin b < 0 -> b = -90).
    assert position_fix_from_marker((0.0, 5.0), -90.0, 5.0) == \
        pytest.approx((0.0, 0.0), abs=1e-9)


def test_fix_general_3_4_5_triangle():
    # Marker at (north 3, east 4) from a drone at origin: range 5, bearing
    # atan2(-4, 3) deg. The solve must return the drone back at the origin.
    b = math.degrees(math.atan2(-4.0, 3.0))
    assert position_fix_from_marker((3.0, 4.0), b, 5.0) == \
        pytest.approx((0.0, 0.0), abs=1e-9)


def test_fix_translates_with_known_marker():
    # Same relative geometry but the marker is at (12, 4): the recovered drone
    # pose shifts by exactly the marker translation -> (9, 0).
    b = math.degrees(math.atan2(-4.0, 3.0))
    assert position_fix_from_marker((12.0, 4.0), b, 5.0) == \
        pytest.approx((9.0, 0.0), abs=1e-9)


@pytest.mark.parametrize("marker,bearing,rng", [
    ((0.0, 0.0), float("nan"), 5.0),
    ((0.0, 0.0), float("inf"), 5.0),
    ((0.0, 0.0), 0.0, -1.0),               # negative range
    ((0.0, 0.0), 0.0, float("nan")),
    (("a", 0.0), 0.0, 5.0),                # non-numeric marker coord
    ((0.0,), 0.0, 5.0),                    # wrong-length marker
])
def test_fix_fails_loud_on_bad_input(marker, bearing, rng):
    with pytest.raises(ValueError):
        position_fix_from_marker(marker, bearing, rng)


# ============================================================
# Lever A — keep_outs_from_overhead_corners
# ============================================================
def test_corners_build_one_keep_out():
    kos = keep_outs_from_overhead_corners(
        {"crate": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]})
    assert len(kos) == 1
    assert kos[0].id == "crate"
    assert kos[0].polygon_m == ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))


def test_corners_sorted_by_id_deterministic():
    kos = keep_outs_from_overhead_corners({
        "z": [[2, 0], [3, 0], [3, 1]],
        "a": [[0, 0], [1, 0], [1, 1]],
        "m": [[5, 5], [6, 5], [6, 6]],
    })
    assert [k.id for k in kos] == ["a", "m", "z"]


def test_corners_degenerate_polygon_fails_loud():
    # Only 2 distinct vertices -> a line, not an obstacle. The arena rule (reused)
    # must reject it loudly, never silently drop the would-be keep-out.
    with pytest.raises(ConfigError, match="distinct"):
        keep_outs_from_overhead_corners({"bad": [[0.0, 0.0], [1.0, 1.0]]})


def test_corners_non_dict_fails_loud():
    with pytest.raises(ValueError, match="dict"):
        keep_outs_from_overhead_corners([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
