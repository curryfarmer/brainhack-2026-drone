"""NAV-2 tests for the coordinate-frame helper (mission/planning/frame.py).

Pins discord_to_ned against hand-computed values for several origins +
headings (including non-zero c2_heading_deg and the dead_reckon spot check),
malformed-input loud failure, and the advisory sector predicate over
hand-picked in/out points. Pure — math/stdlib only, no SDK/cv2/numpy.

Source: finals/mission/planning/frame.py; frame == flight/dead_reckon.py
(psi_NED = -yaw_deg, CCW+, 0 = +north).
"""
from __future__ import annotations

import math

import pytest

from finals.errors import ConfigError
from finals.mission.planning.frame import (bearing_from_c2_deg,
                                          discord_to_ned, in_sector)


def _close(a, b, tol=1e-9):
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol


# ---- discord_to_ned: heading 0 == identity (local frame == arena frame) ----
def test_heading_zero_is_offset_only():
    # facing +north: forward -> +north, right -> +east.
    assert _close(discord_to_ned((3.0, 2.0), (0.0, 0.0), 0.0), (3.0, 2.0))
    # origin translation only.
    assert _close(discord_to_ned((1.0, 1.0), (5.0, 5.0), 0.0), (6.0, 6.0))
    # right-only / forward-only decompose cleanly.
    assert _close(discord_to_ned((0.0, 4.0), (0.0, 0.0), 0.0), (0.0, 4.0))
    assert _close(discord_to_ned((4.0, 0.0), (0.0, 0.0), 0.0), (4.0, 0.0))


# ---- heading +90 = facing WEST (CCW), the dead_reckon spot check -----------
def test_heading_90_faces_west():
    # forward 2 m at heading +90 -> moves -east; right points +north.
    assert _close(discord_to_ned((2.0, 0.0), (0.0, 0.0), 90.0), (0.0, -2.0))
    assert _close(discord_to_ned((0.0, 1.0), (0.0, 0.0), 90.0), (1.0, 0.0))
    # combined + origin: dN = right, dE = -forward.
    assert _close(discord_to_ned((2.0, 3.0), (1.0, 1.0), 90.0), (4.0, -1.0))


def test_heading_180_flips_both():
    assert _close(discord_to_ned((3.0, 2.0), (0.0, 0.0), 180.0), (-3.0, -2.0))


def test_heading_360_equals_0():
    a = discord_to_ned((3.0, 2.0), (1.0, 1.0), 360.0)
    b = discord_to_ned((3.0, 2.0), (1.0, 1.0), 0.0)
    assert _close(a, b)


def test_heading_minus_90_faces_east():
    # heading -90 (CW from north) faces EAST: forward -> +east, right -> -north.
    assert _close(discord_to_ned((2.0, 0.0), (0.0, 0.0), -90.0), (0.0, 2.0))


def test_heading_45_decomposes():
    s = math.sqrt(2) / 2.0
    # forward 1 at +45: dN = cos45 = s, dE = -sin45 = -s.
    assert _close(discord_to_ned((1.0, 0.0), (0.0, 0.0), 45.0), (s, -s))


# ---- discord_to_ned malformed input fails loud -----------------------------
@pytest.mark.parametrize("bad", [
    [1.0],                # too short
    [1.0, 2.0, 3.0],      # too long
    [1.0, "east"],        # non-number
    [1.0, True],          # bool sneaking in as 1
    [1.0, float("nan")],  # non-finite
    "north,east",         # not a sequence pair
    None,
])
def test_discord_to_ned_bad_coord_raises(bad):
    with pytest.raises(ConfigError, match="coord"):
        discord_to_ned(bad, (0.0, 0.0), 0.0)


def test_discord_to_ned_bad_origin_raises():
    with pytest.raises(ConfigError, match="c2_origin_m"):
        discord_to_ned((1.0, 1.0), [0.0], 0.0)


def test_discord_to_ned_bad_heading_raises():
    with pytest.raises(ConfigError, match="c2_heading_deg"):
        discord_to_ned((1.0, 1.0), (0.0, 0.0), float("inf"))


# ---- bearing_from_c2_deg cardinal directions -------------------------------
def test_bearing_cardinals():
    o = (0.0, 0.0)
    assert bearing_from_c2_deg((1.0, 0.0), o) == 0.0      # due north
    assert bearing_from_c2_deg((-1.0, 0.0), o) == 180.0   # due south
    assert bearing_from_c2_deg((0.0, 1.0), o) == -90.0    # due east (CW)
    assert bearing_from_c2_deg((0.0, -1.0), o) == 90.0    # due west (CCW)
    assert bearing_from_c2_deg(o, o) == 0.0               # at origin -> 0


# ---- in_sector advisory predicate ------------------------------------------
def test_sector_includes_centre_excludes_outside():
    o = (0.0, 0.0)
    # A wedge centred WEST (+90), +/- 30 deg. A point due west is dead centre.
    assert in_sector((0.0, -5.0), o, sector_center_deg=90.0,
                     sector_half_width_deg=30.0) is True
    # A point due north (bearing 0) is 90 deg off centre -> outside a 30 wedge.
    assert in_sector((5.0, 0.0), o, sector_center_deg=90.0,
                     sector_half_width_deg=30.0) is False


def test_sector_boundary_is_inclusive():
    o = (0.0, 0.0)
    # Wedge centred north (0), half-width 90: a point due east is at bearing
    # -90, exactly on the edge -> inside (closed boundary).
    assert in_sector((0.0, 1.0), o, 0.0, 90.0) is True
    # Just past the edge (bearing slightly more than 90 off) -> outside.
    assert in_sector((-0.001, 1.0), o, 0.0, 90.0) is False


def test_sector_wraps_across_180():
    o = (0.0, 0.0)
    # Wedge centred at +170, width 30 spans into -180/+180: a point due south
    # (bearing 180) is 10 deg off centre -> inside.
    assert in_sector((-1.0, 0.0), o, sector_center_deg=170.0,
                     sector_half_width_deg=30.0) is True


def test_sector_full_circle_always_true():
    o = (0.0, 0.0)
    for pt in [(1.0, 0.0), (0.0, 1.0), (-1.0, -1.0), (0.0, -3.0)]:
        assert in_sector(pt, o, 0.0, 180.0) is True


def test_sector_origin_point_always_inside():
    # The shared origin has no bearing -> treated as inside any sector.
    assert in_sector((0.0, 0.0), (0.0, 0.0), 90.0, 5.0) is True


def test_sector_bad_half_width_raises():
    with pytest.raises(ConfigError, match="sector_half_width_deg"):
        in_sector((1.0, 0.0), (0.0, 0.0), 0.0, -5.0)


def test_sector_bad_center_raises():
    with pytest.raises(ConfigError, match="sector_center_deg"):
        in_sector((1.0, 0.0), (0.0, 0.0), float("nan"), 30.0)


def test_sector_delta_wraps_across_the_180_seam():
    # center=-170, half=30 -> wedge spans [-200,-140] == [160,180] U [-180,-140].
    # A point at bearing +175 is INSIDE only via the INNER wrap180 on the delta:
    # wrap180(175 - (-170)) = wrap180(345) = -15, |−15| = 15 <= 30. A mutant that
    # drops that inner wrap computes |345| = 345 > 30 -> WRONGLY outside. The
    # existing +170/south test never wraps (delta 10), so it does not catch this.
    o = (0.0, 0.0)
    pt175 = (math.cos(math.radians(175.0)), -math.sin(math.radians(175.0)))
    assert bearing_from_c2_deg(pt175, o) == pytest.approx(175.0, abs=1e-6)
    assert in_sector(pt175, o, sector_center_deg=-170.0,
                     sector_half_width_deg=30.0) is True
    # control: bearing +100 is genuinely outside the same wedge (delta 90).
    pt100 = (math.cos(math.radians(100.0)), -math.sin(math.radians(100.0)))
    assert in_sector(pt100, o, sector_center_deg=-170.0,
                     sector_half_width_deg=30.0) is False
