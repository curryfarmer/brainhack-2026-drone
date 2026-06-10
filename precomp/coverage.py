"""Boustrophedon (lawnmower) coverage waypoint generator.

Pure functions. No drone/asyncio. Easy to unit-test and replay.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Waypoint:
    north: float          # NED north (m)
    east: float           # NED east  (m)
    down: float           # NED down  (m) — negative = up
    yaw_deg: float        # heading
    is_turn: bool = False # True when this is a row-transition waypoint (slow-down hint)

    def as_tuple(self) -> Tuple[float, float, float, float]:
        return (self.north, self.east, self.down, self.yaw_deg)


def generate_lawnmower(
    origin_north: float,
    origin_east: float,
    width_north: float,
    width_east: float,
    altitude: float,
    lane_spacing: float = 3.5,
    along_axis: str = "north",
) -> List[Waypoint]:
    """Lawnmower waypoints over a rectangular region in the NED frame.

    Args:
        origin_north / origin_east: NED coordinates of one corner of the region.
        width_north / width_east: rectangle extent. Negative values flip direction.
        altitude: positive metres above takeoff (converted internally to NED down = -altitude).
        lane_spacing: distance between adjacent sweep lanes (m).
        along_axis: "north" means sweeps run parallel to the north axis with lanes
            stepping east. "east" swaps roles.

    Returns:
        Ordered list of Waypoint. Turn waypoints (row transitions) have is_turn=True.
    """
    if along_axis not in ("north", "east"):
        raise ValueError(f"along_axis must be 'north' or 'east', got {along_axis!r}")
    if lane_spacing <= 0:
        raise ValueError("lane_spacing must be positive")

    down = -float(altitude)

    if along_axis == "north":
        sweep_len = width_north
        lane_axis_len = width_east
        sweep_yaw_a = 0.0 if sweep_len >= 0 else 180.0
        sweep_yaw_b = 180.0 if sweep_len >= 0 else 0.0
    else:  # along_axis == "east"
        sweep_len = width_east
        lane_axis_len = width_north
        sweep_yaw_a = 90.0 if sweep_len >= 0 else -90.0
        sweep_yaw_b = -90.0 if sweep_len >= 0 else 90.0

    n_lanes = max(1, int(math.ceil(abs(lane_axis_len) / lane_spacing)) + 1)
    lane_step = math.copysign(lane_spacing, lane_axis_len)

    waypoints: List[Waypoint] = []
    for i in range(n_lanes):
        lane_offset = i * lane_step
        if i == n_lanes - 1:
            # Snap last lane exactly to the far edge so we don't overshoot.
            lane_offset = lane_axis_len

        going_forward = (i % 2 == 0)
        yaw = sweep_yaw_a if going_forward else sweep_yaw_b

        if along_axis == "north":
            start_n = origin_north + (0.0 if going_forward else sweep_len)
            end_n   = origin_north + (sweep_len if going_forward else 0.0)
            e = origin_east + lane_offset
            start_wp = Waypoint(start_n, e, down, yaw, is_turn=(i > 0))
            end_wp   = Waypoint(end_n,   e, down, yaw, is_turn=False)
        else:
            start_e = origin_east + (0.0 if going_forward else sweep_len)
            end_e   = origin_east + (sweep_len if going_forward else 0.0)
            n = origin_north + lane_offset
            start_wp = Waypoint(n, start_e, down, yaw, is_turn=(i > 0))
            end_wp   = Waypoint(n, end_e,   down, yaw, is_turn=False)

        waypoints.append(start_wp)
        waypoints.append(end_wp)

    return waypoints


def filter_unvisited(
    waypoints: List[Waypoint],
    current_north: float,
    current_east: float,
    visited_radius: float = 1.0,
) -> List[Waypoint]:
    """Drop waypoints we already reached (used after a supervisor restart).

    Walks the list, skipping leading waypoints within `visited_radius` of the
    current pose. Stops dropping at the first waypoint outside the radius so we
    don't skip the rest of the mission if the drone crashed mid-route.
    """
    out: List[Waypoint] = []
    skipping = True
    for wp in waypoints:
        if skipping:
            d = math.hypot(wp.north - current_north, wp.east - current_east)
            if d < visited_radius:
                continue
            skipping = False
        out.append(wp)
    return out


if __name__ == "__main__":
    # Smoke test / visual sanity
    wps = generate_lawnmower(
        origin_north=0.0, origin_east=0.0,
        width_north=40.0, width_east=40.0,
        altitude=2.5, lane_spacing=3.5,
    )
    print(f"Generated {len(wps)} waypoints")
    for i, wp in enumerate(wps[:6]):
        print(f"  {i}: N={wp.north:.1f} E={wp.east:.1f} D={wp.down:.1f} yaw={wp.yaw_deg:.0f} turn={wp.is_turn}")
    print("  ...")
    for i, wp in enumerate(wps[-4:], start=len(wps) - 4):
        print(f"  {i}: N={wp.north:.1f} E={wp.east:.1f} D={wp.down:.1f} yaw={wp.yaw_deg:.0f} turn={wp.is_turn}")
