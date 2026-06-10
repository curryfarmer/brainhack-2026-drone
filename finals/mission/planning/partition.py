"""partition.py — per-drone map partition for SPATIAL deconfliction.

STUB (session S11, NAV map-partition). Contract + seam only; the ADVISORY
SectorGuard (finals/guards.py) is the ACTIVE deconfliction-by-space mechanism
until this is filled. Kept as a stub by explicit choice ("keep as stub first").

Idea (user, 2026-06-10): one overall map (ArenaMap holds the WHOLE arena —
bounds, every keep-out, every pad, lanes, the C2 frame), partitioned so each
drone "covers a certain area only".

The binding constraint: the HULA drones are POSITION-BLIND
(PositionQuality.NONE — reliable altitude + heading + battery, but NO horizontal
XY). So an area assignment can NEVER be closed-loop enforced in flight. A real
partition is therefore enforced at PLAN TIME:

  - each drone's visibility-graph plan is confined to its OWN region (keep-in);
  - every OTHER drone's region becomes an extra keep-out for this drone, so the
    inflated transit corridors never overlap -> spatial separation BY
    CONSTRUCTION (the inflation margin absorbs the open-loop dead-reckon drift);
  - pads are assigned by region;
  - the dead-reckon SectorGuard stays as the ADVISORY runtime backstop (it can
    only ever observe an ESTIMATED pose, never gate control).

Filling this stub (a future session):
  1. add an optional region polygon per drone (arena.json or DroneConfig.zone),
     validated loud like the keep-outs (NAV-2 from_dict pattern);
  2. implement region_to_keep_outs() below (and a keep-in clip for the planner);
  3. navigate.from_config passes the drone's region + the derived keep-outs into
     visibility_graph.plan;
  4. test that two drones' planned (inflated) corridors are disjoint.

See finals/docs/module_map.md (S11 NAV map-partition row). PURE: stdlib only.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from finals.mission.planning.types import KeepOut, Point


@dataclass(frozen=True)
class DroneRegion:
    """The arena sub-area one drone is confined to (a keep-IN region).

    drone_id matches DroneConfig.id. keep_in_polygon_m is a ring of
    (north_m, east_m) vertices (same frame + winding freedom as KeepOut). A
    drone's transit plan must stay INSIDE this polygon; every OTHER drone's
    region becomes a keep-out for this drone (see region_to_keep_outs).
    """

    drone_id: str
    keep_in_polygon_m: Tuple[Point, ...]


def region_to_keep_outs(mine: DroneRegion,
                        others: Tuple[DroneRegion, ...]) -> Tuple[KeepOut, ...]:
    """STUB (session S11, NAV map-partition). Convert the OTHER drones' keep-in
    regions into keep-out polygons for `mine`'s planner, so planned corridors
    never overlap.

    Not implemented: kept as a stub by design — the advisory SectorGuard is the
    active spatial-deconfliction mechanism for now. See finals/docs/module_map.md
    for how to fill it (the module docstring lists the four steps).
    """
    raise NotImplementedError(
        "finals.mission.planning.partition.region_to_keep_outs: session S11 "
        "(NAV map-partition) — kept as a stub by design; advisory SectorGuard "
        "is the active spatial deconfliction. See finals/docs/module_map.md")
