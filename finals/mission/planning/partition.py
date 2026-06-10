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
    """Convert the OTHER drones' keep-in regions into keep-out polygons for
    `mine`'s planner, so the inflated transit corridors never overlap — spatial
    deconfliction BY CONSTRUCTION at PLAN time (the only kind a POSITION-BLIND
    drone can honour; the runtime SectorGuard stays an advisory backstop).

    Each other region becomes a KeepOut id `region_<drone_id>`; the visibility-
    graph planner then routes `mine` around every neighbour's territory. A region
    matching `mine.drone_id` is skipped (a drone is never walled out of its own
    area). Output sorted by drone_id (deterministic). Validation REUSES
    KeepOut.from_dict (>= 3 distinct finite vertices) so a malformed region fails
    LOUD exactly like a bad arena keep-out.

    NOTE: this returns the OTHER-region keep-outs only; the matching keep-IN clip
    (confining `mine`'s plan to its own polygon) is the planner's follow-on (the
    module docstring step 2) — these keep-outs alone already force disjoint
    corridors when the regions tile the arena. PURE: stdlib only.
    """
    if not isinstance(mine, DroneRegion):
        raise ValueError(
            f"partition.region_to_keep_outs: `mine` must be a DroneRegion, got "
            f"{type(mine).__name__}")
    out = []
    for other in others:
        if not isinstance(other, DroneRegion):
            raise ValueError(
                f"partition.region_to_keep_outs: every `others` entry must be a "
                f"DroneRegion, got {type(other).__name__}")
        if other.drone_id == mine.drone_id:
            continue                       # never wall a drone out of its own area
        ko = KeepOut.from_dict(
            {"id": f"region_{other.drone_id}",
             "polygon_m": list(other.keep_in_polygon_m)},
            index=other.drone_id)
        out.append(ko)
    return tuple(sorted(out, key=lambda k: k.id))
