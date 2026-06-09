"""visibility_graph — PURE visibility-graph A* path planner (Challenge 2A).

plan(start_m, goal_m, arena, inflation_m, max_leg_cm) inflates the keep-out
polygons by inflation_m, builds a visibility graph over {start, goal, inflated
corners}, A*-searches the shortest collision-free polyline, then converts it to
a list[Leg] (absolute heading_deg CCW+, distance_cm) — subdividing any leg
longer than max_leg_cm so cumulative open-loop drift stays under the inflation
margin. Raises PlanningError (NAV-1 adds it to finals.errors) when no
collision-free path exists or the goal sits inside a keep-out.

Pure stdlib (numpy lazily if NAV-1 needs it). Consumes finals.mission.planning
.types + polygon_tools.

STUB — session S11 (NAV-1). See finals/docs/module_map.md.
"""
from __future__ import annotations

from typing import List, Tuple

from finals.mission.planning.types import ArenaMap, Leg

_STUB = ("finals.mission.planning.visibility_graph: session S11 (NAV-1) — "
         "see finals/docs/module_map.md")

Point = Tuple[float, float]  # (north_m, east_m)


def plan(start_m: Point, goal_m: Point, arena: ArenaMap, inflation_m: float,
         max_leg_cm: float) -> List[Leg]:
    """Collision-free transit plan start_m->goal_m as an ordered list of Legs. (NAV-1)"""
    raise NotImplementedError(_STUB)
