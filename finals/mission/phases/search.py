"""search — SentryScan (default) + OpenLoopLawnmower (config-gated upgrade).

Planned behavior (S8):
- SentryScan (DEFAULT): hover at the assigned altitude band, then repeat
  [Hover(scan_dwell_s) -> Rotate(45)] x8. For a MOVING convoy under
  zero-trust positioning, a stationary rotating observer over the route is
  the highest-yield, lowest-risk searcher: zero translational drift, stable
  hover frames (best for YOLO/ArUco), the convoy re-crosses the footprint.
- OpenLoopLawnmower: a pure planner emits [Rotate, Move(FORWARD, lane_cm),
  Hover(scan_pause)] primitive lists over the drone's config zone. Yaw error
  compounds per turn — ENABLED ONLY after onsite gate E measures relative-move
  and rotation accuracy (command 1 m + 4x90 deg, tape-measure the closure).
- Both are pluggable via config (DroneConfig.zone / phases); selection is a
  config edit on briefing day.

Derives from: hover/rotate primitives (pyhulax docs); root coverage.py's
lawnmower logic informs the lane math but its Waypoint output is position-
based — reused ONLY if a MEASURED-quality position source ever materializes.

STUB — session S8.
"""
from __future__ import annotations

from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases import register_phase
from finals.types import Action

_STUB = "finals.mission.phases.search: session S8 — see finals/docs/module_map.md"


@register_phase
class SentryScan(MissionPhase):
    name = "sentry_scan"

    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)

    def step(self, ctx: AgentContext) -> Action:
        raise NotImplementedError(_STUB)


@register_phase
class OpenLoopLawnmower(MissionPhase):
    name = "lawnmower"

    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)

    def step(self, ctx: AgentContext) -> Action:
        raise NotImplementedError(_STUB)
