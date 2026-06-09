"""navigate — open-loop transit phase: fly C2 -> pad-vicinity over planner Legs.

Planned shape (S11, NAV-5): from_config resolves the goal (a pad_id or a coord)
from the ArenaMap + DroneConfig.zone["navigate"], calls the visibility-graph
planner (finals.mission.planning.visibility_graph.plan) for a frozen list[Leg],
and step() flies each leg OPEN-LOOP — Rotate to the leg's ABSOLUTE compass
heading (re-zeroing yaw creep against the trusted compass) via _servo, then
Move(FORWARD, distance_cm) — advancing on last_action_ok, Abort on a failed
action, Done after the final leg. Position stays PositionQuality.NONE; the
DeadReckoner pose is ADVISORY (sector geofence) only, never a control input.

Derives from: search.py SentryScan (the precomputed-plan + from_config +
no-op-trap template); planning.visibility_graph (NAV-1); _servo (NAV-3).

STUB — session S11 (NAV-5). See finals/docs/module_map.md.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases import register_phase
from finals.types import Action

if TYPE_CHECKING:  # type hints only — no import-time coupling to config
    from finals.config import DroneConfig, FinalsConfig

_STUB = ("finals.mission.phases.navigate: session S11 (NAV-5) — "
         "see finals/docs/module_map.md")


@register_phase
class Navigate(MissionPhase):
    name = "navigate"

    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)

    @classmethod
    def from_config(cls, drone_cfg: "DroneConfig",
                    cfg: "FinalsConfig") -> "Navigate":
        raise NotImplementedError(_STUB)

    def step(self, ctx: AgentContext) -> Action:
        raise NotImplementedError(_STUB)
