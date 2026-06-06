"""takeoff_demo — the first end-to-end phase and the perpetual smoke test.

Planned behavior (S4): Takeoff(height_cm from config/altitude band) ->
Hover(2) -> [Move(FORWARD, 100) -> Rotate(90)] x4 (a square) -> Land -> Done.
Requires NO position feedback and NO detection — this is both VM gate V1
(SITL) and onsite FLIGHT 1/2 (first real flight + the dead-reckoning
calibration square).

Failure branches matter as much as the happy path: a failed Move
(last_action_ok False) -> Abort with the underlying error in the reason.

Derives from: mapping_drone.py's waypoint demo intent (lines 343-355)
re-expressed in relative moves; hula_connection.py state-machine advice.

STUB — session S4.
"""
from __future__ import annotations

from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases import register_phase
from finals.types import Action

_STUB = "finals.mission.phases.takeoff_demo: session S4 — see finals/docs/module_map.md"


@register_phase
class TakeoffDemo(MissionPhase):
    name = "takeoff_demo"

    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)

    def step(self, ctx: AgentContext) -> Action:
        raise NotImplementedError(_STUB)
