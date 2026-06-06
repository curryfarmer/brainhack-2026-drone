"""DeadReckoner — pure 2D pose + yaw integration of COMPLETED actions.

Planned surface (S3):
- DRPose(north_m, east_m, alt_m, yaw_deg) frozen dataclass.
- DeadReckoner: note_action_complete(action: Action, yaw_after_deg) integrates
  Move through the current yaw (body frame -> world), Rotate updates yaw,
  Takeoff/Land set altitude, resets on Land. Pure math, no I/O.
- Quality is ALWAYS PositionQuality.DEAD_RECKONING: drift compounds per move;
  the estimate annotates sightings and feeds GeofenceLite (advisory) but is
  NEVER used for closed-loop control decisions.

Derives from: the body->NED yaw-rotation math of detection_to_world.py /
mapping_drone.py, reduced to relative-move integration. Shared with
MockAdapter's simulated pose so the math has a single source of truth.

STUB — session S3.
"""
from __future__ import annotations

_STUB = "finals.flight.dead_reckon: session S3 — see finals/docs/module_map.md"


class DeadReckoner:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)
