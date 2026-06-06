"""land_on_pad — visual-servo landing on a VALID H-pad (the precision-landing
primitive).

Planned shape (S11), internal sub-states of this single phase:
- PAD_ACQUIRE: hover; pad seen in >= 3 of last 5 frames. Not found -> rotate
  scan (8x45 deg) -> bounded lateral steps -> acquire_timeout_s -> fallback.
- PAD_CENTER: requires camera pitched down (pyhulax set_camera_angle); step =
  clamp(k * offset_norm * altitude_m, min/max); deadband 10% of frame for 3
  consecutive ticks -> centered. Image-axis -> body-direction mapping behind
  a config sign-flip (verified onsite gate).
- PAD_DESCEND: confirm-before-descend — Move(DOWN, descend_step_cm) ONLY if
  centered AND pad in N-of-M recent frames; drift -> back to PAD_CENTER; pad
  lost -> ascend one step, re-acquire (max 2 retries).
- LAND_COMMIT: ToF altitude <= commit_alt_m (~0.5) -> Land (final blind drop;
  the marker leaves the FOV anyway).
- Fallback: total landing wall budget exceeded at ANY stage -> Land in place
  + loud UNVERIFIED_LANDING event. Never hover until the battery dies.

PadClassifier seam (briefing-dependent): ArucoPadClassifier primary
(valid_marker_ids from config); shape-based ("H" on circle) classifier stub.
The pure servo math (compute_centering_step, descend_gate) is unit-tested
with synthetic candidate sequences; SITL-tested with a scripted classifier.

Serialized landings: the orchestrator grants ONE landing slot at a time —
descending crosses the other drones' altitude bands.

Derives from: ArUco pattern of potential_detection_targets.py (audited);
relative-move vocabulary only.

STUB — session S11 (post-briefing; servo math may land earlier if time allows).
"""
from __future__ import annotations

from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases import register_phase
from finals.types import Action

_STUB = "finals.mission.phases.land_on_pad: session S11 — see finals/docs/module_map.md"


@register_phase
class LandOnPad(MissionPhase):
    name = "land_on_pad"

    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)

    def step(self, ctx: AgentContext) -> Action:
        raise NotImplementedError(_STUB)
