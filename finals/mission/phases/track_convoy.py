"""track_convoy — investigate/track behavior on the Sighting stream.

BRIEFING-DEPENDENT: whether tracking (vs. per-sighting logging) scores at all
is unknown. This stub stays a stub until the briefing text lands; everything
it would need (Sighting stream via AgentContext.sightings, bearing_deg,
Move/Rotate vocabulary) already exists in the contracts.

Planned shape (S11): triggered from search on an N-of-M persistent sighting
matching config classes/confidence; rotate to center the target bearing,
small bounded approach moves, give up after lost_timeout_s or
investigate_budget_s and return Done (back to search via the phase queue).
All tunables in config for briefing-day edits.

STUB — session S11 (post-briefing).
"""
from __future__ import annotations

from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases import register_phase
from finals.types import Action

_STUB = "finals.mission.phases.track_convoy: session S11 — see finals/docs/module_map.md"


@register_phase
class TrackConvoy(MissionPhase):
    name = "track_convoy"

    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)

    def step(self, ctx: AgentContext) -> Action:
        raise NotImplementedError(_STUB)
