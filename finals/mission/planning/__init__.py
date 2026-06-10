"""finals.mission.planning — pure 2-D path-planning for the Challenge-2A landing
navigation (open-loop transit to a landing pad while avoiding obstacles).

Subpackage layout (S11):
- types.py            frozen map/geometry contracts (Leg, KeepOut, LandingPad, ArenaMap)
- polygon_tools.py    inflate / segment-intersect / point-in-polygon predicates (NAV-1)
- visibility_graph.py plan(start, goal, arena, ...) -> list[Leg] (NAV-1)

All PURE stdlib (numpy imported lazily by NAV-1 if needed — the conventions scan
bans a top-level numpy import in pure modules); never imports an SDK.

Session: S11 (NAV-0 lands the package + frozen contracts).
"""
from __future__ import annotations

from finals.mission.planning.types import ArenaMap, KeepOut, LandingPad, Leg

__all__ = ["ArenaMap", "KeepOut", "LandingPad", "Leg"]
