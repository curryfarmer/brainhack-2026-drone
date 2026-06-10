"""takeoff — minimal liftoff phase that STAYS AIRBORNE (no Land).

Behavior: Takeoff(height_cm) -> Done. Unlike takeoff_demo (which ALWAYS ends
with Land, even with legs=0) this phase takes the drone off and HOLDS it
airborne so a following transit phase can fly. It is the missing first leg of
the Challenge-2A landing sequence `[takeoff, navigate, land_on_pad]`:
takeoff_demo cannot be that first leg because it lands at the end, and
navigate ASSUMES it is already airborne (it never issues Takeoff/Land).

Tunables come from config via `from_config(drone_cfg, cfg)` (the soft
construction convention main.py uses for every phase that defines it):
- `DroneConfig.zone["takeoff"]` may hold `height_cm`; unknown keys raise
  ConfigError (typo guard, same policy as the config loader). Keys starting
  with "_" are ignored (JSON-comment convention).
- height_cm defaults from `DroneConfig.altitude_band_m * 100` when set (the
  shared _height_from_band rule — search.py / takeoff_demo), else 80 (the
  pyhulax default). For the Challenge-2A LANDING mission the band is NOT a
  separation mechanism (the ~1.1 m ceiling kills altitude bands — see
  finals/guards.py SectorGuard / orchestrator deconfliction rationale); it is
  reused here only as a convenient per-drone takeoff height knob.
Onsite rule "tune config, not code" holds: every number here is a config edit.

Failure branch (defensive): if the agent ever reports last_action_ok False
back into step(), the phase returns Abort carrying the underlying error.
Under the S4 DroneAgent policy the agent fails the drone directly on a command
error and never re-steps the phase, so this branch is belt-and-suspenders for
any future agent that reports instead of failing — it is unit tested by
constructing the AgentContext by hand.

A phase instance is single-shot: one drone, one mission, one pass through the
plan (fresh instance per mission per the MissionPhase contract).

Derives from: takeoff_demo.py (the precomputed-plan + from_config + no-op-trap
+ Abort-on-failure template) and search.py (_zone_kwargs / _height_from_band /
_check_height_cm shared helpers). The ONLY difference from a legs=0 takeoff_demo
is the deliberate ABSENCE of the trailing Land — that is the whole point of
this phase.

Implemented — session S11 (NAV-8). See finals/docs/module_map.md.
"""
from __future__ import annotations

from typing import TYPE_CHECKING, List

from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases import register_phase
from finals.mission.phases.search import (_check_height_cm, _height_from_band,
                                          _zone_kwargs)
from finals.types import Abort, Action, Done, Takeoff

if TYPE_CHECKING:  # type hints only — keeps the import graph minimal
    from finals.config import DroneConfig, FinalsConfig


@register_phase
class TakeoffHold(MissionPhase):
    """Takeoff(height_cm) -> Done. Takes off and STAYS AIRBORNE (no Land)."""

    name = "takeoff"

    #: Constructor keywords settable from DroneConfig.zone["takeoff"].
    _TUNABLES = ("height_cm",)

    def __init__(self, *, height_cm: int = 80):
        # Config-shaped value validated HERE, loudly, before any flight — a bad
        # height must die at wiring time, not as a mid-air adapter refusal
        # (the same no-op-trap policy as takeoff_demo / search).
        _check_height_cm("takeoff", height_cm)
        self.height_cm = height_cm

        # The hula_connection.py:46-50 state variable as an index into a
        # precomputed plan (frozen Actions, so the plan cannot be mutated).
        # Deliberately NO trailing Land — this phase leaves the drone airborne.
        self._plan: List[Action] = [Takeoff(height_cm=height_cm)]
        self._idx = 0

    @classmethod
    def from_config(cls, drone_cfg: "DroneConfig",
                    cfg: "FinalsConfig") -> "TakeoffHold":
        """Build from config (`cfg` unused — keeps the factory signature
        uniform across phases). The altitude band IS the takeoff height when
        set and height_cm was not given explicitly (the shared
        _height_from_band rule)."""
        kwargs = _zone_kwargs(drone_cfg, "takeoff", cls._TUNABLES)
        _height_from_band(kwargs, drone_cfg)
        return cls(**kwargs)

    def step(self, ctx: AgentContext) -> Action:
        if ctx.last_action_ok is False:
            # Defensive branch (see module docstring): never advance past a
            # failed action — fail the drone loudly with the real cause.
            return Abort(
                f"takeoff[{ctx.drone_id}]: {ctx.last_action!r} failed "
                f"({ctx.last_action_error}) — aborting instead of continuing "
                f"the mission from an unknown attitude")
        if self._idx >= len(self._plan):
            return Done(
                f"takeoff complete: airborne at {self.height_cm} cm, HOLDING "
                f"(no land — a following phase flies from here)")
        action = self._plan[self._idx]
        self._idx += 1
        return action
