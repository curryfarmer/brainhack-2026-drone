"""takeoff_demo — the first end-to-end phase and the perpetual smoke test.

Behavior: Takeoff(height_cm) -> Hover(hover_s) -> [Move(FORWARD, leg_cm) ->
Rotate(turn_deg)] x legs -> Land -> Done. Requires NO position feedback and
NO detection — this is both VM gate V1 (SITL) and onsite FLIGHT 1/2 (first
real flight + the dead-reckoning calibration square: with the defaults the
square closes, so any DR residual is pure drift/sign error).

Tunables come from config via `from_config(drone_cfg, cfg)` (the soft
construction convention main.py uses for every phase that defines it):
- `DroneConfig.zone["takeoff_demo"]` may hold any of height_cm / hover_s /
  leg_cm / legs / turn_deg; unknown keys raise ConfigError (typo guard, same
  policy as the config loader). Keys starting with "_" are ignored
  (JSON-comment convention).
- height_cm defaults from `DroneConfig.altitude_band_m * 100` when set (the
  swarm vertical-separation band IS the takeoff height), else 80 (pyhulax
  default).
Onsite rule "tune config, not code" holds: every number here is a config edit.

Failure branch (defensive): if the agent ever reports last_action_ok False
back into step(), the phase returns Abort carrying the underlying error.
Under the S4 DroneAgent policy the agent fails the drone directly on a
command error and never re-steps the phase, so this branch is belt-and-
suspenders for any future agent that reports instead of failing — it is unit
tested by constructing the AgentContext by hand.

A phase instance is single-shot: one drone, one mission, one pass through
the plan (fresh instance per mission per the MissionPhase contract).

Derives from: mapping_drone.py:343-355 (the two-waypoint demo intent),
re-expressed as relative moves. Bugs fixed in adaptation:
- mapping_drone.py's waypoints are absolute NED offsets computed from
  get_uwb_position(), whose `state` flag is read but never validated — and
  its line-129 cousin waits FOREVER for UWB lock. This phase needs no
  position source at all; it composes body-frame relative moves.
- mapping_drone.py hardcodes every distance/height inline; here every
  tunable is config-driven (the onsite "tune config, not code" rule).
- The demo's teardown lands implicitly at the end of a try-block; here Land
  is an explicit plan step and Done is an explicit control action, so the
  agent (not luck) owns the landing.

Session: S4 (implemented).
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING, List

from finals.errors import ConfigError
from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases import register_phase
from finals.types import (Abort, Action, Direction, Done, Hover, Land, Move,
                          Rotate, Takeoff)

if TYPE_CHECKING:  # type hints only — keeps the import graph minimal
    from finals.config import DroneConfig, FinalsConfig

#: Constructor keywords settable from DroneConfig.zone["takeoff_demo"].
_TUNABLES = ("height_cm", "hover_s", "leg_cm", "legs", "turn_deg")


@register_phase
class TakeoffDemo(MissionPhase):
    """Takeoff -> hover -> square (or configured polygon) -> land -> Done."""

    name = "takeoff_demo"

    def __init__(self, *, height_cm: int = 80, hover_s: float = 2.0,
                 leg_cm: int = 100, legs: int = 4, turn_deg: float = 90.0):
        # Config-shaped values are validated HERE, loudly, before any flight:
        # a bad zone dict must die at wiring time, not as a mid-air adapter
        # refusal (the adapter would refuse too — but on the ground is better).
        def _bad(key: str, value, why: str) -> ConfigError:
            return ConfigError(
                f"takeoff_demo: {key}={value!r} invalid — {why} — check "
                f'DroneConfig.zone["takeoff_demo"] (or altitude_band_m for '
                f"height_cm)")

        if not isinstance(height_cm, int) or isinstance(height_cm, bool) \
                or not height_cm > 0:
            raise _bad("height_cm", height_cm, "must be an int > 0 (cm)")
        if not isinstance(hover_s, (int, float)) or isinstance(hover_s, bool) \
                or not math.isfinite(hover_s) or hover_s < 0:
            raise _bad("hover_s", hover_s, "must be a finite number >= 0 (s)")
        if not isinstance(leg_cm, int) or isinstance(leg_cm, bool) \
                or not leg_cm > 0:
            raise _bad("leg_cm", leg_cm, "must be an int > 0 (cm)")
        if not isinstance(legs, int) or isinstance(legs, bool) or legs < 0:
            raise _bad("legs", legs, "must be an int >= 0 (0 = takeoff/"
                                     "hover/land only)")
        if not isinstance(turn_deg, (int, float)) or isinstance(turn_deg, bool) \
                or not math.isfinite(turn_deg):
            raise _bad("turn_deg", turn_deg, "must be a finite number (deg, "
                                             "+ve = CCW)")

        self.height_cm = height_cm
        self.hover_s = float(hover_s)
        self.leg_cm = leg_cm
        self.legs = legs
        self.turn_deg = float(turn_deg)

        # The hula_connection.py:46-50 state variable, as an index into a
        # precomputed plan (frozen Actions, so the plan cannot be mutated).
        self._plan: List[Action] = [
            Takeoff(height_cm=height_cm),
            Hover(duration_s=float(hover_s)),
        ]
        for _ in range(legs):
            self._plan.append(Move(direction=Direction.FORWARD,
                                   distance_cm=leg_cm))
            self._plan.append(Rotate(angle_deg=float(turn_deg)))
        self._plan.append(Land())
        self._idx = 0

    @classmethod
    def from_config(cls, drone_cfg: "DroneConfig",
                    cfg: "FinalsConfig") -> "TakeoffDemo":
        """Build from config (the main.py soft convention; `cfg` is unused
        here but keeps the factory signature uniform across phases)."""
        params = drone_cfg.zone.get("takeoff_demo", {})
        if not isinstance(params, dict):
            raise ConfigError(
                f'drone {drone_cfg.id!r}: zone["takeoff_demo"] must be an '
                f"object of tunables {sorted(_TUNABLES)}, got {params!r}")
        kwargs = {k: v for k, v in params.items() if not k.startswith("_")}
        unknown = sorted(set(kwargs) - set(_TUNABLES))
        if unknown:
            raise ConfigError(
                f'drone {drone_cfg.id!r}: zone["takeoff_demo"] unknown '
                f"key(s) {unknown} — valid keys: {sorted(_TUNABLES)} (typo?)")
        if "height_cm" not in kwargs and drone_cfg.altitude_band_m is not None:
            # The altitude band IS the takeoff height: vertical separation
            # is the swarm's primary collision guarantee, so the band wins
            # over the generic default whenever it is configured.
            kwargs["height_cm"] = int(round(drone_cfg.altitude_band_m * 100))
        return cls(**kwargs)

    def step(self, ctx: AgentContext) -> Action:
        if ctx.last_action_ok is False:
            # Defensive branch (see module docstring): never advance past a
            # failed action — fail the drone loudly with the real cause.
            return Abort(
                f"takeoff_demo[{ctx.drone_id}]: {ctx.last_action!r} failed "
                f"({ctx.last_action_error}) — aborting instead of flying the "
                f"rest of the pattern from an unknown position")
        if self._idx >= len(self._plan):
            return Done(
                f"takeoff_demo complete: takeoff {self.height_cm} cm, "
                f"hover {self.hover_s:g} s, {self.legs} x (forward "
                f"{self.leg_cm} cm, rotate {self.turn_deg:g} deg), landed")
        action = self._plan[self._idx]
        self._idx += 1
        return action
