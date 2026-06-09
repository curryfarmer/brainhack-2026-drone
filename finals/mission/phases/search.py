"""search — SentryScan (default) + OpenLoopLawnmower (config-gated upgrade).

Both are PURE MissionPhases (no I/O, no SDK, no position feedback): they emit a
precomputed list of frozen Actions, exactly like takeoff_demo. Detection is NOT
the phase's job — the PerceptionLoop runs as a parallel task and logs sightings
to the bus / sightings.csv; the phase only keeps the drone over the route long
enough for the convoy to be seen.

- SentryScan (DEFAULT): Takeoff -> repeat [Hover(dwell_s), Rotate(step_deg)] for
  `revolutions` full turns -> Land -> Done. For a MOVING convoy under zero-trust
  positioning, a stationary rotating observer over the route is the highest-yield,
  lowest-risk searcher: zero translational drift, stable hover frames (best for
  ArUco/YOLO), the convoy re-crosses the footprint. Defaults cover >= 1 convoy lap
  (sim/check_detection.py samples ~40 s for one lap).
- OpenLoopLawnmower (config-gated, OFF by default): a body-frame boustrophedon of
  [Move(FORWARD, leg), Hover(scan_pause)] lanes with U-turns shifted by lane_cm.
  OPEN-LOOP — yaw error compounds per turn, so this is ENABLED ONLY after onsite
  gate E measures relative-move + rotation accuracy. It deliberately does NOT use
  root coverage.py's Waypoint math: that output is absolute-position-based and is
  reused only if a MEASURED-quality position source ever materializes (it has not).

Both register in PHASE_REGISTRY and are selected BY NAME from config
(DroneConfig.phases); tunables come from DroneConfig.zone[<name>] via from_config
(the main.py soft-construction convention) — briefing-day edits are config, not code.

Derives from: takeoff_demo.py (the precomputed-plan + from_config pattern);
hover/rotate primitives (pyhulax docs). Session: S8.
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


def _height_from_band(kwargs: dict, drone_cfg: "DroneConfig") -> None:
    """Shared from_config rule: the altitude band IS the takeoff height when set
    and height_cm was not given explicitly (vertical separation is the swarm's
    primary collision guarantee, so the band wins over the generic default)."""
    if "height_cm" not in kwargs and drone_cfg.altitude_band_m is not None:
        kwargs["height_cm"] = int(round(drone_cfg.altitude_band_m * 100))


def _zone_kwargs(drone_cfg: "DroneConfig", phase_name: str,
                 tunables: tuple) -> dict:
    """DroneConfig.zone[phase_name] -> validated kwargs (drops _comment keys,
    rejects typos) — the same policy as takeoff_demo.from_config."""
    params = drone_cfg.zone.get(phase_name, {})
    if not isinstance(params, dict):
        raise ConfigError(
            f"drone {drone_cfg.id!r}: zone[{phase_name!r}] must be an object of "
            f"tunables {sorted(tunables)}, got {params!r}")
    kwargs = {k: v for k, v in params.items() if not k.startswith("_")}
    unknown = sorted(set(kwargs) - set(tunables))
    if unknown:
        raise ConfigError(
            f"drone {drone_cfg.id!r}: zone[{phase_name!r}] unknown key(s) "
            f"{unknown} — valid keys: {sorted(tunables)} (typo?)")
    return kwargs


def _check_height_cm(name: str, height_cm) -> None:
    if (not isinstance(height_cm, int) or isinstance(height_cm, bool)
            or not height_cm > 0):
        raise ConfigError(
            f"{name}: height_cm={height_cm!r} invalid — must be an int > 0 (cm) "
            f'— check zone["{name}"] (or altitude_band_m for height_cm)')


@register_phase
class SentryScan(MissionPhase):
    """Takeoff -> [Hover, Rotate] x (revolutions full turns) -> Land -> Done."""

    name = "sentry_scan"

    #: Constructor keywords settable from DroneConfig.zone["sentry_scan"].
    _TUNABLES = ("height_cm", "dwell_s", "step_deg", "revolutions")

    def __init__(self, *, height_cm: int = 80, dwell_s: float = 2.0,
                 step_deg: float = 45.0, revolutions: float = 3.0):
        # Config-shaped values are validated HERE, loudly, before any flight —
        # a no-op searcher (0 dwell, 0 step, 0 revolutions) is a config trap that
        # must die on the ground, not waste the mission orbiting nothing.
        def _bad(key: str, value, why: str) -> ConfigError:
            return ConfigError(
                f"sentry_scan: {key}={value!r} invalid — {why} — check "
                f'zone["sentry_scan"] (or altitude_band_m for height_cm)')

        _check_height_cm("sentry_scan", height_cm)
        if (not isinstance(dwell_s, (int, float)) or isinstance(dwell_s, bool)
                or not math.isfinite(dwell_s) or dwell_s <= 0):
            raise _bad("dwell_s", dwell_s, "must be a finite number > 0 (s) — "
                                           "the per-look observation dwell")
        if (not isinstance(step_deg, (int, float)) or isinstance(step_deg, bool)
                or not math.isfinite(step_deg) or step_deg == 0):
            raise _bad("step_deg", step_deg, "must be a finite non-zero number "
                                             "(deg, +ve = CCW) — 0 never turns")
        if (not isinstance(revolutions, (int, float))
                or isinstance(revolutions, bool)
                or not math.isfinite(revolutions) or revolutions <= 0):
            raise _bad("revolutions", revolutions,
                       "must be a finite number > 0 (full turns to scan)")

        self.height_cm = height_cm
        self.dwell_s = float(dwell_s)
        self.step_deg = float(step_deg)
        self.revolutions = float(revolutions)
        # Steps to complete `revolutions` full turns at |step_deg| per step.
        self.steps = max(1, int(round(360.0 * self.revolutions
                                      / abs(self.step_deg))))

        self._plan: List[Action] = [Takeoff(height_cm=height_cm)]
        for _ in range(self.steps):
            self._plan.append(Hover(duration_s=self.dwell_s))
            self._plan.append(Rotate(angle_deg=self.step_deg))
        self._plan.append(Land())
        self._idx = 0

    @classmethod
    def from_config(cls, drone_cfg: "DroneConfig",
                    cfg: "FinalsConfig") -> "SentryScan":
        """Build from config (`cfg` unused — keeps the factory signature uniform
        across phases)."""
        kwargs = _zone_kwargs(drone_cfg, "sentry_scan", cls._TUNABLES)
        _height_from_band(kwargs, drone_cfg)
        return cls(**kwargs)

    def step(self, ctx: AgentContext) -> Action:
        if ctx.last_action_ok is False:
            # Defensive: never advance past a failed action (the agent already
            # fails the drone on a command error; belt-and-suspenders).
            return Abort(
                f"sentry_scan[{ctx.drone_id}]: {ctx.last_action!r} failed "
                f"({ctx.last_action_error}) — aborting instead of scanning from "
                f"an unknown attitude")
        if self._idx >= len(self._plan):
            return Done(
                f"sentry_scan complete: takeoff {self.height_cm} cm, "
                f"{self.steps} x (hover {self.dwell_s:g} s, rotate "
                f"{self.step_deg:g} deg) = {self.revolutions:g} rev, landed")
        action = self._plan[self._idx]
        self._idx += 1
        return action


@register_phase
class OpenLoopLawnmower(MissionPhase):
    """Body-frame boustrophedon: Takeoff -> per lane [Move(FORWARD, leg_cm),
    Hover(scan_pause_s)] with alternating U-turns shifted by lane_cm -> Land ->
    Done.

    OPEN-LOOP — there is no position feedback and yaw error compounds at every
    turn. ENABLE ONLY after onsite gate E has measured relative-move + rotation
    accuracy (command 1 m + 4x90 deg, tape-measure the closure). OFF by default:
    SentryScan is the default searcher; a config must name "lawnmower" to use it.
    """

    name = "lawnmower"

    #: Constructor keywords settable from DroneConfig.zone["lawnmower"].
    _TUNABLES = ("height_cm", "lanes", "leg_cm", "lane_cm", "turn_deg",
                 "scan_pause_s")

    def __init__(self, *, height_cm: int = 80, lanes: int = 4,
                 leg_cm: int = 400, lane_cm: int = 300, turn_deg: float = 90.0,
                 scan_pause_s: float = 1.0):
        def _bad(key: str, value, why: str) -> ConfigError:
            return ConfigError(
                f"lawnmower: {key}={value!r} invalid — {why} — check "
                f'zone["lawnmower"] (or altitude_band_m for height_cm)')

        _check_height_cm("lawnmower", height_cm)
        if not isinstance(lanes, int) or isinstance(lanes, bool) or lanes < 1:
            raise _bad("lanes", lanes, "must be an int >= 1 (number of passes)")
        if not isinstance(leg_cm, int) or isinstance(leg_cm, bool) or leg_cm <= 0:
            raise _bad("leg_cm", leg_cm, "must be an int > 0 (cm per pass)")
        if (not isinstance(lane_cm, int) or isinstance(lane_cm, bool)
                or lane_cm <= 0):
            raise _bad("lane_cm", lane_cm, "must be an int > 0 (cm between lanes)")
        if (not isinstance(turn_deg, (int, float)) or isinstance(turn_deg, bool)
                or not math.isfinite(turn_deg) or turn_deg == 0):
            raise _bad("turn_deg", turn_deg, "must be a finite non-zero number "
                                             "(deg, +ve = CCW) for the U-turns")
        if (not isinstance(scan_pause_s, (int, float))
                or isinstance(scan_pause_s, bool)
                or not math.isfinite(scan_pause_s) or scan_pause_s < 0):
            raise _bad("scan_pause_s", scan_pause_s,
                       "must be a finite number >= 0 (s) per pass")

        self.height_cm = height_cm
        self.lanes = lanes
        self.leg_cm = leg_cm
        self.lane_cm = lane_cm
        self.turn_deg = float(turn_deg)
        self.scan_pause_s = float(scan_pause_s)

        self._plan: List[Action] = [Takeoff(height_cm=height_cm)]
        turn = self.turn_deg
        for i in range(lanes):
            self._plan.append(Move(direction=Direction.FORWARD,
                                   distance_cm=leg_cm))
            self._plan.append(Hover(duration_s=self.scan_pause_s))
            if i < lanes - 1:
                # U-turn shifted by lane_cm; alternate direction each lane so
                # the passes sweep a rectangle (boustrophedon).
                self._plan.append(Rotate(angle_deg=turn))
                self._plan.append(Move(direction=Direction.FORWARD,
                                       distance_cm=lane_cm))
                self._plan.append(Rotate(angle_deg=turn))
                turn = -turn
        self._plan.append(Land())
        self._idx = 0

    @classmethod
    def from_config(cls, drone_cfg: "DroneConfig",
                    cfg: "FinalsConfig") -> "OpenLoopLawnmower":
        kwargs = _zone_kwargs(drone_cfg, "lawnmower", cls._TUNABLES)
        _height_from_band(kwargs, drone_cfg)
        return cls(**kwargs)

    def step(self, ctx: AgentContext) -> Action:
        if ctx.last_action_ok is False:
            return Abort(
                f"lawnmower[{ctx.drone_id}]: {ctx.last_action!r} failed "
                f"({ctx.last_action_error}) — aborting instead of flying the "
                f"rest of the pattern from an unknown position")
        if self._idx >= len(self._plan):
            return Done(
                f"lawnmower complete: takeoff {self.height_cm} cm, {self.lanes} "
                f"x (forward {self.leg_cm} cm, hover {self.scan_pause_s:g} s), "
                f"lane spacing {self.lane_cm} cm, landed")
        action = self._plan[self._idx]
        self._idx += 1
        return action
