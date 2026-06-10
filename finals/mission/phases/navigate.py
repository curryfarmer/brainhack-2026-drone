"""navigate — open-loop transit phase: fly C2 -> pad-vicinity over planner Legs.

from_config resolves the goal (a pad_id, an explicit [north_m, east_m], or a
marker_id naming a known field beacon in ArenaMap.markers — the NAV-FIX
beacon-region approach) from the ArenaMap + DroneConfig.zone["navigate"],
calls the visibility-graph planner
(finals.mission.planning.visibility_graph.plan) for a FROZEN tuple[Leg, ...],
and step() flies each leg OPEN-LOOP: Rotate to the leg's ABSOLUTE compass
heading (re-zeroing accumulated yaw creep against the TRUSTED compass every
leg) via _servo.bearing_error_to_rotate, then Move(FORWARD, distance_cm).
It advances on last_action_ok, Aborts on a failed action (never flies on from an
unknown attitude), and returns Done after the final leg's Move resolves.

TRANSIT semantics: navigate assumes the drone is ALREADY AIRBORNE — a prior
`takeoff` phase owns liftoff, so this phase NEVER issues Takeoff/Land. Position
stays PositionQuality.NONE; the DeadReckoner pose is ADVISORY (sector geofence)
only, never a control input here.

WHY absolute-heading re-orient (load-bearing): each leg commands a Rotate toward
the leg's ABSOLUTE heading_deg computed from the trusted compass yaw
(error = wrap180(heading_deg - yaw_deg)), NOT a relative turn between legs. So a
leg starting from a drifted yaw still ends pointing at the true world heading —
the per-leg re-orient RE-ZEROES accumulated yaw creep instead of compounding it.

GATE ALTITUDE RULE (NAV-ARCH — load-bearing for the arch course): an arch is a
black/yellow frame the drone flies THROUGH the GAP of and CANNOT overfly (the
~1.1 m operating ceiling sits below the crossbar; the LANDING mission runs with
NO altitude bands — see guards.py NAV-8 / configs/landing_real.json). So the
planner's gate handling is purely HORIZONTAL (north/east): visibility_graph
routes a Leg through the gate's span, and this phase flies that Leg at the drone's
single fixed transit altitude — the height a prior `takeoff` phase established and
HOLDS for the whole transit (navigate never issues a vertical Move). There is no
per-gate height because there is no climb-over option and no band to pick: the
operator sets ONE transit altitude clear under every arch crossbar at gate D, and
every drone flies the arch course at that one height. This is consistent with
(not a contradiction of) the no-altitude-bands ceiling: bands separate drones
vertically (illegal here); the gate height is the COMMON arch-clearance altitude
all drones share, with TIME+SPACE (sector) deconfliction doing the separation.

GEOFENCE: the advisory sector geofence is NOT implemented here — that is
guards.py GeofenceLite / the NAV-8 orchestrator's territory. This phase stays
pure and focused on transit (a deliberate scope decision).

HEADING CONVENTION (binding): heading_deg comes straight from
visibility_graph.plan, whose docstring derives heading = atan2(-dE, dN) as the
INVERSE of dead_reckon's FORWARD map and pins it against the REAL DeadReckoner.
_servo.bearing_error_to_rotate consumes that absolute heading directly in the
same CCW-positive yaw frame (target CCW of the nose -> positive/CCW Rotate), so
there is NO sign juggling at this call site.

Derives from: search.py SentryScan / takeoff_demo.py (the from_config +
_zone_kwargs validated-tunables + no-op-trap + Abort-on-failure template);
planning.visibility_graph (NAV-1, the plan + heading convention); _servo (NAV-3,
the absolute-heading Rotate math).

Implemented — session S11 (NAV-5). See finals/docs/module_map.md.
"""
from __future__ import annotations

import dataclasses
import math
from typing import TYPE_CHECKING, Optional, Tuple

from finals.errors import ConfigError
from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases import register_phase
from finals.mission.phases._servo import bearing_error_to_rotate, wrap180
from finals.mission.planning.types import Leg
from finals.mission.planning.visibility_graph import plan
from finals.types import Abort, Action, Direction, Done, Move

if TYPE_CHECKING:  # type hints only — no import-time coupling to config
    from finals.config import DroneConfig, FinalsConfig
    from finals.mission.obstacle_map import ObstacleMap

#: Constructor keywords settable from DroneConfig.zone["navigate"]. Exactly ONE
#: of pad_id / goal_ne_m / marker_id names the goal; the rest are transit
#: tunables. NAV-FIX adds marker_id: target a KNOWN field-beacon coordinate
#: (arena.markers) as the waypoint — each beacon sits ~20-30 cm from its pad, so
#: "navigate to the beacon region with slack" puts the drone over the pad for
#: the visual servo / pad-detector to refine (docs/field_markers.md).
_TUNABLES = ("pad_id", "goal_ne_m", "marker_id", "inflation_m", "max_leg_cm",
             "heading_tol_deg", "max_step_deg", "total_budget_s")


def _zone_kwargs(drone_cfg: "DroneConfig") -> dict:
    """DroneConfig.zone["navigate"] -> validated kwargs (drops _comment keys,
    rejects typos loudly) — the same no-silent-drop policy as search.py /
    takeoff_demo.from_config. A typo here would otherwise vanish and the drone
    would fly the DEFAULT transit instead of the briefed one."""
    params = drone_cfg.zone.get("navigate", {})
    if not isinstance(params, dict):
        raise ConfigError(
            f"drone {drone_cfg.id!r}: zone[\"navigate\"] must be an object of "
            f"tunables {sorted(_TUNABLES)}, got {params!r}")
    kwargs = {k: v for k, v in params.items() if not k.startswith("_")}
    unknown = sorted(set(kwargs) - set(_TUNABLES))
    if unknown:
        raise ConfigError(
            f"drone {drone_cfg.id!r}: zone[\"navigate\"] unknown key(s) "
            f"{unknown} — valid keys: {sorted(_TUNABLES)} (typo?)")
    return kwargs


def _pos_float(name: str, value, *, allow_zero: bool) -> float:
    """A finite float that is > 0 (or >= 0 when allow_zero). bool is rejected
    (True/False sneaking in as a tunable is always a wiring bug). Fail loud on
    the ground — a 0/neg margin or step is a no-op trap that would make the
    transit either never converge or divide-by-zero in the planner."""
    rel = ">= 0" if allow_zero else "> 0"
    if (not isinstance(value, (int, float)) or isinstance(value, bool)
            or not math.isfinite(value)
            or (value < 0.0) or (value == 0.0 and not allow_zero)):
        raise ConfigError(
            f"navigate: {name}={value!r} invalid — must be a finite number "
            f"{rel} — check zone[\"navigate\"][{name!r}]")
    return float(value)


@register_phase
class Navigate(MissionPhase):
    """Open-loop transit over planner Legs: per leg Rotate-to-absolute-heading
    (compass) then Move(FORWARD); Done after the final Move. Assumes airborne."""

    name = "navigate"

    def __init__(self, *, goal_m: Tuple[float, float], legs: Tuple[Leg, ...],
                 heading_tol_deg: float, max_step_deg: float,
                 total_budget_s: float, goal_desc: str = ""):
        # Tunables are validated in from_config (the construction boundary the
        # planner is also called behind); here we only hold the frozen plan +
        # the servo/budget bounds. legs is a tuple => the plan cannot be mutated
        # mid-mission. An empty plan is a no-op trap and is rejected upstream.
        self.goal_m = goal_m
        self.goal_desc = goal_desc or f"{goal_m}"
        self._legs: Tuple[Leg, ...] = tuple(legs)
        self.heading_tol_deg = float(heading_tol_deg)
        self.max_step_deg = float(max_step_deg)
        self.total_budget_s = float(total_budget_s)

        # Per-leg re-orient iteration bound (convention 3): a compass that never
        # converges within tol must not Rotate forever. Each Rotate closes the
        # error by up to max_step_deg, so ceil(360/max_step) + a slack margin
        # covers the worst legal turn (a full circle) and then some; exceeding
        # it means the yaw is not converging (a stuck/oscillating feed) and we
        # Abort naming the residual error rather than spinning.
        self._rot_cap = int(math.ceil(360.0 / self.max_step_deg)) + 4

        # Sub-step state machine, per leg:
        #   "rotate"     -> re-orient to the leg's ABSOLUTE heading (0+ Rotates)
        #   "await_move" -> the leg's Move was issued; we are waiting for it to
        #                   resolve. Advancing _leg_idx happens HERE, only AFTER
        #                   the agent re-steps with last_action_ok True (a
        #                   failed Move is caught by the guard at the top of
        #                   step() while _leg_idx still names the current leg, so
        #                   the Abort message is accurate — no off-by-one).
        self._leg_idx = 0
        self._substep = "rotate"
        self._rot_count = 0           # Rotates issued for the current leg
        self._start_elapsed: Optional[float] = None   # set on first step()

    @classmethod
    def from_config(cls, drone_cfg: "DroneConfig",
                    cfg: "FinalsConfig",
                    obstacle_map: "Optional[ObstacleMap]" = None) -> "Navigate":
        """Resolve the goal + transit tunables from config and PRE-PLAN the
        Legs (fail loud at wiring time, never mid-air). `cfg` carries the arena.

        WS-6 extension: `obstacle_map` is the SHARED collective map (one instance
        threaded into every drone by finals.main). Any keep-out a drone or the
        operator pre-flight tap contributed is MERGED with the static arena
        keep-outs before planning, so this drone routes around an obstacle it
        never saw itself. None / empty -> today's static-arena-only behaviour.
        """
        kwargs = _zone_kwargs(drone_cfg)

        # --- arena is mandatory: the planner needs the keep-outs + the C2
        # launch origin. Name arena_name so the operator knows which knob. ---
        arena = getattr(cfg, "arena", None) if cfg is not None else None
        if arena is None:
            arena_name = getattr(cfg, "arena_name", None) if cfg is not None \
                else None
            raise ConfigError(
                f"drone {drone_cfg.id!r}: navigate needs an arena (keep-outs + "
                f"C2 launch origin) but cfg.arena is None — set arena_name in "
                f"the profile config (it loads finals/configs/arenas/"
                f"<arena_name>.json). Got arena_name={arena_name!r}.")

        # --- goal: EXACTLY ONE of pad_id / goal_ne_m / marker_id. ---
        # marker_id (NAV-FIX) targets a KNOWN field-beacon coordinate from
        # arena.markers — the beacon-region approach: each beacon is ~20-30 cm
        # from its pad, so the open-loop transit only needs to reach the beacon
        # coord and the visual servo / pad-detector refines the touchdown.
        goal_sources = [k for k in ("pad_id", "goal_ne_m", "marker_id")
                        if k in kwargs]
        if len(goal_sources) > 1:
            raise ConfigError(
                f"drone {drone_cfg.id!r}: zone[\"navigate\"] sets MULTIPLE goal "
                f"sources {goal_sources} — give EXACTLY ONE (pad_id names an "
                f"arena pad; goal_ne_m is an explicit [north_m, east_m]; "
                f"marker_id names a known field beacon from arena.markers).")
        if not goal_sources:
            raise ConfigError(
                f"drone {drone_cfg.id!r}: zone[\"navigate\"] sets NO goal — name "
                f"EXACTLY ONE of pad_id (an arena pad), goal_ne_m (an explicit "
                f"[north_m, east_m]), or marker_id (a known field beacon from "
                f"arena.markers).")

        if "pad_id" in kwargs:
            pad_id = kwargs["pad_id"]
            pads = {p.id: p for p in arena.pads}
            if pad_id not in pads:
                raise ConfigError(
                    f"drone {drone_cfg.id!r}: zone[\"navigate\"].pad_id "
                    f"{pad_id!r} is not a pad in this arena — available pad "
                    f"ids: {sorted(pads)}. Check the pad_id (typo?) or the "
                    f"arena_name.")
            goal_m = pads[pad_id].center_m
            goal_desc = f"pad {pad_id!r} center {tuple(goal_m)}"
        elif "marker_id" in kwargs:
            marker_id = kwargs["marker_id"]
            if not isinstance(marker_id, int) or isinstance(marker_id, bool):
                raise ConfigError(
                    f"drone {drone_cfg.id!r}: zone[\"navigate\"].marker_id must "
                    f"be an int ArUco beacon id (e.g. 11/45/51/67/101), got "
                    f"{marker_id!r}.")
            markers = {m.id: m for m in arena.markers}
            if marker_id not in markers:
                raise ConfigError(
                    f"drone {drone_cfg.id!r}: zone[\"navigate\"].marker_id "
                    f"{marker_id} is not a beacon in this arena — available "
                    f"marker ids: {sorted(markers)}. Check the marker_id (typo?) "
                    f"or the arena_name (does this arena declare markers?).")
            goal_m = markers[marker_id].point_m
            goal_desc = (f"beacon {marker_id} region (known coord {tuple(goal_m)}; "
                         f"servo/pad-detector refines touchdown)")
        else:
            raw = kwargs["goal_ne_m"]
            if (not isinstance(raw, (list, tuple)) or len(raw) != 2
                    or any(not isinstance(c, (int, float))
                           or isinstance(c, bool) or not math.isfinite(c)
                           for c in raw)):
                raise ConfigError(
                    f"drone {drone_cfg.id!r}: zone[\"navigate\"].goal_ne_m must "
                    f"be [north_m, east_m] finite numbers, got {raw!r}.")
            goal_m = (float(raw[0]), float(raw[1]))
            goal_desc = f"goal_ne_m {goal_m}"

        inflation_m = _pos_float("inflation_m", kwargs.get("inflation_m", 0.5),
                                 allow_zero=False)
        max_leg_cm = _pos_float("max_leg_cm", kwargs.get("max_leg_cm", 100.0),
                                allow_zero=False)
        heading_tol_deg = _pos_float(
            "heading_tol_deg", kwargs.get("heading_tol_deg", 5.0),
            allow_zero=True)
        max_step_deg = _pos_float(
            "max_step_deg", kwargs.get("max_step_deg", 45.0), allow_zero=False)
        total_budget_s = _pos_float(
            "total_budget_s", kwargs.get("total_budget_s", 120.0),
            allow_zero=False)

        # WS-6: merge the shared collective map. Static arena keep-outs stay
        # AUTHORITATIVE (ObstacleMap.merge only ADDS ids the arena lacks), so a
        # contribution can make us MORE cautious, never less. Empty/None map ->
        # arena unchanged. Rebuild a frozen ArenaMap with the merged keep-outs.
        if obstacle_map is not None and len(obstacle_map) > 0:
            merged = obstacle_map.merge(arena.keep_out)
            if len(merged) != len(arena.keep_out):
                arena = dataclasses.replace(arena, keep_out=tuple(merged))

        start_m = arena.c2_origin_m
        # plan() raises ValueError on out-of-domain args (guarded above) and
        # PlanningError when the goal is trapped / unreachable. We let
        # PlanningError PROPAGATE: a goal in a keep-out is a loud config-time
        # failure the operator must fix, NOT something to swallow.
        legs = plan(start_m, goal_m, arena, inflation_m, max_leg_cm)

        # No-op trap: a degenerate plan (start already at goal, or any path that
        # collapsed to zero legs) means navigate has nothing to do. Refuse on
        # the ground rather than silently Done-ing a transit that never flew.
        if not legs:
            raise ConfigError(
                f"drone {drone_cfg.id!r}: navigate planned ZERO legs from C2 "
                f"origin {tuple(start_m)} to {goal_desc} (start already at the "
                f"goal within rounding?) — a no-op transit is a wiring bug. "
                f"Check the goal vs c2_origin_m, or drop the navigate phase.")

        # ORIGIN-CAL heading offset: arena.heading_offset_deg = Δ, the onsite
        # misalignment between the compass-yaw frame and arena-north
        # (Δ = boot_yaw_reading − arena_heading_aimed). plan() emits each
        # leg.heading_deg in the ARENA-north frame; the open-loop Rotate compares
        # that target against the sensor yaw, so to physically aim the nose along
        # the arena heading the Rotate TARGET must be leg.heading_deg + Δ (the yaw
        # the compass reads when the nose is on that arena heading). Bake Δ into
        # every leg ONCE here so the step() Rotate target AND the non-convergence
        # residual report (both read leg.heading_deg) stay the SAME quantity — a
        # single source of truth, no rotate-vs-diagnostic drift. wrap180 keeps the
        # baked heading in the leg's [-180,180] convention. Δ defaults to 0.0 →
        # legs unchanged (today's behaviour verbatim, same object identity).
        offset = arena.heading_offset_deg
        if offset:
            legs = tuple(
                dataclasses.replace(
                    leg, heading_deg=wrap180(leg.heading_deg + offset))
                for leg in legs)

        return cls(goal_m=goal_m, legs=tuple(legs),
                   heading_tol_deg=heading_tol_deg, max_step_deg=max_step_deg,
                   total_budget_s=total_budget_s, goal_desc=goal_desc)

    def on_enter(self, ctx: AgentContext) -> None:
        """Capture the per-phase deadline reference on entry (pure). step()
        also captures it defensively on the first call, so a phase exercised
        WITHOUT the agent calling on_enter (the hand-built-ctx unit tests) still
        gets a budget clock."""
        if self._start_elapsed is None:
            self._start_elapsed = ctx.mission_elapsed_s

    def step(self, ctx: AgentContext) -> Action:
        # 0) Never fly on from a failed action — the attitude is unknown.
        if ctx.last_action_ok is False:
            return Abort(
                f"navigate[{ctx.drone_id}]: leg {self._leg_idx + 1}/"
                f"{len(self._legs)} {self._substep!r} {ctx.last_action!r} "
                f"failed ({ctx.last_action_error}) — aborting instead of "
                f"continuing transit to {self.goal_desc} from an unknown "
                f"attitude. CHECK: the flight link / the underlying error.")

        # First step (and on_enter) anchors the budget clock.
        if self._start_elapsed is None:
            self._start_elapsed = ctx.mission_elapsed_s

        # 1) Per-phase deadline (convention 3): transit must not overrun.
        elapsed = ctx.mission_elapsed_s - self._start_elapsed
        if elapsed > self.total_budget_s:
            return Abort(
                f"navigate[{ctx.drone_id}]: transit to {self.goal_desc} "
                f"OVERRAN its budget — {elapsed:.1f} s elapsed > "
                f"total_budget_s {self.total_budget_s:.1f} s at leg "
                f"{self._leg_idx + 1}/{len(self._legs)} ({self._substep!r}). "
                f"CHECK: per-command timeouts / yaw not converging / a "
                f"too-tight budget for this route.")

        # 2) The previous leg's Move has now resolved OK (a failure would have
        # tripped the guard above) — advance to the next leg. Done only after
        # the FINAL leg's Move resolves.
        if self._substep == "await_move":
            self._leg_idx += 1
            self._substep = "rotate"

        # 3) The trusted compass is the ONE input this open-loop phase cannot
        # fly without. A None yaw means no heading reference — refuse rather
        # than re-orient against a fabricated heading. Hoisted ABOVE the
        # leg-advance loop below: yaw is constant for this ctx, so checking it
        # once is behaviorally identical to re-checking per skipped leg, and the
        # loop must never re-orient against a fabricated heading.
        yaw = ctx.telemetry.yaw_deg
        if yaw is None:
            return Abort(
                f"navigate[{ctx.drone_id}]: telemetry.yaw_deg is None at leg "
                f"{self._leg_idx + 1}/{len(self._legs)} — this open-loop "
                f"transit re-orients to each leg's ABSOLUTE compass heading "
                f"and cannot fly without a yaw feed. CHECK: the compass / "
                f"telemetry source.")

        # Advance past any zero-distance (sub-cm, rounds-to-0) legs in a BOUNDED
        # loop instead of recursing: each iteration handles one leg's rotate +
        # forward Move, and a leg whose distance rounds to 0 cm (REFUSED by the
        # adapter) is skipped — treated as already flown (the inflation margin
        # absorbs the sub-cm shortfall). The loop is bounded by the finite leg
        # list (each iteration either returns OR advances _leg_idx by one toward
        # len(self._legs)); the budget / last_action_ok / yaw guards stay ABOVE
        # it. Done only after the FINAL leg resolves.
        while True:
            if self._leg_idx >= len(self._legs):
                return Done(
                    f"navigate complete: {len(self._legs)} leg(s) flown to "
                    f"{self.goal_desc} (open-loop, per-leg compass re-orient)")

            leg = self._legs[self._leg_idx]

            if self._substep == "rotate":
                # Re-orient to the leg's ABSOLUTE heading using the trusted
                # compass. error = wrap180(heading_deg - yaw): re-zeroes
                # accumulated yaw creep every leg instead of compounding a
                # relative delta.
                rot = bearing_error_to_rotate(
                    leg.heading_deg, yaw, self.heading_tol_deg,
                    self.max_step_deg)
                if rot is not None:
                    self._rot_count += 1
                    if self._rot_count > self._rot_cap:
                        # WRAPPED residual: convergence is judged on
                        # wrap180(target - yaw) (the deadband in
                        # bearing_error_to_rotate), so the reported error must
                        # be the SAME wrapped quantity — an unwrapped diff would
                        # print e.g. "350 deg" for a true -10 deg error and
                        # point the operator at the wrong CHECK clause.
                        residual = wrap180(leg.heading_deg - yaw)
                        return Abort(
                            f"navigate[{ctx.drone_id}]: leg {self._leg_idx + 1}/"
                            f"{len(self._legs)} re-orient did NOT converge "
                            f"within {self._rot_cap} Rotate steps — target "
                            f"heading {leg.heading_deg:.1f} deg vs yaw "
                            f"{yaw:.1f} deg (residual ~{residual:.1f} deg > tol "
                            f"{self.heading_tol_deg:g} deg). CHECK: a stuck/"
                            f"oscillating compass feed or a too-small "
                            f"max_step_deg vs heading_tol_deg.")
                    return rot
                # Inside the deadband -> the nose is on the leg heading; fly it.
                self._rot_count = 0

            # Fly the current leg forward. round() because Move.distance_cm is
            # an int (the FlightAdapter cm contract). A tiny final/sub leg
            # rounding to 0 cm would be REFUSED by the adapter, so we never
            # command it: advance to the next leg and loop (no wasted tick).
            dist_cm = int(round(leg.distance_cm))
            if dist_cm <= 0:
                self._leg_idx += 1
                self._substep = "rotate"
                continue
            # Mark the Move in flight; the NEXT step() (after it resolves OK)
            # advances the leg index.
            self._substep = "await_move"
            return Move(direction=Direction.FORWARD, distance_cm=dist_cm)
