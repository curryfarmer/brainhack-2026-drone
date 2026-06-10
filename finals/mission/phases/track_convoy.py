"""track_convoy — active bearing-pursuit tracking of a moving convoy marker.

THE reactive MissionPhase. Every other phase (takeoff_demo, sentry_scan,
lawnmower) emits a PRECOMPUTED plan and ignores ctx.sightings; this one is the
reason the contract feeds `ctx.sightings` + `Sighting.bearing_deg` to step():
it LOCKS a target marker id and chases the moving car, re-deciding every tick
from the freshest sighting. Still pure (no I/O, no SDK, no sleep) — unit-tested
by stepping with hand-built AgentContexts, exactly like the other phases.

Control law — built on the ONE signal that exists on real HULA (no measured
position: pos_quality is NONE there, so this NEVER reads est_north/est_east):
- Steering: err = wrap180(bearing_deg - telemetry.yaw_deg). bearing_deg is an
  absolute heading (yaw - px_offset*HFOV, CCW+, sign pinned in types.py /
  dead_reckon.py); when the target is centered, bearing_deg == yaw_deg. So to
  face it, Rotate(clamp(err, +/-max_step_deg)) (Rotate +ve = CCW, matches the
  bearing sign).
- Approach: the camera is NADIR, so rotating only spins the image — FOLLOWING a
  moving ground target requires TRANSLATION. Once centered, Move(FORWARD,
  approach_cm) closes the ground distance, then Hover(track_dwell_s) re-observes.
  A centered BEARING only says the car is in our forward direction, not how far;
  the marker's pixel offset from frame centre says how far. So a DEADBAND
  (center_px_frac): while the marker sits near frame centre the car is
  essentially UNDER us — HOLD, don't step, or a forward move over-walks a slow/
  near-stationary car out of the footprint (the failure mode the small-footprint
  drones hit on the VM). Step only once it drifts past the deadband.
  This is open-loop (no position feedback), so it is BOUNDED two ways: a
  cumulative max_chase_cm displacement cap (the runaway guard) and gated OFF by
  default (approach_enabled) — the same gate-E discipline as OpenLoopLawnmower
  (enable only after onsite move/rotate accuracy is measured). With approach
  off the phase degrades to a safe rotate-and-dwell investigator: it keeps the
  target centered and dwells over it (logging/photographing) until it leaves.
- lead_gain (optional, default 0): leads a moving target's azimuth using the
  bearing rate across steps. Mount-agnostic (bearing + ts only). Range-based
  approach scaling is deliberately NOT implemented — it needs the camera-mount
  calibration that only onsite gate E provides.

State machine (self._state): INIT (Takeoff unless already airborne — so it runs
standalone ["track_convoy"] AND after a non-landing search) -> ACQUIRE (lock the
id that reaches acquire_hits within acquire_window_s) -> TRACK (steer/approach,
re-acquire on a lost_timeout_s gap) -> Done on investigate_budget_s or the chase
cap (agent auto-lands on the empty phase queue).

All tunables live in DroneConfig.zone["track_convoy"] (briefing-day = config
edits, not code). Validated loudly in __init__: a no-op tracker dies on the
ground, never mid-mission.

Derives from: search.py (the from_config + zone-tunable + fail-loud-validation
pattern) and the MissionPhase reactive contract (phase.py). Session: S11.
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING, List, Optional

from finals.errors import ConfigError
from finals.mission.phase import AgentContext, MissionPhase
from finals.mission.phases import register_phase
from finals.types import (Abort, Action, Direction, Done, Hover, Move, Rotate,
                          Takeoff)

if TYPE_CHECKING:  # type hints only — keeps the import graph minimal
    from finals.config import DroneConfig, FinalsConfig
    from finals.mission.convoy_registry import ConvoyRegistry


def _wrap180(deg: float) -> float:
    """Fold an angle into (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@register_phase
class TrackConvoy(MissionPhase):
    """Lock a convoy marker id and pursue it by bearing. See module docstring."""

    name = "track_convoy"

    #: Constructor keywords settable from DroneConfig.zone["track_convoy"].
    _TUNABLES = (
        "height_cm", "track_marker_ids", "acquire_hits", "acquire_window_s",
        "acquire_dwell_s", "acquire_budget_s", "center_tol_deg", "max_step_deg",
        "track_dwell_s", "approach_enabled", "approach_cm", "center_px_frac",
        "max_chase_cm", "lead_gain", "reacquire_dwell_s", "lost_timeout_s",
        "investigate_budget_s", "min_sightings_to_pass",
    )

    def __init__(self, *, height_cm: int = 80,
                 track_marker_ids: Optional[List[int]] = None,
                 acquire_hits: int = 3, acquire_window_s: float = 5.0,
                 acquire_dwell_s: float = 0.5, acquire_budget_s: float = 30.0,
                 center_tol_deg: float = 8.0, max_step_deg: float = 30.0,
                 track_dwell_s: float = 0.5, approach_enabled: bool = False,
                 approach_cm: int = 50, center_px_frac: float = 0.30,
                 max_chase_cm: int = 1000,
                 lead_gain: float = 0.0, reacquire_dwell_s: float = 0.5,
                 lost_timeout_s: float = 4.0,
                 investigate_budget_s: float = 90.0,
                 min_sightings_to_pass: int = 0,
                 sector_deg: "Optional[List[float]]" = None,
                 registry: "Optional[ConvoyRegistry]" = None):
        def _bad(key: str, value, why: str) -> ConfigError:
            return ConfigError(
                f"track_convoy: {key}={value!r} invalid — {why} — check "
                f'zone["track_convoy"] (or altitude_band_m for height_cm)')

        def _pos_num(key: str, value) -> None:
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value) or value <= 0):
                raise _bad(key, value, "must be a finite number > 0")

        def _pos_int(key: str, value) -> None:
            if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
                raise _bad(key, value, "must be an int > 0")

        if (not isinstance(height_cm, int) or isinstance(height_cm, bool)
                or height_cm <= 0):
            raise _bad("height_cm", height_cm, "must be an int > 0 (cm)")
        if track_marker_ids is not None:
            if (not isinstance(track_marker_ids, (list, tuple))
                    or not all(isinstance(m, int) and not isinstance(m, bool)
                               for m in track_marker_ids)):
                raise _bad("track_marker_ids", track_marker_ids,
                           "must be null (any id) or a list of int marker ids")
        _pos_int("acquire_hits", acquire_hits)
        _pos_int("approach_cm", approach_cm)
        _pos_int("max_chase_cm", max_chase_cm)
        for key, val in (("acquire_window_s", acquire_window_s),
                         ("acquire_dwell_s", acquire_dwell_s),
                         ("acquire_budget_s", acquire_budget_s),
                         ("center_tol_deg", center_tol_deg),
                         ("max_step_deg", max_step_deg),
                         ("track_dwell_s", track_dwell_s),
                         ("center_px_frac", center_px_frac),
                         ("reacquire_dwell_s", reacquire_dwell_s),
                         ("lost_timeout_s", lost_timeout_s),
                         ("investigate_budget_s", investigate_budget_s)):
            _pos_num(key, val)
        if center_px_frac > 1.5:                          # a frame corner is ~1.41
            raise _bad("center_px_frac", center_px_frac,
                       "must be <= 1.5 (a normalized radial offset from frame "
                       "centre; 1 = a frame edge, a corner ~1.41 — a bigger "
                       "deadband than the whole frame would never approach)")
        if not isinstance(approach_enabled, bool):
            raise _bad("approach_enabled", approach_enabled, "must be a bool")
        if (not isinstance(lead_gain, (int, float))
                or isinstance(lead_gain, bool)
                or not math.isfinite(lead_gain) or lead_gain < 0):
            raise _bad("lead_gain", lead_gain, "must be a finite number >= 0")
        if (not isinstance(min_sightings_to_pass, int)
                or isinstance(min_sightings_to_pass, bool)
                or min_sightings_to_pass < 0):
            raise _bad("min_sightings_to_pass", min_sightings_to_pass,
                       "must be an int >= 0 (0 = off; > 0 = fail loud with Abort "
                       "if fewer sightings of the convoy were captured)")
        if registry is not None and not all(
                hasattr(registry, m) for m in
                ("claim", "renew", "release", "claimable_ids")):
            raise _bad("registry", type(registry).__name__,
                       "must be a ConvoyRegistry or None (needs "
                       "claim/renew/release/claimable_ids)")
        sector = self._validate_sector(sector_deg, _bad)

        self.height_cm = height_cm
        self.track_marker_ids = (set(track_marker_ids)
                                 if track_marker_ids is not None else None)
        self.acquire_hits = acquire_hits
        self.acquire_window_s = float(acquire_window_s)
        self.acquire_dwell_s = float(acquire_dwell_s)
        self.acquire_budget_s = float(acquire_budget_s)
        self.center_tol_deg = float(center_tol_deg)
        self.max_step_deg = float(max_step_deg)
        self.track_dwell_s = float(track_dwell_s)
        self.approach_enabled = approach_enabled
        self.approach_cm = approach_cm
        self.center_px_frac = float(center_px_frac)
        self.max_chase_cm = max_chase_cm
        self.lead_gain = float(lead_gain)
        self.reacquire_dwell_s = float(reacquire_dwell_s)
        self.lost_timeout_s = float(lost_timeout_s)
        self.investigate_budget_s = float(investigate_budget_s)
        self.min_sightings_to_pass = min_sightings_to_pass
        #: Shared C2 ConvoyRegistry (None = static track_marker_ids behavior).
        #: from_config leaves it None; main._build_phases injects via
        #: bind_registry so dynamic assignment turns on without a config-shape
        #: change. See the class docstring.
        self.registry = registry
        #: WS-7A soft-zone: this drone's assigned sector as
        #: (center_deg, half_width_deg), or None = no soft-zoning (today's
        #: byte-for-byte behavior). from_config leaves it None; main._build_phases
        #: injects drone_cfg.sector_deg via bind_sector. With it set AND a
        #: registry bound, TRACK flags the registry when the tracked convoy's
        #: bearing leaves the wedge (still KEEPS tracking — soft, never a cut).
        self.sector_deg = sector

        # --- mutable per-mission state (a fresh instance per drone) ---
        self._state = "init"
        self._takeoff_issued = False
        self._t_enter: Optional[float] = None          # phase entry monotonic
        self._t_acquire_start: Optional[float] = None  # last entry into ACQUIRE
        self._target_id: Optional[int] = None
        self._obs: List[tuple] = []                    # (ts, marker_id) window
        self._t_last_seen: Optional[float] = None
        self._just_moved = False
        self._chase_used_cm = 0
        self._seen_count = 0
        self._lost_events = 0
        #: WS-7A: re-acquires triggered by an INTENTIONAL handover (the lock
        #: moved to the neighbour we exited toward), counted apart from
        #: _lost_events (lock expired / stolen) so the give-up message
        #: distinguishes a cooperative handoff from a real loss.
        self._handover_events = 0
        self._prev_bearing: Optional[float] = None
        self._prev_bearing_ts: Optional[float] = None
        #: WS-7A: whether we've ALREADY told the registry the current target left
        #: our sector — so we flag the exit edge ONCE and the re-entry edge ONCE,
        #: not on every tick. Reset whenever the target id changes.
        self._exited_flagged = False

    @staticmethod
    def _validate_sector(sector_deg, _bad):
        """Validate an optional [center_deg, half_width_deg] sector and return it
        as a (center, half_width) float tuple, or None. Same shape as
        config._parse_drone's sector_deg check — fail loud so a malformed wedge
        dies at construction, never silently disables soft-zoning mid-flight."""
        if sector_deg is None:
            return None
        if (not isinstance(sector_deg, (list, tuple)) or len(sector_deg) != 2
                or any(not isinstance(v, (int, float)) or isinstance(v, bool)
                       or not math.isfinite(v) for v in sector_deg)):
            raise _bad("sector_deg", sector_deg,
                       "must be null or [center_deg, half_width_deg] (two "
                       "finite numbers, deg, CCW+)")
        center, half = float(sector_deg[0]), float(sector_deg[1])
        if half < 0:
            raise _bad("sector_deg", sector_deg,
                       "half_width_deg must be >= 0 (a negative wedge would "
                       "flag every convoy as out-of-sector)")
        return (center, half)

    @classmethod
    def from_config(cls, drone_cfg: "DroneConfig",
                    cfg: "FinalsConfig") -> "TrackConvoy":
        # Lazy import: reuse search.py's zone/band helpers without a module-load
        # coupling between the two phase modules.
        from finals.mission.phases.search import (_height_from_band,
                                                  _zone_kwargs)
        kwargs = _zone_kwargs(drone_cfg, "track_convoy", cls._TUNABLES)
        _height_from_band(kwargs, drone_cfg)
        return cls(**kwargs)

    def bind_registry(self, registry: "ConvoyRegistry") -> None:
        """Inject the shared C2 ConvoyRegistry after from_config (done by
        main._build_phases). With a registry bound, ACQUIRE only locks ids it can
        CLAIM and TRACK renews the lock each tick — turning a static
        track_marker_ids list into dynamic, dedup'd, swarm-wide assignment. With
        none, behavior is exactly as before. Idempotent."""
        if registry is not None and not all(
                hasattr(registry, m) for m in
                ("claim", "renew", "release", "claimable_ids")):
            raise ConfigError(
                f"track_convoy.bind_registry: {type(registry).__name__!r} is not "
                f"a ConvoyRegistry (needs claim/renew/release/claimable_ids)")
        self.registry = registry

    def bind_sector(self, sector_deg: "Optional[List[float]]") -> None:
        """Inject this drone's assigned sector after from_config (done by
        main._build_phases from drone_cfg.sector_deg). With a sector AND a
        registry bound, TRACK soft-zones: it flags the registry when the tracked
        convoy's bearing leaves the wedge (KEEPS tracking) so an idle neighbour
        can be handed it. None = no soft-zoning (today's behavior). Validated
        loud + idempotent — same discipline as bind_registry."""
        def _bad(key, value, why):
            return ConfigError(
                f"track_convoy.bind_sector: {key}={value!r} invalid — {why}")
        self.sector_deg = self._validate_sector(sector_deg, _bad)

    def on_enter(self, ctx: AgentContext) -> None:
        self._t_enter = ctx.now

    # ---------------- the reactive loop ----------------
    def step(self, ctx: AgentContext) -> Action:
        if ctx.last_action_ok is False:
            # Never pursue from an unknown attitude (mirrors sentry_scan).
            return Abort(
                f"track_convoy[{ctx.drone_id}]: {ctx.last_action!r} failed "
                f"({ctx.last_action_error}) — aborting instead of tracking from "
                f"an unknown state")
        if self._t_enter is None:          # defensive: on_enter not called
            self._t_enter = ctx.now
        elapsed = ctx.now - self._t_enter

        if self._state == "init":
            action = self._step_init(ctx)
            if action is not None:
                return action              # Takeoff in flight; re-step when up

        if self._state == "acquire":
            action = self._step_acquire(ctx, elapsed)
            if action is not None:
                return action
            # locked this tick -> fall through and steer immediately

        if self._state == "track":
            return self._step_track(ctx, elapsed)

        return Done(f"track_convoy[{ctx.drone_id}]: idle in state "
                    f"{self._state!r} — handing back")

    # ---------------- states ----------------
    def _airborne(self, ctx: AgentContext) -> bool:
        # We commanded takeoff and it did not abort (the abort guard would have
        # returned already), OR telemetry already shows us flying.
        return self._takeoff_issued or ctx.telemetry.is_flying is True

    def _step_init(self, ctx: AgentContext) -> Optional[Action]:
        if not self._airborne(ctx):
            self._takeoff_issued = True
            return Takeoff(height_cm=self.height_cm)
        self._enter_acquire(ctx)
        return None

    def _enter_acquire(self, ctx: AgentContext) -> None:
        # acquire_budget_s is measured PER acquire attempt (initial + every
        # re-acquire after a lost lock), so a mid-chase loss gets a fresh budget
        # to re-find the car; investigate_budget_s stays the phase-wide hard cap.
        self._state = "acquire"
        self._t_acquire_start = ctx.now

    def _step_acquire(self, ctx: AgentContext,
                      elapsed: float) -> Optional[Action]:
        if elapsed > self.investigate_budget_s:
            return self._finish(
                ctx, f"track_convoy[{ctx.drone_id}]: investigate budget "
                f"{self.investigate_budget_s:g}s elapsed before a lock — "
                f"handing back", serviced=False)
        if self._t_acquire_start is None:          # defensive: always set above
            self._t_acquire_start = ctx.now
        acq_elapsed = ctx.now - self._t_acquire_start
        self._ingest(ctx)
        locked = self._best_candidate(self._claimable(ctx))
        if locked is not None:
            if self.registry is not None and not self.registry.claim(
                    ctx.drone_id, locked, ctx.now):
                # Lost a race between the claimable filter and the claim (another
                # drone won this id this tick) — keep scanning for a free convoy.
                return Hover(duration_s=self.acquire_dwell_s)
            self._target_id = locked
            self._state = "track"
            self._t_last_seen = ctx.now
            self._exited_flagged = False   # fresh target starts in-zone
            return None                    # steer this same tick
        if acq_elapsed > self.acquire_budget_s:
            return self._finish(
                ctx, f"track_convoy[{ctx.drone_id}]: no id reached "
                f"{self.acquire_hits} hit(s) within {self.acquire_budget_s:g}s "
                f"— nothing to track, handing back", serviced=False)
        return Hover(duration_s=self.acquire_dwell_s)

    def _step_track(self, ctx: AgentContext, elapsed: float) -> Action:
        if elapsed > self.investigate_budget_s:
            return self._done(ctx, f"investigate budget "
                                   f"{self.investigate_budget_s:g}s reached")
        fresh = [s for s in ctx.sightings if s.marker_id == self._target_id]
        if fresh:
            self._t_last_seen = ctx.now
            self._seen_count += len(fresh)
            if (self.registry is not None and not self.registry.renew(
                    ctx.drone_id, self._target_id, ctx.now)):
                # Our lock moved off us. Distinguish the two reasons (the lock is
                # the same, the cause is not): an INTENTIONAL soft-zone handover
                # (a neighbour accepted the convoy we flagged out of our sector,
                # so it is now CLAIMED by someone else) vs a real LOSS (expired /
                # stolen — now unowned). Either way we no longer own it (no
                # release), drop it and re-acquire whatever we can still claim.
                if self._handed_over(ctx):
                    self._handover_events += 1
                else:
                    self._lost_events += 1
                self._drop_target(ctx)
                return Hover(duration_s=self.reacquire_dwell_s)
            self._soft_zone(ctx, fresh)   # flag the exit/re-entry edge (WS-7A)
            return self._steer(ctx, fresh)
        # lost this tick (no fresh sighting)
        assert self._t_last_seen is not None
        if ctx.now - self._t_last_seen > self.lost_timeout_s:
            self._lost_events += 1         # drop the lock and re-acquire
            # We still OWN the lock here — hand it back so a better-placed drone
            # can pick the convoy up while we re-find it (expire() would free it
            # eventually anyway; releasing now is the cooperative path).
            self._release(ctx, serviced=False)
            self._drop_target(ctx)         # fresh acquire_budget for the re-find
        return Hover(duration_s=self.reacquire_dwell_s)

    def _drop_target(self, ctx: AgentContext) -> None:
        """Stop tracking the current target and re-enter ACQUIRE with a fresh
        budget. Shared by every re-acquire path (renew-False handover/loss, the
        no-sighting timeout) so the soft-zone exit flag is always reset with the
        target it referred to."""
        self._target_id = None
        self._obs.clear()
        self._exited_flagged = False
        self._enter_acquire(ctx)

    def _steer(self, ctx: AgentContext, fresh: List) -> Action:
        usable = [s for s in fresh if s.bearing_deg is not None]
        yaw = ctx.telemetry.yaw_deg
        if not usable or yaw is None:
            # No bearing (e.g. camera_hfov_deg null on real HULA): degrade to a
            # stable dwell over the target — never crash, never fly blind.
            return Hover(duration_s=self.track_dwell_s)
        s = max(usable, key=lambda x: x.ts)
        aim = s.bearing_deg + self._lead(s.bearing_deg, s.ts)
        err = _wrap180(aim - yaw)
        if abs(err) > self.center_tol_deg:
            return Rotate(angle_deg=_clamp(err, -self.max_step_deg,
                                           self.max_step_deg))
        # Centered on the target bearing.
        if not self.approach_enabled:
            return Hover(duration_s=self.track_dwell_s)     # safe observer
        if self._just_moved:
            self._just_moved = False
            return Hover(duration_s=self.track_dwell_s)     # look after a move
        # NADIR DEADBAND: a centered bearing only says the car is in our FORWARD
        # direction, not how far. The marker's pixel offset from frame centre
        # does: near-centre = the car is essentially UNDER us, so a forward step
        # would over-walk a slow/near-stationary car out of the footprint (how
        # the small-band drones lost their cars on the VM). HOLD until it drifts
        # past center_px_frac, then step. off is None (missing/degenerate frame
        # geometry) -> keep the prior always-step behavior; never crash, never
        # freeze on bad data.
        off = self._offcenter(s)
        if off is not None and off < self.center_px_frac:
            return Hover(duration_s=self.track_dwell_s)     # car under us — hold
        remaining = self.max_chase_cm - self._chase_used_cm
        if remaining <= 0:
            return self._done(ctx, f"chase cap {self.max_chase_cm} cm reached")
        step_cm = int(min(self.approach_cm, remaining))
        self._chase_used_cm += step_cm
        self._just_moved = True
        return Move(direction=Direction.FORWARD, distance_cm=step_cm)

    # ---------------- helpers ----------------
    def _ingest(self, ctx: AgentContext) -> None:
        """Fold this tick's sightings into the acquire window, then prune."""
        for s in ctx.sightings:
            if s.marker_id is None:
                continue
            if (self.track_marker_ids is not None
                    and s.marker_id not in self.track_marker_ids):
                continue
            self._obs.append((s.ts, s.marker_id))
        cutoff = ctx.now - self.acquire_window_s
        self._obs = [o for o in self._obs if o[0] >= cutoff]

    def _best_candidate(self,
                        claimable: Optional[set] = None) -> Optional[int]:
        """The id with the most hits in-window once it reaches acquire_hits;
        ties broken by the most recent observation. When `claimable` is given
        (registry bound), only ids the registry says this drone may still claim
        are eligible — so a convoy another drone already owns is never chased."""
        counts: dict = {}
        latest: dict = {}
        for ts, mid in self._obs:
            counts[mid] = counts.get(mid, 0) + 1
            latest[mid] = max(latest.get(mid, ts), ts)
        ready = [mid for mid, n in counts.items() if n >= self.acquire_hits]
        if claimable is not None:
            ready = [mid for mid in ready if mid in claimable]
        if not ready:
            return None
        return max(ready, key=lambda mid: (counts[mid], latest[mid]))

    def _claimable(self, ctx: AgentContext) -> Optional[set]:
        """The in-window candidate ids this drone may still claim, per the
        registry. None when no registry is bound -> _best_candidate keeps its
        original 'any id that reached acquire_hits' behavior (static mode)."""
        if self.registry is None:
            return None
        candidates = {mid for _ts, mid in self._obs}
        return set(self.registry.claimable_ids(ctx.drone_id, ctx.now,
                                               candidates))

    def _release(self, ctx: AgentContext, *, serviced: bool) -> None:
        """Hand our convoy lock back to the registry. No-op without a registry or
        a current target. serviced=True -> SERVICED (counts to 5-of-5, never
        re-claimable); False -> back to the pool (loss / clean handover)."""
        if self.registry is not None and self._target_id is not None:
            self.registry.release(ctx.drone_id, self._target_id, ctx.now,
                                  serviced=serviced)

    def _handed_over(self, ctx: AgentContext) -> bool:
        """True iff our just-lost lock moved to ANOTHER drone (an intentional
        soft-zone handover) rather than expiring/being freed (a real loss). Read
        from the registry: a CLAIMED-by-someone-else entry == handed over. Best
        effort — any registry quirk reads as a plain loss, never crashes."""
        if self.registry is None or self._target_id is None:
            return False
        owner = self.registry.owner_of(self._target_id, ctx.now)
        return owner is not None and owner != ctx.drone_id

    def _soft_zone(self, ctx: AgentContext, fresh: List) -> None:
        """WS-7A soft zoning: from the freshest sighting's bearing, decide if the
        tracked convoy is IN or OUT of THIS drone's sector and flag the registry
        ONCE per edge. Pure book-keeping — it NEVER drops the lock or changes the
        steer (soft, never a hard cut); the orchestrator matcher reads the flag
        and looks for an idle neighbour. No-op without both a sector and a
        registry bound (default = today's behavior), or if the freshest sighting
        has no bearing (can't decide -> leave the flag as-is)."""
        if (self.sector_deg is None or self.registry is None
                or self._target_id is None):
            return
        usable = [s for s in fresh if s.bearing_deg is not None]
        if not usable:
            return                         # no bearing this tick -> can't decide
        from finals.mission.planning.frame import bearing_in_sector
        bearing = max(usable, key=lambda x: x.ts).bearing_deg
        center, half = self.sector_deg
        inside = bearing_in_sector(bearing, center, half)
        if not inside and not self._exited_flagged:
            # EXIT edge: the convoy just left our wedge. Flag it (carrying the
            # bearing so the matcher knows which neighbour it entered) and keep
            # tracking. flag_exited raises only on a wiring bug (we are the owner
            # here — renew() just succeeded), so let it surface loudly.
            self.registry.flag_exited(ctx.drone_id, self._target_id, ctx.now,
                                      exited=True, exit_bearing_deg=bearing)
            self._exited_flagged = True
        elif inside and self._exited_flagged:
            # RE-ENTRY edge: it came back before any neighbour took it. Clear the
            # flag + any standing offer; we keep it.
            self.registry.flag_exited(ctx.drone_id, self._target_id, ctx.now,
                                      exited=False)
            self._exited_flagged = False

    def _lead(self, bearing: float, ts: float) -> float:
        """Azimuth lead for a moving target from the bearing rate (0 unless
        lead_gain > 0). Updates the previous-bearing memory each call."""
        lead = 0.0
        if (self.lead_gain and self._prev_bearing is not None
                and self._prev_bearing_ts is not None):
            dt = ts - self._prev_bearing_ts
            if dt > 0:
                rate = _wrap180(bearing - self._prev_bearing) / dt
                lead = self.lead_gain * rate * self.track_dwell_s
        self._prev_bearing = bearing
        self._prev_bearing_ts = ts
        return lead

    @staticmethod
    def _offcenter(s) -> Optional[float]:
        """Radial offset of the marker from frame CENTRE, normalized so 0 = dead
        centre and 1 = a frame edge (a corner ~1.41); None if the frame geometry
        is missing or degenerate. This is the 'how far, not just which way'
        signal the bearing discards: small = car is under us, large = car has
        drifted toward the footprint edge (the deadband uses it in _steer)."""
        shape = getattr(s, "frame_shape", None)
        bbox = getattr(s, "bbox_xyxy", None)
        if not shape or not bbox:
            return None
        h, w = shape[0], shape[1]
        if not h or not w:
            return None
        cx = (bbox[0] + bbox[2]) / 2.0
        cy = (bbox[1] + bbox[3]) / 2.0
        return math.hypot((cx - w / 2.0) / (w / 2.0),
                          (cy - h / 2.0) / (h / 2.0))

    def _done(self, ctx: AgentContext, why: str) -> Action:
        """Successful end of a TRACK: release the lock as SERVICED, then Done
        (subject to the min_sightings_to_pass guard in _finish)."""
        msg = (f"track_convoy[{ctx.drone_id}]: tracked id={self._target_id} "
               f"({self._seen_count} sighting(s), chased {self._chase_used_cm} cm, "
               f"{self._lost_events} reacquire(s), "
               f"{self._handover_events} handover(s)) — {why}")
        return self._finish(ctx, msg, serviced=True)

    def _finish(self, ctx: AgentContext, done_msg: str, *,
                serviced: bool) -> Action:
        """End the phase: release any lock (serviced or back-to-pool), then Done —
        UNLESS min_sightings_to_pass is set and we saw fewer than that, in which
        case fail LOUD with Abort instead of reporting a clean Done over a convoy
        we never actually saw (closes the silent-success hole). Default 0 = off,
        so the give-up paths keep returning Done with their existing messages."""
        self._release(ctx, serviced=serviced)
        if (self.min_sightings_to_pass
                and self._seen_count < self.min_sightings_to_pass):
            return Abort(
                f"track_convoy[{ctx.drone_id}]: only {self._seen_count} "
                f"sighting(s) of the convoy — need >= "
                f"{self.min_sightings_to_pass} (min_sightings_to_pass) — failing "
                f"loud instead of reporting success ({done_msg})")
        return Done(done_msg)
