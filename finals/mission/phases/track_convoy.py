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
        "track_dwell_s", "approach_enabled", "approach_cm", "max_chase_cm",
        "lead_gain", "reacquire_dwell_s", "lost_timeout_s",
        "investigate_budget_s",
    )

    def __init__(self, *, height_cm: int = 80,
                 track_marker_ids: Optional[List[int]] = None,
                 acquire_hits: int = 3, acquire_window_s: float = 5.0,
                 acquire_dwell_s: float = 0.5, acquire_budget_s: float = 30.0,
                 center_tol_deg: float = 8.0, max_step_deg: float = 30.0,
                 track_dwell_s: float = 0.5, approach_enabled: bool = False,
                 approach_cm: int = 50, max_chase_cm: int = 1000,
                 lead_gain: float = 0.0, reacquire_dwell_s: float = 0.5,
                 lost_timeout_s: float = 4.0,
                 investigate_budget_s: float = 90.0):
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
                         ("reacquire_dwell_s", reacquire_dwell_s),
                         ("lost_timeout_s", lost_timeout_s),
                         ("investigate_budget_s", investigate_budget_s)):
            _pos_num(key, val)
        if not isinstance(approach_enabled, bool):
            raise _bad("approach_enabled", approach_enabled, "must be a bool")
        if (not isinstance(lead_gain, (int, float))
                or isinstance(lead_gain, bool)
                or not math.isfinite(lead_gain) or lead_gain < 0):
            raise _bad("lead_gain", lead_gain, "must be a finite number >= 0")

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
        self.max_chase_cm = max_chase_cm
        self.lead_gain = float(lead_gain)
        self.reacquire_dwell_s = float(reacquire_dwell_s)
        self.lost_timeout_s = float(lost_timeout_s)
        self.investigate_budget_s = float(investigate_budget_s)

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
        self._prev_bearing: Optional[float] = None
        self._prev_bearing_ts: Optional[float] = None

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
            return Done(f"track_convoy[{ctx.drone_id}]: investigate budget "
                        f"{self.investigate_budget_s:g}s elapsed before a lock "
                        f"— handing back")
        if self._t_acquire_start is None:          # defensive: always set above
            self._t_acquire_start = ctx.now
        acq_elapsed = ctx.now - self._t_acquire_start
        self._ingest(ctx)
        locked = self._best_candidate()
        if locked is not None:
            self._target_id = locked
            self._state = "track"
            self._t_last_seen = ctx.now
            return None                    # steer this same tick
        if acq_elapsed > self.acquire_budget_s:
            return Done(f"track_convoy[{ctx.drone_id}]: no id reached "
                        f"{self.acquire_hits} hit(s) within "
                        f"{self.acquire_budget_s:g}s — nothing to track, "
                        f"handing back")
        return Hover(duration_s=self.acquire_dwell_s)

    def _step_track(self, ctx: AgentContext, elapsed: float) -> Action:
        if elapsed > self.investigate_budget_s:
            return self._done(ctx, f"investigate budget "
                                   f"{self.investigate_budget_s:g}s reached")
        fresh = [s for s in ctx.sightings if s.marker_id == self._target_id]
        if fresh:
            self._t_last_seen = ctx.now
            self._seen_count += len(fresh)
            return self._steer(ctx, fresh)
        # lost this tick
        assert self._t_last_seen is not None
        if ctx.now - self._t_last_seen > self.lost_timeout_s:
            self._lost_events += 1         # drop the lock and re-acquire
            self._target_id = None
            self._obs.clear()
            self._enter_acquire(ctx)       # fresh acquire_budget for the re-find
        return Hover(duration_s=self.reacquire_dwell_s)

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

    def _best_candidate(self) -> Optional[int]:
        """The id with the most hits in-window once it reaches acquire_hits;
        ties broken by the most recent observation."""
        counts: dict = {}
        latest: dict = {}
        for ts, mid in self._obs:
            counts[mid] = counts.get(mid, 0) + 1
            latest[mid] = max(latest.get(mid, ts), ts)
        ready = [mid for mid, n in counts.items() if n >= self.acquire_hits]
        if not ready:
            return None
        return max(ready, key=lambda mid: (counts[mid], latest[mid]))

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

    def _done(self, ctx: AgentContext, why: str) -> Done:
        return Done(
            f"track_convoy[{ctx.drone_id}]: tracked id={self._target_id} "
            f"({self._seen_count} sighting(s), chased {self._chase_used_cm} cm, "
            f"{self._lost_events} reacquire(s)) — {why}")
