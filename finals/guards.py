"""In-flight guards: Guard ABC, concrete watchdogs, SafetyController, AbortListener.

Surface (S5, implemented):
- TripAction (ordered IntEnum): ADVISORY < DEGRADE_DETECTION < HOLD_THIS <
  LAND_THIS < LAND_ALL. Callers log EVERY trip and act on the MAX severity.
- Trip(guard, action, reason) — reason meets the errors.py message bar:
  WHAT tripped, WHICH drone, MEASURED vs LIMIT, WHAT TO CHECK.
- GuardContext: the read-only snapshot guards judge. ONE type for both
  evaluation sites — per-drone (agent loop) and swarm-level (the "mission"
  pseudo id, orchestrator tick); fields that do not apply are None, and a
  guard must SKIP (return None) on missing inputs, never guess.
- evaluate_guards(guards, gctx, error_action=...): the only way callers run
  guards. A guard that RAISES is itself a trip (error_action; traceback to
  stderr; the remaining guards still evaluated) — a buggy guard must never
  silently disable guarding. One of this file's two blanket-catch sites
  (whitelisted in tests/test_conventions.py).
- Concrete guards (every threshold is a config/constructor param — the
  onsite rule is tune config, not code): TelemetryWatchdog, VideoWatchdog,
  BatteryGuard, MissionClockGuard, LoopOverrunGuard, GeofenceLite,
  PhaseTimeout.
- SafetyController: the LANDING SLOT (at most ONE drone in a NORMAL landing
  at a time — serialized descent through the altitude bands is half the
  collision guarantee) + the bounded land-retry ladder (default 1 Hz for
  30 s -> operator alarm) + idempotent trip execution. emergency_land NEVER
  goes through this class, so it can never wait on the slot (the agent's
  latched safe-down calls the adapter directly).
- AbortListener: dedicated THREAD watching for the abort key ('q' + Enter).

Reconciliations against the S4-landed reality (each deliberate, reviewed):
 1. WHERE GUARDS RUN: the original stub predates S4's agents-as-tasks
    architecture ("evaluated by the orchestrator each tick BEFORE phases
    step"). Per-drone guards are evaluated by EACH AGENT every loop
    iteration — after the telemetry re-read, before the phase may act;
    swarm-level guards (MissionClockGuard, LoopOverrunGuard) run on the
    orchestrator's 1 Hz tick under the "mission" pseudo id.
 2. TRIP SHAPE: the stub said check() -> TripAction | None. Enriched to
    Trip(guard, action, reason) so every trip carries its actionable
    message. ADVISORY is added to the stub's four actions because
    GeofenceLite and BatteryGuard's warn threshold are event-only by
    design. HOLD_THIS stays in the vocabulary and the agent mapping, but NO
    S5 concrete guard emits it (future yield/servo phases will).
 3. TelemetryWatchdog vs the agent's inline backstop: this guard (default
    2 s) is the POLICY layer — an orderly LAND_THIS while the link still
    half-works; the agent's 5 s SensorTimeout -> emergency_land stays as
    the MECHANISM backstop for truly dead telemetry. The tighter guard
    fires first; the emergency latch makes a double-trip harmless. The
    layering is pinned by tests/test_guards.py.
 4. CommandGuard is NOT a class here: its intent — every command completes
    or raises, and moves are NEVER blind-retried (a re-sent relative move
    doubles the distance) — is enforced by construction in the S4
    DroneAgent (typed FlightError handling, no re-step after a command
    error, emergency-land-exactly-once latch). Satisfied by design;
    duplicating it as a guard would be a second, drifting source of truth.
 5. VideoWatchdog logic is real and unit-tested against FrameStamped.ts-
    style stamps with injected values (SIM-4 assumes this guard works, so
    its deadline/response ships NOW) — but main._build_guards does NOT
    construct it until S7 plumbs a frame timestamp into the agent's
    GuardContext: built before any frame source exists, it would log a
    guaranteed-false "no frame EVER" DEGRADE on every sim run. It only
    DEGRADEs; it never lands (a blind drone can still fly home).
 6. BatteryGuard's floor trip is a CLEAN landing: LAND_THIS ends the agent
    loop via the normal break -> land -> DONE path with stopped_reason
    naming the guard — battery exhaustion handled IN TIME is a controlled
    outcome, not a failure. The warn threshold (config
    guards.battery_warn_pct) and an unreadable battery are one-shot
    ADVISORY events.
 7. MissionClockGuard defaults OFF (config landing_reserve_s = 0; main.py
    builds it only when > 0): a non-zero default would instant-trip short
    --budget smoke runs. When enabled it fires LAND_ALL at budget - reserve
    so every drone is DOWN before the budget, not at it.
 8. LoopOverrunGuard's stub ladder ("shed detection -> display -> LAND_ALL")
    is reduced to two stages (DEGRADE_DETECTION -> LAND_ALL): the shedding
    targets (detector workers, display) arrive in S7; the staging logic
    ships now and the S7 components subscribe to DEGRADE_DETECTION.
 9. GeofenceLite is ADVISORY ONLY: position is DEAD_RECKONING (drifts,
    untrusted — finals/flight/dead_reckon.py); acting on it could fly a
    drone INTO the boundary it imagines it is escaping.
10. STALE STUB NOTE: the stub said "one of the THREE whitelisted sites";
    the conventions test whitelists two FILES (this one +
    mission/orchestrator.py). This file hosts two blanket-catch SITES: the
    evaluate_guards wrapper and the SafetyController retry ladder.
11. AbortListener honesty: a thread cannot execute coroutines against the
    async adapters, and a WEDGED event loop cannot run them either — so the
    stub's "directly fires emergency_land" is not implementable from here.
    The listener sets the threading.Event, wakes the loop (best-effort
    loop.call_soon_threadsafe via on_abort), and SCREAMS; the
    orchestrator's per-tick poll of the event is the reliable consumer.
    True loop-independent kill arrives with the blocking SDK (S9).
    Safety-only channel: it can only land everything, never steer.

Derives from: qualifier_run.py:383-393 (the proven crash -> emergency-land
path: bring the drone down BEFORE re-raising; a throw from one drone's
landing must not prevent landing the others) and the mapping_drone.py gap
audit in docs/finals/README.md (the UWB wait-forever at line 129; telemetry
tasks that never exit — the bug classes every guard here exists to close).

Session: S5 (implemented).
"""
from __future__ import annotations

import asyncio
import enum
import math
import sys
import threading
import time
import traceback
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import (TYPE_CHECKING, Callable, Dict, List, Optional, Sequence,
                    TextIO)

from finals.errors import FlightError, FlightTimeout
from finals.events import EventLog, EventLogError
from finals.types import Telemetry

if TYPE_CHECKING:  # type hints only — keeps the import graph minimal
    from finals.flight.adapter import FlightAdapter


# ============================================================
# Trip vocabulary
# ============================================================
class TripAction(enum.IntEnum):
    """Ordered by severity — callers act on the MAX of all trips this tick."""

    ADVISORY = 1            # event only; flight unaffected (reconciliation 2)
    DEGRADE_DETECTION = 2   # shed detection load / restart video; never lands
    HOLD_THIS = 3           # this drone skips its phase step this tick
    LAND_THIS = 4           # this drone lands CLEAN (agent break path), DONE
    LAND_ALL = 5            # mission stop: every drone lands clean


@dataclass(frozen=True)
class Trip:
    """One guard firing once. reason meets the errors.py message bar."""

    guard: str              # guard class name (greppable in mission.jsonl)
    action: TripAction
    reason: str


@dataclass(frozen=True)
class GuardContext:
    """Read-only inputs for one guard evaluation. Per-drone fields are None
    at the swarm level and vice versa — guards skip on missing inputs."""

    drone_id: str                                # "mission" for swarm-level
    now: float                                   # caller's monotonic clock
    mission_elapsed_s: float
    telemetry: Optional[Telemetry] = None        # per-drone only
    phase_name: Optional[str] = None             # per-drone only
    phase_elapsed_s: Optional[float] = None      # None until phase entered
    last_frame_ts: Optional[float] = None        # per-drone; None = no frame yet
    tick_latency_s: Optional[float] = None       # swarm-level only: the
                                                 # BEAT-TO-BEAT supervision gap
                                                 # (None on the first beat) —
                                                 # a starved loop shows here;
                                                 # the drain-only duration
                                                 # would NOT (it has no awaits
                                                 # to starve)


# ============================================================
# Guard ABC + the evaluation wrapper
# ============================================================
class Guard(ABC):
    """One watchdog. Instances hold per-drone latch/counter state, so the
    wiring MUST build a fresh instance per drone per mission (main.py's
    _build_guards does) — a shared instance would cross-contaminate latches."""

    @abstractmethod
    def check(self, gctx: GuardContext) -> Optional[Trip]:
        """Judge one snapshot. No I/O, no sleeps, no SDK calls. Return a
        Trip to fire, None to stay quiet. Missing inputs (None fields the
        guard needs) -> None, never a guess."""

    def _trip(self, action: TripAction, reason: str) -> Trip:
        return Trip(guard=type(self).__name__, action=action, reason=reason)


def _check_threshold(owner: str, name: str, value, *,
                     zero_ok: bool = False) -> float:
    """Shared constructor gate: finite number, > 0 (or >= 0 when zero_ok) —
    an unbounded/NaN threshold would silently disable the guard."""
    if (not isinstance(value, (int, float)) or isinstance(value, bool)
            or not math.isfinite(value)
            or value < 0 or (value == 0 and not zero_ok)):
        raise ValueError(
            f"{owner}: {name} must be finite and "
            f"{'>= 0' if zero_ok else '> 0'}, got {value!r} — a bad "
            f"threshold would silently disable the guard; check the "
            f"config/wiring")
    return float(value)


def evaluate_guards(guards: Sequence[Guard], gctx: GuardContext, *,
                    error_action: TripAction = TripAction.LAND_THIS
                    ) -> List[Trip]:
    """Run every guard against one snapshot; return ALL trips (callers log
    each and act on the max severity).

    A guard that RAISES — or returns something that is not a Trip — is
    itself converted to a trip with `error_action` (per-drone callers pass
    LAND_THIS, the orchestrator passes LAND_ALL): a buggy guard must never
    silently disable guarding, and one broken guard must never stop the
    rest from being evaluated. Always logged with the full traceback.
    """
    trips: List[Trip] = []
    for guard in guards:
        name = type(guard).__name__
        try:
            trip = guard.check(gctx)
        except Exception:
            # WHITELISTED blanket-catch site (tests/test_conventions.py):
            # a raising guard is a trip, never a silent disable. Traceback
            # always printed; the remaining guards still run.
            tb = traceback.format_exc()
            print(f"[guards] ERROR {gctx.drone_id}: guard {name}.check() "
                  f"RAISED — treating as a {error_action.name} trip; "
                  f"remaining guards still evaluated:\n{tb}",
                  file=sys.stderr, flush=True)
            trips.append(Trip(
                guard=name, action=error_action,
                reason=f"{gctx.drone_id}: guard {name} raised instead of "
                       f"judging (guard bug; traceback on stderr) — treated "
                       f"as {error_action.name} because a broken guard "
                       f"cannot be trusted to stay quiet — check the guard "
                       f"code/thresholds"))
            continue
        if trip is None:
            continue
        if not isinstance(trip, Trip):
            print(f"[guards] ERROR {gctx.drone_id}: guard {name}.check() "
                  f"returned {trip!r} — not a Trip/None (guard bug); "
                  f"treating as a {error_action.name} trip",
                  file=sys.stderr, flush=True)
            trips.append(Trip(
                guard=name, action=error_action,
                reason=f"{gctx.drone_id}: guard {name} returned {trip!r} "
                       f"instead of Trip/None (guard bug) — treated as "
                       f"{error_action.name} — check the guard code"))
            continue
        trips.append(trip)
    return trips


# ============================================================
# Concrete guards
# ============================================================
class TelemetryWatchdog(Guard):
    """Stale telemetry -> LAND_THIS. Policy layer over the agent's 5 s
    SensorTimeout mechanism backstop (reconciliation 3): land in an orderly
    way while the link still half-works instead of waiting for it to die."""

    def __init__(self, stale_s: float = 2.0):
        self._stale_s = _check_threshold("TelemetryWatchdog", "stale_s", stale_s)

    def check(self, gctx: GuardContext) -> Optional[Trip]:
        if gctx.telemetry is None:
            return None
        age_s = gctx.telemetry.age_s(gctx.now)
        if age_s <= self._stale_s:
            return None
        return self._trip(
            TripAction.LAND_THIS,
            f"{gctx.drone_id}: telemetry age {age_s:.1f} s exceeds the "
            f"{self._stale_s:.1f} s guard limit (agent emergency backstop "
            f"fires later) — telemetry poller/link degrading; landing this "
            f"drone cleanly while commands still work — check Wi-Fi / the "
            f"backend poller")


class VideoWatchdog(Guard):
    """Stale/absent video -> DEGRADE_DETECTION (never lands — a blind drone
    can still fly home). Edge-triggered: one trip per stale episode, re-arms
    when a fresh frame is seen. No frame EVER seen is judged against the
    first check() time (the guard's anchor)."""

    def __init__(self, stale_s: float = 2.0):
        self._stale_s = _check_threshold("VideoWatchdog", "stale_s", stale_s)
        self._anchor: Optional[float] = None     # first check() time
        self._was_stale = False                  # edge latch

    def check(self, gctx: GuardContext) -> Optional[Trip]:
        ref = gctx.last_frame_ts
        never = ref is None
        if never:
            if self._anchor is None:
                self._anchor = gctx.now
            ref = self._anchor
        age_s = gctx.now - ref
        if age_s <= self._stale_s:
            self._was_stale = False              # fresh frame re-arms
            return None
        if self._was_stale:
            return None                          # already reported this episode
        self._was_stale = True
        what = ("no frame EVER received since guard start"
                if never else f"last frame is {age_s:.1f} s old")
        return self._trip(
            TripAction.DEGRADE_DETECTION,
            f"{gctx.drone_id}: video stale — {what} (limit "
            f"{self._stale_s:.1f} s) — degrading detection; flight "
            f"unaffected — check the video source/stream")


class BatteryGuard(Guard):
    """Floor -> LAND_THIS (clean land, DONE — reconciliation 6); warn and
    unreadable battery -> one-shot ADVISORY each (latched; batteries do not
    recover mid-flight, so the warn never re-arms)."""

    def __init__(self, floor_pct: float, warn_pct: float):
        self._floor_pct = _check_threshold("BatteryGuard", "floor_pct",
                                           floor_pct, zero_ok=True)
        self._warn_pct = _check_threshold("BatteryGuard", "warn_pct",
                                          warn_pct, zero_ok=True)
        if not (0.0 <= self._floor_pct <= self._warn_pct <= 100.0):
            raise ValueError(
                f"BatteryGuard: need 0 <= floor_pct <= warn_pct <= 100, got "
                f"floor={floor_pct!r} warn={warn_pct!r} — the warn must come "
                f"BEFORE the floor on the way down; check min_battery_pct / "
                f"guards.battery_warn_pct in the config")
        self._warned = False
        self._warned_unknown = False

    def check(self, gctx: GuardContext) -> Optional[Trip]:
        if gctx.telemetry is None:
            return None
        pct = gctx.telemetry.battery_pct
        if pct is None:
            if self._warned_unknown:
                return None
            self._warned_unknown = True
            return self._trip(
                TripAction.ADVISORY,
                f"{gctx.drone_id}: battery level UNKNOWN (telemetry carries "
                f"no battery_pct) — battery guarding is BLIND on this drone "
                f"— check the telemetry backend exposes battery")
        if pct <= self._floor_pct:
            return self._trip(
                TripAction.LAND_THIS,
                f"{gctx.drone_id}: battery {pct:.0f}% is at/under the "
                f"{self._floor_pct:.0f}% floor — landing this drone cleanly "
                f"NOW while there is power to do it — check battery health/"
                f"mission length if this fired early")
        if pct <= self._warn_pct and not self._warned:
            self._warned = True
            return self._trip(
                TripAction.ADVISORY,
                f"{gctx.drone_id}: battery {pct:.0f}% is at/under the "
                f"{self._warn_pct:.0f}% warn threshold (floor "
                f"{self._floor_pct:.0f}%) — plan the remaining mission "
                f"accordingly")
        return None


class MissionClockGuard(Guard):
    """LAND_ALL at budget - reserve, so drones are DOWN before the budget,
    not at it (reconciliation 7). One-shot: the stop it causes is latched
    anyway; re-tripping every tick would only spam the log."""

    def __init__(self, budget_s: float, landing_reserve_s: float):
        self._budget_s = _check_threshold("MissionClockGuard", "budget_s",
                                          budget_s)
        self._reserve_s = _check_threshold("MissionClockGuard",
                                           "landing_reserve_s",
                                           landing_reserve_s)
        if self._reserve_s >= self._budget_s:
            raise ValueError(
                f"MissionClockGuard: landing_reserve_s ({landing_reserve_s!r}) "
                f">= budget_s ({budget_s!r}) — the guard would trip at t=0; "
                f"check guards.landing_reserve_s vs mission_budget_s")
        self._fired = False

    def check(self, gctx: GuardContext) -> Optional[Trip]:
        threshold_s = self._budget_s - self._reserve_s
        if self._fired or gctx.mission_elapsed_s < threshold_s:
            return None
        self._fired = True
        return self._trip(
            TripAction.LAND_ALL,
            f"{gctx.drone_id}: mission clock {gctx.mission_elapsed_s:.1f} s "
            f"reached the land-all threshold {threshold_s:.1f} s (budget "
            f"{self._budget_s:.0f} s minus landing reserve "
            f"{self._reserve_s:.0f} s) — landing ALL drones so they are "
            f"down BEFORE the budget expires — check guards.landing_reserve_s "
            f"if this fired earlier than planned")


class LoopOverrunGuard(Guard):
    """Supervision-loop health from the BEAT-TO-BEAT gap (GuardContext.
    tick_latency_s): n_ticks consecutive overruns (gap > factor x period)
    -> DEGRADE_DETECTION once; another n_ticks still overrunning ->
    LAND_ALL. A healthy beat resets the ladder (reconciliation 8: two
    stages until S7's shed targets exist). The gap — not any single
    section's duration — is what a starved/blocked event loop stretches."""

    def __init__(self, period_s: float, factor: float = 2.0,
                 n_ticks: int = 5):
        self._period_s = _check_threshold("LoopOverrunGuard", "period_s",
                                          period_s)
        self._factor = _check_threshold("LoopOverrunGuard", "factor", factor)
        if self._factor <= 1.0:
            raise ValueError(
                f"LoopOverrunGuard: factor must be > 1 (a factor <= 1 trips "
                f"on a HEALTHY loop), got {factor!r} — check "
                f"guards.loop_overrun_factor")
        if not isinstance(n_ticks, int) or isinstance(n_ticks, bool) \
                or n_ticks < 1:
            raise ValueError(
                f"LoopOverrunGuard: n_ticks must be an int >= 1, got "
                f"{n_ticks!r} — check guards.loop_overrun_ticks")
        self._n_ticks = n_ticks
        self._consecutive = 0
        self._degrade_fired = False
        self._land_all_fired = False

    def check(self, gctx: GuardContext) -> Optional[Trip]:
        latency_s = gctx.tick_latency_s
        if latency_s is None:
            return None
        limit_s = self._factor * self._period_s
        if latency_s <= limit_s:
            self._consecutive = 0
            self._degrade_fired = False
            self._land_all_fired = False
            return None
        self._consecutive += 1
        measured = (f"beat gap {latency_s:.3f} s > limit {limit_s:.3f} s "
                    f"({self._factor:g} x {self._period_s:g} s period) for "
                    f"{self._consecutive} consecutive beat(s)")
        if not self._degrade_fired and self._consecutive >= self._n_ticks:
            self._degrade_fired = True
            return self._trip(
                TripAction.DEGRADE_DETECTION,
                f"{gctx.drone_id}: supervision loop overrunning — {measured} "
                f"— shedding detection load — check CPU load / a blocking "
                f"call on the event loop")
        if not self._land_all_fired and self._consecutive >= 2 * self._n_ticks:
            self._land_all_fired = True
            return self._trip(
                TripAction.LAND_ALL,
                f"{gctx.drone_id}: supervision loop STILL overrunning after "
                f"shedding — {measured} — the loop can no longer supervise "
                f"flying drones; landing ALL — check CPU load / a blocking "
                f"call on the event loop")
        return None


class GeofenceLite(Guard):
    """ADVISORY ONLY (reconciliation 9): the position is DEAD_RECKONING and
    must never be acted on. Edge-triggered per breach episode."""

    def __init__(self, radius_m: float, alt_max_m: Optional[float] = None):
        self._radius_m = _check_threshold("GeofenceLite", "radius_m", radius_m)
        self._alt_max_m = (None if alt_max_m is None else
                           _check_threshold("GeofenceLite", "alt_max_m",
                                            alt_max_m))
        self._breached = False                   # edge latch

    def check(self, gctx: GuardContext) -> Optional[Trip]:
        t = gctx.telemetry
        if t is None or t.position_m is None:
            return None
        north_m, east_m, alt_m = t.position_m
        radius = math.hypot(north_m, east_m)
        over_r = radius > self._radius_m
        over_a = self._alt_max_m is not None and alt_m > self._alt_max_m
        if not (over_r or over_a):
            self._breached = False
            return None
        if self._breached:
            return None                          # one advisory per episode
        self._breached = True
        parts = []
        if over_r:
            parts.append(f"radius {radius:.1f} m > {self._radius_m:.1f} m")
        if over_a:
            parts.append(f"altitude {alt_m:.1f} m > {self._alt_max_m:.1f} m")
        return self._trip(
            TripAction.ADVISORY,
            f"{gctx.drone_id}: dead-reckoned position outside the soft "
            f"geofence ({'; '.join(parts)}) — ADVISORY ONLY: the estimate "
            f"drifts and is never acted on — check the drone visually / "
            f"the mission pattern")


class PhaseTimeout(Guard):
    """A phase running past its budget -> LAND_THIS (a wedged phase on a
    real aircraft burns battery going nowhere; land it cleanly)."""

    def __init__(self, timeout_s: float):
        self._timeout_s = _check_threshold("PhaseTimeout", "timeout_s",
                                           timeout_s)

    def check(self, gctx: GuardContext) -> Optional[Trip]:
        if gctx.phase_elapsed_s is None:
            return None
        if gctx.phase_elapsed_s <= self._timeout_s:
            return None
        return self._trip(
            TripAction.LAND_THIS,
            f"{gctx.drone_id}: phase {gctx.phase_name!r} has run "
            f"{gctx.phase_elapsed_s:.1f} s, over its {self._timeout_s:.1f} s "
            f"budget — landing this drone cleanly — check the phase logic "
            f"or raise guards.phase_timeout_s")


# ============================================================
# SafetyController
# ============================================================
def _consume_task_exception(task: asyncio.Task) -> None:
    """Done-callback for the shielded trip-landing task: mark its exception
    as observed so a cancelled awaiter cannot leave 'exception was never
    retrieved' noise. NOT a swallow — the retry ladder already screamed
    (CRITICAL stderr + safety_escalation event) before raising."""
    if not task.cancelled():
        task.exception()


class SafetyController:
    """Serialized NORMAL landings + bounded retry ladder + idempotent trips.

    One instance per mission, shared by every agent, used inside ONE
    asyncio.run (the slot semaphore is created lazily so one controller
    serves exactly one running mission loop; asyncio binds it at first
    await).

    - land(): acquire the landing slot (BOUNDED wait, actionable timeout),
      then attempt adapter.land() — each attempt individually bounded by
      timeout_s + grace — until success, or until EITHER the attempt count
      (ceil(window/period)) OR the wall-clock window runs out, whichever
      comes first. The wall-clock bound matters in the HANG failure mode:
      attempts that each burn the full outer deadline must not stretch a
      "30 s" ladder into minutes before the operator hears about it.
      Exhaustion -> OPERATOR ALARM (CRITICAL stderr + safety_escalation
      event) + raise FlightError (the agent's _fail then emergency-lands,
      latched, slot-free). Repeatable per drone — multi-phase missions
      land more than once; only an ESCALATED drone latches (re-entry
      re-raises instead of burning another window on a dead link).
    - trip(): idempotent trip execution, COMPLETION-shared — the FIRST
      trip for a drone starts the landing as its own task; every trip call
      (first or re-trip) awaits THAT task's outcome, so a racing second
      trip source can never observe "landed" while the first descent is
      still in the air. The landing task is shielded: a cancelled trip
      caller never cancels a safe-down in flight.
    - emergency_land is deliberately NOT here: the agent calls the adapter
      directly (never-raise contract), so an emergency can never wait on
      the slot. Pinned by tests/test_guards.py. Consequence, documented:
      a drone whose slot wait times out fails over to its agent's FAILED
      path and emergency-lands CONCURRENTLY with the slot holder's descent
      — when the slot itself is wedged, safety beats serialization; the
      orchestrator's settle grace stays the hard floor.
    """

    def __init__(self, events: EventLog, *,
                 land_retry_period_s: float = 1.0,
                 land_retry_window_s: float = 30.0,
                 command_timeout_s: float = 15.0,
                 command_grace_s: float = 2.0,
                 slot_wait_s: float = 120.0,
                 clock: Callable[[], float] = time.monotonic):
        for name, value, zero_ok in (
                ("land_retry_period_s", land_retry_period_s, False),
                ("land_retry_window_s", land_retry_window_s, False),
                ("command_timeout_s", command_timeout_s, False),
                ("command_grace_s", command_grace_s, True),
                ("slot_wait_s", slot_wait_s, False)):
            _check_threshold("SafetyController", name, value, zero_ok=zero_ok)
        if land_retry_window_s < land_retry_period_s:
            raise ValueError(
                f"SafetyController: land_retry_window_s "
                f"({land_retry_window_s!r}) < land_retry_period_s "
                f"({land_retry_period_s!r}) — the ladder would never retry; "
                f"check guards.land_retry_* in the config")
        self._events = events
        self._period_s = float(land_retry_period_s)
        self._window_s = float(land_retry_window_s)
        self._command_timeout_s = float(command_timeout_s)
        self._command_grace_s = float(command_grace_s)
        self._slot_wait_s = float(slot_wait_s)
        self._clock = clock
        self._attempts = max(1, math.ceil(self._window_s / self._period_s))
        self._slot: Optional[asyncio.Semaphore] = None   # lazy: loop-bound
        self._tripped: Dict[str, str] = {}               # drone -> first reason
        self._trip_tasks: Dict[str, asyncio.Task] = {}   # drone -> landing task
        self._escalations: Dict[str, str] = {}           # drone -> alarm text

    # -------- introspection --------
    @property
    def land_attempts(self) -> int:
        return self._attempts

    @property
    def land_bound_s(self) -> float:
        """Worst-case land() duration: slot wait + the wall-clock window +
        one full outer deadline (the last attempt may start just inside the
        window edge) + one inter-attempt gap. Callers logging deadlines
        should cite THIS, not the single-command outer."""
        outer = self._command_timeout_s + self._command_grace_s
        return self._slot_wait_s + self._window_s + outer + self._period_s

    # -------- helpers --------
    def _try_log(self, drone_id: str, event: str, **data) -> None:
        try:
            self._events.log(drone_id, event, **data)
        except EventLogError as e:
            # Forensics must never block a landing.
            print(f"[SafetyController] WARNING: could not log {event!r}: {e}",
                  file=sys.stderr, flush=True)

    def _slot_sem(self) -> asyncio.Semaphore:
        if self._slot is None:
            # Lazy so one controller serves exactly one mission loop;
            # asyncio binds the semaphore to a loop at its first await.
            self._slot = asyncio.Semaphore(1)
        return self._slot

    # -------- the landing paths --------
    async def land(self, adapter: "FlightAdapter", drone_id: str) -> None:
        """NORMAL landing: slot + retry ladder. Raises FlightError on
        escalation (caller safe-downs via its own latch)."""
        prior = self._escalations.get(drone_id)
        if prior is not None:
            raise FlightError(
                f"{drone_id}: landing previously ESCALATED — refusing to "
                f"burn another {self._window_s:.0f} s ladder on it; first "
                f"alarm: {prior}")
        sem = self._slot_sem()
        try:
            # asyncio.timeout, NOT wait_for: 3.11's wait_for can swallow an
            # external cancellation that races the acquire completing — a
            # settle-deadline cancel would then be lost and the full ladder
            # would keep running on a task the orchestrator thinks is dead.
            async with asyncio.timeout(self._slot_wait_s):
                await sem.acquire()
        except TimeoutError:
            raise FlightTimeout(
                f"{drone_id}: landing slot still held by another drone "
                f"after {self._slot_wait_s:.1f} s — serialized descent "
                f"could not start — check heartbeat.json for which drone's "
                f"landing is stuck") from None
        try:
            await self._land_with_retries(adapter, drone_id)
        finally:
            sem.release()

    async def trip(self, adapter: "FlightAdapter", drone_id: str,
                   reason: str) -> None:
        """Idempotent, COMPLETION-shared trip execution: the first trip for
        a drone starts the landing as its own (shielded) task; every call
        awaits that task's outcome. A re-trip therefore never reports
        success while the first descent is still in the air — both callers
        see the landing complete, or both see the same escalation."""
        task = self._trip_tasks.get(drone_id)
        if task is None:
            self._tripped[drone_id] = reason
            self._try_log(drone_id, "safety_trip", reason=reason)
            task = asyncio.get_running_loop().create_task(
                self.land(adapter, drone_id),
                name=f"safety-trip:{drone_id}")
            # If every awaiting caller is cancelled, the safe-down still
            # runs to completion; its failure was already screamed by the
            # ladder — this only marks the exception as observed.
            task.add_done_callback(_consume_task_exception)
            self._trip_tasks[drone_id] = task
        else:
            self._try_log(drone_id, "safety_retrip_ignored", reason=reason,
                          first_trip=self._tripped.get(drone_id, "?"))
        # shield: a cancelled trip CALLER must never cancel a safe-down
        # already in flight (same argument as the agent's emergency latch).
        await asyncio.shield(task)

    async def _land_with_retries(self, adapter: "FlightAdapter",
                                 drone_id: str) -> None:
        outer_s = self._command_timeout_s + self._command_grace_s
        t_start = self._clock()
        last_error = "(no attempt ran)"
        attempts_made = 0
        for attempt in range(1, self._attempts + 1):     # bounded by count
            attempts_made = attempt
            try:
                async with asyncio.timeout(outer_s):
                    await adapter.land(timeout_s=self._command_timeout_s)
                if attempt > 1:
                    self._try_log(drone_id, "safety_land_recovered",
                                  attempt=attempt)
                return
            except TimeoutError:
                last_error = (f"land() still running after the outer "
                              f"{outer_s:.1f} s deadline (its own timeout_s="
                              f"{self._command_timeout_s:.1f} never fired — "
                              f"backend hang)")
                print(f"[SafetyController] {drone_id}: land attempt "
                      f"{attempt}/{self._attempts} hung: {last_error}",
                      file=sys.stderr, flush=True)
            except Exception:
                # WHITELISTED blanket-catch site (tests/test_conventions.py):
                # a throw from THIS attempt must not skip the remaining
                # retries — landing is the one command we keep retrying.
                # Always with traceback.
                tb = traceback.format_exc()
                last_error = tb.strip().splitlines()[-1]
                print(f"[SafetyController] {drone_id}: land attempt "
                      f"{attempt}/{self._attempts} failed — retrying:\n{tb}",
                      file=sys.stderr, flush=True)
            if attempt >= self._attempts:
                break
            if self._clock() - t_start >= self._window_s:
                # The HANG failure mode: attempts can each burn the full
                # outer deadline, so the WALL CLOCK — not the attempt count
                # — must bound how late the operator alarm can arrive.
                break
            await asyncio.sleep(self._period_s)          # bounded: count+wall
        elapsed_s = self._clock() - t_start
        alarm = (
            f"{drone_id}: landing FAILED after {attempts_made} attempt(s) "
            f"over {elapsed_s:.0f} s (window {self._window_s:.0f} s, "
            f"~{self._period_s:g} s apart) — last error: {last_error} — "
            f"OPERATOR ALARM: the drone may still be airborne; be ready "
            f"for a manual kill — check the link/props/battery")
        self._escalations[drone_id] = alarm
        print(f"[SafetyController] CRITICAL {alarm}",
              file=sys.stderr, flush=True)
        self._try_log(drone_id, "safety_escalation",
                      attempts=attempts_made, elapsed_s=round(elapsed_s, 1),
                      window_s=self._window_s, last_error=last_error)
        raise FlightError(alarm)


# ============================================================
# AbortListener
# ============================================================
class AbortListener:
    """Operator abort channel on a dedicated THREAD (reconciliation 11):
    'q' + Enter -> abort_event.set() + best-effort on_abort() (wire the
    orchestrator's request_stop_threadsafe there for a prompt loop wakeup;
    the orchestrator's per-tick poll of abort_event is the reliable path).

    - source: anything with readline() (injectable for tests); default
      sys.stdin, resolved at start(). Under pytest stdin reads raise
      OSError; a service may have sys.stdin None — both are handled typed
      and just disable the key (Ctrl+C remains).
    - The thread is daemon: a real stdin readline cannot be unblocked, and
      a blocked abort thread must never hold the process open after the
      mission ends. stop() joins with a timeout and reports whether the
      thread actually ended (fake-source tests assert True).
    - One-shot and safety-only: after 'q' it fires once and exits; it can
      only LAND everything (via the stop machinery), never steer.
    """

    def __init__(self, abort_event: threading.Event, *,
                 source: Optional[TextIO] = None,
                 on_abort: Optional[Callable[[], None]] = None):
        if not isinstance(abort_event, threading.Event):
            raise ValueError(
                f"AbortListener: abort_event must be a threading.Event, got "
                f"{type(abort_event).__name__!r} — check the main.py wiring")
        self._abort_event = abort_event
        self._source = source
        self._on_abort = on_abort
        self._stop_requested = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError(
                "AbortListener.start() called twice — one listener, one "
                "thread, one mission; check the wiring")
        if self._source is None and sys.stdin is None:
            print("[AbortListener] WARNING: no stdin (service/pythonw?) — "
                  "abort key disabled; Ctrl+C remains the abort path",
                  file=sys.stderr, flush=True)
            return
        print("[AbortListener] abort key armed: press 'q' + Enter to LAND "
              "ALL drones", file=sys.stderr, flush=True)
        self._thread = threading.Thread(
            target=self._run, name="abort-listener", daemon=True)
        self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> bool:
        """Ask the thread to wind down and join it. Returns True when the
        thread has actually ended (False = still blocked on a real stdin
        readline — daemon, so it cannot hold the process open)."""
        self._stop_requested.set()
        if self._thread is None:
            return True
        self._thread.join(timeout_s)
        return not self._thread.is_alive()

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # -------- the thread body --------
    def _run(self) -> None:
        source = self._source if self._source is not None else sys.stdin
        # Bounded by the stop event (convention 3) + the one-shot return
        # paths below.
        while not self._stop_requested.is_set():
            try:
                line = source.readline()
            except (EOFError, OSError, ValueError) as e:
                # Typed: pytest's captured stdin raises OSError; a closed
                # StringIO raises ValueError. The key is disabled, loudly.
                print(f"[AbortListener] stdin unavailable "
                      f"({type(e).__name__}: {e}) — abort key disabled; "
                      f"Ctrl+C remains", file=sys.stderr, flush=True)
                return
            if line == "":                       # EOF — same story
                print("[AbortListener] stdin EOF — abort key disabled; "
                      "Ctrl+C remains", file=sys.stderr, flush=True)
                return
            if self._stop_requested.is_set():
                return                           # mission over — stale input
            if line.strip().lower() != "q":
                continue                         # not the key; keep watching
            print("=" * 64 + "\n[AbortListener] OPERATOR ABORT ('q'): "
                  "landing ALL drones\n" + "=" * 64,
                  file=sys.stderr, flush=True)
            self._abort_event.set()
            if self._on_abort is not None:
                try:
                    self._on_abort()
                except RuntimeError as e:
                    # Loop already closed (mission over) — the abort_event
                    # poll/flag is still set; nothing airborne to wake.
                    print(f"[AbortListener] loop wakeup skipped: {e}",
                          file=sys.stderr, flush=True)
            return                               # one-shot
