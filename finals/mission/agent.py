"""DroneAgent — one drone's runtime: phase queue, single in-flight command,
hard deadline on every await, emergency-land-exactly-once on failure.

Surface (S4, implemented):
- AgentState: INIT -> (run) -> READY -> RUNNING -> LANDING -> DONE; any
  failure -> FAILED. FAILED is TERMINAL: there is NO auto-restart — a
  crash-restart that re-arms a real aircraft is unsafe (the deliberate
  departure from qualifier_run.py:407-513's supervisor, see orchestrator.py).
- run(deadline, stop_event): the agent's whole life as ONE coroutine (the
  orchestrator runs one task per agent — gather-isolation, pinned by
  tests/test_mock_adapter.py's two-adapter test). Sequential awaits inside
  one task are what GUARANTEE the FlightAdapter non-reentrancy contract:
  at most one in-flight command per drone, by construction.
- Every command runs under asyncio.wait_for with an OUTER deadline of
  command_timeout_s + command_grace_s (hover: + duration_s): the adapter is
  passed timeout_s=command_timeout_s and is expected to enforce it; the
  outer wait_for is the belt-and-suspenders watchdog for a backend that
  hangs PAST its own deadline (the mapping_drone.py:129 wait-forever class).
  An outer trip is converted to a typed FlightTimeout naming the adapter bug.
- On FlightError/FlightTimeout from any command: emergency_land EXACTLY ONCE
  (a per-agent latch, asserted via MockAdapter.calls in tests), log, FAILED.
  The safe-down is commanded BEFORE the failure is logged: safety beats
  forensics, and a full-disk EventLogError must not block the landing.
  emergency_land is always commanded on failure even if the agent believes
  the drone is grounded — after a FlightTimeout the true airborne state is
  UNKNOWN (the SDK may still be executing the command).
- Done(reason) -> on_exit -> next phase (on_enter) -> queue empty -> LANDING
  (land only if airborne — a phase that already landed is not re-landed) ->
  DONE. Abort(reason) -> FAILED + safe-down. Wait(t) -> bounded idle that
  wakes early on the stop event.
- Telemetry is re-read every tick and its age checked against
  telemetry_stale_s BEFORE the phase may act on it; stale telemetry raises
  SensorTimeout -> safe-down (acting on a dead poller's last words is the
  mapping_drone.py watchdog-gap class). The agent's clock and the adapter's
  telemetry stamps must share one monotonic domain (inject the same fake in
  tests).
- shutdown(): emergency-land if still airborne (abnormal path; same latch),
  then disconnect — both bounded by wait_for; never raises on purpose
  (disconnect's contract is never-raise; a hang is caught as TimeoutError
  and screamed to stderr + events).
- Blanket exception catching is deliberately ABSENT here (whitelist is
  guards.py + orchestrator.py only): the agent handles typed FlightError /
  SensorTimeout / asyncio.TimeoutError; anything unexpected (a phase bug,
  EventLogError) propagates out of run() to the orchestrator's top-loop
  net, which logs the traceback and calls fail_safe() — same latch, same
  exactly-once landing.
- Guards (S5): when wired, the per-drone guard list runs through
  finals.guards.evaluate_guards every loop iteration — after the telemetry
  re-read, BEFORE the phase may act. Every trip is logged (guard_trip) and
  the agent acts on the MAX severity: ADVISORY/DEGRADE_DETECTION -> event
  only; HOLD_THIS -> bounded idle (hold_poll_s, wakes on stop/deadline),
  phase NOT stepped this tick; LAND_THIS -> the clean break path below
  (land -> DONE, stopped_reason names the guard); LAND_ALL -> set the
  SHARED stop event (every agent winds down clean) and break. A guard that
  raises becomes a trip INSIDE the wrapper (the catch lives in guards.py),
  so this file gains no new exception handling for it.
- SafetyController (S5): when wired, Land commands (phase-commanded and
  the final descent) route through it — the landing slot (at most one
  NORMAL landing at a time) plus the bounded land-retry ladder. That path
  is internally bounded (slot wait + per-attempt deadlines + a fixed
  attempt count), so it deliberately does NOT run under the single-command
  outer wait_for, which would kill the ladder mid-attempt and mislabel a
  retrying landing as a backend hang; action_start carries route="safety"
  and the ladder's total bound instead. emergency_land NEVER routes
  through the controller: the latched safe-down calls the adapter directly
  and can never wait on the landing slot.

Event vocabulary written to EventLog (the run's forensic story, and the
replay-plot input — simulation.md Tier 0): agent_connect, origin (initial
pose/origin: the replay prereq), phase_enter, action_start,
action_complete (the executed-action record DeadReckoner replays),
action_failed, phase_done, phase_abort, guard_trip, agent_stopped,
agent_landing, agent_done, agent_failed, emergency_land, agent_disconnect.

Derives from: the per-drone dict + state-loop pattern of
hula_connection.py:39-63 (officially recommended), formalized as one agent
object per drone stepping pure phases. Bugs fixed in adaptation:
- mapping_drone.py:129's unbounded wait-for-sensor loop class: every await
  here is bounded by wait_for, the run loop by deadline + stop event + the
  finite phase queue (convention 3), and stale telemetry is a typed failure
  instead of a silent trust.
- mapping_drone.py's telemetry tasks that never exit: this agent owns NO
  background tasks at all — one coroutine, one drone, sequential awaits.
- qualifier_run.py's restart-on-crash: dropped; FAILED is terminal and the
  drone is already safed down when the state flips.

Session: S4 (implemented).
"""
from __future__ import annotations

import asyncio
import dataclasses
import enum
import math
import sys
import time
from typing import Callable, List, Optional, Sequence

from finals.errors import FlightError, FlightTimeout, SensorTimeout
from finals.events import EventLog
from finals.flight.adapter import FlightAdapter
from finals.guards import (Guard, GuardContext, SafetyController, Trip,
                           TripAction, evaluate_guards)
from finals.mission.phase import AgentContext, MissionPhase
from finals.sightings import SightingBus
from finals.types import (Abort, Action, Done, Hover, Land, Move, Rotate,
                          Takeoff, Telemetry, Wait)


#: The telemetry-staleness MECHANISM backstop (SensorTimeout -> emergency
#: land + FAILED). The S5 TelemetryWatchdog POLICY layer must stay STRICTLY
#: tighter than this or the layering inverts — every stale-telemetry event
#: would become an emergency instead of an orderly guard landing. Config
#: validation enforces guards.telemetry_stale_s < this (finals/config.py).
DEFAULT_TELEMETRY_STALE_S = 5.0


class AgentState(enum.Enum):
    INIT = "INIT"          # constructed, run() not yet called
    READY = "READY"        # connected, mission not yet started
    RUNNING = "RUNNING"    # stepping phases
    LANDING = "LANDING"    # final descent after the phase queue drained
    DONE = "DONE"          # clean end (includes a clean budget/operator stop)
    FAILED = "FAILED"      # terminal; drone safed down; never restarted


class DroneAgent:
    """One drone's runtime. Single-shot: one instance flies one mission."""

    def __init__(self, drone_id: str, adapter: FlightAdapter,
                 phases: List[MissionPhase], events: EventLog, *,
                 bus: Optional[SightingBus] = None,
                 command_timeout_s: float = 15.0,
                 command_grace_s: float = 2.0,
                 telemetry_stale_s: float = DEFAULT_TELEMETRY_STALE_S,
                 guards: Sequence[Guard] = (),
                 safety: Optional[SafetyController] = None,
                 hold_poll_s: float = 0.5,
                 clock: Callable[[], float] = time.monotonic):
        if not isinstance(drone_id, str) or not drone_id:
            raise ValueError(
                f"DroneAgent: drone_id must be a non-empty str, got "
                f"{drone_id!r} — check the wiring")
        if not isinstance(adapter, FlightAdapter):
            raise ValueError(
                f"DroneAgent({drone_id!r}): adapter must be a FlightAdapter "
                f"instance, got {type(adapter).__name__!r} — check the "
                f"wiring (main.py builds adapters per drone)")
        if (not isinstance(phases, list) or not phases
                or not all(isinstance(p, MissionPhase) for p in phases)):
            raise ValueError(
                f"DroneAgent({drone_id!r}): phases must be a non-empty list "
                f"of MissionPhase instances, got {phases!r} — check the "
                f"phase construction in the wiring")
        for name, value, zero_ok in (("command_timeout_s", command_timeout_s, False),
                                     ("command_grace_s", command_grace_s, True),
                                     ("telemetry_stale_s", telemetry_stale_s, False),
                                     ("hold_poll_s", hold_poll_s, False)):
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value)
                    or value < 0 or (value == 0 and not zero_ok)):
                raise ValueError(
                    f"DroneAgent({drone_id!r}): {name} must be finite and "
                    f"{'>= 0' if zero_ok else '> 0'}, got {value!r} — an "
                    f"unbounded deadline would defeat the whole watchdog")
        if not all(isinstance(g, Guard) for g in guards):
            raise ValueError(
                f"DroneAgent({drone_id!r}): guards must be Guard instances, "
                f"got {guards!r} — check the main._build_guards wiring")
        if safety is not None and not isinstance(safety, SafetyController):
            raise ValueError(
                f"DroneAgent({drone_id!r}): safety must be a SafetyController "
                f"or None, got {type(safety).__name__!r} — check the main.py "
                f"wiring")

        self.drone_id = drone_id
        self._adapter = adapter
        self._phases = list(phases)
        self._events = events
        self._bus = bus
        self._command_timeout_s = float(command_timeout_s)
        self._command_grace_s = float(command_grace_s)
        self._telemetry_stale_s = float(telemetry_stale_s)
        self._guards = list(guards)
        self._safety = safety
        self._hold_poll_s = float(hold_poll_s)
        self._clock = clock

        self._state = AgentState.INIT
        self._phase_idx = 0
        self._phases_completed = 0
        self._phase_entered_at: Optional[float] = None
        self._pending_trip: Optional[Trip] = None
        self._airborne = False
        self._emergency_landed = False     # the EXACTLY-ONCE latch
        self._disconnected = False
        self._failure: Optional[str] = None
        self._stopped_reason: Optional[str] = None
        self._bus_cursor = 0
        self._t_start: Optional[float] = None
        self._last_telemetry: Optional[Telemetry] = None
        self._last_action: Optional[Action] = None
        self._last_action_ok: Optional[bool] = None
        self._last_action_error: Optional[str] = None

    # ---------------- read-only introspection ----------------
    @property
    def state(self) -> AgentState:
        return self._state

    @property
    def failure(self) -> Optional[str]:
        return self._failure

    @property
    def phases_completed(self) -> int:
        return self._phases_completed

    def status(self) -> dict:
        """Non-blocking heartbeat snapshot. Uses CACHED telemetry so it stays
        readable after the agent dies (heartbeat must survive agent death)."""
        t = self._last_telemetry
        phase = (self._phases[self._phase_idx].name
                 if self._phase_idx < len(self._phases) else None)
        return {
            "state": self._state.name,
            "phase": phase,
            "phase_idx": self._phase_idx,
            "n_phases": len(self._phases),
            "phases_completed": self._phases_completed,
            "airborne": self._airborne,
            "last_action": (type(self._last_action).__name__
                            if self._last_action is not None else None),
            "last_action_ok": self._last_action_ok,
            "last_action_error": self._last_action_error,
            "battery_pct": t.battery_pct if t is not None else None,
            "telemetry_age_s": (round(t.age_s(self._clock()), 3)
                                if t is not None else None),
            "emergency_landed": self._emergency_landed,
            "failure": self._failure,
            "stopped_reason": self._stopped_reason,
        }

    # ---------------- logging helpers ----------------
    def _log(self, event: str, **data) -> None:
        self._events.log(self.drone_id, event, **data)

    @staticmethod
    def _action_fields(action: Action) -> dict:
        """Action dataclass -> JSON-friendly dict (enums by NAME so the
        replay reader and a 2 a.m. grep see 'FORWARD', not 0)."""
        return {k: (v.name if isinstance(v, enum.Enum) else v)
                for k, v in dataclasses.asdict(action).items()}

    def _outer_s(self, extra_s: float = 0.0) -> float:
        return self._command_timeout_s + self._command_grace_s + extra_s

    # ---------------- the agent's life ----------------
    async def run(self, *, deadline: float,
                  stop_event: asyncio.Event) -> None:
        """Connect -> phases -> land -> DONE; any flight failure -> safe-down
        -> FAILED. Returns normally in both cases; only non-flight bugs
        (phase exceptions, log-write failures) propagate, for the
        orchestrator's whitelisted net + fail_safe()."""
        if self._state is not AgentState.INIT:
            raise RuntimeError(
                f"DroneAgent({self.drone_id!r}).run() called twice — agents "
                f"are single-shot (state {self._state.name}); build a fresh "
                f"agent per mission")
        self._t_start = self._clock()

        # -- connect --
        try:
            self._log("agent_connect", timeout_s=self._command_timeout_s)
            await asyncio.wait_for(
                self._adapter.connect(timeout_s=self._command_timeout_s),
                self._outer_s())
        except FlightError as e:
            await self._fail(f"connect failed: {e}", exc=e)
            return
        except asyncio.TimeoutError as e:
            await self._fail(
                f"connect() still running after the outer deadline "
                f"{self._outer_s():.1f} s (its own timeout_s="
                f"{self._command_timeout_s:.1f} never fired — backend hang) "
                f"— check the backend/link", exc=e)
            return
        self._state = AgentState.READY

        # -- origin event: the replay-plot prereq (simulation.md Tier 0) --
        try:
            t0 = self._adapter.telemetry()
        except FlightError as e:
            await self._fail(
                f"telemetry unavailable immediately after connect: {e}",
                exc=e)
            return
        self._last_telemetry = t0
        self._log(
            "origin",
            position_m=t0.position_m, altitude_m=t0.altitude_m,
            yaw_deg=t0.yaw_deg, position_quality=t0.position_quality.name,
            is_flying=t0.is_flying,
            frame="north/east/up, yaw CCW+ from boot heading — see "
                  "finals/flight/dead_reckon.py (binding)")

        # -- phase loop. Bounds (convention 3): finite phase queue + mission
        # deadline + stop event, re-checked every iteration. --
        self._state = AgentState.RUNNING
        entered = False
        while self._phase_idx < len(self._phases):
            now = self._clock()
            if stop_event.is_set():
                self._stopped_reason = "stop requested (budget/operator)"
                break
            if now >= deadline:
                self._stopped_reason = (
                    f"mission deadline reached "
                    f"({now - self._t_start:.1f} s elapsed)")
                break

            try:
                ctx = self._build_ctx(now)
            except (FlightError, SensorTimeout) as e:
                await self._fail(str(e), exc=e)
                return

            # -- per-drone guards (S5): evaluated BEFORE the phase may act.
            # The wrapper converts a raising guard into a trip; nothing to
            # catch here. Acted on by MAX severity; every trip is logged. --
            if self._guards:
                trips = evaluate_guards(self._guards, GuardContext(
                    drone_id=self.drone_id, now=now,
                    mission_elapsed_s=now - self._t_start,
                    telemetry=ctx.telemetry,
                    phase_name=self._phases[self._phase_idx].name,
                    phase_elapsed_s=(now - self._phase_entered_at
                                     if self._phase_entered_at is not None
                                     else None)))
                if trips:
                    worst = max(trips, key=lambda tr: tr.action)
                    for tr in trips:
                        self._log("guard_trip", guard=tr.guard,
                                  action=tr.action.name, reason=tr.reason)
                    if worst.action is TripAction.LAND_ALL:
                        self._pending_trip = worst
                        self._stopped_reason = (f"guard {worst.guard} "
                                                f"tripped LAND_ALL: "
                                                f"{worst.reason}")
                        stop_event.set()    # every agent winds down clean
                        break
                    if worst.action is TripAction.LAND_THIS:
                        self._pending_trip = worst
                        self._stopped_reason = (f"guard {worst.guard}: "
                                                f"{worst.reason}")
                        break
                    if worst.action is TripAction.HOLD_THIS:
                        # Bounded idle; the phase is NOT stepped this tick.
                        await self._wait(Wait(duration_s=self._hold_poll_s),
                                         deadline, stop_event)
                        continue
                    # ADVISORY / DEGRADE_DETECTION: logged above; the
                    # detection-shedding consumers arrive with S7.

            phase = self._phases[self._phase_idx]
            if not entered:
                self._log("phase_enter", phase=phase.name,
                          index=self._phase_idx)
                phase.on_enter(ctx)
                entered = True
                self._phase_entered_at = now

            action = phase.step(ctx)

            if isinstance(action, Done):
                phase.on_exit(ctx)
                self._log("phase_done", phase=phase.name,
                          reason=action.reason)
                self._phases_completed += 1
                self._phase_idx += 1
                entered = False
                self._phase_entered_at = None
                continue
            if isinstance(action, Abort):
                self._log("phase_abort", phase=phase.name,
                          reason=action.reason)
                await self._fail(
                    f"phase {phase.name} aborted: {action.reason}")
                return
            if isinstance(action, Wait):
                await self._wait(action, deadline, stop_event)
                self._note(action, ok=True)
                continue
            if isinstance(action, (Takeoff, Move, Rotate, Hover, Land)):
                try:
                    await self._execute(action)
                except FlightError as e:
                    self._note(action, ok=False, error=str(e))
                    await self._fail(
                        f"{type(action).__name__} failed: {e}", exc=e)
                    return
                self._note(action, ok=True)
                continue
            raise TypeError(
                f"{self.drone_id}: phase {phase.name!r}.step() returned "
                f"{action!r} — not in the finals.types Action vocabulary "
                f"(phase bug; see finals/mission/phase.py)")

        if self._stopped_reason is not None:
            self._log("agent_stopped", reason=self._stopped_reason,
                      phase_idx=self._phase_idx)

        # -- final descent (skip if a phase already landed us) --
        self._state = AgentState.LANDING
        self._log("agent_landing", airborne=self._airborne)
        if self._airborne:
            land = Land()
            try:
                await self._execute(land)
            except FlightError as e:
                self._note(land, ok=False, error=str(e))
                await self._fail(f"final land failed: {e}", exc=e)
                return
            self._note(land, ok=True)

        self._state = AgentState.DONE
        self._log("agent_done", phases_completed=self._phases_completed,
                  stopped_reason=self._stopped_reason)

    # ---------------- per-tick context ----------------
    def _build_ctx(self, now: float) -> AgentContext:
        telemetry = self._adapter.telemetry()      # FlightError -> caller
        self._last_telemetry = telemetry
        age_s = telemetry.age_s(now)
        if age_s > self._telemetry_stale_s:
            raise SensorTimeout(
                f"{self.drone_id}: telemetry is STALE — age {age_s:.1f} s "
                f"exceeds the {self._telemetry_stale_s:.1f} s trust limit "
                f"(last stamp ts={telemetry.ts:.1f}) — telemetry poller/"
                f"link dead? check Wi-Fi / the backend poller thread; "
                f"refusing to act on stale state, safing down")
        sightings = []
        if self._bus is not None:
            self._bus_cursor, sightings = self._bus.drain_after(
                self._bus_cursor, drone_id=self.drone_id)
        assert self._t_start is not None           # set first thing in run()
        return AgentContext(
            drone_id=self.drone_id,
            now=now,
            mission_elapsed_s=now - self._t_start,
            telemetry=telemetry,
            sightings=sightings,
            last_action=self._last_action,
            last_action_ok=self._last_action_ok,
            last_action_error=self._last_action_error,
        )

    def _note(self, action: Action, *, ok: bool,
              error: Optional[str] = None) -> None:
        self._last_action = action
        self._last_action_ok = ok
        self._last_action_error = error

    # ---------------- command execution ----------------
    async def _execute(self, action: Action) -> None:
        """One flight command under the double deadline (adapter timeout_s +
        outer wait_for). Raises typed FlightError/FlightTimeout only."""
        a = self._adapter
        t = self._command_timeout_s
        name = type(action).__name__
        fields = self._action_fields(action)
        # Hover blocks duration_s by design on real backends; its outer
        # deadline gets that long PLUS the usual command allowance. A
        # non-finite duration would poison the wait_for timeout — leave it
        # out; the adapter's argument gate refuses the action itself.
        extra = (action.duration_s
                 if isinstance(action, Hover) and math.isfinite(action.duration_s)
                 and action.duration_s > 0 else 0.0)
        outer = self._outer_s(extra)
        route_fields = {}
        if isinstance(action, Land) and self._safety is not None:
            # The safety landing path carries its own (larger) total bound —
            # log THAT so the forensic record cites the deadline that binds.
            outer = self._safety.land_bound_s
            route_fields["route"] = "safety"

        self._log("action_start", action=name, timeout_s=t,
                  outer_deadline_s=outer, **route_fields, **fields)
        t0 = self._clock()
        try:
            if isinstance(action, Takeoff):
                await asyncio.wait_for(
                    a.takeoff(height_cm=action.height_cm, timeout_s=t), outer)
            elif isinstance(action, Move):
                await asyncio.wait_for(
                    a.move(action.direction, action.distance_cm, timeout_s=t),
                    outer)
            elif isinstance(action, Rotate):
                await asyncio.wait_for(
                    a.rotate(action.angle_deg, timeout_s=t), outer)
            elif isinstance(action, Hover):
                await asyncio.wait_for(a.hover(action.duration_s), outer)
            elif isinstance(action, Land):
                if self._safety is not None:
                    # safety.land/trip are internally bounded (slot wait +
                    # per-attempt deadlines + a fixed attempt count); the
                    # standard outer wait_for would kill the retry ladder
                    # mid-attempt and mislabel it a backend hang. A guard-
                    # tripped descent goes through the latched trip() entry
                    # (idempotent across racing trip sources).
                    if self._pending_trip is not None:
                        await self._safety.trip(a, self.drone_id,
                                                self._pending_trip.reason)
                    else:
                        await self._safety.land(a, self.drone_id)
                else:
                    await asyncio.wait_for(a.land(timeout_s=t), outer)
            else:   # _execute is only called with the five flight actions
                raise TypeError(
                    f"{self.drone_id}: _execute() got non-flight action "
                    f"{action!r} — agent dispatch bug")
        except asyncio.TimeoutError as e:
            # The belt-and-suspenders watchdog: the adapter's OWN deadline
            # never fired (hung executor thread / blocking SDK call — the
            # mapping_drone.py wait-forever class).
            err = FlightTimeout(
                f"{self.drone_id}: {name}{fields} still running after the "
                f"agent's outer deadline {outer:.1f} s — the adapter's own "
                f"timeout_s={t:.1f} never fired (backend hang / blocked "
                f"executor) — check the backend link and its executor "
                f"thread; safing down")
            self._log("action_failed", action=name,
                      elapsed_s=self._clock() - t0, error=str(err),
                      error_type="FlightTimeout", **fields)
            raise err from e
        except FlightError as e:
            self._log("action_failed", action=name,
                      elapsed_s=self._clock() - t0, error=str(e),
                      error_type=type(e).__name__, **fields)
            raise

        if isinstance(action, Takeoff):
            self._airborne = True
        elif isinstance(action, Land):
            self._airborne = False
        self._log("action_complete", action=name,
                  elapsed_s=self._clock() - t0, **fields)

    async def _wait(self, action: Wait, deadline: float,
                    stop_event: asyncio.Event) -> None:
        """Bounded idle: never sleeps past the mission deadline and wakes
        early when the stop event fires."""
        if not isinstance(action.duration_s, (int, float)) \
                or not math.isfinite(action.duration_s) or action.duration_s < 0:
            raise ValueError(
                f"{self.drone_id}: phase returned Wait({action.duration_s!r}) "
                f"— duration_s must be finite and >= 0 (phase bug)")
        remaining = max(0.0, min(float(action.duration_s),
                                 deadline - self._clock()))
        try:
            await asyncio.wait_for(stop_event.wait(), timeout=remaining)
        except asyncio.TimeoutError:
            pass    # the wait simply elapsed — the normal case

    # ---------------- failure / teardown ----------------
    async def _fail(self, reason: str, exc: Optional[BaseException] = None) -> None:
        """FAILED transition + the exactly-once emergency_land. The safe-down
        is commanded BEFORE anything is logged: safety beats forensics (a
        log-write failure must not block the landing)."""
        first = self._state is not AgentState.FAILED
        self._state = AgentState.FAILED
        if self._failure is None:
            self._failure = reason

        if not self._emergency_landed:
            self._emergency_landed = True
            hung = False
            try:
                # shield: if THIS task gets cancelled mid-safe-down (e.g.
                # the orchestrator's settle-deadline force-down racing this
                # failure), the emergency_land command itself must keep
                # running to completion — cancelling the safe-down is the
                # one cancellation that may not happen. Still bounded by
                # the wait_for inside the shield.
                await asyncio.shield(asyncio.wait_for(
                    self._adapter.emergency_land(), self._outer_s()))
            except asyncio.TimeoutError:
                hung = True
                print(
                    f"[DroneAgent] CRITICAL {self.drone_id}: emergency_land "
                    f"did not return within {self._outer_s():.1f} s — the "
                    f"drone may STILL BE AIRBORNE — check the link and be "
                    f"ready for a manual kill", file=sys.stderr, flush=True)
            if not hung:
                self._airborne = False
            self._log("emergency_land", context="failure", hung=hung)

        if first:
            self._log("agent_failed", reason=reason,
                      error_type=type(exc).__name__ if exc is not None else None)

    async def fail_safe(self, reason: str) -> None:
        """Orchestrator entry point for an agent whose task died/was
        cancelled OUTSIDE the typed failure paths. No-op when the agent
        already ended cleanly; otherwise the standard latched fail path."""
        if self._state is AgentState.DONE:
            return
        await self._fail(reason)

    async def shutdown(self) -> None:
        """Land-if-still-airborne (abnormal; same latch) -> disconnect.
        Idempotent. Both awaits bounded; a hang is logged, never raised."""
        if self._airborne and not self._emergency_landed:
            self._emergency_landed = True
            hung = False
            try:
                # shield: same argument as in _fail — never cancel a
                # safe-down that is already in flight.
                await asyncio.shield(asyncio.wait_for(
                    self._adapter.emergency_land(), self._outer_s()))
            except asyncio.TimeoutError:
                hung = True
                print(
                    f"[DroneAgent] CRITICAL {self.drone_id}: emergency_land "
                    f"during shutdown did not return within "
                    f"{self._outer_s():.1f} s — the drone may STILL BE "
                    f"AIRBORNE", file=sys.stderr, flush=True)
            if not hung:
                self._airborne = False
            self._log("emergency_land", context="shutdown", hung=hung)

        if not self._disconnected:
            self._disconnected = True
            try:
                await asyncio.wait_for(self._adapter.disconnect(),
                                       self._outer_s())
                self._log("agent_disconnect")
            except asyncio.TimeoutError:
                self._log("agent_disconnect_hung",
                          outer_deadline_s=self._outer_s(),
                          check="disconnect() never returned — backend hang; "
                                "the link is abandoned, not released")
