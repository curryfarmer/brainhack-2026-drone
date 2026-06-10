"""Orchestrator — supervises N DroneAgents as independent asyncio tasks.

Surface (S4, implemented):
- run() -> int exit code (0 = every agent ended DONE, 1 = any FAILED or the
  supervision had to force the issue). One task per agent (agent.run drives
  connect -> phases -> land); the supervision loop beats at heartbeat_period_s
  (1 Hz default) and per tick:
    * drains the SightingBus with a SEQ cursor (drain_after — NEVER a ts
      cursor, see finals/sightings.py) and logs each sighting as an event
      under its drone's id;
    * rewrites runs dir heartbeat.json atomically via events.write_heartbeat
      (per-drone agent.status(): state, phase, last cmd, battery, telemetry
      age — plus tick latency and budget telemetry). write_heartbeat is a
      small synchronous fsync on the loop; documented trade-off at 1 Hz.
- Mission budget: at budget_s the stop event is set — agents finish their
  in-flight command, land, and report DONE (a budget stop is a CLEAN end).
  budget_s + settle_grace_s is the HARD deadline: past it, still-pending
  agent tasks are cancelled, fail_safe()d (their latch guarantees the single
  emergency_land), and the run is reported failed.
- One agent's death NEVER stops the others: tasks are independent; outcomes
  are reconciled at the end (task.cancelled()/task.exception() — an agent
  task that died outside its own typed handling is logged WITH traceback and
  fail_safe()d). This file is one of the two whitelisted `except Exception`
  sites (tests/test_conventions.py): the top-loop net around the tick body
  and the per-agent reconciliation/shutdown nets — every catch logs the
  traceback, none is silent.
- Clean shutdown (always, via finally): cancel leftovers -> reconcile
  outcomes -> per-agent shutdown() (land-if-airborne + disconnect) -> final
  bus drain -> final heartbeat -> loud parseable summary + run_end event.
- Operator kill: KeyboardInterrupt / CancelledError (what Ctrl+C under
  asyncio.run delivers) sets stop, cancels all agent tasks, fail_safe()s
  every agent (emergency land, latched), then re-raises after the finally
  cleanup.
- Operator ABORT KEY (S5): the AbortListener 'q' channel is ORDERLY, not
  the Ctrl+C path — the tick polls abort_event (a threading.Event) and on
  the first observation sets stop + logs operator_abort; agents land clean
  and end DONE. request_stop_threadsafe() is the listener's prompt-wakeup
  hook (loop.call_soon_threadsafe -> stop.set) so agents wake mid-Wait
  instead of at the next beat; the poll stays the reliable channel.
- Swarm-level guards (S5): evaluated each tick through
  finals.guards.evaluate_guards with error_action=LAND_ALL (a buggy
  mission-level guard cannot be trusted to stay quiet) under the "mission"
  pseudo id (MissionClockGuard; LoopOverrunGuard consuming the BEAT-TO-BEAT
  gap — the number a starved loop stretches; the drain-only duration in the
  heartbeat cannot measure starvation, it has no awaits).
  Every trip is logged (guard_trip); LAND_THIS-or-worse sets stop — at
  mission level there is no single drone to act on, so any land-grade trip
  means land ALL via the same stop machinery the budget uses. An AGENT
  setting the shared stop (a per-drone LAND_ALL trip) is noticed and
  logged once as stop_signalled.
- NO auto-restart in flight profiles: a crash-restart that re-arms 3 real
  aircraft is unsafe. Per-drone FAILED is terminal; the others continue.

Mission-level events are logged under the pseudo drone id "mission"
(run_start, budget_expired, guard_trip, operator_abort, stop_signalled,
tick_error, run_end, ...) — agents therefore must not use that id
(enforced at construction).

Derives from: qualifier_run.py:407-513 supervisor — kept: the wall-clock
budget computed ONCE up front, long-lived singletons owned outside the
per-attempt scope, traceback printing on every swallowed exception, the
final score/summary block. Dropped/fixed in adaptation:
- the restart-on-crash loop (unsafe re-arm on real aircraft — the deliberate
  departure; FAILED drones stay down);
- its bare-Exception swallows around receiver/detector init that continued
  half-wired (here every component either works or the affected agent fails
  loudly);
- the unbounded `await mission_task` (here every wait carries a timeout and
  the supervision loop is bounded by the hard deadline — convention 3).

Session: S4 (implemented).
"""
from __future__ import annotations

import asyncio
import math
import sys
import threading
import time
import traceback
from typing import TYPE_CHECKING, Callable, Dict, List, Optional, Sequence

from finals.events import EventLog, EventLogError, write_heartbeat
from finals.guards import Guard, GuardContext, TripAction, evaluate_guards
from finals.mission.agent import AgentState, DroneAgent
from finals.sightings import SightingBus

if TYPE_CHECKING:  # type-only: the registry/map are duck-typed (expire/snapshot)
    from finals.mission.convoy_registry import ConvoyRegistry
    from finals.mission.pad_validity import PadValidityMap

#: Bounded wait for cancelled agent tasks to actually return (a cancelled
#: agent unwinds through bounded awaits, so this is generous headroom).
_REAP_TIMEOUT_S = 30.0
#: Pseudo drone id for mission-level events.
_MISSION_ID = "mission"


class Orchestrator:
    """Owns the supervision loop; the agents own their drones."""

    def __init__(self, agents: List[DroneAgent], events: EventLog,
                 run_dir: str, *,
                 budget_s: float,
                 bus: Optional[SightingBus] = None,
                 heartbeat_period_s: float = 1.0,
                 settle_grace_s: float = 60.0,
                 swarm_guards: Sequence[Guard] = (),
                 abort_event: Optional[threading.Event] = None,
                 convoy_registry: Optional["ConvoyRegistry"] = None,
                 validity_map: Optional["PadValidityMap"] = None,
                 drone_sectors: Optional[Dict[str, Sequence[float]]] = None,
                 convoy_ids: Optional[Sequence[int]] = None,
                 coverage_weak_below: int = 3,
                 clock: Callable[[], float] = time.monotonic):
        if not isinstance(agents, list) or not agents \
                or not all(isinstance(a, DroneAgent) for a in agents):
            raise ValueError(
                f"Orchestrator: agents must be a non-empty list of "
                f"DroneAgent, got {agents!r} — check the main.py wiring")
        ids = [a.drone_id for a in agents]
        if len(ids) != len(set(ids)):
            raise ValueError(
                f"Orchestrator: duplicate drone ids {ids} — each agent owns "
                f"ONE drone; check the config/wiring")
        if _MISSION_ID in ids:
            raise ValueError(
                f"Orchestrator: drone id {_MISSION_ID!r} is reserved for "
                f"mission-level events — rename the drone in the config")
        for name, value, zero_ok in (("budget_s", budget_s, False),
                                     ("heartbeat_period_s", heartbeat_period_s, False),
                                     ("settle_grace_s", settle_grace_s, True)):
            if (not isinstance(value, (int, float)) or isinstance(value, bool)
                    or not math.isfinite(value)
                    or value < 0 or (value == 0 and not zero_ok)):
                raise ValueError(
                    f"Orchestrator: {name} must be finite and "
                    f"{'>= 0' if zero_ok else '> 0'}, got {value!r} — an "
                    f"unbounded supervision loop is the bug class this "
                    f"module exists to prevent")
        if not all(isinstance(g, Guard) for g in swarm_guards):
            raise ValueError(
                f"Orchestrator: swarm_guards must be Guard instances, got "
                f"{swarm_guards!r} — check the main.py wiring")
        if abort_event is not None and not isinstance(abort_event,
                                                      threading.Event):
            raise ValueError(
                f"Orchestrator: abort_event must be a threading.Event or "
                f"None, got {type(abort_event).__name__!r} — check the "
                f"main.py wiring")

        self._agents = list(agents)
        self._events = events
        self._run_dir = run_dir
        self._budget_s = float(budget_s)
        self._bus = bus
        self._heartbeat_period_s = float(heartbeat_period_s)
        self._settle_grace_s = float(settle_grace_s)
        self._swarm_guards = list(swarm_guards)
        self._abort_event = abort_event
        # WS-2 convoy coordination (None for non-convoy missions): expired each
        # beat so a drone that dropped Wi-Fi frees its convoy; snapshotted into
        # the heartbeat as the swarm-wide ownership view.
        self._registry = convoy_registry
        # PAD-VALID landing coordination (None for non-landing missions): the
        # shared pad-validity / claim store. Snapshot-ONLY here (no expire —
        # landing is terminal, a claimed pad is never freed); folded into the
        # heartbeat each beat as the swarm-wide pad-validity view.
        self._validity_map = validity_map
        # WS-7A soft-zone handover: per-drone assigned sector
        # {drone_id: (center_deg, half_width_deg)}, used by the per-tick matcher
        # to find which idle neighbour a flagged-exited convoy entered. Empty/
        # None = no soft-zoning (the matcher is a no-op). Validated loud: a
        # sector for an unknown drone or a malformed wedge is a wiring bug.
        self._drone_sectors = self._validate_sectors(drone_sectors, ids)
        # Live convoy-coverage read-back (the operator's "did we actually catch
        # all 5?"): the KNOWN target id set (the 5-of-5 denominator), kept
        # independent of the registry so a registry-less coverage sweep still
        # reports — plus a running per-marker-id sighting tally drained from the
        # bus. Empty known set => no coverage block at all (non-convoy missions
        # are byte-for-byte unchanged). SEEN (any decode) and SERVICED (a drone
        # locked + confirmed + released, from the registry) are reported
        # SEPARATELY: that is exactly the "glimpsed vs confirmed" distinction.
        self._known_convoy_ids = self._validate_convoy_ids(convoy_ids)
        if (not isinstance(coverage_weak_below, int)
                or isinstance(coverage_weak_below, bool)
                or coverage_weak_below < 0):
            raise ValueError(
                f"Orchestrator: coverage_weak_below must be an int >= 0, got "
                f"{coverage_weak_below!r} — it is the read-count under which a "
                f"seen convoy id is flagged WEAK in the summary")
        self._coverage_weak_below = coverage_weak_below
        self._clock = clock

        # Per-run counters (reset by run(); instance state, never module
        # globals — convention 4).
        self._tick = 0
        self._cursor = 0
        self._n_sightings = 0
        #: Per-marker-id read tally accumulated in _drain_bus (the coverage
        #: read-back). Keyed by marker_id; convoy ids AND any stray decodes.
        self._id_counts: Dict[int, int] = {}
        #: True once the all-convoys-serviced early-stop has been signalled, so
        #: it logs + sets stop exactly once.
        self._coverage_stop_logged = False
        self._last_tick_latency_s = 0.0
        #: Beat-to-beat gap (None until the second beat) — what a starved/
        #: blocked event loop stretches; feeds LoopOverrunGuard. The drain
        #: duration above CANNOT measure starvation (it has no awaits).
        self._last_beat_gap_s: Optional[float] = None
        self._t_start: Optional[float] = None
        self._abort_handled = False
        self._stop_seen = False
        # request_stop_threadsafe plumbing — set by run(), used cross-thread.
        self._loop_ref: Optional[asyncio.AbstractEventLoop] = None
        self._stop_ref: Optional[asyncio.Event] = None

    @staticmethod
    def _validate_sectors(drone_sectors: Optional[Dict[str, Sequence[float]]],
                          ids: Sequence[str]) -> Dict[str, tuple]:
        """Validate the optional per-drone sector map and normalize to
        {drone_id: (center_deg, half_width_deg)}. None/empty -> {} (no
        soft-zoning). A sector for an unknown drone, a wrong-length entry, or a
        non-finite / negative-half-width value is a wiring bug -> ValueError
        (loud, like every other constructor guard here)."""
        if not drone_sectors:
            return {}
        if not isinstance(drone_sectors, dict):
            raise ValueError(
                f"Orchestrator: drone_sectors must be a dict "
                f"{{drone_id: [center_deg, half_width_deg]}} or None, got "
                f"{type(drone_sectors).__name__!r} — check the main.py wiring")
        known = set(ids)
        out: Dict[str, tuple] = {}
        for did, sec in drone_sectors.items():
            if did not in known:
                raise ValueError(
                    f"Orchestrator: drone_sectors has an entry for {did!r} which "
                    f"is not one of the agents {sorted(known)} — check the "
                    f"main.py sector wiring")
            if (not isinstance(sec, (list, tuple)) or len(sec) != 2
                    or any(not isinstance(v, (int, float))
                           or isinstance(v, bool) or not math.isfinite(v)
                           for v in sec)):
                raise ValueError(
                    f"Orchestrator: drone_sectors[{did!r}]={sec!r} must be "
                    f"[center_deg, half_width_deg] (two finite numbers, deg) — "
                    f"check the config sector_deg")
            if sec[1] < 0:
                raise ValueError(
                    f"Orchestrator: drone_sectors[{did!r}] half_width_deg="
                    f"{sec[1]!r} must be >= 0 — a negative wedge matches nothing")
            out[did] = (float(sec[0]), float(sec[1]))
        return out

    # ---------------- logging helpers (never let forensics kill flight) ----
    def _log_mission(self, event: str, **data) -> None:
        self._events.log(_MISSION_ID, event, **data)

    def _try_log(self, drone_id: str, event: str, **data) -> None:
        try:
            self._events.log(drone_id, event, **data)
        except EventLogError as e:
            print(f"[Orchestrator] WARNING: could not log {event!r}: {e}",
                  file=sys.stderr, flush=True)

    def _net(self, where: str) -> None:
        """The whitelisted blanket-catch handler: ALWAYS a full traceback to
        stderr plus a best-effort event. Call from inside a handler only."""
        tb = traceback.format_exc()
        print(f"[Orchestrator] ERROR in {where} — supervision continues:\n{tb}",
              file=sys.stderr, flush=True)
        self._try_log(_MISSION_ID, "tick_error", where=where, traceback=tb)

    # ---------------- operator abort (thread-side hook) ----------------
    def request_stop_threadsafe(self) -> None:
        """Prompt-stop entry for OTHER THREADS (the AbortListener's wakeup
        hook): schedules stop.set() on the orchestrator's loop so agents
        wake mid-Wait instead of at the next 1 Hz poll. Best-effort — the
        per-tick abort_event poll is the reliable channel; before run() or
        after the loop closed this just notes it and returns."""
        loop, stop = self._loop_ref, self._stop_ref
        if loop is None or stop is None:
            print("[Orchestrator] abort wakeup before run() — the abort "
                  "event poll catches it at start", file=sys.stderr,
                  flush=True)
            return
        try:
            loop.call_soon_threadsafe(stop.set)
        except RuntimeError as e:        # loop already closed: mission over
            print(f"[Orchestrator] abort wakeup skipped ({e}) — loop "
                  f"closed; nothing airborne to stop", file=sys.stderr,
                  flush=True)

    # ---------------- the supervision loop ----------------
    async def run(self) -> int:
        self._tick = 0
        self._cursor = 0
        self._n_sightings = 0
        self._id_counts = {}
        self._coverage_stop_logged = False
        self._last_tick_latency_s = 0.0
        self._last_beat_gap_s = None
        self._abort_handled = False
        self._stop_seen = False
        self._t_start = self._clock()
        deadline = self._t_start + self._budget_s
        hard_deadline = deadline + self._settle_grace_s
        stop = asyncio.Event()
        self._stop_ref = stop
        self._loop_ref = asyncio.get_running_loop()

        self._log_mission(
            "run_start", drones=[a.drone_id for a in self._agents],
            budget_s=self._budget_s, settle_grace_s=self._settle_grace_s,
            heartbeat_period_s=self._heartbeat_period_s,
            run_dir=self._run_dir)

        tasks: Dict[str, asyncio.Task] = {
            a.drone_id: asyncio.get_running_loop().create_task(
                a.run(deadline=deadline, stop_event=stop),
                name=f"agent:{a.drone_id}")
            for a in self._agents}

        try:
            # Bounds (convention 3): all tasks done OR the hard deadline.
            prev_beat: Optional[float] = None
            while True:
                pending = [t for t in tasks.values() if not t.done()]
                if not pending:
                    break
                now = self._clock()
                # Beat-to-beat gap: the supervision-health number a starved
                # loop stretches (LoopOverrunGuard input). Healthy ~= the
                # heartbeat period, since the wait below times out at it.
                self._last_beat_gap_s = (None if prev_beat is None
                                         else now - prev_beat)
                prev_beat = now
                if now >= hard_deadline:
                    names = [n for n, t in tasks.items() if not t.done()]
                    print(
                        f"[Orchestrator] ERROR: agents {names} still not "
                        f"settled {now - deadline:.1f} s past the mission "
                        f"budget (settle grace {self._settle_grace_s:.1f} s "
                        f"exhausted) — cancelling their tasks and forcing "
                        f"safe-down; check what the agents were stuck on",
                        file=sys.stderr, flush=True)
                    self._try_log(_MISSION_ID, "settle_deadline_exceeded",
                                  still_pending=names,
                                  grace_s=self._settle_grace_s)
                    for t in pending:
                        t.cancel()
                    break       # reconciliation below force-lands them
                # -- operator abort key (S5): orderly land-all, NOT the
                # Ctrl+C cancel path. First observation wins attribution. --
                if (self._abort_event is not None
                        and self._abort_event.is_set()
                        and not self._abort_handled):
                    self._abort_handled = True
                    self._stop_seen = True
                    print("[Orchestrator] OPERATOR ABORT (abort key): "
                          "landing all drones cleanly",
                          file=sys.stderr, flush=True)
                    stop.set()
                    self._try_log(_MISSION_ID, "operator_abort",
                                  kind="abort_key")
                # -- swarm-level guards (S5): a raising guard is a LAND_ALL
                # trip (converted inside the wrapper, traceback logged). --
                if self._swarm_guards:
                    trips = evaluate_guards(
                        self._swarm_guards,
                        GuardContext(
                            drone_id=_MISSION_ID, now=now,
                            mission_elapsed_s=now - self._t_start,
                            tick_latency_s=self._last_beat_gap_s),
                        error_action=TripAction.LAND_ALL)
                    for tr in trips:
                        self._try_log(_MISSION_ID, "guard_trip",
                                      guard=tr.guard, action=tr.action.name,
                                      reason=tr.reason)
                    if trips and not stop.is_set():
                        worst = max(trips, key=lambda tr: tr.action)
                        if worst.action >= TripAction.LAND_THIS:
                            # Mission level has no single drone to act on:
                            # any land-grade trip means land ALL, clean.
                            self._stop_seen = True
                            print(f"[Orchestrator] guard {worst.guard} "
                                  f"tripped {worst.action.name}: landing "
                                  f"all drones cleanly — {worst.reason}",
                                  file=sys.stderr, flush=True)
                            stop.set()
                if stop.is_set() and not self._stop_seen:
                    # Someone ELSE set the shared stop — an agent's
                    # per-drone LAND_ALL trip (every orchestrator-side
                    # setter marks _stop_seen itself, so no deadline gate is
                    # needed and a trip landing AT the budget edge still
                    # gets attributed). The tripping agent's own guard_trip/
                    # agent_stopped events carry the detail.
                    self._stop_seen = True
                    self._try_log(_MISSION_ID, "stop_signalled",
                                  source="agent",
                                  elapsed_s=round(now - self._t_start, 3))
                if now >= deadline and not stop.is_set():
                    stop.set()
                    self._stop_seen = True
                    self._try_log(_MISSION_ID, "budget_expired",
                                  budget_s=self._budget_s,
                                  note="stop signalled; agents land and "
                                       "end DONE (clean)")
                self._tick += 1
                t0 = time.perf_counter()
                try:
                    if self._registry is not None:
                        lost = self._registry.expire(now)
                        if lost:
                            self._try_log(_MISSION_ID, "convoy_lock_expired",
                                          convoy_ids=lost,
                                          note="no heartbeat for lock_ttl_s — "
                                               "freed for re-claim")
                        # WS-7A: after expire (so a just-freed convoy is not
                        # offered), match flagged-exited convoys to idle
                        # neighbours. Wrapped by the same tick-body net below.
                        self._match_handovers(now)
                        # Read-and-release early-stop: once every KNOWN convoy
                        # is SERVICED (locked + confirmed + released by some
                        # drone), the mission objective is met — land all
                        # cleanly NOW instead of burning the rest of the budget.
                        # all_serviced() uses the registry's seeded known set
                        # (the 5-of-5 denominator); fires exactly once and only
                        # when there IS a denominator (empty known => False).
                        if (not self._coverage_stop_logged
                                and not stop.is_set()
                                and self._registry.all_serviced()):
                            self._coverage_stop_logged = True
                            self._stop_seen = True
                            stop.set()
                            self._try_log(
                                _MISSION_ID, "coverage_complete",
                                serviced=self._registry.serviced_ids(),
                                note="all known convoys serviced — landing all "
                                     "drones early (clean)")
                    self._drain_bus()
                    self._last_tick_latency_s = time.perf_counter() - t0
                    self._write_heartbeat(now, stop.is_set())
                except EventLogError as e:
                    # Forensics failed; flight continues. Loud, typed.
                    print(f"[Orchestrator] WARNING: tick {self._tick} "
                          f"heartbeat/event write failed: {e}",
                          file=sys.stderr, flush=True)
                except Exception:
                    # WHITELISTED top-loop net: a supervision bug must never
                    # take down flying drones. Always with traceback.
                    self._net(f"tick {self._tick}")
                await asyncio.wait(pending, timeout=self._heartbeat_period_s)
        except (KeyboardInterrupt, asyncio.CancelledError) as e:
            kind = type(e).__name__
            print(f"[Orchestrator] OPERATOR ABORT ({kind}): cancelling "
                  f"agents and emergency-landing everything",
                  file=sys.stderr, flush=True)
            self._stop_seen = True      # attributed here, not stop_signalled
            stop.set()
            for t in tasks.values():
                if not t.done():
                    t.cancel()
            self._try_log(_MISSION_ID, "operator_abort", kind=kind)
            raise       # finally below still lands/disconnects everyone
        finally:
            await self._settle_and_shutdown(tasks, stop)

        exit_code = 0 if all(a.state is AgentState.DONE
                             for a in self._agents) else 1
        self._try_log(_MISSION_ID, "run_end", exit_code=exit_code,
                      states={a.drone_id: a.state.name for a in self._agents},
                      sightings_drained=self._n_sightings)
        return exit_code

    # ---------------- shutdown / reconciliation ----------------
    async def _settle_and_shutdown(self, tasks: Dict[str, asyncio.Task],
                                   stop: asyncio.Event) -> None:
        # Captured BEFORE the unconditional set below: an abort key or an
        # agent-initiated LAND_ALL can fire and finish every task between
        # two beats, so the tick loop never observes it — attribute the
        # stop here instead, in the same priority order as the tick.
        externally_stopped = stop.is_set()
        stop.set()      # whatever path got us here, agents must wind down
        if (self._abort_event is not None and self._abort_event.is_set()
                and not self._abort_handled):
            self._abort_handled = True
            self._stop_seen = True
            self._try_log(_MISSION_ID, "operator_abort", kind="abort_key")
        if (externally_stopped and not self._stop_seen
                and self._t_start is not None):
            self._stop_seen = True
            self._try_log(_MISSION_ID, "stop_signalled", source="agent",
                          elapsed_s=round(self._clock() - self._t_start, 3))
        pending = [t for t in tasks.values() if not t.done()]
        for t in pending:
            t.cancel()
        if tasks:
            await asyncio.wait(set(tasks.values()), timeout=_REAP_TIMEOUT_S)

        agents_by_id = {a.drone_id: a for a in self._agents}
        for drone_id, task in tasks.items():
            agent = agents_by_id[drone_id]
            try:
                if not task.done():
                    print(f"[Orchestrator] ERROR: {drone_id}: agent task "
                          f"ignored cancellation for {_REAP_TIMEOUT_S:.0f} s "
                          f"— forcing safe-down anyway; check for an "
                          f"unbounded await in the backend",
                          file=sys.stderr, flush=True)
                    await agent.fail_safe(
                        "agent task unresponsive to cancellation")
                elif task.cancelled():
                    self._try_log(drone_id, "agent_task_cancelled")
                    await agent.fail_safe("agent task cancelled before "
                                          "settling")
                elif task.exception() is not None:
                    exc = task.exception()
                    tb = "".join(traceback.format_exception(
                        type(exc), exc, exc.__traceback__))
                    print(f"[Orchestrator] ERROR: {drone_id}: agent task "
                          f"crashed outside its typed handling — drone is "
                          f"being safed down:\n{tb}",
                          file=sys.stderr, flush=True)
                    self._try_log(drone_id, "agent_task_crashed",
                                  error=str(exc),
                                  error_type=type(exc).__name__,
                                  traceback=tb)
                    await agent.fail_safe(f"agent task crashed: {exc!r}")
            except Exception:
                # WHITELISTED: reconciliation of one agent must never block
                # the reconciliation/shutdown of the others.
                self._net(f"reconcile {drone_id}")

        for agent in self._agents:
            try:
                await agent.shutdown()
            except Exception:
                # WHITELISTED: same isolation argument as above.
                self._net(f"shutdown {agent.drone_id}")

        try:
            self._drain_bus()                       # nothing left behind
            self._write_heartbeat(self._clock(), stop.is_set(), final=True)
        except EventLogError as e:
            print(f"[Orchestrator] WARNING: final heartbeat/drain failed: "
                  f"{e}", file=sys.stderr, flush=True)
        except Exception:
            self._net("final heartbeat")

        self._print_summary()

    # ---------------- per-tick work ----------------
    def _match_handovers(self, now: float) -> None:
        """WS-7A soft-zone matcher — runs each beat beside expire(). For every
        convoy a drone flagged as having left its sector, find the IDLE neighbour
        whose sector the convoy ENTERED (from the flagged exit bearing) and offer
        it the convoy; the neighbour's acquire loop takes it via claim(). If no
        idle neighbour's sector contains the bearing, the convoy stays flagged
        and its original owner keeps tracking (the 'keep tracking but flagged'
        path). No-op without a registry, a sector map, or any flagged exit.

        'Idle' = owns NOTHING in the registry (inferred from owner_of over the
        known ids — no agent.py change). Offering is idempotent (re-offering the
        same neighbour just refreshes); a convoy already offered to a still-idle,
        still-matching neighbour is left alone."""
        if self._registry is None or not self._drone_sectors:
            return
        from finals.mission.planning.frame import bearing_in_sector
        flagged = self._registry.flagged_exits(now)
        if not flagged:
            return
        idle = self._idle_drones(now)
        for convoy_id, owner, exit_bearing, offered_to in flagged:
            if exit_bearing is None:
                continue                   # owner had no bearing to match on
            target = self._neighbour_for(exit_bearing, owner, idle,
                                         bearing_in_sector)
            if target is None:
                continue                   # nobody idle owns that wedge -> stay
            if target == offered_to:
                continue                   # already offered to this neighbour
            try:
                self._registry.offer_to(convoy_id, target, now)
            except Exception:
                # A matcher race (the owner re-entered / lost the lock between
                # flagged_exits and offer_to) must never down the supervision
                # loop; log with traceback and move on. The convoy stays where
                # it is and is retried next beat.
                self._net(f"handover offer convoy {convoy_id} -> {target}")
                continue
            self._try_log(_MISSION_ID, "convoy_handover_offered",
                          convoy_id=convoy_id, from_drone=owner,
                          to_drone=target, exit_bearing_deg=round(exit_bearing, 2),
                          note="convoy left owner's sector; offered to an idle "
                               "neighbour whose sector it entered")
            # The offeree only TRANSFERS the lock on its own next acquire (claim
            # honours the offer); the original owner releases then. We do NOT
            # force-release here — the owner keeps tracking until the handover
            # actually completes, so a never-taken offer never strands a convoy.

    def _idle_drones(self, now: float) -> set:
        """The drones that own NO convoy right now (the handover candidates).
        Inferred from the registry snapshot's in_flight map (drone -> convoy),
        so no agent.py state is read. A drone with a sector but no current lock
        is idle."""
        snap = self._registry.snapshot(now)
        busy = set(snap.get("in_flight", {}).values())
        return {did for did in self._drone_sectors if did not in busy}

    def _neighbour_for(self, bearing_deg: float, owner: Optional[str],
                       idle: set, in_sector_fn: Callable) -> Optional[str]:
        """The idle drone (other than `owner`) whose sector CONTAINS the exit
        bearing — the neighbour the convoy entered. None if no idle drone's wedge
        covers it. Deterministic: ids checked in sorted order, first match wins
        (sectors should not overlap, but a tie must be stable)."""
        for did in sorted(idle):
            if did == owner:
                continue
            center, half = self._drone_sectors[did]
            if in_sector_fn(bearing_deg, center, half):
                return did
        return None

    def _drain_bus(self) -> None:
        """Seq-cursor drain (lossless by construction — see SightingBus):
        every sighting published since the last tick, logged exactly once."""
        if self._bus is None:
            return
        self._cursor, items = self._bus.drain_after(self._cursor)
        for s in items:
            self._events.log(
                s.drone_id, "sighting", source=s.source,
                class_name=s.class_name, marker_id=s.marker_id,
                confidence=s.confidence, ts=s.ts,
                frame_number=s.frame_number, bearing_deg=s.bearing_deg)
            # Coverage read-back: count reads per marker id (the operator's
            # "did we catch all 5?"). Non-marker detections (marker_id None)
            # are not convoy reads, so they never enter the tally.
            if s.marker_id is not None:
                self._id_counts[s.marker_id] = (
                    self._id_counts.get(s.marker_id, 0) + 1)
        self._n_sightings += len(items)

    def _write_heartbeat(self, now: float, stop_signalled: bool,
                         final: bool = False) -> None:
        assert self._t_start is not None        # set first thing in run()
        payload = {
            "ts": time.time(),
            "tick": self._tick,
            "final": final,
            "elapsed_s": round(now - self._t_start, 3),
            "budget_s": self._budget_s,
            "stop_signalled": stop_signalled,
            "tick_latency_s": round(self._last_tick_latency_s, 6),
            "beat_gap_s": (None if self._last_beat_gap_s is None
                           else round(self._last_beat_gap_s, 6)),
            "sightings_drained": self._n_sightings,
            "drones": {a.drone_id: a.status() for a in self._agents},
        }
        if self._registry is not None:
            # The swarm-wide ownership view: serviced / in_flight / remaining
            # (the 5-of-5 tally) + done. `now` folds in staleness even if a beat
            # raced ahead of expire().
            payload["convoys"] = self._registry.snapshot(now)
        if self._validity_map is not None:
            # The swarm-wide pad-validity view: per-beacon validity + the
            # broadcast invalid_ids (red pads the others skip) + claimed_by
            # (which drone owns which valid pad). `now` stamps read-age.
            payload["pad_validity"] = self._validity_map.snapshot(now)
        cov = self._coverage_tally()
        if cov is not None:
            payload["coverage"] = cov
        write_heartbeat(self._run_dir, payload)

    # ---------------- summary ----------------
    def _print_summary(self) -> None:
        elapsed = (self._clock() - self._t_start
                   if self._t_start is not None else 0.0)
        lines = [
            "=" * 72,
            f"MISSION SUMMARY  elapsed={elapsed:.1f}s  "
            f"budget={self._budget_s:.0f}s  "
            f"sightings={self._n_sightings}  ticks={self._tick}",
            "=" * 72,
            f"{'drone':<10} {'state':<8} {'phases':<8} failure",
        ]
        for a in self._agents:
            st = a.status()
            lines.append(
                f"{a.drone_id:<10} {st['state']:<8} "
                f"{st['phases_completed']}/{st['n_phases']:<6} "
                f"{st['failure'] or '-'}")
        lines.append("=" * 72)
        cov = self._coverage_tally()
        if cov is not None:
            # The operator's read-back: SEEN (decoded at all) vs SERVICED
            # (locked + confirmed + released — only with a registry). reads/
            # weak/missing make a thin or absent convoy id impossible to miss.
            served = (f"serviced {cov['serviced_n']}/{cov['of']} "
                      f"{cov['serviced']}  " if "serviced" in cov else "")
            lines.append(
                f"CONVOY COVERAGE  {served}seen {cov['seen_n']}/{cov['of']} "
                f"{cov['seen']}")
            lines.append("  reads: " + "  ".join(
                f"{cid}:{cov['reads'][cid]}" for cid in cov["known"]))
            if cov["weak"]:
                lines.append(
                    f"  WEAK (<{self._coverage_weak_below} reads): {cov['weak']}")
            if cov["missing"]:
                lines.append(f"  MISSING (0 reads): {cov['missing']}")
            if cov["other_ids"]:
                lines.append(
                    f"  other ids decoded (not a known convoy): {cov['other_ids']}")
            lines.append("=" * 72)
        print("\n".join(lines), flush=True)

    def _coverage_tally(self) -> Optional[dict]:
        """The convoy read-back: of the KNOWN target ids, which have been SEEN
        (decoded at all) and how many reads each — plus, with a registry bound,
        which are SERVICED (locked + confirmed + released). Returns None for a
        non-convoy mission (no known set) so heartbeat/summary stay clean. SEEN
        != SERVICED on purpose: a 1-frame fluke decode is 'seen', not 'caught'."""
        if not self._known_convoy_ids:
            return None
        known = sorted(self._known_convoy_ids)
        reads = {cid: self._id_counts.get(cid, 0) for cid in known}
        seen = sorted(cid for cid in known if reads[cid] > 0)
        tally = {
            "known": known,
            "reads": reads,
            "seen": seen,
            "seen_n": len(seen),
            "of": len(known),
            "weak": sorted(cid for cid in known
                           if 0 < reads[cid] < self._coverage_weak_below),
            "missing": sorted(cid for cid in known if reads[cid] == 0),
            # stray (non-known) decoded ids — surfaced so a dict/world mismatch
            # or misread is visible, never silently dropped.
            "other_ids": {mid: n for mid, n in sorted(self._id_counts.items())
                          if mid not in self._known_convoy_ids},
        }
        if self._registry is not None:
            serviced = sorted(c for c in self._registry.serviced_ids()
                              if c in self._known_convoy_ids)
            tally["serviced"] = serviced
            tally["serviced_n"] = len(serviced)
        return tally

    @staticmethod
    def _validate_convoy_ids(convoy_ids) -> frozenset:
        """The known convoy id set (the 5-of-5 denominator) as a frozenset of
        ints. None -> empty (no coverage block). Loud on a bad shape — a
        malformed known set would silently disable the read-back."""
        if convoy_ids is None:
            return frozenset()
        try:
            ids = list(convoy_ids)
        except TypeError:
            raise ValueError(
                f"Orchestrator: convoy_ids must be an iterable of ints or None, "
                f"got {convoy_ids!r}")
        for c in ids:
            if not isinstance(c, int) or isinstance(c, bool):
                raise ValueError(
                    f"Orchestrator: convoy_ids must be ints, got {c!r} in "
                    f"{ids!r} — these are the known convoy marker ids")
        return frozenset(ids)
