"""In-flight guards: Guard ABC, concrete watchdogs, SafetyController, AbortListener.

Planned surface (S5) — evaluated by the orchestrator each tick BEFORE phases
step; a guard that RAISES is itself treated as a trip (a buggy guard must
never silently disable guarding):

- Guard.check(ctx, now) -> TripAction | None where TripAction is one of
  DEGRADE_DETECTION | HOLD_THIS | LAND_THIS | LAND_ALL.
- Concrete guards (initial thresholds; bench-tuned via config, never code):
  TelemetryWatchdog (stale > 2 s -> LAND_THIS), CommandGuard (timeout/typed
  SDK error; moves are NEVER blind-retried — a re-sent relative move doubles
  the distance), VideoWatchdog (stale/ERROR -> DEGRADE + bounded restart;
  never auto-lands), BatteryGuard (warn 30% / floor 20% -> LAND_THIS),
  MissionClockGuard (budget minus landing reserve -> LAND_ALL),
  LoopOverrunGuard (tick > 2x period x5 -> shed detection -> display ->
  LAND_ALL), GeofenceLite (advisory — position is untrusted), PhaseTimeout.
- SafetyController: idempotent trip execution; per-drone landing with retries
  (land @1 Hz for 30 s -> escalate to operator alarm). One of the THREE
  whitelisted `except Exception` sites in the package (a throw from drone 1's
  landing must not prevent landing drones 2-3) — every swallow logged with
  traceback.
- AbortListener: dedicated THREAD (not asyncio — must work even if the event
  loop wedges) watching for the kill key; sets a threading.Event AND directly
  fires emergency_land per drone. Safety-only channel: it can only land,
  never steer (competition-rules question flagged in the plan).

Derives from: qualifier_run.py supervisor/emergency-land paths; the watchdog
gap list audited out of mapping_drone.py (UWB wait-forever, telemetry tasks
that never exit).

STUB — session S5.
"""
from __future__ import annotations

_STUB = "finals.guards: session S5 — see finals/docs/module_map.md"


class Guard:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)


class SafetyController:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)


class AbortListener:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)
