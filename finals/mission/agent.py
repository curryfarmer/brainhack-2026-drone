"""DroneAgent — one drone's runtime: phase queue, single in-flight command,
watchdog hooks.

Planned surface (S4):
- AgentState: INIT, READY, RUNNING, LANDING, DONE, FAILED. FAILED is TERMINAL
  for flight profiles (no auto-restart that re-arms a real aircraft — the
  deliberate departure from qualifier_run.py's supervisor, documented there).
- tick(now, loop) — NON-BLOCKING, called by the orchestrator at tick_hz:
  * command task in flight? check .done(); record ok/error into the next ctx.
  * watchdog hooks every tick (the guards themselves live in finals.guards):
    battery floor -> force Land; telemetry stale -> log loud then FAILED +
    emergency_land; mission budget exceeded -> force Land.
  * else build AgentContext -> action = phase.step(ctx) -> dispatch as
    loop.create_task(self._execute(action)) wrapped so ANY exception is
    captured into last_action_error (never lost to the void) and the FAILED
    transition fires emergency_land EXACTLY once.
- Done(reason) advances the phase queue (on_exit/on_enter hooks); queue empty
  -> LANDING -> DONE. Abort(reason) -> FAILED + safe-down.
- shutdown(): land-if-flying -> disconnect -> stop sources. Never raises.

Derives from: the per-drone dict + state-loop pattern of
hula_connection.py:39-63 (officially recommended), with every watchdog gap
catalogued from mapping_drone.py (UWB wait-forever, telemetry tasks that
never exit) turned into an explicit guard hook.

STUB — session S4.
"""
from __future__ import annotations

_STUB = "finals.mission.agent: session S4 — see finals/docs/module_map.md"


class DroneAgent:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)
