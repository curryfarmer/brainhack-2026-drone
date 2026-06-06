"""Orchestrator — THE single 10 Hz asyncio loop ticking all agents.

Planned surface (S4):
- run() -> int (exit code): start perception loops (one task per drone with a
  VideoSource); tick loop at cfg.tick_hz calling agent.tick(now, loop) for
  every agent; exit when all agents are DONE/FAILED, the mission budget is
  exceeded, or AbortRequested fires.
- Guards evaluated each tick BEFORE agents step (finals.guards, S5); a guard
  that raises is itself a trip.
- Landing slot: ONE drone may be in landing states at a time (descending
  crosses the other drones' altitude bands — serialized landings are half of
  the swarm collision guarantee, altitude bands are the other half).
- Operator kill: KeyboardInterrupt and the AbortListener 'q' channel both
  route to concurrent emergency_land on every flying agent BEFORE anything
  else happens.
- finally: per-agent shutdown (land if flying -> disconnect -> sources
  stopped), detector.stop(), then a loud parseable summary table (per drone:
  phases completed, sightings count, failures).
- One of the THREE whitelisted `except Exception` sites lives here (the top
  loop) — always logged with traceback, never silent.
- NO auto-restart in flight profiles: a crash-restart that re-arms 3 real
  aircraft is unsafe. Per-drone FAILED is terminal; the others continue.
  (Deliberate departure from qualifier_run.py:407-513's supervisor restart.)

Derives from: qualifier_run.py supervisor (long-lived singletons, budget
clock, traceback printing — minus the restart), hula_connection.py main loop.

STUB — session S4.
"""
from __future__ import annotations

_STUB = "finals.mission.orchestrator: session S4 — see finals/docs/module_map.md"


class Orchestrator:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)
