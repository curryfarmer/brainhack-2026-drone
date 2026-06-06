"""SightingLog — append-only per-sighting CSV — and SightingBus, the
thread-safe handoff from detector worker threads to the asyncio orchestrator.

Planned surface (S2):
- SightingLog(csv_path): .append(s: Sighting) -> int (sighting_id). Header
  written once at open; each row flush()+os.fsync() — a crash loses at most
  the in-flight row. NO dedup: the convoy MOVES; barrel_log.py's
  dedup/running-mean is the wrong tool (and its _flush() rewrites the whole
  file per add — verified barrel_log.py:74-87). Column order mirrors
  finals.types.Sighting fields.
- SightingBus(maxlen=500): .publish(s) from ANY thread (detector callbacks);
  .drain_since(ts, drone_id=None) and .latest(drone_id, source=None) polled by
  the orchestrator each tick. deque + threading.Lock.
- Track association (nearest-neighbor gating) is DEFERRED until the briefing
  says whether tracking (vs. per-sighting logging) scores.

Derives from: barrel_log.py (lock discipline, crash-safe persistence intent)
with the persistence model inverted to append-only for moving targets.

STUB — session S2.
"""
from __future__ import annotations

_STUB = "finals.sightings: session S2 — see finals/docs/module_map.md"


class SightingLog:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)


class SightingBus:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)
