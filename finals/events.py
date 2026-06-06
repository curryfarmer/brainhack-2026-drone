"""EventLog — append-only JSONL mission log + 1 Hz heartbeat.json + crash hooks.

Planned surface (S2):
- EventLog(run_dir): .log(drone_id, event, **data) appends one JSON line
  ({"ts", "mono", "drone", "event", "data"}) to mission.jsonl and the per-drone
  drone_<id>.jsonl, flushed per line (append-only = inherently crash-safe).
- write_heartbeat(run_dir, snapshot): atomic tmp+os.replace rewrite at 1 Hz —
  per-drone agent state, last telemetry, battery, frame age, last command +
  result, tick latency. After any crash this file is the forensic snapshot of
  the last good second.
- install_crash_hooks(run_dir): faulthandler.enable(fault.txt) + sys.excepthook
  writing crash.txt with full traceback before exit.

Derives from: barrel_log.py (atomic tmp+os.replace flush discipline, lock
usage) and the qualifier runs/<timestamp>/ output convention.

STUB — session S2.
"""
from __future__ import annotations

_STUB = "finals.events: session S2 — see finals/docs/module_map.md"


class EventLog:
    def __init__(self, *args, **kwargs):
        raise NotImplementedError(_STUB)


def write_heartbeat(*args, **kwargs):
    raise NotImplementedError(_STUB)


def install_crash_hooks(*args, **kwargs):
    raise NotImplementedError(_STUB)
