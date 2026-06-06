"""Crash-safe run logging: JSONL event log, atomic heartbeat, crash hooks.

The post-crash forensics cluster. Everything here assumes the process dies at
the worst possible byte:

- Events are append-only JSONL, flushed per line. Append + flush survives a
  process crash (the page cache holds flushed lines); readers tolerate a torn
  trailing line (`read_events`).
- The heartbeat is rewritten atomically (tmp + fsync + os.replace), so a
  concurrent reader can NEVER observe partial JSON — the published filename
  only ever points at fully-fsynced bytes. After any crash, heartbeat.json is
  the forensic snapshot of the last good second.
- Crash hooks cover the code path you can never test by accident:
  faulthandler (hard faults / deadlocked-thread dumps) -> fault.txt, and a
  chained sys.excepthook (uncaught exceptions) -> crash.txt.

Derives from: barrel_log.py (lock discipline; atomic tmp+os.replace pattern)
and the qualifier runs/<YYYYMMDD_HHMMSS>/ output convention. Bugs fixed in
adaptation:
- barrel_log._flush() rewrites the WHOLE file per add (barrel_log.py:74-87) —
  events here are append-only: no rewrite, no window where the log is empty.
- The qualifier run-dir creation used exist_ok=True with no collision
  handling — two runs starting in the same second silently merged their
  outputs. create_run_dir uses atomic os.mkdir + bounded suffixing.
- os.replace on Windows raises PermissionError while ANY reader holds the
  destination open (CPython's open() does not pass FILE_SHARE_DELETE);
  write_heartbeat retries with a bounded backoff instead of dying at 1 Hz.

Session: S2 (implemented).
"""
from __future__ import annotations

import faulthandler
import json
import os
import re
import sys
import threading
import time
import traceback
from typing import Dict, Iterator, Optional, TextIO

from finals.errors import SensorError

# Immutable module constants only (convention 4: no module-level mutable globals).
_RUN_DIR_COLLISION_ATTEMPTS = 100
_HEARTBEAT_REPLACE_ATTEMPTS = 50
_HEARTBEAT_REPLACE_BACKOFF_S = 0.01      # 50 x 10 ms = 0.5 s worst case < 1 Hz beat
_CRASH_HOOK_ATTR = "_finals_crash_hook_run_dir"
_SAFE_ID_RE = re.compile(r"[^A-Za-z0-9_.-]")


class EventLogError(SensorError):
    """Event/heartbeat/crash-log persistence failed. The message carries the
    path, the errno, and what to check — never silently dropped."""


# ============================================================
# Run directory
# ============================================================
def create_run_dir(base: str = "./runs_finals") -> str:
    """Create runs_finals/<YYYYMMDD_HHMMSS>/ and print a LOUD banner with the
    absolute path — post-crash, the first thing a human needs is "where are
    the logs". Returns the absolute path.

    Same-second collisions (supervisor restart loops) get a bounded _01.._99
    suffix via atomic os.mkdir create-or-fail — never exist_ok=True, which
    would silently merge two runs' outputs (the qualifier bug).
    """
    base_abs = os.path.abspath(base)
    try:
        os.makedirs(base_abs, exist_ok=True)
    except OSError as e:
        raise EventLogError(
            f"cannot create run base dir {base_abs!r} — errno {e.errno} "
            f"({e.strerror}) — check the drive exists and is writable"
        ) from e
    stamp = time.strftime("%Y%m%d_%H%M%S")
    for n in range(_RUN_DIR_COLLISION_ATTEMPTS):          # bounded (convention 3)
        name = stamp if n == 0 else f"{stamp}_{n:02d}"
        path = os.path.join(base_abs, name)
        try:
            os.mkdir(path)            # atomic create-or-fail: no TOCTOU race
        except FileExistsError:
            continue                  # same-second collision -> next suffix
        except OSError as e:
            raise EventLogError(
                f"cannot create run dir {path!r} — errno {e.errno} "
                f"({e.strerror}) — check permissions / disk space"
            ) from e
        print(f"{'=' * 20} RUN DIR: {path} {'=' * 20}", flush=True)
        return path
    raise EventLogError(
        f"{_RUN_DIR_COLLISION_ATTEMPTS} run dirs already exist for second "
        f"{stamp!r} under {base_abs!r} — runaway restart loop? check the "
        f"supervisor before flying anything"
    )


# ============================================================
# Event log
# ============================================================
class EventLog:
    """Append-only JSONL mission log, thread-safe (detector callbacks fire on
    worker threads).

    Every .log() call appends ONE line
        {"ts": <time.time()>, "mono": <time.monotonic()>,
         "drone": ..., "event": ..., "data": {...}}
    to BOTH mission.jsonl and drone_<id>.jsonl, flushed per line. Flush (not
    fsync) is deliberate: events are high-rate forensics where surviving a
    process crash is enough; fsync per event would stall the 10 Hz tick
    (fsync is reserved for the score-relevant sighting CSV).

    Reopening an existing run dir newline-terminates a torn trailing line so
    new lines never merge into the fragment (read_events then skips it).
    """

    def __init__(self, run_dir: str):
        self.run_dir = os.path.abspath(run_dir)
        self._lock = threading.Lock()
        self._closed = False
        self._drone_files: Dict[str, TextIO] = {}      # lazy: drone_id -> handle
        self._mission = self._open(os.path.join(self.run_dir, "mission.jsonl"))

    def _open(self, path: str) -> TextIO:
        """Open a JSONL file for append; repair a torn tail first (kill-test
        reload: a crash mid-line must not corrupt the next line)."""
        try:
            needs_newline = False
            if os.path.exists(path) and os.path.getsize(path) > 0:
                with open(path, "rb") as probe:
                    probe.seek(-1, os.SEEK_END)
                    needs_newline = probe.read(1) != b"\n"
            f = open(path, "a", encoding="utf-8", newline="\n")
            if needs_newline:
                f.write("\n")          # fragment becomes its own (skippable) line
                f.flush()
                print(
                    f"[EventLog] WARNING: {path} had a torn unterminated tail "
                    f"— newline-terminated it so new lines stay parseable "
                    f"(read_events will skip the fragment)",
                    file=sys.stderr, flush=True,
                )
            return f
        except OSError as e:
            raise EventLogError(
                f"cannot open event log {path!r} — errno {e.errno} "
                f"({e.strerror}) — check the run dir still exists / disk space"
            ) from e

    def log(self, drone_id: str, event: str, **data) -> None:
        """Append one event line to mission.jsonl AND drone_<id>.jsonl.
        Thread-safe; raises EventLogError on any write failure."""
        record = {
            "ts": time.time(),
            "mono": time.monotonic(),
            "drone": drone_id,
            "event": event,
            "data": data,
        }
        # Serialize OUTSIDE the lock; default=repr so an exotic payload can
        # never crash the mission loop — it degrades to a loud, greppable
        # repr string instead of being silently dropped. ensure_ascii=False:
        # logs are read by humans at 2 a.m.; 'Δ' escapes are not.
        line = json.dumps(record, ensure_ascii=False, default=repr) + "\n"
        with self._lock:
            if self._closed:
                raise EventLogError(
                    f"EventLog.log({drone_id!r}, {event!r}) after close() — "
                    f"run dir {self.run_dir} — check shutdown ordering"
                )
            f = self._drone_files.get(drone_id)
            if f is None:                                  # lazy per-drone open
                safe = _SAFE_ID_RE.sub("_", drone_id) or "unknown"
                f = self._open(os.path.join(self.run_dir, f"drone_{safe}.jsonl"))
                self._drone_files[drone_id] = f
            try:
                self._mission.write(line)
                self._mission.flush()
                f.write(line)
                f.flush()
            except OSError as e:
                raise EventLogError(
                    f"event write failed for drone {drone_id!r} event "
                    f"{event!r} in {self.run_dir} — errno {e.errno} "
                    f"({e.strerror}) — check disk space / run dir not deleted "
                    f"mid-mission"
                ) from e

    def close(self) -> None:
        """Idempotent. Tests (and Windows tmp-dir cleanup) need closed handles."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            for f in [self._mission, *self._drone_files.values()]:
                try:
                    f.close()
                except OSError:
                    print(f"[EventLog] WARNING: close failed for {f.name}",
                          file=sys.stderr, flush=True)

    def __enter__(self) -> "EventLog":
        return self

    def __exit__(self, *exc) -> bool:
        self.close()
        return False


# ============================================================
# Heartbeat
# ============================================================
def write_heartbeat(run_dir: str, snapshot: dict) -> None:
    """Atomically rewrite <run_dir>/heartbeat.json with `snapshot`.

    INTENT: this file is the post-crash forensic snapshot of the last good
    second — per-drone agent state, last telemetry, battery, frame age, last
    command + result, tick latency. The orchestrator rewrites it at 1 Hz.

    GUARANTEE: a concurrent reader can never observe partial JSON. The bytes
    are written to a tmp file, fsynced, and only then published with the
    atomic os.replace — the heartbeat.json name only ever points at a file
    whose content is complete on disk.

    Windows note: os.replace is MoveFileEx(REPLACE_EXISTING); it raises
    PermissionError while ANY reader holds heartbeat.json open (CPython's
    open() does not pass FILE_SHARE_DELETE). Readers hold the small file for
    sub-ms, so a bounded retry (50 x 10 ms) absorbs it; persistent denial
    (a viewer / antivirus / indexer pinning the file) raises EventLogError.

    Single-writer assumption (documented): one 1 Hz heartbeat loop per run.
    The fixed tmp name means a stale tmp from a crash (or from a failed
    best-effort cleanup below) is simply overwritten by the next beat —
    self-healing; tmp removal on the error paths is best-effort only.
    """
    final = os.path.join(run_dir, "heartbeat.json")
    tmp = final + ".tmp"
    text = json.dumps(snapshot, indent=2, ensure_ascii=False, default=repr)
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())   # tmp is COMPLETE on disk before publish
    except OSError as e:
        _remove_quietly(tmp)
        raise EventLogError(
            f"heartbeat tmp write failed: {tmp!r} — errno {e.errno} "
            f"({e.strerror}) — check disk space / run dir exists"
        ) from e

    last_err: Optional[OSError] = None
    for _attempt in range(_HEARTBEAT_REPLACE_ATTEMPTS):     # bounded retry
        try:
            os.replace(tmp, final)                          # atomic publish
            return
        except PermissionError as e:                        # Windows sharing violation
            last_err = e
            time.sleep(_HEARTBEAT_REPLACE_BACKOFF_S)
        except OSError as e:
            _remove_quietly(tmp)
            raise EventLogError(
                f"heartbeat replace {tmp!r} -> {final!r} failed — errno "
                f"{e.errno} ({e.strerror}) — check the run dir was not "
                f"deleted mid-mission"
            ) from e
    _remove_quietly(tmp)
    raise EventLogError(
        f"heartbeat replace -> {final!r} still sharing-violated after "
        f"{_HEARTBEAT_REPLACE_ATTEMPTS} tries "
        f"(~{_HEARTBEAT_REPLACE_ATTEMPTS * _HEARTBEAT_REPLACE_BACKOFF_S:.1f} s, "
        f"winerror {getattr(last_err, 'winerror', '?')}) — check: a viewer / "
        f"antivirus / search indexer is holding heartbeat.json open; exclude "
        f"the run dir from real-time scanning"
    ) from last_err


def _remove_quietly(path: str) -> None:
    """Best-effort tmp cleanup on an error path — the caller is already
    raising the real error; a cleanup failure must not mask it."""
    try:
        os.remove(path)
    except OSError:
        pass


# ============================================================
# Crash hooks
# ============================================================
def install_crash_hooks(run_dir: str) -> None:
    """Enable faulthandler -> <run_dir>/fault.txt and chain sys.excepthook ->
    <run_dir>/crash.txt (full traceback + timestamp), then call the previous
    hook. Idempotent: calling again with the same run_dir is a no-op.

    Idempotence lives on the installed hook itself (an attribute on the
    function assigned to sys.excepthook) — no module-level mutable globals
    (convention 4). The fault.txt file object is also pinned as a hook
    attribute (belt-and-suspenders: faulthandler.enable() keeps its own
    strong reference, but the pin makes the lifetime explicit and survives
    a later faulthandler re-point).

    Limitation (deliberate, see S5/S7): sys.excepthook does not fire for
    uncaught exceptions on worker threads (threading.excepthook does); the
    detector threads get their own guard wrappers in later sessions.
    """
    run_dir = os.path.abspath(run_dir)
    if getattr(sys.excepthook, _CRASH_HOOK_ATTR, None) == run_dir:
        return                                # already installed for this run

    fault_path = os.path.join(run_dir, "fault.txt")
    try:
        fault_file = open(fault_path, "w", encoding="utf-8")
    except OSError as e:
        raise EventLogError(
            f"cannot open {fault_path!r} for faulthandler — errno {e.errno} "
            f"({e.strerror}) — check the run dir exists"
        ) from e
    faulthandler.enable(file=fault_file)      # re-points if already enabled

    prev_hook = sys.excepthook                # chain, never clobber
    crash_path = os.path.join(run_dir, "crash.txt")

    def _crash_hook(exc_type, exc, tb):
        try:
            with open(crash_path, "a", encoding="utf-8") as f:
                f.write(
                    f"\n=== UNCAUGHT {exc_type.__name__} at "
                    f"{time.strftime('%Y-%m-%d %H:%M:%S')} "
                    f"(epoch {time.time():.3f}) ===\n"
                )
                traceback.print_exception(exc_type, exc, tb, file=f)
        except OSError as e:
            # Never mask the real crash with a logging failure — the chained
            # previous hook below still prints the traceback to stderr.
            print(f"[crash-hook] could not write {crash_path}: {e}",
                  file=sys.stderr, flush=True)
        prev_hook(exc_type, exc, tb)

    setattr(_crash_hook, _CRASH_HOOK_ATTR, run_dir)   # idempotence marker
    _crash_hook._finals_fault_file = fault_file        # pin the fd alive
    sys.excepthook = _crash_hook


# ============================================================
# Reader
# ============================================================
def read_events(path: str) -> Iterator[dict]:
    """Yield parsed event dicts from a JSONL file, skipping torn/unparseable
    lines with a loud counted warning instead of crashing — crash-time files
    WILL have torn tails.

    A final line with no terminator is skipped EVEN IF it happens to parse:
    a missing terminator means the write was cut, so completeness cannot be
    trusted. Determinism beats coincidence.
    """
    try:
        f = open(path, "r", encoding="utf-8", errors="replace", newline="")
    except OSError as e:
        raise EventLogError(
            f"cannot read events file {path!r} — errno {e.errno} "
            f"({e.strerror}) — check the path (wrong run dir?)"
        ) from e
    skipped = 0
    with f:
        for lineno, raw in enumerate(f, 1):   # newline="" keeps terminators
            if not raw.endswith("\n"):
                skipped += 1
                print(
                    f"[read_events] WARNING: skipped torn trailing line "
                    f"{lineno} in {path}: {raw[:80]!r}",
                    file=sys.stderr, flush=True,
                )
                continue
            line = raw.rstrip("\r\n")
            if not line:
                continue                      # blank line: noise, not data
            try:
                yield json.loads(line)
            except json.JSONDecodeError as e:
                skipped += 1
                print(
                    f"[read_events] WARNING: skipped unparseable line "
                    f"{lineno} in {path} ({e.msg}): {line[:80]!r}",
                    file=sys.stderr, flush=True,
                )
    if skipped:
        print(f"[read_events] WARNING: skipped {skipped} bad line(s) in {path}",
              file=sys.stderr, flush=True)    # the COUNTED summary


# ============================================================
# Manual smoke demo
# ============================================================
if __name__ == "__main__":
    demo_dir = create_run_dir("./runs_finals")
    install_crash_hooks(demo_dir)

    with EventLog(demo_dir) as ev:
        ev.log("alpha", "takeoff", height_cm=80)
        ev.log("alpha", "move", direction="FORWARD", distance_cm=100)
        ev.log("bravo", "takeoff", height_cm=80)
        ev.log("alpha", "telemetry", battery_pct=91.5, alt_m=0.8)
        ev.log("bravo", "rotate", angle_deg=90.0)
        ev.log("alpha", "sighting", source="aruco", marker_id=17)
        ev.log("bravo", "telemetry", battery_pct=88.0, alt_m=1.2)
        ev.log("alpha", "phase_done", phase="takeoff_demo")
        ev.log("bravo", "guard", name="battery", status="ok")
        ev.log("alpha", "land")

    write_heartbeat(demo_dir, {
        "tick": 42,
        "drones": {
            "alpha": {"battery_pct": 91.5, "phase": "takeoff_demo",
                      "last_cmd": "land", "frame_age_s": 0.13},
            "bravo": {"battery_pct": 88.0, "phase": "takeoff_demo",
                      "last_cmd": "rotate", "frame_age_s": 0.09},
        },
    })

    n_read = sum(1 for _ in read_events(os.path.join(demo_dir, "mission.jsonl")))
    print(f"read back {n_read} events from mission.jsonl")

    print(f"\nrun dir tree ({demo_dir}):")
    for root, _dirs, files in os.walk(demo_dir):
        for fname in sorted(files):
            fpath = os.path.join(root, fname)
            print(f"  {fname:<24} {os.path.getsize(fpath):>6} bytes")
