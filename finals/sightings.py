"""SightingLog — append-only per-sighting CSV — and SightingBus, the
thread-safe handoff from detector worker threads to the asyncio orchestrator.

- SightingLog appends one CSV row per sighting with flush()+os.fsync() — a
  crash loses at most the in-flight row. NO dedup, ever: the convoy MOVES, so
  barrel_log.py's dedup/running-mean is the wrong tool. Columns mirror
  finals.types.Sighting fields via dataclasses.fields() — no hand-maintained
  column list to drift.
- SightingBus is the bounded in-memory fan-in (deque + threading.Lock) that
  detector callbacks publish into and the orchestrator polls each tick.
- Track association (nearest-neighbor gating) is DEFERRED until the briefing
  says whether tracking (vs. per-sighting logging) scores.

Derives from: barrel_log.py (lock discipline, crash-safe persistence intent)
with the persistence model inverted to append-only for moving targets. Bugs
fixed in adaptation:
- barrel_log._flush() rewrites the WHOLE file per add (barrel_log.py:74-87):
  O(n) per sighting and a crash window where the file is mid-rewrite. Here:
  append + fsync, O(1) per row, the file is never rewritten.
- barrel_log's hand-maintained fieldnames list (barrel_log.py:79-83) can
  drift from its dataclass; here the schema derives from the dataclass.
- barrel_log silently trusts an existing CSV on load; here a torn last row
  (crash mid-append) is detected, loudly warned about, and truncated so ids
  stay correct and future appends stay parseable.

Session: S2 (implemented).
"""
from __future__ import annotations

import csv
import dataclasses
import os
import sys
import threading
from collections import deque
from enum import IntEnum
from typing import (Any, Deque, List, Optional, Tuple, Union,
                    get_args, get_origin, get_type_hints)

from finals.errors import SensorError
from finals.types import Sighting

# Cell-internal separator for tuple fields: never collides with CSV commas.
_TUPLE_SEP = ";"


class SightingLogError(SensorError):
    """Sighting CSV persistence failed or the file is corrupt. The message
    carries the path, the row, and what to check."""


# ============================================================
# CSV cell codec — dispatch on the RESOLVED type hint, never on a
# hand-maintained per-column table (which would drift when Sighting
# gains fields). types.py uses `from __future__ import annotations`,
# so hints must be resolved via typing.get_type_hints().
# ============================================================
def _strip_optional(hint: Any) -> "Tuple[Any, bool]":
    """Optional[X] -> (X, True); X -> (X, False)."""
    if get_origin(hint) is Union:
        non_none = [a for a in get_args(hint) if a is not type(None)]
        if len(non_none) == 1:
            return non_none[0], True
    return hint, False


def _encode_cell(name: str, value: Any, hint: Any) -> str:
    inner, is_optional = _strip_optional(hint)
    if value is None:
        if not is_optional:
            raise SightingLogError(
                f"Sighting.{name} is None but its declared type {hint!r} is "
                f"not Optional — an upstream detector produced an invalid "
                f"Sighting; check the detector that emitted it"
            )
        return ""                                          # None <-> ""
    if get_origin(inner) is tuple:
        args = get_args(inner)
        if len(value) != len(args):
            # zip would silently truncate — and a wrong-arity row, once
            # fsynced, either bricks the whole-file reload (mid-file) or is
            # silently truncated as a fake torn tail (last row). Refuse here.
            raise SightingLogError(
                f"Sighting.{name} has {len(value)} element(s) but its "
                f"declared type {hint!r} requires exactly {len(args)} — an "
                f"upstream detector produced an invalid Sighting (e.g. a raw "
                f"HxWxC frame.shape where (h, w) was meant); check the "
                f"detector that emitted it"
            )
        return _TUPLE_SEP.join(
            _encode_cell(name, v, a) for v, a in zip(value, args))
    if isinstance(inner, type) and issubclass(inner, IntEnum):
        return str(int(value))      # IntEnum BEFORE int; value is the pinned contract
    if inner is float:
        return repr(float(value))   # repr(float) roundtrips exactly in py3
    if inner is int:
        return str(int(value))
    if inner is str:
        s = str(value)
        if is_optional and s == "":
            # '' and None both encode to an empty cell, so '' would silently
            # reload as None after a crash — the one value that mutates only
            # across the recovery reload. Refuse it loudly instead.
            raise SightingLogError(
                f"Sighting.{name} is '' — indistinguishable from None in the "
                f"CSV cell encoding; use None for absent values (check the "
                f"code that built this Sighting)"
            )
        if "\n" in s or "\r" in s:
            raise SightingLogError(
                f"Sighting.{name} contains a newline ({s[:60]!r}) — embedded "
                f"newlines would break the line-oriented torn-tail recovery; "
                f"sanitize the value upstream"
            )
        return s
    raise SightingLogError(
        f"no CSV encoder for Sighting.{name} (type {hint!r}) — the Sighting "
        f"dataclass gained a new field type; add a dispatch case in "
        f"finals/sightings.py"
    )


def _decode_cell(name: str, cell: str, hint: Any) -> Any:
    """Inverse of _encode_cell. Raises ValueError for DATA problems (recovery
    classifies torn rows by catching it) and SightingLogError for SCHEMA
    problems (always fatal)."""
    inner, is_optional = _strip_optional(hint)
    if cell == "" and is_optional:
        return None
    if get_origin(inner) is tuple:
        parts = cell.split(_TUPLE_SEP)
        args = get_args(inner)
        if len(parts) != len(args):
            raise ValueError(
                f"{name}: expected {len(args)} tuple element(s), got {len(parts)}")
        return tuple(_decode_cell(name, p, a) for p, a in zip(parts, args))
    if isinstance(inner, type) and issubclass(inner, IntEnum):
        return inner(int(cell))
    if inner is float:
        return float(cell)
    if inner is int:
        return int(cell)
    if inner is str:
        return cell
    raise SightingLogError(
        f"no CSV decoder for Sighting.{name} (type {hint!r}) — add a "
        f"dispatch case in finals/sightings.py"
    )


# ============================================================
# SightingLog
# ============================================================
class SightingLog:
    """Append-only, crash-safe, thread-safe sighting CSV.

    - Header written once; columns = Sighting fields in declared order.
    - .append(s) returns the 1-based sighting_id (= row order; there is no id
      column). Each row is flush()ed and os.fsync()ed — a crash loses at most
      the in-flight row.
    - Reopening an existing file loads its rows (ids continue) and tolerates
      a torn last row: skipped with a loud counted warning and TRUNCATED
      (the partial row was never fsync-complete and is unrecoverable; leaving
      it in place would corrupt the next append and shift row-order ids).
      An unparseable MID-file row raises instead — append+fsync cannot
      produce one, so it signals external tampering, and skipping it would
      shift ids already handed out.
    - One process per run dir (documented assumption — two "a"-mode handles
      from different processes would interleave rows).
    """

    def __init__(self, csv_path: str):
        self.csv_path = os.path.abspath(csv_path)
        self._fieldnames = [f.name for f in dataclasses.fields(Sighting)]
        self._hints = get_type_hints(Sighting)   # resolves string annotations
        self._lock = threading.Lock()
        self._closed = False
        self._failed = False                     # poisoned by a failed append
        self._rows: List[Sighting] = []          # mirror: snapshot() + id continuation

        parent = os.path.dirname(self.csv_path)
        if parent:
            try:
                os.makedirs(parent, exist_ok=True)
            except OSError as e:
                raise SightingLogError(
                    f"cannot create directory {parent!r} for sighting log — "
                    f"errno {e.errno} ({e.strerror}) — check the drive/path"
                ) from e

        need_header = self._recover_existing()   # BEFORE opening append handle
        try:
            # newline="" is REQUIRED by the csv module; lineterminator="\n"
            # gives ONE canonical terminator so torn-tail byte math is exact.
            self._f = open(self.csv_path, "a", encoding="utf-8", newline="")
        except OSError as e:
            raise SightingLogError(
                f"cannot open {self.csv_path!r} for append — errno {e.errno} "
                f"({e.strerror}) — check the directory exists / disk space"
            ) from e
        self._writer = csv.writer(self._f, lineterminator="\n")
        if need_header:
            try:
                self._writer.writerow(self._fieldnames)
                self._f.flush()
                os.fsync(self._f.fileno())       # the header survives a crash too
            except OSError as e:
                raise SightingLogError(
                    f"cannot write CSV header to {self.csv_path!r} — errno "
                    f"{e.errno} ({e.strerror}) — check disk space"
                ) from e

    # ---------------- recovery ----------------
    def _recover_existing(self) -> bool:
        """Load complete rows into memory; truncate a torn tail.
        Returns True iff a header still needs to be written.

        Truncation offsets are computed from the RAW BYTES (never from
        re-encoded decoded text — U+FFFD replacement would drift the offset),
        so truncation always lands on an exact line boundary. Invalid UTF-8
        in a terminated line is external tampering (this writer is strict
        UTF-8) and raises rather than silently loading a mangled row.
        """
        try:
            with open(self.csv_path, "rb") as f:
                blob = f.read()
        except FileNotFoundError:
            return True
        except OSError as e:
            raise SightingLogError(
                f"cannot read existing sighting log {self.csv_path!r} — "
                f"errno {e.errno} ({e.strerror}) — check permissions"
            ) from e
        if not blob:
            return True                          # created-then-killed: empty

        keep = len(blob)
        torn_tail = b""
        if not blob.endswith(b"\n"):             # torn (a): no trailing newline
            nl = blob.rfind(b"\n")
            keep = nl + 1 if nl >= 0 else 0
            torn_tail = blob[keep:]

        line_blobs = blob[:keep].split(b"\n")[:-1]   # each was \n-terminated
        if not line_blobs:                       # only a torn header fragment
            self._truncate_tail(0, torn_tail or blob)
            return True

        header = next(csv.reader([self._decode_line(line_blobs[0], 1)]), None)
        if header != self._fieldnames:
            raise SightingLogError(
                f"{self.csv_path}: header mismatch — file has {header}, "
                f"Sighting now declares {self._fieldnames} — the dataclass "
                f"changed since this CSV was written; move the old file aside"
            )

        offset = len(line_blobs[0]) + 1          # +1 for the \n terminator
        last_index = len(line_blobs)
        for i, lb in enumerate(line_blobs[1:], start=2):
            try:
                line = self._decode_line(lb, i)
                if not line:
                    raise ValueError("blank line")
                cells = next(csv.reader([line]))
                if len(cells) != len(self._fieldnames):
                    raise ValueError(
                        f"{len(cells)} cell(s), expected {len(self._fieldnames)}")
                values = {n: _decode_cell(n, c, self._hints[n])
                          for n, c in zip(self._fieldnames, cells)}
                self._rows.append(Sighting(**values))
            except (ValueError, csv.Error) as e:
                if i == last_index and not torn_tail:
                    # torn (b): terminated-but-unparseable LAST row — treat
                    # as torn (crash exactly between row bytes and fsync).
                    self._truncate_tail(offset, lb + b"\n", reason=str(e))
                    return False
                raise SightingLogError(
                    f"{self.csv_path} line {i}: unparseable mid-file row "
                    f"({e}) — append+fsync cannot produce this; the file was "
                    f"edited by hand or shared between processes — fix it or "
                    f"move it aside"
                ) from e
            offset += len(lb) + 1

        if torn_tail:                            # clean body + ragged tail
            self._truncate_tail(keep, torn_tail)
        return False

    def _decode_line(self, lb: bytes, lineno: int) -> str:
        """Strict UTF-8 on purpose: this writer can never produce invalid
        bytes in a TERMINATED line (a crash mid-char leaves a torn,
        unterminated tail instead), so invalid UTF-8 here = tampering."""
        try:
            return lb.decode("utf-8")
        except UnicodeDecodeError as e:
            raise SightingLogError(
                f"{self.csv_path} line {lineno}: invalid UTF-8 ({e}) — this "
                f"writer only ever produces strict UTF-8, so the file was "
                f"modified externally; fix it or move it aside"
            ) from e

    def _truncate_tail(self, offset: int, tail: bytes,
                       reason: str = "no trailing newline") -> None:
        print(
            f"[SightingLog] WARNING: skipped 1 torn trailing row in "
            f"{self.csv_path} ({reason}) — discarding {len(tail)} byte(s): "
            f"{tail[:120]!r}",
            file=sys.stderr, flush=True,
        )
        try:
            with open(self.csv_path, "r+b") as f:
                f.truncate(offset)
        except OSError as e:
            raise SightingLogError(
                f"cannot truncate torn tail of {self.csv_path!r} — errno "
                f"{e.errno} ({e.strerror}) — check the file is not open in "
                f"another program (Excel? antivirus?)"
            ) from e

    # ---------------- core API ----------------
    def append(self, s: Sighting) -> int:
        """Append one row; flush + fsync; return the 1-based sighting_id.
        Thread-safe. A crash loses at most THIS row.

        A write/flush/fsync failure POISONS this instance: the bytes may or
        may not have reached the file, so 'id == row order' can no longer be
        guaranteed through this handle — silently continuing would hand out
        ids that point at the wrong rows after a reload. Recovery is to
        construct a new SightingLog on the same path: reopen truncates any
        torn tail and recomputes ids from what is actually on disk."""
        row = [_encode_cell(n, getattr(s, n), self._hints[n])
               for n in self._fieldnames]        # encode OUTSIDE the lock
        with self._lock:
            if self._closed:
                raise SightingLogError(
                    f"append after close(): {self.csv_path} — check shutdown "
                    f"ordering"
                )
            if self._failed:
                raise SightingLogError(
                    f"append to {self.csv_path} refused: a previous append "
                    f"failed, so the on-disk row order is indeterminate and "
                    f"ids can no longer be trusted through this handle — "
                    f"recreate SightingLog({os.path.basename(self.csv_path)!r}) "
                    f"to recover (reopen truncates any torn tail and "
                    f"recomputes ids from disk)"
                )
            try:
                self._writer.writerow(row)
                self._f.flush()
                os.fsync(self._f.fileno())       # os.fsync looked up per call
            except OSError as e:
                self._failed = True              # poison: loud beats id drift
                try:
                    self._f.close()
                except OSError:
                    pass    # cleanup-of-cleanup; the real error is raised below
                raise SightingLogError(
                    f"sighting append failed (drone {s.drone_id!r}, source "
                    f"{s.source!r}) to {self.csv_path} — errno {e.errno} "
                    f"({e.strerror}) — check disk space / file locked by "
                    f"antivirus or Excel. This SightingLog is now poisoned "
                    f"(on-disk state indeterminate); recreate it on the same "
                    f"path to recover with correct ids"
                ) from e
            self._rows.append(s)
            return len(self._rows)               # 1-based id == row order

    def snapshot(self) -> List[Sighting]:
        """All rows appended or recovered so far (Sighting is frozen, so the
        shallow copy is safe to hand out)."""
        with self._lock:
            return list(self._rows)

    def close(self) -> None:
        """Idempotent. Tests (and Windows tmp-dir cleanup) need closed handles."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            try:
                self._f.close()
            except OSError:
                print(f"[SightingLog] WARNING: close failed for {self.csv_path}",
                      file=sys.stderr, flush=True)

    def __enter__(self) -> "SightingLog":
        return self

    def __exit__(self, *exc) -> bool:
        self.close()
        return False


# ============================================================
# SightingBus
# ============================================================
class SightingBus:
    """Bounded thread-safe fan-in: detector callbacks .publish() from ANY
    thread; the orchestrator polls .drain_after()/.drain_since()/.latest()
    each tick. Eviction is deque(maxlen)'s job — oldest sightings fall off
    at maxlen. All reads are NON-destructive.

    Cursor protocol (binding for the orchestrator): use .drain_after(seq).
    Publish order (seq) is assigned under the bus lock, so a seq cursor can
    NEVER miss a sighting. A ts cursor would be LOSSY by construction:
    Sighting.ts is stamped at frame capture on worker threads, so a slow
    detector (YOLO inference) publishes an OLDER ts after a faster one
    (ArUco on the same frame) — advancing a ts cursor past it would hide it
    forever. drain_since() exists for ad-hoc time-window queries only."""

    def __init__(self, maxlen: int = 500):
        self._dq: "Deque[Tuple[int, Sighting]]" = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._next_seq = 1

    def publish(self, s: Sighting) -> int:
        """Append from any thread. Returns this sighting's publish seq."""
        with self._lock:
            seq = self._next_seq
            self._next_seq += 1
            self._dq.append((seq, s))
            return seq

    def drain_after(self, seq: int,
                    drone_id: Optional[str] = None) -> "Tuple[int, List[Sighting]]":
        """Lossless non-destructive cursor read: every sighting published
        after `seq` (optionally one drone's), plus the new cursor value.
        Start with seq=0; feed each returned cursor into the next call.
        Bounded-buffer caveat: sightings evicted at maxlen before being
        drained are gone — size maxlen for tick cadence, not minutes."""
        with self._lock:
            out = [s for q, s in self._dq
                   if q > seq and (drone_id is None or s.drone_id == drone_id)]
            newest = self._dq[-1][0] if self._dq else seq
        return newest, out

    def drain_since(self, ts: float, drone_id: Optional[str] = None) -> List[Sighting]:
        """NON-destructive read of sightings with s.ts STRICTLY greater than
        `ts` (optionally filtered by drone). For ad-hoc time-window queries
        ('what arrived in the last 2 s?') — NOT for cursoring: a ts cursor
        silently loses out-of-order publishes (see class docstring); cursor
        consumers use drain_after()."""
        with self._lock:
            return [s for _q, s in self._dq
                    if s.ts > ts and (drone_id is None or s.drone_id == drone_id)]

    def latest(self, drone_id: str, source: Optional[str] = None) -> Optional[Sighting]:
        """Most recently published sighting for a drone (optionally from one
        detector source), or None."""
        with self._lock:
            for _q, s in reversed(self._dq):     # bounded by maxlen
                if s.drone_id == drone_id and (source is None or s.source == source):
                    return s
        return None


# ============================================================
# Manual smoke demo
# ============================================================
if __name__ == "__main__":
    import time

    from finals.events import create_run_dir
    from finals.types import PositionQuality

    demo_dir = create_run_dir("./runs_finals")
    demo_csv = os.path.join(demo_dir, "sightings.csv")
    now = time.time()

    demo_sightings = [
        Sighting(drone_id="alpha", ts=now, source="aruco",
                 class_name="aruco_17", marker_id=17,
                 bbox_xyxy=(120.0, 88.5, 240.25, 198.0), confidence=1.0,
                 frame_shape=(480, 640), frame_number=101,
                 drone_yaw_deg=45.0, drone_alt_m=1.2),
        Sighting(drone_id="alpha", ts=now + 0.5, source="aruco",
                 class_name="aruco_23", marker_id=23,
                 bbox_xyxy=(10.0, 20.0, 60.0, 70.0), confidence=1.0,
                 frame_shape=(480, 640), frame_number=106,
                 pos_quality=PositionQuality.MEASURED,
                 est_north_m=12.5, est_east_m=-3.75),
        Sighting(drone_id="bravo", ts=now + 1.0, source="yolo",
                 class_name="robomaster", marker_id=None,
                 bbox_xyxy=(300.5, 200.0, 420.0, 310.5), confidence=0.62,
                 frame_shape=(480, 640), frame_number=53,
                 drone_yaw_deg=-90.0, drone_alt_m=1.7, bearing_deg=271.5),
        Sighting(drone_id="bravo", ts=now + 1.0 / 3.0, source="aruco",
                 class_name="aruco_17", marker_id=17,
                 bbox_xyxy=(0.0, 0.0, 32.0, 32.0), confidence=1.0,
                 frame_shape=(480, 640)),
        Sighting(drone_id="charlie", ts=now + 2.0, source="aruco",
                 class_name="aruco_42", marker_id=42,
                 bbox_xyxy=(55.5, 66.25, 77.0, 88.125), confidence=1.0,
                 frame_shape=(480, 640), frame_number=7,
                 frame_path=os.path.join(demo_dir, "frames", "f7.jpg")),
    ]

    with SightingLog(demo_csv) as slog:
        ids = [slog.append(s) for s in demo_sightings]
        assert ids == [1, 2, 3, 4, 5], ids
        assert slog.snapshot() == demo_sightings, "roundtrip mismatch"
    print(f"appended sightings 1..5; in-memory snapshot equals inputs")

    with SightingLog(demo_csv) as slog2:         # reopen: ids continue
        assert slog2.snapshot() == demo_sightings, "reload mismatch"
        sixth = slog2.append(demo_sightings[0])
        assert sixth == 6, f"id continuation broken: expected 6, got {sixth}"
        print(f"reopened log continued ids: next append returned id {sixth}")

    print(f"\nraw {demo_csv}:")
    with open(demo_csv, "r", encoding="utf-8") as f:
        for line in f:
            print(f"  {line.rstrip()}")

    print(f"\nrun dir tree ({demo_dir}):")
    for root, _dirs, files in os.walk(demo_dir):
        for fname in sorted(files):
            fpath = os.path.join(root, fname)
            print(f"  {fname:<24} {os.path.getsize(fpath):>6} bytes")
