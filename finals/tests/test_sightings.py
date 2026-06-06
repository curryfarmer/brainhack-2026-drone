"""finals.sightings — append+fsync CSV SightingLog (kill-test reload, exact
roundtrip, id continuation) and the thread-safe SightingBus."""
from __future__ import annotations

import csv
import dataclasses
import os
import threading
import time

import pytest

from finals.sightings import SightingBus, SightingLog, SightingLogError
from finals.types import PositionQuality, Sighting


# ============================================================
# Fixture (module-local — conftest.py untouched)
# ============================================================
@pytest.fixture
def make_sighting():
    """Factory with sensible defaults; override any field by keyword."""
    def _make(**over) -> Sighting:
        base = dict(
            drone_id="alpha", ts=1234.5, source="aruco", class_name="aruco_17",
            marker_id=17, bbox_xyxy=(10.0, 20.0, 110.5, 220.25), confidence=1.0,
            frame_shape=(480, 640), frame_number=7, drone_yaw_deg=90.0,
            drone_alt_m=1.2, bearing_deg=None, pos_quality=PositionQuality.NONE,
            est_north_m=None, est_east_m=None, frame_path=None,
        )
        base.update(over)
        return Sighting(**base)
    return _make


# ============================================================
# SightingLog
# ============================================================
def test_sightinglog_header_matches_dataclass_fields(tmp_path, make_sighting):
    path = str(tmp_path / "s.csv")
    with SightingLog(path) as log:
        log.append(make_sighting())
    with open(path, "r", encoding="utf-8", newline="") as f:
        header = next(csv.reader(f))
    # Pins column order AND the absence of a hand-maintained id column.
    assert header == [f.name for f in dataclasses.fields(Sighting)]


def test_sightinglog_roundtrip_varied_values(tmp_path, make_sighting):
    inputs = [
        make_sighting(),                                              # baseline, some Nones
        make_sighting(drone_id="bravo", source="yolo", class_name="robomaster",
                      marker_id=None, confidence=0.4375,
                      bbox_xyxy=(-1.5, 0.0, 3.25, 4.125),             # negative coords
                      frame_number=None, drone_yaw_deg=-179.99),
        make_sighting(ts=0.1 + 0.2,                                   # 0.30000000000000004
                      confidence=1 / 3, bearing_deg=123.456789012345,
                      pos_quality=PositionQuality.MEASURED,
                      est_north_m=-12.3456789, est_east_m=1e-9),
        make_sighting(frame_path="runs/2026-06-06/frames/a b,c.jpg"), # comma: CSV quoting
        make_sighting(class_name='convoy_Δ17"q'),                     # utf-8 + embedded quote
    ]
    path = str(tmp_path / "s.csv")
    with SightingLog(path) as log:
        assert [log.append(s) for s in inputs] == [1, 2, 3, 4, 5]     # 1-based ids
        assert log.snapshot() == inputs            # in-memory mirror (same objects)

    # The REAL roundtrip: reopen so every value passes through the CSV
    # DECODE path (quoting, float repr, utf-8 read, enum reconstruction) —
    # an in-process snapshot alone would compare the inputs to themselves.
    with SightingLog(path) as log2:
        snap = log2.snapshot()
    assert snap == inputs                                             # exact equality
    # IntEnum == int would mask a raw-int decode; pin the real types
    # ON THE DECODED objects.
    assert all(isinstance(s.pos_quality, PositionQuality) for s in snap)
    assert all(isinstance(s.bbox_xyxy, tuple) for s in snap)
    assert all(isinstance(s.frame_shape[0], int) for s in snap)
    assert all(isinstance(s.ts, float) for s in snap)


def test_sightinglog_fsync_called_once_per_append(tmp_path, make_sighting, monkeypatch):
    real_fsync = os.fsync
    calls: list = []

    def counting_fsync(fd: int) -> None:
        calls.append(fd)
        real_fsync(fd)            # still really fsync — masks no closed-fd bug

    monkeypatch.setattr(os, "fsync", counting_fsync)
    with SightingLog(str(tmp_path / "s.csv")) as log:
        baseline = len(calls)     # header write may fsync; delta is what matters
        for i in range(3):
            log.append(make_sighting(frame_number=i))
            assert len(calls) == baseline + i + 1   # exactly one fsync per row


def test_sightinglog_reopen_continues_ids_and_keeps_one_header(tmp_path, make_sighting):
    path = str(tmp_path / "s.csv")
    first = [make_sighting(frame_number=i) for i in range(3)]
    with SightingLog(path) as log:
        for s in first:
            log.append(s)

    with SightingLog(path) as log2:
        assert log2.snapshot() == first
        s4 = make_sighting(frame_number=99)
        assert log2.append(s4) == 4               # ids continue across reopen

    with SightingLog(path) as log3:
        assert log3.snapshot() == first + [s4]

    with open(path, "r", encoding="utf-8") as f:
        lines = f.read().splitlines()
    assert lines.count(lines[0]) == 1             # header written exactly once


def test_sightinglog_torn_last_row_recovery(tmp_path, make_sighting, capsys):
    path = str(tmp_path / "s.csv")
    first = [make_sighting(frame_number=i) for i in range(3)]
    with SightingLog(path) as log:
        for s in first:
            log.append(s)
    with open(path, "a", encoding="utf-8", newline="") as f:
        f.write("alpha,123.45,aruco")             # torn row, NO trailing newline

    with SightingLog(path) as log2:               # reopen = kill-test reload
        snap = log2.snapshot()
        err = capsys.readouterr().err
        assert "skipped 1 torn trailing row" in err   # loud COUNTED warning
        assert snap == first                      # prior rows intact
        s4 = make_sighting(frame_number=99)
        assert log2.append(s4) == 4               # torn row consumed no id

    with SightingLog(path) as log3:               # appends after recovery parse
        assert log3.snapshot() == first + [s4]


def test_sightinglog_torn_terminated_last_row_recovery(tmp_path, make_sighting, capsys):
    """Torn variant (b): the last row is newline-TERMINATED but unparseable
    (wrong cell count). Truncation here uses the per-line byte-offset
    arithmetic — stressed with a non-ASCII (multi-byte utf-8) row earlier in
    the file, where a chars-vs-bytes bug would land the cut mid-row."""
    path = str(tmp_path / "s.csv")
    first = [
        make_sighting(class_name="convoy_Δ17", frame_number=0),   # multi-byte row
        make_sighting(frame_number=1),
    ]
    with SightingLog(path) as log:
        for s in first:
            log.append(s)
    with open(path, "a", encoding="utf-8", newline="") as f:
        f.write("alpha,123\n")                    # terminated, wrong cell count

    with SightingLog(path) as log2:
        err = capsys.readouterr().err
        assert "skipped 1 torn trailing row" in err
        assert log2.snapshot() == first           # prior rows byte-exact intact
        s3 = make_sighting(frame_number=99)
        assert log2.append(s3) == 3               # torn row consumed no id

    with SightingLog(path) as log3:               # post-recovery file is clean
        assert log3.snapshot() == first + [s3]


def test_sightinglog_header_mismatch_raises(tmp_path):
    path = str(tmp_path / "s.csv")
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write("old_col_a,old_col_b\n")          # a previous Sighting shape
    with pytest.raises(SightingLogError, match="header mismatch"):
        SightingLog(path)


def test_sightinglog_midfile_corruption_raises(tmp_path, make_sighting):
    """A corrupt MID-file row must refuse loudly, never skip — skipping
    would shift the 1-based ids already handed out."""
    path = str(tmp_path / "s.csv")
    with SightingLog(path) as log:
        for i in range(3):
            log.append(make_sighting(frame_number=i))
    with open(path, "r", encoding="utf-8") as f:
        lines = f.read().splitlines(keepends=True)
    lines.insert(2, "garbage,row\n")              # between row 1 and row 2
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.writelines(lines)
    with pytest.raises(SightingLogError, match="unparseable mid-file row"):
        SightingLog(path)


def test_sightinglog_invalid_utf8_raises_as_tampering(tmp_path, make_sighting):
    """Invalid bytes in a TERMINATED line can never come from this writer
    (strict utf-8) — recovery must flag tampering, not load a mangled row."""
    path = str(tmp_path / "s.csv")
    with SightingLog(path) as log:
        for i in range(2):
            log.append(make_sighting(frame_number=i))
        log.append(make_sighting(frame_number=2))   # one row after the victim
    with open(path, "r+b") as f:
        data = f.read()
        f.seek(0)
        f.write(data.replace(b"alpha", b"alp\xffa", 1))   # corrupt one byte
    with pytest.raises(SightingLogError, match="invalid UTF-8"):
        SightingLog(path)


def test_encode_guards_reject_invalid_sightings(tmp_path, make_sighting):
    """The loud-validation contract: an invalid Sighting raises AT APPEND
    TIME (frozen dataclasses don't validate), before anything hits disk."""
    with SightingLog(str(tmp_path / "s.csv")) as log:
        # Wrong tuple arity — zip-truncation would brick the reload later.
        with pytest.raises(SightingLogError, match="requires exactly 4"):
            log.append(make_sighting(bbox_xyxy=(1.0, 2.0)))
        with pytest.raises(SightingLogError, match="requires exactly 2"):
            log.append(make_sighting(frame_shape=(480, 640, 3)))  # raw HxWxC shape
        with pytest.raises(SightingLogError, match="requires exactly 4"):
            log.append(make_sighting(bbox_xyxy=(1.0, 2.0, 3.0, 4.0, 5.0)))
        # None in a non-Optional field.
        with pytest.raises(SightingLogError, match="not Optional"):
            log.append(make_sighting(confidence=None))
        # Embedded newline would break line-oriented recovery.
        with pytest.raises(SightingLogError, match="newline"):
            log.append(make_sighting(class_name="evil\nrow"))
        # '' for Optional[str] would silently reload as None.
        with pytest.raises(SightingLogError, match="indistinguishable from None"):
            log.append(make_sighting(frame_path=""))
        # Nothing reached the file; a valid append still gets id 1.
        assert log.append(make_sighting()) == 1
    with SightingLog(str(tmp_path / "s.csv")) as log2:
        assert len(log2.snapshot()) == 1          # disk agrees: one clean row


def test_sightinglog_append_after_close_raises(tmp_path, make_sighting):
    log = SightingLog(str(tmp_path / "s.csv"))
    log.append(make_sighting())
    log.close()
    log.close()                                   # idempotent
    with pytest.raises(SightingLogError, match="after close"):
        log.append(make_sighting())


def test_sightinglog_failed_append_poisons_then_recovers(tmp_path, make_sighting, monkeypatch):
    """A flush/fsync failure leaves the on-disk row order indeterminate, so
    the instance must POISON itself (loud refusal) instead of silently
    handing out ids that desync from row order. A fresh SightingLog on the
    same path recovers with ids that match what is actually on disk."""
    path = str(tmp_path / "s.csv")
    log = SightingLog(path)
    try:
        assert log.append(make_sighting(frame_number=0)) == 1

        real_fsync = os.fsync
        boom = {"armed": True}

        def failing_fsync(fd):
            if boom["armed"]:
                boom["armed"] = False
                raise OSError(28, "No space left on device")
            real_fsync(fd)

        monkeypatch.setattr(os, "fsync", failing_fsync)
        with pytest.raises(SightingLogError, match="errno 28"):
            log.append(make_sighting(frame_number=1))
        # Poisoned: refuses further appends with a recovery hint.
        with pytest.raises(SightingLogError, match="recreate"):
            log.append(make_sighting(frame_number=2))
        assert len(log.snapshot()) == 1           # mirror never lied
    finally:
        log.close()

    with SightingLog(path) as log2:               # recovery: ids match disk
        n_recovered = len(log2.snapshot())
        next_id = log2.append(make_sighting(frame_number=99))
        assert next_id == n_recovered + 1
    with SightingLog(path) as log3:               # and the file stays clean
        assert len(log3.snapshot()) == n_recovered + 1


def test_sightinglog_threaded_appends_unique_ids(tmp_path, make_sighting):
    n_threads, n_each = 4, 50
    barrier = threading.Barrier(n_threads)
    errors: list = []
    ids: list = []                                # list.append is GIL-atomic

    with SightingLog(str(tmp_path / "s.csv")) as log:
        def worker(tid: int) -> None:
            try:
                barrier.wait(timeout=10.0)
                for i in range(n_each):
                    ids.append(log.append(make_sighting(frame_number=tid * 1000 + i)))
            except Exception as exc:              # collected + re-asserted
                errors.append(exc)

        threads = [threading.Thread(target=worker, args=(t,), daemon=True)
                   for t in range(n_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=30.0)
        assert not any(t.is_alive() for t in threads), "appender thread deadlocked"
        assert errors == []
        assert sorted(ids) == list(range(1, n_threads * n_each + 1))  # no dup/skip
        snap = log.snapshot()
    assert len(snap) == n_threads * n_each
    assert {s.frame_number for s in snap} == {
        t * 1000 + i for t in range(n_threads) for i in range(n_each)
    }


# ============================================================
# SightingBus
# ============================================================
def test_bus_threaded_publish_then_drain_gets_all(make_sighting):
    bus = SightingBus(maxlen=1000)
    n_threads, n_each = 4, 100
    barrier = threading.Barrier(n_threads)
    errors: list = []

    def worker(tid: int) -> None:
        try:
            barrier.wait(timeout=10.0)
            for i in range(n_each):
                key = tid * 1000 + i
                bus.publish(make_sighting(ts=1.0 + key, frame_number=key))
        except Exception as exc:
            errors.append(exc)

    threads = [threading.Thread(target=worker, args=(t,), daemon=True)
               for t in range(n_threads)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=30.0)
    assert not any(t.is_alive() for t in threads)
    assert errors == []

    got = bus.drain_since(0.0)
    assert len(got) == n_threads * n_each
    assert {s.frame_number for s in got} == {
        t * 1000 + i for t in range(n_threads) for i in range(n_each)
    }
    assert len(bus.drain_since(0.0)) == n_threads * n_each  # NON-destructive


def test_bus_reads_race_concurrent_publishes(make_sighting):
    """The race the lock actually protects: drain/latest ITERATE the deque
    while detector threads publish — without the lock this raises
    'RuntimeError: deque mutated during iteration' within a second."""
    bus = SightingBus(maxlen=10_000)
    n_threads, n_each = 4, 500
    start = threading.Barrier(n_threads + 1)      # +1: the reading main thread
    errors: list = []

    def publisher(tid: int) -> None:
        try:
            start.wait(timeout=10.0)
            for i in range(n_each):
                bus.publish(make_sighting(drone_id=f"d{tid}",
                                          ts=1.0 + tid * 1000 + i))
        except Exception as exc:
            errors.append(exc)

    threads = [threading.Thread(target=publisher, args=(t,), daemon=True)
               for t in range(n_threads)]
    for t in threads:
        t.start()
    start.wait(timeout=10.0)
    deadline = time.monotonic() + 30.0
    cursor, total_seen = 0, 0
    while any(t.is_alive() for t in threads):     # read INSIDE the publish window
        assert time.monotonic() < deadline, "publishers wedged"
        cursor, fresh = bus.drain_after(cursor)   # iterates under contention
        total_seen += len(fresh)
        bus.drain_since(0.0)
        bus.latest("d0")
    for t in threads:
        t.join(timeout=10.0)
    assert errors == []
    cursor, fresh = bus.drain_after(cursor)       # final sweep
    total_seen += len(fresh)
    assert total_seen == n_threads * n_each       # lossless cursor: every publish seen


def test_bus_all_methods_share_one_mutex(make_sighting):
    """Deterministic mutual-exclusion pin. The concurrent race test above is
    probabilistic (a removed lock survives lucky interleavings); this one
    kills it on EVERY run: while the bus's lock is held, every method must
    block, and must complete once it is released."""
    bus = SightingBus()
    bus.publish(make_sighting(ts=1.0))

    assert bus._lock.acquire(timeout=5.0)         # a no-op lock dies right here
    results: list = []
    worker = threading.Thread(
        target=lambda: (bus.publish(make_sighting(ts=2.0)),
                        bus.drain_after(0),
                        bus.drain_since(0.0),
                        bus.latest("alpha"),
                        results.append("done")),
        daemon=True)
    try:
        worker.start()
        worker.join(timeout=0.25)
        # Lock held => the worker must still be stuck on its FIRST bus call.
        assert not results, "bus method ran while the lock was held — no mutual exclusion"
    finally:
        bus._lock.release()
    worker.join(timeout=10.0)
    assert results == ["done"]                    # and it completes after release


def test_bus_drain_after_is_lossless_for_out_of_order_publishes(make_sighting):
    """The exact race a ts cursor loses: a slow detector publishes an OLDER
    frame-ts AFTER a faster one. The seq cursor must still deliver it."""
    bus = SightingBus()
    bus.publish(make_sighting(drone_id="alpha", source="aruco", ts=100.05))
    cursor, got = bus.drain_after(0)
    assert [s.ts for s in got] == [100.05]

    # YOLO finishes late and publishes the SAME frame's older ts afterwards.
    bus.publish(make_sighting(drone_id="alpha", source="yolo", ts=100.00))
    cursor, got = bus.drain_after(cursor)
    assert [s.ts for s in got] == [100.00]        # NOT lost (a ts cursor loses it)

    _cursor, got = bus.drain_after(cursor)
    assert got == []                              # cursor is stable when idle


def test_bus_drain_after_filters_drone_and_publish_returns_seq(make_sighting):
    bus = SightingBus()
    assert bus.publish(make_sighting(drone_id="alpha", ts=1.0)) == 1
    assert bus.publish(make_sighting(drone_id="bravo", ts=2.0)) == 2
    assert bus.publish(make_sighting(drone_id="alpha", ts=3.0)) == 3

    cursor, got = bus.drain_after(0, drone_id="alpha")
    assert [s.ts for s in got] == [1.0, 3.0]
    assert cursor == 3                            # advances past filtered-out items
    _cursor, got = bus.drain_after(cursor, drone_id="alpha")
    assert got == []


def test_bus_drain_since_is_strictly_greater_and_filters_drone(make_sighting):
    bus = SightingBus()
    bus.publish(make_sighting(drone_id="alpha", ts=1.0))
    bus.publish(make_sighting(drone_id="alpha", ts=2.0))
    bus.publish(make_sighting(drone_id="bravo", ts=2.5))
    bus.publish(make_sighting(drone_id="alpha", ts=3.0))

    assert [s.ts for s in bus.drain_since(2.0)] == [2.5, 3.0]   # ts==2.0 excluded
    assert [s.ts for s in bus.drain_since(2.0, drone_id="alpha")] == [3.0]
    assert bus.drain_since(99.0) == []


def test_bus_latest_filters_drone_and_source(make_sighting):
    bus = SightingBus()
    bus.publish(make_sighting(drone_id="alpha", source="yolo", ts=1.0))
    bus.publish(make_sighting(drone_id="alpha", source="aruco", ts=2.0))
    bus.publish(make_sighting(drone_id="bravo", source="aruco", ts=3.0))

    assert bus.latest("alpha").ts == 2.0
    assert bus.latest("alpha", source="yolo").ts == 1.0
    assert bus.latest("bravo").ts == 3.0
    assert bus.latest("charlie") is None
    assert bus.latest("alpha", source="nope") is None


def test_bus_maxlen_evicts_oldest(make_sighting):
    bus = SightingBus(maxlen=10)
    for i in range(13):
        drone = "old" if i < 3 else "new"
        bus.publish(make_sighting(drone_id=drone, ts=1.0 + i))

    remaining = sorted(s.ts for s in bus.drain_since(0.0))
    assert remaining == [4.0 + i for i in range(10)]   # oldest 3 evicted
    assert bus.latest("old") is None                   # evicted drone gone
    assert bus.latest("new").ts == 13.0


def test_bus_default_maxlen_is_500(make_sighting):
    bus = SightingBus()
    for i in range(501):
        bus.publish(make_sighting(ts=1.0 + i))
    got = bus.drain_since(0.0)
    assert len(got) == 500
    assert min(s.ts for s in got) == 2.0               # the very first evicted
