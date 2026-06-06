"""finals.events — JSONL event log, atomic heartbeat, crash hooks, kill-test
reload. Crash-safety is the feature: these tests simulate the process dying
at the worst possible byte and a reader racing the heartbeat writer."""
from __future__ import annotations

import faulthandler
import json
import os
import subprocess
import sys
import threading
import time

import pytest

from finals.events import (EventLog, EventLogError, create_run_dir,
                           install_crash_hooks, read_events, write_heartbeat)


# ============================================================
# Fixtures / helpers (module-local — conftest.py untouched)
# ============================================================
@pytest.fixture
def run_dir(tmp_path) -> str:
    """An EXISTING dir for EventLog (creating it is create_run_dir's job)."""
    d = tmp_path / "run"
    d.mkdir()
    return str(d)


@pytest.fixture
def excepthook_guard():
    """Save/restore sys.excepthook + faulthandler state around hook tests,
    so a test failure can never leave the suite with a hijacked hook or a
    faulthandler pointing at a dead fd.

    Restore target: sys.__stderr__ (the real fd-2 stream), NOT bare
    enable() — under pytest capture, bare enable() would bind faulthandler
    to the per-test capture TemporaryFile, whose fd dies with the test, so
    a later hard fault would dump into a deleted file. fd 2 always lives.
    (Known limitation: pytest's own faulthandler plugin registered a private
    console dup we cannot reach via public API; dumps after these tests land
    on fd 2, which pytest may still capture — but never on a dead fd.)"""
    orig_hook = sys.excepthook
    fh_was_enabled = faulthandler.is_enabled()
    yield
    sys.excepthook = orig_hook
    if fh_was_enabled:
        faulthandler.enable(file=sys.__stderr__ or sys.stderr)
    else:
        faulthandler.disable()


def _read_jsonl(path: str) -> list:
    """STRICT reader: any unparseable line fails the test. Used wherever
    zero corruption is the assertion."""
    with open(path, "r", encoding="utf-8") as f:
        return [json.loads(line) for line in f.read().splitlines()]


# ============================================================
# create_run_dir
# ============================================================
def test_create_run_dir_creates_dir_and_prints_banner(tmp_path, capsys):
    p = create_run_dir(str(tmp_path))
    assert isinstance(p, str)
    assert os.path.isdir(p)
    assert os.path.dirname(p) == str(tmp_path)
    captured = capsys.readouterr()
    assert os.path.basename(p) in (captured.out + captured.err)  # loud banner
    assert p in (captured.out + captured.err)                    # absolute path shown


def test_create_run_dir_collision_suffix_is_deterministic(tmp_path, monkeypatch):
    # Pin the clock so the collision branch is exercised on EVERY run, not
    # just when both calls land in the same wall-clock second.
    monkeypatch.setattr(time, "strftime", lambda fmt: "20990101_120000")
    p1 = create_run_dir(str(tmp_path))
    p2 = create_run_dir(str(tmp_path))
    p3 = create_run_dir(str(tmp_path))
    assert os.path.basename(p1) == "20990101_120000"
    assert os.path.basename(p2) == "20990101_120000_01"   # the _NN suffix branch
    assert os.path.basename(p3) == "20990101_120000_02"
    assert all(os.path.isdir(p) for p in (p1, p2, p3))


def test_create_run_dir_bounded_exhaustion_raises(tmp_path, monkeypatch):
    monkeypatch.setattr(time, "strftime", lambda fmt: "20990101_120000")
    (tmp_path / "20990101_120000").mkdir()
    for n in range(1, 100):
        (tmp_path / f"20990101_120000_{n:02d}").mkdir()   # all 100 names taken
    with pytest.raises(EventLogError, match="runaway restart loop"):
        create_run_dir(str(tmp_path))


# ============================================================
# EventLog basics
# ============================================================
def test_eventlog_line_schema_and_clock_fields(run_dir):
    t0_wall, t0_mono = time.time(), time.monotonic()
    with EventLog(run_dir) as log:
        log.log("alpha", "takeoff", height_cm=80, note="convoy_Δ17",
                nested={"a": [1, 2]})
    t1_wall, t1_mono = time.time(), time.monotonic()

    recs = _read_jsonl(os.path.join(run_dir, "mission.jsonl"))
    assert len(recs) == 1
    rec = recs[0]
    assert set(rec) == {"ts", "mono", "drone", "event", "data"}  # exact schema
    assert rec["drone"] == "alpha" and rec["event"] == "takeoff"
    assert rec["data"] == {"height_cm": 80, "note": "convoy_Δ17",
                           "nested": {"a": [1, 2]}}
    assert t0_wall <= rec["ts"] <= t1_wall      # wall clock
    assert t0_mono <= rec["mono"] <= t1_mono    # monotonic clock

    # Pin the FILE encoding, not just the JSON value roundtrip: the raw
    # utf-8 bytes of Δ must be on disk (ensure_ascii=False + encoding="utf-8");
    # a cp1252 regression would corrupt or crash this.
    with open(os.path.join(run_dir, "mission.jsonl"), "rb") as f:
        assert "convoy_Δ17".encode("utf-8") in f.read()


def test_eventlog_per_drone_routing(run_dir):
    with EventLog(run_dir) as log:
        log.log("alpha", "e1")
        log.log("bravo", "e2")
        log.log("alpha", "e3")
        log.log("bravo", "e4")
        log.log("alpha", "e5")

    mission = _read_jsonl(os.path.join(run_dir, "mission.jsonl"))
    assert [r["event"] for r in mission] == ["e1", "e2", "e3", "e4", "e5"]

    alpha = _read_jsonl(os.path.join(run_dir, "drone_alpha.jsonl"))
    assert [r["event"] for r in alpha] == ["e1", "e3", "e5"]
    assert all(r["drone"] == "alpha" for r in alpha)

    bravo = _read_jsonl(os.path.join(run_dir, "drone_bravo.jsonl"))
    assert [r["event"] for r in bravo] == ["e2", "e4"]
    assert all(r["drone"] == "bravo" for r in bravo)

    assert not os.path.exists(os.path.join(run_dir, "drone_charlie.jsonl"))


def test_eventlog_sanitizes_path_illegal_drone_ids(run_dir):
    """':' would create an NTFS alternate data stream, '/' a subpath, ''
    no filename at all — the per-drone FILENAME degrades safely while
    mission.jsonl keeps the raw id for forensics."""
    with EventLog(run_dir) as log:
        log.log("a:b", "tick", i=1)
        log.log("x/y", "tick", i=2)
        log.log("", "tick", i=3)

    assert _read_jsonl(os.path.join(run_dir, "drone_a_b.jsonl"))[0]["drone"] == "a:b"
    assert _read_jsonl(os.path.join(run_dir, "drone_x_y.jsonl"))[0]["drone"] == "x/y"
    assert _read_jsonl(os.path.join(run_dir, "drone_unknown.jsonl"))[0]["drone"] == ""
    mission = _read_jsonl(os.path.join(run_dir, "mission.jsonl"))
    assert [r["drone"] for r in mission] == ["a:b", "x/y", ""]   # raw ids kept
    # No stray sanitization artifacts (e.g. a 0-byte 'drone_a' ADS host file).
    assert sorted(n for n in os.listdir(run_dir) if n.startswith("drone_")) == [
        "drone_a_b.jsonl", "drone_unknown.jsonl", "drone_x_y.jsonl"]


def test_eventlog_write_failure_raises_typed_error(run_dir):
    log = EventLog(run_dir)
    try:
        log.log("alpha", "tick", i=0)            # opens handles, proves health

        class _DeadFile:
            name = "mission.jsonl (dead stub)"

            def write(self, _line):
                raise OSError(28, "No space left on device")

            def flush(self):
                pass

            def close(self):
                pass

        log._mission = _DeadFile()               # simulate the disk dying
        with pytest.raises(EventLogError, match="errno 28") as exc_info:
            log.log("alpha", "tick", i=1)
        msg = str(exc_info.value)
        assert "alpha" in msg and "disk space" in msg   # actionable message
    finally:
        log.close()


def test_eventlog_context_manager_and_idempotent_close(run_dir):
    with EventLog(run_dir) as log:
        assert isinstance(log, EventLog)
        log.log("alpha", "tick")
    log.close()                                  # second close: no raise
    with pytest.raises(EventLogError, match="after close"):
        log.log("alpha", "too_late")             # loud, never silently dropped


# ============================================================
# Threaded stress (8 threads x 200 events)
# ============================================================
def test_eventlog_threaded_stress_1600_events(run_dir):
    n_threads, n_events = 8, 200
    barrier = threading.Barrier(n_threads)
    errors: list = []

    with EventLog(run_dir) as log:
        def worker(tid: int) -> None:
            try:
                barrier.wait(timeout=10.0)       # all release together: max contention
                for seq in range(n_events):
                    log.log(f"d{tid}", "tick", tid=tid, seq=seq)
            except Exception as exc:             # collected + re-asserted below
                errors.append(exc)

        threads = [threading.Thread(target=worker, args=(i,), daemon=True)
                   for i in range(n_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=30.0)
        assert not any(t.is_alive() for t in threads), "writer thread deadlocked"
    assert errors == []

    recs = _read_jsonl(os.path.join(run_dir, "mission.jsonl"))  # strict: 0 corrupt
    assert len(recs) == n_threads * n_events
    assert {(r["drone"], r["data"]["seq"]) for r in recs} == {
        (f"d{i}", s) for i in range(n_threads) for s in range(n_events)
    }
    for tid in range(n_threads):
        per = _read_jsonl(os.path.join(run_dir, f"drone_d{tid}.jsonl"))
        # One thread per drone id + internal lock => program order preserved.
        assert [r["data"]["seq"] for r in per] == list(range(n_events))
        assert all(r["drone"] == f"d{tid}" for r in per)


# ============================================================
# Kill simulation / read_events
# ============================================================
def test_read_events_skips_torn_trailing_line_with_counted_warning(run_dir, capsys):
    with EventLog(run_dir) as log:
        for i in range(5):
            log.log("alpha", "tick", i=i)
    mission = os.path.join(run_dir, "mission.jsonl")
    with open(mission, "a", encoding="utf-8", newline="") as f:
        f.write('{"ts": 1.0, "mono": 2.0, "dro')   # torn mid-write, no newline

    recovered = list(read_events(mission))
    err = capsys.readouterr().err
    assert [r["data"]["i"] for r in recovered] == [0, 1, 2, 3, 4]  # prior data intact
    assert "skipped torn trailing line" in err          # the per-line warning
    assert "skipped 1 bad line(s)" in err               # the COUNTED summary


def test_read_events_skips_unparseable_middle_line(run_dir, capsys):
    with EventLog(run_dir) as log:
        for i in range(4):
            log.log("alpha", "tick", i=i)
    mission = os.path.join(run_dir, "mission.jsonl")
    with open(mission, "r", encoding="utf-8") as f:
        lines = f.read().splitlines(keepends=True)
    lines.insert(2, "not json at all\n")            # corrupt MIDDLE line
    with open(mission, "w", encoding="utf-8", newline="") as f:
        f.writelines(lines)

    recovered = list(read_events(mission))
    err = capsys.readouterr().err
    assert [r["data"]["i"] for r in recovered] == [0, 1, 2, 3]
    assert "skipped unparseable line" in err            # the per-line warning
    assert "skipped 1 bad line(s)" in err               # the COUNTED summary


def test_read_events_skips_parseable_but_unterminated_tail(run_dir, capsys):
    """The policy under test: an unterminated final line is skipped EVEN IF
    it happens to parse — a missing terminator means the write was cut, so
    completeness cannot be trusted (determinism beats coincidence)."""
    with EventLog(run_dir) as log:
        log.log("alpha", "tick", i=0)
    mission = os.path.join(run_dir, "mission.jsonl")
    complete_but_torn = json.dumps(
        {"ts": 1.0, "mono": 2.0, "drone": "alpha", "event": "tick",
         "data": {"i": 99}})
    assert json.loads(complete_but_torn)                # sanity: it parses
    with open(mission, "a", encoding="utf-8", newline="") as f:
        f.write(complete_but_torn)                      # NO trailing newline

    recovered = list(read_events(mission))
    err = capsys.readouterr().err
    assert [r["data"]["i"] for r in recovered] == [0]   # torn line NOT yielded
    assert "skipped torn trailing line" in err
    assert "skipped 1 bad line(s)" in err


def test_eventlog_reopen_after_torn_tail_stays_parseable(run_dir, capsys):
    with EventLog(run_dir) as log:
        for i in range(5):
            log.log("alpha", "tick", i=i)
    mission = os.path.join(run_dir, "mission.jsonl")
    drone_file = os.path.join(run_dir, "drone_alpha.jsonl")
    for path in (mission, drone_file):              # kill BOTH files mid-line
        with open(path, "a", encoding="utf-8", newline="") as f:
            f.write('{"torn": ')

    with EventLog(run_dir) as log2:                 # reopen the SAME run dir
        for i in range(5, 8):
            log2.log("alpha", "tick", i=i)
    err = capsys.readouterr().err
    # The repair fires for mission.jsonl (eager, at __init__) AND for the
    # per-drone file (lazy, at first log) — both must warn loudly.
    assert err.count("torn unterminated tail") == 2

    for path in (mission, drone_file):              # fragments skipped, not merged
        recovered = list(read_events(path))
        assert [r["data"]["i"] for r in recovered] == list(range(8))


def test_read_events_missing_file_raises_typed_error(tmp_path):
    with pytest.raises(EventLogError, match="check the path"):
        list(read_events(str(tmp_path / "no_such.jsonl")))


# ============================================================
# Heartbeat
# ============================================================
def test_write_heartbeat_writes_json_and_leaves_no_tmp(run_dir):
    snap = {"seq": 0, "drones": {"alpha": {"battery_pct": 87.5}}}
    write_heartbeat(run_dir, snap)
    with open(os.path.join(run_dir, "heartbeat.json"), "r", encoding="utf-8") as f:
        assert json.load(f) == snap
    assert [n for n in os.listdir(run_dir) if n.endswith(".tmp")] == []


def test_write_heartbeat_atomic_under_concurrent_reader(run_dir):
    """Writer loop + reader loop concurrently; the reader must NEVER see
    partial JSON or empty content. On Windows this genuinely exercises the
    os.replace PermissionError retry (reader-held handles deny the rename)."""
    hb = os.path.join(run_dir, "heartbeat.json")
    n_writes = 300
    writer_errors: list = []
    writer_done = threading.Event()

    def writer() -> None:
        try:
            for i in range(n_writes):
                # Size-varying payload makes torn reads detectable.
                write_heartbeat(run_dir, {"seq": i, "pad": "x" * (i % 64)})
        except Exception as exc:                   # collected + re-asserted
            writer_errors.append(exc)
        finally:
            writer_done.set()

    t = threading.Thread(target=writer, daemon=True)
    deadline = time.monotonic() + 30.0
    good_reads = 0
    bad_payloads: list = []
    t.start()
    try:
        while not writer_done.is_set():
            assert time.monotonic() < deadline, "writer did not finish within 30 s"
            try:
                with open(hb, "r", encoding="utf-8") as f:  # close before parse
                    text = f.read()
            except FileNotFoundError:
                continue            # tolerated: before the first replace lands
            except PermissionError:
                continue            # tolerated: transient Windows open denial
            try:
                snap = json.loads(text)
            except json.JSONDecodeError:
                bad_payloads.append(repr(text[:80]))        # atomicity broken
                continue
            assert snap["pad"] == "x" * (snap["seq"] % 64)  # one atomic write
            good_reads += 1
            time.sleep(0.001)       # keep reader duty cycle low enough that the
                                    # writer's bounded retry can never exhaust
    finally:
        t.join(timeout=30.0)
    assert not t.is_alive()
    assert writer_errors == [], f"write_heartbeat raised under contention: {writer_errors!r}"
    assert bad_payloads == [], f"reader observed non-atomic content: {bad_payloads[:3]}"
    assert good_reads > 0
    with open(hb, "r", encoding="utf-8") as f:
        assert json.load(f)["seq"] == n_writes - 1          # last write won
    assert [n for n in os.listdir(run_dir) if n.endswith(".tmp")] == []


def test_write_heartbeat_retry_is_bounded_on_persistent_denial(run_dir, monkeypatch):
    def always_denied(src, dst):
        raise PermissionError("simulated permanent sharing violation")

    monkeypatch.setattr(os, "replace", always_denied)
    start = time.monotonic()
    with pytest.raises(EventLogError, match="sharing-violated"):
        write_heartbeat(run_dir, {"seq": 0})
    assert time.monotonic() - start < 10.0      # bounded retry, not a hang
    assert [n for n in os.listdir(run_dir) if n.endswith(".tmp")] == []


def test_write_heartbeat_other_oserror_raises_and_cleans_tmp(run_dir, monkeypatch):
    """Non-PermissionError OSError must NOT enter the retry loop: it raises
    immediately and the tmp file is removed."""
    def broken_replace(src, dst):
        raise OSError(22, "Invalid argument")

    monkeypatch.setattr(os, "replace", broken_replace)
    start = time.monotonic()
    with pytest.raises(EventLogError, match="errno 22"):
        write_heartbeat(run_dir, {"seq": 0})
    assert time.monotonic() - start < 0.4           # no 0.5 s retry loop entered
    assert [n for n in os.listdir(run_dir) if n.endswith(".tmp")] == []


def test_write_heartbeat_recovers_after_transient_denial(run_dir, monkeypatch):
    real_replace = os.replace
    denials = {"n": 0}

    def flaky(src, dst):
        if denials["n"] < 2:
            denials["n"] += 1
            raise PermissionError("simulated transient sharing violation")
        real_replace(src, dst)

    monkeypatch.setattr(os, "replace", flaky)
    write_heartbeat(run_dir, {"ok": 1})         # must succeed silently
    assert denials["n"] == 2
    with open(os.path.join(run_dir, "heartbeat.json"), "r", encoding="utf-8") as f:
        assert json.load(f) == {"ok": 1}


# ============================================================
# Crash hooks
# ============================================================
# Paths travel via sys.argv — no Windows backslash/quoting hazards in -c code.
CRASH_CHILD = (
    "import sys; "
    "sys.path.insert(0, sys.argv[1]); "
    "from finals.events import install_crash_hooks; "
    "install_crash_hooks(sys.argv[2]); "
    "raise RuntimeError('boom')"
)


def test_crash_hooks_subprocess_writes_crash_txt(tmp_path, repo_root):
    """The forensic path you can never test by accident: a real uncaught
    exception in a real interpreter must leave crash.txt behind."""
    crash_run = tmp_path / "crashrun"
    crash_run.mkdir()
    proc = subprocess.run(
        [sys.executable, "-c", CRASH_CHILD, repo_root, str(crash_run)],
        capture_output=True, text=True, timeout=60,
    )
    assert proc.returncode != 0, (
        f"child should die; stdout={proc.stdout!r} stderr={proc.stderr!r}")
    crash = crash_run / "crash.txt"
    assert crash.is_file(), f"crash.txt missing; child stderr: {proc.stderr!r}"
    body = crash.read_text(encoding="utf-8")
    assert "RuntimeError" in body and "boom" in body
    assert "Traceback (most recent call last)" in body
    assert (crash_run / "fault.txt").exists()      # faulthandler target created
    assert "RuntimeError" in proc.stderr           # previous hook still chained


def test_install_crash_hooks_is_idempotent(tmp_path, excepthook_guard):
    install_crash_hooks(str(tmp_path))
    first = sys.excepthook
    assert first is not sys.__excepthook__         # something was installed
    install_crash_hooks(str(tmp_path))
    assert sys.excepthook is first                 # no double-wrap
    assert faulthandler.is_enabled()


def test_crash_hook_chains_previous_hook_and_writes_crash_txt(tmp_path, excepthook_guard):
    seen: list = []
    sys.excepthook = lambda et, ev, tb: seen.append(ev)   # guard restores it
    install_crash_hooks(str(tmp_path))
    try:
        raise RuntimeError("in-process boom")
    except RuntimeError as exc:
        sys.excepthook(type(exc), exc, exc.__traceback__)  # invoke manually

    body = (tmp_path / "crash.txt").read_text(encoding="utf-8")
    assert "in-process boom" in body and "RuntimeError" in body
    assert len(seen) == 1 and "in-process boom" in str(seen[0])  # chained once


def test_crash_hook_still_chains_when_crash_txt_unwritable(tmp_path, excepthook_guard, capsys):
    """The 2 a.m. guarantee: a logging failure inside the crash hook must
    never mask the real crash — the previous hook is ALWAYS chained."""
    seen: list = []
    sys.excepthook = lambda et, ev, tb: seen.append(ev)
    install_crash_hooks(str(tmp_path))
    (tmp_path / "crash.txt").mkdir()        # open(..., "a") now raises OSError

    try:
        raise RuntimeError("unloggable boom")
    except RuntimeError as exc:
        sys.excepthook(type(exc), exc, exc.__traceback__)   # must not raise

    assert len(seen) == 1 and "unloggable boom" in str(seen[0])  # crash NOT masked
    assert "could not write" in capsys.readouterr().err          # loud fallback
