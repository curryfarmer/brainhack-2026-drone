"""bench_flight tests — propless scripted-flight logger over MockAdapter.

Zero SDK / zero hardware: the whole file runs on the BARE venv (no
cv2/numpy/pyhulax). MockAdapter is pure-Python, and replay_plot.build_tracks is
pure stdlib (matplotlib is only pulled in by plot_tracks, which we never call),
so NOTHING here is importorskip-gated.

What each pin catches (mutation kill-check in the docstring of each test):
- the JSONL captures EVERY scripted command with PRE+POST telemetry (exact
  record count — a dropped record fails);
- rotate CHANGES yaw in the post snapshot (MockAdapter's real behaviour);
- is_flying FLIPS on takeoff/land;
- the --props-off-confirmed gate REFUSES (non-zero exit) when absent;
- an injected abort mid-sequence calls emergency_land + stops early;
- teardown NEVER raises even when a command fails;
- the action_complete records replay through the REAL replay_plot.build_tracks
  unchanged (the schema-compat proof).
"""
from __future__ import annotations

import asyncio
import os

import pytest

from finals.events import EventLog, read_events
from finals.flight.mock_adapter import MockAdapter
from finals.tools import bench_flight as bf
from finals.tools.bench_flight import (BenchFlightError, main, parse_commands,
                                       run_sequence, telemetry_snapshot)
from finals.types import Direction


# ============================================================
# helpers
# ============================================================
def _events(run_dir: str):
    return list(read_events(os.path.join(run_dir, "mission.jsonl")))


def _of(events, name):
    return [e for e in events if e.get("event") == name]


def _run(adapter, steps, run_dir, **kw):
    with EventLog(run_dir) as ev:
        return asyncio.run(run_sequence(adapter, adapter.drone_id, steps, ev,
                                        **kw))


# ============================================================
# command-script parsing
# ============================================================
def test_parse_default_sequence():
    """The default 'takeoff:60,hover:2,rotate:90,land' parses to 4 typed steps
    with the right args. MUTATION: change rotate arg expectation to 91 -> fails."""
    steps = parse_commands(bf._DEFAULT_COMMANDS)
    assert steps == [
        ("takeoff", {"height_cm": 60}),
        ("hover", {"duration_s": 2.0}),
        ("rotate", {"angle_deg": 90.0}),
        ("land", {}),
    ]


def test_parse_move_and_defaults():
    """move:forward maps to Direction.FORWARD with the 50 cm default; an
    explicit cm overrides. MUTATION: default 50 -> a wrong default fails here."""
    steps = parse_commands("takeoff,move:forward,move:left:120,land")
    assert steps[0] == ("takeoff", {"height_cm": 60})       # takeoff default
    assert steps[1] == ("move", {"direction": Direction.FORWARD,
                                 "distance_cm": 50})        # move default cm
    assert steps[2] == ("move", {"direction": Direction.LEFT,
                                 "distance_cm": 120})


def test_parse_rejects_bad_steps():
    """Every malformed step is a LOUD BenchFlightError, never a silent drop.
    MUTATION: if any branch silently accepted, one of these would not raise."""
    for spec in ("", "   ", "takeoff,,land", "fly:10", "hover",
                 "rotate", "move", "move:sideways:50", "rotate:nope",
                 "hover:-1", "takeoff:0", "move:forward:-5", "land:1",
                 "takeoff:inf"):
        with pytest.raises(BenchFlightError):
            parse_commands(spec)


def test_parse_hover_zero_is_allowed():
    """hover:0 is a valid (>=0) duration; rotate:-90 (CCW negative) is valid.
    MUTATION: a require_positive on hover/rotate would wrongly reject these."""
    steps = parse_commands("hover:0,rotate:-90")
    assert steps[0] == ("hover", {"duration_s": 0.0})
    assert steps[1] == ("rotate", {"angle_deg": -90.0})


# ============================================================
# the sequence runner — JSONL completeness + telemetry response
# ============================================================
def test_every_command_logged_with_pre_post(tmp_path):
    """EXACT record count: one action_complete AND one bench_command per
    command, each bench_command carrying a pre AND a post telemetry snapshot.
    MUTATION: drop a single events.log() call -> these counts fail."""
    run_dir = str(tmp_path)
    adapter = MockAdapter("bench")
    steps = parse_commands("takeoff:60,hover:1,rotate:90,land")
    summary = _run(adapter, steps, run_dir)
    assert summary["commands_completed"] == 4
    assert summary["aborted"] is False and summary["failed"] is None

    events = _events(run_dir)
    assert len(_of(events, "origin")) == 1
    acs = _of(events, "action_complete")
    bcs = _of(events, "bench_command")
    assert len(acs) == 4                       # one per command, exactly
    assert len(bcs) == 4
    for bc in bcs:
        d = bc["data"]
        assert "pre" in d and "post" in d
        # pre/post carry the four telemetry fields + the airborne flag.
        for snap in (d["pre"], d["post"]):
            assert set(("battery_pct", "altitude_m", "yaw_deg",
                        "is_flying", "airborne")) <= set(snap)


def test_rotate_changes_yaw_in_post(tmp_path):
    """MockAdapter's DeadReckoner yaws on rotate -> the rotate command's POST
    yaw differs from its PRE yaw by ~the commanded angle. MUTATION: if the post
    snapshot were taken BEFORE the command, pre==post and this fails."""
    run_dir = str(tmp_path)
    adapter = MockAdapter("bench")
    _run(adapter, parse_commands("takeoff:60,rotate:90,land"), run_dir)
    bcs = _of(_events(run_dir), "bench_command")
    rot = next(b for b in bcs if b["data"]["command"] == "rotate")
    pre_yaw = rot["data"]["pre"]["yaw_deg"]
    post_yaw = rot["data"]["post"]["yaw_deg"]
    assert pre_yaw == pytest.approx(0.0, abs=1e-6)
    assert post_yaw == pytest.approx(90.0, abs=1e-6)
    assert post_yaw != pre_yaw


def test_is_flying_flips_on_takeoff_and_land(tmp_path):
    """is_flying False->True across takeoff, True->False across land — the
    bench proof that the airframe armed/disarmed. MUTATION: a stuck flag fails."""
    run_dir = str(tmp_path)
    adapter = MockAdapter("bench")
    _run(adapter, parse_commands("takeoff:60,land"), run_dir)
    bcs = _of(_events(run_dir), "bench_command")
    takeoff = next(b for b in bcs if b["data"]["command"] == "takeoff")
    land = next(b for b in bcs if b["data"]["command"] == "land")
    assert takeoff["data"]["pre"]["is_flying"] is False
    assert takeoff["data"]["post"]["is_flying"] is True
    assert land["data"]["pre"]["is_flying"] is True
    assert land["data"]["post"]["is_flying"] is False


def test_battery_telemetry_plumbing_moves(tmp_path):
    """Telemetry plumbing is live: the battery decays across the sequence so
    the last post battery < the first pre battery (props-off proof #2).
    MUTATION: a frozen snapshot would keep these equal."""
    run_dir = str(tmp_path)
    adapter = MockAdapter("bench", battery_decay_pct_per_cmd=2.0)
    _run(adapter, parse_commands("takeoff:60,hover:1,rotate:90,land"), run_dir)
    bcs = _of(_events(run_dir), "bench_command")
    first_pre = bcs[0]["data"]["pre"]["battery_pct"]
    last_post = bcs[-1]["data"]["post"]["battery_pct"]
    assert last_post < first_pre


# ============================================================
# replay_plot schema compatibility (the strongest proof)
# ============================================================
def test_log_replays_through_real_replay_plot(tmp_path):
    """The action_complete records feed the REAL replay_plot.build_tracks
    UNCHANGED — one track, action_names exactly matching the issued commands.
    build_tracks is pure stdlib (no matplotlib) so this runs on the bare venv.
    MUTATION: an extra field on action_complete -> reconstruct_action's
    exact-field-set firewall raises ReplayPlotError and this fails."""
    from finals.tools.replay_plot import build_tracks

    run_dir = str(tmp_path)
    adapter = MockAdapter("bench")
    _run(adapter, parse_commands("takeoff:60,hover:1,rotate:90,"
                                 "move:forward:100,land"), run_dir)
    tracks = build_tracks(os.path.join(run_dir, "mission.jsonl"))
    assert set(tracks) == {"bench"}
    assert tracks["bench"].action_names == [
        "Takeoff", "Hover", "Rotate", "Move", "Land"]
    # The Move advanced the DR track (it integrated through the +90 yaw).
    final = tracks["bench"].poses[-1]
    assert final.yaw_deg == pytest.approx(90.0, abs=1e-6)


# ============================================================
# abort + teardown safety
# ============================================================
def test_abort_midsequence_calls_emergency_land(tmp_path):
    """An abort_event set BEFORE the run lands everything: zero commands issued
    (the per-iteration check trips on the first step), emergency_land recorded
    in the adapter's .calls, and a bench_abort event logged. MUTATION: remove
    the abort_event check and commands would be issued + no bench_abort."""
    import threading
    run_dir = str(tmp_path)
    adapter = MockAdapter("bench")
    abort = threading.Event()
    abort.set()
    summary = _run(adapter, parse_commands("takeoff:60,rotate:90,land"),
                   run_dir, abort_event=abort)
    assert summary["aborted"] is True
    assert summary["commands_completed"] == 0
    assert ("emergency_land", {}) in adapter.calls
    assert len(_of(_events(run_dir), "bench_abort")) == 1
    # No flight command was issued (takeoff/rotate/land never recorded).
    issued = {name for name, _ in adapter.calls}
    assert issued.isdisjoint({"takeoff", "rotate"})


def test_abort_after_first_command_stops_early(tmp_path):
    """abort_event set the instant the FIRST command completes: the run does
    command #1, sees the abort before #2, emergency-lands (commands_completed
    stays 1, not 3). MUTATION: a missing mid-loop abort check would run all 3."""
    import threading
    run_dir = str(tmp_path)
    abort = threading.Event()

    class _AbortAfterTakeoff(MockAdapter):
        async def takeoff(self, *a, **k):
            await super().takeoff(*a, **k)
            abort.set()                    # trip right after command #1

    adapter = _AbortAfterTakeoff("bench")
    summary = _run(adapter, parse_commands("takeoff:60,rotate:90,land"),
                   run_dir, abort_event=abort)
    assert summary["aborted"] is True
    assert summary["commands_completed"] == 1
    assert ("emergency_land", {}) in adapter.calls
    # The 2nd command (rotate) was never issued.
    assert "rotate" not in {name for name, _ in adapter.calls}


def test_teardown_never_raises_when_command_fails(tmp_path):
    """A scripted command failure (fail_on takeoff) propagates as FlightError,
    but the finally STILL safed the drone down: emergency_land recorded, a
    bench_failed event logged. MUTATION: if the finally re-raised or were
    skipped, emergency_land would be missing."""
    from finals.errors import FlightError
    run_dir = str(tmp_path)
    adapter = MockAdapter("bench", fail_on={"takeoff": FlightError("boom")})
    with pytest.raises(FlightError):
        _run(adapter, parse_commands("takeoff:60,land"), run_dir)
    assert ("emergency_land", {}) in adapter.calls
    assert len(_of(_events(run_dir), "bench_failed")) == 1


def test_telemetry_snapshot_never_raises_when_disconnected():
    """telemetry_snapshot swallows a FlightError into {'error': ...} so a
    dropped link mid-run cannot crash the sequence. MUTATION: drop the typed
    catch and this raises FlightError."""
    adapter = MockAdapter("bench")        # never connected -> telemetry raises
    snap = telemetry_snapshot(adapter)
    assert "error" in snap
    assert snap["airborne"] is None       # MockAdapter has no _airborne attr


# ============================================================
# CLI gate + end-to-end
# ============================================================
def test_props_off_gate_refuses(capsys):
    """No --props-off-confirmed -> exit 2, nothing flown, WHAT/WHY printed.
    MUTATION: remove the gate (return 0) and this fails on the exit code."""
    rc = main(["--mock"])
    assert rc == 2
    err = capsys.readouterr().err
    assert "REFUSED" in err and "props-off-confirmed" in err


def test_props_off_gate_is_first(tmp_path, capsys):
    """The gate fires even with an otherwise-invalid --commands: default-deny
    means we never reach parsing. MUTATION: if parsing ran first, the error
    text would mention the command, not the props gate."""
    rc = main(["--mock", "--commands", "bogus:step"])
    assert rc == 2
    assert "props-off-confirmed" in capsys.readouterr().err


def test_e2e_mock_run_exit0(tmp_path, capsys):
    """The documented E2E: --mock --props-off-confirmed runs the full default
    sequence with no hardware, writes a JSONL, prints a SUMMARY, exits 0.
    MUTATION: any unhandled error in the path would flip the exit code."""
    rc = main(["--mock", "--props-off-confirmed", "--no-abort-key",
               "--out-dir", str(tmp_path)])
    assert rc == 0
    out = capsys.readouterr().out
    assert "SUMMARY" in out and "4/4 commands completed" in out
    # The JSONL exists under the created run dir and has the 4 action_completes.
    run_dirs = [d for d in os.listdir(tmp_path)
                if os.path.isdir(os.path.join(tmp_path, d))]
    assert len(run_dirs) == 1
    events = _events(os.path.join(tmp_path, run_dirs[0]))
    assert len(_of(events, "action_complete")) == 4


def test_e2e_custom_commands(tmp_path, capsys):
    """--commands flows through main to the runner: a 2-command square leg
    completes and is logged. MUTATION: if --commands were ignored (default
    used), the count would be 4, not 2."""
    rc = main(["--mock", "--props-off-confirmed", "--no-abort-key",
               "--commands", "takeoff:50,land", "--out-dir", str(tmp_path)])
    assert rc == 0
    run_dirs = [d for d in os.listdir(tmp_path)
                if os.path.isdir(os.path.join(tmp_path, d))]
    events = _events(os.path.join(tmp_path, run_dirs[0]))
    assert len(_of(events, "action_complete")) == 2


def test_e2e_bad_commands_exit2(capsys):
    """A bad --commands (past the props gate) exits 2 with a loud parse error.
    MUTATION: a swallowed parse error would not surface the exit code."""
    rc = main(["--mock", "--props-off-confirmed", "--no-abort-key",
               "--commands", "fly:to:the:moon"])
    assert rc == 2
    assert "unknown command" in capsys.readouterr().err
