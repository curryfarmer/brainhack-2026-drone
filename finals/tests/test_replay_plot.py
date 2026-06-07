"""replay_plot tests — fixture-pinned DR replay + rendering (SIM-2).

What each pin catches:
- closure: the commanded square returns to its post-takeoff position. DR
  replay is deterministic (it integrates commands, not telemetry), so the
  tolerance is float roundoff, not physics.
- signed shoelace area +1.0 m²: closure is chirality-blind (a mirrored
  square also closes). The SIGN of the area is the left/right-turn truth —
  +90° CCW turns trace a CCW polygon in east-X/north-Y, so the area is
  positive — and the magnitude pins the cm->m boundary (a slip renders
  ±1e4 m²). Yaw-invariant by construction.
- boot-yaw first leg: closure + area are yaw-invariant, so they cannot see
  an "assume boot yaw 0" seeding bug. This pins the first move end to the
  fixture's REAL EKF boot yaw (-95.97°, hardcoded from the file).
- heading_vector vs the real DeadReckoner: the quiver/bearing-ray math must
  be the exact FORWARD row the track math uses, or plots lie tangentially.

Synthetic logs ALWAYS end with a newline: read_events skips an unterminated
final line even when it parses (determinism beats coincidence).
"""
from __future__ import annotations

import json
import math
import os
import shutil

import pytest

from finals.flight.dead_reckon import DeadReckoner, DRPose
from finals.tools.replay_plot import (_ARROW_LEN_M, _BEARING_RAY_M,
                                      ReplayPlotError, build_tracks,
                                      heading_vector, main, plot_tracks)
from finals.types import Direction, Move

FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "fixtures", "sim1_v1_square.jsonl")

# Hardcoded from the fixture's origin event (the whole point: seeding must
# come from the FILE, so a zero-seeding mutant cannot fake these).
ORIGIN_POS = (0.02517550438642502, -0.04198383167386055, 0.012403085827827454)
ORIGIN_YAW_DEG = -95.9705810546875


def _line(drone: str, event: str, **data) -> str:
    return json.dumps({"ts": 0.0, "mono": 0.0, "drone": drone,
                       "event": event, "data": data})


def _origin(drone: str = "alpha", **over) -> str:
    data = {"position_m": [0.0, 0.0, 0.0], "yaw_deg": 0.0,
            "position_quality": "MEASURED", "is_flying": False}
    data.update(over)
    return _line(drone, "origin", **data)


def _write_log(tmp_path, lines) -> str:
    p = tmp_path / "mission.jsonl"
    p.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return str(p)


def _signed_area_m2(corners) -> float:
    """Shoelace over (east, north) pairs; CCW positive."""
    return 0.5 * sum(
        corners[i][0] * corners[(i + 1) % len(corners)][1]
        - corners[(i + 1) % len(corners)][0] * corners[i][1]
        for i in range(len(corners)))


def _fixture_track():
    tracks = build_tracks(FIXTURE)
    assert set(tracks) == {"alpha"}
    return tracks["alpha"]


def _move_ends(track):
    return [track.poses[i + 1]
            for i, name in enumerate(track.action_names) if name == "Move"]


# ---------------- fixture-pinned DR replay ----------------
def test_fixture_single_drone_track():
    track = _fixture_track()
    assert len(track.action_names) == 11          # Takeoff, Hover, 4x(Move,Rotate), Land
    assert len(track.poses) == 12                 # origin + one per action
    assert track.poses[0] == track.origin
    assert track.action_names[0] == "Takeoff"
    assert track.action_names[-1] == "Land"
    assert track.sightings == []                  # headless fixture: no detector


def test_fixture_square_closes():
    track = _fixture_track()
    post_takeoff, final = track.poses[1], track.poses[-1]
    assert post_takeoff.alt_m == pytest.approx(0.8)         # Takeoff 80 cm
    assert final.north_m == pytest.approx(post_takeoff.north_m, abs=1e-9)
    assert final.east_m == pytest.approx(post_takeoff.east_m, abs=1e-9)
    assert final.alt_m == 0.0                               # Land zeroes alt
    assert final.yaw_deg == pytest.approx(track.origin.yaw_deg, abs=1e-9)


def test_fixture_signed_area_is_plus_one():
    ends = _move_ends(_fixture_track())
    assert len(ends) == 4                         # the 4 square corners
    area = _signed_area_m2([(p.east_m, p.north_m) for p in ends])
    assert area == pytest.approx(1.0, abs=1e-9)   # CCW (+90 turns), 1 m legs


def test_fixture_first_leg_follows_boot_yaw():
    track = _fixture_track()
    assert track.origin.yaw_deg == pytest.approx(ORIGIN_YAW_DEG)
    assert track.origin.north_m == pytest.approx(ORIGIN_POS[0])
    theta = math.radians(ORIGIN_YAW_DEG)
    first = _move_ends(track)[0]                  # FORWARD 100 cm from boot yaw
    assert first.north_m == pytest.approx(ORIGIN_POS[0] + math.cos(theta), abs=1e-9)
    assert first.east_m == pytest.approx(ORIGIN_POS[1] - math.sin(theta), abs=1e-9)


@pytest.mark.parametrize("yaw", [0.0, 30.0, 90.0, -120.0, 180.0])
def test_heading_vector_matches_dead_reckoner(yaw):
    dr = DeadReckoner(DRPose(0.0, 0.0, 0.0, yaw))
    dr.note_action_complete(Move(direction=Direction.FORWARD, distance_cm=100))
    east, north = heading_vector(yaw)             # 1 m FORWARD == unit nose vector
    assert dr.pose.east_m == pytest.approx(east, abs=1e-12)
    assert dr.pose.north_m == pytest.approx(north, abs=1e-12)


# ---------------- CLI / rendering ----------------
@pytest.mark.parametrize("pass_dir", [True, False])
def test_cli_save_png(tmp_path, pass_dir):
    pytest.importorskip("matplotlib")
    mission = tmp_path / "mission.jsonl"
    shutil.copyfile(FIXTURE, mission)
    out = tmp_path / "out.png"
    target = str(tmp_path) if pass_dir else str(mission)
    assert main([target, "--save", str(out)]) == 0
    blob = out.read_bytes()
    assert blob[:8] == b"\x89PNG\r\n\x1a\n"
    assert len(blob) > 1000


def test_cli_missing_mission_jsonl_exits_2(tmp_path, capsys):
    assert main([str(tmp_path)]) == 2
    assert "mission.jsonl" in capsys.readouterr().err


def test_cli_save_dir_must_exist(tmp_path, capsys):
    mission = tmp_path / "mission.jsonl"
    shutil.copyfile(FIXTURE, mission)
    missing = tmp_path / "no_such_dir" / "out.png"
    assert main([str(tmp_path), "--save", str(missing)]) == 2
    assert "does not exist" in capsys.readouterr().err


def test_cli_empty_save_path_exits_2(tmp_path, capsys):
    assert main([str(tmp_path), "--save", ""]) == 2
    assert "non-empty" in capsys.readouterr().err


def test_plot_geometry_pins_axes_quivers_and_rays(tmp_path):
    """A PNG existing proves nothing about geometry: pin the RENDERED
    artists — an east/north axis swap, a quiver U/V swap, or a bearing-ray
    frame error must fail HERE, not in someone's eyeball."""
    pytest.importorskip("matplotlib")
    from matplotlib.quiver import Quiver

    # Fixture track: line vertices + quiver vectors.
    tracks = build_tracks(FIXTURE)
    fig = plot_tracks(tracks, save=str(tmp_path / "fx.png"), source="fx")
    ax = fig.axes[0]
    track = tracks["alpha"]
    xy = ax.lines[0].get_xydata()
    assert len(xy) == len(track.poses)
    for got, p in zip(xy, track.poses):
        assert got[0] == pytest.approx(p.east_m)    # X IS east
        assert got[1] == pytest.approx(p.north_m)   # Y IS north
    quivers = [c for c in ax.collections if isinstance(c, Quiver)]
    assert len(quivers) == len(track.poses)         # 12 poses <= decimation cap
    e0, n0 = heading_vector(track.poses[0].yaw_deg)
    assert float(quivers[0].U[0]) == pytest.approx(_ARROW_LEN_M * e0)
    assert float(quivers[0].V[0]) == pytest.approx(_ARROW_LEN_M * n0)

    # Synthetic single sighting: bearing 90 (CCW from north = WEST nose)
    # must ray to (east - 1.5, north).
    path = _write_log(tmp_path, [
        _origin(),
        _line("alpha", "action_complete", action="Takeoff", elapsed_s=1.0,
              height_cm=80),
        _line("alpha", "sighting", source="aruco", class_name="marker",
              marker_id=3, confidence=1.0, ts=0.0, frame_number=1,
              bearing_deg=90.0)])
    tracks2 = build_tracks(path)
    fig2 = plot_tracks(tracks2, save=str(tmp_path / "ray.png"), source="ray")
    ray = fig2.axes[0].lines[-1]                    # last artist = the ray
    end = ray.get_xydata()[-1]
    assert end[0] == pytest.approx(-_BEARING_RAY_M)  # west of (0, 0)
    assert end[1] == pytest.approx(0.0, abs=1e-12)


# ---------------- typed failure paths ----------------
def test_action_before_origin_raises(tmp_path):
    path = _write_log(tmp_path, [
        _line("ghost", "action_complete", action="Move", elapsed_s=1.0,
              direction="FORWARD", distance_cm=100)])
    with pytest.raises(ReplayPlotError, match="ghost.*before origin|before origin.*ghost"):
        build_tracks(path)


def test_unknown_action_raises(tmp_path):
    path = _write_log(tmp_path, [
        _origin(),
        _line("alpha", "action_complete", action="Teleport", elapsed_s=1.0)])
    with pytest.raises(ReplayPlotError, match="Teleport"):
        build_tracks(path)


def test_unexpected_field_raises(tmp_path):
    path = _write_log(tmp_path, [
        _origin(),
        _line("alpha", "action_complete", action="Move", elapsed_s=1.0,
              direction="FORWARD", distance_cm=100, warp_factor=9)])
    with pytest.raises(ReplayPlotError, match="warp_factor"):
        build_tracks(path)


def test_unknown_direction_raises(tmp_path):
    path = _write_log(tmp_path, [
        _origin(),
        _line("alpha", "action_complete", action="Move", elapsed_s=1.0,
              direction="SIDEWAYS", distance_cm=100)])
    with pytest.raises(ReplayPlotError, match="SIDEWAYS"):
        build_tracks(path)


def test_duplicate_origin_raises(tmp_path):
    path = _write_log(tmp_path, [_origin(), _origin()])
    with pytest.raises(ReplayPlotError, match="two origin"):
        build_tracks(path)


def test_no_drone_events_raises(tmp_path):
    path = _write_log(tmp_path, [
        _line("mission", "run_start", drones=["alpha"]),
        _line("mission", "run_end", exit_code=0)])
    with pytest.raises(ReplayPlotError, match="no drone origin"):
        build_tracks(path)


def test_takeoff_missing_height_cm_raises(tmp_path):
    """Dataclass defaults must NOT fill dropped fields: Takeoff() would
    silently claim 80 cm whatever the mission actually flew."""
    path = _write_log(tmp_path, [
        _origin(),
        _line("alpha", "action_complete", action="Takeoff", elapsed_s=1.0)])
    with pytest.raises(ReplayPlotError, match="height_cm"):
        build_tracks(path)


def test_null_action_field_value_raises_typed(tmp_path):
    path = _write_log(tmp_path, [
        _origin(),
        _line("alpha", "action_complete", action="Move", elapsed_s=1.0,
              direction="FORWARD", distance_cm=None)])
    with pytest.raises(ReplayPlotError, match="DeadReckoner refuses"):
        build_tracks(path)


def test_unhashable_direction_raises_typed(tmp_path):
    path = _write_log(tmp_path, [
        _origin(),
        _line("alpha", "action_complete", action="Move", elapsed_s=1.0,
              direction=["FORWARD"], distance_cm=100)])
    with pytest.raises(ReplayPlotError, match="FORWARD"):
        build_tracks(path)


def test_string_origin_yaw_raises_not_seeds_zero(tmp_path):
    """Strings are schema drift, NOT PositionQuality.NONE nulls — silently
    seeding 0.0 would render the whole track rotated."""
    path = _write_log(tmp_path, [_origin(yaw_deg="-95.97")])
    with pytest.raises(ReplayPlotError, match="yaw_deg"):
        build_tracks(path)


def test_string_origin_position_raises(tmp_path):
    path = _write_log(tmp_path, [_origin(position_m=["0", "0", "0"])])
    with pytest.raises(ReplayPlotError, match="position_m"):
        build_tracks(path)


def test_nan_origin_yaw_raises_typed(tmp_path):
    # json.dumps happily emits NaN and read_events parses it back.
    path = _write_log(tmp_path, [_origin(yaw_deg=float("nan"))])
    with pytest.raises(ReplayPlotError, match="usable pose"):
        build_tracks(path)


@pytest.mark.parametrize("head, match", [
    (b"\xef\xbb\xbf", "BOM"),
    (b"\xff\xfe", "UTF-16"),
])
def test_bom_and_utf16_rejected_typed(tmp_path, capsys, head, match):
    """PowerShell-mangled copies must die NAMED, not as 'origin missing'
    ghosts (read_events would skip the mangled lines one by one)."""
    p = tmp_path / "mission.jsonl"
    p.write_bytes(head + b'{"x": 1}\n')
    assert main([str(tmp_path)]) == 2
    assert match in capsys.readouterr().err


def test_null_origin_warns_and_seeds_zero(tmp_path, capsys):
    path = _write_log(tmp_path, [
        _origin(position_m=None, yaw_deg=None),
        _line("alpha", "action_complete", action="Takeoff", elapsed_s=1.0,
              height_cm=80)])
    tracks = build_tracks(path)
    assert tracks["alpha"].origin == DRPose(0.0, 0.0, 0.0, 0.0)
    err = capsys.readouterr().err
    assert "UNKNOWN boot pose" in err and "alpha" in err


def test_never_connected_drone_noted_not_fatal(tmp_path, capsys):
    path = _write_log(tmp_path, [
        _origin(),
        _line("alpha", "action_complete", action="Takeoff", elapsed_s=1.0,
              height_cm=80),
        _line("bravo", "agent_connect", timeout_s=30.0)])   # killed pre-origin
    tracks = build_tracks(path)
    assert set(tracks) == {"alpha"}
    assert "bravo" in capsys.readouterr().err


# ---------------- sightings ----------------
def test_sighting_marks_use_pose_at_sighting_time(tmp_path):
    path = _write_log(tmp_path, [
        _origin(),                                          # yaw 0 = +north
        _line("alpha", "action_complete", action="Takeoff", elapsed_s=1.0,
              height_cm=80),
        _line("alpha", "action_complete", action="Move", elapsed_s=1.0,
              direction="FORWARD", distance_cm=100),
        _line("alpha", "sighting", source="aruco", class_name="marker",
              marker_id=7, confidence=0.9, ts=0.0, frame_number=1,
              bearing_deg=0.0),
        _line("alpha", "action_complete", action="Move", elapsed_s=1.0,
              direction="FORWARD", distance_cm=100),
        _line("alpha", "sighting", source="yolo", class_name="convoy",
              marker_id=None, confidence=0.5, ts=0.0, frame_number=2,
              bearing_deg=None)])
    track = build_tracks(path)["alpha"]
    first, second = track.sightings
    assert (first.east_m, first.north_m) == (pytest.approx(0.0), pytest.approx(1.0))
    assert first.bearing_deg == 0.0                        # mid-track, NOT end pose
    assert first.label == "marker#7"
    assert (second.east_m, second.north_m) == (pytest.approx(0.0), pytest.approx(2.0))
    assert second.bearing_deg is None                      # scatter only, no ray
    assert second.label == "convoy"


def test_sighting_placed_at_capture_time_not_log_row(tmp_path):
    """The bus is drained per orchestrator tick, so a sighting ROW can land
    after a later move's action_complete. Placement must follow data.ts
    (capture, monotonic clock shared with row "mono"), not row order —
    row order would misplace the mark by a whole leg."""
    def _row(mono, drone, event, **data):
        return json.dumps({"ts": 0.0, "mono": mono, "drone": drone,
                           "event": event, "data": data})
    path = _write_log(tmp_path, [
        _row(10.0, "alpha", "origin", position_m=[0.0, 0.0, 0.0],
             yaw_deg=0.0),
        _row(11.0, "alpha", "action_complete", action="Takeoff",
             elapsed_s=1.0, height_cm=80),
        _row(12.0, "alpha", "action_complete", action="Move", elapsed_s=1.0,
             direction="FORWARD", distance_cm=100),
        _row(14.0, "alpha", "action_complete", action="Move", elapsed_s=1.0,
             direction="FORWARD", distance_cm=100),
        # Logged AFTER the second move, captured BETWEEN the moves (ts 13):
        _row(14.5, "alpha", "sighting", source="aruco", class_name="marker",
             marker_id=1, confidence=1.0, ts=13.0, frame_number=1,
             bearing_deg=0.0)])
    mark = build_tracks(path)["alpha"].sightings[0]
    assert (mark.east_m, mark.north_m) == (pytest.approx(0.0),
                                           pytest.approx(1.0))  # NOT 2.0


def test_bool_bearing_renders_no_ray(tmp_path):
    path = _write_log(tmp_path, [
        _origin(),
        _line("alpha", "sighting", source="aruco", class_name="marker",
              marker_id=1, confidence=1.0, ts=0.0, frame_number=1,
              bearing_deg=True)])  # bool is NOT a bearing
    assert build_tracks(path)["alpha"].sightings[0].bearing_deg is None
