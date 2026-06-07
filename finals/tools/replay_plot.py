"""replay_plot — DR replay of a run's mission.jsonl into per-drone track plots.

The flight evidence tool (simulation.md Tier 0): feed each drone's COMPLETED
command events through the REAL finals.flight.dead_reckon.DeadReckoner and
render the integrated track east-on-X / north-on-Y. The plotter never
reimplements the pose math — a chirality/scale/seeding bug here would
silently bless a broken flight, so the math has exactly one source of truth
and the fixture test pins this file to it (tests/test_replay_plot.py:
square closure + signed shoelace area +1 m²).

Binding design decisions:
- DR is seeded from each drone's "origin" event (DRPose from position_m +
  yaw_deg). EKF boot yaw is NOT 0 (the SIM-1 fixture booted at -95.97 deg);
  assuming 0 renders every track rotated. On real-hardware logs origin
  fields may be null (PositionQuality.NONE is a supported mode) — those
  seed 0.0 per missing field with a LOUD stderr warning instead of erroring,
  because this tool must still render the finals' own logs.
- One subplot PER DRONE, never merged axes: "north" is each drone's OWN
  zero-yaw boot heading (dead_reckon.py frame convention, binding), so
  tracks from different drones live in different frames and overlaying them
  would be geometrically false. Spawn-pose offsets in configs/sitl3.json's
  _comment are documentation, not plot inputs.
- action_complete events replay through a full 8-type Action map even though
  Wait/Done/Abort never reach the log today (agent.py handles Wait inline;
  Done/Abort become phase events): if the log shape ever grows them they
  feed DeadReckoner (a no-op there) instead of dying as "unknown".
- Unknown action names, unknown Direction names and non-fitting field sets
  are typed ReplayPlotError failures naming the drone, file and fix hint —
  schema drift must never render as a silently-wrong picture.
- matplotlib is imported lazily inside plot_tracks() only: backend selection
  (Agg when --save is given — the VM is headless) must precede the first
  pyplot import, and the parsing/DR core stays importable in a venv without
  matplotlib (which would drag numpy into the bare-venv suite).
- Sighting marks sit at the DR pose current at frame CAPTURE time:
  Sighting.ts is stamped at capture on the monotonic clock
  (sightings.py SightingBus docstring; perception.py clock=time.monotonic),
  the same domain as every event row's "mono" — the bus is drained per
  orchestrator tick, so the log ROW can land after a later move's
  action_complete and row order would misplace the mark by a whole leg.
  Rows without usable ts/mono fall back to row order. Bearing rays use the
  sighting's bearing_deg, which shares the CCW-from-north per-drone frame
  (vision/perception.py bearing_from_bbox, S7).

Derives from: finals/events.py read_events (torn-tail-tolerant reader) +
finals/flight/dead_reckon.py (the binding frame convention and the math).

CLI: python -m finals.tools.replay_plot <run_dir|mission.jsonl> [--save out.png]

Pure stdlib + (lazily) matplotlib; no cv2/gz/numpy — finals/tools/ is inside
the conventions scan.

Session: SIM-2 (implemented).
"""
from __future__ import annotations

import argparse
import bisect
import dataclasses
import math
import os
import sys
from types import MappingProxyType
from typing import Dict, List, Optional, Sequence, Tuple

from finals.errors import FinalsError
from finals.events import read_events
from finals.flight.dead_reckon import DeadReckoner, DRPose
from finals.types import (Abort, Action, Direction, Done, Hover, Land, Move,
                          Rotate, Takeoff, Wait)


class ReplayPlotError(FinalsError):
    """Replay input is missing/malformed. Message carries the path, the
    drone and what to check."""


#: action_complete "action" name -> dataclass. All 8 types on purpose (see
#: module docstring) even though only 5 reach the log today.
_ACTION_TYPES = MappingProxyType({
    "Takeoff": Takeoff, "Move": Move, "Rotate": Rotate, "Hover": Hover,
    "Land": Land, "Wait": Wait, "Done": Done, "Abort": Abort,
})
#: Event metadata keys that are NOT Action dataclass fields.
_META_KEYS = frozenset({"action", "elapsed_s"})

_QUIVER_MAX = 25      # max yaw arrows per track (decimated, always incl. last)
_ARROW_LEN_M = 0.3    # fixed arrow length in DATA units (honest under equal aspect)
_BEARING_RAY_M = 1.5  # sighting bearing-ray length in metres


def _warn(msg: str) -> None:
    print(f"[replay_plot] WARNING: {msg}", file=sys.stderr, flush=True)


@dataclasses.dataclass(frozen=True)
class SightingMark:
    """Where the drone WAS (DR pose) when it logged a sighting."""

    east_m: float
    north_m: float
    bearing_deg: Optional[float]  # CCW-from-north per-drone frame; None = no ray
    label: str


@dataclasses.dataclass
class DroneTrack:
    drone_id: str
    origin: DRPose
    poses: List[DRPose]        # poses[0] == origin; +1 per completed action
    action_names: List[str]    # parallel to poses[1:]
    sightings: List[SightingMark]


def reconstruct_action(data: dict, *, drone: str, path: str) -> Action:
    """action_complete data -> typed Action (enums were logged by NAME,
    agent.py _action_fields). The exact-field-set check is the schema-drift
    firewall: extra AND missing fields die typed — dataclass defaults must
    never silently fill a field the log dropped (a Takeoff without height_cm
    would otherwise default to 80 and lie about the altitude band)."""
    name = data.get("action")
    cls = _ACTION_TYPES.get(name)
    if cls is None:
        raise ReplayPlotError(
            f"unknown action {name!r} in action_complete for drone {drone!r} "
            f"in {path} — known: {sorted(_ACTION_TYPES)}; schema drift? check "
            f"finals/types.py against the commit that produced this run")
    fields = {k: v for k, v in data.items() if k not in _META_KEYS}
    expected = {f.name for f in dataclasses.fields(cls)}
    if set(fields) != expected:
        raise ReplayPlotError(
            f"action_complete fields {sorted(fields)} do not match "
            f"{name}({sorted(expected)}) for drone {drone!r} in {path} — "
            f"schema drift; check finals/types.py against the commit that "
            f"produced this run")
    if "direction" in fields:
        try:
            fields["direction"] = Direction[fields["direction"]]
        except (KeyError, TypeError):   # unknown name OR unhashable junk
            raise ReplayPlotError(
                f"unknown Direction {fields['direction']!r} for drone "
                f"{drone!r} in {path} — known: {[d.name for d in Direction]}"
            ) from None
    return cls(**fields)


def _is_num(v) -> bool:
    """JSON number check; bool is an int subclass but is NOT a number here."""
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _seed_pose(data: dict, *, drone: str, path: str) -> DRPose:
    """origin event data -> initial DRPose. ONLY null fields take the
    lenient seed-0.0 path (real-hardware logs carry nulls under
    PositionQuality.NONE); any OTHER non-conforming type is schema drift
    and dies typed — a string yaw_deg silently seeding 0.0 would render
    the whole track rotated."""
    pos = data.get("position_m")
    yaw = data.get("yaw_deg")
    nulls = []
    if pos is None:
        nulls.append("position_m")
        pos = (0.0, 0.0, 0.0)
    elif not (isinstance(pos, (list, tuple)) and len(pos) == 3
              and all(_is_num(c) for c in pos)):
        raise ReplayPlotError(
            f"origin position_m for drone {drone!r} in {path} must be null "
            f"or [north, east, alt] numbers, got {pos!r} — schema drift? "
            f"check the commit that produced this run")
    if yaw is None:
        nulls.append("yaw_deg")
        yaw = 0.0
    elif not _is_num(yaw):
        raise ReplayPlotError(
            f"origin yaw_deg for drone {drone!r} in {path} must be null or "
            f"a number, got {yaw!r} — schema drift? check the commit that "
            f"produced this run")
    if nulls:
        _warn(f"drone {drone!r} origin has null {', '.join(nulls)} — "
              f"seeding 0.0 (track is relative to an UNKNOWN boot pose)")
    return DRPose(float(pos[0]), float(pos[1]), float(pos[2]), float(yaw))


def build_tracks(mission_path: str) -> Dict[str, DroneTrack]:
    """One chronological pass over the events file (append-only, so file
    order is time order) -> per-drone DR tracks, first-seen drone first.

    pose_monos mirrors each track's poses with the producing row's "mono"
    stamp, so a sighting can be placed at the pose current at frame CAPTURE
    time (its ts shares the monotonic clock — module docstring) instead of
    at its log-row position, which the per-tick bus drain can delay past a
    later move's completion."""
    tracks: Dict[str, DroneTrack] = {}
    reckoners: Dict[str, DeadReckoner] = {}
    pose_monos: Dict[str, List[float]] = {}
    seen = set()
    for ev in read_events(mission_path):
        if not isinstance(ev, dict):
            _warn(f"non-object event line in {mission_path}: {ev!r} — skipped")
            continue
        drone, event, data = ev.get("drone"), ev.get("event"), ev.get("data")
        mono = ev.get("mono")
        if isinstance(drone, str) and drone != "mission":
            seen.add(drone)
        if event not in ("origin", "action_complete", "sighting"):
            continue
        if not isinstance(data, dict):
            raise ReplayPlotError(
                f"event {event!r} for drone {drone!r} in {mission_path} has "
                f"no data object (got {data!r}) — torn or foreign log?")
        if event == "origin":
            if drone in tracks:
                raise ReplayPlotError(
                    f"two origin events for drone {drone!r} in {mission_path} "
                    f"— merged/concatenated log? agents log origin exactly "
                    f"once after connect")
            origin = _seed_pose(data, drone=drone, path=mission_path)
            try:
                reckoners[drone] = DeadReckoner(origin)
            except ValueError as e:   # non-finite (NaN/Inf survive JSON)
                raise ReplayPlotError(
                    f"origin for drone {drone!r} in {mission_path} is not a "
                    f"usable pose: {e}") from e
            pose_monos[drone] = [mono if _is_num(mono) else float("-inf")]
            tracks[drone] = DroneTrack(drone_id=drone, origin=origin,
                                       poses=[origin], action_names=[],
                                       sightings=[])
        elif drone not in tracks:
            raise ReplayPlotError(
                f"{event} before origin for drone {drone!r} in {mission_path} "
                f"— torn log? agents log origin immediately after connect "
                f"(the replay prereq, mission/agent.py)")
        elif event == "action_complete":
            dr = reckoners[drone]
            action = reconstruct_action(data, drone=drone, path=mission_path)
            try:
                dr.note_action_complete(action)
            except (TypeError, ValueError) as e:   # null/str/NaN field values
                raise ReplayPlotError(
                    f"action_complete {data.get('action')!r} for drone "
                    f"{drone!r} in {mission_path} carries field values "
                    f"DeadReckoner refuses: {e} — corrupt log?") from e
            monos = pose_monos[drone]
            monos.append(mono if _is_num(mono) else monos[-1])
            tracks[drone].poses.append(dr.pose)
            tracks[drone].action_names.append(data["action"])
        else:  # sighting — place at capture time when ts is usable
            ts = data.get("ts")
            monos = pose_monos[drone]
            if _is_num(ts):
                p = tracks[drone].poses[
                    max(0, bisect.bisect_right(monos, ts) - 1)]
            else:
                p = tracks[drone].poses[-1]   # fall back to row order
            label = str(data.get("class_name") or "sighting")
            if data.get("marker_id") is not None:
                label = f"{label}#{data['marker_id']}"
            bearing = data.get("bearing_deg")
            tracks[drone].sightings.append(SightingMark(
                east_m=p.east_m, north_m=p.north_m,
                bearing_deg=float(bearing) if _is_num(bearing) else None,
                label=label))
    if not tracks:
        raise ReplayPlotError(
            f"no drone origin/action events in {mission_path} — is this a "
            f"flight run dir? (replay-profile runs have no drones)")
    silent = sorted(seen - set(tracks))
    if silent:
        _warn(f"drone(s) {silent} appear in {mission_path} but logged no "
              f"origin (killed before connect?) — not plotted")
    return tracks


def heading_vector(yaw_deg: float) -> Tuple[float, float]:
    """Unit nose vector as (east, north) — the FORWARD row of
    DeadReckoner._integrate_move reordered for plotting. Test-pinned to the
    real DR so the quiver/ray math cannot drift from the track math."""
    theta = math.radians(yaw_deg)
    return (-math.sin(theta), math.cos(theta))


def _check_encoding(path: str) -> None:
    """Reject BOM'd/UTF-16 files TYPED: read_events would skip their lines
    one by one and the user would chase 'origin missing' ghosts. PowerShell
    5.1 plants both ('>' redirection = UTF-16; Out-File -Encoding utf8 =
    BOM) — a real trap for a Windows-side copy of a VM run."""
    with open(path, "rb") as f:
        head = f.read(3)
    if head[:2] in (b"\xff\xfe", b"\xfe\xff"):
        raise ReplayPlotError(
            f"{path} is UTF-16 (BOM {head[:2]!r}) — read_events needs plain "
            f"UTF-8 JSONL; PowerShell '>' redirection writes UTF-16, "
            f"re-copy the original file as bytes (scp/cp, not type/echo)")
    if head == b"\xef\xbb\xbf":
        raise ReplayPlotError(
            f"{path} starts with a UTF-8 BOM — its first line (often the "
            f"origin) would be skipped as torn; strip the BOM or re-copy "
            f"the original file as bytes (PowerShell Out-File adds BOMs)")


def resolve_mission_path(arg: str) -> str:
    """Run dir -> its mission.jsonl; a bare events file passes through."""
    if os.path.isdir(arg):
        path = os.path.join(arg, "mission.jsonl")
        if not os.path.isfile(path):
            raise ReplayPlotError(
                f"no mission.jsonl in {arg} — is it a run dir? it contains: "
                f"{sorted(os.listdir(arg))}")
    elif os.path.isfile(arg):
        path = arg
    else:
        raise ReplayPlotError(
            f"{arg!r} is neither a run dir nor an events file (cwd "
            f"{os.getcwd()}) — pass runs_finals/<ts> or a mission.jsonl path")
    _check_encoding(path)
    return path


def plot_tracks(tracks: Dict[str, DroneTrack], *, save: Optional[str],
                source: str):
    """One subplot per drone (frames differ — module docstring). Returns the
    figure so tests can pin the RENDERED geometry (axis order, quiver U/V,
    ray endpoints), not just that a PNG exists."""
    import matplotlib
    if save is not None:
        matplotlib.use("Agg", force=True)  # BEFORE pyplot import (headless)
    import matplotlib.pyplot as plt

    n = len(tracks)
    fig, axes = plt.subplots(1, n, figsize=(5.0 * n, 5.5), squeeze=False)
    for ax, track in zip(axes[0], tracks.values()):
        xs = [p.east_m for p in track.poses]
        ys = [p.north_m for p in track.poses]
        ax.plot(xs, ys, "-", color="tab:blue", linewidth=1.5, zorder=1)
        ax.plot(xs[0], ys[0], "o", color="tab:green", markersize=9,
                label="start", zorder=3)
        ax.plot(xs[-1], ys[-1], "s", color="tab:red", markersize=8,
                label="end", zorder=3)
        step = max(1, math.ceil(len(track.poses) / _QUIVER_MAX))
        idx = list(range(0, len(track.poses), step))
        if idx[-1] != len(track.poses) - 1:
            idx.append(len(track.poses) - 1)
        for i in idx:
            p = track.poses[i]
            de, dn = (c * _ARROW_LEN_M for c in heading_vector(p.yaw_deg))
            # angles/scale_units="xy", scale=1: arrows are exactly
            # _ARROW_LEN_M in data units — autoscaled quivers lie under
            # equal aspect and across subplots.
            ax.quiver(p.east_m, p.north_m, de, dn, angles="xy",
                      scale_units="xy", scale=1.0, width=0.008,
                      color="tab:orange", zorder=2)
        for s in track.sightings:
            ax.plot(s.east_m, s.north_m, "x", color="tab:purple",
                    markersize=8, zorder=4)
            if s.bearing_deg is not None:
                be, bn = heading_vector(s.bearing_deg)
                ax.plot([s.east_m, s.east_m + be * _BEARING_RAY_M],
                        [s.north_m, s.north_m + bn * _BEARING_RAY_M],
                        "--", color="tab:purple", linewidth=1.0, zorder=2)
            ax.annotate(s.label, (s.east_m, s.north_m), fontsize=7,
                        xytext=(3, 3), textcoords="offset points")
        last = track.poses[-1]
        ax.set_title(
            f"{track.drone_id} — {len(track.action_names)} actions\n"
            f"final N{last.north_m:+.2f} E{last.east_m:+.2f} "
            f"alt {last.alt_m:.2f} yaw {last.yaw_deg:+.1f}°", fontsize=9)
        ax.set_xlabel("east (m)")
        ax.set_ylabel("north (m)  [per-drone boot frame]")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, alpha=0.4)
        ax.legend(loc="best", fontsize=7)
    fig.suptitle(source, fontsize=9)
    fig.tight_layout()
    if save is not None:
        fig.savefig(save, dpi=150)
        plt.close(fig)
        print(f"[replay_plot] wrote {os.path.abspath(save)}")
    else:
        plt.show()
    return fig


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(
        prog="python -m finals.tools.replay_plot",
        description="Replay a run's mission.jsonl through the real "
                    "DeadReckoner and plot per-drone tracks.")
    p.add_argument("run_path",
                   help="run dir (runs_finals/<ts>) or a mission.jsonl path")
    p.add_argument("--save", metavar="OUT_PNG",
                   help="write a PNG instead of opening a window "
                        "(forces the Agg backend; required on headless VMs)")
    args = p.parse_args(argv)
    try:
        if args.save is not None:
            if not args.save:
                raise ReplayPlotError(
                    "--save needs a non-empty output path (e.g. out.png)")
            out_dir = os.path.dirname(os.path.abspath(args.save))
            if not os.path.isdir(out_dir):
                raise ReplayPlotError(
                    f"--save target dir {out_dir} does not exist — create it "
                    f"first (savefig would die with a raw FileNotFoundError)")
        path = resolve_mission_path(args.run_path)
        plot_tracks(build_tracks(path), save=args.save, source=path)
    except FinalsError as e:  # ReplayPlotError + EventLogError: typed, actionable
        print(f"replay_plot: {e}", file=sys.stderr, flush=True)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
