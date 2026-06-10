"""calibrate_origin — turn measured cage numbers into a CHECKABLE origin frame.

The ORIGIN-CAL gate-D aid (post-simplification: every pad is landable, we pick
the coords; no per-pad beacon). Feed it an arena JSON (the measured cage: corner
origin, bounds, c2_origin, chosen pad coords, heading_offset_deg) and it:

  1. VALIDATES the arena through the REAL loader (ArenaMap.from_dict) — the same
     bounds-ordering / pads-in-bounds / origin-in-bounds / unique-id rules the
     flight config enforces, so a fat-fingered measurement dies HERE, on the
     ground, not as a mid-air mis-plan.
  2. Plans each drone's transit C2 -> its assigned pad over the REAL planner
     (visibility_graph.plan) and prints a per-drone CALIBRATION CARD: the
     straight-line bearing + distance, and every Leg's arena heading, the
     compass yaw the operator should READ after the heading_offset_deg (Delta)
     correction, and the leg distance. Onsite you eyeball each Rotate/Move
     against the card.
  3. Renders a TOP-DOWN view — a matplotlib PNG (with --save; skipped cleanly if
     matplotlib is absent) AND an ASCII map that ALWAYS prints — of the corner
     origin, the N/E axes, the bounds rectangle, C2, the pads, and any keep-outs.

It never reimplements the geometry: the leg headings/distances come from
visibility_graph.plan (the binding heading convention, pinned against the REAL
DeadReckoner), the offset bake matches navigate.from_config verbatim
(wrap180(heading + Delta)), and the bearing from frame.bearing_from_c2_deg. A
divergence here would bless a wrong calibration, so there is one source of truth.

Per-drone pad assignments + transit tunables (inflation_m, max_leg_cm) are read
from a landing config JSON (--config, default finals/configs/landing_real.json):
each drone's zone["navigate"] names its goal (pad_id / goal_ne_m / marker_id),
exactly as the flight does. With no config (or --all-pads) it cards every pad
from C2 with default tunables.

CLI:
  python -m finals.tools.calibrate_origin [ARENA_JSON]
      [--config LANDING_JSON] [--all-pads]
      [--inflation-m M] [--max-leg-cm CM]
      [--save OUT_PNG] [--no-plot]

Defaults: ARENA_JSON = finals/configs/arenas/cage.json,
          --config   = finals/configs/landing_real.json.

Pure stdlib + (lazily) matplotlib; no cv2/gz/numpy — finals/tools/ is inside the
conventions scan. The planner / frame / loader it imports are all pure-stdlib.

Session: ORIGIN-CAL (implemented).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

from finals.errors import FinalsError
from finals.mission.phases._servo import wrap180
from finals.mission.planning.frame import bearing_from_c2_deg
from finals.mission.planning.types import ArenaMap, Leg
from finals.mission.planning.visibility_graph import plan

_DEFAULT_ARENA = os.path.join("finals", "configs", "arenas", "cage.json")
_DEFAULT_CONFIG = os.path.join("finals", "configs", "landing_real.json")
_DEFAULT_INFLATION_M = 0.5
_DEFAULT_MAX_LEG_CM = 100.0


class CalibrateError(FinalsError):
    """A calibration input is missing/malformed. Message names the path + fix."""


# ---------------------------------------------------------------------------
# Card model
# ---------------------------------------------------------------------------
class DroneCard:
    """One drone's transit calibration: the assigned goal + the planned legs (or
    a failure reason if the goal is unreachable)."""

    def __init__(self, label: str, goal_desc: str, goal_m: Tuple[float, float],
                 legs: Optional[Tuple[Leg, ...]], error: Optional[str]):
        self.label = label
        self.goal_desc = goal_desc
        self.goal_m = goal_m
        self.legs = legs
        self.error = error


def _load_json(path: str, what: str) -> Any:
    if not os.path.isfile(path):
        raise CalibrateError(
            f"{what} {path!r} does not exist — pass the path explicitly or "
            f"create it (calibrate_origin reads the MEASURED cage numbers from "
            f"the arena JSON).")
    try:
        with open(path, "r", encoding="utf-8") as fh:
            return json.load(fh)
    except (OSError, ValueError) as e:
        raise CalibrateError(f"{what} {path!r} is not readable JSON: {e}")


def _assignments_from_config(cfg_raw: Any, arena: ArenaMap, path: str,
                             inflation_default: float, max_leg_default: float
                             ) -> List[Tuple[str, Tuple[float, float], str,
                                              float, float]]:
    """Pull (label, goal_m, goal_desc, inflation_m, max_leg_cm) per drone from a
    landing config's drones[*].zone["navigate"]. Mirrors navigate.from_config's
    goal resolution (EXACTLY ONE of pad_id / goal_ne_m / marker_id) so the card
    targets the same point the flight would. A bad assignment raises loudly —
    the operator is calibrating, a silent wrong goal defeats the point."""
    drones = cfg_raw.get("drones") if isinstance(cfg_raw, dict) else None
    if not isinstance(drones, list) or not drones:
        raise CalibrateError(
            f"config {path!r} has no non-empty 'drones' list — cannot read "
            f"per-drone pad assignments. Use --all-pads to card every pad "
            f"instead.")
    pads = {p.id: p for p in arena.pads}
    markers = {m.id: m for m in arena.markers}
    out: List[Tuple[str, Tuple[float, float], str, float, float]] = []
    for i, d in enumerate(drones):
        if not isinstance(d, dict):
            raise CalibrateError(f"config {path!r}: drones[{i}] is not an object")
        label = str(d.get("id", f"drone{i}"))
        zone = d.get("zone", {})
        nav = zone.get("navigate", {}) if isinstance(zone, dict) else {}
        if not isinstance(nav, dict):
            raise CalibrateError(
                f"config {path!r}: {label} zone['navigate'] is not an object")
        infl = float(nav.get("inflation_m", inflation_default))
        max_leg = float(nav.get("max_leg_cm", max_leg_default))
        sources = [k for k in ("pad_id", "goal_ne_m", "marker_id") if k in nav]
        if len(sources) != 1:
            raise CalibrateError(
                f"config {path!r}: {label} zone['navigate'] names {sources or 'NO'} "
                f"goal source(s) — give EXACTLY ONE of pad_id / goal_ne_m / "
                f"marker_id (same rule as navigate).")
        if "pad_id" in nav:
            pid = nav["pad_id"]
            if pid not in pads:
                raise CalibrateError(
                    f"config {path!r}: {label} pad_id {pid!r} is not a pad in "
                    f"this arena — available: {sorted(pads)}.")
            goal_m = pads[pid].center_m
            goal_desc = f"pad {pid!r} {tuple(round(c, 2) for c in goal_m)}"
        elif "marker_id" in nav:
            mid = nav["marker_id"]
            if mid not in markers:
                raise CalibrateError(
                    f"config {path!r}: {label} marker_id {mid!r} is not a beacon "
                    f"in this arena — available: {sorted(markers)}.")
            goal_m = markers[mid].point_m
            goal_desc = f"beacon {mid} {tuple(round(c, 2) for c in goal_m)}"
        else:
            raw = nav["goal_ne_m"]
            if (not isinstance(raw, (list, tuple)) or len(raw) != 2
                    or any(not isinstance(c, (int, float)) or isinstance(c, bool)
                           for c in raw)):
                raise CalibrateError(
                    f"config {path!r}: {label} goal_ne_m must be "
                    f"[north_m, east_m] numbers, got {raw!r}")
            goal_m = (float(raw[0]), float(raw[1]))
            goal_desc = f"goal_ne_m {tuple(round(c, 2) for c in goal_m)}"
        out.append((label, goal_m, goal_desc, infl, max_leg))
    return out


def _assignments_all_pads(arena: ArenaMap, inflation_default: float,
                          max_leg_default: float
                          ) -> List[Tuple[str, Tuple[float, float], str,
                                           float, float]]:
    """Fallback: one card per pad, labelled by pad id, default tunables."""
    return [(p.id, p.center_m, f"pad {p.id!r} {tuple(round(c, 2) for c in p.center_m)}",
             inflation_default, max_leg_default)
            for p in arena.pads]


def build_cards(arena: ArenaMap,
                assignments: List[Tuple[str, Tuple[float, float], str,
                                        float, float]]) -> List[DroneCard]:
    """Plan each assignment over the REAL planner; on PlanningError record the
    reason instead of crashing (so one trapped pad does not blank the rest)."""
    cards: List[DroneCard] = []
    for label, goal_m, goal_desc, infl, max_leg in assignments:
        try:
            legs = plan(arena.c2_origin_m, goal_m, arena, infl, max_leg)
            cards.append(DroneCard(label, goal_desc, goal_m, tuple(legs), None))
        except FinalsError as e:        # PlanningError: trapped / unreachable
            cards.append(DroneCard(label, goal_desc, goal_m, None, str(e)))
        except ValueError as e:         # out-of-domain inflation/max_leg
            cards.append(DroneCard(label, goal_desc, goal_m, None, str(e)))
    return cards


# ---------------------------------------------------------------------------
# Text card
# ---------------------------------------------------------------------------
def format_cards(arena: ArenaMap, cards: List[DroneCard], *,
                 arena_name: str) -> str:
    off = arena.heading_offset_deg
    lines: List[str] = []
    lines.append(f"ORIGIN-CAL calibration — arena {arena_name!r}")
    n0, e0, n1, e1 = arena.bounds_m
    lines.append(
        f"  cage bounds : north {n0:.2f}..{n1:.2f} m (long {n1 - n0:.2f}), "
        f"east {e0:.2f}..{e1:.2f} m (short {e1 - e0:.2f})")
    lines.append(f"  origin      : corner (0,0); +north along long wall, "
                 f"+east along short wall")
    lines.append(f"  C2 launch   : N{arena.c2_origin_m[0]:.2f} "
                 f"E{arena.c2_origin_m[1]:.2f} m  "
                 f"(c2_heading_deg {arena.c2_heading_deg:+.1f} advisory)")
    lines.append(
        f"  heading_offset_deg (Delta) : {off:+.2f} deg  "
        f"[compass-yaw - arena-north; the Rotate target = arena heading + Delta]")
    if off == 0.0:
        lines.append("    (Delta = 0 -> Rotate target == arena heading; set it "
                     "from the boot-yaw reading at gate D)")
    lines.append("")
    for card in cards:
        lines.append(f"[{card.label}] -> {card.goal_desc}")
        if card.error is not None:
            lines.append(f"    UNREACHABLE: {card.error}")
            lines.append("")
            continue
        bearing = bearing_from_c2_deg(card.goal_m, arena.c2_origin_m)
        dn = card.goal_m[0] - arena.c2_origin_m[0]
        de = card.goal_m[1] - arena.c2_origin_m[1]
        straight = math.hypot(dn, de)
        legs = card.legs or ()
        total_cm = sum(l.distance_cm for l in legs)
        lines.append(
            f"    straight line : bearing {bearing:+.1f} deg "
            f"(read {wrap180(bearing + off):+.1f} on compass), "
            f"distance {straight:.2f} m")
        lines.append(
            f"    planned       : {len(legs)} leg(s), path {total_cm / 100.0:.2f} m")
        for j, leg in enumerate(legs):
            target = wrap180(leg.heading_deg + off)
            lines.append(
                f"      leg {j + 1:>2}: rotate to arena {leg.heading_deg:+7.1f} "
                f"deg  (compass reads {target:+7.1f})   move {leg.distance_cm / 100.0:5.2f} m")
        lines.append("")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# ASCII top-down (always available)
# ---------------------------------------------------------------------------
def ascii_map(arena: ArenaMap, cards: List[DroneCard], *, rows: int = 22) -> str:
    """A coarse top-down sketch: north UP (rows), east RIGHT (cols), the corner
    origin at the BOTTOM-LEFT. C2 = '*', pads = 'o' (first letter of id below),
    keep-out vertices = '#'. The PNG is the precise view; this is the always-on
    terminal aid (no matplotlib needed)."""
    n0, e0, n1, e1 = arena.bounds_m
    nspan = n1 - n0
    espan = e1 - e0
    rows = max(8, min(rows, 40))
    # chars are ~2x taller than wide -> double the east scale for a square look.
    cols = max(8, min(60, int(round(rows * (espan / nspan) * 2.0)))) if nspan else 20
    grid = [[" " for _ in range(cols)] for _ in range(rows)]

    def cell(pt: Tuple[float, float]) -> Optional[Tuple[int, int]]:
        if nspan <= 0 or espan <= 0:
            return None
        fn = (pt[0] - n0) / nspan
        fe = (pt[1] - e0) / espan
        if not (0.0 <= fn <= 1.0 and 0.0 <= fe <= 1.0):
            return None
        r = rows - 1 - min(rows - 1, int(round(fn * (rows - 1))))
        c = min(cols - 1, int(round(fe * (cols - 1))))
        return r, c

    # border
    for c in range(cols):
        grid[0][c] = grid[rows - 1][c] = "-"
    for r in range(rows):
        grid[r][0] = grid[r][cols - 1] = "|"
    for r, c in ((0, 0), (0, cols - 1), (rows - 1, 0), (rows - 1, cols - 1)):
        grid[r][c] = "+"
    # keep-out vertices
    for k in arena.keep_out:
        for v in k.polygon_m:
            rc = cell(v)
            if rc:
                grid[rc[0]][rc[1]] = "#"
    # pads
    for p in arena.pads:
        rc = cell(p.center_m)
        if rc:
            grid[rc[0]][rc[1]] = "o"
    # C2 last (most important, wins a tie)
    rc = cell(arena.c2_origin_m)
    if rc:
        grid[rc[0]][rc[1]] = "*"

    out = ["  N^  (east ->)   * = C2   o = pad   # = keep-out"]
    out += ["  " + "".join(r) for r in grid]
    out.append("  (0,0) corner at bottom-left; "
               f"grid {nspan:.1f} m N x {espan:.1f} m E")
    return "\n".join(out)


# ---------------------------------------------------------------------------
# matplotlib top-down (optional)
# ---------------------------------------------------------------------------
def plot_map(arena: ArenaMap, cards: List[DroneCard], *, save: str,
             arena_name: str):
    """Precise top-down PNG: bounds, corner origin + N/E axes, C2, pads, keep-out
    polygons, and each drone's planned polyline. matplotlib is imported lazily
    (Agg backend before pyplot) so the tool stays importable without it."""
    import matplotlib
    matplotlib.use("Agg", force=True)   # headless: pick backend before pyplot
    import matplotlib.pyplot as plt

    n0, e0, n1, e1 = arena.bounds_m
    fig, ax = plt.subplots(figsize=(6.5, 6.5 * max(0.4, (n1 - n0) / (e1 - e0))))
    # bounds rectangle (east on X, north on Y)
    ax.plot([e0, e1, e1, e0, e0], [n0, n0, n1, n1, n0], "-",
            color="0.5", linewidth=1.2, zorder=1)
    # corner origin + axes
    ax.plot(e0, n0, "+", color="black", markersize=12, zorder=5)
    ax.annotate("(0,0)", (e0, n0), fontsize=8, xytext=(4, 4),
                textcoords="offset points")
    ax.annotate("", xy=(e0, n0 + 0.18 * (n1 - n0)), xytext=(e0, n0),
                arrowprops=dict(arrowstyle="->", color="tab:green"))
    ax.annotate("N", (e0, n0 + 0.18 * (n1 - n0)), color="tab:green", fontsize=9)
    ax.annotate("", xy=(e0 + 0.18 * (e1 - e0), n0), xytext=(e0, n0),
                arrowprops=dict(arrowstyle="->", color="tab:blue"))
    ax.annotate("E", (e0 + 0.18 * (e1 - e0), n0), color="tab:blue", fontsize=9)
    # keep-outs
    for k in arena.keep_out:
        ks = list(k.polygon_m) + [k.polygon_m[0]]
        ax.fill([p[1] for p in ks], [p[0] for p in ks], color="tab:red",
                alpha=0.25, zorder=2)
        ax.plot([p[1] for p in ks], [p[0] for p in ks], "-",
                color="tab:red", linewidth=1.0, zorder=2)
    # C2
    ax.plot(arena.c2_origin_m[1], arena.c2_origin_m[0], "*", color="tab:orange",
            markersize=16, zorder=6, label="C2")
    # pads
    for p in arena.pads:
        ax.plot(p.center_m[1], p.center_m[0], "o", color="tab:green",
                markersize=10, zorder=5)
        ax.annotate(p.id, (p.center_m[1], p.center_m[0]), fontsize=7,
                    xytext=(5, 2), textcoords="offset points")
    # planned routes
    for card in cards:
        if not card.legs:
            continue
        pts = _route_points(arena.c2_origin_m, card.legs, arena.heading_offset_deg)
        ax.plot([q[1] for q in pts], [q[0] for q in pts], "--", linewidth=1.3,
                zorder=4, label=card.label)
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.set_title(f"ORIGIN-CAL top-down — arena {arena_name!r}", fontsize=10)
    ax.legend(loc="best", fontsize=7)
    fig.tight_layout()
    fig.savefig(save, dpi=150)
    plt.close(fig)
    print(f"[calibrate_origin] wrote {os.path.abspath(save)}")
    return fig


def _route_points(start_m: Tuple[float, float], legs: Sequence[Leg],
                  offset_deg: float) -> List[Tuple[float, float]]:
    """Re-integrate the planned legs into (north,east) waypoints for the plot.
    Uses the SAME forward map as dead_reckon (a FORWARD move of d at heading h
    advances (d*cos h, -d*sin h)); legs are in the arena frame so offset is NOT
    re-applied here (it only shifts the compass Rotate target, not the geometry)."""
    pts = [start_m]
    n, e = start_m
    for leg in legs:
        h = math.radians(leg.heading_deg)
        d = leg.distance_cm / 100.0
        n += d * math.cos(h)
        e += -d * math.sin(h)
        pts.append((n, e))
    return pts


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def run(arena_path: str, *, config_path: Optional[str], all_pads: bool,
        inflation_m: float, max_leg_cm: float, save: Optional[str],
        no_plot: bool) -> str:
    """Validate + card + render; return the full text report (card + ascii)."""
    arena_raw = _load_json(arena_path, "arena JSON")
    arena_name = os.path.splitext(os.path.basename(arena_path))[0]
    # The REAL loader = the validation. A bad measurement dies here.
    arena = ArenaMap.from_dict(arena_raw, name=arena_name)

    if all_pads or config_path is None:
        assignments = _assignments_all_pads(arena, inflation_m, max_leg_cm)
    else:
        cfg_raw = _load_json(config_path, "config JSON")
        assignments = _assignments_from_config(
            cfg_raw, arena, config_path, inflation_m, max_leg_cm)

    cards = build_cards(arena, assignments)
    report = format_cards(arena, cards, arena_name=arena_name)
    report += "\n" + ascii_map(arena, cards)

    if save is not None and not no_plot:
        try:
            plot_map(arena, cards, save=save, arena_name=arena_name)
        except ImportError:
            print("[calibrate_origin] matplotlib not installed — skipped PNG "
                  "(ASCII map above is the fallback).", file=sys.stderr)
    return report


def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(
        prog="python -m finals.tools.calibrate_origin",
        description="Validate a measured cage arena + print per-drone "
                    "heading/distance calibration cards + a top-down view.")
    p.add_argument("arena", nargs="?", default=_DEFAULT_ARENA,
                   help=f"arena JSON (default {_DEFAULT_ARENA})")
    p.add_argument("--config", default=_DEFAULT_CONFIG,
                   help=f"landing config JSON for per-drone pad assignments "
                        f"(default {_DEFAULT_CONFIG}); ignored with --all-pads")
    p.add_argument("--all-pads", action="store_true",
                   help="card every pad from C2 (ignore --config assignments)")
    p.add_argument("--inflation-m", type=float, default=_DEFAULT_INFLATION_M,
                   help=f"keep-out inflation fallback (default "
                        f"{_DEFAULT_INFLATION_M}); per-drone config overrides")
    p.add_argument("--max-leg-cm", type=float, default=_DEFAULT_MAX_LEG_CM,
                   help=f"max leg length fallback cm (default {_DEFAULT_MAX_LEG_CM})")
    p.add_argument("--save", metavar="OUT_PNG",
                   help="write a top-down PNG (needs matplotlib; ASCII always "
                        "prints regardless)")
    p.add_argument("--no-plot", action="store_true",
                   help="skip the PNG even if --save is given")
    args = p.parse_args(argv)
    try:
        if args.save is not None:
            if not args.save:
                raise CalibrateError("--save needs a non-empty output path")
            out_dir = os.path.dirname(os.path.abspath(args.save))
            if not os.path.isdir(out_dir):
                raise CalibrateError(
                    f"--save target dir {out_dir} does not exist — create it "
                    f"first (savefig would die with a raw FileNotFoundError).")
        report = run(args.arena, config_path=args.config,
                     all_pads=args.all_pads, inflation_m=args.inflation_m,
                     max_leg_cm=args.max_leg_cm, save=args.save,
                     no_plot=args.no_plot)
    except FinalsError as e:
        print(f"calibrate_origin: {e}", file=sys.stderr, flush=True)
        return 2
    print(report)
    return 0


if __name__ == "__main__":   # pragma: no cover
    sys.exit(main())
