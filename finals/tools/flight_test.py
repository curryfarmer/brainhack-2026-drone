"""flight_test.py — one-command flight test for the HULA drone(s).

The unit test the operator asked for: the drone ASCENDS, SCANS for a landing
pad, and LANDS ITSELF on that pad. This is a thin, fail-loud LAUNCHER — it does
NOT re-implement any flight logic. It curates a config and hands off to
`finals.main`, so EVERY existing safety system runs unchanged:

`--drones {1,3}` selects the fleet (default 1 — the single-drone test is
byte-for-byte unchanged). `--drones 3` is the multi-drone sibling: 3 drones,
each running the SAME in-place `[takeoff, land_on_pad]` chain over its OWN pad,
deconflicted by TIME (the SafetyController launch + landing slots) + physical
placement (no drone translates). It requires ALL 3 drones + the laptop on ONE
shared Wi-Fi network with distinct IPs (discovery must find all 3) — the
solo-AP 192.168.100.1 path reaches only ONE drone and is bring-up only.

  - preflight P0-P10 (discovery, connect, telemetry/battery sanity, video, LEDs)
  - the default-deny operator GO prompt (type GO within 60 s or it refuses)
  - the SafetyController (landing slot + retry ladder, emergency_land)
  - the AbortListener ('q' + Enter -> LAND the drone, any time)
  - the guards (telemetry/video watchdogs, battery floor, mission clock)

The flight itself is the SITL-PROVEN phase chain `takeoff -> land_on_pad`:
TakeoffHold ascends and HOLDS; LandOnPad's PAD_ACQUIRE rotate-scans the FOV for
the pad marker, then centres (pixel servo), descends, and commits to a Land.
NO new flight behaviour ships here — only packaging.

SAFETY — this arms a REAL aircraft. Two gates, by design:
  1. `--live` is REQUIRED to actually fly. Without it the tool runs the plan in
     --dry-run (prints the resolved plan, exits 0, no props) so you can read
     exactly what WOULD happen.
  2. `finals.main` then runs preflight + the operator GO prompt before any arm.

SETUP (read before flying):
  - Place the test pad WITHIN the launch point's camera footprint. The scan is a
    rotate-in-place sweep; it cannot translate to a pad parked metres away.
  - The in-flight ArUco detector decodes DICT_7X7_1000 (the field dict) -> the
    test pad must carry a DICT_7X7_1000 marker, and --marker-id (or the config)
    must be ITS id.
  - Props off until you have rehearsed the abort ('q') at least once.

Usage:
  # Validate the plan, no hardware, no props (safe to run anywhere):
  python finals\\tools\\flight_test.py --dry-run

  # SITL rehearsal of the landing stack on the VM (needs the SITL stack up;
  # the canonical sim run is `bash sim/run_landing.sh land1`):
  python finals\\tools\\flight_test.py --sitl --dry-run

  # REAL flight (on the drone Wi-Fi, offline). --live is the safety gate:
  python finals\\tools\\flight_test.py --live --plane-id 6 --marker-id 7

  # 3-DRONE flight test — pass the drone codes at runtime with --plane-ids:
  python finals\\tools\\flight_test.py --drones 3 --plane-ids 7 10 12 --dry-run
  python finals\\tools\\flight_test.py --drones 3 --plane-ids 7 10 12 --live

Exit code is finals.main's: 0 ok | 1 error/FAILED | 2 config | 3 preflight
(or 2 from this launcher: missing config, or a single-drone override under
--drones 3).
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import sys
import tempfile
import time
from typing import Any, Dict, List, Optional

# Repo importable whether launched as a path or `-m finals.tools.flight_test`.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

_CONFIG_DIR = os.path.join(_REPO_ROOT, "finals", "configs")
#: The curated single-drone real-flight config (takeoff -> land_on_pad).
REAL_CONFIG = os.path.join(_CONFIG_DIR, "flight_test_real.json")
#: The curated 3-drone real-flight config (--drones 3): the SIMPLER sibling of
#: landing_real.json — 3 drones, each phases [takeoff, land_on_pad], NO navigate
#: and NO arena (pure in-place scan per drone). Deconfliction is TIME (the
#: SafetyController launch + landing slots) + physical PLACEMENT; spatial safety
#: is NOT from config since no drone translates. All 3 drones + the laptop must
#: share ONE Wi-Fi network with distinct IPs (discovery must find all 3).
REAL_3X_CONFIG = os.path.join(_CONFIG_DIR, "flight_test_3x_real.json")
#: SITL rehearsal reuses the PROVEN L1 landing config (takeoff -> navigate ->
#: land_on_pad in the landing world). No bespoke sim world to maintain; the
#: canonical sim run is `bash sim/run_landing.sh land1`, which brings up PX4 +
#: gz + the camera bridge that this profile needs. --sitl here is for dry-run
#: validation and for running finals.main when that stack is already up.
SITL_CONFIG = os.path.join(_CONFIG_DIR, "sitl1_landing.json")
#: The 3-drone SITL landing rehearsal (--sitl --drones 3) — reuses the PROVEN
#: gate-L2 sitl3_landing.json (3 concurrent PX4 camera-drones, staggered +
#: serialized landing). No bespoke 3x sim config to maintain.
SITL_3X_CONFIG = os.path.join(_CONFIG_DIR, "sitl3_landing.json")


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="flight_test",
        description="HULA flight test: ascend -> scan for a landing pad -> land "
                    "on it. Thin launcher over finals.main (all safety systems "
                    "reused). --live is required to fly. --drones {1,3}: 1 (the "
                    "default) is the single-drone test; 3 is the multi-drone "
                    "in-place-scan sibling (all 3 on ONE shared Wi-Fi network).")
    p.add_argument("--drones", type=int, choices=(1, 3), default=1,
                   help="how many drones to fly: 1 (default — the single-drone "
                        "test, unchanged) or 3 (the 3-drone in-place-scan test; "
                        "real -> flight_test_3x_real.json, --sitl -> "
                        "sitl3_landing.json). The single-value overrides "
                        "(--plane-id/--marker-id/--height-cm) are 1-drone only "
                        "and are REFUSED with --drones 3.")
    p.add_argument("--live", action="store_true",
                   help="ACTUALLY FLY (arms a real aircraft). Without it the "
                        "run is forced to --dry-run (plan only, no props).")
    p.add_argument("--sitl", action="store_true",
                   help="rehearse the landing stack in SITL (uses the proven "
                        "sitl1_landing.json; needs the SITL stack up — see "
                        "sim/run_landing.sh). No real aircraft, so --live is "
                        "not required.")
    p.add_argument("--dry-run", action="store_true",
                   help="resolve config + print the plan and exit 0 (never "
                        "flies); implied for a real run without --live.")
    p.add_argument("--plane-id", type=int, default=None,
                   help="override the single drone's Dola plane_id (1-drone "
                        "alias for --plane-ids)")
    p.add_argument("--plane-ids", nargs="+", default=None, metavar="ID",
                   help="set each drone's Dola plane_id in fleet order, e.g. "
                        "--plane-ids 7 10 12 (also accepts 7,10,12). The runtime "
                        "way to enter the drone codes without editing the config; "
                        "the count MUST match the config's drone count. Unlike "
                        "--plane-id, this is allowed with --drones 3.")
    p.add_argument("--marker-id", type=int, default=None,
                   help="override the pad's valid ArUco marker id "
                        "(land_on_pad.valid_marker_ids = [this])")
    p.add_argument("--height-cm", type=int, default=None,
                   help="override the takeoff height in cm")
    p.add_argument("--budget", type=float, default=None,
                   help="mission wall-clock budget override (s)")
    p.add_argument("--config", default=None,
                   help="explicit base config path (overrides the real/sitl default)")
    return p.parse_args(argv)


def base_config_path(args: argparse.Namespace) -> str:
    """The config the launch starts from: an explicit --config always wins;
    else the curated default selected by (--sitl, --drones):
      drones==1: --sitl -> sitl1_landing.json   else flight_test_real.json
      drones==3: --sitl -> sitl3_landing.json   else flight_test_3x_real.json"""
    if args.config:
        return args.config
    if args.drones == 3:
        return SITL_3X_CONFIG if args.sitl else REAL_3X_CONFIG
    return SITL_CONFIG if args.sitl else REAL_CONFIG


def _parse_plane_ids(raw: Optional[List[str]]) -> Optional[List[int]]:
    """Parse --plane-ids tokens into ints. Accepts space- AND comma-separated
    (['7','10','12'] or ['7,10,12'] or ['7,', '10', '12']). Raises ValueError
    (loud) on a non-integer token."""
    if not raw:
        return None
    out: List[int] = []
    for tok in raw:
        for piece in tok.replace(",", " ").split():
            out.append(int(piece))
    return out or None


def _single_overrides(args: argparse.Namespace) -> bool:
    """The 1-drone-only overrides — they patch ONLY drones[0]."""
    return any(v is not None
               for v in (args.plane_id, args.marker_id, args.height_cm))


def needs_patch(args: argparse.Namespace) -> bool:
    """True iff a CLI override has to be folded into the config JSON (finals.main
    has no CLI hook for plane_id(s) / marker_id / height_cm — only --budget, which
    we pass through natively)."""
    return _single_overrides(args) or args.plane_ids is not None


def patch_config(base: Dict[str, Any], *, plane_id: Optional[int] = None,
                 plane_ids: Optional[List[int]] = None,
                 marker_id: Optional[int] = None,
                 height_cm: Optional[int] = None) -> Dict[str, Any]:
    """Return a deep copy of the config with drone plane_id(s) / takeoff height /
    valid pad marker overridden. plane_ids sets EVERY drone's plane_id in fleet
    order (the runtime drone-code input); plane_id / marker_id / height_cm patch
    ONLY the first drone. Loud if the config has no drone to patch, or if the
    plane_ids count does not match the fleet size."""
    cfg = copy.deepcopy(base)
    drones = cfg.get("drones")
    if not isinstance(drones, list) or not drones:
        raise ValueError(
            "flight_test: the base config has no drone to patch — a flight "
            "test needs at least one drone; CHECK the --config file "
            "(expected a 'drones' list)")
    if plane_ids is not None:
        if len(plane_ids) != len(drones):
            raise ValueError(
                f"flight_test: --plane-ids has {len(plane_ids)} id(s) "
                f"({plane_ids}) but the config has {len(drones)} drone(s) — "
                f"pass one plane_id per drone, in fleet order.")
        for d, pid in zip(drones, plane_ids):
            d["plane_id"] = pid
    drone = drones[0]
    if plane_id is not None:
        drone["plane_id"] = plane_id
    zone = drone.setdefault("zone", {})
    if height_cm is not None:
        zone.setdefault("takeoff", {})["height_cm"] = height_cm
    if marker_id is not None:
        zone.setdefault("land_on_pad", {})["valid_marker_ids"] = [marker_id]
    return cfg


def build_finals_argv(args: argparse.Namespace, config_path: str) -> List[str]:
    """The argv handed to finals.main.main. Encodes the SAFETY gating: a real
    profile always carries --i-know-this-arms-real-drones, and a real run WITHOUT
    --live (or any --dry-run) is forced to --dry-run so nothing arms by accident.
    SITL never arms a real aircraft, so --live is not required there."""
    profile = "sitl" if args.sitl else "real"
    argv = ["--profile", profile, "--config", config_path]
    if profile == "real":
        argv.append("--i-know-this-arms-real-drones")
    # Double-gate: real flight requires --live; otherwise plan only.
    force_dry = args.dry_run or (profile == "real" and not args.live)
    if force_dry:
        argv.append("--dry-run")
    if args.budget is not None:
        argv += ["--budget", str(args.budget)]
    return argv


def _write_resolved_config(cfg: Dict[str, Any]) -> str:
    """Write a patched config to a temp file finals.main can load; return its
    path. Kept (delete=False) for forensics — the path is printed."""
    fd, path = tempfile.mkstemp(prefix="flight_test_resolved_", suffix=".json")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)
    return path


def _print_preamble(armed: bool, profile: str, config_path: str,
                    finals_argv: List[str], drones: int = 1) -> None:
    bar = "=" * 72
    fleet = f"{drones}-drone" if drones != 1 else "single-drone"
    print(bar)
    print(f"  HULA FLIGHT TEST ({fleet})  —  ascend -> scan for pad -> land on pad")
    print(f"  profile={profile}   drones={drones}   config={config_path}")
    print(bar)
    if not armed:
        print("  PLAN ONLY (no props). Add --live to ACTUALLY FLY a real drone.")
        if drones == 3:
            print("  3x LIVE prereq: all 3 drones + the laptop on ONE shared "
                  "Wi-Fi network")
            print("                  with DISTINCT IPs — discovery (Dola) must "
                  "find all 3.")
        print(f"  finals.main {' '.join(finals_argv)}")
        print(bar)
        return
    # The honest, multi-step safety brief (drop terseness — flight is live).
    print(f"  LIVE FLIGHT — {drones} real aircraft about to ARM.")
    print("    1. Clear the area; props will spin.")
    print("    2. finals.main will run preflight P0-P10, then prompt: type GO.")
    print("    3. Abort ANY time: press 'q' then Enter -> the drone(s) land.")
    print("    4. Place each pad in its drone's camera view; each carries a")
    print("       DICT_7X7_1000 marker whose id matches that drone's "
          "land_on_pad.valid_marker_ids.")
    if drones == 3:
        print("    5. All 3 drones + the laptop MUST be on ONE shared Wi-Fi "
              "network with")
        print("       DISTINCT IPs — discovery (Dola) must find all 3 (the "
              "solo-AP path")
        print("       192.168.100.1 reaches only ONE drone).")
    print(f"  finals.main {' '.join(finals_argv)}")
    print(bar)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    # The single-value overrides patch ONE drone — they are meaningless (and
    # would silently mis-patch only the first of three) in 3x mode. Refuse loud
    # (exit 2). --plane-ids is the fleet-wide path and IS allowed with --drones 3.
    if args.drones == 3 and _single_overrides(args):
        print(
            "\nflight_test ERROR: --plane-id/--marker-id/--height-cm are "
            "single-drone overrides; for --drones 3 use --plane-ids 7 10 12 "
            "(one per drone), or edit finals/configs/flight_test_3x_real.json.\n",
            file=sys.stderr)
        return 2
    try:
        fleet_ids = _parse_plane_ids(args.plane_ids)
    except ValueError:
        print(f"\nflight_test ERROR: --plane-ids must be integer drone codes, "
              f"got {args.plane_ids!r}.\n", file=sys.stderr)
        return 2
    base_path = base_config_path(args)
    if not os.path.isfile(base_path):
        print(f"\nflight_test ERROR: config not found: {base_path!r} — run from "
              f"the repo root, or pass --config.\n", file=sys.stderr)
        return 2

    config_path = base_path
    if needs_patch(args):
        with open(base_path, "r", encoding="utf-8") as f:
            base = json.load(f)
        try:
            patched = patch_config(base, plane_id=args.plane_id,
                                   plane_ids=fleet_ids,
                                   marker_id=args.marker_id,
                                   height_cm=args.height_cm)
        except ValueError as e:
            print(f"\n{e}\n", file=sys.stderr)
            return 2
        config_path = _write_resolved_config(patched)

    finals_argv = build_finals_argv(args, config_path)
    profile = "sitl" if args.sitl else "real"
    armed = (profile == "real") and args.live and not args.dry_run
    if config_path != base_path:
        print(f"flight_test: resolved config written to {config_path}")
    _print_preamble(armed, profile, config_path, finals_argv,
                    drones=args.drones)

    if armed:
        # A last beat for the operator to read the brief / ctrl-C out before
        # finals.main starts the preflight gate.
        print("  arming finals.main in 3 s — ctrl-C to cancel...")
        time.sleep(3.0)

    from finals.main import main as finals_main
    return finals_main(finals_argv)


if __name__ == "__main__":
    raise SystemExit(main())
