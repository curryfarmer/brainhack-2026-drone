"""verify_runbook — assert the onsite RUNBOOK and the code are in SYNC, fail LOUD
on any drift. The single-purpose CI guard for docs/finals/onsite_test_plan.md.

WHY this exists (WS-7B): the runbook is the 2-hour onsite run-card — every config
it names, every phase it mentions, every P0..P10 gate it tabulates, and every SDF
crate the sims fly past is a promise. Promises rot: a new config (the WS-4
followbox / WS-5 dyn sims) the runbook forgot, an SDF crate whose footprint no
longer matches the arena keep-out the planner detours around, a phase renamed out
from under a gate row — each is a silent landmine an onsite operator steps on at
the worst possible moment. This tool turns each into a loud, local, exit-non-zero
assertion BEFORE the hardware window.

The checks (each an independent function -> CheckResult; main() composes + reports):

STATIC (always run, fast, no hardware):
  - suite        : `python -m pytest finals/tests -q -p no:randomly` is GREEN, and
                   (where cv2 is importable) the cv2-gated tests are NOT all
                   silently skipped — a whole-suite skip is itself drift.
  - configs      : every shipped finals/configs/*.json loads + validates clean
                   (load_config; the same path --dry-run exercises) AND the real
                   bench config passes the preflight static gate P0 (config sanity:
                   distinct plane_ids / bands-or-sectors / frame_backend pyhulax).
  - drift        : the high-value detector —
                     (a) every config/arena/world the runbook LINKS exists on disk;
                     (b) every shipped finals/configs/*.json is REFERENCED by the
                         runbook (an orphan config the runbook forgot is drift);
                     (c) every runbook-named phase is in PHASE_REGISTRY;
                     (d) the P0..P10 rows in the runbook == the gate ids
                         finals.preflight.run_preflight actually runs;
                     (e) SDF<->keep-out: each crate-bearing SDF world's
                         <pose>+<box><size> footprint has a MATCHING arena keep-out
                         polygon in its paired arena JSON (the WS-4 manual SDF/arena
                         sync, now a CI assertion). gz(x=E,y=N) -> arena(north=y,
                         east=x).

SMOKE (VM-only, gated behind --smoke, default OFF — useful without hardware):
  - land1  -> the run logs `navigate` complete;
  - track3 / followbox1 -> sightings.csv has rows > 0;
  - warm-up sims -> navigate logs >= 2 legs.
  These are DOCUMENTED here and asserted only when --smoke + a --run-dir of real
  evidence is supplied; CI never requires them.

VERDICT: PASS iff suite GREEN and all configs load and ZERO unflagged drift
(smokes optional). On any failure each result prints WHAT / WHICH / WHY / CHECK
and main() exits non-zero. Exit 0 only on PASS.

Pure stdlib at import (subprocess/json/re/xml.etree/glob/os/argparse/dataclasses):
finals/tools/ is inside the conventions SDK scan, so NO cv2/gz/numpy/mavsdk import
here — the pytest run + any smoke is a SUBPROCESS, and the SDF parse is xml.etree.

CLI: python -m finals.tools.verify_runbook [--smoke --run-dir <dir>] [--no-suite]

Session: WS-7B (implemented).
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# Repo-root-relative anchors. This file is finals/tools/verify_runbook.py, so the
# repo root is three parents up. Every path below is built from REPO so the tool
# runs from any CWD (and the tests can point it at a fixture tree).
_THIS = os.path.abspath(__file__)
REPO = os.path.dirname(os.path.dirname(os.path.dirname(_THIS)))

RUNBOOK_REL = os.path.join("docs", "finals", "onsite_test_plan.md")
CONFIGS_GLOB = os.path.join("finals", "configs", "*.json")
ARENAS_DIR = os.path.join("finals", "configs", "arenas")
WORLDS_DIR = os.path.join("sim", "worlds")

#: The config the static preflight P0 gate is run against (the real bench fleet —
#: the one whose duplicate-plane_id / non-distinct-band / wrong-frame-backend drift
#: P0 is built to catch). bench.json is profile "bench" -> preflight applies.
PREFLIGHT_STATIC_CONFIG_REL = os.path.join("finals", "configs", "bench.json")

#: The curated SDF world <-> arena keep-out pairing (the WS-4 manual sync turned
#: into an assertion). Each crate-bearing world's footprint MUST be mirrored by its
#: arena's keep_out polygons. landing_view.sdf is landing_px4 + an overview cam
#: (same crate) so it pairs with the same arena. Worlds with NO obstacle (the
#: convoy circle/lane worlds, empty_cam) are deliberately absent — they have no
#: keep-out to sync. A new crate-bearing world added without an entry here is
#: caught by `_check_sdf_worlds_paired` (every box-bearing world must be paired).
SDF_ARENA_PAIRS: Dict[str, str] = {
    os.path.join(WORLDS_DIR, "followbox1_px4.sdf"):
        os.path.join(ARENAS_DIR, "sitl_followbox1.json"),
    os.path.join(WORLDS_DIR, "followbox_multi_px4.sdf"):
        os.path.join(ARENAS_DIR, "sitl_followbox_multi.json"),
    os.path.join(WORLDS_DIR, "landing_px4.sdf"):
        os.path.join(ARENAS_DIR, "sitl_landing.json"),
    os.path.join(WORLDS_DIR, "landing_view.sdf"):
        os.path.join(ARENAS_DIR, "sitl_landing.json"),
}

#: Half-extent tolerance (m) when matching an SDF box footprint to an arena
#: keep-out bbox. The arena polygons are hand-authored mirrors, so they should be
#: EXACT; a small epsilon absorbs float formatting only (0.005 m = 5 mm, far below
#: any real authoring slip the checker must catch).
_FOOTPRINT_TOL_M = 0.005


@dataclass
class CheckResult:
    """One check's verdict. `ok` False fails the whole run (every check here is a
    hard gate). `problems` is the WHAT/WHICH/WHY/CHECK lines printed on failure;
    `detail` is the one-line PASS summary."""
    id: str
    name: str
    ok: bool
    detail: str
    problems: List[str] = field(default_factory=list)


# ============================================================
# Small fail-loud helpers
# ============================================================
class VerifyError(Exception):
    """A wiring fault in the tool itself (a missing fixture, an unreadable file) —
    distinct from a DRIFT finding (which is a CheckResult with ok=False). Raised
    only when the tool cannot even perform a check; main() catches it -> exit 2."""


def _read_text(path: str) -> str:
    try:
        with open(path, "r", encoding="utf-8") as f:
            return f.read()
    except OSError as e:
        raise VerifyError(
            f"cannot read {path}: {e} — CHECK the file exists and is readable "
            f"(the verify tool runs from the repo root)") from e


def _bbox_of_polygon(poly) -> Tuple[float, float, float, float]:
    """(north_min, east_min, north_max, east_max) of a ring of (north, east)
    points. The arena keep-out polygons are axis-aligned rectangles (crate
    footprints), so the bbox IS the footprint."""
    norths = [p[0] for p in poly]
    easts = [p[1] for p in poly]
    return (min(norths), min(easts), max(norths), max(easts))


# ============================================================
# Check: full suite green (+ cv2-gated not all-skipped)
# ============================================================
def check_suite(*, repo: str = REPO, run: bool = True) -> CheckResult:
    """Run `python -m pytest finals/tests -q -p no:randomly` and assert GREEN.
    `-p no:randomly` makes the run deterministic (the suite carries two documented
    order-sensitive timing races — budget-expiry + stale-telemetry — that only
    trip under random ordering; a green ordered run is the contract). When cv2 is
    importable, also assert the cv2-gated tests did NOT all skip (a whole-suite
    skip would hide vision regressions = drift)."""
    name = "full suite green (pytest -p no:randomly)"
    if not run:
        return CheckResult("suite", name, True,
                           "skipped (--no-suite) — STATIC drift checks still ran")
    cmd = [sys.executable, "-m", "pytest", "finals/tests", "-q",
           "-p", "no:randomly"]
    try:
        proc = subprocess.run(cmd, cwd=repo, capture_output=True, text=True,
                              timeout=900)
    except (OSError, subprocess.TimeoutExpired) as e:
        return CheckResult(
            "suite", name, False, "",
            [f"WHAT: the test suite did not complete | WHICH: `{' '.join(cmd)}` "
             f"| WHY: {type(e).__name__}: {e} | CHECK: run it by hand from "
             f"{repo}"])
    out = (proc.stdout or "") + (proc.stderr or "")
    tail = "\n".join(out.strip().splitlines()[-6:])
    if proc.returncode != 0:
        return CheckResult(
            "suite", name, False, "",
            [f"WHAT: pytest FAILED (exit {proc.returncode}) | WHICH: "
             f"finals/tests | WHY: a test is red (or collection errored) | "
             f"CHECK: re-run `{' '.join(cmd)}`; tail:\n{tail}"])
    # cv2-gated guard: if cv2 imports here, the vision tests must have RUN, not
    # all-skipped. A run where everything skipped (e.g. a broken cv2) reports
    # "passed" but proves nothing — that silent gap is itself drift.
    cv2_present = _cv2_importable()
    summary = _pytest_summary_line(out)
    if cv2_present and summary is not None:
        passed, skipped = summary
        if passed == 0 and skipped > 0:
            return CheckResult(
                "suite", name, False, "",
                [f"WHAT: cv2 is importable but the suite reports 0 passed / "
                 f"{skipped} skipped | WHICH: finals/tests | WHY: a whole-suite "
                 f"skip hides every regression (a broken cv2/conftest gate) | "
                 f"CHECK: `python -c \"import cv2\"` then re-run the suite — "
                 f"summary was: {tail}"])
    detail = f"green: {summary[0] if summary else '?'} passed"
    if summary and summary[1]:
        detail += f", {summary[1]} skipped"
    detail += f"; cv2 {'present (vision ran)' if cv2_present else 'absent'}"
    return CheckResult("suite", name, True, detail)


def _cv2_importable() -> bool:
    """True iff cv2 imports in a SUBPROCESS (never imported into this pure
    module). A failed import — the bare dev venv — is a clean False, not an
    error."""
    try:
        r = subprocess.run([sys.executable, "-c", "import cv2"],
                           capture_output=True, timeout=60)
        return r.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def _pytest_summary_line(out: str) -> Optional[Tuple[int, int]]:
    """(passed, skipped) parsed from pytest's `N passed, M skipped in ...s`
    summary line, or None if it can't be found. Tolerant: either count may be
    absent (just `N passed`)."""
    passed = skipped = None
    for line in reversed(out.strip().splitlines()):
        if " passed" in line or " skipped" in line or " failed" in line:
            mp = re.search(r"(\d+) passed", line)
            ms = re.search(r"(\d+) skipped", line)
            if mp or ms:
                passed = int(mp.group(1)) if mp else 0
                skipped = int(ms.group(1)) if ms else 0
                return passed, skipped
    return None


# ============================================================
# Check: every shipped config loads/validates clean + preflight static gate
# ============================================================
def check_configs(*, repo: str = REPO) -> CheckResult:
    """Every finals/configs/*.json loads + validates via load_config (the same
    code path --dry-run runs), AND the real bench config passes preflight P0
    (config sanity). A config that no longer loads, or a bench config whose
    plane_ids stopped being distinct, is caught HERE on the ground."""
    name = "all configs load + bench preflight P0"
    # load_config / run_preflight are PURE finals modules (no SDK at import) —
    # imported lazily inside the check so this tool's top level stays SDK-free
    # AND a config-module import error surfaces as a tool fault, not a silent skip.
    from finals.config import load_config
    from finals.errors import ConfigError

    problems: List[str] = []
    cfgs = sorted(glob.glob(os.path.join(repo, CONFIGS_GLOB)))
    if not cfgs:
        return CheckResult(
            "configs", name, False, "",
            [f"WHAT: no configs found | WHICH: {CONFIGS_GLOB} under {repo} | "
             f"WHY: wrong CWD or a moved configs dir | CHECK: run from the repo "
             f"root; ls finals/configs/*.json"])
    loaded = []
    bench_cfg = None
    bench_path = os.path.join(repo, PREFLIGHT_STATIC_CONFIG_REL)
    for path in cfgs:
        try:
            cfg = load_config(path)
            loaded.append(os.path.basename(path))
            if os.path.abspath(path) == os.path.abspath(bench_path):
                bench_cfg = cfg
        except ConfigError as e:
            problems.append(
                f"WHAT: config fails to load/validate | WHICH: "
                f"{os.path.relpath(path, repo)} | WHY: {e} | CHECK: fix the "
                f"JSON (this is the --dry-run validation path)")

    # Preflight static gate P0 on the real bench config (config sanity: distinct
    # plane_ids, distinct bands OR sectors, frame_backend pyhulax). run_preflight's
    # _p0_config is the exact gate; we call it directly (async, no hardware).
    if bench_cfg is not None:
        p0 = _run_preflight_p0(bench_cfg)
        if not p0[0]:
            problems.append(
                f"WHAT: bench config fails preflight P0 (config sanity) | WHICH: "
                f"{PREFLIGHT_STATIC_CONFIG_REL} | WHY: {p0[1]} | CHECK: the "
                f"plane_ids must be distinct, the bands/sectors a valid separation "
                f"mechanism, frame_backend 'pyhulax'")
    elif os.path.isfile(bench_path):
        problems.append(
            f"WHAT: bench config did not load so P0 could not run | WHICH: "
            f"{PREFLIGHT_STATIC_CONFIG_REL} | WHY: see its load error above | "
            f"CHECK: fix the bench config first")

    if problems:
        return CheckResult("configs", name, False, "", problems)
    return CheckResult("configs", name, True,
                       f"{len(loaded)} configs load clean; bench P0 PASS")


def _run_preflight_p0(cfg) -> Tuple[bool, str]:
    """Drive finals.preflight._p0_config (the static config-sanity gate) on cfg.
    It is an async coroutine returning (ok, detail, data); run it on a throwaway
    loop. No hardware, no SDK — P0 only reads cfg fields."""
    import asyncio
    from finals.preflight import _p0_config
    ok, detail, _ = asyncio.run(_p0_config(cfg))
    return ok, detail


# ============================================================
# Check: drift detector (the high-value part)
# ============================================================
def check_drift(*, repo: str = REPO,
                runbook_rel: str = RUNBOOK_REL,
                sdf_arena_pairs: Optional[Dict[str, str]] = None) -> CheckResult:
    """The runbook<->code drift detector. Composes the sub-checks (a..e) into one
    result so the report has a single DRIFT line; every sub-problem is a full
    WHAT/WHICH/WHY/CHECK entry."""
    name = "runbook <-> code drift"
    pairs = SDF_ARENA_PAIRS if sdf_arena_pairs is None else sdf_arena_pairs
    runbook_path = os.path.join(repo, runbook_rel)
    runbook = _read_text(runbook_path)

    problems: List[str] = []
    problems += _drift_linked_paths_exist(runbook, repo, runbook_rel)
    problems += _drift_orphan_configs(runbook, repo)
    problems += _drift_phases_registered(runbook, repo, runbook_rel)
    problems += _drift_preflight_gates(runbook, repo, runbook_rel)
    problems += _drift_sdf_keepout(repo, pairs)
    problems += _drift_sdf_worlds_paired(repo, pairs)

    if problems:
        return CheckResult("drift", name, False, "", problems)
    return CheckResult(
        "drift", name, True,
        "linked paths exist; no orphan configs; phases registered; P0..P10 "
        "match; SDF<->keep-out in sync")


_PATH_RE = re.compile(
    r"(finals/configs/arenas/[A-Za-z0-9_]+\.json"
    r"|finals/configs/[A-Za-z0-9_]+\.json"
    r"|sim/worlds/[A-Za-z0-9_]+\.sdf"
    r"|sim/run_(?:landing|vision)\.sh)")


def _runbook_linked_paths(runbook: str) -> List[str]:
    """Every repo path the runbook NAMES (configs, arenas, worlds, sim scripts),
    de-duplicated in first-seen order. Markdown links and bare code spans both
    match — the regex keys on the path shape, not the link syntax."""
    seen: List[str] = []
    for m in _PATH_RE.finditer(runbook):
        p = m.group(1)
        if p not in seen:
            seen.append(p)
    return seen


def _drift_linked_paths_exist(runbook: str, repo: str,
                              runbook_rel: str) -> List[str]:
    """(a) every config/arena/world/script the runbook LINKS exists on disk."""
    out: List[str] = []
    for rel in _runbook_linked_paths(runbook):
        if not os.path.isfile(os.path.join(repo, rel)):
            out.append(
                f"WHAT: runbook links a file that does not exist | WHICH: {rel} | "
                f"WHY: the path was renamed/removed or mistyped in "
                f"{runbook_rel} | CHECK: fix the link or restore the file")
    return out


def _runbook_referenced_configs(runbook: str) -> set:
    """The set of finals/configs/*.json basenames the runbook mentions (by full
    path OR bare basename). A config named ANYWHERE in the prose counts as
    referenced — the runbook may cite a config in a table cell without a link."""
    refs = set()
    for rel in _runbook_linked_paths(runbook):
        if rel.startswith("finals/configs/") and rel.endswith(".json") \
                and "/arenas/" not in rel:
            refs.add(os.path.basename(rel))
    # Also catch bare basenames in prose / sim-subcommand mentions, e.g. a row
    # that says "sitl1_followbox1.json" without the dir prefix.
    for m in re.finditer(r"\b([A-Za-z0-9_]+\.json)\b", runbook):
        refs.add(m.group(1))
    return refs


def _drift_orphan_configs(runbook: str, repo: str) -> List[str]:
    """(b) every shipped finals/configs/*.json is REFERENCED by the runbook. An
    orphan config the runbook forgot is drift (this is what flags a new
    followbox/dyn config until the runbook is synced)."""
    referenced = _runbook_referenced_configs(runbook)
    out: List[str] = []
    for path in sorted(glob.glob(os.path.join(repo, CONFIGS_GLOB))):
        base = os.path.basename(path)
        if base not in referenced:
            out.append(
                f"WHAT: a shipped config is NOT mentioned by the runbook | "
                f"WHICH: finals/configs/{base} | WHY: the runbook forgot a "
                f"profile the operator can run (orphan config) | CHECK: add a "
                f"row/mention for {base} in {RUNBOOK_REL}, or delete the config")
    return out


def _registry_phase_names(repo: str) -> set:
    """The PHASE_REGISTRY keys (lazy import — pure finals module)."""
    from finals.mission.phases import PHASE_REGISTRY
    return set(PHASE_REGISTRY)


def _runbook_named_phases(runbook: str, repo: str) -> set:
    """Phase names the runbook mentions that are ALSO registry names. We scan for
    each registry name as a whole word (the runbook prose names phases like
    `land_on_pad`, `track_convoy`, `navigate`) — so a typo'd phase name in the
    runbook (`land_on_pads`) simply won't match any registry name, which the
    EXISTS direction below still catches via the explicit phase-token scan."""
    names = _registry_phase_names(repo)
    found = set()
    for nm in names:
        if re.search(rf"`{re.escape(nm)}`|\b{re.escape(nm)}\b", runbook):
            found.add(nm)
    return found


# Phase-like tokens the runbook uses in a phase context: a backticked snake_case
# word that looks like a phase (these are what a "runbook-named phase" means for
# the drift check). We then assert each is registered.
_PHASE_TOKEN_RE = re.compile(r"`([a-z][a-z_]*[a-z])`")
# Backticked snake_case tokens that are NOT phases (commands, fields, flags,
# files) — excluded so the phase-registration check only judges real phase names.
_NON_PHASE_TOKENS = {
    "preflight_only", "no_setpoint_set", "set_target_ip", "set_led",
    "robust_connect", "enable_battery_failsafe", "pixel_offset_to_move",
    "k_lateral", "tol_px", "descend_step_cm", "descend_persist_frames",
    "commit_alt_m", "acquire_timeout_s", "valid_marker_ids", "max_leg_cm",
    "inflation_m", "plane_id", "led_rgb", "altitude_band_m", "camera_hfov_deg",
    "min_battery_pct", "go_timeout_s", "tick_hz", "frame_backend",
    "flight_backend", "marker_backend", "arena_name", "sightings",
    "save_marker_frames", "observed_keep_out", "convoy_ids", "convoy_lock_ttl_s",
    "track_marker_ids", "sector_deg", "goal_ne_m", "pad_id", "keep_out",
    "i_know_this_arms_real_drones",
}


def _drift_phases_registered(runbook: str, repo: str,
                             runbook_rel: str) -> List[str]:
    """(c) every PHASE the runbook names (a snake_case backticked token that is
    not a known non-phase) is in PHASE_REGISTRY. Catches a renamed phase the
    runbook still references."""
    registry = _registry_phase_names(repo)
    out: List[str] = []
    for m in _PHASE_TOKEN_RE.finditer(runbook):
        tok = m.group(1)
        if tok in _NON_PHASE_TOKENS or tok in registry:
            continue
        # Only flag tokens that LOOK like a phase reference: appear next to the
        # word "phase", or are a bare phase-name list. To stay precise we flag a
        # token only if the runbook explicitly couples it with "phase".
        ctx_re = re.compile(rf"`{re.escape(tok)}`[^\n]*phase|phase[^\n]*`"
                            rf"{re.escape(tok)}`")
        if ctx_re.search(runbook):
            out.append(
                f"WHAT: runbook names a phase that is NOT registered | WHICH: "
                f"phase {tok!r} in {runbook_rel} | WHY: the phase was renamed or "
                f"never existed (registry has {sorted(registry)}) | CHECK: fix "
                f"the runbook name or register the phase")
    return out


_PREFLIGHT_GATE_RE = re.compile(r"\bP(\d+)\b")


def _runbook_preflight_gates(runbook: str) -> set:
    """The set of P-gate ids the runbook tabulates (P0, P1, ...). Scans the whole
    runbook; the P0..P10 table is the canonical source."""
    return {f"P{m.group(1)}" for m in _PREFLIGHT_GATE_RE.finditer(runbook)}


def _code_preflight_gates(repo: str) -> set:
    """The P-gate ids finals.preflight.run_preflight actually runs, read from the
    source (each gate is an `await _gate("P{n}", ...)` line). Reading the source —
    not running run_preflight (which needs agents/hardware) — keeps this a pure
    static check."""
    src = _read_text(os.path.join(repo, "finals", "preflight.py"))
    return set(re.findall(r'_gate\(\s*"(P\d+)"', src))


def _drift_preflight_gates(runbook: str, repo: str,
                           runbook_rel: str) -> List[str]:
    """(d) the P0..P10 rows the runbook tabulates == the gates run_preflight runs.
    Drift either way: a gate the code runs the runbook forgot to document, or a
    gate the runbook promises the code dropped."""
    doc = _runbook_preflight_gates(runbook)
    code = _code_preflight_gates(repo)
    out: List[str] = []
    missing_in_doc = sorted(code - doc, key=_gate_key)
    missing_in_code = sorted(doc - code, key=_gate_key)
    if missing_in_doc:
        out.append(
            f"WHAT: preflight gate(s) the code RUNS are undocumented | WHICH: "
            f"{missing_in_doc} | WHY: run_preflight runs them but the runbook "
            f"P0..P10 table omits them | CHECK: add the row(s) to {runbook_rel}")
    if missing_in_code:
        out.append(
            f"WHAT: preflight gate(s) the runbook PROMISES the code dropped | "
            f"WHICH: {missing_in_code} | WHY: the runbook tabulates them but "
            f"run_preflight no longer runs them | CHECK: restore the gate or "
            f"remove the stale row from {runbook_rel}")
    return out


def _gate_key(g: str) -> int:
    return int(g[1:])


# ---- (e) the SDF <-> arena keep-out checker -------------------------------
_XML_COMMENT_RE = re.compile(r"<!--.*?-->", re.DOTALL)


def _strip_xml_comments(text: str) -> str:
    """Remove every <!-- ... --> block. The repo's SDF comments contain `--`
    (e.g. `--delay-s`) which strict XML forbids inside a comment; the checker
    reads only element data, so stripping comments is lossless + robust."""
    return _XML_COMMENT_RE.sub("", text)


def parse_sdf_boxes(sdf_text: str) -> List[Tuple[str, Tuple[float, float, float],
                                                 Tuple[float, float, float]]]:
    """Parse every STATIC box obstacle model in an SDF world into
    (model_name, (gx, gy, gz) pose, (sx, sy, sz) box size).

    Only models whose link/collision geometry is a <box> are returned (the
    ground_plane is a <plane>, the drone/convoy are <include>s — none match). gz
    is ENU: pose = "x(EAST) y(NORTH) z(UP) roll pitch yaw"; box size = "sx sy sz".
    Robust to namespaced/plain SDF (the gz worlds are plain sdf:1.9).

    The repo's SDF worlds carry authoring comments containing `--` (e.g.
    `--delay-s`), which libsdformat/gz tolerate but the strict XML spec (and
    xml.etree) forbid INSIDE a comment. We strip every <!-- ... --> comment block
    BEFORE parsing — the checker never reads comment content (only <model>/<pose>/
    <box>), so dropping comments is lossless and makes the parse robust to that
    real-world quirk without editing the (working) worlds."""
    root = ET.fromstring(_strip_xml_comments(sdf_text))
    world = root.find("world")
    if world is None:
        return []
    boxes = []
    for model in world.findall("model"):
        name = model.get("name", "<unnamed>")
        pose_el = model.find("pose")
        if pose_el is None or pose_el.text is None:
            continue
        try:
            pose_vals = [float(x) for x in pose_el.text.split()]
        except ValueError:
            continue
        if len(pose_vals) < 3:
            continue
        # Find a <box><size> anywhere under this model (link/collision/geometry).
        size_el = None
        for box in model.iter("box"):
            size_el = box.find("size")
            if size_el is not None and size_el.text is not None:
                break
        if size_el is None or size_el.text is None:
            continue
        try:
            size_vals = [float(x) for x in size_el.text.split()]
        except ValueError:
            continue
        if len(size_vals) < 3:
            continue
        boxes.append((name,
                      (pose_vals[0], pose_vals[1], pose_vals[2]),
                      (size_vals[0], size_vals[1], size_vals[2])))
    return boxes


def sdf_box_to_arena_bbox(pose, size) -> Tuple[float, float, float, float]:
    """An SDF box (gz pose (x=E,y=N,z), size (sx,sy,sz)) -> the arena keep-out
    bounding box (north_min, east_min, north_max, east_max).

    FRAME (binding, mirrored in every followbox/landing arena _comment):
      gz x = EAST, gz y = NORTH. So the box CENTRE in arena coords is
      (north = gy, east = gx). The footprint half-extents are sy/2 along NORTH
      and sx/2 along EAST (size is given as sx sy sz in gz axes). Height (sz) does
      NOT change the 2-D keep-out, so z/sz are ignored."""
    gx, gy, _gz = pose
    sx, sy, _sz = size
    north_c, east_c = gy, gx
    hn, he = sy / 2.0, sx / 2.0
    return (north_c - hn, east_c - he, north_c + hn, east_c + he)


def _arena_keepout_bboxes(arena_path: str) -> List[Tuple[str, Tuple[float, float,
                                                                    float, float]]]:
    """[(keep_out_id, bbox)] for an arena JSON. bbox = (north_min, east_min,
    north_max, east_max) of each keep_out polygon. Comment keys (_x) are
    ignored by the arena loader; here we read the raw JSON directly."""
    raw = json.loads(_read_text(arena_path))
    out = []
    for ko in raw.get("keep_out", []):
        kid = ko.get("id", "<no-id>")
        poly = ko.get("polygon_m", [])
        if not poly:
            continue
        pts = [(float(p[0]), float(p[1])) for p in poly]
        out.append((kid, _bbox_of_polygon(pts)))
    return out


def match_box_to_keepout(box_bbox, keepout_bboxes, tol: float = _FOOTPRINT_TOL_M
                         ) -> Optional[str]:
    """The keep-out id whose bbox matches box_bbox within tol on all four edges,
    or None. The arena polygons are exact mirrors, so a match is edge-equality."""
    bn0, be0, bn1, be1 = box_bbox
    for kid, (kn0, ke0, kn1, ke1) in keepout_bboxes:
        if (abs(bn0 - kn0) <= tol and abs(be0 - ke0) <= tol
                and abs(bn1 - kn1) <= tol and abs(be1 - ke1) <= tol):
            return kid
    return None


def _drift_sdf_keepout(repo: str, pairs: Dict[str, str]) -> List[str]:
    """(e) every crate <pose>+<box><size> in each paired SDF world has a MATCHING
    arena keep-out polygon, and (the reverse) every arena keep-out is backed by an
    SDF box. Turns the WS-4 manual SDF/arena sync into a CI assertion."""
    out: List[str] = []
    for sdf_rel, arena_rel in pairs.items():
        sdf_path = os.path.join(repo, sdf_rel)
        arena_path = os.path.join(repo, arena_rel)
        if not os.path.isfile(sdf_path):
            out.append(
                f"WHAT: paired SDF world missing | WHICH: {sdf_rel} | WHY: the "
                f"SDF<->arena pair points at a file that is not there | CHECK: "
                f"restore the world or fix SDF_ARENA_PAIRS")
            continue
        if not os.path.isfile(arena_path):
            out.append(
                f"WHAT: paired arena missing | WHICH: {arena_rel} (for "
                f"{sdf_rel}) | WHY: the SDF has obstacles but its arena mirror is "
                f"gone | CHECK: restore the arena JSON or fix SDF_ARENA_PAIRS")
            continue
        boxes = parse_sdf_boxes(_read_text(sdf_path))
        keepouts = _arena_keepout_bboxes(arena_path)
        matched_kids = set()
        for name, pose, size in boxes:
            box_bbox = sdf_box_to_arena_bbox(pose, size)
            kid = match_box_to_keepout(box_bbox, keepouts)
            if kid is None:
                out.append(
                    f"WHAT: an SDF crate has NO matching arena keep-out | WHICH: "
                    f"model {name!r} in {sdf_rel} (footprint north "
                    f"[{box_bbox[0]:.3f},{box_bbox[2]:.3f}] east "
                    f"[{box_bbox[1]:.3f},{box_bbox[3]:.3f}]) | WHY: the planner "
                    f"detours around arena keep-outs, not SDF crates — an "
                    f"unmirrored crate is a crate the drone flies INTO | CHECK: "
                    f"add a keep_out polygon mirroring this footprint to "
                    f"{arena_rel} (gz x=E,y=N -> arena north=y,east=x)")
            else:
                matched_kids.add(kid)
        # Reverse: an arena keep-out with no backing SDF box is a phantom
        # obstacle the planner detours around that does not exist in the world.
        for kid, bbox in keepouts:
            if kid not in matched_kids:
                out.append(
                    f"WHAT: an arena keep-out has NO backing SDF crate | WHICH: "
                    f"keep_out {kid!r} in {arena_rel} (north "
                    f"[{bbox[0]:.3f},{bbox[2]:.3f}] east "
                    f"[{bbox[1]:.3f},{bbox[3]:.3f}]) | WHY: the planner will "
                    f"detour around an obstacle that is not in {sdf_rel} (a "
                    f"phantom keep-out) | CHECK: remove the keep_out or add the "
                    f"crate to the SDF")
    return out


def _world_has_box(sdf_path: str) -> bool:
    try:
        return bool(parse_sdf_boxes(_read_text(sdf_path)))
    except (ET.ParseError, VerifyError):
        # A malformed world is its own problem (the sims would not launch); the
        # pairing check below would not learn anything useful from it.
        return False


def _drift_sdf_worlds_paired(repo: str, pairs: Dict[str, str]) -> List[str]:
    """Every crate-bearing SDF world MUST appear in SDF_ARENA_PAIRS — so a new
    obstacle world added without a sync entry is caught (rather than silently
    skipping the keep-out check for it)."""
    paired = {os.path.normpath(p) for p in pairs}
    out: List[str] = []
    for sdf_path in sorted(glob.glob(os.path.join(repo, WORLDS_DIR, "*.sdf"))):
        rel = os.path.normpath(os.path.relpath(sdf_path, repo))
        if rel in paired:
            continue
        if _world_has_box(sdf_path):
            out.append(
                f"WHAT: a crate-bearing SDF world is not SDF<->arena paired | "
                f"WHICH: {rel} | WHY: it has <box> obstacles but no entry in "
                f"verify_runbook.SDF_ARENA_PAIRS, so its footprints are never "
                f"checked against an arena keep-out | CHECK: add it to "
                f"SDF_ARENA_PAIRS with its mirror arena")
    return out


# ============================================================
# Check: SITL smoke (VM-only, --smoke; positive-evidence asserts)
# ============================================================
#: Per-profile smoke contracts (documented; asserted only under --smoke against a
#: --run-dir of real evidence). Each names the run that produces the evidence and
#: the POSITIVE assertion that proves the profile worked end-to-end.
SMOKE_CONTRACTS = {
    "land1": "sim/run_landing.sh land1 -> mission.jsonl logs a `navigate` "
             "action_complete (the transit ran, not just takeoff)",
    "followbox1": "sim/run_landing.sh followbox1 -> sightings.csv has rows > 0 "
                  "(the convoy car was seen) AND navigate logged >= 2 legs",
    "track3": "sim/run_vision.sh track3 -> sightings.csv has rows > 0 across the "
              "3 drones (the chase saw the cars)",
    "dyn3": "sim/run_vision.sh dyn3 -> sightings.csv has rows > 0 + the heartbeat "
            "shows 3 distinct convoy owners",
}


def check_smoke(*, run_dir: Optional[str], repo: str = REPO,
                enabled: bool = False) -> CheckResult:
    """VM-only positive-evidence smoke. Default DISABLED (--smoke off): returns a
    PASS that documents the contracts without requiring hardware. When enabled
    with a --run-dir, asserts the run produced the positive evidence
    (sightings.csv rows > 0 and/or a navigate action_complete in mission.jsonl)."""
    name = "SITL smoke (positive evidence)"
    if not enabled:
        contracts = "; ".join(f"{k}: {v.split(' -> ')[0]}"
                              for k, v in SMOKE_CONTRACTS.items())
        return CheckResult(
            "smoke", name, True,
            f"disabled (no --smoke) — VM-only; contracts documented [{contracts}]")
    if not run_dir:
        return CheckResult(
            "smoke", name, False, "",
            ["WHAT: --smoke given without --run-dir | WHICH: smoke gate | WHY: "
             "positive-evidence smoke needs a run's evidence dir | CHECK: pass "
             "--run-dir <runs_finals/...> from a VM SITL run"])
    problems: List[str] = []
    sightings = os.path.join(run_dir, "sightings.csv")
    mission = os.path.join(run_dir, "mission.jsonl")
    saw_evidence = False
    if os.path.isfile(sightings):
        rows = _count_csv_rows(sightings)
        saw_evidence = True
        if rows <= 0:
            problems.append(
                f"WHAT: sightings.csv has NO rows | WHICH: {sightings} | WHY: the "
                f"flight saw no markers (a dead camera/bridge or a missed convoy) "
                f"| CHECK: the gz bridge + convoy drive; sightings must be > 0")
    if os.path.isfile(mission):
        saw_evidence = True
        if not _mission_has_navigate_complete(mission):
            problems.append(
                f"WHAT: no completed `navigate` in mission.jsonl | WHICH: "
                f"{mission} | WHY: the open-loop transit never finished (only "
                f"takeoff?) | CHECK: the planner legs + nav budget")
    if not saw_evidence:
        problems.append(
            f"WHAT: no evidence files in --run-dir | WHICH: {run_dir} | WHY: "
            f"neither sightings.csv nor mission.jsonl is present | CHECK: point "
            f"--run-dir at a completed run's evidence dir")
    if problems:
        return CheckResult("smoke", name, False, "", problems)
    return CheckResult("smoke", name, True,
                       f"positive evidence present in {run_dir}")


def _count_csv_rows(path: str) -> int:
    """Data rows in a CSV (total lines minus a header), tolerant of a trailing
    newline. Returns 0 for a header-only or empty file."""
    text = _read_text(path)
    lines = [ln for ln in text.splitlines() if ln.strip()]
    return max(0, len(lines) - 1)


def _mission_has_navigate_complete(path: str) -> bool:
    """True iff mission.jsonl has an action/phase complete event naming navigate.
    Tolerant line-by-line JSON parse (a torn tail line is skipped, not fatal)."""
    text = _read_text(path)
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError:
            continue
        kind = str(row.get("kind", row.get("event", "")))
        phase = str(row.get("phase", ""))
        action = str(row.get("action", ""))
        if "navigate" in (phase, action) and (
                "complete" in kind or "exit" in kind or kind == "phase_exit"):
            return True
    return False


# ============================================================
# Report + main
# ============================================================
def _print_report(results: List[CheckResult]) -> None:
    print("=" * 72)
    print("RUNBOOK VERIFY  (docs/finals/onsite_test_plan.md <-> code)")
    print("=" * 72)
    for r in results:
        tag = "PASS" if r.ok else "FAIL"
        print(f"  [{r.id:<7}] {tag}  {r.name}")
        if r.ok:
            print(f"            {r.detail}")
        else:
            for p in r.problems:
                print(f"            - {p}")
    print("=" * 72)


def run_all_checks(*, repo: str = REPO, run_suite: bool = True,
                   smoke: bool = False,
                   run_dir: Optional[str] = None) -> List[CheckResult]:
    """Run every check in order and return the results. The order is cheap->dear:
    drift + configs (fast, local) before the suite (slow); smoke last."""
    results: List[CheckResult] = []
    results.append(check_drift(repo=repo))
    results.append(check_configs(repo=repo))
    results.append(check_suite(repo=repo, run=run_suite))
    results.append(check_smoke(run_dir=run_dir, repo=repo, enabled=smoke))
    return results


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m finals.tools.verify_runbook",
        description="Verify docs/finals/onsite_test_plan.md is in sync with the "
                    "code; fail loud on any drift.")
    parser.add_argument("--no-suite", action="store_true",
                        help="skip the pytest run (drift + config checks only)")
    parser.add_argument("--smoke", action="store_true",
                        help="enable the VM-only positive-evidence SITL smoke "
                             "(needs --run-dir)")
    parser.add_argument("--run-dir",
                        help="a completed SITL run's evidence dir for --smoke")
    args = parser.parse_args(argv)

    try:
        results = run_all_checks(repo=REPO, run_suite=not args.no_suite,
                                 smoke=args.smoke, run_dir=args.run_dir)
    except VerifyError as e:
        print(f"verify_runbook: TOOL FAULT — {e}", file=sys.stderr)
        return 2

    _print_report(results)
    failed = [r for r in results if not r.ok]
    if failed:
        print(f"VERDICT: DRIFT — {len(failed)} check(s) FAILED: "
              f"{[r.id for r in failed]}", file=sys.stderr)
        return 1
    print("VERDICT: PASS — runbook and code are in sync.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
