"""finals.tools.verify_runbook — the runbook<->code drift detector (WS-7B).

Two halves, both PURE (stdlib + pytest; NO cv2/gz/numpy — the parser is xml.etree,
the suite run is a subprocess the tool owns and these tests never invoke):

1. The drift detector PASSES on the REAL tree (the runbook is in sync after the
   WS-7B sync). This is the regression guard: if a future change adds an orphan
   config, renames a phase, drops a preflight gate, or desyncs an SDF crate from
   its arena keep-out, exactly these checks go red.

2. The detector FLAGS deliberately-broken fixtures: a runbook referencing a
   nonexistent config, an orphan config the runbook forgot, an SDF crate with no
   matching arena keep-out (and the reverse — a phantom keep-out). The SDF<->keep-
   out parser is unit-tested on the known-good followbox pair AND on a hand-built
   mismatched pair (must flag) — that is the mutation-resistant core.
"""
from __future__ import annotations

import json
import os

import pytest

from finals.tools import verify_runbook as vr

REPO = vr.REPO


# ============================================================
# SDF box parser + frame mapping (the trickiest unit)
# ============================================================
def test_parse_sdf_boxes_followbox1_known_good():
    """The real followbox1 world has exactly ONE crate at gz(0,2), 1.0x1.0x3.0."""
    sdf = _read(os.path.join(REPO, "sim", "worlds", "followbox1_px4.sdf"))
    boxes = vr.parse_sdf_boxes(sdf)
    assert len(boxes) == 1, boxes
    name, pose, size = boxes[0]
    assert name == "crate"
    assert pose == (0.0, 2.0, 1.5)        # gz x=E=0, y=N=2, z=1.5
    assert size == (1.0, 1.0, 3.0)        # 1.0x1.0 footprint, 3.0 tall


def test_parse_sdf_strips_double_dash_comment():
    """The repo SDFs carry `--delay-s` inside XML comments, which strict XML
    forbids — the parser must strip comments and still read the crate (a raw
    xml.etree.fromstring would raise here)."""
    sdf = _read(os.path.join(REPO, "sim", "worlds", "followbox1_px4.sdf"))
    assert "--delay-s" in sdf                      # the offending token is present
    boxes = vr.parse_sdf_boxes(sdf)                # would raise if not stripped
    assert boxes


def test_parse_sdf_multi_finds_three_crates():
    sdf = _read(os.path.join(REPO, "sim", "worlds", "followbox_multi_px4.sdf"))
    boxes = vr.parse_sdf_boxes(sdf)
    assert {b[0] for b in boxes} == {"crate_a", "crate_b", "crate_c"}


def test_parse_sdf_ignores_plane_and_include():
    """ground_plane is a <plane> and the convoy car is an <include> — neither is a
    box, so parse_sdf_boxes returns ONLY the crate(s)."""
    sdf = _read(os.path.join(REPO, "sim", "worlds", "followbox1_px4.sdf"))
    assert "ground_plane" in sdf and "convoy_robot_7" in sdf
    assert {b[0] for b in vr.parse_sdf_boxes(sdf)} == {"crate"}


def test_sdf_box_to_arena_bbox_frame_mapping():
    """gz pose (x=E, y=N), size (sx,sy,sz) -> arena bbox (n_min,e_min,n_max,e_max),
    with north from gz-y, east from gz-x. The crate at gz(0,2) 1.0x1.0 -> north
    [1.5,2.5], east [-0.5,0.5] (matches sitl_followbox1.json by hand)."""
    bbox = vr.sdf_box_to_arena_bbox((0.0, 2.0, 1.5), (1.0, 1.0, 3.0))
    assert bbox == pytest.approx((1.5, -0.5, 2.5, 0.5))


def test_sdf_box_to_arena_bbox_asymmetric_size():
    """A non-square footprint maps sx->east half-extent, sy->north half-extent
    (the two axes are NOT interchangeable — this pins the mapping)."""
    # gz centre E=1.0, N=3.0; sx=2.0 (east extent), sy=4.0 (north extent).
    bbox = vr.sdf_box_to_arena_bbox((1.0, 3.0, 0.5), (2.0, 4.0, 1.0))
    # north = gy=3 +/- sy/2=2 -> [1,5]; east = gx=1 +/- sx/2=1 -> [0,2]
    assert bbox == pytest.approx((1.0, 0.0, 5.0, 2.0))


def test_match_box_to_keepout_exact_and_miss():
    keepouts = [("crate", (1.5, -0.5, 2.5, 0.5))]
    assert vr.match_box_to_keepout((1.5, -0.5, 2.5, 0.5), keepouts) == "crate"
    # shifted half a metre north -> no match
    assert vr.match_box_to_keepout((2.0, -0.5, 3.0, 0.5), keepouts) is None


# ============================================================
# The SDF<->keep-out drift sub-check: real tree PASSES
# ============================================================
def test_sdf_keepout_real_tree_in_sync():
    """Every crate in the 3 paired worlds has a matching arena keep-out AND every
    arena keep-out is backed by a crate — zero findings on the shipped tree."""
    problems = vr._drift_sdf_keepout(REPO, vr.SDF_ARENA_PAIRS)
    assert problems == [], problems


def test_every_crate_bearing_world_is_paired():
    problems = vr._drift_sdf_worlds_paired(REPO, vr.SDF_ARENA_PAIRS)
    assert problems == [], problems


# ============================================================
# The SDF<->keep-out drift sub-check: hand-built MISMATCH must flag
# ============================================================
def _write(path: str, text: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


_SDF_ONE_CRATE = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="mini">
    <model name="ground_plane"><static>true</static><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal>
      <size>100 100</size></plane></geometry></collision></link></model>
    <model name="crate">
      <static>true</static>
      <pose>0 2 0.75 0 0 0</pose>
      <link name="link"><collision name="col">
        <geometry><box><size>1.0 1.0 1.5</size></box></geometry>
      </collision></link>
    </model>
  </world>
</sdf>
"""

# An arena whose ONE keep-out matches the crate above (north 1.5..2.5, east -0.5..0.5).
_ARENA_MATCH = {
    "bounds_m": [-1.0, -3.0, 6.0, 3.0],
    "c2_origin_m": [0.0, 0.0],
    "c2_heading_deg": 0.0,
    "keep_out": [{"id": "crate",
                  "polygon_m": [[1.5, -0.5], [2.5, -0.5], [2.5, 0.5], [1.5, 0.5]]}],
    "pads": [],
}


def _mini_pairs(tmp_path, sdf_text: str, arena_obj: dict):
    """Lay a one-world/one-arena mini repo under tmp_path and return
    (repo, pairs) for the SDF<->keep-out checker."""
    repo = str(tmp_path)
    sdf_rel = os.path.join("sim", "worlds", "mini_px4.sdf")
    arena_rel = os.path.join("finals", "configs", "arenas", "mini.json")
    _write(os.path.join(repo, sdf_rel), sdf_text)
    _write(os.path.join(repo, arena_rel), json.dumps(arena_obj))
    return repo, {sdf_rel: arena_rel}


def test_sdf_keepout_matching_pair_passes(tmp_path):
    repo, pairs = _mini_pairs(tmp_path, _SDF_ONE_CRATE, _ARENA_MATCH)
    assert vr._drift_sdf_keepout(repo, pairs) == []


def test_sdf_keepout_crate_without_keepout_flags(tmp_path):
    """A crate in the SDF with NO matching arena keep-out is the headline drift —
    the planner would fly into it. Must produce a finding naming the crate."""
    arena_no_keepout = dict(_ARENA_MATCH, keep_out=[])
    repo, pairs = _mini_pairs(tmp_path, _SDF_ONE_CRATE, arena_no_keepout)
    problems = vr._drift_sdf_keepout(repo, pairs)
    assert problems, "an unmirrored crate must be flagged"
    assert any("NO matching arena keep-out" in p and "crate" in p
               for p in problems), problems


def test_sdf_keepout_shifted_polygon_flags(tmp_path):
    """A keep-out that is present but MISPLACED (shifted off the crate footprint)
    flags BOTH directions: the crate has no match, and the keep-out is a phantom."""
    shifted = dict(_ARENA_MATCH, keep_out=[
        {"id": "crate", "polygon_m": [[3.5, -0.5], [4.5, -0.5],
                                       [4.5, 0.5], [3.5, 0.5]]}])
    repo, pairs = _mini_pairs(tmp_path, _SDF_ONE_CRATE, shifted)
    problems = vr._drift_sdf_keepout(repo, pairs)
    assert any("NO matching arena keep-out" in p for p in problems), problems
    assert any("NO backing SDF crate" in p for p in problems), problems


def test_sdf_keepout_phantom_keepout_flags(tmp_path):
    """An EXTRA arena keep-out with no backing crate is a phantom obstacle the
    planner detours around for nothing."""
    phantom = dict(_ARENA_MATCH, keep_out=_ARENA_MATCH["keep_out"] + [
        {"id": "ghost", "polygon_m": [[10.0, 10.0], [11.0, 10.0],
                                      [11.0, 11.0], [10.0, 11.0]]}])
    repo, pairs = _mini_pairs(tmp_path, _SDF_ONE_CRATE, phantom)
    problems = vr._drift_sdf_keepout(repo, pairs)
    assert any("NO backing SDF crate" in p and "ghost" in p
               for p in problems), problems


# ============================================================
# Orphan-config + missing-link drift on a fixture runbook
# ============================================================
def _runbook_referencing(*config_basenames: str) -> str:
    """A minimal runbook body that mentions the given config basenames."""
    lines = ["# fixture runbook", ""]
    lines += [f"- run `finals/configs/{b}`" for b in config_basenames]
    return "\n".join(lines) + "\n"


def test_orphan_config_flagged(tmp_path):
    """A configs dir with a config the runbook never names -> orphan finding."""
    repo = str(tmp_path)
    cfgdir = os.path.join(repo, "finals", "configs")
    _write(os.path.join(cfgdir, "alpha.json"), "{}")
    _write(os.path.join(cfgdir, "forgotten.json"), "{}")
    runbook = _runbook_referencing("alpha.json")   # forgotten.json omitted
    problems = vr._drift_orphan_configs(runbook, repo)
    assert any("forgotten.json" in p and "NOT mentioned" in p
               for p in problems), problems
    assert not any("alpha.json" in p for p in problems), problems


def test_linked_path_missing_flagged(tmp_path):
    """A runbook that LINKS a config which does not exist on disk -> finding."""
    repo = str(tmp_path)
    _write(os.path.join(repo, "finals", "configs", "real.json"), "{}")
    runbook = ("see [`finals/configs/real.json`](x) and "
               "[`finals/configs/ghost.json`](y)\n")
    problems = vr._drift_linked_paths_exist(runbook, repo, "rb.md")
    assert any("finals/configs/ghost.json" in p for p in problems), problems
    assert not any("real.json" in p for p in problems), problems


# ============================================================
# Phase-registration + preflight-gate drift
# ============================================================
def test_phase_registration_real_runbook_clean():
    runbook = _read(os.path.join(REPO, vr.RUNBOOK_REL))
    assert vr._drift_phases_registered(runbook, REPO, vr.RUNBOOK_REL) == []


def test_unregistered_phase_flagged():
    runbook = ("the drone runs the `warp_drive` phase after takeoff, then the "
               "`navigate` phase.\n")
    problems = vr._drift_phases_registered(runbook, REPO, "rb.md")
    assert any("phase 'warp_drive'" in p for p in problems), problems
    # navigate is a real registered phase -> it is never the SUBJECT of a finding
    # (it may appear in the registry list printed in another finding's hint).
    assert not any("phase 'navigate'" in p for p in problems), problems


def test_preflight_gates_real_runbook_match():
    runbook = _read(os.path.join(REPO, vr.RUNBOOK_REL))
    assert vr._drift_preflight_gates(runbook, REPO, vr.RUNBOOK_REL) == []


def test_preflight_gate_dropped_from_runbook_flagged():
    """A runbook that documents only P0..P5 (the code runs P0..P10) -> finding
    naming the undocumented gates."""
    runbook = "Gates: P0 P1 P2 P3 P4 P5 only.\n"
    problems = vr._drift_preflight_gates(runbook, REPO, "rb.md")
    assert problems
    joined = " ".join(problems)
    assert "P10" in joined and "undocumented" in joined


def test_code_preflight_gates_are_p0_through_p10():
    """The source-read of run_preflight's gate ids is the authoritative P0..P10."""
    gates = vr._code_preflight_gates(REPO)
    assert gates == {f"P{n}" for n in range(11)}, gates


# ============================================================
# The whole drift check + configs check on the REAL tree
# ============================================================
def test_check_drift_real_tree_passes():
    """After the WS-7B runbook sync, the full drift detector is GREEN."""
    res = vr.check_drift(repo=REPO)
    assert res.ok, res.problems


def test_check_configs_real_tree_passes():
    """Every shipped config loads + the bench config passes preflight P0."""
    res = vr.check_configs(repo=REPO)
    assert res.ok, res.problems


def test_run_all_checks_no_suite_passes():
    """The composed run (suite skipped) PASSes on the real tree — the smoke gate
    is disabled by default and the static checks are green."""
    results = vr.run_all_checks(repo=REPO, run_suite=False, smoke=False)
    failed = [r.id for r in results if not r.ok]
    assert failed == [], [(r.id, r.problems) for r in results if not r.ok]


# ============================================================
# Smoke gate: disabled by default, asserts positive evidence when enabled
# ============================================================
def test_smoke_disabled_by_default_passes():
    res = vr.check_smoke(run_dir=None, repo=REPO, enabled=False)
    assert res.ok and "disabled" in res.detail


def test_smoke_enabled_without_run_dir_fails():
    res = vr.check_smoke(run_dir=None, repo=REPO, enabled=True)
    assert not res.ok
    assert any("--run-dir" in p for p in res.problems)


def test_smoke_enabled_zero_sightings_fails(tmp_path):
    """A run whose sightings.csv has only a header (0 data rows) is NOT positive
    evidence — the smoke must fail loud."""
    run_dir = str(tmp_path)
    _write(os.path.join(run_dir, "sightings.csv"), "ts,drone,id\n")
    res = vr.check_smoke(run_dir=run_dir, repo=REPO, enabled=True)
    assert not res.ok
    assert any("NO rows" in p for p in res.problems)


def test_smoke_enabled_positive_evidence_passes(tmp_path):
    run_dir = str(tmp_path)
    _write(os.path.join(run_dir, "sightings.csv"),
           "ts,drone,id\n1.0,alpha,7\n2.0,alpha,7\n")
    _write(os.path.join(run_dir, "mission.jsonl"),
           json.dumps({"kind": "action_complete", "action": "navigate"}) + "\n")
    res = vr.check_smoke(run_dir=run_dir, repo=REPO, enabled=True)
    assert res.ok, res.problems


# ============================================================
# main() exit codes
# ============================================================
def test_main_no_suite_exits_zero(capsys):
    rc = vr.main(["--no-suite"])
    out = capsys.readouterr().out
    assert rc == 0
    assert "VERDICT: PASS" in out


def test_main_smoke_without_run_dir_exits_nonzero(capsys):
    rc = vr.main(["--no-suite", "--smoke"])
    assert rc == 1


def _read(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        return f.read()
