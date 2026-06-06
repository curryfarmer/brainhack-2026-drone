# Authorship & Provenance

This codebase has three distinct authors: **Claude Code sessions** (run locally, committing as "Mr Stark"), **organizer-provided starter code** (carried into the repo by the initial commit), and **teammate uploads** through the GitHub web UI (the "ohhwengjo" commits). Knowing which is which matters for finals prep:

- Claude-generated files were written against a documented design (see the [design rationale](quali/design-rationale.md)) and follow consistent conventions — they are the safest base to extend.
- Starter-code provenance is *inferred*, not certain — treat those scripts as reference examples, not battle-tested infrastructure.
- The teammate uploads on May 22 added real value (trained weights, a second mission main) **but also overwrote several newer files with older snapshots** — see [The May 22 regression](#the-may-22-regression) below.

This doc is the single place that records who/what wrote each part, what was lost in the regression, and where the recovered content now lives. For what each file *does*, see the [codebase guide](quali/codebase.md). For the rest of the doc set, see the [docs index](README.md).

> Historical note: the original four root markdown docs — `README.md` (488-line version), `CONTEXT.md`, `APPROACH.md`, `HANDOVER.md` — were all Claude-written. They have been deleted and their content reorganized into the `docs/` tree as part of this restructure. Do not look for them at the repo root.

## Commit-level provenance

Full history as of the restructure, oldest first. Commits authored as **Mr Stark** were Claude Code sessions on the dev machine (that was the configured git identity). Commits authored as **ohhwengjo** were made through the GitHub web UI ("Add files via upload" / "Create" / "Delete").

| Commit | Date | Author | Message | Provenance |
|---|---|---|---|---|
| `289ec7d` | 2026-05-20 | Mr Stark | Initial commit: drone perception + avoidance stack | Claude Code session. Committed *everything* at once: the Claude-written stack **and** the presumed organizer starter scripts (see sections below for the split). |
| `9d747ff` | 2026-05-20 | Mr Stark | fix: standardize MAVSDK connect string + handle gRPC drops | Claude Code (`udpin://0.0.0.0:14540` fix across scripts) |
| `409a847` | 2026-05-20 | Mr Stark | docs: add clone + PX4 SITL setup to install instructions | Claude Code (README §1.0 / §1.4) |
| `a9dc297` | 2026-05-20 | Mr Stark | fix: detect dead mavsdk_server early + troubleshooting docs | Claude Code (README §7.1) |
| `0126598` | 2026-05-20 | Mr Stark | refactor: delegate collect_yolo_data flight to drone_control.Drone | Claude Code |
| `cbbcbb6` | 2026-05-20 | Mr Stark | feat: screen-capture fallback + README §10/§11 (AI edits + waypoints) | Claude Code (`mss` fallback in `collect_yolo_data.py`) |
| `83c7314` | 2026-05-21 | Mr Stark | feat: self-healing mavsdk cleanup + run.sh launcher for drop-in workflow | Claude Code (`run.sh`, `_kill_stale_servers()`) |
| `5127897` | 2026-05-21 | Mr Stark | docs: §13 labelling pipeline (Roboflow + labelImg + X-AnyLabeling) | Claude Code — **last good commit before the regression** |
| `16789f0` | 2026-05-22 | ohhwengjo | Create peepepe | GitHub web upload (placeholder file) |
| `9a01e32` | 2026-05-22 | ohhwengjo | Add files via upload | GitHub web upload — **the regression commit** (see next section). Added `qualifier_main.py`, `barrel_logger.py`, `KEY2.py`, `best.pt`, `barrels.json`, `visited_cells.json`; overwrote 9 existing files with older snapshots. |
| `8d777c8` | 2026-05-22 | ohhwengjo | Merge pull request #1 from curryfarmer/test2.1 | GitHub web merge of PR #1 |
| `8db828e` | 2026-05-22 | ohhwengjo | Delete poopoo directory | GitHub web cleanup |
| `549fb57` | 2026-05-22 | ohhwengjo | Add files via upload | GitHub web upload — updated root `qualifier_main.py` |
| `973c2bf` | 2026-05-22 | ohhwengjo | Create test2.3 | GitHub web upload (`qualifier_main updates/test2.3` marker) |
| `250fdf6` | 2026-05-22 | ohhwengjo | Create a | GitHub web upload (placeholder) |
| `8d80c76` | 2026-05-22 | ohhwengjo | Add files via upload | GitHub web upload — `qualifier_main updates/test2.4/` (`Detector.py` + `qualifier_main.py`) |
| `79fd22a` | 2026-05-22 | ohhwengjo | Delete qualifier_main updates/test2.4/a | GitHub web cleanup |
| `e1bab36` | 2026-05-22 | ohhwengjo | Create a | GitHub web upload (placeholder) |
| `6501101` | 2026-05-22 | ohhwengjo | Add files via upload | GitHub web upload (`test2.5` APPROACH snapshot) |
| `f0e3032` | 2026-05-22 | ohhwengjo | Delete qualifier_main updates/test2.5/a | GitHub web cleanup |
| `93314fe` | 2026-05-22 | ohhwengjo | Add files via upload | GitHub web upload — full repo snapshot under `qualifier_main updates/test2.5/test2.5/test2.5/` (incl. `__pycache__`) |
| `53494c0` | 2026-05-22 | ohhwengjo | Delete qualifier_main updates/test2.5/test2.5/APPROACH.md | GitHub web cleanup |

## The May 22 regression

Commit `9a01e32` ("Add files via upload") was a GitHub web upload of a local working copy that was **older** than what was already on `main`. GitHub's drag-and-drop upload silently replaced the newer files. What was lost:

| File | What was lost |
|---|---|
| `README.md` (root) | **389 lines** (488 → ~296): get-the-code instructions (§1.0), PX4 SITL setup (§1.4), MAVSDK gRPC troubleshooting (§7.1), image-capture workflow (§9), AI-edits log (§10), waypoint generation today + roadmap (§11), `run.sh` drop-in workflow (§12), labelling pipeline (§13) |
| `CONTEXT.md` | §8 "MVP algorithm (`qualifier_run.py`)" — the walkthrough of the mission/detection loop design |
| `Detector.py` | The `config_path` constructor parameter (the `model_config.json`-loading patch) — **restored 2026-06-06** |
| `requirements.txt` | `mss` (screen-capture fallback), `torch>=2.1`, and the `ultralytics<9` pin — **restored 2026-06-06** |
| `drone_control.py` | `_kill_stale_servers()` self-healing cleanup at the top of `connect()` (auto-kills zombie `mavsdk_server` before binding UDP :14540) — **restored 2026-06-06** |
| `get_position.py`, `imutest.py`, `keyboardcontrol.py` | Connect strings reverted from `udpin://0.0.0.0:14540` back to the legacy `udp://:14540` scheme — the exact scheme the old troubleshooting docs warned segfaults `mavsdk_server` under MAVSDK 2.x — **restored 2026-06-06** (all back on `udpin://`) |
| several files | "Edited by Claude" provenance markers stripped from file headers |

The last good versions of every regressed file are recoverable from git (and were re-applied to the working tree on 2026-06-06):

```bash
git show 5127897:README.md          # the full 488-line README
git show 5127897:CONTEXT.md         # includes §8 MVP algorithm
git show 5127897:Detector.py        # has the config_path parameter
git show 5127897:requirements.txt   # has mss / torch>=2.1 / ultralytics<9
git show 5127897:drone_control.py   # has _kill_stale_servers()
```

**Where things stand after the docs restructure:**

- The lost **documentation** content has been recovered and reorganized into the `docs/` tree: the labelling pipeline (§13) and image-capture workflow (§9) live in the [training pipeline guide](quali/training-pipeline.md); the PX4 SITL setup (§1.0/§1.4), gRPC troubleshooting (§7.1), and `run.sh` drop-in workflow (§12) live in the [deployment guide](quali/deployment.md); the waypoint roadmap (§11) and MVP algorithm (CONTEXT §8) live in the [design rationale](quali/design-rationale.md); the AI-edit log (§10) is reproduced verbatim at the bottom of this document.
- The lost **code** changes were restored to the working tree on 2026-06-06 from `5127897` — `Detector.py` (`config_path`), `requirements.txt` (`mss` / `torch>=2.1` / `ultralytics<9`), `drone_control.py` (`_kill_stale_servers()`), and the `udpin://` connect strings are all back. The known-issues entries in the [codebase guide](quali/codebase.md) record the history.

## Claude-generated (high confidence)

All of the following were written by Claude Code (initial commit `289ec7d` plus the follow-up Mr Stark commits). These follow a documented design and are the recommended base for finals work.

**Qualifier MVP** (the documented mission design — see the [design rationale](quali/design-rationale.md)):

- `qualifier_run.py` — asyncio supervisor: mission loop + detection loop + crash-restart within the 10-minute budget
- `coverage.py` — pure lawnmower (boustrophedon) waypoint generator
- `detection_to_world.py` — bbox + depth + pose → world-NED back-projection
- `barrel_log.py` — thread-safe dedup + scoring + crash-safe CSV persistence

Note: `qualifier_run.py` is now one of **two** mission entry points. The other, `qualifier_main.py` (teammate-written DFS exploration, see below), took over as the actively-iterated main on May 22. Both exist in the repo; the [codebase guide](quali/codebase.md) covers the differences.

**Perception + planning stack:**

- `Detector.py` (YOLO inference wrapper; `config_path` regressed May 22, restored 2026-06-06), `GlobalMapper.py`, `GlobalMapper_new.py`, `AvoidancePlanner.py`, `PointCloudPlanner.py`, `PointCloudPlanner_new.py`, `VelocityPlanner.py`, `RRTStarPlanner.py`
- `avoid.py`, `avoid_with_detect.py`, `vel_avoidance.py` — earlier integrated avoidance demos. `avoid.py` was once labelled "current best" in the old docs; that is **no longer true** — it has been superseded by the two qualifier mains.
- `depthcloud.py`, `depth_receiver.py`, `get_position_with_task.py`, `top_down.py`

**Training pipeline** (the end-to-end data → weights flow, documented in the [training pipeline guide](quali/training-pipeline.md)):

- `collect_yolo_data.py` (capture during manual flight, with `mss` screen-grab fallback), `validate_labels.py`, `split_train_val.py`, `gen_data_yaml.py`, `gen_smoke_data.py`, `train_yolo.py`, `eval_model.py`, `deploy_model.py`, `import_roboflow.py`, and the `pipeline.py` orchestrator

**Flight wrappers and launcher:**

- `drone_control.py` (the `Drone` class used by `qualifier_run.py`; `_kill_stale_servers()` regressed May 22, restored 2026-06-06), `drone_control_new.py`
- `run.sh` — drop-in launcher (kills stale `mavsdk_server`, names the UDP :14540 owner, then `exec`s Python). Added in `83c7314`.

**Documentation:** all four original root markdown docs (`README.md`, `CONTEXT.md`, `APPROACH.md`, `HANDOVER.md`) were Claude-written; their content now lives in `docs/`.

**About the `_new` / `_old` suffix files:** `GlobalMapper_new.py`, `PointCloudPlanner_new.py`, `RRTExample_new.py`, `drone_control_new.py`, `Train_YOLO_Models_new.ipynb`, `get_video_old.py` are all Claude refactor variants committed in the same initial commit — they are *not* human rewrites of Claude code (or vice versa). The `_new` files are generally the cleaner take; the originals were kept for reference.

## Likely organizer-provided starter code

These read as minimal pedagogical examples typical of a competition starter kit: one concept per script, simple top-to-bottom flow, tutorial-style comments.

- Flight basics: `basic_offboard.py`, `takeoff_and_land.py`, `go_to.py`
- Planner/detector examples: `RRTExample.py` (and its Claude refactor variant `RRTExample_new.py`), `UseDetectorExample.py`
- Telemetry probes: `get_battery.py`, `get_position.py`, `get_depth.py`, `get_flightmode.py`, `imu.py`, `imutest.py`, `is_arm_air.py`
- Camera/media: `get_video.py`, `get_video_old.py`, `photo.py`, `save_photo.py`, `gzphotodetectorsaver.py`
- Diagnostics/control: `drone_diagnostics.py`, `keyboardcontrol.py` (later edited by Claude — `udpin://` scheme fix; regressed May 22, restored 2026-06-06)

**Important caveat:** all of these were committed by Claude Code in the initial commit `289ec7d` — git blame attributes them to "Mr Stark". The organizer-provided classification is **inferred from style only** and is not certain. A few small probes (e.g. `depthtest.py`) are ambiguous and could belong to either group. Treat these as reference examples: fine to read and crib from, but the production path (`drone_control.py` → `qualifier_run.py` / `qualifier_main.py`) is what has actually been exercised.

## Teammate contributions (ohhwengjo, May 22)

Uploaded through the GitHub web UI on 2026-05-22 (includes the merge of PR #1 from `curryfarmer/test2.1`):

- `qualifier_main.py` — the second mission entry point: DFS exploration over a cell grid (instead of the pre-baked lawnmower; 2 m cells in the current copy), writing `barrels.json` and `visited_cells.json` as it goes. **Version warning:** the root copy (added in `9a01e32`, updated in `549fb57`) is OLDER than the copy under `qualifier_main updates/test2.5/test2.5/test2.5/` (uploaded later in `93314fe`). Diff them before reusing either — see the [codebase guide](quali/codebase.md).
- `barrel_logger.py` — teammate's barrel logging module. Note the name collision with Claude's `barrel_log.py`; they are different implementations.
- `KEY2.py` — keyboard-control variant.
- `best.pt` — **6.2 MB trained YOLO weights** (the actual barrel-trained model, vs the COCO-pretrained `yolov10n.pt` placeholder). Caveat: `model_config.json` still points at `yolov10n.pt` — before any scored run, verify which weights are actually wired in.
- `barrels.json`, `visited_cells.json` — run artifacts from a DFS test flight (committed output, not source).
- The entire `qualifier_main updates/` tree — iteration snapshots: `test2.3` (marker file), `test2.4/` (`Detector.py` + `qualifier_main.py`), and `test2.5/test2.5/test2.5/` (a triple-nested full repo snapshot, including `__pycache__` and a second `best.pt`/`yolov10n.pt`). The nesting is an upload artifact, not intentional structure.
- Re-uploaded copies of `Train_YOLO_Models.ipynb` / `Train_YOLO_Models_new.ipynb` inside the `test2.5` snapshot (the root notebooks are from the Claude initial commit).

For finals: the DFS exploration logic and the trained `best.pt` are the valuable pieces here. The snapshot tree should be reconciled (pick the newest `qualifier_main.py`, promote it, delete the rest) rather than carried forward as-is.

## AI-edit log (recovered)

The following is reproduced **verbatim** from §10 of the pre-regression 488-line README (recoverable via `git show 5127897:README.md`), as a historical record of which files Claude edited during the May 20–21 debug sessions and the rules those sessions followed. Section references (§10, §12, etc.) refer to the *old* README, which no longer exists; the recovered content now lives in the `docs/quali/` guides. Note also that the regression commit later stripped the "Edited by Claude" markers from several of these files and reverted some of the listed fixes; the fixes (and markers) were restored to the working tree on 2026-06-06, so the table below — which describes the state *as of commit `5127897`* — once again matches the current code.

> This file and a handful of others have been touched by Claude (Anthropic's AI assistant) during debug sessions. Every Claude-edited file has a top-of-file marker:
>
> - Python: `# Edited by Claude — <one-line reason>. See README §10.`
> - Markdown: `<!-- Edited by Claude — ... -->`

### Files Claude has edited (historical §10.1)

| File | What changed |
|---|---|
| `collect_yolo_data.py` | Round 1 udpin scheme; Round 2 sanity-ping + arm hardening; Round 3 ripped out hand-rolled MAVSDK and delegated to `drone_control.Drone`; screen-capture fallback added in `_grab_screen()` |
| `keyboardcontrol.py` | `MAVSDK_ADDRESS` changed `udp://:14540` → `udpin://0.0.0.0:14540` |
| `get_position.py` | Same `udpin://` fix + comment update |
| `imutest.py` | Same `udpin://` fix |
| `takeoff_and_land.py` | Stale log message updated to match `udpin://` |
| `requirements.txt` | Added `mss` for the screen-capture fallback |
| `drone_control.py` | Round 4 — `_kill_stale_servers()` called at top of `connect()` to auto-clean any zombie `mavsdk_server` before the wrapper binds UDP :14540 |
| `run.sh` | Round 4 — new drop-in launcher; kills stale servers, names the port owner, then `exec`s the script (see §12) |
| `README.md` | This document — install/troubleshooting/inventory sections |

### Rules the AI follows (historical §10.3)

> 1. **Reuse first.** Use `drone_control.Drone`, `basic_offboard.py`, `coverage.py`, etc. before writing new code.
> 2. **Document in README.** Anything new lands in this file, in a section like the one you're reading. Keep it human-readable — no terse compression here.
> 3. **Mark edits.** Every Claude-edited file gets the `Edited by Claude` marker; every new file is added to §10.1.
> 4. **Smallest change.** Minimum lines to fix the problem. No feature creep, no speculative abstractions.
> 5. **Surface failures.** No silent `except Exception: pass`. Print a one-line diagnostic and either continue or re-raise.

These rules remain a good contract for any future AI-assisted sessions during finals prep — with rule 2 updated to "document in `docs/`" now that the monolithic README is gone. See the [docs index](README.md) for where each topic lives, and the [finals workspace](finals/README.md) for what comes next.
