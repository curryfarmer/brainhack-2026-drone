# Documentation index

Drone autonomy stack for the **DSTA BrainHack 2026 RoboVerse** challenge — PX4 SITL + Gazebo Harmonic + MAVSDK + YOLO, hunting yellow/red fuel barrels in a GNSS-denied arena.

## Which doc do I need?

| Doc | What it covers |
|---|---|
| [Deployment guide](quali/deployment.md) | Getting the code onto a machine, Python env, Gazebo/PX4 install, `run.sh` drop-in workflow, configuration, pre-run checklist, troubleshooting (incl. MAVSDK gRPC) |
| [Simulator testing](quali/simulator-testing.md) | The PX4 SITL + Gazebo testing software you're given: smoke tests, dev loop, example/diagnostic scripts, sensor topics |
| [Codebase reference](quali/codebase.md) | File-by-file inventory, data pipeline diagram, module APIs, hardcoded constants, known issues |
| [Training pipeline](quali/training-pipeline.md) | Capturing images in sim, labelling (Roboflow/labelImg/X-AnyLabeling), validate → split → train → deploy YOLO weights |
| [Design rationale](quali/design-rationale.md) | Why the mission is built the way it is: lawnmower coverage, supervisor restart, dedup scoring, risk register |
| [Authorship & history](AUTHORSHIP.md) | Who/what wrote which files (including AI-assisted edits), the doc reorganization history, and the May 22 regression |
| [Finals](finals/README.md) | Real-drone stage — hardware/software stack extracted from the official example code (Hula drone, RealSense, RKNN NPU, UWB); format/scoring await the briefing |

## Two mission entry points

The repo currently has **two** autonomous mission mains. Know which one you are running:

- `qualifier_run.py` — lawnmower coverage + asyncio supervisor (mission loop + detection loop + crash restart). This is the documented design; see the [design rationale](quali/design-rationale.md). Outputs `runs/<timestamp>/barrels.csv`.
- `qualifier_main.py` — teammate-written DFS exploration over a grid of cells (2 m cells in the current copy). The copy at repo root was uploaded 2026-05-22; the **newest iteration lives in `qualifier_main updates/test2.5/`**. Outputs `barrels.json` + `visited_cells.json`. See the [codebase reference](quali/codebase.md).

They are independent — pick one per run; do not launch both.

Related: `best.pt` (trained barrel weights, 6.2 MB) now exists at repo root, but `model_config.json` still points at the COCO `yolov10n.pt` placeholder — verify which weights are actually wired in before a scored run (details in the [simulator testing guide](quali/simulator-testing.md)).

## About these docs

Reorganized **2026-06-06** from the original root `README`/`CONTEXT`/`APPROACH`/`HANDOVER` documents (since deleted). Content was carried over, restructured by task, and updated where the repo had drifted — e.g. stale paths that pointed at a separate code subdirectory (the repo root *is* the code directory now). Full history, including what was lost and recovered around the May 22 upload regression, is in [AUTHORSHIP.md](AUTHORSHIP.md).

Two reference-code archives live alongside the docs — **read-only baselines, never edit or import from them**:

- [`quali/example_code/`](quali/example_code/) — the **pristine, untampered** qualifier handout (verified: no Claude edit markers, pre-patch `Detector.py`/`drone_control.py`, legacy `udp://` strings). The repo root holds the *live, modified* versions; diff against this baseline to see exactly what the team changed.
- [`finals/example_code/`](finals/example_code/) — the official finals example scripts (Hula/RealSense/RKNN/UWB stack).

These docs are also the standing context base for planning future work — keep them current when the code moves.
