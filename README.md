# BrainHack 2026 — RoboVerse Drone

> **New here? Read [`REPO_MAP.md`](REPO_MAP.md) first** — it states what the drone actually flies
> (`finals/`) versus what is simulation, pre-comp, reference, or artifacts.

This repo holds two bodies of work:
- **`finals/` — the competition deliverable** (SWARM challenge: 3 HULA drones over Wi-Fi via
  pyhulax). Start at [`finals/docs/module_map.md`](finals/docs/module_map.md); entry
  `python -m finals.main --profile real --config finals/configs/landing_real.json`.
- **`precomp/` — the Challenge-1 qualifier stack + YOLO training loop** (PX4 SITL + Gazebo Harmonic
  + MAVSDK + ultralytics). Filed away from the competition code; not flown onsite. Documented below.

---

## Qualifier (pre-comp) — quick start

Original qualifier task: find yellow (50 pt) and red (100 pt) fuel barrels in a 40×40×8 m
GNSS-denied arena within a 10-minute scored run.

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install --upgrade pip wheel setuptools
pip install -r requirements.txt        # torch + Gazebo bindings need extra steps — see deployment guide

# With PX4 SITL running — run precomp scripts FROM THE REPO ROOT (cwd contract: classes.txt/data/ stay at root):
python precomp/qualifier_run.py --no-detector --budget 60    # 60 s flight test, no YOLO
```

Full install (Gazebo Harmonic, PX4 SITL build, torch flavour): [docs/quali/deployment.md](docs/quali/deployment.md)

## Two qualifier mission entry points (now under `precomp/`)

| Script | What it is |
|---|---|
| `precomp/qualifier_run.py` | Lawnmower coverage + crash-restart supervisor — the documented design ([rationale](docs/quali/design-rationale.md)) |
| `precomp/qualifier_main.py` | DFS cell-grid exploration — team iteration ([details](docs/quali/codebase.md)) |

## Documentation

| Doc | What's in it |
|---|---|
| [docs/README.md](docs/README.md) | Index — start here |
| [docs/quali/deployment.md](docs/quali/deployment.md) | Install, configure, run the mission, pre-run checklist, troubleshooting |
| [docs/quali/simulator-testing.md](docs/quali/simulator-testing.md) | The PX4 SITL + Gazebo testing software, smoke tests, dev loop, example scripts |
| [docs/quali/codebase.md](docs/quali/codebase.md) | Architecture, file inventory, module APIs, constants, known issues |
| [docs/quali/training-pipeline.md](docs/quali/training-pipeline.md) | YOLO pipeline: capture → label → train → deploy |
| [docs/quali/design-rationale.md](docs/quali/design-rationale.md) | Competition rules, why it's built this way, risk register, open questions |
| [docs/AUTHORSHIP.md](docs/AUTHORSHIP.md) | Provenance: what Claude wrote vs organizer starter code vs teammate uploads |
| [docs/finals/README.md](docs/finals/README.md) | Finals stack (Hula drone, RealSense depth cam, RKNN NPU, UWB positioning) + sim→real migration plan |

## Status (precomp)

- Trained weights live under the gitignored `models/` (see `models/latest_path.txt` + `model_config.json`); verify which weights are wired in before any scored run. Loose `*.pt` are gitignored.
- A May 22 upload regressed several files (`Detector.py` `config_path`, `requirements.txt` deps, `drone_control.py` self-healing); restored 2026-06-06 from git commit `5127897`. History in [docs/AUTHORSHIP.md](docs/AUTHORSHIP.md).
- Pose-drift under GNSS-denied remains the top qualifier risk — see [R1](docs/quali/design-rationale.md#9-risks).
