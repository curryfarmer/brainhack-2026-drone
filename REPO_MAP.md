# REPO MAP — what is competition code, what is not

One screen. If you are at the onsite window and need to know **what the drone runs**, read
the COMPETITION row and stop.

| Bucket | Path | Runs on the drone? | What it is |
|---|---|---|---|
| **COMPETITION** | `finals/` | **YES — this is the deliverable** | The mission stack flown on 3 HULA drones via pyhulax over Wi-Fi. Entry: `python -m finals.main --profile real --config finals/configs/{landing_real,convoy_real}.json` |
| **SIM** | `sim/` | No (VM only) | PX4-SITL + Gazebo rehearsal that *proves* `finals/` logic before the real window. Never flown onsite. |
| **PRECOMP** | `precomp/` | No | Filed-away pre-competition code: Challenge-1 qualifier nav, the YOLO **training loop**, and bare drone smoke scripts. Kept for reference + retraining; not part of the flight stack. |
| **REFERENCE** | `docs/finals/example_code/`, `docs/quali/example_code/` | No | Pristine, read-only official handout/example code (incl. `kolomee.py`, `hula_connection.py`, `dola.py`). Audited, never edited; `finals/` vendors-with-fixes, never imports. |
| **ARTIFACTS** | `runs*/`, `logs/`, `data/`, `models/`, `__pycache__/`, caches | No | Gitignored, regenerable outputs. Safe to delete. |

## COMPETITION — the three sub-buckets inside `finals/`

| Sub-bucket | Examples | Notes |
|---|---|---|
| **FLIGHT-RUNTIME** | `main.py`, `config.py`, `guards.py`, `mission/**`, `flight/{pyhulax_adapter,dead_reckon,discovery,proximity,adapter}.py`, `vision/{pyhulax_video,perception,aruco,detector,video}.py`, `planning/**` | What executes under `--profile real`. Real-flight SDK deps: **pyhulax** (drone) + **cv2** (ArUco). |
| **ONSITE-BENCH-TOOL** | `python -m finals.main --preflight-only` (`preflight.py`, P0–P10), `finals/tools/{hula_smoke,flight_test,calibrate_origin}.py` | Real hardware, props-off / bench. Not flight themselves. |
| **DEV-TEST-SIM-ONLY** | `finals/tests/**`, `flight/{sitl_adapter,mock_adapter}.py`, `vision/gazebo_video.py`, `finals/tools/{replay_plot,verify_runbook}.py` | Never touched in a real run. SITL/mock/gazebo backends + the test suite + dev tools. |

## PRECOMP — what got filed away (in `precomp/`)

Flat folder, ~49 scripts + 2 notebooks + `run.sh`. Three groups (see `docs/finals/smokes.md` for the
smoke catalog): **qualifier nav** (`drone_control.py`, `qualifier_run/main.py`, `GlobalMapper.py`,
`RRT*`, `AvoidancePlanner.py`, `coverage.py`, …), **YOLO training loop** (`train_yolo.py`, `pipeline.py`,
`collect_yolo_data.py`, `gen_*`, `validate_labels.py`, `eval_model.py`, `deploy_model.py`, `Detector.py`,
notebooks), **drone smokes** (`get_*.py`, `takeoff_and_land.py`, `basic_offboard.py`, `keyboardcontrol.py`,
`depth*`, `imu*`, …).

> **cwd contract:** precomp scripts read `classes.txt` / `data/` / `models/` / `model_config.json` by
> relative path. Run them **from the repo root** (`python precomp/train_yolo.py`), not from inside
> `precomp/`. Those I/O artifacts deliberately stay at the repo root.

## See also

- `finals/docs/module_map.md` — per-module status table (start-here for a finals session).
- `docs/finals/onsite_test_plan.md` — the 2-hour window runbook (preflight P0–P10, gates A–G).
- `docs/finals/smokes.md` — smoke-test catalog (SIM-SITL / ONSITE-HARDWARE / PRECOMP).
