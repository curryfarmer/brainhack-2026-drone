# Smoke-test catalog — sorted by stack / environment

Every smoke we own, grouped by **where it runs**. Three environments, no overlap:

1. **SIM-SITL** — a PX4-SITL + Gazebo VM (`ssh bhvm`). Proves `finals/` logic before the
   real window. No HULA hardware.
2. **ONSITE-HARDWARE** — the real HULA fleet + the operator laptop, at the cage. Props-off
   or short hops.
3. **PRECOMP-QUALIFIER** — filed in `precomp/`; the Challenge-1 drone utils + YOLO training
   smoke. Not flown onsite. Run **from the repo root** (cwd contract — see `REPO_MAP.md`).

Columns: **Smoke** · **Run** · **What it proves** · **Stack deps**. Dev-only tools that are
*not* smokes are listed at the bottom.

---

## 1. SIM-SITL smokes (VM only — PX4 + Gazebo Harmonic)

Orchestration is in `sim/run_*.sh`; the Python helpers under them are the transport + asset
+ check pieces. Headline subcommand shown; `bash sim/run_*.sh` with no args prints the rest.

| Smoke | Run | What it proves | Stack deps |
|---|---|---|---|
| `sim/run_landing.sh` | `bash sim/run_landing.sh land1` (L1, 1 drone) / `land3` (L2, 3 drones) | Challenge-2A LANDING rehearsal: full `[takeoff, navigate, land_on_pad]` flies to VERIFIED_LANDING; 3-drone staggered + serialized descent | PX4 SITL, gz-harmonic, gz→TCP bridge |
| `sim/run_vision.sh` | `bash sim/run_vision.sh track3` / `dyn3` / `lanes3` | Convoy vision-in-the-loop: onboard cam → ArUco decode → sightings → tracker/coverage over 3 PX4 cam-drones | PX4 SITL, gz, cv2 (ArUco) |
| `sim/run_convoy.sh` | `bash sim/run_convoy.sh all` | The convoy WORLD itself: cars drive, markers decode in-frame (world validity, not the package) | gz-harmonic, ros_gz, cv2 |
| `sim/launch_sitl.sh` | `bash sim/launch_sitl.sh start N` | Bare PX4-SITL multi-instance bring-up (the substrate the gates run on) | PX4 SITL |
| `sim/sitl_smoke.py` | `python sim/sitl_smoke.py` | Raw-MAVSDK environment smoke (SIM-0: the VM can talk MAVSDK at all) | mavsdk, PX4 SITL |
| `sim/check_detection.py` | `python sim/check_detection.py` | Detection check against a running convoy world | gz, cv2 |
| `sim/pty_q_harness.py` | `python sim/pty_q_harness.py -- <cmd>` | The operator `q`+Enter abort drill under a PTY (abort needs a TTY) | stdlib pty |
| `sim/gz_camera_bridge.py` | (started by `run_*.sh bridge`) | The SIM-4 frame transport: gz camera topic → localhost TCP (no gz binding in `finals/`) | gz python |
| `sim/convoy_driver.py` | (started by `run_convoy.sh`) | Drives convoy cars via rclpy Twist over ros_gz — `--delay-s` holds straight-lane cars through the settle | rclpy, ros_gz |
| `sim/gen_markers.py` | `python sim/gen_markers.py` | Generates the ArUco marker + convoy-robot world assets | cv2 |
| `sim/gz_video_record.py` | `python sim/gz_video_record.py --topic T` | Records a gz camera topic to mp4 (evidence capture, NAV-9) | gz python |

> The bound for **all** SIM-SITL gates: the gz server is **single-threaded** = the lockstep
> master for every PX4 instance (camera `update_rate` had to drop 15→5 Hz for 3 drones to fly).
> SIM-ONLY — real HULA renders per-drone. Full gate list + evidence: `docs/finals/onsite_test_plan.md`.

## 2. ONSITE-HARDWARE smokes (real HULA fleet + laptop)

Order at the window: **`hula_smoke` → `--preflight-only` → `flight_test`**. `calibrate_origin`
is run once, before, when the cage is measured.

| Smoke | Run | What it proves | Stack deps |
|---|---|---|---|
| `finals/tools/hula_smoke.py` | `python -m finals.tools.hula_smoke` | OFFLINE no-flight fleet bring-up: discover → connect → telemetry → video → ArUco decode, props-off | pyhulax, cv2 |
| `finals.main --preflight-only` | `python -m finals.main --profile real --config finals/configs/landing_real.json --preflight-only` | The **exact mission fleet** runs the P0-P9 hardware gate (P10 GO skipped, never flies). Builds what flies. | pyhulax, cv2 |
| `finals/tools/flight_test.py` | `python -m finals.tools.flight_test --drones 1 --live` (then `--drones 3`) | One-command real flight test: a HULA actually takes off / lands under the stack | pyhulax |
| `finals/tools/calibrate_origin.py` | `python -m finals.tools.calibrate_origin` | Turns measured cage numbers into a checkable origin frame (config, not flight) | stdlib |

> The hardware gate detail (P0-P10, bench checks B1-B8, abort drills) is the runbook:
> `docs/finals/onsite_test_plan.md`. The software composition behind these is proven green by
> the e2e suite — see `docs/finals/e2e_coverage.md`.

## 3. PRECOMP-QUALIFIER smokes (filed in `precomp/`, run from repo root)

Challenge-1 era; kept for reference + retraining, **not** flown onsite. cwd contract: invoke as
`python precomp/<script>.py` from the repo root so `classes.txt` / `data/` / `models/` resolve.

| Group | Scripts | What they prove | Stack deps |
|---|---|---|---|
| Drone telemetry/util smokes | `get_battery`, `get_position`(`_with_task`), `get_flightmode`, `get_video`, `get_depth`, `drone_diagnostics` | The PX4 link reports each signal | mavsdk |
| Manual flight smokes | `takeoff_and_land`, `basic_offboard`, `go_to`, `keyboardcontrol`, `top_down` | Offboard velocity / position commands move the SITL drone | mavsdk |
| Camera/depth smokes | `photo`, `save_photo`, `depthtest`, `depthcloud`, `depth_receiver`, `imu`, `imutest` | Sensor capture pipelines (RGB, depth cloud, IMU) | mavsdk, cv2/realsense |
| Training-pipeline smoke | `gen_smoke_data.py`, then `pipeline.py` | The YOLO capture→label→train loop runs end-to-end on synthetic data | ultralytics, torch |

---

## Dev tools (NOT smokes — no hardware, no sim)

Listed so they are not miscategorized. Pure-software helpers over recorded data / the suite:

| Tool | Run | Purpose |
|---|---|---|
| `finals/tools/verify_runbook.py` | `python -m finals.tools.verify_runbook` | Asserts the runbook + code + configs are in sync (configs load, bench P0, suite green) |
| `finals/tools/replay_plot.py` | `python -m finals.tools.replay_plot <run_dir>` | Plots a recorded run's telemetry / sightings (post-hoc analysis) |
