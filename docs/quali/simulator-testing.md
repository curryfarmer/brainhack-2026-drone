# Simulator & Testing Guide

How to test this stack — from zero-dependency module self-tests up to full missions against the PX4 SITL + Gazebo Harmonic simulator the organizers provide. Install steps for every component live in the [deployment guide](deployment.md); this page assumes the environment is already set up and focuses on *running* things.

## The simulation stack you're given

The qualifier "testing software" is three cooperating pieces:

1. **PX4 SITL** — the autopilot firmware compiled for software-in-the-loop. It flies the simulated vehicle `x500_vision_0`, runs the EKF2 state estimator (NED frame), and speaks MAVLink on UDP. Every script in this repo expects it at `udpin://0.0.0.0:14540` (the PX4 SITL default).
2. **Gazebo Harmonic** — the physics + sensor simulator, running the `roboverse` world. It publishes the vehicle's RGB camera (IMX214 sensor) and depth camera over **gz-transport** topics, which the Python side reads via the `gz.transport13` / `gz.msgs10` bindings (system packages, not PyPI — see [deployment](deployment.md)).
3. **MAVSDK gRPC bridge** — every MAVSDK `System()` in Python spawns a `mavsdk_server` subprocess. That server talks MAVLink to PX4 on UDP `:14540` and exposes a gRPC API to your script on `localhost:50051`. Most "drone won't connect" failures are actually a stale `mavsdk_server` holding the UDP port — `run.sh` auto-cleans these, and `drone_control.Drone.connect()` self-cleans too via a `_kill_stale_servers()` helper (regressed in the May 22 upload, restored 2026-06-06); see [deployment](deployment.md) for the troubleshooting recipes.

To launch the simulator locally:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_vision     # x500 quad with RGB + depth cameras
```

This boots PX4 on `:14540` and spawns the vehicle in Gazebo. Leave it running in its own terminal — every script in this repo will discover it. Older notes in this repo referenced `make px4_sitl gz_x500_depth`; whichever target you use, what matters is that the world publishes `/depth_camera` and the IMX214 RGB topic for vehicle `x500_vision_0` — verify with `gz topic -l` before trusting any config default.

Without Gazebo Harmonic installed you can still import `qualifier_run` for unit testing — the sensor/depth paths will refuse to start, but the pure-logic modules below work everywhere.

## Smoke tests (no simulator needed)

### Pure-module self-tests

The three pure-logic modules each have a `__main__` self-test. Run these first after any environment change or code edit — they need **no PX4, no Gazebo, no GPU**:

```bash
source .venv/bin/activate

# Smoke-test the pure-logic modules first (no PX4 / Gazebo needed)
python coverage.py             # prints generated waypoints
python detection_to_world.py   # prints a synthetic back-projection
python barrel_log.py           # prints dedup test results
```

If `coverage.py` prints a sensible boustrophedon waypoint list, `detection_to_world.py` back-projects its synthetic bbox to a plausible NED point, and `barrel_log.py` reports its dedup tests passing, the math core is healthy.

The training-pipeline scripts (`validate_labels.py`, `split_train_val.py`, `gen_data_yaml.py`) also run offline against the fixtures in `tests/fixtures/` — see the [training pipeline guide](training-pipeline.md).

### First flight test (needs PX4 SITL, no YOLO)

Once the simulator is up, the cheapest end-to-end check is a flight-only mission — full takeoff/coverage/land path with the detector disabled:

```bash
# Flight-only run, 60-second budget, no YOLO
python qualifier_run.py --no-detector --budget 60
```

A fresh clone is runnable as-is for `--no-detector` smoke tests — no trained weights required.

## Dev loop with the simulator

A typical iteration looks like this:

**Terminal 1 — simulator.** Start PX4 SITL + Gazebo as above and leave it running.

**Terminal 2 — mission.** Activate the venv at the repo root and launch through `run.sh` (it kills zombie `mavsdk_server` processes and names whoever owns UDP `:14540` before handing off to Python — full detail in [deployment](deployment.md)):

```bash
source .venv/bin/activate

./run.sh qualifier_run.py --no-detector --budget 60   # flight only
./run.sh qualifier_run.py --weights best.pt --device cuda   # full mission
./run.sh qualifier_run.py --altitude 3.0 --display    # dev: show detections
```

There are **two mission entry points** in the repo — be deliberate about which one you're iterating on:

- `qualifier_run.py` — the documented asyncio supervisor: lawnmower coverage + detection loop + crash-restart. Output goes to `runs/<timestamp>/`:

  ```
  runs/20260518_140523/
  ├── barrels.csv         # crash-safe scoring log (rewritten on every sighting)
  └── detections/         # YOLO-annotated frames (one per detection event)
  ```

- `qualifier_main.py` — a teammate's alternative main using DFS exploration over a cell grid (2 m cells in the current copy). It writes `barrels.json` and `visited_cells.json` at the repo root.

See [codebase.md](codebase.md) for how the two relate and the [design rationale](design-rationale.md) for why the supervisor pattern exists (the 10-minute judge clock does not stop on a crash).

**Before any run with detection enabled, verify which weights are wired in.** Trained barrel weights now exist at the repo root (`best.pt`, 6.2 MB), but `model_config.json` still points at the COCO-pretrained `yolov10n.pt` placeholder. Pass `--weights best.pt` explicitly (or fix the config) and confirm via `model.names` that the class names match — a COCO model will never find a `yellow_barrel`.

Dev-loop gotchas (from hard experience):

- **Restart PX4 SITL between attempts.** A vehicle still armed from a previous run causes `Arm failed` on the next one.
- `--display` is great for watching detections during dev, but turn it **off** for scored runs (saves CPU, avoids GUI lockups).
- If the first RPC dies with `AioRpcError: Socket closed`, a stale `mavsdk_server` is holding `:14540` — `run.sh` handles this, or run the cleanup snippet in [deployment](deployment.md).
- If the health loop hangs forever with no traceback, PX4 isn't actually running (or is on a different port): `pgrep -fa px4` and `ss -ulpn | grep 14540`.

## Example & diagnostic scripts

The repo carries a layer of small single-purpose scripts — most are organizer starter code or early experiments (see [authorship](../AUTHORSHIP.md) for provenance). They are the fastest way to verify one subsystem in isolation before debugging a full mission.

### Flight basics

| Script | What it does |
|---|---|
| `takeoff_and_land.py` | Arm + hover + land. The minimal "is the sim alive" flight test. |
| `basic_offboard.py` | Minimum-correct offboard demo. Also used for the pose-drift sanity check (hover with GPS disabled, watch NED drift). |
| `go_to.py` | **GPS goto — unusable in the GNSS-denied qualifier.** Keep only as a MAVSDK reference. |
| `keyboardcontrol.py`, `KEY2.py` | Manual teleop over MAVSDK (pynput key handler → velocity-body setpoints). Useful for capturing training footage. ⚠ **Manual control during a scored run = disqualification — never launch these in a run script.** |

### Telemetry probes

| Script | What it does |
|---|---|
| `get_battery.py` | Streams battery state. |
| `get_position.py` | Streams NED position. Uses `udpin://0.0.0.0:14540` like the rest of the stack (the legacy `udp://:14540` scheme regression was restored 2026-06-06). |
| `get_flightmode.py` | Streams the active flight mode. |
| `is_arm_air.py` | Streams armed + in-air booleans. |
| `imu.py` / `imutest.py` | Raw IMU streams (sets `set_rate_imu`, prints readings). |
| `drone_diagnostics.py` | One-shot health check: battery, GPS info, EKF health flags. |

### Cameras & depth

| Script | What it does |
|---|---|
| `get_video.py` | Gazebo Transport RGB subscriber + OpenCV display. Use this to confirm the RGB topic string. |
| `get_video_old.py` | **Deprecated** — GStreamer UDP 5600 RTP H.264 path, not used by the current stack. |
| `save_photo.py` | RGB frames → `captured_images/`. |
| `photo.py` | MAVSDK camera mode / photo trigger. |
| `get_depth.py` | MAVSDK distance-sensor stream. |
| `depthtest.py` | Subscribes `/depth_camera` and prints depth stats. Use this to confirm the depth topic. |

### Perception & planning demos

| Script | What it does |
|---|---|
| `UseDetectorExample.py` | Wires `Detector.py` (threaded YOLO worker pool) to the Gazebo RGB topic — the standard YOLO + camera integration test. `Detector.py` accepts `config_path` again (regressed May 22, restored 2026-06-06) — but `model_config.json` still points at the COCO placeholder, so verify what it points at or pass `model_path` explicitly. |
| `RRTExample.py` | RRT* planner demo on an accumulated obstacle cloud. Known issue: its `__main__` block is missing `from GlobalMapper import GlobalMapper`. |

### Legacy main loops

`avoid.py` is the old reactive grid-heading navigation loop (20 Hz, N/E/S/W headings, `AvoidancePlanner` local avoidance). Older docs called it "current best" — **that is stale**. It has been superseded by the two qualifier mains (`qualifier_run.py`, `qualifier_main.py`) and is kept only as reference. The same goes for its byte-for-byte duplicate `avoid_with_detect.py` (detection was never actually wired in) and the velocity-setpoint variant `vel_avoidance.py`. See [codebase.md](codebase.md) for the full inventory.

## Sensor topics quick reference

The three values you will type most often while testing:

| Item | Value |
|---|---|
| Depth topic (gz-transport) | `/depth_camera` |
| RGB topic (gz-transport) | `/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image` |
| MAVSDK / PX4 address | `udpin://0.0.0.0:14540` (mavsdk_server gRPC on `localhost:50051`) |

Always cross-check topic strings against the actual world with `gz topic -l` — the RGB topic default was originally inferred, not confirmed, and the competition world may differ. The full table of hardcoded constants (camera intrinsics, image resolution, takeoff altitudes, loop rates, weight paths) lives in [codebase.md](codebase.md).
