<!-- Edited by Claude — see §10 for the list of files Claude has touched and why. -->

# Qualifier MVP — User Manual

User manual for the four new modules that wire the existing drone stack into an autonomous run for the **RoboVerse Qualifier 2026** challenge. For repo-wide context and the file-by-file inventory of the pre-existing code, see `CONTEXT.md`. For the design rationale and risks, see `APPROACH.md`.

> **Working with AI?** Read **§10 — AI-assisted edits** before changing anything. It lists every file Claude has touched (and why), the rules the AI is told to follow, and the inventory of files that should be **reused** rather than rewritten.

```
qualifier_run.py        ── main entry point (asyncio supervisor + mission/detection loops)
coverage.py             ── lawnmower waypoint generator (pure)
detection_to_world.py   ── bbox + depth + pose → world NED point (pure)
barrel_log.py           ── thread-safe dedup + scoring + CSV persistence
```

Everything else in `Codes/` is **pre-existing** and is consumed by these four files as libraries — you should not need to touch it to run the MVP.

---

## 1. Install

### 1.0 Get the code

Two ways. Pick whichever fits your workflow — both end up in a working folder you can `cd` into.

**Drop-in (recommended for the drone PC).** No git, no clone — just download a fresh ZIP every time you want the latest code. This is what the rig actually does. See §12 for the full loop.

```bash
wget https://github.com/curryfarmer/brainhack-2026-drone/archive/refs/heads/main.zip
unzip main.zip
cd brainhack-2026-drone-main
chmod +x run.sh
```

**Clone (for development on your laptop).**

```bash
# HTTPS (uses a personal-access token or git credential manager)
git clone https://github.com/curryfarmer/brainhack-2026-drone.git
cd brainhack-2026-drone

# OR SSH (recommended once your key is on github.com/settings/keys)
git clone git@github.com:curryfarmer/brainhack-2026-drone.git
cd brainhack-2026-drone
```

If you don't have collaborator access yet, ask the repo owner (`curryfarmer`) to add you under **Settings → Collaborators**.

> The pre-trained `yolov10n.pt` (5.6 MB) is checked in, so a fresh clone is runnable as-is for `--no-detector` smoke tests. Training data and trained barrel weights are **not** committed — see `CONTEXT.md` for the side-channel that distributes them.

### 1.1 Python environment

One-time setup (already done on this machine, repeat on the competition rig):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip wheel setuptools
pip install -r requirements.txt
```

### 1.2 Torch flavour

`requirements.txt` does not pin a torch wheel — install the right one for your platform.

```bash
# CPU dev (Mac, no GPU)
pip install torch torchvision

# CUDA training VM / competition rig
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

### 1.3 Gazebo Harmonic

The Gazebo Python bindings (`gz.transport13`, `gz.msgs10`) are **not** on PyPI. Install Gazebo Harmonic from system packages:

```bash
brew install gz-harmonic                          # macOS
sudo apt install gz-harmonic                      # Ubuntu 24.04 (after adding packages.osrfoundation.org)
```

Without Gazebo Harmonic you can still import `qualifier_run` for unit testing — sensor/depth paths will refuse to start.

### 1.4 PX4 SITL

The drone scripts all expect PX4 SITL on `udpin://0.0.0.0:14540`. On the competition rig PX4 is launched separately; for local dev:

```bash
# clone + build PX4 (one-time)
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_sitl gz_x500_depth        # x500 quad with depth camera + Gazebo Harmonic
```

This boots PX4 on `:14540` and spawns the x500 model in Gazebo. Leave it running in its own terminal — every script in this repo will discover it.

---

## 2. Quick start

PX4 SITL must be running and publishing telemetry on `udpin://0.0.0.0:14540`. Gazebo Harmonic must publish the depth + RGB topics named in the config.

```bash
source .venv/bin/activate

# Smoke-test the pure-logic modules first (no PX4 / Gazebo needed)
python coverage.py             # prints generated waypoints
python detection_to_world.py   # prints a synthetic back-projection
python barrel_log.py           # prints dedup test results

# Flight-only run, 60-second budget, no YOLO
python qualifier_run.py --no-detector --budget 60

# Full mission with custom barrel weights
python qualifier_run.py --weights barrel_best.pt --device cuda

# Override altitude + display detections during dev
python qualifier_run.py --altitude 3.0 --display
```

Output goes to `runs/<timestamp>/`:
```
runs/20260518_140523/
├── barrels.csv         # crash-safe scoring log (rewritten on every sighting)
└── detections/         # YOLO-annotated frames (one per detection event)
```

---

## 3. Configuration

Every tunable lives in `MissionConfig` (top of `qualifier_run.py`). Three ways to set values:

### CLI flags (overrides config)

| Flag | Effect |
|---|---|
| `--weights PATH` | YOLO weights file (defaults to `yolov10n.pt`, **untrained for barrels**) |
| `--device {cpu,cuda,mps}` | Torch device for inference |
| `--altitude M` | Cruise altitude in metres |
| `--budget S` | Wall-clock budget (defaults to 600 s) |
| `--no-detector` | Skip YOLO entirely (flight test) |
| `--display` | Show OpenCV window with detections |
| `--config PATH` | Load a JSON file with any of the fields below |

### JSON config (`--config run_config.json`)

```json
{
  "origin_north": 0.0,
  "origin_east": 0.0,
  "arena_north_m": 40.0,
  "arena_east_m": 40.0,
  "cruise_altitude_m": 2.5,
  "lane_spacing_m": 3.5,
  "along_axis": "north",
  "cruise_speed_mps": 1.2,
  "yolo_weights": "barrel_best.pt",
  "yolo_device": "cuda",
  "yolo_conf": 0.45,
  "depth_topic": "/depth_camera",
  "rgb_topic": "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image",
  "detection_class_map": {
    "yellow_barrel": "yellow_barrel",
    "red_barrel": "red_barrel"
  }
}
```

`K` (camera intrinsics) can also be in the JSON as a 3×3 nested list — it's converted to numpy on load.

### Edit `MissionConfig` defaults in source

Permanent defaults — change `qualifier_run.py:MissionConfig` directly.

---

## 4. Module reference

### `coverage.py`

```python
from coverage import generate_lawnmower, filter_unvisited, Waypoint

wps = generate_lawnmower(
    origin_north=0.0, origin_east=0.0,
    width_north=40.0, width_east=40.0,
    altitude=2.5,
    lane_spacing=3.5,
    along_axis="north",     # or "east"
)
# -> list[Waypoint]; each Waypoint has .north, .east, .down, .yaw_deg, .is_turn

# After a crash, drop already-visited waypoints
remaining = filter_unvisited(wps, current_north, current_east, visited_radius=1.5)
```

Pure. No I/O, no dependencies on drone / asyncio. Trivially unit-testable.

### `detection_to_world.py`

```python
from detection_to_world import Pose, detection_to_world, project_pixel_to_world

pose = Pose(north=5.0, east=3.0, down=-2.5, yaw_rad=0.0)
det = detection_to_world(
    bbox_xyxy=(x1, y1, x2, y2),   # YOLO bbox
    depth_frame=depth_hxw,        # float32 depth map (metres)
    pose=pose,
    K=K_3x3,
    class_name="yellow_barrel",
    confidence=0.87,
    ts=time.time(),
    camera_pitch_deg=0.0,         # +ve = tilted down
)
# det is WorldDetection(.class_name, .confidence, .north, .east, .down, .range_m, .ts) or None
```

`None` is returned when the bbox centre has no valid depth (invalid pixel, NaN, out of range). `median_patch_depth()` smooths over a 5×5 window by default.

Coordinate frames:
- **Camera**: +Z forward, +X right, +Y down (OpenCV).
- **Body**: +X forward, +Y right, +Z down.
- **NED**: +N north, +E east, +D down.

Yaw is taken in **radians**, NED convention (0 = facing north, positive clockwise viewed from above).

### `barrel_log.py`

```python
from barrel_log import BarrelLog

log = BarrelLog("runs/20260518/barrels.csv", dedup_radius=2.0)

entry, is_new = log.add("yellow_barrel", north=5.0, east=3.0, down=-1.0, confidence=0.9)
# is_new=True only on first sighting of a barrel within dedup_radius.

log.score()           # int — 50/100 per class per first-seen
log.count_by_class()  # dict
log.snapshot()        # list[BarrelEntry]
```

- Lock-protected for use from Detector worker threads.
- Atomic file write (`.tmp` + `os.replace`) so a crash during flush can't corrupt the CSV.
- `autoload=True` reads an existing CSV on construct — used by the supervisor on restart so the second attempt continues the same log.
- Repeat sightings refine position via running mean and bump `last_seen` + `sightings` count.
- Scoring table is `BarrelLog.SCORES = {"yellow_barrel": 50, "red_barrel": 100}`. Update there if the rule pamphlet changes.

### `qualifier_run.py`

Wires everything together. The interesting parts:

**`MissionState`** — shared container passed to both loops. Holds `pose_state` (telemetry), `barrel_log`, `depth_rx`, `detector`, current `cfg`, `stop_event`.

**`mission_loop(drone, state)`**
1. Connect to PX4, start `position_monitor_task`.
2. `arm_and_takeoff()` (calls `offboard.start()` internally, see `drone_control.py`).
3. Generate lawnmower waypoints from the current `cfg`.
4. Filter out already-visited waypoints (for supervisor restart).
5. For each waypoint, `_go_to_waypoint()`:
   - Check `AvoidancePlanner.compute_position_ned` on latest depth.
   - If `info["blocked"]`, head toward the planner's `target_ned` (sidestep). Otherwise drive toward the waypoint.
   - Yaw follows travel direction when deviating; otherwise it follows the lane heading.
   - Stop when within `waypoint_radius_m` (default 0.8 m) or wall-clock deadline.
6. `drone.land()`. On any exception, attempt an emergency `land()` before re-raising.

**`detection_loop(state, rgb_rx)`**
- 5 Hz RGB pull → `Detector.submit_image(frame, ctx={pose, depth, ts})`.
- Detector worker thread runs YOLO, calls back into `make_detection_callback`.
- Callback maps YOLO class name → canonical barrel class via `cfg.detection_class_map`, projects bbox → world NED, registers in `BarrelLog`.

**`supervisor(cfg)`**
- 10-minute wall clock; on crash, re-enter the loop with remaining budget.
- Long-lived objects (detector, depth/rgb receivers, barrel log) are built once and reused across attempts — model load is too expensive to redo.
- `state.cfg` per-attempt is built via `dataclasses.replace(cfg, wall_clock_budget_s=remaining)` so the numpy K matrix is preserved (a dict round-trip would lose it).
- `state.stop_event.clear()` rather than reassigning, so any closure that captured the original Event reference still works.

---

## 5. Pre-run checklist

Run through this before any scored attempt.

- [ ] **YOLO weights** trained on yellow + red barrels (use `Train_YOLO_Models_new.ipynb`).
- [ ] `cfg.detection_class_map` updated to match the *exact* class names emitted by the model (check via `m.names` after loading).
- [ ] `cfg.depth_topic` and `cfg.rgb_topic` match the topics being published by the Gazebo world released for the run.
- [ ] `cfg.K`, `cfg.img_width`, `cfg.img_height` match the actual camera in the world (current defaults: 640×480, fx=fy=433).
- [ ] `cfg.origin_north`, `cfg.origin_east`, `cfg.arena_north_m`, `cfg.arena_east_m` set for the released arena layout.
- [ ] `cfg.cruise_altitude_m` chosen so the camera FOV covers both ground and elevated barrels (see `APPROACH.md §9 R2`).
- [ ] **Pose-drift sanity check** — run `python basic_offboard.py` with PX4 GPS disabled (`param set GPS_1_CONFIG 0`); confirm NED pose doesn't drift more than ~1 m over 60 s of hover. (`APPROACH.md §9 R1`.)
- [ ] Wall-clock budget left at 600 s (10 min judge clock).
- [ ] `--display` **off** for the scored run (saves CPU and avoids GUI lockups).
- [ ] `keyboardcontrol.py` not launched — manual control = DQ.

---

## 6. Extending

### Add a new barrel class
1. Train YOLO with the new class.
2. Add `"new_class": "new_class"` to `cfg.detection_class_map`.
3. Add scoring in `BarrelLog.SCORES`.

### Two-altitude pass (e.g. one low for yellow, one high for red)
Currently `mission_loop` runs one lawnmower at `cfg.cruise_altitude_m`. To add a second altitude, build two waypoint lists and concatenate:
```python
low = generate_lawnmower(..., altitude=1.5, lane_spacing=5.0)
high = generate_lawnmower(..., altitude=4.0, lane_spacing=5.0)
waypoints = low + high
```

### RRT* fallback when reactive deviation can't clear
Stubbed in `APPROACH.md §4`. Build into `_go_to_waypoint`:
- Track time-since-deviation.
- If > 3 s of `blocked=True`, push frame into `GlobalMapper.update_frame()`.
- Call `RRTStarPlanner.plan(current_pose, target, mapper.get_global_points())`.
- Follow the first 2–3 returned waypoints, then resume coverage.

### Replay / bench mode
Recommended addition: record `(timestamp, pose, depth_frame, rgb_frame)` tuples to disk during a real flight (or in sim), then replay them through `detection_loop`'s callback path without flying — useful for iterating on YOLO + projection math without burning sim time. Not yet built.

---

## 7. Failure modes & debugging

| Symptom | Likely cause | Check |
|---|---|---|
| `ModuleNotFoundError: gz` | Gazebo Harmonic not installed | `brew install gz-harmonic` |
| `No telemetry pose received within 10 s` | PX4 not running, or wrong UDP port | `cfg.px4_address`; `netstat -an \| grep 14540` |
| Drone drifts away from waypoints | EKF2 dead-reckoning without vision/mocap | See `APPROACH.md §9 R1`; configure an external odometry source |
| YOLO finds nothing | Wrong weights, wrong class map, low confidence threshold | Run with `--display`; check `cfg.detection_class_map` matches `model.names` |
| `Arm failed` on second attempt | Vehicle still armed from previous attempt | Restart PX4 SITL between attempts during dev; in flight, the emergency-land in `mission_loop`'s exception handler should disarm |
| Barrel double-counted | Pose drift > `dedup_radius` between sightings | Increase `BarrelLog(dedup_radius=...)`; default 2.0 m |
| Detector callback runs but no entries in CSV | `context["depth"]` was None when the YOLO frame ran — depth stream lagging | Verify depth receiver is publishing; check `cfg.depth_topic` |

### 7.1 MAVSDK gRPC failures

The `mavsdk_server` subprocess (spawned by `System()`) is the most common silent failure point. Three diagnostics + one cleanup snippet cover ~all cases:

| gRPC error | Where it fires | Cause | Fix |
|---|---|---|---|
| `AioRpcError: Socket closed` (UNAVAILABLE) | first `drone.action.arm()` | Stale `mavsdk_server` holding UDP `:14540` from a previous crashed run. New server can't bind, exits, next RPC dies. | Run the cleanup below. |
| `AioRpcError: recvmsg:Connection reset by peer` (UNAVAILABLE) | first `drone.offboard.set_velocity_body` | `MAVSDK_ADDRESS` uses legacy `udp://:14540`. MAVSDK 2.x reads that as `udpout` and the server segfaults on the first outbound write. | Use `udpin://0.0.0.0:14540`. (Already fixed in this repo; regression-catcher.) |
| Health loop hangs forever (no traceback) | inside `connect()` | PX4 SITL not running, or running on a different port. | `pgrep -fa px4` and `ss -ulpn \| grep 14540` — confirm PX4 owns the port. |

**Cleanup snippet (run on the drone PC before relaunching the script):**

```bash
pkill -9 -f mavsdk_server
pkill -9 -f collect_yolo_data
sleep 1
ss -ulpn | grep 14540   # should now be empty or owned only by PX4
```

`collect_yolo_data.py` now delegates the full flight lifecycle (connect, arm, takeoff, offboard prime, land) to `drone_control.Drone` — the wrapper `qualifier_run.py` uses in production. That wrapper has a proven `arm_and_takeoff()` sequence (arm → takeoff → sleep 20 → NED-prime → offboard.start) and there is no longer any hand-rolled MAVSDK code in `collect_yolo_data.py`.

---

## 8. Help wanted from human

Things the code can't decide for you:

1. **Pose source under GNSS-denied** — verify EKF2 has a vision/mocap fix; the whole stack assumes `telemetry.position_velocity_ned()` is bounded. See `APPROACH.md §9 R1`.
2. **YOLO training data + labelling** — capture from manual flight, label `yellow_barrel` / `red_barrel` in `labelImg` or Roboflow. Need ≥200 imgs per class.
3. **Verify RGB topic string** — the default in `MissionConfig.rgb_topic` is a guess from the camera-related files; confirm against your actual `gz topic -l`.
4. **Red-barrel altitude** — judges' map will tell you; pick single- vs two-altitude pass.
5. **Competition rig GPU** — verify YOLO runs at ≥10 FPS with chosen weights at `--device cuda`. If not, drop to `yolov10n.pt`-class architecture and 320 input resolution.

---

## 9. Image capture (Gazebo + screen fallback)

`collect_yolo_data.py` records training data while you fly. The capture button keys come from `keyboardcontrol.py`'s key handler:

| Key | Effect |
|---|---|
| `C` | Single shot now |
| `P` | Burst — queue 10 frames |
| `O` | Toggle continuous auto-save |
| `[` `]` | Slow / speed the auto-save tick (0.25 / 0.5 / 1.0 / 2.0 s) |
| `N` | Cycle filename prefix (`yellow` → `red` → `mixed`) |

The frame source is automatic:

1. **Primary — Gazebo camera.** `gz.transport13.Node().subscribe()` listens on `CAMERA_TOPIC` (default `/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image`). Every frame fills `_latest_frame_bgr`.
2. **Fallback — screen grab via [`mss`](https://pypi.org/project/mss/).** If the Gazebo subscribe call returns `False`, or `_latest_frame_bgr` is still `None` when a capture is requested, `_grab_screen()` snapshots the primary monitor and returns it as BGR. Put the Gazebo viewport on the primary monitor and the recorded JPEGs will be the cropped Gazebo view; if Gazebo is offline, you get the desktop instead — same path through the rest of the pipeline.

`mss` is now listed in `requirements.txt`. If it can't be imported the fallback prints a one-line warning and silently no-ops — the script still flies fine, you just can't capture.

Output paths are unchanged: `data/train/images/<prefix>_<ts>.jpg` for the image, `session_meta/<prefix>_<ts>.json` for the pose sidecar.

---

## 10. AI-assisted edits

This file and a handful of others have been touched by Claude (Anthropic's AI assistant) during debug sessions. Every Claude-edited file has a top-of-file marker:

- Python: `# Edited by Claude — <one-line reason>. See README §10.`
- Markdown: `<!-- Edited by Claude — ... -->`

### 10.1 Files Claude has edited

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

### 10.2 Files Claude has NOT touched (reuse these)

When asking for new behaviour, **point the AI at these first** so it composes them instead of writing new code.

**Flight wrappers (use these — do not re-implement):**
- `drone_control.py` — `Drone` class with `connect`, `arm_and_takeoff`, `land`, `send_velocity` (NED), `rotate_to_yaw`. Proven, used by `qualifier_run.py`. *(Has one Claude-added helper — `_kill_stale_servers()` — but the public surface is unchanged.)*
- `drone_control_new.py` — alternative wrapper with `set_takeoff_altitude` and a more-forgiving health gate (`wait_until_ready`). Uses `goto_location` instead of offboard.
- `basic_offboard.py` — minimum-correct offboard example.

**Mission pieces:**
- `qualifier_run.py` — production asyncio supervisor: mission loop + detection loop + watchdogs.
- `coverage.py` — pure lawnmower waypoint generator (see §11).
- `detection_to_world.py` — bbox + depth + pose → world-NED point.
- `barrel_log.py` — thread-safe dedup + scoring + CSV.

**Perception + planners (untouched, kept as-is):**
- `Detector.py`, `UseDetectorExample.py` — YOLO inference wrapper.
- `AvoidancePlanner.py`, `RRTStarPlanner.py`, `RRTExample.py`, `VelocityPlanner.py`, `PointCloudPlanner.py` — reactive + global path planning.
- `GlobalMapper.py`, `GlobalMapper_new.py` — occupancy mapping.
- `depthcloud.py`, `depth_receiver.py`, `depthtest.py`, `get_depth.py` — depth sensor handling.

**Helpers/utilities (untouched):**
- `get_battery.py`, `get_flightmode.py`, `get_position_with_task.py`, `get_video.py`, `is_arm_air.py`, `imu.py`, `drone_diagnostics.py`, `photo.py`, `save_photo.py`, `top_down.py`, `go_to.py` — small MAVSDK probes.
- `gzphotodetectorsaver.py` — Gazebo photo saver.
- `barrel_log.py`, `pipeline.py` — training pipeline glue.

**Training scripts (untouched):**
- `train_yolo.py`, `eval_model.py`, `deploy_model.py`, `gen_data_yaml.py`, `gen_smoke_data.py`, `split_train_val.py`, `validate_labels.py`, `import_roboflow.py`, `Train_YOLO_Models.ipynb`, `Train_YOLO_Models_new.ipynb`.

If you (or the AI) is about to write code that does anything any of the above already does, **stop and use the existing file instead.**

### 10.3 Rules the AI follows

1. **Reuse first.** Use `drone_control.Drone`, `basic_offboard.py`, `coverage.py`, etc. before writing new code.
2. **Document in README.** Anything new lands in this file, in a section like the one you're reading. Keep it human-readable — no terse compression here.
3. **Mark edits.** Every Claude-edited file gets the `Edited by Claude` marker; every new file is added to §10.1.
4. **Smallest change.** Minimum lines to fix the problem. No feature creep, no speculative abstractions.
5. **Surface failures.** No silent `except Exception: pass`. Print a one-line diagnostic and either continue or re-raise.

---

## 11. Waypoint generation (today + roadmap)

Today the project uses one strategy:

### 11.1 Pre-baked lawnmower (current behaviour)

`coverage.generate_lawnmower(origin_north, origin_east, width_north, width_east, altitude, lane_spacing=3.5, along_axis="north")` returns a list of `Waypoint(north, east, down, yaw_deg, is_turn)` covering a rectangular region in a boustrophedon (snake) sweep. The number of lanes is `ceil(|width_east| / lane_spacing) + 1`, and every other lane reverses direction so the drone walks back and forth without dead heads.

Generation happens **once** at the start of `qualifier_run.mission_loop()` (around line 355). The list is then handed to `_go_to_waypoint(...)` one element at a time. The drone is considered "at" a waypoint when its NED distance is below `cfg.waypoint_radius_m` (default 0.8 m). On supervisor restart, `coverage.filter_unvisited(...)` drops the leading waypoints we already reached so we don't re-fly completed lanes.

In other words, **the waypoint list itself is static**. What is "dynamic" is the *path between* waypoints: while flying toward a target, `qualifier_run` keeps the AvoidancePlanner (`AvoidancePlanner.py`) in the loop. If the depth stream sees an obstacle, the planner deviates the velocity command to clear it, then re-targets the original waypoint once free. The original list never gets regenerated.

### 11.2 What "dynamic" generation would look like

If you want the *list* to change at runtime, two natural extensions:

- **Frontier-based exploration.** After every detection or every N seconds, look at the occupancy grid (`GlobalMapper.py`) and re-issue waypoints at the boundary of unexplored cells. Replace `generate_lawnmower(...)` at line 355 with a call into a `frontier_next(map)` helper that returns the next K waypoints. The mission loop already consumes the list one element at a time so swapping the source is a single-line change.
- **Re-targeting around detections.** When a barrel is logged in `barrel_log`, generate a tighter sweep around it (e.g. four waypoints at 1.5 m radius). Insert those at the head of the queue. Easiest is a `collections.deque` instead of a `list` so you can `appendleft(...)` without reshuffling.

Neither is implemented today — flag the request and we'll add it as its own PR with the same reuse-first ethos.

---

## 12. Drop-in workflow (no git)

The drone PC runs the code by downloading the GitHub ZIP, unzipping it, and launching from inside the extracted folder. There is **no `git pull`** on the rig — every code change cycle is a fresh download. This section is the single source of truth for that flow.

### 12.1 One-time setup (per drone PC)

These only need to be done once. They install Python deps and the Gazebo Python bindings into the rig's system Python.

```bash
# Python deps (covers mavsdk, ultralytics, opencv, mss, etc.)
pip install -r requirements.txt

# Gazebo Harmonic — provides gz.transport13 + gz.msgs10
sudo apt install gz-harmonic
```

If you need torch on a specific accelerator, see §1.2.

### 12.2 Every-run loop

```bash
# 1. Grab the latest code as a ZIP from GitHub. Refresh this each iteration.
wget https://github.com/curryfarmer/brainhack-2026-drone/archive/refs/heads/main.zip -O latest.zip
unzip -o latest.zip
cd brainhack-2026-drone-main

# 2. Make the launcher executable (first time only — the ZIP loses the bit).
chmod +x run.sh

# 3. Launch any script through run.sh. The launcher kills zombie mavsdk_server
#    processes and reports who else owns UDP :14540 before handing off to Python.
./run.sh collect_yolo_data.py
```

For other scripts:

```bash
./run.sh basic_offboard.py
./run.sh keyboardcontrol.py
./run.sh qualifier_run.py --no-detector --budget 60
```

### 12.3 What `run.sh` does (and why)

The launcher is a five-step shell wrapper:

1. `pkill -9 -f mavsdk_server` — kill any subprocess from a previous crashed run that's still holding UDP `:14540`. Silent if nothing matches.
2. `sleep 0.5` — give the kernel a moment to release the bound port.
3. `ss -ulpn | grep :14540` — check who, if anyone, still owns the port.
4. If the port is owned by something we didn't just kill (e.g. PX4 itself), print a warning line naming the process plus the manual fix recipes.
5. `exec python3 <your script> <args...>` — replace the shell with Python so Ctrl-C still cleans up the way you expect.

`drone_control.Drone.connect()` ALSO runs the same `pkill` internally (see `_kill_stale_servers()`), so even if you forget the launcher, scripts that use the wrapper (`collect_yolo_data.py`, `qualifier_run.py`) are still self-healing.

### 12.4 If `run.sh` still reports a bind error

Two cases:

- **PX4 SITL is configured to *bind* :14540 itself.** Some PX4 launch scripts run `mavlink start -u 14540 ...` where `-u` means "bind UDP". When that happens, MAVSDK cannot also `udpin://` the same port — it has to `udpout://` (send to) instead. One-line fix: edit `drone_control.py:27` and change
  ```python
  await self.drone.connect(system_address="udpin://0.0.0.0:14540")
  ```
  to
  ```python
  await self.drone.connect(system_address="udpout://127.0.0.1:14540")
  ```
  Then re-zip / re-download and run again. The launcher's warning line will tell you whether PX4 is the binder — look at the `users:(("px4",...))` portion of the `ss` output.

- **Some unrelated process is holding :14540** (a stray QGroundControl, a leftover SITL from a previous reboot, etc.). Find the PID and kill it:
  ```bash
  sudo lsof -iUDP:14540
  sudo kill -9 <PID>
  ```

Once `ss -ulpn | grep :14540` reports an empty line, MAVSDK can bind cleanly.

### 12.5 If the screen-capture fallback ever runs

If you see `[CAM] Screen-capture fallback enabled (mss).` in the console, the Gazebo camera topic was unavailable. Put the Gazebo window on the primary monitor so the recorded frames contain the simulated scene. If `mss` itself fails to import (`[CAM] screen fallback unavailable`), `pip install mss` and try again.

---

## 13. Labelling pipeline (turn captured frames into YOLO training data)

After `collect_yolo_data.py` you have a folder of un-labelled images at `data/train/images/<prefix>_<ts>.jpg` plus pose sidecars at `session_meta/<prefix>_<ts>.json`. To train YOLO you also need a `.txt` next to each image with the bounding boxes — that's what the labelling step produces.

### 13.1 What format the rest of the pipeline expects

Every captured image must have a matching label file with the same stem:

```
data/train/images/yellow_1779194401320_2.jpg
data/train/labels/yellow_1779194401320_2.txt   <-- you create this
```

Each line of the `.txt` is one bounding box, **normalized to image dimensions**:

```
<class_id> <cx> <cy> <w> <h>
```

- `class_id` is the 0-based index into `classes.txt`. The default project file is:
  ```
  yellow_barrel
  red_barrel
  ```
  so class **0 = yellow_barrel**, class **1 = red_barrel**.
- `cx, cy, w, h` are all floats in `[0, 1]`. `cx, cy` is the **centre** of the box, `w, h` are the **full width and height**, all divided by image width / image height.

An image with no barrels visible should produce an **empty** `.txt` file (still required — it's the "background" class signal). `validate_labels.py` accepts empty files.

### 13.2 Pick a labelling tool

All three options output the exact format above — pick whichever matches your workflow.

| Tool | Install | UI | AI-assist | Best for |
|---|---|---|---|---|
| **Roboflow** (web) | sign up at roboflow.com — nothing to install locally | browser | yes (auto-label after a few examples) | most users; output goes straight into `import_roboflow.py` |
| **labelImg** | `pip install labelImg` | Qt desktop app | none | working offline, small batches |
| **X-AnyLabeling** | `pip install x-anylabeling` | Qt desktop app | SAM one-click box | thousands of images, want speed |

#### 13.2a Roboflow (recommended)

1. Make a free account at https://roboflow.com.
2. Create a new **Object Detection** project. Class names exactly `yellow_barrel` and `red_barrel` (matches `classes.txt` — order matters because Roboflow assigns class IDs by the order you create them).
3. Upload the folder `data/train/images/`. Roboflow batches them automatically.
4. Label each frame:
   - Press `B` to draw a bounding box. Click-drag a tight rectangle around each barrel.
   - Press the number key for the class (`1` = yellow_barrel, `2` = red_barrel by default) OR pick from the dropdown.
   - Press `→` for next image.
5. Once enough images are labelled (≥20 per class for a sanity train, ≥200 per class for a real model), Roboflow's **Smart Polygon / Auto-Label** can pre-label the rest. Always verify the auto-labels before exporting.
6. **Export → YOLOv8** → "Show download code" → choose `.zip`. Save the file as `data/roboflow_export.zip`.
7. Merge it into the project layout:
   ```bash
   ./run.sh import_roboflow.py --zip data/roboflow_export.zip --target-split train
   ```
   This dumps labels into `data/train/labels/`, validates the class set against `classes.txt`, and refuses to overwrite existing files (so you can re-run safely).

#### 13.2b labelImg (offline)

```bash
pip install labelImg
mkdir -p data/train/labels
labelImg data/train/images data/classes.txt
```

The last argument is the predefined class list. Key bindings worth knowing:

| Key | Action |
|---|---|
| `W` | Create a new rectangle |
| `D` | Next image |
| `A` | Previous image |
| `Ctrl+S` | Save |
| `Ctrl+Shift+S` | Set save dir |

**Before drawing:** click `View → Auto Save mode` and switch the format dropdown (top-left) to **YOLO**, not PascalVOC. Then set the save directory to `data/train/labels`. Each save produces a `.txt` with the correct YOLO format.

#### 13.2c X-AnyLabeling (SAM-assisted, fastest for big batches)

```bash
pip install x-anylabeling
x-anylabeling
```

- `Open Dir` → `data/train/images`
- `Change Output Dir` → `data/train/labels`
- File → Auto Labeling → load `SAM` model (downloads on first use, ~400 MB)
- Click on a barrel → SAM proposes a box → confirm with `Enter`, assign class.

Saves in YOLO format if you switch the format toggle in the bottom-right corner.

### 13.3 After labelling — validate → split → train

These three scripts are pre-existing and you should run them in order. All accept `./run.sh` since they don't touch MAVSDK, but plain `python3` works too:

```bash
# 1. Validate. Refuses to proceed if any image is broken, mis-sized, or
#    has fewer than 5 images per class. Writes data/.validated.flag.
python3 validate_labels.py --data-root data --classes-file classes.txt

# 2. 80/20 train/val split. Pose-aware: whole spatial bins go to one side
#    so near-duplicate frames don't leak across the split.
python3 split_train_val.py --data-root data --meta-dir session_meta

# 3. Generate Ultralytics data.yaml for the trainer.
python3 gen_data_yaml.py --data-root data --classes-file classes.txt

# 4. Train.
python3 train_yolo.py --data data/data.yaml --epochs 50 --device cuda
```

`train_yolo.py` writes weights to `models/yolo_<timestamp>/weights/best.pt` and the latest path to `models/latest_path.txt`. Plug that into `qualifier_run.py --weights <path>` to fly the mission with your new model.

### 13.4 Common labelling mistakes

- **Wrong class order.** If you label `red_barrel` as class 0 in Roboflow but `classes.txt` says `yellow_barrel` is class 0, `import_roboflow.py` will reject the merge with `class set mismatch`. Fix the export-side class order; do not edit `classes.txt`.
- **Tight vs loose boxes.** Boxes should hug the barrel — extra margin trains the model to detect "rectangle of stuff around a barrel". `validate_labels.py` enforces a 2×2 pixel minimum but won't catch generous-margin labels.
- **Skipping background frames.** Empty-label `.txt` files (no barrels visible) are valuable training signal. Don't delete them.
- **Class imbalance.** Aim for roughly equal counts per class. `validate_labels.py` HARD-fails below 5 images per class and WARNs below 20.
