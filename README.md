# Qualifier MVP — User Manual

User manual for the four new modules that wire the existing drone stack into an autonomous run for the **RoboVerse Qualifier 2026** challenge. For repo-wide context and the file-by-file inventory of the pre-existing code, see `CONTEXT.md`. For the design rationale and risks, see `APPROACH.md`.

```
qualifier_run.py        ── main entry point (asyncio supervisor + mission/detection loops)
coverage.py             ── lawnmower waypoint generator (pure)
detection_to_world.py   ── bbox + depth + pose → world NED point (pure)
barrel_log.py           ── thread-safe dedup + scoring + CSV persistence
```

Everything else in `Codes/` is **pre-existing** and is consumed by these four files as libraries — you should not need to touch it to run the MVP.

---

## 1. Install

### 1.0 Clone the repo

The codebase lives at https://github.com/curryfarmer/brainhack-2026-drone (private). Clone with either method:

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

`collect_yolo_data.py` now does an explicit gRPC sanity ping at the end of `connect()` and prints a `[FATAL] mavsdk_server died after health-handshake.` message instead of a deep traceback when it detects this failure mode.

---

## 8. Help wanted from human

Things the code can't decide for you:

1. **Pose source under GNSS-denied** — verify EKF2 has a vision/mocap fix; the whole stack assumes `telemetry.position_velocity_ned()` is bounded. See `APPROACH.md §9 R1`.
2. **YOLO training data + labelling** — capture from manual flight, label `yellow_barrel` / `red_barrel` in `labelImg` or Roboflow. Need ≥200 imgs per class.
3. **Verify RGB topic string** — the default in `MissionConfig.rgb_topic` is a guess from the camera-related files; confirm against your actual `gz topic -l`.
4. **Red-barrel altitude** — judges' map will tell you; pick single- vs two-altitude pass.
5. **Competition rig GPU** — verify YOLO runs at ≥10 FPS with chosen weights at `--device cuda`. If not, drop to `yolov10n.pt`-class architecture and 320 input resolution.
