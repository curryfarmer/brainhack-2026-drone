# Codes/ — Context Map

Drone autonomy stack for the **RoboVerse Qualifier 2026** (Brainhack). PX4 SITL + Gazebo Harmonic + MAVSDK + YOLO. Goal: find yellow + red fuel barrels in a 40 m × 40 m × 8 m GNSS-denied space port within 10 minutes. See `APPROACH.md` for the planned solution architecture.

---

## 1. Quick start

```bash
cd Codes
source .venv/bin/activate

# PX4 SITL must already be running:
#   cd ~/PX4-Autopilot && make px4_sitl gz_x500_vision
# (or whatever world publishes /depth_camera and the IMX214 camera topic)

python avoid.py                  # main reactive nav loop (current best)
python UseDetectorExample.py     # YOLO + Gazebo RGB camera test
```

Connect target: `udpin://0.0.0.0:14540` (PX4 SITL default).

---

## 2. Environment

| Thing | Value |
|---|---|
| Python | 3.12 (venv at `Codes/.venv`) |
| Torch | 2.2.2 CPU (Mac). **CUDA build needed on Linux training VM** — install via `pip install torch --index-url https://download.pytorch.org/whl/cu121` |
| NumPy | pinned `<2` (torch 2.2 wheels were built against NumPy 1.x) |
| Gazebo Python | `gz.transport13`, `gz.msgs10` — **not on PyPI**. Install via system pkg: `brew install gz-harmonic` (macOS) / apt `gz-harmonic` from packages.osrfoundation.org (Ubuntu 24.04). |
| PX4 SITL | UDP `:14540`, NED frame, vehicle `x500_vision_0` |
| Custom YOLO weights | not yet trained for barrels. `yolov10n.pt` in repo is COCO-pretrained — **must retrain** before qualifier (see `Train_YOLO_Models_new.ipynb`). |

---

## 3. Data pipeline

```
                            Gazebo Harmonic (world: roboverse)
   ┌────────────────────────────┼────────────────────────────────┐
   │                            │                                │
   ▼                            ▼                                ▼
 RGB camera                  Depth camera                    PX4 SITL
 /world/roboverse/.../        /depth_camera                  udpin://...:14540
  IMX214/image                                                    │
   │                            │                                 │
   ▼                            ▼                                 ▼
 Detector.py            depth_receiver.py            drone_control.Drone  +
 (YOLO worker pool)    (gz.transport13 → np)        get_position_with_task
   │                            │                       .SharedState
   │ bbox+class                 │ depth HxW                       │
   │                            │                                 │ pose (NED, yaw)
   │                            ▼                                 │
   │                ┌─────────┬─────────────────────┐             │
   │                ▼         ▼                     ▼             │
   │    AvoidancePlanner  GlobalMapper        VelocityPlanner     │
   │    /VelocityPlanner  (accum 2D cloud)   (vel-only variant)   │
   │      depth → NED       depth+pose →                          │
   │      setpoint or vel   obstacle cloud                        │
   │                            │                                 │
   │                            ▼                                 │
   │                    RRTStarPlanner / PointCloudPlanner        │
   │                    (global path on accumulated cloud)        │
   │                            │                                 │
   └────────────┐               │                                 │
                ▼               ▼                                 ▼
              avoid.py (main loop) ────────► drone_control.Drone.send_position_ned()
              (20 Hz; rotates grid heading when blocked)
```

---

## 4. File inventory

Legend: **LIB** = library class importable elsewhere · **MAIN** = autonomous entry point · **EX** = example/demo · **TEST** = diagnostic · **DUP** = near-duplicate of another file · **BUG** = known issue.

### Planning / avoidance

| File | Role | Key API | Notes |
|---|---|---|---|
| `AvoidancePlanner.py` | LIB | `AvoidancePlanner.compute_position_ned(depth, pose, K)` | Histogram + clearance, returns NED setpoint + `blocked`. Tunables: `safe_distance=2.5`, `critical_distance=0.8`, `max_speed=1.0`, `num_bins=36`. |
| `VelocityPlanner.py` | LIB · DUP | `VelocityPlanner.compute_velocity(...)` | Near-identical to AvoidancePlanner but emits velocity only. Used by `vel_avoidance.py`. |
| `RRTStarPlanner.py` | LIB | `RRTStarPlanner.plan(start, goal, cloud)` | RRT* w/ KDTree collision, path smoothing. `safety_margin=0.6`, `step_size=1.0`, `max_iter=3000`. |
| `PointCloudPlanner.py` / `_new.py` | LIB | `is_collision_free(p1, p2)`, `get_nearest_obstacle(p)` | KDTree wrapper. `_new` adds bounds + float casts. |

### Mapping

| File | Role | Notes |
|---|---|---|
| `GlobalMapper.py` / `_new.py` | LIB + EX | Accumulates depth → 2D NED obstacle cloud. `_new` adds bounds checks, `latest_pose_from_state()`, `collect_and_map_frame()`. Tunables: `obs_h_min=0.1`, `obs_h_max=1.5`, `z_min=0.3`, `z_max=5.0`. |
| `top_down.py` | LIB helper | Used by GlobalMapper for top-down transform. |
| `depthcloud.py` | EX (experimental) | Full frontier-explorer prototype (log-odds occupancy, DFS, viz thread). |

### Sensors

| File | Role | Topic / source |
|---|---|---|
| `depth_receiver.py` | LIB | `gz.transport13` → `/depth_camera`. Thread-safe latest-frame. |
| `depthtest.py` | TEST | prints depth stats |
| `get_depth.py` | TEST | MAVSDK distance-sensor stream |
| `get_video.py` | EX | Gazebo Transport RGB + OpenCV display |
| `get_video_old.py` | EX (deprecated) | GStreamer UDP 5600 RTP H.264 |
| `save_photo.py` | TEST | RGB → `captured_images/` |
| `photo.py` | EX | MAVSDK camera mode/photo trigger |

### Detection

| File | Role | Notes |
|---|---|---|
| `Detector.py` | LIB | Threaded YOLO worker pool. `submit_image(img, ctx)` → async detections; non-blocking display queue. Configurable `model_path`, `confidence_threshold`, `num_workers`, `device`. |
| `UseDetectorExample.py` | EX | Wires Detector to Gazebo RGB topic. RGB→BGR conversion, per-frame metadata. |
| `gzphotodetectorsaver.py` | BUG | Burst capture w/ YOLO. `_process_task` references undefined `msg` → crashes. |
| `yolov10n.pt` | weight | COCO classes — **not barrel-trained**. |

### Drone control

| File | Role | Notes |
|---|---|---|
| `drone_control.py` / `_new.py` | LIB | `Drone` class: connect, arm/takeoff, send vel/pos NED, yaw control. `_new` adds geo-fencing, `wait_until_ready()`. |
| `basic_offboard.py` | EX | min offboard demo |
| `takeoff_and_land.py` | EX | arm + hover + land |
| `go_to.py` | EX | **GPS goto** — unusable GNSS-denied. |
| `keyboardcontrol.py` | EX | ⚠ **DQ if used in scored run**. |

### Telemetry / diagnostics

`get_position.py`, `get_position_with_task.py` (SharedState — concurrent pose+yaw streams, this is the **canonical pose source** for main loops), `imu.py`, `imutest.py`, `get_battery.py`, `get_flightmode.py`, `is_arm_air.py`, `drone_diagnostics.py`.

### Main navigation loops

| File | Role |
|---|---|
| `avoid.py` | MAIN — grid-heading nav (N/E/S/W), AvoidancePlanner local, 20 Hz, position setpoints. |
| `avoid_with_detect.py` | DUP — byte-for-byte same as `avoid.py`. **Detection not wired in yet** — this is the qualifier gap. |
| `vel_avoidance.py` | variant — same logic but velocity setpoints via VelocityPlanner. |

### Training

| File | Role |
|---|---|
| `Train_YOLO_Models.ipynb` / `_new.ipynb` | Colab pipeline: split data → `data.yaml` → `yolo detect train` → export `best.pt`+`best.onnx`. Both notebooks are effectively identical. |

---

## 5. Hardcoded constants

| Item | Value | Files |
|---|---|---|
| PX4 system addr | `udpin://0.0.0.0:14540` | drone_control, basic_offboard, etc. |
| Depth topic | `/depth_camera` | depth_receiver, depthtest |
| RGB topic | `/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image` | UseDetectorExample, save_photo, gzphotodetectorsaver |
| Camera intrinsics K | `[[433,0,320],[0,433,240],[0,0,1]]` (640×480, fx=fy=433) | AvoidancePlanner, GlobalMapper, RRTExample |
| Image resolution | 640 × 480 | AvoidancePlanner, avoid.py |
| Takeoff alt | 2.0–5.0 m | various |
| Grid headings | `[0°, 90°, 180°, -90°]` (N/E/S/W) | avoid.py |
| Control loop | 20 Hz | avoid.py |
| YOLO weights path | `yolov10n.pt`, `yolo11m.pt` | Detector, notebooks |

---

## 6. Known issues / cleanup backlog

1. `avoid.py` == `avoid_with_detect.py` byte-for-byte. Detection isn't wired into the main loop — **this is the qualifier blocker**.
2. `VelocityPlanner.py` ≈ `AvoidancePlanner.py`. Consolidate after qualifier.
3. `gzphotodetectorsaver.py:_process_task` — undefined `msg` → crash.
4. `get_position.py` uses `udp://:14540` (different scheme — works but inconsistent).
5. Both YOLO training notebooks effectively identical — pick one canonical.
6. `RRTExample.py` `__main__` block is missing `from GlobalMapper import GlobalMapper`.

---

## 7. Qualifier-specific notes (RoboVerse 2026)

- **Mission**: detect yellow (ground) + red (elevated) fuel barrels in 40×40×8 m space port. 10-min runs, best of multiple attempts.
- **Scoring**: yellow 50 pt, red 100 pt, +20 pt per 30 s under 5 min for full-class sweeps.
- **DQ rules**: no keyboard/joystick during scored run — **disable `keyboardcontrol.py`** in any run script.
- **GNSS-denied**: all pose comes from PX4 EKF2 fused with vision/odometry. `go_to.py` (GPS goto) is useless. Position drift over 10 min is the silent killer — see `APPROACH.md §Risks`.
- **Multi-altitude search needed**: red barrels are elevated → single low pass misses them. Either two altitude passes or wide vertical FOV.
- **No clock stop on crash**: 10-min wall clock keeps running even if code dies → main loop must restart on exception (supervisor).
- **Map released 1 day prior** — coverage waypoints can be precomputed the night before.

---

## 8. MVP algorithm (qualifier_run.py)

### Plain-English summary

The drone flies a "lawnmower" pattern over the arena — straight lanes back and forth, like mowing a lawn — at one fixed altitude. While flying, two things happen at the same time:

1. **Flying part**: it heads to the next waypoint in the lawn pattern. If the depth camera sees something in the way, a reactive planner picks a sideways direction with clear space and steers around it. Once clear, it goes back to the lawn pattern.
2. **Looking part**: 5 times a second it grabs an RGB photo, runs YOLO on it, and for any barrel it finds, it uses the depth value at the bbox center plus the drone's current position and yaw to figure out where the barrel actually is in the world. That world position gets logged.

A small bookkeeping module keeps the score. If it sees the same barrel twice (within 2 m), it doesn't double-count — it just averages the position and bumps a "seen again" counter. Yellow barrels = 50 pts, red = 100 pts.

The whole thing runs under a watchdog called the supervisor. The competition gives a 10-minute clock that doesn't stop even if the code crashes. So if anything explodes mid-flight, the supervisor catches the crash, restarts the mission with whatever time is left, and the score log persists across restarts so nothing already found is forgotten.

That's the whole MVP: fly a lawn pattern, dodge things, spot barrels, project them into the world, log them, survive crashes.

### Technical breakdown

End-to-end autonomous attempt. Four new files (`qualifier_run.py`, `coverage.py`, `detection_to_world.py`, `barrel_log.py`) glue pre-existing libs into one mission. Algorithm in plain steps:

**Pre-flight (sync, once per attempt)**
1. Load `MissionConfig` (defaults + JSON + CLI overrides).
2. Build long-lived singletons: `Detector` (YOLO worker pool), `DepthReceiver` (gz.transport13 sub on `cfg.depth_topic`), RGB `GzNode` sub, `BarrelLog` (autoload prior CSV if restart).
3. Generate lawnmower waypoints via `coverage.generate_lawnmower(origin, arena_w, arena_h, alt, lane_spacing, along_axis)` → `list[Waypoint(north, east, down, yaw_deg, is_turn)]`. Boustrophedon, single altitude.

**Per-attempt: `run_attempt()` spawns 2 cooperating asyncio tasks**

*A. `mission_loop` (flight)*
1. `Drone.connect()` → `position_monitor_task` (background NED pose + yaw into `SharedState`).
2. `arm_and_takeoff()` to `cfg.cruise_altitude_m`.
3. `filter_unvisited(waypoints, current_n, current_e, radius=1.5)` — drops already-visited if supervisor restarted mid-run.
4. For each waypoint, `_go_to_waypoint()` loop @ ~20 Hz:
   - Read latest depth frame + pose.
   - `AvoidancePlanner.compute_position_ned(depth, pose, K)` → `(target_ned, info)`. Internally: histogram of free directions over `num_bins=36`, clearance check vs `safe_distance=2.5` / `critical_distance=0.8`.
   - If `info["blocked"]` → drive toward planner sidestep `target_ned` (yaw follows travel direction).
   - Else → drive toward the waypoint (yaw follows lane heading).
   - Stop when `|pos - wp| < waypoint_radius_m` (0.8) or wall-clock deadline hit.
5. After last waypoint or budget exhausted → `drone.land()`. Any exception → emergency `land()` then re-raise to supervisor.

*B. `detection_loop` (perception)*
1. 5 Hz pull RGB frame from `gz` sub.
2. Snapshot current pose + latest depth frame + ts → `ctx`.
3. `Detector.submit_image(frame, ctx)` (non-blocking; queues to YOLO worker pool).
4. Worker thread invokes callback per detection:
   - Map YOLO class → canonical (`yellow_barrel` / `red_barrel`) via `cfg.detection_class_map`.
   - `detection_to_world(bbox, depth, pose, K, class, conf, ts, camera_pitch)`:
     - Take bbox-center pixel, sample depth via `median_patch_depth` (5×5).
     - Back-project to camera frame, rotate camera→body (OpenCV→FRD), rotate body→NED by yaw, translate by pose. Returns `WorldDetection(north, east, down, range, ...)` or `None` if depth invalid.
   - `BarrelLog.add(class, n, e, d, conf)`:
     - KDTree-style match against existing entries within `dedup_radius=2.0`.
     - New → append, write CSV atomically (`.tmp` + `os.replace`).
     - Repeat → running-mean position refine, bump `sightings` + `last_seen`. `is_new=False` so it doesn't double-score.
   - Lock-protected for multi-worker safety.

**Supervisor `supervisor(cfg)`**
- Wall clock = 600 s. Loop: build per-attempt `state.cfg = replace(cfg, wall_clock_budget_s=remaining)` (preserves numpy `K`), `state.stop_event.clear()` (reuse, don't reassign), `await run_attempt()`. On exception → log traceback, restart while budget > 0.
- Singletons (`Detector`, receivers, `BarrelLog`) survive across attempts — model load too slow to redo. Log autoloads its own CSV so score continues.

**Scoring readout**
- `BarrelLog.score()` = sum of `SCORES[class]` over first-seen entries. `{yellow_barrel: 50, red_barrel: 100}`.
- CSV at `runs/<ts>/barrels.csv` is crash-safe (rewritten every sighting).

**Key coupling diagram**

```
config + waypoints ─┐
                    ▼
SharedState ──► mission_loop ──► AvoidancePlanner ──► Drone.send_position_ned
   (pose)          │
                   │ (pose snapshot into ctx)
                   ▼
DepthReceiver ──► detection_loop ──► Detector.submit_image
   (depth)             │                    │ (worker callback)
                       │                    ▼
RGB sub ───────────────┘            detection_to_world ──► BarrelLog.add
                                                              │
                                                              ▼
                                                       barrels.csv + score()
```

**Where MVP is incomplete (gaps to fill)**
- No global RRT* fallback when reactive planner stays blocked >3s (stubbed only — see README §6).
- Single altitude → red elevated barrels may be missed. Two-altitude concat is a 3-line change.
- `yolov10n.pt` is COCO — barrel-trained weights still needed.
- Pose drift under GNSS-denied EKF2 is unbounded without vision/mocap fusion (silent killer over 10 min).

---

## 9. Reference docs

- PX4 EKF2 GNSS-denied: <https://docs.px4.io/main/en/advanced_config/gnss_degraded_or_denied_flight>
- PX4 external (vision) pose: <https://px4.gitbook.io/px4-user-guide/robotics/ros/ros1/external_position_estimation>
- PX4 EKF2 tuning: <https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf>
- Boustrophedon coverage planning (arxiv 1907.09224): <https://arxiv.org/pdf/1907.09224>
- Incremental UAV coverage in unknown area: <https://journals.sagepub.com/doi/10.1177/17568293241262323>
