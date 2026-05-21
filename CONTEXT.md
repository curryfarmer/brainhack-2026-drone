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

## 8. Reference docs

- PX4 EKF2 GNSS-denied: <https://docs.px4.io/main/en/advanced_config/gnss_degraded_or_denied_flight>
- PX4 external (vision) pose: <https://px4.gitbook.io/px4-user-guide/robotics/ros/ros1/external_position_estimation>
- PX4 EKF2 tuning: <https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf>
- Boustrophedon coverage planning (arxiv 1907.09224): <https://arxiv.org/pdf/1907.09224>
- Incremental UAV coverage in unknown area: <https://journals.sagepub.com/doi/10.1177/17568293241262323>
