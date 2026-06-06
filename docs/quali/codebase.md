# Codebase reference

Reference map of the drone autonomy stack built for the **RoboVerse Qualifier 2026** (BrainHack): PX4 SITL + Gazebo Harmonic + MAVSDK + YOLO. Goal: find yellow + red fuel barrels in a 40 m × 40 m × 8 m GNSS-denied space port within 10 minutes. The repo **root is the code directory** — every path on this page is relative to the top of the checkout. This page covers the architecture, the file-by-file inventory, the module APIs, hardcoded constants, and the known-issues backlog. For setup and running see the [deployment guide](deployment.md) and [simulator testing](simulator-testing.md); for *why* the mission code is shaped this way see the [design rationale](design-rationale.md); for the YOLO data/train/deploy flow see the [training pipeline](training-pipeline.md). Per-file authorship markers ("Edited by Claude" headers etc.) are catalogued in [AUTHORSHIP](../AUTHORSHIP.md).

---

## Architecture

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

**Note:** the `avoid.py` loop at the bottom of the diagram is the **legacy** path — it has been superseded by the two qualifier mission entry points. The mission architecture of `qualifier_run.py` (supervisor + mission loop + detection loop) lives in [design-rationale.md](design-rationale.md). `qualifier_main.py` (the teammate DFS stack) consumes the same sensor/control libraries shown above (`Detector`, `depth_receiver`, `drone_control`, `get_position_with_task`, `AvoidancePlanner`).

---

## File inventory

Legend: **LIB** = library class importable elsewhere · **MAIN** = autonomous entry point · **EX** = example/demo · **TEST** = diagnostic · **DUP** = near-duplicate of another file · **BUG** = known issue.

### Mission entry points

| File | Role | Notes |
|---|---|---|
| `qualifier_run.py` | MAIN | Asyncio supervisor + lawnmower coverage + detection loop — the documented mission design (see [design-rationale.md](design-rationale.md)). Outputs `runs/<timestamp>/barrels.csv` + annotated detection frames. Module APIs below. |
| `qualifier_main.py` | MAIN | Teammate's DFS cell-grid exploration (docstring says 1 m cells; `CELL_SIZE_M` in the current copy is 2.0). YOLO runs in a background thread via `Detector` + `BarrelLogger`. Outputs `barrels.json` + `visited_cells.json`. Uploaded May 22. |
| `qualifier_main updates/` | snapshots | Teammate iteration snapshots of `qualifier_main.py`. `test2.3` is a bare file (29,075 B, same size as root copy); `test2.4/` has `qualifier_main.py` (30,646 B) + its own `Detector.py`; `test2.5/` (nested three dirs deep) is a full repo snapshot. **Note: `test2.5` holds a NEWER `qualifier_main.py` than root — 31,722 vs 29,075 bytes.** Diff before extending the root copy. |
| `avoid.py` | MAIN (legacy) | Grid-heading nav (N/E/S/W), AvoidancePlanner local, 20 Hz, position setpoints. **No longer "current best"** — superseded by the two qualifier mains above. |
| `avoid_with_detect.py` | DUP (legacy) | Byte-for-byte same as `avoid.py`. Detection was never wired in here — that gap was closed by the qualifier mains (see Known issues #1). |
| `vel_avoidance.py` | variant (legacy) | Same logic as `avoid.py` but velocity setpoints via VelocityPlanner. |

### Mission support modules (qualifier stack)

| File | Role | Notes |
|---|---|---|
| `coverage.py` | LIB | Lawnmower waypoint generator (pure — no I/O, no drone deps). Used by `qualifier_run.py`. |
| `detection_to_world.py` | LIB | bbox + depth + pose → world NED point (pure). Used by `qualifier_run.py`. |
| `barrel_log.py` | LIB | Thread-safe dedup + scoring + crash-safe CSV persistence. Used by `qualifier_run.py`. |
| `barrel_logger.py` | LIB | JSON-based variant of `barrel_log.py` used by `qualifier_main.py`. Owns its own pixel→NED projection and cluster-merge; saves `barrels.json` at end of run. No scoring table, no crash-safe incremental write. |
| `barrels.json` / `visited_cells.json` | output | Committed sample outputs of a `qualifier_main.py` run (detected barrels in world NED; cell coverage record). |
| `run.sh` | tool | Launcher for the no-git ZIP-download workflow: kills zombie `mavsdk_server`, warns if UDP `:14540` is already owned, then `exec python3 <script>`. |

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
| `Detector.py` | LIB | Threaded YOLO worker pool. `submit_image(img, ctx)` → async detections; non-blocking display queue. Configurable `model_path`, `confidence_threshold`, `num_workers`, `device`, plus `config_path` to read the deployed model path from `model_config.json` (regressed May 22, restored 2026-06-06 — see Known issues #7). |
| `UseDetectorExample.py` | EX | Wires Detector to Gazebo RGB topic. RGB→BGR conversion, per-frame metadata. |
| `gzphotodetectorsaver.py` | BUG | Burst capture w/ YOLO. `_process_task` references undefined `msg` → crashes. |
| `yolov10n.pt` | weight | COCO classes — **not barrel-trained**. Placeholder baseline. |
| `best.pt` | weight | 6.2 MB trained barrel weights at repo root. **But `model_config.json` still points at the COCO `yolov10n.pt` placeholder — verify which weights each entry point actually loads before flying** (`qualifier_main.py` hardcodes `best.pt`; `qualifier_run.py` defaults to `yolov10n.pt` unless `--weights` is passed). |
| `model_config.json` | config | Deployment pointer written by `deploy_model.py` (model path + version + metrics). Its note says Detector reads it via `config_path='model_config.json'` — accurate again now that `config_path` is restored (Known issues #7). **But it still points at the COCO `yolov10n.pt` placeholder** — update it or pass `--weights` explicitly (Known issues #9). |

### Drone control

| File | Role | Notes |
|---|---|---|
| `drone_control.py` / `_new.py` | LIB | `Drone` class: connect, arm/takeoff, send vel/pos NED, yaw control. `_new` adds geo-fencing, `wait_until_ready()`. |
| `basic_offboard.py` | EX | min offboard demo |
| `takeoff_and_land.py` | EX | arm + hover + land |
| `go_to.py` | EX | **GPS goto** — unusable GNSS-denied. |
| `keyboardcontrol.py` | EX | ⚠ **DQ if used in scored run**. |
| `KEY2.py` | EX | Velocity-body keyboard teleop via `pynput` (`VelocityBodyYawspeed` at 20 Hz; w/s throttle, a/d yaw, u/j pitch, h/k roll, ESC lands). Same ⚠ **DQ warning** as `keyboardcontrol.py` — never in a scored run. |

### Telemetry / diagnostics

`get_position.py`, `get_position_with_task.py` (SharedState — concurrent pose+yaw streams, this is the **canonical pose source** for main loops), `imu.py`, `imutest.py`, `get_battery.py`, `get_flightmode.py`, `is_arm_air.py`, `drone_diagnostics.py`.

### Training pipeline

Full walkthrough in [training-pipeline.md](training-pipeline.md).

| File | Role | Notes |
|---|---|---|
| `pipeline.py` | MAIN (orchestrator) | In-process orchestrator for the YOLO pipeline — each stage imported and called via its `run(ctx)` entry point. |
| `collect_yolo_data.py` | stage | Hybrid keyboard-fly + auto-tick + burst capture in sim; screen-capture fallback (lazily imports `mss`, which is back in `requirements.txt` — see Known issues #8). |
| `import_roboflow.py` | stage | Pull labelled datasets exported from Roboflow into the local layout. |
| `validate_labels.py` | stage | Sanity-checks YOLO label files (bbox ranges, class ids) before training. |
| `split_train_val.py` | stage | Deterministic train/val split. |
| `gen_data_yaml.py` | stage | Emits `data.yaml` for ultralytics from the split + `classes.txt`. **Root `classes.txt` does not exist — create it first (one class name per line; see `tests/fixtures/data_ok/classes.txt` for the format).** |
| `train_yolo.py` | stage | Wraps `yolo detect train`. |
| `eval_model.py` | stage | mAP / per-class eval of a trained checkpoint. |
| `deploy_model.py` | stage | Versioned, no-source-edit deployment — copies weights, writes `model_config.json` + `models/latest_path.txt`. |
| `gen_smoke_data.py` | tool | Generates synthetic smoke-test data for pipeline dry runs. |
| `models/latest_path.txt` | output | Pointer to the most recently deployed weights (currently `yolov10n.pt` — i.e. the placeholder, not `best.pt`). |
| `tests/fixtures/` | TEST data | Smoke-test datasets: `data_ok/` (valid layout incl. `classes.txt`, `data.yaml`, session metadata), `data_bad_bbox/` (deliberately broken labels), `sample.jpg`. |
| `Train_YOLO_Models.ipynb` / `_new.ipynb` | legacy | Colab notebooks: split data → `data.yaml` → `yolo detect train` → export `best.pt`+`best.onnx`. Both effectively identical. **Superseded by the script pipeline** — see [training-pipeline.md](training-pipeline.md). |

---

## Module API reference

The qualifier stack proper is four Claude-designed modules plus the teammate's two-module stack. The four:

```
qualifier_run.py        ── main entry point (asyncio supervisor + mission/detection loops)
coverage.py             ── lawnmower waypoint generator (pure)
detection_to_world.py   ── bbox + depth + pose → world NED point (pure)
barrel_log.py           ── thread-safe dedup + scoring + CSV persistence
```

Everything else in the repo root is **pre-existing** and is consumed by these files as libraries — you should not need to touch it to run the MVP.

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

### `qualifier_main.py` + `barrel_logger.py` (teammate stack)

The second, independently developed mission entry point (uploaded May 22). Same sensor/control libraries, different search strategy and barrel bookkeeping.

**Strategy** (from its docstring): discretise the depot floor into grid cells (`CELL_SIZE_M = 2.0` in the current copy; the docstring still says 1 m × 1 m). DFS from the takeoff cell; at each cell try directions in `DIRECTION_ORDER = ["N", "E", "W", "S"]`. Before moving to a neighbour cell, rotate to face it and check depth clearance in the centre column (`_check_blocked_ahead()`, `BLOCKED_DISTANCE_M = 2.0`); if blocked, mark that cell BLOCKED and try the next direction. When all four neighbours are visited or blocked, pop the DFS stack and backtrack along visited cells. When the stack is empty the search is complete — save outputs and land.

**Structure:**
- `RGBReceiver(topic, detector)` — mirrors `depth_receiver.DepthReceiver` but for RGB uint8 frames, and pumps frames straight into the `Detector` instead of holding them.
- `QualifierMission` — owns the DFS state machine. Key methods: `setup()` (connect, wait for pose/depth, arm/takeoff), `run()` (DFS main loop at `LOOP_HZ = 10`), `shutdown()` (save JSON outputs + land), plus internals `_cell_to_ned()`, `_check_blocked_ahead()`, `_rotate_to()`, `_next_unvisited_direction()`, `_backtrack_direction()`.
- Tunables at the top of the file: `CELL_SIZE_M=2.0`, `ALTITUDE_M=1.0`, `MAX_RUNTIME_S=600`, `MAX_CELLS_VISITED=2000`, `WAYPOINT_TOLERANCE_M=0.4`, `WAYPOINT_TIMEOUT_S=12.0`, `BLOCKED_DISTANCE_M=2.0`, `LOOP_HZ=10.0`, `YOLO_MODEL_PATH="best.pt"` (hardcoded), `BARREL_CLASS_NAMES=["Yellow Barrel", "Red Barrel"]`, `YOLO_CONFIDENCE=0.5`.
- `BARREL_CLASS_NAMES` must match the trained model's class names *exactly* (check `model.names`); an empty list `[]` accepts all detected classes — useful for first runs.

**Outputs:** `barrels.json` (deduplicated barrels in world NED) and `visited_cells.json` (coverage record for scoring/debugging) — written once at shutdown, not incrementally.

**`barrel_logger.BarrelLogger`** — the JSON-based counterpart of `barrel_log.BarrelLog`:

```python
from barrel_logger import BarrelLogger

logger = BarrelLogger(
    K,                              # 3x3 camera intrinsics
    barrel_class_names=["Yellow Barrel", "Red Barrel"],
    get_pose_fn=get_pose,           # -> {'north','east','down','yaw'} (yaw radians) or None
    get_depth_fn=get_depth,         # -> float32 depth frame (metres) or None
    merge_radius_m=1.0,
    min_confidence=0.5,
    max_depth_m=10.0,
)

detector.callback = logger.on_detection   # Detector worker-thread callback
logger.get_barrels()                       # list[dict] snapshot
logger.save_json("barrels.json")           # final dump
```

- `on_detection(detections, annotated_image, context)` runs in the Detector worker thread — pulls current pose + depth via the injected callables, samples a median 5-px depth patch at the bbox centre, projects pixel + depth → camera optical → body FRD → world NED (yaw-only rotation), then merges.
- Cluster merge uses **horizontal distance only** (altitude estimate is noisy) within `merge_radius_m`; merged entries keep best confidence, bump `hits`, and refine position via running mean.
- Detections beyond `max_depth_m` (default 10 m) are discarded as likely background/noise.
- Differences vs `barrel_log.py`: JSON instead of CSV, no scoring table, no `autoload`/crash-safe incremental rewrite — if the process dies mid-run, nothing is persisted.

**Caveat:** the newest iteration of this stack is in `qualifier_main updates/test2.5/` (31,722 B) — newer than the root `qualifier_main.py` (29,075 B). Diff the two before extending.

---

## Hardcoded constants

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
| YOLO weights path | `yolov10n.pt`, `yolo11m.pt` | UseDetectorExample (passed to Detector — whose own `model_path` default is `yolov8n.pt`), notebooks |

Additions since that table was written: `qualifier_main.py` hardcodes `YOLO_MODEL_PATH = "best.pt"` and the same topics/intrinsics as above; `qualifier_run.py` keeps all of its tunables in `MissionConfig` (CLI/JSON-overridable) rather than module constants — see the [deployment guide](deployment.md) for the config surface. Before any run, verify which weights file is actually wired in: `best.pt` exists at root, but `model_config.json` and `models/latest_path.txt` still point at the COCO `yolov10n.pt` placeholder.

---

## Known issues

1. ~~`avoid.py` == `avoid_with_detect.py` byte-for-byte. Detection isn't wired into the main loop — **this is the qualifier blocker**.~~ **RESOLVED** — both `qualifier_run.py` (detection loop feeding `BarrelLog`) and `qualifier_main.py` (Detector background thread feeding `BarrelLogger`) wire detection into their mission loops. The `avoid*.py` pair remains as untouched legacy code.
2. `VelocityPlanner.py` ≈ `AvoidancePlanner.py`. Consolidate after qualifier.
3. `gzphotodetectorsaver.py:_process_task` — undefined `msg` → crash.
4. ~~`get_position.py`, `imutest.py`, and `keyboardcontrol.py` use the legacy `udp://:14540` scheme (reverted by the May 22 upload) — the scheme the gRPC troubleshooting table in the [deployment guide](deployment.md) warns can segfault `mavsdk_server` under MAVSDK 2.x on the first outbound write.~~ **RESOLVED — restored 2026-06-06 from `5127897`** — all three use `udpin://0.0.0.0:14540` again.
5. Both YOLO training notebooks effectively identical — pick one canonical. (Both are now legacy anyway — superseded by the script pipeline, see [training-pipeline.md](training-pipeline.md).)
6. `RRTExample.py` `__main__` block is missing `from GlobalMapper import GlobalMapper`.
7. ~~**REGRESSION (May 22 upload):** `Detector.py` lost the `config_path` parameter that read the deployed model path from `model_config.json`.~~ **RESOLVED — restored 2026-06-06 from `5127897`** — `Detector(config_path='model_config.json')` works again. But `model_config.json` still points at the COCO `yolov10n.pt` placeholder (issue #9) — update it or pass `--weights best.pt` explicitly.
8. ~~**REGRESSION (May 22 upload):** `requirements.txt` lost `mss`, `torch>=2.1`, and the `ultralytics<9` pin.~~ **RESOLVED — restored 2026-06-06 from `5127897`** — all three entries are back; a fresh `pip install -r requirements.txt` covers the `mss` screen-capture fallback in `collect_yolo_data.py`.
9. **Weights ambiguity:** `best.pt` (6.2 MB, trained) sits at root, but `model_config.json` and `models/latest_path.txt` point at the COCO `yolov10n.pt` placeholder, and `qualifier_run.py` defaults to `yolov10n.pt` unless `--weights` is passed (`qualifier_main.py` hardcodes `best.pt`). Verify the actual weights wired into whichever entry point you fly.
10. **Duplicated mission stacks:** `qualifier_run.py` and `qualifier_main.py` solve the same mission with disjoint barrel bookkeeping (`barrel_log.py` CSV vs `barrel_logger.py` JSON). Pick one canonical stack for finals; the teammate-stack breakdown above plus the roadmap notes in [design-rationale.md](design-rationale.md) are the starting point.
11. ~~**REGRESSION (May 22 upload):** `drone_control.py` lost the `_kill_stale_servers()` self-healing cleanup that ran at the top of `connect()` (auto-killing zombie `mavsdk_server` before binding UDP `:14540`).~~ **RESOLVED — restored 2026-06-06 from `5127897`** — `connect()` self-heals again; `run.sh` and the manual cleanup snippet in the [deployment guide](deployment.md) remain belt-and-braces.
