# Finals — Real-Drone Stage

> Status 2026-06-06: the **hardware/software stack below is extracted from the official finals material**: [`example_code/`](example_code/) (12 scripts), [`UWBParserThread_Core_Documentation.pdf`](UWBParserThread_Core_Documentation.pdf), and the briefing images in [`reference_images/`](reference_images/). Detailed format/scoring/logistics still await the full briefing text — `context _dump.md` is reserved for it.

> **Team scope decided 2026-06-06: SWARM CHALLENGE ONLY** (pre-U team — not the mapping challenge). All mission code runs on the **C2 laptop** via the **pyhulax SDK** over Wi-Fi (`hula_connection.py`/`dola.py` path) with detection on laptop-streamed frames; the onboard stack below (MAVSDK serial, RKNN NPU, RealSense, ROS 2 UWB) is **mapping-challenge reference material only**. The finals codebase lives in the **`finals/` package** at repo root — the plain-language strategy is in [`finals/README.md`](../../finals/README.md); start every work session from [`finals/docs/module_map.md`](../../finals/docs/module_map.md) (status table, session roadmap, binding conventions). pyhulax SDK reference: https://pyhulax.xenops.ae

## 1. Format & scoring

From the briefing images in [`reference_images/`](reference_images/):

- **3 × Highgreat HULA drones** per team, operated from a **participants' C2 terminal** (ground station controls the swarm — see `hula_connection.py`).
- **Targets**: a **convoy of 5 RoboMaster ground robots** driving a route through the arena — i.e. *moving* targets, unlike the static qualifier barrels. The convoy route passes threat markers (red icons on the briefing map).
- **Landing zones**: the arena has marked **valid vs invalid landing zones** (H-pads) — where you land matters; plan end-of-mission landings onto valid pads.

<!-- TODO from briefing text: rounds, arena dimensions, time limit, exact scoring table, what the threat/explosion icons mean, whether detection or tracking of the convoy is scored. -->

Hints from the example code: `potential_detection_targets.py` demos **ArUco markers (DICT_6X6_250)** and mentions QR codes as likely targets — expect fiducial detection alongside object classes (RoboMaster robots are car-like; note the example `class_names = ["person", "car", "bicycle"]`).

## 2. Hardware stack (from example code)

| Component | What | Evidence |
|---|---|---|
| Airframe | **Highgreat HULA drone × 3** — controlled over Wi-Fi via the `pyhulax` SDK from the C2 terminal; **swarm** operation | `hula_connection.py`, briefing image |
| Drone discovery | **Dola** UDP broadcast listener on port **8668** — maps `plane_id` → IP, serial, Wi-Fi mode (docstring says 8688; code says 8668 — trust the code) | `dola.py` |
| Flight controller | **PX4 on real hardware**, reached via MAVSDK over **serial `/dev/ttyS6` @ 921600 baud** (not UDP like SITL) | `mapping_drone.py:139` |
| Companion computer | **Rockchip NPU board** (RK35xx-class) — inference runs through **RKNN Lite** (`rknnlite.api`) | `rknndecoder.py`, `getDepthAndDetect.py` |
| Camera | **Intel RealSense D430/D450**: RGB 640×480 BGR8 @30 FPS, depth 640×480 Z16 @30 FPS, stereo IR 640×480 Y8 @30 FPS; intrinsics from the RealSense API | `getRGB.py`, `getDepth.py`, `getInfra.py` |
| Positioning | **UWB tags** — a serial UWB transceiver (auto-detected COM port, **921600 baud**) parsed by `UWBParserThread`: thread-safe `get_tag_position(tag_id) → (x_m, y_m, unix_time)`, per-tag dict, configurable `x_origin`/`y_origin`. On the drone side the pose arrives on the **ROS 2 topic `uwb_tag`** (`PoseStamped`, BEST_EFFORT): N ← `pose.position.y`, E ← `pose.position.x` | [`UWBParserThread_Core_Documentation.pdf`](UWBParserThread_Core_Documentation.pdf), `mapping_drone.py:51-91` |

<!-- TODO from briefing: confirm exact Rockchip board, RealSense model, battery counts/flight time, comms details. -->

## 3. Software stack (from example code)

- **Control**: same **MAVSDK** API as the qualifiers (arm, takeoff, offboard `VelocityNedYaw`), just a serial connection string. `mapping_drone.py` is effectively the finals version of our control loop: P-controller waypoint flight (KP=0.1, max 0.5 m/s horizontal / 0.3 m/s vertical, 0.8 m takeoff, 0.2 m waypoint threshold) + hover deadband.
- **Inference**: **YOLOv11n exported to `.rknn`** (quantized), input **640×640** RGB uint8, output (1, 8400, 84) decoded by `rknndecoder.py` (sigmoid → xywh→xyxy → NMS; conf 0.25, IoU 0.45). **Not ultralytics at runtime** — ultralytics is only used to train/export; the NPU runs RKNN Lite.
- **Depth + detection**: depth frame aligned to color (`rs.align`), bbox-centre depth via `depth_frame.get_distance(cx, cy)`, then `rs2_deproject_pixel_to_point(intrinsics, …)` → camera-frame XYZ (`getDepthAndDetect.py`). This replaces our manual K-matrix math in `detection_to_world.py` — RealSense provides calibrated intrinsics directly.
- **Mapping**: `generateTopDown.py` builds a top-down occupancy grid from deprojected depth — **5 cm cells, 200×200 grid (10 m × 10 m)**, depth clamped to 0.2–5.0 m, morphological close/open denoise. `getDepthPointCloud.py` shows raw `rs.pointcloud()` access.
- **Swarm**: `hula_connection.py` discovers all drones via Dola, connects with `DroneAPI().connect(ip)`, streams video per-drone (`VideoStream.latest_frame.to_rgb()` → numpy, ready for detection).
- **ROS 2** (`rclpy`) is in the loop for UWB only — spun in a daemon thread alongside asyncio/MAVSDK.

## 4. Sim → real differences

- [x] **Pose source** — ANSWERED: real positioning is **UWB via ROS 2 `uwb_tag`**, not EKF2 vision fusion. The qualifier's biggest risk (R1 pose drift, see the [design rationale](../quali/design-rationale.md)) is resolved by external UWB — but verify UWB update rate, latency, and accuracy on the rig, and how altitude (D) is sourced (example uses PX4 telemetry `position_velocity_ned().down_m` for D, UWB for N/E).
- [x] **Camera intrinsics** — ANSWERED: no hand-tuned K matrix; RealSense supplies calibrated intrinsics at runtime (`getDepthAndDetect.py:53-63`).
- [x] **Sensor transport** — ANSWERED: `pyrealsense2` replaces gz-transport; `depth_receiver.py` and the Gazebo RGB subscriber are dead code for finals.
- [ ] **Inference port** — our trained weights must be re-exported: ultralytics `.pt` → ONNX → **`.rknn`** (RKNN toolkit, quantization needs a calibration image set). Class count changes the output tensor (84 = 4+80 COCO; a 2-class model gives 4+2) — `rknndecoder.py` handles it generically but verify.
- [ ] **Wind / lighting / latency** — still unmodelled; expect retuning of speeds, thresholds, confidence.
- [ ] **YOLO sim-to-real gap** — weights trained on Gazebo renders will degrade on real imagery; plan a real-image capture + fine-tune session (see [training pipeline](../quali/training-pipeline.md)).
- [ ] **Swarm coordination** — entirely new vs qualifiers: 3 HULA drones from one C2 terminal; decide task split (e.g. one mapper + two trackers) once the briefing lands.
- [ ] **Moving targets** — the qualifier's `barrel_log.py` dedup (running-mean position, dedup radius) assumes **static** objects; the finals convoy of RoboMaster robots **moves**, so per-sighting logging or actual tracking replaces dedup. Re-think scoring-log design.
- [ ] **Landing discipline** — finals arenas mark valid vs invalid landing zones; mission end must navigate to a valid H-pad, not land-in-place like `qualifier_run.py` does.

## 5. What carries over from qualifiers

**Reusable as-is (pure logic):**
- `coverage.py` — lawnmower waypoint generation (feed waypoints to `mapping_drone.py`-style flight)
- `barrel_log.py` — dedup + scoring + crash-safe CSV (target classes may change)
- The supervisor/crash-restart pattern from `qualifier_run.py`

**Reusable with adaptation:**
- MAVSDK control path (`drone_control.py`) — same API; swap connect string to `serial:///dev/ttyS6:921600`. The example `mapping_drone.py` already implements the equivalent loop with UWB in place of EKF2 N/E.
- `detection_to_world.py` — the back-projection math is superseded by RealSense deprojection, but the camera→body→NED frame transform (using yaw + camera pitch) is still needed to turn camera-frame XYZ into world NED.
- Training pipeline — train as before, then **add an export step**: `.pt` → ONNX → `.rknn`.

**Dead for finals:**
- Gazebo-bound receivers (`depth_receiver.py`, gz-transport subscribers), screen-capture fallback, PX4 SITL specifics.

## 6. Risks & test plan

<!-- TODO: risk register + ordered test plan (bench → tethered hover → free hover → waypoint → full mission). Seeds: UWB dropout behavior; RKNN quantization accuracy loss vs .pt; serial link saturation at 921600 baud with video streaming; NPU FPS with yolo11n at 640x640. -->

## 7. Logistics

<!-- TODO: dates, venue, hardware access windows, who brings what. -->

## Example code index

| File | What it demonstrates |
|---|---|
| `hula_connection.py` | Connect to Hula drones via `pyhulax`, video streams, movement commands |
| `dola.py` | UDP discovery of Hula aircraft (port 8668) |
| `mapping_drone.py` | Full control loop: MAVSDK serial + ROS 2 UWB + P-controller waypoint flight |
| `getRGB.py` / `getDepth.py` / `getInfra.py` | RealSense stream basics (color / depth / stereo IR) |
| `getSyncDepthColor.py` | Aligned depth+color frames |
| `getDepthAndDetect.py` | RKNN YOLOv11 inference + per-detection 3D deprojection |
| `getDepthPointCloud.py` | Raw point cloud from depth |
| `generateTopDown.py` | Top-down occupancy grid (5 cm cells, 10×10 m) |
| `rknndecoder.py` | YOLOv11 RKNN output decoding (sigmoid, NMS, box scaling) |
| `potential_detection_targets.py` | ArUco marker detection + depth lookup (likely finals targets) |
