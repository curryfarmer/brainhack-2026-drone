# Finals — Real-Drone Stage

> Status 2026-06-06: the **hardware/software stack below is extracted from the official finals material**: [`example_code/`](example_code/) (12 scripts), [`UWBParserThread_Core_Documentation.pdf`](UWBParserThread_Core_Documentation.pdf), and the briefing images in [`reference_images/`](reference_images/). Detailed format/scoring/logistics still await the full briefing text — `context _dump.md` is reserved for it.

> **Team scope decided 2026-06-06: SWARM CHALLENGE ONLY** (pre-U team — not the mapping challenge). All mission code runs on the **C2 laptop** via the **pyhulax SDK** over Wi-Fi (`hula_connection.py`/`dola.py` path) with detection on laptop-streamed frames; the onboard stack below (MAVSDK serial, RKNN NPU, RealSense, ROS 2 UWB) is **mapping-challenge reference material only**. The finals codebase lives in the **`finals/` package** at repo root — the plain-language strategy is in [`finals/README.md`](../../finals/README.md); start every work session from [`finals/docs/module_map.md`](../../finals/docs/module_map.md) (status table, session roadmap, binding conventions). pyhulax SDK reference: https://pyhulax.xenops.ae

## 1. Format & scoring

From the briefing images in [`reference_images/`](reference_images/):

- **3 × Highgreat HULA drones** per team, operated from a **participants' C2 terminal** (ground station controls the swarm — see `hula_connection.py`).
- **Targets**: a **convoy of 5 RoboMaster ground robots** driving a route through the arena — i.e. *moving* targets, unlike the static qualifier barrels. The convoy route passes threat markers (red icons on the briefing map).
- **Landing zones**: the arena has marked **valid vs invalid landing zones** (H-pads) — where you land matters; plan end-of-mission landings onto valid pads.
- **Arena/cage size** (user onsite estimate, 2026-06-10): roughly **5.3 m × 11.3 m** (17.5 × 37 "footlengths" ≈ 0.305 m each — i.e. feet — with a **huge stated margin of error**). Narrow and elongated (~1:2). Treat as order-of-magnitude only and tape-measure at gate D; it is *smaller and longer* than the `configs/arenas/sample.json` 12×10 placeholder. Comfortably inside the 50 m HULA Wi-Fi comm range (§2.0).
- **Field ArUco markers** (user, 2026-06-10): dictionary **`cv2.aruco.DICT_7X7_1000`** (not the `DICT_6X6_250` our detector currently hardcodes — **must change or we read nothing**); **5 markers, ids 11/45/51/67/101 at FIXED known (x,y) m**: 11=(1.35,4.40), 45=(1.30,7.85), 51=(4.40,4.40), 67=(1.95,8.70), 101=(4.40,7.85). STATIC (not the moving convoy) and **not the landing pads** (role still TBD). Monocular only — no depth. Known coords ⇒ absolute position fixes for our position-blind nav + ground-truth for the guessed bounds. Full analysis + optimisation levers: [`../../finals/docs/field_markers.md`](../../finals/docs/field_markers.md).

<!-- TODO from briefing text: rounds, arena dimensions, time limit, exact scoring table, what the threat/explosion icons mean, whether detection or tracking of the convoy is scored. -->

Hints from the example code: `potential_detection_targets.py` demos **ArUco markers (DICT_6X6_250)** and mentions QR codes as likely targets — expect fiducial detection alongside object classes (RoboMaster robots are car-like; note the example `class_names = ["person", "car", "bicycle"]`).

## 2. Hardware stack (from example code)

### 2.0 HULA airframe — official manufacturer spec (swarm challenge)

Datamined 2026-06-10 from the **official HULA manual**
(<https://ds-api.hg-fly.net/manuals/Hula_EN.html>). This is the consumer/EDU
HULA spec; we drive it via the **pyhulax SDK** (not the Scratch/APP path the
manual documents), so treat the *programming* sections as N/A and the
*physical/sensor envelope* as load-bearing. Confirm anything starred onsite
(gate F).

| Domain | Spec (exact manual values) |
|---|---|
| Airframe | weight **100 g (±3 g)**; **189.3×184.6×50 mm**; axle distance 128 mm; prop 75 mm / 3"; motor "L8.5 20"; **max tilt 20°** |
| Battery | **1200 mAh, 3.8 V** Li-ion, 31 g; **flight time 9–10 min**; charge ≈1 h (box) / ≈1 h 40 (USB) |
| Camera | photo **1920×1080 (JPG)**; video **720p/30fps (MP4)**, auto-drops to 360p/30 in line-patrol mode; **field of view 71°** |
| Positioning | **optical flow ±20 cm H / ±10 cm V**; **QR-code floor ±5 cm H / ±10 cm V**; "support expansion UWB positioning" |
| Obstacle avoidance | four-direction infrared, effective **30–50 cm**, knob-adjustable |
| Laser | 640 nm, max lighting power 1.5 W |
| Comms | PCB antenna; **2.412–2.462 GHz + 5.745–5.825 GHz**; ≤14 dBm EIRP; **range 50 m**; default Wi-Fi password `12345678` |
| Connectivity | **direct** (device joins aircraft Wi-Fi) or **networking** (both device + aircraft on a router — *required for multi-drone*) |
| Flight envelope | max height **10 m**; max horizontal speed **1.5 m/s (APP) / 3 m/s (optical-flow mode)**; climb **1.2 m/s**; descent **1.0 m/s**; wind < Class 3; temp 0–40 °C |
| Built-in modes | Scratch / Program Lab programming, "AI recognition", "line patrol", flight stunts — all **bypassed**; we command via pyhulax |
| Expansion | serial expansion port (UWB module etc.) |

**What this changes for our code** (each is an onsite-verifiable config value,
not a rewrite):

1. **Battery is the hard mission clock.** 9–10 min/charge means our
   `mission_budget_s` of 500–600 s ≈ one *whole* pack. Budget margin, and plan
   a fresh battery per scored run (sim already drains 3 packs per 3-drone run).
2. **Real video is 720p (1280×720), not the 640×480 sim camera.** ArUco decode
   standoff is *better* than our 640 px range math (≈2× px/marker), so the
   sentry-altitude floor in `simulation.md` Tier 2 is conservative — good news;
   re-derive at 1280 px at gate F.
3. **`camera_hfov_deg` now has a starting figure: 71°** (was `null` →
   `bearing_deg` null). Caveat: the manual doesn't say H vs diagonal — if 71° is
   diagonal, 16:9 HFOV ≈ 64°. Seed the config, still bench-confirm gate F.
4. **Onboard positioning EXISTS** (optical flow ±20 cm, QR-floor ±5 cm). Our
   landing-nav is built **position-blind** (open-loop compass dead-reckon,
   assumes `PositionQuality.NONE`). The real question is what *pyhulax* exposes
   at the arena: no QR floor mat ⇒ optical-flow-only or NONE. Keep open-loop DR
   as the floor; treat any usable `PositionQuality` (and the 3 m/s optical-flow
   speed unlock) as a bonus, not a dependency.
5. **Multi-drone needs networking mode** (all 3 on a router) — matches the
   `dola` discovery + `pyhulax` connect path we already use.
6. **4-direction IR avoidance (30–50 cm)** is a hardware backstop near crates,
   but too short-range to plan with — keep the visibility-graph keep-out
   inflation as the real obstacle margin.
7. **50 m comm range** bounds how far a drone can fly from the C2 laptop.

### 2.1 Onboard stack (mapping-challenge reference only)

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
- [x] **Physical-flight simulation** — ANSWERED (research pass, 2026-06-06): the swarm mission is simulable **end-to-end** (3× PX4 SITL + Gazebo Harmonic in the qualifier VM, moving ArUco convoy, per-drone 640×480 cameras into our real detector) and **partwise** (headless 3× SITL flight-only; pure-Python kinematic tier on Windows). PX4 SITL is a *physics stand-in* behind the FlightAdapter seam — the finals drones are HULA/pyhulax, and what no sim covers (move() units, HULA dynamics, camera HFOV, `.to_rgb()` order, Wi-Fi ×3 video) is exactly the onsite-window scope. Full matrix + tier recipes + rejected alternatives (incl. turtlesim): [`finals/docs/simulation.md`](../../finals/docs/simulation.md).
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
