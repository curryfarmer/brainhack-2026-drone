# RoboVerse Qualifier 2026 — Solution Approach (draft v0)

> Working doc. Not implementation. Read top-to-bottom, then we discuss before any code goes in.
> See `CONTEXT.md` for what the existing modules do.

---

## TL;DR — Recommended approach

> **Pre-baked coverage waypoints + reactive depth avoidance + multi-altitude sweep + bbox-to-world barrel logger with spatial dedup, wrapped in a crash-restart supervisor.**

Reuses ~80% of the existing code as-is. Only new code: (a) coverage waypoint generator, (b) detection-to-world projection + dedup store, (c) supervisor. The "main loop" replaces `avoid.py` / `avoid_with_detect.py` (which are currently the same file).

---

## 1. Problem decomposition

The qualifier reduces to four sub-problems. Map each to existing code where possible.

| Sub-problem | Existing tool | Gap |
|---|---|---|
| **A. Localize without GPS** | PX4 EKF2 fuses IMU + vision_odometry. `get_position_with_task.SharedState` reads NED telemetry. | Need to verify the Gazebo `roboverse` world actually provides vision_odometry — see §Risks. If not, drift will be unrecoverable. |
| **B. Sweep the 40×40 m space port** | nothing — `avoid.py` does grid-direction flips reactively but doesn't cover area. | New: coverage waypoint generator (boustrophedon). |
| **C. Don't crash into walls/objects** | `AvoidancePlanner` (reactive depth) + optional `GlobalMapper` + `RRTStarPlanner` for replan when stuck. | Wire avoidance as a *deviation* layer on top of waypoints, not as the primary planner. |
| **D. Detect + log barrels (yellow/red), count each once** | `Detector.py` already runs threaded YOLO. | New: project bbox center → world NED via depth, 1 m grid-cell dedup, persisted log file. Plus a custom-trained YOLO with `yellow_barrel`, `red_barrel` classes. |

---

## 2. Architecture (proposed)

```
                 ┌─────────────────────────────────────────────────────────┐
                 │             qualifier_run.py  (new supervisor)          │
                 │  try: run_mission()                                     │
                 │  except: log + restart  (10-min wall clock continues)   │
                 └────────────────────────────┬────────────────────────────┘
                                              │
                              ┌───────────────┴───────────────┐
                              ▼                               ▼
                  ┌────────────────────────┐      ┌──────────────────────────┐
                  │  mission_loop (async)  │      │  detection_loop (async)  │
                  │                        │      │                          │
                  │  Drone (drone_control) │      │  Detector (YOLO threaded)│
                  │       │                │      │       │                  │
                  │       │ pose ◄──── SharedState ◄──────┤ pose at frame ts │
                  │       │                │      │       │                  │
                  │  coverage_waypoints ───┤      │   for each detection:    │
                  │       │                │      │     - read depth at bbox │
                  │       │                │      │     - project to NED     │
                  │  depth_receiver ──┐    │      │     - barrel_log.add()   │
                  │       │           │    │      │       (1 m dedup, class) │
                  │       ▼           ▼    │      │                          │
                  │  AvoidancePlanner.deviate │  │   barrel_log shared (lock)│
                  │  (if blocked, sidestep or  │  │                          │
                  │   call RRT* replan)        │  │                          │
                  │       │                    │  └──────────────────────────┘
                  │       ▼                    │
                  │  Drone.send_position_ned() │
                  └────────────────────────────┘
```

Two cooperating asyncio tasks:
- **mission_loop** — owns flight: waypoint sequencing + reactive deviation.
- **detection_loop** — owns perception: YOLO + barrel logging.

They communicate via the existing `SharedState` (pose) plus a new lock-protected `BarrelLog`.

---

## 3. Coverage strategy

### 3.1 Pattern: boustrophedon (lawnmower)

Why: arena is rectangular and known size (40×40 m, 10×10 grid of 4 m cells). Boustrophedon is provably complete coverage for rectilinear regions and is the standard for UAV mapping ([arxiv 1907.09224](https://arxiv.org/pdf/1907.09224)). The map is released the night before, so we can hand-tune entry corner + spacing offline.

### 3.2 Lane spacing

Camera FOV (intrinsic K = `[[433,0,320],[0,433,240],[0,0,1]]` for 640×480) → horizontal FOV ≈ 2·atan(320/433) ≈ **73°**. At a 2.5 m sensing range that's ~3.7 m of width per pass. Pick **3.5 m lane spacing** to overlap slightly. 40 m / 3.5 m ≈ **12 lanes**. With turns, ~480 m of flight; at conservative **0.8 m/s** ≈ 600 s = 10 min — *that's the entire run*. Too slow.

**Mitigation options** (pick during testing):
- (a) Push cruise speed to 1.5 m/s on straight segments → ~5 min for full sweep, leaves 5 min for retries / red-barrel chase.
- (b) Widen lanes to 5 m, accept ~25 % missed coverage edges. Combine with detection-driven re-visits.
- (c) Run two altitudes interleaved: 1.0 m for yellow (depth-down), 3.5 m for red (depth-fwd looking up). Each altitude is 6 lanes wide-spaced at 6 m. Risk: still misses small barrels.

### 3.3 Recommended: option (a) — speed up

`max_speed=1.0` in `AvoidancePlanner` is a software-side cap; PX4 can do more. Raise the cap to **1.5 m/s** for straight segments, throttle back to 0.5 m/s only when `clearance < safe_distance`. Keep one altitude (~2.5 m) and rely on camera vertical FOV to catch both yellow (below) and red (above-level) barrels. *This needs a vertical-FOV check before committing.*

### 3.4 Waypoint generator

New module: `coverage.py` → `generate_lawnmower(corner, width, height, spacing, altitude) -> List[(n, e, d, yaw)]`. Pure function. ~30 lines. Output consumed by `mission_loop`.

---


## 4. Avoidance integration

`AvoidancePlanner` today is **the** planner — it dictates direction. For coverage we need it as a **deviation** layer:

```
target_setpoint = next_waypoint
if AvoidancePlanner.compute_position_ned(depth, pose, K).blocked:
    # local sidestep
    target_setpoint = AvoidancePlanner.compute_position_ned(...).clear_direction
    # remember we deviated; resume waypoint after N seconds clear
elif clearance < safe_distance:
    # slow down but keep heading
    speed *= clamp(clearance / safe_distance, 0.3, 1.0)
```

Escalation if reactive can't clear in ~3 s:
1. Push the current depth frame into `GlobalMapper` (already in repo).
2. Call `RRTStarPlanner.plan(pose, next_waypoint, mapper.get_global_points())`.
3. Follow first 2–3 RRT* waypoints, then resume coverage.

Existing functions reused: `AvoidancePlanner.compute_position_ned`, `AvoidancePlanner.emergency_override`, `GlobalMapper.update_frame`, `GlobalMapper.get_global_points`, `RRTStarPlanner.plan`. **Zero new planner code.**

---

## 5. Detection → world coordinates

This is the part the existing code does *not* do at all.

### 5.1 Pipeline

For each detection frame from `Detector.py`:

1. Detector callback emits `(bbox, class_id, conf, frame, ts)`.
2. Look up the **depth frame** captured nearest to `ts` (`depth_receiver.get_frame()` — already thread-safe).
3. Look up the **pose at `ts`** from `SharedState` (already thread-safe).
4. Take depth `Z = depth[v, u]` at bbox center `(u, v)`. Use median over a 5×5 patch to denoise.
5. Back-project: with K, body-frame point `[X, Y, Z] = K⁻¹ · [u·Z, v·Z, Z]`.
6. Rotate by drone yaw, translate by drone NED position → world NED point.
7. Push into `BarrelLog.add(class, ned_point, conf, ts)`.

### 5.2 Dedup

`BarrelLog` keeps a dict keyed by `(class, round(n / 1.0), round(e / 1.0))`. First-add scores the barrel. Repeat sightings only update confidence/last-seen. 1 m bucket is conservative — barrels are ~0.6 m diameter and pose drift over 10 min could exceed 1 m, so we may need to widen later (2 m) once we know drift behavior.

### 5.3 Persistence

Log to `runs/<timestamp>/barrels.csv` every sighting + every dedup hit. Cheap, lets us recover state after a crash + restart (supervisor reads existing file on startup).

### 5.4 Reuse map

`PointCloudPlanner` (KDTree on 2D points) is exactly what we need for nearest-neighbor lookup of an already-detected barrel. Reuse its KDTree to make dedup O(log n) instead of dict scan when the log grows.

---

## 6. Supervisor (crash-restart)

Judges don't stop the clock for code crashes. Wrap the whole mission in:

```python
async def supervisor():
    deadline = time.time() + 600    # 10 min wall clock
    while time.time() < deadline:
        try:
            await run_mission(deadline_remaining=deadline - time.time())
        except Exception as e:
            log_exception(e)
            await asyncio.sleep(1)
            # mission_loop must be re-entrant: BarrelLog persisted, pose re-read,
            # coverage_waypoints filtered to "not yet visited"
```

Re-entrancy requirements:
- `BarrelLog` reads `barrels.csv` on construct.
- `coverage_waypoints` skips waypoints already within 1 m of current pose (assume we got there before crash).
- `Drone.connect()` is idempotent enough — verify `wait_until_ready()` in `drone_control_new.py` handles re-connect cleanly.

---

## 7. YOLO training (offline, before competition)

Existing `Train_YOLO_Models_new.ipynb` is the ready pipeline. Tasks:

1. Capture training data using `save_photo.py` + `gzphotodetectorsaver.py` (after its `msg`-scope bug is fixed) — fly the drone manually in Gazebo, save frames.
2. Label in [Roboflow](https://roboflow.com) or `labelImg`. Two classes: `yellow_barrel`, `red_barrel`. Target ≥200 images per class, varied angles + lighting.
3. Train on Linux CUDA VM (this is what the user's CUDA torch is for): `!yolo detect train data=data.yaml model=yolo11s.pt epochs=80 imgsz=640`.
4. Export `best.pt`, drop into `Codes/`, point `Detector.py` model path at it.

Notebook is Colab-ready but a local CUDA VM is faster and avoids Colab session timeouts.

---

## 8. Module map (what I'd build vs reuse)

| New file (proposed) | Purpose | Approx LOC |
|---|---|---|
| `coverage.py` | Lawnmower waypoint generator | 50 |
| `barrel_log.py` | Dedup + CSV persistence + KDTree lookup | 80 |
| `detection_to_world.py` | bbox+depth+pose → NED | 40 |
| `qualifier_run.py` | Supervisor + mission_loop + detection_loop wiring | 150 |

| Reused as-is | |
|---|---|
| `drone_control_new.py` | flight wrapper |
| `get_position_with_task.SharedState` | pose telemetry |
| `depth_receiver.py` | depth stream |
| `Detector.py` | YOLO worker pool |
| `AvoidancePlanner.py` | reactive deviation (use as library, not main loop) |
| `GlobalMapper_new.py` | only invoked on RRT* fallback |
| `RRTStarPlanner.py` | only invoked on RRT* fallback |
| `PointCloudPlanner_new.py` | KDTree helper for BarrelLog |

| Retired for the scored run | |
|---|---|
| `avoid.py` / `avoid_with_detect.py` | replaced by `qualifier_run.py` |
| `keyboardcontrol.py` | DQ if invoked |
| `go_to.py` | GPS goto unusable |
| `vel_avoidance.py` | superseded |
| `gzphotodetectorsaver.py` | replaced by `qualifier_run.py` detection_loop |

---

## 9. Risks & open questions (must resolve before coding)

### R1 — Pose source under GNSS-denied 🔴 **highest risk**
PX4 EKF2 in GNSS-denied mode needs an external pose source (vision_odometry, mocap, or LPE-from-VIO) ([PX4 doc](https://docs.px4.io/main/en/advanced_config/gnss_degraded_or_denied_flight)). The current `drone_control.py` reads `telemetry.position_velocity_ned()` and assumes it's valid. If the Gazebo `roboverse` world does **not** publish vision_odometry, the NED position drifts unboundedly with IMU integration — the whole strategy collapses.

**Action**: before any planning code, fly `basic_offboard.py` with PX4 GPS disabled (`param set GPS_1_CONFIG 0`) and watch whether `pose.north / east` stays bounded over 60 s. If it drifts, we have to set up a vision-odometry source (PX4 supports MAVLink `ODOMETRY` msg from an external VIO node).

### R2 — Vertical FOV for red barrels
The 433 px focal length on a 480 px image gives vertical FOV ≈ 2·atan(240/433) ≈ **58°**. At 2.5 m altitude, that's ±1.4 m of vertical sight at 2.5 m range. Red barrels "not on ground level" — how high? If they're at 4–6 m on shelves, a 2.5 m alt sweep misses them.

**Action**: ask judges or check sample world. If red is above 4 m, plan a second-altitude pass at 5 m.

### R3 — Drift vs 1 m dedup bucket
1 m bucket may falsely double-count if drift between two sightings of the same barrel exceeds 1 m. Mitigation: widen bucket to 2 m, accept loss of distinguishing two barrels within 2 m of each other. The arena has 4 m grid spacing, so 2 m bucket is safe.

### R4 — PX4 SITL on Mac + Gazebo Harmonic
PX4 SITL + GZ Harmonic has known issues on macOS ([PX4 forum](https://discuss.px4.io/t/47829)). Development might need Linux earlier than expected.

### R5 — `avoid.py` == `avoid_with_detect.py`
Detection isn't wired in. This is the qualifier blocker — the whole point of `qualifier_run.py` is to close that gap.

### R6 — Coverage time budget vs speed
At 0.8 m/s full lawnmower ≈ 10 min = full run. No slack. Either raise speed (depends on avoidance reliability) or accept partial coverage with detection-driven biasing (e.g. "if a red barrel was seen but not yet *fully* localized, return after pass").

---

## 10. Phased build order

If we go ahead with this, the smallest-risky-step sequence is:

1. **Verify R1 (pose drift in GNSS-denied SITL)** — 30 min of testing, no code changes. *Blocks everything else.*
2. **Train barrel YOLO** — can happen in parallel on CUDA VM while flight code is built.
3. **Build `barrel_log.py` + `detection_to_world.py`** — pure logic, unit-testable with synthetic depth + pose.
4. **Build `coverage.py`** — pure logic, plot output with matplotlib to verify lanes.
5. **Build `qualifier_run.py` minus supervisor** — single-attempt mission. Test in Gazebo end-to-end with stub barrels.
6. **Wire avoidance deviation + RRT* fallback** — incremental, gated by clearance metric.
7. **Add supervisor wrapper** — last (least risky failure mode is "supervisor crashes too" → no worse than step 5).
8. **Time trials in sim** — measure speed vs detection rate, tune lane spacing + cruise speed.

---

## 11. What I'd want to talk through

Before writing any code I'd want your call on:

1. **Pose source plan** — has the Brainhack `roboverse` world been verified to publish vision_odometry / ARK localization / mocap? If not, what's plan B (run an external VIO node like ORB-SLAM3, or accept dead-reckoning + visual landmarks)?
2. **Red-barrel altitude** — do we know roughly how high "not on ground level" means?
3. **Camera mounting** — fixed forward, or gimballed? The IMX214 topic name suggests fixed. If fixed, vertical sweep is the only way to scan up/down.
4. **Allowed compute on competition rig** — does the venue's GPU run our YOLO at ≥15 FPS, or do we need to downscale to YOLOv10n / 320 input?
5. **Whether to use `avoid.py` / `avoid_with_detect.py` as the basis for `qualifier_run.py`** or write a clean file. My vote: clean file, but reuse every utility import.

Once these are settled I'll write a more concrete implementation plan and we go module-by-module.
