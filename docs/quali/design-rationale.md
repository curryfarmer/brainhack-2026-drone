# Design rationale, competition rules, risks & open questions

This document began life as the team's pre-implementation solution approach (the former `APPROACH.md`), written **before** any qualifier code existed. It is preserved here near-verbatim because it explains *why* the stack is shaped the way it is — and it has been annotated with **(as built: …)** notes wherever reality caught up with the plan. For what the code actually does today, see the [codebase guide](codebase.md); to run it, see the [deployment guide](deployment.md).

> **TL;DR — Recommended approach**
>
> **Pre-baked coverage waypoints + reactive depth avoidance + multi-altitude sweep + bbox-to-world barrel logger with spatial dedup, wrapped in a crash-restart supervisor.**

Reuses ~80% of the existing code as-is. Only new code: (a) coverage waypoint generator, (b) detection-to-world projection + dedup store, (c) supervisor. The "main loop" replaces `avoid.py` / `avoid_with_detect.py` (which are currently the same file). *(As built: this plan was executed as `qualifier_run.py` + `coverage.py` + `detection_to_world.py` + `barrel_log.py`. A second, independently written entry point — `qualifier_main.py`, a DFS cell-grid explorer — was added later by a teammate; both are covered in the [codebase guide](codebase.md).)*

---

## Competition brief (RoboVerse 2026 qualifiers)

This is the single home for the qualifier rules as we understood them.

- **Mission**: detect yellow (ground) + red (elevated) fuel barrels in 40×40×8 m space port. 10-min runs, best of multiple attempts.
- **Scoring**: yellow 50 pt, red 100 pt, +20 pt per 30 s under 5 min for full-class sweeps.
- **DQ rules**: no keyboard/joystick during scored run — **disable `keyboardcontrol.py`** in any run script.
- **GNSS-denied**: all pose comes from PX4 EKF2 fused with vision/odometry. `go_to.py` (GPS goto) is useless. Position drift over 10 min is the silent killer — see [§9 Risks](#9-risks).
- **Multi-altitude search needed**: red barrels are elevated → single low pass misses them. Either two altitude passes or wide vertical FOV.
- **No clock stop on crash**: 10-min wall clock keeps running even if code dies → main loop must restart on exception (supervisor).
- **Map released 1 day prior** — coverage waypoints can be precomputed the night before.

---

## 1. Problem decomposition

The qualifier reduces to four sub-problems. Map each to existing code where possible.

| Sub-problem | Existing tool | Gap |
|---|---|---|
| **A. Localize without GPS** | PX4 EKF2 fuses IMU + vision_odometry. `get_position_with_task.SharedState` reads NED telemetry. | Need to verify the Gazebo `roboverse` world actually provides vision_odometry — see [§9 Risks](#9-risks). If not, drift will be unrecoverable. |
| **B. Sweep the 40×40 m space port** | nothing — `avoid.py` does grid-direction flips reactively but doesn't cover area. | New: coverage waypoint generator (boustrophedon). *(As built: `coverage.py`.)* |
| **C. Don't crash into walls/objects** | `AvoidancePlanner` (reactive depth) + optional `GlobalMapper` + `RRTStarPlanner` for replan when stuck. | Wire avoidance as a *deviation* layer on top of waypoints, not as the primary planner. |
| **D. Detect + log barrels (yellow/red), count each once** | `Detector.py` already runs threaded YOLO. | New: project bbox center → world NED via depth, 1 m grid-cell dedup, persisted log file. Plus a custom-trained YOLO with `yellow_barrel`, `red_barrel` classes. *(As built: `detection_to_world.py` + `barrel_log.py`.)* |

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

*(As built: this is exactly the structure of `qualifier_run.py` — supervisor + mission_loop + detection_loop. See [As-built algorithm](#as-built-algorithm-qualifier_runpy) below for the step-by-step walkthrough.)*

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

`max_speed=1.0` in `AvoidancePlanner` is a software-side cap; PX4 can do more. Raise the cap to **1.5 m/s** for straight segments, throttle back to 0.5 m/s only when `clearance < safe_distance`. Keep one altitude (~2.5 m) and rely on camera vertical FOV to catch both yellow (below) and red (above-level) barrels. *This needs a vertical-FOV check before committing.* *(As built: `MissionConfig` in `qualifier_run.py` defaults to `cruise_speed_mps=1.2` and `cruise_altitude_m=2.5` — a compromise between (a) and the original 0.8 m/s; tune during time trials.)*

### 3.4 Waypoint generator

New module: `coverage.py` → `generate_lawnmower(corner, width, height, spacing, altitude) -> List[(n, e, d, yaw)]`. Pure function. ~30 lines. Output consumed by `mission_loop`. *(As built: `coverage.py` exists with `generate_lawnmower()` returning `Waypoint(north, east, down, yaw_deg, is_turn)` plus a `filter_unvisited()` helper for supervisor restarts — somewhat more than 30 lines, but still pure and unit-testable.)*

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

*(As built: the deviation layer landed in `_go_to_waypoint()` inside `qualifier_run.py`. The RRT\* escalation was never wired in — it remains a stub; see [Extending & roadmap](#extending--roadmap).)*

---

## 5. Detection → world coordinates

This is the part the existing code does *not* do at all. *(As built: `detection_to_world.py`.)*

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

`BarrelLog` keeps a dict keyed by `(class, round(n / 1.0), round(e / 1.0))`. First-add scores the barrel. Repeat sightings only update confidence/last-seen. 1 m bucket is conservative — barrels are ~0.6 m diameter and pose drift over 10 min could exceed 1 m, so we may need to widen later (2 m) once we know drift behavior. *(As built: `barrel_log.py` uses a radius match, not a grid bucket, and defaults to `dedup_radius=2.0` — the wider value this paragraph predicted we'd need.)*

### 5.3 Persistence

Log to `runs/<timestamp>/barrels.csv` every sighting + every dedup hit. Cheap, lets us recover state after a crash + restart (supervisor reads existing file on startup). *(As built: `barrel_log.py` writes the CSV atomically — `.tmp` + `os.replace` — and `autoload=True` re-reads it on construct.)*

### 5.4 Reuse map

`PointCloudPlanner` (KDTree on 2D points) is exactly what we need for nearest-neighbor lookup of an already-detected barrel. Reuse its KDTree to make dedup O(log n) instead of dict scan when the log grows.

---

## 6. Supervisor

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

*(As built: `supervisor(cfg)` in `qualifier_run.py` implements exactly this — long-lived singletons survive across attempts, `BarrelLog` autoloads its CSV, and `coverage.filter_unvisited()` drops already-reached waypoints.)*

---

## 7. YOLO training

The original plan was a Colab notebook flow; that has been **superseded** by a local end-to-end pipeline — see [training-pipeline.md](training-pipeline.md) for the actual collect → label → validate → split → train → deploy workflow.

What survives from the original plan is the data-capture rationale: capture frames by flying the drone in Gazebo, label two classes (`yellow_barrel`, `red_barrel`) in Roboflow or labelImg, and target **≥200 images per class with varied angles and lighting** — anything less and the detector will not generalize across the arena.

---

## 8. Module map

*Historical note: the tables below were a proposal when written. All four "new" files were subsequently built under the proposed names, so the build column is now an accurate inventory. `qualifier_main.py` (a teammate's DFS cell-grid explorer, outputs `barrels.json` + `visited_cells.json`) arrived later as a parallel alternative main — it was not part of this plan.*

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

*(As built: the retirement happened — `avoid.py` is no longer the "current best" entry point. The two mission mains are `qualifier_run.py` and `qualifier_main.py`; see the [codebase guide](codebase.md).)*

---

## 9. Risks

Original risk register, kept verbatim, with a one-line current status added where the outcome is known.

### R1 — Pose source under GNSS-denied (highest risk)

PX4 EKF2 in GNSS-denied mode needs an external pose source (vision_odometry, mocap, or LPE-from-VIO) ([PX4 doc](https://docs.px4.io/main/en/advanced_config/gnss_degraded_or_denied_flight)). The current `drone_control.py` reads `telemetry.position_velocity_ned()` and assumes it's valid. If the Gazebo `roboverse` world does **not** publish vision_odometry, the NED position drifts unboundedly with IMU integration — the whole strategy collapses.

**Action**: before any planning code, fly `basic_offboard.py` with PX4 GPS disabled (`param set GPS_1_CONFIG 0`) and watch whether `pose.north / east` stays bounded over 60 s. If it drifts, we have to set up a vision-odometry source (PX4 supports MAVLink `ODOMETRY` msg from an external VIO node).

**Status:** still open — the pose-drift sanity check remains on the pre-run checklist in the [deployment guide](deployment.md).

### R2 — Vertical FOV for red barrels

The 433 px focal length on a 480 px image gives vertical FOV ≈ 2·atan(240/433) ≈ **58°**. At 2.5 m altitude, that's ±1.4 m of vertical sight at 2.5 m range. Red barrels "not on ground level" — how high? If they're at 4–6 m on shelves, a 2.5 m alt sweep misses them.

**Action**: ask judges or check sample world. If red is above 4 m, plan a second-altitude pass at 5 m.

**Status:** still open — see [Open questions](#open-questions); the two-altitude pass is a documented 3-line extension.

### R3 — Drift vs 1 m dedup bucket

1 m bucket may falsely double-count if drift between two sightings of the same barrel exceeds 1 m. Mitigation: widen bucket to 2 m, accept loss of distinguishing two barrels within 2 m of each other. The arena has 4 m grid spacing, so 2 m bucket is safe.

**Status:** addressed — `barrel_log.py` shipped with `dedup_radius=2.0` as the default.

### R4 — PX4 SITL on Mac + Gazebo Harmonic

PX4 SITL + GZ Harmonic has known issues on macOS ([PX4 forum](https://discuss.px4.io/t/47829)). Development might need Linux earlier than expected.

**Status:** plan SITL work on Linux; install steps live in the [deployment guide](deployment.md) and the sim dev loop in [simulator-testing.md](simulator-testing.md).

### R5 — `avoid.py` == `avoid_with_detect.py`

Detection isn't wired in. This is the qualifier blocker — the whole point of `qualifier_run.py` is to close that gap.

**Status:** resolved — detection is wired into both `qualifier_run.py` and `qualifier_main.py`.

### R6 — Coverage time budget vs speed

At 0.8 m/s full lawnmower ≈ 10 min = full run. No slack. Either raise speed (depends on avoidance reliability) or accept partial coverage with detection-driven biasing (e.g. "if a red barrel was seen but not yet *fully* localized, return after pass").

**Status:** partially addressed — `cruise_speed_mps` defaults to 1.2 in `MissionConfig`; final tuning needs sim time trials.

---

## 10. Build order (historical)

This was the planned smallest-risky-step sequence. It has largely been executed (steps 3–5 and 7 are done; step 6's RRT* fallback was never wired; steps 1, 2 and 8 remain live work). Kept for reference:

1. **Verify R1 (pose drift in GNSS-denied SITL)** — 30 min of testing, no code changes. *Blocks everything else.*
2. **Train barrel YOLO** — can happen in parallel on CUDA VM while flight code is built.
3. **Build `barrel_log.py` + `detection_to_world.py`** — pure logic, unit-testable with synthetic depth + pose.
4. **Build `coverage.py`** — pure logic, plot output with matplotlib to verify lanes.
5. **Build `qualifier_run.py` minus supervisor** — single-attempt mission. Test in Gazebo end-to-end with stub barrels.
6. **Wire avoidance deviation + RRT* fallback** — incremental, gated by clearance metric.
7. **Add supervisor wrapper** — last (least risky failure mode is "supervisor crashes too" → no worse than step 5).
8. **Time trials in sim** — measure speed vs detection rate, tune lane spacing + cruise speed.

---

## As-built algorithm (qualifier_run.py)

This is the best end-to-end description of what `qualifier_run.py` actually does. (The alternative entry point `qualifier_main.py` — DFS exploration over a cell grid — is described in the [codebase guide](codebase.md).)

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
- No global RRT* fallback when reactive planner stays blocked >3s (stubbed only — see [Extending & roadmap](#extending--roadmap) below).
- Single altitude → red elevated barrels may be missed. Two-altitude concat is a 3-line change.
- `yolov10n.pt` is COCO — barrel-trained weights still needed. *(Update: a trained `best.pt` (6.2 MB) now sits at repo root, but `model_config.json` still points at the COCO `yolov10n.pt` placeholder — verify which weights are actually wired in before any scored run.)*
- Pose drift under GNSS-denied EKF2 is unbounded without vision/mocap fusion (silent killer over 10 min).

---

## Extending & roadmap

### Add a new barrel class
1. Train YOLO with the new class (see [training-pipeline.md](training-pipeline.md)).
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
Stubbed in [§4 Avoidance integration](#4-avoidance-integration) above. Build into `_go_to_waypoint`:
- Track time-since-deviation.
- If > 3 s of `blocked=True`, push frame into `GlobalMapper.update_frame()`.
- Call `RRTStarPlanner.plan(current_pose, target, mapper.get_global_points())`.
- Follow the first 2–3 returned waypoints, then resume coverage.

### Replay / bench mode
Recommended addition: record `(timestamp, pose, depth_frame, rgb_frame)` tuples to disk during a real flight (or in sim), then replay them through `detection_loop`'s callback path without flying — useful for iterating on YOLO + projection math without burning sim time. Not yet built.

### Waypoint generation today: pre-baked lawnmower

`coverage.generate_lawnmower(origin_north, origin_east, width_north, width_east, altitude, lane_spacing=3.5, along_axis="north")` returns a list of `Waypoint(north, east, down, yaw_deg, is_turn)` covering a rectangular region in a boustrophedon (snake) sweep. The number of lanes is `ceil(|width_east| / lane_spacing) + 1`, and every other lane reverses direction so the drone walks back and forth without dead heads.

Generation happens **once** at the start of `qualifier_run.mission_loop()`. The list is then handed to `_go_to_waypoint(...)` one element at a time. The drone is considered "at" a waypoint when its NED distance is below `cfg.waypoint_radius_m` (default 0.8 m). On supervisor restart, `coverage.filter_unvisited(...)` drops the leading waypoints we already reached so we don't re-fly completed lanes.

In other words, **the waypoint list itself is static**. What is "dynamic" is the *path between* waypoints: while flying toward a target, `qualifier_run` keeps the AvoidancePlanner (`AvoidancePlanner.py`) in the loop. If the depth stream sees an obstacle, the planner deviates the velocity command to clear it, then re-targets the original waypoint once free. The original list never gets regenerated.

### Dynamic waypoint generation (roadmap)

If you want the *list* to change at runtime, two natural extensions:

- **Frontier-based exploration.** After every detection or every N seconds, look at the occupancy grid (`GlobalMapper.py`) and re-issue waypoints at the boundary of unexplored cells. Replace `generate_lawnmower(...)` in `mission_loop` with a call into a `frontier_next(map)` helper that returns the next K waypoints. The mission loop already consumes the list one element at a time so swapping the source is a single-line change.
- **Re-targeting around detections.** When a barrel is logged in `barrel_log`, generate a tighter sweep around it (e.g. four waypoints at 1.5 m radius). Insert those at the head of the queue. Easiest is a `collections.deque` instead of a `list` so you can `appendleft(...)` without reshuffling.

Neither is implemented inside `qualifier_run.py` today. However, `qualifier_main.py` (added later, independently) implements a related idea: DFS exploration over grid cells, tracking visited cells at runtime instead of pre-baking a route. See the [codebase guide](codebase.md) for how it differs.

---

## Open questions

Merged from the original approach doc, the qualifier manual, and the training-pipeline handover. One line of status per item where determinable.

1. **Pose source under GNSS-denied** — verify EKF2 has a vision/mocap fix; the whole stack assumes `telemetry.position_velocity_ned()` is bounded (see [R1](#r1--pose-source-under-gnss-denied-highest-risk)). *Status: open — run the pose-drift check from the [deployment guide](deployment.md) checklist before any scored attempt.*
2. **Red-barrel altitude** — how high is "not on ground level"? Drives the single- vs two-altitude decision (see [R2](#r2--vertical-fov-for-red-barrels)). *Status: open — the judges' map (released 1 day prior) will answer it.*
3. **Camera mounting** — fixed forward, or gimballed? The IMX214 topic name suggests fixed. If fixed, vertical sweep is the only way to scan up/down. *Status: presumed fixed; unconfirmed.*
4. **Competition rig GPU** — verify YOLO runs at ≥10 FPS with the chosen weights at `--device cuda`. If not, drop to a `yolov10n.pt`-class architecture and 320 input resolution. *Status: open.*
5. **RGB topic string** — the default in `MissionConfig.rgb_topic` is a guess from the camera-related files; confirm against the actual `gz topic -l` of the released world. *Status: open.*
6. **YOLO training data + labelling** — capture from manual flight, label `yellow_barrel` / `red_barrel`, ≥200 images per class. *Status: pipeline built (see [training-pipeline.md](training-pipeline.md)); a `best.pt` exists at repo root but `model_config.json` still points at the COCO placeholder (the `Detector.py` `config_path` patch, regressed May 22, was restored 2026-06-06) — verify the wiring.*
7. **Sim-to-real transfer** — does sim-captured training data transfer to real cameras? Augmentation is tuned for it but not validated. *Status: open.*
8. **Whether the Colab notebook path is still needed** — `train_yolo.py` does the same job locally. *Status: effectively superseded; keep the notebook only as a fallback.*
9. **`avoid.py` as the basis for the mission main, or a clean file?** *Status: resolved — clean file (`qualifier_run.py`) reusing the utility imports; `qualifier_main.py` later arrived as a second clean main.*

---

## References

- PX4 EKF2 GNSS-denied: <https://docs.px4.io/main/en/advanced_config/gnss_degraded_or_denied_flight>
- PX4 external (vision) pose: <https://px4.gitbook.io/px4-user-guide/robotics/ros/ros1/external_position_estimation>
- PX4 EKF2 tuning: <https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf>
- Boustrophedon coverage planning (arxiv 1907.09224): <https://arxiv.org/pdf/1907.09224>
- Incremental UAV coverage in unknown area: <https://journals.sagepub.com/doi/10.1177/17568293241262323>
