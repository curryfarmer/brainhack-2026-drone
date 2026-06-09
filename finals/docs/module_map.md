# finals/ module map — START HERE each session

One session = one row of the roadmap below. Read this file + the named source
examples for your session; you should not need to read the whole repo.

Mission context: DSTA BrainHack 2026 finals, **swarm challenge only**. All
code runs on the C2 laptop; 3× HULA drones over Wi-Fi via pyhulax; detection
on laptop-streamed frames; moving RoboMaster convoy; valid/invalid landing
pads; ONE 2-hour onsite hardware window. Full background:
[`docs/finals/README.md`](../../docs/finals/README.md).

## How to run / test

```bash
pytest finals/tests                          # zero hardware, always green
python -m finals.main --profile mock --dry-run   # resolve + print the plan
python -m finals.main --profile replay           # frames -> sightings.csv (S7; needs cv2)
python -m finals.main --profile sitl --phases takeoff_demo   # VM (from S4+S6)
python -m finals.main --profile sitl --config finals/configs/sitl3.json  # 3× swarm (SIM-2)
python -m finals.main --profile bench --preflight-only   # S10 onsite bench tool: P0–P9, props off
python -m finals.tools.replay_plot <run_dir> [--save out.png]   # DR-replay tracks (SIM-2)
```

Onsite 2-hour window runbook (preflight P0–P10, bench B1–B8, gates A–G, hard
rules): [`docs/finals/onsite_test_plan.md`](../../docs/finals/onsite_test_plan.md).

Vision deps note (S7): the test suite stays green WITHOUT cv2/numpy (vision
tests skip; everything else runs); `--profile replay` and the cv2-gated
tests need `pip install opencv-python "numpy<2"` (already in requirements.txt).

Profiles: `mock` (pure logic) | `sitl` (qualifier PX4 SITL + Gazebo VM) |
`replay` (disk frames + detector, 0 drones) | `bench` (real drones, props off,
flight REFUSED) | `real` (gated 3×: CLI flag → preflight P0–P10 → operator GO).

Simulation strategy — what is/isn't simulable, tier recipes, rejected
alternatives: [`simulation.md`](simulation.md). (PX4 SITL is a physics
stand-in behind the FlightAdapter seam; the finals drones are HULA/pyhulax.)
Sim build-out ladder — SIM-0…SIM-5, one fresh session each, smoke gates +
ready-to-paste handover prompts + evidence log: [`sim_sessions.md`](sim_sessions.md).
SIM-1/2 execute S6; SIM-3/4/5 execute S8.

## Binding conventions (every session re-reads these)

1. **No bare `except`.** `except Exception` ONLY in `guards.py` (SafetyController
   safe-down, guard-evaluation wrapper), `mission/orchestrator.py` (top
   loop), — widened S7, reviewed — `vision/detector.py` (the vendored
   worker pool must survive ARBITRARY model/callback exceptions; the root
   Detector.py worker died SILENTLY on them), and — widened S6/SIM-1,
   reviewed — `flight/sitl_adapter.py` (three sites: emergency_land
   per-step + disconnect teardown, whose never-raise contracts a typed
   tuple cannot honor, and the telemetry-stream wrapper — MAVSDK streams
   END SILENTLY when PX4 dies and the wrapper converts that into a loud
   dead-flag) — always logged with traceback. Enforced by
   `tests/test_conventions.py`.
2. Every blocking/awaited op takes `timeout_s` and raises a typed
   `finals.errors` exception with an ACTIONABLE message (what, which drone,
   how long, what to check).
3. Every `while` loop references a deadline, stop event, or iteration bound
   (the `mapping_drone.py:129` infinite-wait bug class).
4. No module-level mutable globals (the `battery_remain` bug class).
   Sanctioned exception: `PHASE_REGISTRY`.
5. Stubs raise `NotImplementedError("finals.<module>: session N — see finals/docs/module_map.md")`.
6. Units in names: `_cm`, `_m`, `_deg`, `_s`. The FlightAdapter contract is in
   **cm** (pyhulax convention); backends convert internally.
7. Every module docstring names its proven source file and the bugs fixed in
   adaptation. **Official examples are audited line-by-line, never copied** —
   the verified bug list lives in the docstrings and `docs/finals/README.md`.
8. SDK imports live INSIDE the backend module that needs them (see
   `tests/test_conventions.py::SDK_ALLOWED`); pure modules stay stdlib(+numpy).
9. Root qualifier files (`drone_control.py`, `get_position_with_task.py`,
   `Detector.py`, …) are imported or vendored-with-fixes — NEVER edited.

## Status table

| Module | Status | Session | Derives from |
|---|---|---|---|
| `types.py` | ✅ implemented | S1 | pyhulax docs (Direction/units); hula_connection.py:46–50 (Action vocabulary) |
| `errors.py` | ✅ implemented | S1 / **S11 (NAV-1)** | failure-mode audit of the official examples. **NAV-1: `PlanningError(FinalsError)` — "refuse to plan this transit" (goal/start inside an inflated keep-out, or no collision-free path); actionable WHAT/WHICH/WHY/CHECK raised from visibility_graph.plan** |
| `config.py` | ✅ implemented | S1 / **S11 (NAV-0, NAV-2)** | qualifier_run.py:72–132 MissionConfig; known-issue #9 weights guards. **NAV-0: optional top-level `arena_name` -> loads finals/configs/arenas/<name>.json into `FinalsConfig.arena` (ArenaMap); `arena` is DERIVED, never JSON; loud-on-missing/malformed. NAV-2: the SEMANTIC arena checks are now HARDENED in `ArenaMap.from_dict` (bounds ordering, pads/c2-origin within CLOSED bounds, unique pad/keep-out ids, >= 3-distinct-vertex keep-outs, non-finite reject) — `_resolve_arena` surfaces them; covered by `test_arena_config.py`** |
| `main.py` | ✅ implemented | S1 / S4 / S7 / **S8** / **S10** | qualifier_run.py parse_args/_amain; S4: bench wiring builds the INNER backend first (BenchAdapter special case); phases via the `from_config` soft convention; S7: `_run_replay` (the no-drone runner) + per-drone perception/VideoWatchdog wiring gated on `_WIRED_FRAME_BACKENDS` — deliberately NOT `frame_backend != "none"`: an unwired backend gets a watchdog with no frame source = guaranteed-false DEGRADE. **S8 (SIM-4): "gazebo" added to the set (one-per-line so parallel sessions add theirs as an isolated hunk) + a gazebo branch in `_build_perception` constructing GazeboRgbSource over the sim/gz_camera_bridge TCP seam — NO orchestrator changes** (sightings.csv appended at the PUBLISH site, perception.py). **S10: `_WIRED_FRAME_BACKENDS` adds "pyhulax" → (`replay`, `gazebo`, `pyhulax`); `_make_shared_pyhulax_api` creates ONE DroneAPI per drone fed to BOTH `_build_adapter` and `_build_perception` (same-link invariant); `_build_agents` extracted (shared by the mission + `--preflight-only` paths); `_amain` lets preflight OWN `connect` (P4) + `source.start` (P6) for bench/real and skips the generic start loop; `--preflight-only` → `_run_preflight_only` (bench/real only, ConfigError otherwise)**. **SIM-5: the gazebo branch resolves a PER-DRONE bridge port via `resolve_gazebo_video_port(cfg, drone_id)` (config.py, mirrors `resolve_sitl_endpoint`) instead of the single top-level port — multi-drone gazebo needs one bridge per drone.** |
| `mission/phase.py` | ✅ contract | S1 (exercised S4) | hula_connection.py:46–50, made pure |
| `mission/phases/__init__.py` | ✅ registry | S1 | — |
| `flight/adapter.py` | ✅ implemented (ABC + BenchAdapter) | S1 / S3 | pyhulax ∩ MAVSDK honest primitives; Bench wraps an inner adapter (S4 wiring needs a special case) |
| `vision/video.py` | ✅ implemented (ABC + ReplaySource) | S1 / S7 | qualifier_run.py:163–186 RgbReceiver contract (latest-frame-copy); ReplaySource: dir-of-png/jpg or video file, paced thread, `frame_number` = monotonic delivery counter (perception dedupe key), `exhausted`/`errored` surfaces; cv2 imported LAZILY (main resolves every backend for --dry-run on cv2-less machines) + injectable `loader` seam so contract tests run dep-free |
| `events.py` | ✅ implemented | S2 | barrel_log.py atomic-flush discipline; runs/<ts>/ convention |
| `sightings.py` | ✅ implemented | S2 | barrel_log.py lock discipline, inverted to append-only+fsync |
| `flight/mock_adapter.py` | ✅ implemented | S3 | — (the test double everything stands on; pipeline order + deviations in its docstring) |
| `flight/dead_reckon.py` | ✅ implemented | S3 | detection_to_world.py body→NED yaw math, reduced; yaw sign flipped to pyhulax CCW (psi_NED = -yaw_deg, documented + test-pinned) |
| `docs/simulation.md` | ✅ doc | — | feasibility research pass (2026-06-06), load-bearing claims verified against primary sources; PX4 SITL = physics stand-in, finals drones are HULA |
| `docs/sim_sessions.md` | ✅ doc | — | sim ladder + handover prompts + evidence log; design pass 2026-06-06 (source-verified: conventions-scan scope, drone_control.py unwrappable for multi-drone, lockstep-RTF risk) |
| `sim/` env scripts (`launch_sitl.sh`, `sitl_smoke.py`, `README.md`) | ✅ implemented | SIM-0 | repo root BY DESIGN — outside the conventions scan/SDK whitelist; raw-MAVSDK harness was sanctioned for env bring-up ONLY and is now banned for flight validation ([`sim_sessions.md`](sim_sessions.md) recap §4-5; VM evidence + probe verdicts in its SIM-0 evidence block) |
| `sim/` world assets (`gen_markers.py`, convoy model+world, `check_detection.py`) | ✅ | **SIM-3** (∥ S4–S7) | simulation.md Tier 2 recipes; markers BOTH ArUco + QR (20×20 cm intel); convoy driver = rclpy→ros_gz→VelocityControl (SIM-0 probe). VM render: ogre2 BLANK → use llvmpipe/ogre1 (sim_sessions SIM-3). ArUco decodes all sentry bands; QR not viable >1 m |
| `sim/` V2a/V2 assets (`gz_camera_bridge.py`, `px4_models/x500_mono_cam_640`, `worlds/convoy_px4.sdf`, `worlds/empty_cam.sdf`, `run_vision.sh`, `pty_q_harness.py`) | ✅ | **SIM-4/5** | bridge = system-py3.10 gz→TCP frame forwarder (mirrors check_detection.py; `--count-secs` stats mode for probe3); x500 + INLINED down-cam (HFOV 99.69, topic `…/x500_mono_cam_640_<i>/…/camera/image`); convoy_px4 = convoy + PX4 flight-sensor systems + GPS origin; `run_vision.sh` stageA/stageB (1 drone) + **SIM-5 launch3/probe3/stageB3 (+abort3/kill3 drills)** — PX4 OWNS the gz server → lockstep, spawn CLEAR of robot starts. **SIM-5 finding: the gz server is single-threaded = the lockstep master for all 3 PX4 instances; 3 cams @ 15 Hz throttle it to RTF 0.29 and only ~1 of 3 flights stays healthy (8 vCPUs don't help — not core-bound); cam `update_rate` 15→5 → RTF 0.86, all 3 fly.** `pty_q_harness.py` = pty so the abort key arms over ssh |
| `tools/replay_plot.py` | ✅ implemented | SIM-2 | feeds mission.jsonl through the REAL DeadReckoner (never reimplements the math); matplotlib-only — inside the conventions scan, cv2/gz forbidden here. Origin-seeded DR (boot yaw ≠ 0), per-drone subplots NEVER merged (each drone's north = its own boot heading), fixture-pinned closure + signed shoelace +1 m² chirality check |
| `configs/sitl3.json` | ✅ implemented | SIM-2 | sitl.json × 3 (UDP 14540/41/42, gRPC 50051–53, bands 1.2/1.7/2.2 = takeoff 120/170/220 cm); `frame_backend none` until SIM-5; THREE_DRONES schema test-pinned |
| `configs/mock_gazebo.json` | ✅ | **SIM-4** | Stage A de-risk: mock flight + gazebo frames (static tower cam) → sentry_scan → sightings.csv, zero PX4 |
| `configs/sitl_vision.json` | ✅ | **SIM-4** | gate V2a: 1 PX4 x500 + onboard 640×480 cam, sentry_scan (revolutions 2); `gazebo_video_host`/`gazebo_video_port` = the bridge endpoint |
| `configs/sitl3_vision.json` | ✅ flight-proven | **SIM-5** | sitl3 + gazebo frames + sentry_scan; per-drone `gazebo_video_port` 5600/5601/5602 (each drone reads its OWN onboard cam via its OWN bridge — `resolve_gazebo_video_port`; multi-drone gazebo port distinctness validated); `command_timeout_s` 90 / `mission_budget_s` 500 sized from the measured 3-cam RTF (`run_vision.sh probe3`). **SIM-5 FULL SIM DONE**: all 3 DONE, 5/5 moving ids by every drone, 4 drills PASS — [`sim_sessions.md`](sim_sessions.md) SIM-5 (VM) |
| `configs/arenas/sample.json` + `configs/mock_arena.json` | ✅ | **S11 (NAV-2)** | a realistic-shaped Challenge-2A arena (≈12×10 m bounds, 4 crate keep-outs, 5 pads = 3 green/2 red, 2 taped lanes, C2 origin+heading) for `test_arena_config.py` + a SITL/`--dry-run`; every value `_comment`-marked briefing/onsite-tunable; `pad_north` sits flush to the wall on purpose (exercises the CLOSED-bounds edge). `mock_arena.json` = a mock profile with `arena_name: "sample"` so the full arena load+validate path runs under `python -m finals.main --profile mock --config finals/configs/mock_arena.json --dry-run` |
| `mission/agent.py` | ✅ implemented | S4 | hula_connection.py:39–63 loop, formalized; mapping_drone.py watchdog gaps CLOSED: outer wait_for deadline on every command, telemetry-staleness check, emergency-land-EXACTLY-ONCE latch (shielded from cancellation), FAILED terminal — no auto-restart. Emits the `origin` + `action_complete` events (replay prereq). S7: injectable `frame_ts_fn` (-> GuardContext.last_frame_ts, the VideoWatchdog feed) + `on_degrade` hook (DEGRADE_DETECTION trips -> PerceptionLoop.shed) + read-only `last_telemetry` (perception's enrichment source) |
| `mission/orchestrator.py` | ✅ implemented | S4 | qualifier_run.py:407–513 supervisor MINUS auto-restart (unsafe on real aircraft); agents = independent asyncio tasks; budget stop + settle-grace hard deadline; 1 Hz atomic heartbeat; seq-cursor SightingBus drain; whitelisted blanket catches always log tracebacks |
| `mission/phases/takeoff_demo.py` | ✅ implemented | S4 | mapping_drone.py:343–355 intent, as relative moves (no UWB dependency); tunables via `zone["takeoff_demo"]` + `altitude_band_m` (`from_config`) |
| `guards.py` | ✅ implemented | S5 | qualifier_run.py:383–393 crash→land path; mapping_drone.py gap audit. Trip vocabulary ADVISORY→LAND_ALL (max severity wins); per-drone guards run in the AGENT loop, swarm guards on the orchestrator tick (stub-vs-S4 reconciliations in the module docstring); a raising guard IS a trip; SafetyController = landing slot + wall-clock-bounded retry ladder + completion-shared trip(); AbortListener thread ('q'+Enter → orderly land-all). VideoWatchdog WIRED in S7 (agent `frame_ts_fn` -> GuardContext.last_frame_ts; built by main only for `_WIRED_FRAME_BACKENDS`) |
| `flight/sitl_adapter.py` | ✅ implemented | **SIM-1** + **SIM-2** (3× swarm) | drone_control.py + get_position_with_task.py + qualifier_run.py:268–331 (all proven) — VENDORED-WITH-FIXES per recap §2 (never wrapped); fixes in the module docstring incl. blind-sleep→polled climb, yaw-snap-to-north hold fix, targeted stale-server cleanup (drill-proven), rotate via position-setpoint yaw (reviewed deviation), stream dead-flag (kill-drill detector), **SIM-2: offboard-start NO_SETPOINT_SET re-prime+retry — the 3× gRPC contention race the single-drone V1 never tripped** (`offboard_start_tries`, deadline-bounded); V1 + 3× gate + kill/abort drills: [`sim_sessions.md`](sim_sessions.md) SIM-1/SIM-2 evidence; per-drone endpoints via config `sitl_address`/`mavsdk_grpc_port` + `resolve_sitl_endpoint` |
| `vision/aruco.py` | ✅ implemented | S7 | **PRIMARY detector** (convoy robots carry markers — detect + read ID, no training). potential_detection_targets.py:5–30 (audited, not copied — its `detectMarkers` unpack is a SYNTAX ERROR; three values, fixed). Pluggable seam: `make_marker_detector(cfg.marker_backend)` -> "aruco" (DICT_6X6_250) or "qr" (cv2.QRCodeDetector, payload sanitized — newlines would poison the CSV codec), SAME Sighting stream. Detectors emit MINIMAL sightings; perception enriches |
| `vision/detector.py` | ✅ implemented | S7 | OPTIONAL YOLO fallback, off by default in configs. Root Detector.py VENDORED with 3 verified bugs fixed: (1) finally-del NameError thread-killer -> per-item guard, loud survival; (2) silent COCO fallback -> NO fallback, `infer` injected + weights config-validated; (3) unbounded queue -> deque(maxlen) drop-OLDEST + rate-limited loud counter (`dropped_total` drives perception auto-shed). Callback contract root-identical (fires ONLY when detections exist). `except Exception` whitelist entry (deliberate, reviewed — see conventions). CannedDetector = same pool, worker-thread callbacks, JSON script |
| `vision/perception.py` | ✅ implemented | S7 | qualifier_run.py:192–252 detection_loop/callback. PerceptionLoop per drone: frame_number dedupe, sync marker detect every new frame, enrichment via dataclasses.replace, **CSV+bus at the PUBLISH site** (bounded bus eviction must never lose score rows — binding for S8), `last_frame_ts()` feeds the agent's VideoWatchdog, `shed()` = the DEGRADE_DETECTION consumer. PURE (removed from SDK_ALLOWED — detectors are injected callables); bearing math here: **yaw MINUS px offset** (CCW+; fixed the types.py sign conflict, test-pinned) |
| `vision/gazebo_video.py` | ✅ implemented | **S8** = SIM-4 | GazeboRgbSource(VideoSource): frames over a localhost TCP socket from `sim/gz_camera_bridge.py` (system-py3.10 gz subscriber) — gz.transport13 can't import in the 3.11 venv, so finals imports NO gz/cv2 (stdlib+numpy client, numpy LAZY). Injectable `receiver` seam + FakeFrameReceiver (mirrors pyhulax_video.py); R8G8B8→BGR via `video_channel_order`; SensorTimeout/staleness; NO auto-reconnect (bridge death → healthy=False → VideoWatchdog DEGRADE, drill-proven). Typed-only catches |
| `mission/phases/search.py` | ✅ implemented | **S8** = SIM-4 | SentryScan (DEFAULT, no-position): Takeoff → [Hover(dwell), Rotate(step)] × revolutions → Land → Done; pure precomputed plan like takeoff_demo, `from_config` zone tunables + altitude-band height + no-op-trap validation. OpenLoopLawnmower (config-gated, OFF by default): body-frame boustrophedon (coverage.py position math deliberately NOT used — no measured position source). Both keep their registry names |
| `flight/pyhulax_adapter.py` | ✅ implemented | **S9** | hula_connection.py:29–37 + https://pyhulax.xenops.ae (audit bar: the mapping_drone.py bug list). PyhulaxAdapter(FlightAdapter) + FakeDroneAPI; per-drone single-thread executor is THE choke point (every blocking SDK call under `wait_for` → hard deadline; TimeoutError → degraded latch + FlightTimeout → agent safes down); name-based SDK-error mapper (real + fake map identically, pyhulax NOT imported); connect = robust_connect single-retry on DroneConnectionError + `enable_battery_failsafe()` ALWAYS + 2 Hz lock-guarded telemetry poller (dead-flag/staleness `_check_alive_fresh`, the mapping_drone.py infinite-wait fix); `move_to` deliberately absent; cm/yaw onsite "unit hop" gate (commented, not silently decided). `EXCEPT_EXCEPTION_WHITELIST` (S9, reviewed): poller tick + emergency_land + disconnect, each full-traceback. Unit-tested pyhulax-free; 4/4 mutation kill-check |
| `flight/discovery.py` | ✅ implemented | **S9** | dola.py (port 8668 — trust the code, not its docstring). `discover_required(plane_ids, timeout_s, *, sock=)` + pure `_parse_packet`; SYNCHRONOUS deadline-bounded scan (no listener thread → dola's stop()/join bug gone); missing planes RAISE PreflightError naming the gap (dola returned `{id: None}`); parse errors TYPED + counted (not print-and-continue); `sock=` injection seam. STDLIB-ONLY (stays in the SDK conventions scan, NOT in SDK_ALLOWED) |
| `vision/pyhulax_video.py` | ✅ implemented | **S9** | hula_connection.py + pyhulax video docs (no auto-reconnect!). PyhulaxVideoSource(VideoSource) + FakeVideoStream over a SHARED DroneAPI stream; None startup-window tolerated (bounded, → SensorTimeout); ERROR(state 4) → bounded stop()/start() restart ladder → healthy=False (no auto-reconnect); `video_channel_order` flag normalizes .to_rgb() to the BGR contract (seam, never hardcoded; fake channel order observable WITHOUT numpy). Typed-only catches (OFF the except-Exception whitelist by design). **S10: WIRED — in `_WIRED_FRAME_BACKENDS`; main shares its DroneAPI with the adapter; preflight P3→P4→P6 does discovery→ip→connect-before-start** |
| `preflight.py` | ✅ implemented | **S10** | mapping_drone.py prompt (audited) + deployment.md checklist. Ordered **P0–P10** gate that refuses flight unless every critical check passes; runnable standalone via `--preflight-only` (P0–P9, props off, never flies) as the primary bench tool. PURE (NO top-level SDK import — not in SDK_ALLOWED; all SDK contact through the injected adapter/source/api; discovery + marker detector lazy/injected). Each gate returns a `CheckResult`; the FIRST critical FAIL tears the fleet down (stop video, disconnect adapters) → `PreflightError` → exit 3; non-critical (P7) WARNs. On mission-path success adapters stay CONNECTED + sources STARTED for `_amain` (P4 connect → P6 stream start = the connect-before-stream-start ordering). P3 applies the discovered IP via `set_target_ip` before P4. P10 default-deny fixes mapping_drone.py:318–327 (invalid input fell THROUGH to arm). Tested pyhulax/cv2-free via FakeDroneAPI/FakeVideoStream + injected discovery/detector |
| `mission/phases/track_convoy.py` | ⬜ stub | **S11** (post-briefing) | Sighting stream + bearing servo |
| `mission/phases/land_on_pad.py` | ⬜ stub | **S11** (post-briefing) | ArUco pattern; PAD_* visual servo |
| `mission/planning/types.py` | ✅ contracts (from_dict HARDENED) | **S11 (NAV-0, NAV-2)** | frozen map/geometry shapes (Leg, KeepOut, LandingPad, ArenaMap + from_dict); frame = (north_m,east_m), heading CCW+ (dead_reckon sign). **NAV-2 hardened from_dict: bounds ordering (strict min<max), CLOSED pads/c2-origin within bounds, unique pad/keep-out ids (first-collision message), >= 3-DISTINCT-vertex keep-outs (collinear-but-distinct allowed; geometry left to NAV-1), non-finite reject; loud actionable ConfigError each** |
| `mission/planning/frame.py` | ✅ implemented | **S11 (NAV-2)** | PURE coordinate-frame plumbing (stdlib `math` only). `discord_to_ned(coord,c2_origin_m,c2_heading_deg)` reuses dead_reckon's body->NED rotation (psi_NED=-yaw_deg, CCW+, 0=+north): coord = (forward_m,right_m) in C2's local boot frame (assumption A8 — the Discord wire format is an onsite unknown kept at the edge; the assumed shape is a VALIDATED contract, loud on malformed). `in_sector` = ADVISORY-only soft-geofence wedge predicate (CLOSED boundary, half-width>=180=>all, at-origin=>in); `bearing_from_c2_deg` helper. Tested in `test_frame.py` |
| `mission/planning/polygon_tools.py` | ✅ implemented | **S11 (NAV-1)** | inflate_polygon (per-vertex outward MITER offset; winding-independent via signed-area sign; near-degenerate clamp) / point_in_polygon (even-odd ray cast, boundary == OUTSIDE, half-open straddle) / segment_intersects_polygon (conservative — touching counts) + **segment_enters_polygon** (proper-crossing variant for visibility edges, so a graph edge may touch the corner it connects). Pure stdlib math. 26 hand-computed tests; mutation kill-check (inflate-sign + proper-crossing strictness KILLED; `>`/`>=` straddle proven EQUIVALENT under the on-edge guard) |
| `mission/planning/visibility_graph.py` | ✅ implemented | **S11 (NAV-1)** | `plan(start,goal,arena,inflation_m,max_leg_cm)->list[Leg]`; visibility-graph A* (euclidean cost+heuristic, deterministic tie-break) over inflated keep-outs; corners buried in another inflated poly dropped. heading_deg = `atan2(-dE, dN)` = INVERSE of dead_reckon's FORWARD map (pinned by the heading-consistency test through the REAL DeadReckoner). Equal-sub-leg subdivision at max_leg_cm. Raises PlanningError (start/goal inside an inflated keep-out → names the id; no collision-free path) + ValueError on non-finite/inflation<0/max_leg<=0. 28 tests |
| `mission/phases/_servo.py` | ✅ implemented | **S11 (NAV-3)** | shared PURE servo math: wrap180/clamp/bearing_error_to_rotate/pixel_offset_to_move; sign matches perception.py (yaw MINUS px offset). **Conventions LOCKED (test_servo.py-pinned): heading = CCW+ yaw frame, error=wrap180(target-yaw), +error→+(CCW) Rotate (same frame as perception bearing_deg); lateral = body-frame chase-the-blob, px=cx-w/2>0 (target RIGHT of centre)→Direction.RIGHT; altitude×similar-triangles scale so a pixel offset→~constant ground cm. wrap180 closed end at +180. Deadbands INCLUSIVE (\|err\|<=tol→None). 5/6 mutation kill-check (sign/LEFT-RIGHT/deadband/clamp/altitude killed; wrap180 >= boundary was equivalent → proved dead-code & removed).** DRY home for land_on_pad + track_convoy after s11 merges |
| `mission/phases/navigate.py` | ⬜ stub | **S11 (NAV-5)** | open-loop transit phase: planner Legs -> Rotate-to-absolute-heading (compass) + Move(FORWARD); position stays NONE, DeadReckoner ADVISORY only |

## Session roadmap (definition of done + test gate per session)

| # | Scope | Done when | Gate |
|---|---|---|---|
| S1 | Skeleton + contracts | this tree exists; pytest green; `--dry-run` × 5 profiles | pytest ✅ |
| S2 | `events.py`, `sightings.py` | JSONL events + 1 Hz heartbeat + crash hooks; append+fsync sighting CSV; kill-test reload | pytest |
| S3 | `flight/adapter.py` (Bench), `mock_adapter.py`, `dead_reckon.py` | contract suite over MockAdapter; scriptable failures; DR math vs hand-computed | pytest |
| S4 | `mission/{phase,agent,orchestrator}.py`, `takeoff_demo`, main wiring (+ run-start initial-pose/origin event in the events schema — replay-plot prereq, [`simulation.md`](simulation.md) Tier 0) | `--profile mock` flies takeoff_demo end-to-end; 2-agent failure-injection test (other completes; emergency_land exactly once) | pytest + mock run |
| S5 | `guards.py` | every guard trips in tests; raising guard = trip; SafetyController idempotent | pytest |
| S6 | `flight/sitl_adapter.py` | telemetry-polling takeoff/land (no blind sleeps); `_body_offset_to_ned` unit-tested; 3× multi-instance launch recipe documented ([`simulation.md`](simulation.md) Tier 1: UDP 14540/41/42, gRPC 50051–53, `PX4_GZ_STANDALONE=1`) | **V1 ✅ SIM-1 2026-06-07** (typed failure in 1.22 s on the kill drill — staleness detector, not a hang; drift 0.019 m). **3× stretch ✅ SIM-2 2026-06-09** (3 concurrent DONE; offboard-start race fixed; kill-px4 staleness + kill-server dead-flag + 'q' abort drills all pass) — [`sim_sessions.md`](sim_sessions.md) |
| S7 | `vision/{video,aruco,perception,detector}.py` | **ArUco first**: detect + read marker IDs on every sampled frame → sightings.csv via the replay profile; vendored YOLO Detector (bugs fixed) lands as the optional config-gated extra | pytest + replay run |
| S8 | `vision/gazebo_video.py`, `phases/search.py` + convoy world (5 moving marker robots, 2 pads) + 640×480 cam model ([`simulation.md`](simulation.md) Tier 2). Executed partwise: assets **SIM-3** (∥ S4–S7), single-drone V2a **SIM-4**, 3× rehearsal **SIM-5 ✅** — [`sim_sessions.md`](sim_sessions.md) | SentryScan + config-gated lawnmower | **VM V2 ✅**: 3× sitl search logs sightings of MOVING markers (V2a = 1 drone SIM-4; full V2 = SIM-5: all 3 DONE, 5/5 ids/drone, 4 drills) |
| S9 | pyhulax leaves (+FakeDroneAPI/FakeVideoStream) | unit tests green WITHOUT pyhulax installed; audit-grade review | **✅ S9 2026-06-09** — pyhulax_adapter + discovery + pyhulax_video real; 575→ suite green pyhulax-free (bench/real `--dry-run` resolve PyhulaxVideoSource/BenchAdapter without importing pyhulax); adversarial review vs the mapping_drone.py bug list + 4/4 mutation kill-check. Live video wiring (`_WIRED_FRAME_BACKENDS`) deferred to S10 (preflight owns discovery→ip + connect-before-start) |
| S10 | `preflight.py` + `docs/finals/onsite_test_plan.md` | P0–P10 runnable via `--preflight-only`; bench B1–B8 scripted | **✅ S10 2026-06-09** — preflight P0–P10 + LIVE pyhulax wiring (`_WIRED_FRAME_BACKENDS` += pyhulax, shared DroneAPI, connect-before-stream-start via P4→P6) + idempotent `connect()`/`set_target_ip` + `discovery_timeout_s` config; [`docs/finals/onsite_test_plan.md`](../../docs/finals/onsite_test_plan.md) (P0–P10, B1–B8, gates A–G, hard rules); suite green WITHOUT pyhulax/cv2 |
| S11 | briefing phases (`track_convoy`, `land_on_pad`) | canned-tested; all tunables in config | pytest + SITL rehearsal |

Then: the 2-hour onsite window (gates A–G) — full runbook in
[`docs/finals/onsite_test_plan.md`](../../docs/finals/onsite_test_plan.md) (S10).
Hard rules onsite: tune config, not code; no new behavior in the last 25
minutes; no multi-drone flight without a proven abort.

## Open questions (defaults in force)

| Question | Default until answered |
|---|---|
| What scores (sightings/tracks/geo)? | PARTIALLY ANSWERED (user, 2026-06-06): convoy robots carry **ArUco markers to detect and READ** → ArUco is the primary detector, YOLO optional. Still open: exact scoring formula, whether repeat reads of the same ID score |
| Convoy marker details (ID list? DICT_6X6_250 confirmed? or QR codes?) | **RESOLVED (user, 2026-06-09): markers are ArUco, NOT QR** — the 2026-06-06 "QR, 20×20 cm" was loose phrasing for ArUco. `marker_backend` stays "aruco" (default → ZERO code change; the QR path remains as a dormant seam). SIM-3 (2026-06-09) confirmed the sim consequence: ArUco (20 cm) decodes at ALL sentry bands 1.2/1.7/2.2 m on 640 px — exactly what we want (a literal QR would NOT have, decode floor ~4 px/module). Still open: exact ID list / DICT (assume DICT_6X6_250) and whether 20×20 cm applies to pad markers too |
| ~~Marker PHYSICAL SIZE?~~ | ANSWERED (user, 2026-06-06): **20×20 cm**. Range math @640 px, f≈433 px: the marker spans ≈29 px at 3 m, ≈58 px at 1.5 m. ArUco at 20 cm reads reliably from ~4–8 m; a true QR needs far more px/module → realistic decode standoff only **~1–2.5 m**, which constrains sentry altitude hard and makes the QR-vs-ArUco confirmation above the highest-value open question. Still measure actual read range at onsite gate F; keep range a config value (`zone` params). Sim detection-range math: [`simulation.md`](simulation.md) Tier 2 |
| Pad validity encoding? | ArUco `valid_marker_ids` in config; shape classifier stub |
| Abort key legal in scored runs? | wired, safety-only (land-all); ask organizers |
| C2-side UWB for swarm challenge? | `use_uwb: false`; UWB code is a leaf |
| pyhulax `move()` units (docs say cm; example shows 0.5)? | contract in cm; onsite "unit hop" gate; fix isolated to adapter |
| `.to_rgb()` channel order? | `video_channel_order: "rgb"`; bench red-object check |
| HULA camera HFOV? | `camera_hfov_deg: null` → bearing_deg null; bench-measure |
