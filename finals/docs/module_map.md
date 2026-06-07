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
```

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
| `errors.py` | ✅ implemented | S1 | failure-mode audit of the official examples |
| `config.py` | ✅ implemented | S1 | qualifier_run.py:72–132 MissionConfig; known-issue #9 weights guards |
| `main.py` | ✅ implemented | S1 / S4 / S7 | qualifier_run.py parse_args/_amain; S4: bench wiring builds the INNER backend first (BenchAdapter special case); phases via the `from_config` soft convention; S7: `_run_replay` (the no-drone runner) + per-drone perception/VideoWatchdog wiring gated on `_WIRED_FRAME_BACKENDS` = {"replay"} — deliberately NOT `frame_backend != "none"`: sitl.json declares "gazebo" before S8 wires it (a watchdog without a frame source = guaranteed-false DEGRADE). **S8: add "gazebo" to the set + a source branch in `_build_perception` — no orchestrator changes** (sightings.csv is appended at the PUBLISH site, perception.py) |
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
| `sim/` world assets (`gen_markers.py`, convoy model+world, `check_detection.py`) | ⬜ | **SIM-3** (∥ S4–S7) | simulation.md Tier 2 recipes; markers BOTH ArUco + QR (20×20 cm intel); convoy driver per SIM-0 ros_gz probe |
| `tools/replay_plot.py` | ⬜ | **SIM-2** | feeds mission.jsonl through the REAL DeadReckoner (never reimplements the math); matplotlib-only — inside the conventions scan, cv2/gz forbidden here |
| `configs/sitl3.json` | ⬜ | **SIM-2** | sitl.json × 3 (UDP 14540/41/42, gRPC 50051–53, bands 1.2/1.7/2.2) |
| `configs/sitl3_vision.json` | ⬜ | **SIM-5** | sitl3 + gazebo frames + search; `command_timeout_s` sized from measured RTF |
| `mission/agent.py` | ✅ implemented | S4 | hula_connection.py:39–63 loop, formalized; mapping_drone.py watchdog gaps CLOSED: outer wait_for deadline on every command, telemetry-staleness check, emergency-land-EXACTLY-ONCE latch (shielded from cancellation), FAILED terminal — no auto-restart. Emits the `origin` + `action_complete` events (replay prereq). S7: injectable `frame_ts_fn` (-> GuardContext.last_frame_ts, the VideoWatchdog feed) + `on_degrade` hook (DEGRADE_DETECTION trips -> PerceptionLoop.shed) + read-only `last_telemetry` (perception's enrichment source) |
| `mission/orchestrator.py` | ✅ implemented | S4 | qualifier_run.py:407–513 supervisor MINUS auto-restart (unsafe on real aircraft); agents = independent asyncio tasks; budget stop + settle-grace hard deadline; 1 Hz atomic heartbeat; seq-cursor SightingBus drain; whitelisted blanket catches always log tracebacks |
| `mission/phases/takeoff_demo.py` | ✅ implemented | S4 | mapping_drone.py:343–355 intent, as relative moves (no UWB dependency); tunables via `zone["takeoff_demo"]` + `altitude_band_m` (`from_config`) |
| `guards.py` | ✅ implemented | S5 | qualifier_run.py:383–393 crash→land path; mapping_drone.py gap audit. Trip vocabulary ADVISORY→LAND_ALL (max severity wins); per-drone guards run in the AGENT loop, swarm guards on the orchestrator tick (stub-vs-S4 reconciliations in the module docstring); a raising guard IS a trip; SafetyController = landing slot + wall-clock-bounded retry ladder + completion-shared trip(); AbortListener thread ('q'+Enter → orderly land-all). VideoWatchdog WIRED in S7 (agent `frame_ts_fn` -> GuardContext.last_frame_ts; built by main only for `_WIRED_FRAME_BACKENDS`) |
| `flight/sitl_adapter.py` | ⬜ stub | **S6** = SIM-1 (V1) + SIM-2 (3×) | drone_control.py + get_position_with_task.py + qualifier_run.py:268–331 (all proven) — VENDORED-WITH-FIXES, not wrapped: connect() hardcodes :14540, `_kill_stale_servers` is a global pkill, System() not injectable ([`sim_sessions.md`](sim_sessions.md) recap §2) |
| `vision/aruco.py` | ✅ implemented | S7 | **PRIMARY detector** (convoy robots carry markers — detect + read ID, no training). potential_detection_targets.py:5–30 (audited, not copied — its `detectMarkers` unpack is a SYNTAX ERROR; three values, fixed). Pluggable seam: `make_marker_detector(cfg.marker_backend)` -> "aruco" (DICT_6X6_250) or "qr" (cv2.QRCodeDetector, payload sanitized — newlines would poison the CSV codec), SAME Sighting stream. Detectors emit MINIMAL sightings; perception enriches |
| `vision/detector.py` | ✅ implemented | S7 | OPTIONAL YOLO fallback, off by default in configs. Root Detector.py VENDORED with 3 verified bugs fixed: (1) finally-del NameError thread-killer -> per-item guard, loud survival; (2) silent COCO fallback -> NO fallback, `infer` injected + weights config-validated; (3) unbounded queue -> deque(maxlen) drop-OLDEST + rate-limited loud counter (`dropped_total` drives perception auto-shed). Callback contract root-identical (fires ONLY when detections exist). `except Exception` whitelist entry (deliberate, reviewed — see conventions). CannedDetector = same pool, worker-thread callbacks, JSON script |
| `vision/perception.py` | ✅ implemented | S7 | qualifier_run.py:192–252 detection_loop/callback. PerceptionLoop per drone: frame_number dedupe, sync marker detect every new frame, enrichment via dataclasses.replace, **CSV+bus at the PUBLISH site** (bounded bus eviction must never lose score rows — binding for S8), `last_frame_ts()` feeds the agent's VideoWatchdog, `shed()` = the DEGRADE_DETECTION consumer. PURE (removed from SDK_ALLOWED — detectors are injected callables); bearing math here: **yaw MINUS px offset** (CCW+; fixed the types.py sign conflict, test-pinned) |
| `vision/gazebo_video.py` | ⬜ stub | **S8** = SIM-4 (assets SIM-3, rehearsal SIM-5) | qualifier_run.py RgbReceiver (proven in sim) |
| `mission/phases/search.py` | ⬜ stub | **S8** = SIM-4 | SentryScan default (no-position searcher); coverage.py lane math informs lawnmower |
| `flight/pyhulax_adapter.py` | ⬜ stub | **S9** | hula_connection.py:29–37 + https://pyhulax.xenops.ae (audit bar: the mapping_drone.py bug list) |
| `flight/discovery.py` | ⬜ stub | **S9** | dola.py (port 8668 — trust the code, not its docstring) |
| `vision/pyhulax_video.py` | ⬜ stub | **S9** | hula_connection.py + pyhulax video docs (no auto-reconnect!) |
| `preflight.py` | ⬜ stub | **S10** | mapping_drone.py prompt (audited) + deployment.md checklist |
| `mission/phases/track_convoy.py` | ⬜ stub | **S11** (post-briefing) | Sighting stream + bearing servo |
| `mission/phases/land_on_pad.py` | ⬜ stub | **S11** (post-briefing) | ArUco pattern; PAD_* visual servo |

## Session roadmap (definition of done + test gate per session)

| # | Scope | Done when | Gate |
|---|---|---|---|
| S1 | Skeleton + contracts | this tree exists; pytest green; `--dry-run` × 5 profiles | pytest ✅ |
| S2 | `events.py`, `sightings.py` | JSONL events + 1 Hz heartbeat + crash hooks; append+fsync sighting CSV; kill-test reload | pytest |
| S3 | `flight/adapter.py` (Bench), `mock_adapter.py`, `dead_reckon.py` | contract suite over MockAdapter; scriptable failures; DR math vs hand-computed | pytest |
| S4 | `mission/{phase,agent,orchestrator}.py`, `takeoff_demo`, main wiring (+ run-start initial-pose/origin event in the events schema — replay-plot prereq, [`simulation.md`](simulation.md) Tier 0) | `--profile mock` flies takeoff_demo end-to-end; 2-agent failure-injection test (other completes; emergency_land exactly once) | pytest + mock run |
| S5 | `guards.py` | every guard trips in tests; raising guard = trip; SafetyController idempotent | pytest |
| S6 | `flight/sitl_adapter.py` | telemetry-polling takeoff/land (no blind sleeps); `_body_offset_to_ned` unit-tested; 3× multi-instance launch recipe documented ([`simulation.md`](simulation.md) Tier 1: UDP 14540/41/42, gRPC 50051–53, `PX4_GZ_STANDALONE=1`) | **VM V1**: sitl takeoff→square→land; PX4 killed mid-move ⇒ FlightTimeout, not a hang. Stretch (after V1): 3× concurrent takeoff→square→land. Executed as SIM-1 (V1) + SIM-2 (3×) — [`sim_sessions.md`](sim_sessions.md) |
| S7 | `vision/{video,aruco,perception,detector}.py` | **ArUco first**: detect + read marker IDs on every sampled frame → sightings.csv via the replay profile; vendored YOLO Detector (bugs fixed) lands as the optional config-gated extra | pytest + replay run |
| S8 | `vision/gazebo_video.py`, `phases/search.py` + convoy world (5 moving marker robots, 2 pads) + 640×480 cam model ([`simulation.md`](simulation.md) Tier 2). Executed partwise: assets **SIM-3** (∥ S4–S7), single-drone V2a **SIM-4**, 3× rehearsal **SIM-5** — [`sim_sessions.md`](sim_sessions.md) | SentryScan + config-gated lawnmower | **VM V2**: 3× sitl search logs sightings of MOVING markers (V2a = 1 drone in SIM-4; full V2 = SIM-5) |
| S9 | pyhulax leaves (+FakeDroneAPI/FakeVideoStream) | unit tests green WITHOUT pyhulax installed; audit-grade review | pytest |
| S10 | `preflight.py` + `docs/onsite_test_plan.md` | P0–P10 runnable via `--preflight-only`; bench B1–B8 scripted | pytest + bench-ready |
| S11 | briefing phases (`track_convoy`, `land_on_pad`) | canned-tested; all tunables in config | pytest + SITL rehearsal |

Then: the 2-hour onsite window (gates A–G; see the plan's onsite section, to be
expanded into `docs/onsite_test_plan.md` in S10). Hard rules onsite: tune
config, not code; no new behavior in the last 25 minutes; no multi-drone
flight without a proven abort.

## Open questions (defaults in force)

| Question | Default until answered |
|---|---|
| What scores (sightings/tracks/geo)? | PARTIALLY ANSWERED (user, 2026-06-06): convoy robots carry **ArUco markers to detect and READ** → ArUco is the primary detector, YOLO optional. Still open: exact scoring formula, whether repeat reads of the same ID score |
| Convoy marker details (ID list? DICT_6X6_250 confirmed? or QR codes?) | PARTIALLY ANSWERED (user, 2026-06-06): markers are **QR codes, 20×20 cm**. CONFIRM: literal QR (cv2.QRCodeDetector / pyhulax onboard `recognize_qr`) vs "QR" said loosely for ArUco — and whether 20×20 applies to convoy markers, pad markers, or both. S7 SHIPPED the pluggable seam: config `marker_backend` ("aruco" default \| "qr"), both paths feed the same Sighting stream — the confirmation is now a ONE-KEY config edit, zero code |
| ~~Marker PHYSICAL SIZE?~~ | ANSWERED (user, 2026-06-06): **20×20 cm**. Range math @640 px, f≈433 px: the marker spans ≈29 px at 3 m, ≈58 px at 1.5 m. ArUco at 20 cm reads reliably from ~4–8 m; a true QR needs far more px/module → realistic decode standoff only **~1–2.5 m**, which constrains sentry altitude hard and makes the QR-vs-ArUco confirmation above the highest-value open question. Still measure actual read range at onsite gate F; keep range a config value (`zone` params). Sim detection-range math: [`simulation.md`](simulation.md) Tier 2 |
| Pad validity encoding? | ArUco `valid_marker_ids` in config; shape classifier stub |
| Abort key legal in scored runs? | wired, safety-only (land-all); ask organizers |
| C2-side UWB for swarm challenge? | `use_uwb: false`; UWB code is a leaf |
| pyhulax `move()` units (docs say cm; example shows 0.5)? | contract in cm; onsite "unit hop" gate; fix isolated to adapter |
| `.to_rgb()` channel order? | `video_channel_order: "rgb"`; bench red-object check |
| HULA camera HFOV? | `camera_hfov_deg: null` → bearing_deg null; bench-measure |
