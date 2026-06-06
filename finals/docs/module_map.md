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
python -m finals.main --profile sitl --phases takeoff_demo   # VM (from S4+S6)
```

Profiles: `mock` (pure logic) | `sitl` (qualifier PX4 SITL + Gazebo VM) |
`replay` (disk frames + detector, 0 drones) | `bench` (real drones, props off,
flight REFUSED) | `real` (gated 3×: CLI flag → preflight P0–P10 → operator GO).

Simulation strategy — what is/isn't simulable, tier recipes, rejected
alternatives: [`simulation.md`](simulation.md). (PX4 SITL is a physics
stand-in behind the FlightAdapter seam; the finals drones are HULA/pyhulax.)

## Binding conventions (every session re-reads these)

1. **No bare `except`.** `except Exception` ONLY in `guards.py` (SafetyController
   safe-down, guard-evaluation wrapper) and `mission/orchestrator.py` (top
   loop) — always logged with traceback. Enforced by `tests/test_conventions.py`.
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
| `main.py` | ✅ through `--dry-run` | S1 (flight-wired **S4**) | qualifier_run.py parse_args/_amain |
| `mission/phase.py` | ✅ contract | S1 (exercised S4) | hula_connection.py:46–50, made pure |
| `mission/phases/__init__.py` | ✅ registry | S1 | — |
| `flight/adapter.py` | ✅ implemented (ABC + BenchAdapter) | S1 / S3 | pyhulax ∩ MAVSDK honest primitives; Bench wraps an inner adapter (S4 wiring needs a special case) |
| `vision/video.py` | ✅ ABC / ReplaySource stub | S1 / **S7** | qualifier_run.py:163–186 RgbReceiver contract |
| `events.py` | ✅ implemented | S2 | barrel_log.py atomic-flush discipline; runs/<ts>/ convention |
| `sightings.py` | ✅ implemented | S2 | barrel_log.py lock discipline, inverted to append-only+fsync |
| `flight/mock_adapter.py` | ✅ implemented | S3 | — (the test double everything stands on; pipeline order + deviations in its docstring) |
| `flight/dead_reckon.py` | ✅ implemented | S3 | detection_to_world.py body→NED yaw math, reduced; yaw sign flipped to pyhulax CCW (psi_NED = -yaw_deg, documented + test-pinned) |
| `docs/simulation.md` | ✅ doc | — | feasibility research pass (2026-06-06), load-bearing claims verified against primary sources; PX4 SITL = physics stand-in, finals drones are HULA |
| `mission/agent.py` | ⬜ stub | **S4** | hula_connection.py:39–63 loop; mapping_drone.py watchdog gaps |
| `mission/orchestrator.py` | ⬜ stub | **S4** | qualifier_run.py:407–513 supervisor MINUS auto-restart (unsafe on real aircraft) |
| `mission/phases/takeoff_demo.py` | ⬜ stub | **S4** | mapping_drone.py:343–355 intent, as relative moves |
| `guards.py` | ⬜ stub | **S5** | qualifier_run.py emergency-land path; mapping_drone.py gap audit |
| `flight/sitl_adapter.py` | ⬜ stub | **S6** | drone_control.py + get_position_with_task.py + qualifier_run.py:268–331 (all proven) |
| `vision/aruco.py` | ⬜ stub | **S7** | **PRIMARY detector** (convoy robots carry ArUco markers — detect + read ID, no training). potential_detection_targets.py:5–30 (audited — it has a syntax error; detectMarkers returns 3 values) |
| `vision/detector.py` | ⬜ stub | **S7** (optional half) | OPTIONAL YOLO fallback, off by default in configs. Root Detector.py VENDORED with 3 verified bugs fixed (finally-NameError thread-killer, silent COCO fallback, unbounded queue) |
| `vision/perception.py` | ⬜ stub | **S7** | qualifier_run.py:192–252 detection_loop/callback |
| `vision/gazebo_video.py` | ⬜ stub | **S8** | qualifier_run.py RgbReceiver (proven in sim) |
| `mission/phases/search.py` | ⬜ stub | **S8** | SentryScan default (no-position searcher); coverage.py lane math informs lawnmower |
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
| S6 | `flight/sitl_adapter.py` | telemetry-polling takeoff/land (no blind sleeps); `_body_offset_to_ned` unit-tested; 3× multi-instance launch recipe documented ([`simulation.md`](simulation.md) Tier 1: UDP 14540/41/42, gRPC 50051–53, `PX4_GZ_STANDALONE=1`) | **VM V1**: sitl takeoff→square→land; PX4 killed mid-move ⇒ FlightTimeout, not a hang. Stretch (after V1): 3× concurrent takeoff→square→land |
| S7 | `vision/{video,aruco,perception,detector}.py` | **ArUco first**: detect + read marker IDs on every sampled frame → sightings.csv via the replay profile; vendored YOLO Detector (bugs fixed) lands as the optional config-gated extra | pytest + replay run |
| S8 | `vision/gazebo_video.py`, `phases/search.py` + convoy world (5 moving ArUco robots, 2 pads) + 640×480 cam model ([`simulation.md`](simulation.md) Tier 2) | SentryScan + config-gated lawnmower | **VM V2**: 3× sitl search logs sightings of MOVING markers |
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
| Convoy marker details (ID list? DICT_6X6_250 confirmed? or QR codes?) | DICT_6X6_250 per example code; marker→robot ID map in config. If they turn out to be QR codes, cv2.QRCodeDetector slots into the same Sighting stream (and pyhulax has onboard recognize_qr as a fallback) |
| **ArUco marker PHYSICAL SIZE?** Readable range scales with size — a ~10 cm marker is reliably decodable from roughly 2–4 m on a 640×480 stream, and motion blur on a moving robot shrinks that further | Constrains sentry altitude + standoff distance directly. Default: fly the LOWEST altitude band the safety rules allow and measure actual read range at onsite gate F (detection flight) before fixing search geometry; treat range as a config value (`zone` params), not an assumption. Sim detection-range math: [`simulation.md`](simulation.md) Tier 2 |
| Pad validity encoding? | ArUco `valid_marker_ids` in config; shape classifier stub |
| Abort key legal in scored runs? | wired, safety-only (land-all); ask organizers |
| C2-side UWB for swarm challenge? | `use_uwb: false`; UWB code is a leaf |
| pyhulax `move()` units (docs say cm; example shows 0.5)? | contract in cm; onsite "unit hop" gate; fix isolated to adapter |
| `.to_rgb()` channel order? | `video_channel_order: "rgb"`; bench red-object check |
| HULA camera HFOV? | `camera_hfov_deg: null` → bearing_deg null; bench-measure |
