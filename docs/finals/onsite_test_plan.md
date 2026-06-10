# Onsite test plan — the 2-hour hardware window (S10)

DSTA BrainHack 2026 **finals swarm challenge**. ONE ~2-hour onsite window, 3×
HULA drones over Wi-Fi via pyhulax, all logic on the C2 laptop. This is the
runbook for that window: what to run, in what order, what each gate proves, and
when to refuse to fly.

It pairs with the code: every preflight gate below maps 1:1 to a gate in
[`finals/preflight.py`](../../finals/preflight.py), runnable standalone as the
primary bench tool:

```bash
python -m finals.main --profile bench --preflight-only   # P0–P9, props off, NEVER flies
python -m finals.main --profile real  --i-know-this-arms-real-drones   # full P0–P10 then flight
```

Exit codes (both paths): **0** all gates passed · **2** config error ·
**3** a critical preflight gate FAILED (fleet safed down — disconnected, video
stopped — and flight refused). Read the one-line-per-gate table on stderr and
the persisted `runs/<ts>/preflight.json`.

**The three flight gates (real profile only), in order:**
1. `--i-know-this-arms-real-drones` CLI flag (acknowledges hardware contact),
2. preflight **P0–P10** all green,
3. the operator types literal **`GO`** at P10 (default-deny — see below).

Derived from [`example_code/mapping_drone.py`](example_code/mapping_drone.py)
confirm-prompt (audited), [`docs/quali/deployment.md`](../quali/deployment.md)
pre-run checklist, and the `real.json` gate comments.

---

## Timeline (target — adjust to the actual window)

| Time | Block | What |
|---|---|---|
| 0:00–0:15 | **Arrival** | Laptop up, Wi-Fi to the drone SSIDs, `pip install pyhulax` present, `pytest finals/tests` green, drones powered + on the ground, props **OFF**. |
| 0:15–0:55 | **Bench (B1–B8)** | Props-off validation. ZERO flight risk. Discovery → connect → telemetry → 3-stream + detector load → real-frame capture → command-semantics probe → link-loss drill → abort rehearsal. |
| 0:55–1:15 | **Onsite config (gates A–G)** | Re-validate every placeholder in `real.json` against the real fleet/arena. Tune **config, not code**. |
| 1:15–1:20 | **Props on, single-drone hover** | One drone, lowest band, full P0–P10 → short hover → land. Proves the abort + the connect-before-stream ordering on real hardware. |
| 1:20–1:50 | **Scored runs** | Full 3× swarm. Re-run preflight before EVERY launch. |
| 1:50–2:00 | **Stop** | No new behavior in the last 25 min (see hard rules). Pull logs. |

---

## Preflight P0–P10 (the refuse-to-fly gate)

Each gate returns one PASS / FAIL / WARN line. The FIRST critical FAIL tears the
fleet down and exits 3. `--preflight-only` runs **P0–P9** (skips the P10 GO
prompt — it never flies); a real launch runs the full **P0–P10**.

| Gate | Proves | Critical | If it FAILS |
|---|---|---|---|
| **P0** config sanity | profile bench/real; plane_ids set + **distinct**; altitude bands distinct (the collision guarantee); `frame_backend == "pyhulax"` | yes | Fix `real.json` — duplicate plane_id means discovery can't tell two drones apart. |
| **P1** log dir writable | `run_dir` exists, is writable, survives a write→fsync→delete probe | yes | Free disk / fix the path — a run that can't persist evidence must not fly. |
| **P2** perception readiness | the marker detector builds (`make_marker_detector("aruco")` → cv2 present); YOLO weights load IF `detector.backend == ultralytics` | yes | `pip install opencv-python "numpy<2"`; check the weights path. |
| **P3** discovery | Dola UDP (port 8668) finds **exactly** the expected plane_ids within `discovery_timeout_s`; the resolved IP is applied to each adapter via `set_target_ip` BEFORE connect | yes | Check Wi-Fi/SSID, drone power, that the plane_id labels match the fleet. A missing plane names the gap. |
| **P4** connect | each drone connects (one `robust_connect` retry); `enable_battery_failsafe()` runs ALWAYS; the 2 Hz telemetry poller delivers a first reading | yes | Wi-Fi/range/battery; another client may hold the link. |
| **P5** telemetry sane | battery ≥ `min_battery_pct`; telemetry fresh; altitude ≈ 0 (**on the ground**) | yes | Charge/swap; a drone that already reads airborne is a zeroing fault to catch on the bench, not in the air. |
| **P6** video fresh | per-drone `PyhulaxVideoSource.start()` succeeds, a first frame arrives, `healthy` is True (stream LEFT running for the mission) | yes | Camera/link; the stream has **no auto-reconnect** — a dead stream stays dead. |
| **P7** detect + tick load | marker detect runs on a live frame; **projected laptop load** = worst detect ms × drones × `tick_hz` | **no (WARN)** | WARN only: lower `tick_hz` or shed detection. A slow laptop is surfaced, never a hard block. |
| **P8** UWB serial | only if `cfg.use_uwb` (finals = **false** → reported skipped) | yes | n/a for swarm finals. |
| **P9** safety systems | identity LED set per drone (`set_led(*led_rgb)`); battery failsafe confirmed (set at P4) | yes | Check the LED mapping — wrong colors = wrong drone in the air. |
| **P10** operator GO | operator types literal **`GO`** within `go_timeout_s` (60 s). **DEFAULT-DENY**: anything else — wrong word OR timeout — refuses. Skipped under `--preflight-only`. | yes | This is the human gate. No GO = no arm. |

> **P10 fixes a real bug.** [`mapping_drone.py:318–327`](example_code/mapping_drone.py)
> prompts `y/n`, and on *invalid* input the `else` branch prints "Invalid input"
> and **falls through to `arm()`** — an unrecognized answer ARMS the drone. Here
> anything but exactly `GO` (including a timeout) is a refusal.

---

## Bench checks B1–B8 (props OFF — zero flight risk)

The bench profile (`flight_backend: "bench"`) wraps the real PyhulaxAdapter but
**refuses every flight command** — so B1–B8 exercise the full discovery → connect
→ telemetry → video → detect stack on real hardware with no way to take off.

| # | Check | How | Pass criterion |
|---|---|---|---|
| **B1** | Discovery | `--preflight-only` (P3) | All 3 plane_ids resolve to IPs; no extras. |
| **B2** | Connect + failsafe | `--preflight-only` (P4) | All 3 connect; failsafe enabled; first telemetry within `command_timeout_s`. |
| **B3** | Telemetry sanity | `--preflight-only` (P5) | Battery, altitude≈0, fresh ts for all 3. Eyeball battery vs the physical LEDs. |
| **B4** | 3 streams + detector load | `--preflight-only` (P6+P7) | All 3 streams healthy; P7 projected load < 1.0 (else lower `tick_hz`). This is the laptop-overload gate — run it with all 3 live. |
| **B5** | Real-frame capture | inspect `runs/<ts>/` after a bench run with a marker held in view | A frame is captured; ArUco decodes the held marker's ID. |
| **B6** | Command-semantics probe (**the "unit hop"**) | with props OFF, issue ONE small `move`/`takeoff` via a scratch script and watch the motor/telemetry response | Confirms the cm-vs-m unit (pyhulax docs say cm; `hula_connection.py:45` shows `move(FORWARD, 0.5)`). If it's metres, the fix is ONE line in `pyhulax_adapter.py` `move()` — **commented there, never silently rescaled.** |
| **B7** | Link-loss drill | walk one drone out of Wi-Fi range mid-bench | Telemetry goes STALE → the adapter raises a typed FlightError in ~1 s (the `_check_alive_fresh` kill detector), NOT a silent hang. |
| **B8** | Abort-channel rehearsal | press the abort key during a bench run | The AbortListener fires an orderly land-all request (safety-only). Prove it works on the bench before any prop spins. |

---

## Onsite config gates A–G (tune config, NOT code)

Every value in `real.json` is a placeholder until validated here. All of these
are JSON edits — no source changes.

| Gate | Re-validate | Field(s) |
|---|---|---|
| **A** | plane_id ↔ physical drone mapping (run discovery, walk to each lit drone) | `drones[].plane_id` |
| **B** | identity LED colors (P9 sets them — confirm by eye) | `drones[].led_rgb` |
| **C** | altitude bands distinct + safe for the ceiling | `drones[].altitude_band_m` (1.2 / 1.7 / 2.2) |
| **D** | camera HFOV — **bench-measure**; until set, sightings carry `bearing_deg=null` | `camera_hfov_deg` |
| **E** | arena / zone / search params from the briefing | `drones[].zone`, budget |
| **F** | **marker decode range** — hold a 20 cm ArUco at each band, find the real read distance | `zone` range params (SIM-3: ArUco decodes at all bands on 640 px; a literal QR would not — markers are ArUco) |
| **G** | abort rehearsal + the operator GO drill, on real hardware before scored flight | — |

---

## Challenge-2A LANDING (44%) — nav gates (added by NAV-9)

The blocks above are the 2B convoy-tag mission (altitude bands + sentry search).
**2A is the separate, equally-weighted LANDING mission**: launch 3 HULA from C2,
navigate position-blind to 3 chosen pads avoiding obstacles, land each inside its
hoop. It uses the unified nav layer (planner → `navigate` → `land_on_pad`) and a
DIFFERENT deconfliction — **TIME + SPACE, never altitude bands** (the ~1.1 m
ceiling + no-overfly rule forbid bands). Config: [`finals/configs/landing_real.json`](../../finals/configs/landing_real.json)
(`flight_backend: pyhulax`), arena [`finals/configs/arenas/sample.json`](../../finals/configs/arenas/sample.json)
(re-measure + overwrite onsite).

### SITL rehearsal (done on the VM, NOT onsite — proves the LOGIC)

A PX4+Gazebo rehearsal of the **backend-agnostic** nav logic (the real drone is
HULA via pyhulax; the nav phases emit the same Action vocabulary above the adapter
boundary, so SITL proves the algorithm — planner detour, open-loop transit, visual
landing, staggered+serialized deconfliction — NOT the HULA hardware constants).
Run via [`sim/run_landing.sh`](../../sim/run_landing.sh) on the VM:

| Gate | Proves | Run |
|---|---|---|
| **L1** | 1 drone flies `[takeoff, navigate, land_on_pad]` and lands near the pad, detouring around the crate keep-out | `bash sim/run_landing.sh land1` |
| **L2** | 3 drones, time-staggered launch + serialized landing, each to its OWN distinct valid pad; zero spurious FlightTimeouts | `bash sim/run_landing.sh land3` |
| **drills** | `q` lands all (orderly); kill instance 2 → it FAILs + exactly-once emergency_land, others finish | `abort3` / `kill3` |
| **footage** | watchable third-person + onboard mp4 of the whole flight | `viewtest` |

Evidence (RTF, final DR-vs-pad error, drill outcomes) is logged in
[`sim_sessions.md`](../../sim_sessions.md). SITL gives the drone real EKF position,
but the nav phases ignore it (position-blind by design), so the open-loop transit
+ visual servo are faithfully exercised.

### Onsite landing bench gates (tune config, not code)

These are the HULA-specific constants SITL cannot establish — bench them in the
0:55–1:15 config block, props off until the servo gate:

| Gate | Re-validate | Field(s) |
|---|---|---|
| **Gate-DR** (drift) | fly 1 m forward + a 4×90° square, tape-measure the closure error → the open-loop drift budget. Sets the planner inflation + leg subdivision so cumulative drift stays inside the inflated corridor. | `zone.navigate.inflation_m`, `max_leg_cm` |
| **Gate-AD** (ArUco decode) | hold a pad marker at the takeoff height and at `commit_alt_m`; find the real acquire/decode range looking DOWN → the acquire window + commit altitude | `zone.land_on_pad.valid_marker_ids` (the real green-pad ids), `acquire_timeout_s`, `commit_alt_m` |
| **Gate-LS** (landing servo) | props-OFF first: hold the drone over a fixed marker, confirm `pixel_offset_to_move` drives the correct LEFT/RIGHT/FWD/BACK; THEN a single props-on descent onto a stationary marker → tune `k_lateral`, `tol_px`, `descend_step_cm`, `descend_persist_frames` | `zone.land_on_pad.*` servo tunables |

(These mirror the spec's NAV-9 Gate-E/F/M; renamed Gate-DR/AD/LS so they don't
collide with the 2B swarm gates A–G above.) The binding rule still holds: **no new
behavior in the last 25 min — tune the JSON, not the code.**

---

## Convoy-tag SITL rehearsals (VM only — prove the 2B LOGIC)

The 2B convoy-tag mission (search → tag → track) is rehearsed on the VM the same
way 2A is: PX4+Gazebo exercises the **backend-agnostic** perception + tracking +
self-assignment logic; the real flight is HULA via pyhulax. These are NOT onsite
steps — they are the evidence the swarm logic is sound before the hardware window.
Run via [`sim/run_vision.sh`](../../sim/run_vision.sh) and
[`sim/run_landing.sh`](../../sim/run_landing.sh) on the VM:

| Gate | Proves | Config | Run |
|---|---|---|---|
| **S-scan** | 3 PX4 camera-drones fly `sentry_scan` over the moving convoy, all 5 ids seen | [`sitl3_vision.json`](../../finals/configs/sitl3_vision.json) (1-drone: [`sitl_vision.json`](../../finals/configs/sitl_vision.json)) | `bash sim/run_vision.sh stageB3` (`stageB`) |
| **S-lanes** | 3 drones hover + photograph 3 diverging lane-cars (S11 Workstream A — `save_marker_frames` JPEGs) | [`sitl3_lanes_vision.json`](../../finals/configs/sitl3_lanes_vision.json) | `bash sim/run_vision.sh lanes3` |
| **S-track** | 3 drones actively `track_convoy` (real bearing-pursuit chase, S11 Workstream B) | [`sitl3_track_vision.json`](../../finals/configs/sitl3_track_vision.json) | `bash sim/run_vision.sh track3` |
| **S-dyn3** | WS-5 DYNAMIC self-assignment, clean case: 3 drones over 3 cars, no hardcoded mapping → 3 distinct owners, serviced 3/3 (the C2 `ConvoyRegistry` single-winner CAS) | [`sitl3_dyn3_vision.json`](../../finals/configs/sitl3_dyn3_vision.json) | `bash sim/run_vision.sh dyn3` |
| **S-dyn5** | WS-5 contention: 3 drones over 5 cars → 3 distinct claims + 2 unclaimed; `dyn5-kill` frees a LOST car → re-claim | [`sitl3_dyn5_vision.json`](../../finals/configs/sitl3_dyn5_vision.json) | `bash sim/run_vision.sh dyn5` (`dyn5-kill`) |
| **S-handover** | WS-7A SOFT-ZONE handover: a car curves OUT of one drone's `sector_deg` wedge → owner flags `exited_zone` (keeps tracking) → C2 offers it to the IDLE neighbour whose sector it entered → `accept_offer` transfers ownership under the registry lock | [`sitl3_handover_vision.json`](../../finals/configs/sitl3_handover_vision.json) · arena [`sitl_handover.json`](../../finals/configs/arenas/sitl_handover.json) | `bash sim/run_vision.sh handover3` |

### Warm-up follow-convoy sims (WS-4 — `navigate` then `track_convoy`)

The warm-up sims fly the FULL `[takeoff → navigate (obstacle detour) → track_convoy]`
chain over a single car, validating the planner-detour + tracker hand-off in one
run (no pad landing). They are the cleanest end-to-end obstacle-avoidance + follow
demos. Run via [`sim/run_landing.sh`](../../sim/run_landing.sh) on the VM:

| Gate | Proves | Config / world / arena | Run |
|---|---|---|---|
| **FB-1** | 1 drone detours ONE crate then follows ONE car | [`sitl1_followbox1.json`](../../finals/configs/sitl1_followbox1.json) · [`followbox1_px4.sdf`](../../sim/worlds/followbox1_px4.sdf) · [`sitl_followbox1.json`](../../finals/configs/arenas/sitl_followbox1.json) | `bash sim/run_landing.sh followbox1` |
| **FB-multi** | 1 drone WEAVES 3 crates (slalom) then follows a car on an irregular route | [`sitl1_followbox_multi.json`](../../finals/configs/sitl1_followbox_multi.json) · [`followbox_multi_px4.sdf`](../../sim/worlds/followbox_multi_px4.sdf) · [`sitl_followbox_multi.json`](../../finals/configs/arenas/sitl_followbox_multi.json) | `bash sim/run_landing.sh followboxmulti` |

> **SDF ↔ arena keep-out sync is a CI assertion.** Each warm-up/landing world's
> crate `<pose>`+`<box>` footprint MUST be mirrored by a keep-out polygon in its
> arena JSON ([`sitl_landing.json`](../../finals/configs/arenas/sitl_landing.json)
> for [`landing_px4.sdf`](../../sim/worlds/landing_px4.sdf) /
> [`landing_view.sdf`](../../sim/worlds/landing_view.sdf)), since the planner
> detours around ARENA keep-outs, not SDF crates. The drift is caught locally by
> `python -m finals.tools.verify_runbook` — run it before any commit that touches
> a world or arena. The 3-lane track config also rehearses on
> [`sitl3_landing.json`](../../finals/configs/sitl3_landing.json) (3-drone landing).

---

## Single-drone & 3-drone flight test (ascend → scan → land)

Before the scored runs, prove the whole flight stack on the smallest real
mission: **ascend → rotate-scan for a landing pad → land on it**.
[`finals/tools/flight_test.py`](../../finals/tools/flight_test.py) is a thin
launcher over `finals.main` — it adds NO flight behavior; it curates a config and
hands off so EVERY safety system runs unchanged (preflight P0–P10, default-deny
GO, the `q`-abort, the SafetyController landing slot).

**Two safety gates, by design:** (1) `--live` is REQUIRED to fly — without it the
run is forced to `--dry-run` (resolved plan, exits 0, no props); (2) `finals.main`
then runs the full preflight + the operator GO prompt before any arm.

**Enter the drone codes at runtime** with `--plane-ids` (no JSON editing):

```bash
# 1 drone (flight_test_real.json — phases [takeoff, land_on_pad], no navigate):
python finals/tools/flight_test.py --dry-run                       # plan only
python finals/tools/flight_test.py --live --plane-id 7             # REAL flight

# 3 drones (flight_test_3x_real.json — same chain ×3, TIME-slot deconfliction):
python finals/tools/flight_test.py --drones 3 --plane-ids 7 10 12 --dry-run
python finals/tools/flight_test.py --drones 3 --plane-ids 7 10 12 --live
```

> **Shared Wi-Fi (BLOCKER for 3×).** Unlike the solo-AP bring-up (`192.168.100.1`
> reaches only ONE drone), the 3× test needs **all 3 drones AND the laptop on ONE
> network with distinct IPs** — discovery (`Dola().get_all_ips() → {plane_id:
> ip}`, per [`connect_all_drones_video.py`](example_code/connect_all_drones_video.py))
> must find all 3. `--plane-ids 7 10 12` sets each drone's plane_id in fleet
> order; the single-value `--plane-id`/`--marker-id`/`--height-cm` are 1-drone
> only and are **refused (exit 2)** under `--drones 3`.
>
> **Deconfliction = TIME + placement, NOT altitude bands.** No drone translates,
> so spatial safety is **physical placement**: stand the drones at well-separated
> spots, each with its OWN pad inside ITS camera footprint. The SafetyController
> serializes takeoff (launch slot) and descent (landing slot) in TIME; the
> distinct `altitude_band_m` exists ONLY to satisfy the multi-drone separation
> guard (flown height is pinned to 100 cm via `zone.takeoff.height_cm`).
>
> **Marker.** The in-flight detector [`vision/aruco.py`](../../finals/vision/aruco.py)
> decodes **DICT_7X7_1000** (the field dict), so each test pad must carry a
> **DICT_7X7_1000** marker whose id is that drone's `valid_marker_ids`.
> (Marker-less pad-mode — `servo_on: "pad"` + `--weights models/pad_v1.pt` — is
> the alternative the scored `landing_real.json` uses.) Tune `commit_alt_m` /
> `k_lateral` with the landing-bench gates above — **config, not code**.

The OFFLINE no-flight bring-up that precedes the flight is
[`finals/tools/hula_smoke.py`](../../finals/tools/hula_smoke.py) (connect →
telemetry → video → ArUco/YOLO; issues NO flight command). Its ArUco scan is
**dict-locked to DICT_7X7_1000** by default (`--aruco-dict`; `--all-dicts` = the
discovery sweep) with the field-id allowlist + per-id frame voting (kills the
cross-dict double-decode); YOLO takes `--yolo-conf` / `--edge-margin` (rejects a
hand/arm at the frame edge) / `--yolo-preproc` (gray-world/clahe vs the
oversaturated cam).

---

## Config inventory (every shipped profile — what runs it)

`verify_runbook` asserts every `finals/configs/*.json` is named here, so a new
config can never be smuggled in unmentioned. The real/bench configs are the onsite
fleet; the `sitl*`/`mock*`/`replay` configs are VM rehearsals + dev fixtures.

| Config | Profile | Used by |
|---|---|---|
| [`real.json`](../../finals/configs/real.json) | real | the scored 2B convoy-tag flight (`--profile real`) |
| [`convoy_real.json`](../../finals/configs/convoy_real.json) | real | 2B convoy-tag real fleet (track_convoy variant) |
| [`landing_real.json`](../../finals/configs/landing_real.json) | real | the scored 2A LANDING flight (`--profile real`) |
| [`flight_test_real.json`](../../finals/configs/flight_test_real.json) | real | the single-drone ascend→scan→land flight test ([`flight_test.py`](../../finals/tools/flight_test.py) default) |
| [`flight_test_3x_real.json`](../../finals/configs/flight_test_3x_real.json) | real | the 3-drone ascend→in-place-scan→land flight test (`flight_test.py --drones 3`); TIME-slot + placement deconfliction |
| [`bench.json`](../../finals/configs/bench.json) | bench | the props-OFF B1–B8 bench tool (`--profile bench --preflight-only`) |
| [`sitl1_landing.json`](../../finals/configs/sitl1_landing.json) | sitl | gate L1 / viewtest 1-drone landing rehearsal |
| [`sitl3_landing.json`](../../finals/configs/sitl3_landing.json) | sitl | gate L2 3-drone staggered+serialized landing |
| [`sitl1_followbox1.json`](../../finals/configs/sitl1_followbox1.json) | sitl | FB-1 warm-up (1 crate + 1 car) |
| [`sitl1_followbox_multi.json`](../../finals/configs/sitl1_followbox_multi.json) | sitl | FB-multi warm-up (3-crate slalom) |
| [`sitl3_vision.json`](../../finals/configs/sitl3_vision.json) | sitl | S-scan 3-drone sentry_scan over the convoy |
| [`sitl_vision.json`](../../finals/configs/sitl_vision.json) | sitl | S-scan single-drone (stageB) |
| [`sitl3_lanes_vision.json`](../../finals/configs/sitl3_lanes_vision.json) | sitl | S-lanes hover+photograph (S11 A) |
| [`sitl3_track_vision.json`](../../finals/configs/sitl3_track_vision.json) | sitl | S-track active chase (S11 B) |
| [`sitl3_dyn3_vision.json`](../../finals/configs/sitl3_dyn3_vision.json) | sitl | S-dyn3 dynamic self-assign (3 cars) |
| [`sitl3_dyn5_vision.json`](../../finals/configs/sitl3_dyn5_vision.json) | sitl | S-dyn5 dynamic self-assign (5 cars / contention) |
| [`sitl3_handover_vision.json`](../../finals/configs/sitl3_handover_vision.json) | sitl | S-handover soft-zone handover (WS-7A) |
| [`sitl.json`](../../finals/configs/sitl.json) | sitl | minimal single-drone SITL smoke (`--profile sitl --dry-run`) |
| [`sitl3.json`](../../finals/configs/sitl3.json) | sitl | 3-drone headless SITL band rehearsal |
| [`mock.json`](../../finals/configs/mock.json) | mock | laptop-only mock flight (no SDK; CI smoke) |
| [`mock_arena.json`](../../finals/configs/mock_arena.json) | mock | mock flight with an arena (NAV dry-run) |
| [`mock_gazebo.json`](../../finals/configs/mock_gazebo.json) | mock | mock flight + gz frames (stageA transport smoke) |
| [`replay.json`](../../finals/configs/replay.json) | replay | laptop-only frame replay (no drones) |

---

## Hard rules (non-negotiable)

1. **Tune config, not code.** Onsite changes are JSON edits. The one sanctioned
   source touch is the documented cm-vs-m unit fix at `pyhulax_adapter.py:move()`
   if B6 proves it — and only with a re-run of `pytest finals/tests` after.
2. **No new behavior in the last 25 minutes.** Late code is untested code.
3. **No multi-drone flight without a proven abort.** B8 + gate G must pass first.
4. **Preflight before EVERY launch.** Battery drains, links drop, state drifts —
   green once is not green forever.
5. **Default-deny.** No GO, wrong word, or timeout at P10 = no arm. The gate
   exists because the audited example armed on invalid input.
6. **Props off until gate G.** Bench (B1–B8) and config (A–G) carry zero flight
   risk by design; keep it that way until the abort is proven on hardware.

---

Back to [`finals/docs/module_map.md`](../../finals/docs/module_map.md) ·
[`docs/finals/README.md`](README.md) ·
[`finals/preflight.py`](../../finals/preflight.py)
