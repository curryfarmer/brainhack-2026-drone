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
