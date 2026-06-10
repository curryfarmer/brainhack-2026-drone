# E2E coverage — what is software-proven vs what only hardware/VM can prove

One screen. The flight-runtime concerns of `finals/` and the test that proves each end to
end, then the gates that *only* the VM or the real cage can close. Full suite: **1689 green**
(`pytest finals/tests -p no:randomly`); `verify_runbook` PASS; all 5 profiles `--dry-run`
resolve; mock + real-fakes missions reach DONE.

## Software-proven now (this box, no drones/VM)

| Flight-runtime concern | Proven end-to-end by |
|---|---|
| **Real-profile MISSION loop** — `_run_mission → _build_agents(pyhulax) → _amain → P0-P10 (incl. operator GO) → orchestrator → DONE`, one shared DroneAPI per drone feeding BOTH flight + video, perception live | `test_real_wiring_e2e.py` *(new this pass)* |
| Real-profile composition + preflight GATE (P0-P9, then disconnect) | `test_orchestrator.py::test_main_real_preflight_only_runs_the_gate` |
| Bench composition (BenchAdapter wraps PyhulaxAdapter, inner-first) + gate | `test_orchestrator.py::test_main_bench_preflight_only_runs_the_gate` |
| Full LANDING mission `[takeoff, navigate, land_on_pad]` → VERIFIED_LANDING (open-loop DR transit + visual servo, position-blind) | `test_nav_e2e.py` |
| Multi-drone deconfliction — staggered launch slots + serialized descent, 3 drones all DONE | `test_nav_e2e.py` (orchestrator e2e) + `test_deconfliction.py` (unit) |
| `land_on_pad` sub-state-machine ACQUIRE→CENTER→DESCEND→COMMIT | `test_land_on_pad.py` (+ `test_servo.py`) |
| Pyhulax adapter (connect, telemetry mapping, command→SDK, dead-flag poller) | `test_pyhulax_adapter.py` + `test_adapter_conformance.py` |
| Pyhulax video source (None-window, channel flip, restart ladder, staleness) | `test_pyhulax_video.py` |
| Perception: frame → marker/YOLO → Sighting → bus → CSV | `test_vision_perception.py`, `test_vision_aruco.py`, `test_vision_detector.py` |
| ArUco field dict (DICT_7X7_1000) + known-coord pads | `test_vision_aruco.py`, `test_pad_validity*.py` |
| Pad detector weights wiring (`models/pad_v1.pt`, ultralytics) | `test_pad_weights_e2e.py` |
| Replay-profile mission (disk frames → DONE) | `test_replay_e2e.py` |
| Config + all 5 profiles load/resolve; convoy + landing real configs | `test_config.py`, `test_landing_config.py`, `test_convoy_config.py` |
| Guards, proximity, dead-reckon, search, navigate, convoy registry/tracking | `test_guards.py`, `test_proximity_guard.py`, `test_dead_reckon.py`, `test_navigate.py`, `test_track_convoy.py`, `test_convoy_registry.py` |

The new `test_real_wiring_e2e.py` closes the one real gap the audit found: the real backend's
**mission loop** was only composed-and-gated (preflight-only), never *flown* in software. It now
runs `main(--profile real)` to DONE over faked pyhulax/Dola/cv2/torch seams — deterministic.

## Only the VM or the cage can prove (NOT runnable here)

These need real renders / real radios — they are the SIM-SITL + ONSITE-HARDWARE smokes, not
re-implemented in the suite. Catalog: `docs/finals/smokes.md`. Runbook: `docs/finals/onsite_test_plan.md`.

| Gate | Where | Why software can't |
|---|---|---|
| L1 / L2 landing SITL (`run_landing.sh`) | VM | Real PX4 flight dynamics + gz renders feeding the visual servo |
| track3 / dyn3 / lanes3 vision SITL (`run_vision.sh`) | VM | Live onboard-camera ArUco decode at the real RTF ceiling |
| P0-P10 hardware gate, B1-B8 bench, abort drills | Cage | pyhulax radio link, real battery/telemetry, real video, operator GO |
| Live flight (`flight_test --live`) | Cage | A HULA actually arming and flying |
