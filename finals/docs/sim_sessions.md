# Sim build-out ladder — SIM-0…SIM-5, one fresh session each

> Created 2026-06-06 from the verified feasibility pass in [`simulation.md`](simulation.md).
> This file is BOTH the plan and the logbook: each part below has a ready-to-paste handover
> prompt, and the session that executes it appends its evidence under **Evidence log** at the
> bottom. A fresh session needs only `module_map.md` + `simulation.md` + this file — no chat
> history.

**Mapping rule**: SIM-1 and SIM-2 execute roadmap **S6** (V1 gate, then the 3× stretch);
SIM-3/4/5 execute **S8** (assets ∥, core, full rehearsal). module_map session numbers are
NEVER renumbered — stub `NotImplementedError` messages reference them. Headless sim is done
after SIM-2; full sim after SIM-5.

## Binding recap (every sim session re-reads this)

1. **PX4 SITL is a physics stand-in** — the finals drones are HULA/pyhulax. Nothing
   PX4-specific may leak above the FlightAdapter; no SITL result retires a HULA-specific risk
   (`simulation.md` fidelity framing).
2. **Root `drone_control.py` is NEVER edited and CANNOT be wrapped for multi-drone**:
   `connect()` hardcodes `udpin://0.0.0.0:14540`, `_kill_stale_servers()` does a GLOBAL
   `pkill -9 -f mavsdk_server` (kills all three drones' servers), and `System()` is built
   internally so the gRPC port can't be injected. SIM-1 REIMPLEMENTS vendored-with-fixes
   (sanctioned by convention 9 "imported or vendored-with-fixes") — the
   `sitl_adapter.py` stub docstring that says "wraps drone_control.Drone.connect()" is STALE
   and overridden here.
3. **Global `pkill -f mavsdk_server` is BANNED while anything runs.** Root `run.sh` does it
   at line 14 — fine as a *launcher* (one process, servers not yet spawned), lethal
   mid-mission. `sim/launch_sitl.sh stop` kills via PID files only; stale-server cleanup is
   always targeted: `pkill -9 -f "mavsdk_server.*-p <grpc_port>"`.
4. **`sim/` (repo root) is outside the conventions scan + SDK whitelist BY DESIGN**
   (`tests/test_conventions.py` walks only `finals/`): raw MAVSDK/cv2/gz/rclpy scripts live
   there. `finals/tools/` IS scanned — matplotlib is fine there, cv2/gz are not.
5. **The raw-MAVSDK harness (`sim/sitl_smoke.py`) is sanctioned ONLY for SIM-0** environment
   validation. From SIM-1 on, every flight goes through `--profile sitl` — the sim flies the
   real mission code or it proves nothing (strategy pillar 2).
6. **Fail-loud applies in `sim/` too**: every wait has a deadline; scripts exit nonzero with
   WHAT / WHICH instance / WHAT-TO-CHECK messages. (And remember for `finals/` work: the
   conventions test greps RAW source — banned phrases stay out of docstrings/comments too.)
7. **Marker duality**: organizer intel says **QR, 20×20 cm**, but literal-QR vs
   loosely-said-ArUco is UNCONFIRMED (module_map open questions). All marker assets and
   detection checks support BOTH types; the px-vs-distance table is reported PER TYPE.
8. **Lockstep RTF vs wall-clock**: an overloaded VM slows sim time but physics stays correct
   — slow runs trip `timeout_s` SPURIOUSLY. Record RTF in every evidence block; the fix is
   config (`command_timeout_s`), never code.
9. **Session bar before commit** (team playbook): pytest green on Windows AND on the VM where
   applicable; adversarial self-review of new modules; mutation kill-check on new tests;
   evidence pasted into this file's log.

## VM access + sync (single source — every prompt points here)

- **VERIFIED VM FACTS (probed over SSH, 2026-06-07 — supersede the stale "24.04 /
  recently flew the qualifier" assumption):** VMware guest on the C2 dev laptop;
  **Ubuntu 22.04.5**, kernel 6.8; user `drone`; **2 vCPU / 7.7 GiB** (bump to 4+ cores in
  VMware settings before SIM-2's 3× swarm — recap §8's RTF-trips-timeouts risk is real at
  2 cores); `~/PX4-Autopilot` built (`build/px4_sitl_default/bin/px4` exists); **Gazebo
  Harmonic 8.11**; GitHub reachable anonymously (repo is public — clone needs no
  credentials); repo NOT yet cloned (SIM-0 does it); passwordless sudo enabled for
  automation.
- **SSH access (Model B — sessions drive the VM directly):** from the C2 laptop,
  `ssh bhvm` (key-auth `Host bhvm` entry in `~/.ssh/config` → `drone@192.168.174.128`).
  NAT/DHCP means the guest IP can drift across VM reboots — if `ssh bhvm` stops resolving,
  re-read `hostname -I` in the VM console and update the HostName line. Keep the VM RUNNING
  (not suspended) during sim sessions.
- **PYTHON GOTCHA (blocks running finals/ on the VM until fixed in SIM-0):** system Python
  is **3.10**, but `finals/guards.py` uses `asyncio.timeout()` (3.11+) — pytest and
  `--profile sitl` crash on 3.10. SIM-0 must install `python3.11` + `python3.11-venv`
  (deadsnakes PPA on 22.04) and build the repo venv on it. Known wrinkle deferred to
  SIM-4: the apt gz Python bindings (`gz.transport13`) are compiled for system 3.10 and
  will NOT import inside a 3.11 venv — irrelevant for headless SIM-0…2; SIM-4
  (gazebo_video) must solve it and record the solution here.
- Sync: **clone once in SIM-0** (`git clone <repo-url> ~/brainhack-2026-drone`), then per
  iteration push from Windows → `git -C ~/brainhack-2026-drone pull` on the VM. Fallback if
  VM git/credentials fail: the ZIP drop-in workflow in `docs/quali/deployment.md`.
- Always invoke scripts as `bash sim/<script>.sh` (exec bits/line endings can be lost via
  Windows; `.gitattributes` pins `*.sh text eol=lf` from SIM-0).
- In-VM run pattern: `source .venv/bin/activate` (SIM-0 verifies/creates it), then
  `./run.sh -m finals.main …` (run.sh is the sanctioned launcher; see recap §3).

## Notes to roadmap sessions (gates owned elsewhere — hand these over verbatim)

- **S4**: besides the run-start initial-pose/origin event already in its row (replay-plot
  prereq), the flight-adapter factory cannot be a bare `flight_cls(drone_id)`:
  `MavsdkSitlAdapter` needs per-drone `(sitl_address, mavsdk_grpc_port)` from config, and
  `BenchAdapter` needs an inner adapter (its docstring already says so). Make the factory
  take `(FinalsConfig, DroneConfig)`.
- **S5**: SIM-2's drills assume guards + the abort listener exist; SIM-4 assumes the
  lost-video guard.
- **S7**: keep the detector seam pluggable (ArUco AND QR — recap §7); SIM-4 wires whichever
  is configured into the convoy world.

## The ladder

| Part | = roadmap | One-line scope | Prereqs | Status |
|---|---|---|---|---|
| SIM-0 | env | VM bring-up: launch/stop scripts, raw smoke 1×+3×, rendering/ros2/resource probes | — | ✅ 2026-06-07 |
| SIM-1 | **S6** | `MavsdkSitlAdapter` (vendored-with-fixes) + per-drone config schema; **VM V1** + kill drill | SIM-0, S4 (S5 recommended) | ✅ 2026-06-07 |
| SIM-2 | S6 stretch | `replay_plot.py` (proven on the SIM-1 fixture) → `sitl3.json` → 3× headless swarm + drills — **HEADLESS SIM DONE** | SIM-1, S5 | ⬜ |
| SIM-3 | S8 assets | Convoy world: markers (ArUco+QR), robots, pads, band-altitude cams; detection check — **∥ with S4–S7** | SIM-0 | ⬜ |
| SIM-4 | **S8** core | `gazebo_video.py` + `search.py`; **V2a**: single drone logs sightings of MOVING markers | SIM-1, SIM-3, S7, S5 | ⬜ |
| SIM-5 | S8 full | 3 camera-drones, full mission, all drills — **FULL SIM DONE** + residual-gaps list for S10 | SIM-2, SIM-4 | ⬜ |

## Smoke matrix — what "thoroughly smoked" means per part

| Part | Automated (pytest) | Scripted smoke (VM) | Manual evidence pasted | Failure/kill drills |
|---|---|---|---|---|
| SIM-0 | full suite green ON THE VM (env proof) | `launch_sitl.sh start 1` + `sitl_smoke.py --instance 0`; then `start 3` + `--all 3` | 3× PASS; `ss -ulpn` 14540–42; RTF; nproc/RAM; GL verdict; llvmpipe camera FPS; ros2/ros_gz verdict | stop-cleanliness (pgrep empty → relaunch); kill one instance, others live, solo relaunch |
| SIM-1 | `_body_offset_to_ned` ≡ DeadReckoner property (any yaw); config schema tests; conventions green | **V1**: `--profile sitl --phases takeoff_demo` | console + mission.jsonl tail; telemetry-vs-DR drift (m); fixture committed | kill-PX4 mid-move-2 → FlightTimeout (stopwatched) + emergency_land + nonzero exit; stale-server recovery + dummy-server-survives proof |
| SIM-2 | replay_plot vs `sim1_v1_square.jsonl` (closed square asserted) | 3× concurrent takeoff_demo via orchestrator | summary 3-ok; per-drone jsonl tails; bands in telemetry; replay PNG committed | kill instance #2 → FAILED + exactly-one emergency_land + others complete; headless `q` abort lands all |
| SIM-3 | suite green UNTOUCHED (proves no `finals/` leak) | `gz sim -s -r` convoy world + `check_detection.py` per band | all convoy+pad IDs read; px-vs-distance table PER MARKER TYPE; annotated frames; FPS per rung | llvmpipe rung exercised for real; two-run determinism |
| SIM-4 | search phase over MockAdapter; topic/conversion units; gz tests skip-if-absent | **V2a**: 1 drone, convoy world, `--phases search` | sightings.csv head + per-ID counts over ≥1 lap; replay plot with bearing rays | kill gz mid-search → video guard fires, no hang; empty-world → zero sightings, clean exit |
| SIM-5 | full suite | full 3-drone mission, `sitl3_vision.json` | summary; every-ID sightings review; 3-track plot; RTF audit: ZERO spurious FlightTimeouts | `q` abort lands 3; kill instance #2; post-run pgrep clean; 2 consecutive runs |

---

## Handover prompts (paste the whole block into a fresh session)

### SIM-0 — environment bring-up

```text
You are doing SIM-0 (sim environment bring-up) for the BrainHack finals repo.

ORIENTATION (read in order): finals/docs/module_map.md (conventions, status);
finals/docs/simulation.md (Tier 1 recipe + rendering ladder); finals/docs/sim_sessions.md
(binding recap §1-9, VM access + sync, smoke matrix — your session contract);
docs/quali/simulator-testing.md + docs/quali/deployment.md (existing VM workflow). Facts: the
VM exists, ~/PX4-Autopilot is built (`make px4_sitl`), it recently flew qualifier missions.

SCOPE (ZERO finals/ changes):
1. .gitattributes at repo root: `*.sh text eol=lf`.
2. sim/README.md — VM runbook: clone/pull sync + ZIP fallback, venv, launch/stop/status
   usage, port map (UDP 14540+i, gRPC 50051+i, MAV_SYS_ID i+1), the global-pkill ban, the
   kill-drill one-liners.
3. sim/launch_sitl.sh — `start N [--world W] [--model M] | stop | status`. start: instance 0
   first (HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,<i>"
   ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i <i>, output to sim/run/px4_<i>.log, PID
   to sim/run/px4_<i>.pid), poll gz-server readiness WITH A DEADLINE before starting
   instances 1+ with PX4_GZ_STANDALONE=1 (simultaneous launch flakes). stop: kill via PID
   files + the gz server it started — NEVER `pkill -f mavsdk_server`. status: pgrep summary
   + `ss -ulpn | grep 1454`. PID files exist precisely so later kill drills are scriptable.
4. sim/sitl_smoke.py — raw-MAVSDK env validation ONLY (sanctioned for SIM-0, banned after:
   recap §5): `--instance N` → mavsdk System(port=50051+N) →
   connect("udpin://0.0.0.0:" + str(14540+N)) → health-ready poll → arm → takeoff ~1.5 m →
   poll altitude ≥1.2 m → land → poll landed → print "PASS instance N". `--all K` runs K
   concurrently (asyncio.gather). EVERY wait has a deadline and a WHAT/WHICH/CHECK failure
   message; exit nonzero on any failure.
5. Probes (record verdicts in the evidence block — later parts consume them):
   a. GL: `glxinfo -B` (OpenGL version/renderer).
   b. Camera rendering: launch the proven qualifier camera model (make px4_sitl
      gz_x500_vision) or `gz sim -s -r` any camera world; measure image-topic FPS with the
      existing gz.transport13 subscriber pattern; repeat under LIBGL_ALWAYS_SOFTWARE=1;
      record the rendering-ladder verdict (which rung the VM supports).
   c. ros2 + ros_gz presence: `which ros2`, `ros2 pkg list | grep ros_gz`. Verdict decides
      SIM-3's convoy driver: present → rclpy waypoint node via ros_gz; absent → gz-native.
   d. Resources: nproc, free -h, and RTF lines from the px4 logs while 3 instances run.

SMOKE (paste everything into sim_sessions.md → Evidence log → SIM-0):
- VM sync: clone, venv, `python -m pytest finals/tests -q` green ON THE VM.
- `bash sim/launch_sitl.sh start 1` → `python sim/sitl_smoke.py --instance 0` → PASS.
- Drill: `bash sim/launch_sitl.sh stop` → `pgrep -fa 'px4|gz sim'` empty → restart works.
- `bash sim/launch_sitl.sh start 3` → `ss -ulpn` shows 14540/14541/14542 →
  `python sim/sitl_smoke.py --all 3` → three concurrent PASSes.
- Drill: kill instance 1 via its PID file; instances 0/2 still answer; solo relaunch of 1.
- All probe outputs + verdicts (a–d).

DONE WHEN: evidence appended to sim_sessions.md, its ladder Status flipped, module_map
`sim/` row flipped, committed + pushed. finals/ untouched (pytest proves it).

▎ ADDENDUM: the VM is reachable as ssh bhvm from this laptop (key auth + passwordless sudo — drive it directly; verified facts and the
  ▎ Python-3.10 gotcha are in sim_sessions.md "VM access + sync"). SIM-0 must also: install python3.11 + python3.11-venv (deadsnakes PPA,
  ▎ Ubuntu 22.04) and build the repo venv on 3.11 before the on-VM pytest gate.
```

### SIM-1 — MavsdkSitlAdapter + VM gate V1 (= roadmap S6)

```text
You are doing SIM-1 = roadmap S6: the SITL flight adapter and the first real-mission sim
flight (VM gate V1).

ORIENTATION (read in order): finals/docs/module_map.md (S6 row + conventions — NOTE the
conventions test greps RAW source: banned phrases stay out of docstrings too);
finals/docs/simulation.md Tier 1; finals/docs/sim_sessions.md (binding recap — especially
§2/§3 —, SIM-0 evidence: ports/RTF/resources, and "Notes to roadmap sessions" to see how the
S4 factory landed); finals/flight/adapter.py (the ABC contract is BINDING);
finals/flight/dead_reckon.py (frame convention: yaw CCW+, psi_NED = -yaw_deg) +
finals/flight/mock_adapter.py; finals/errors.py; finals/config.py; finals/main.py +
finals/mission/ as landed in S4; root drone_control.py, get_position_with_task.py,
qualifier_run.py:268-331 (PROVEN sources — read and adapt, never edit).

BINDING EMPHASIS — the sitl_adapter.py stub docstring is STALE where it says to wrap
drone_control.Drone.connect() incl. _kill_stale_servers. Do NOT wrap drone_control.Drone:
its connect() hardcodes udpin://0.0.0.0:14540, its _kill_stale_servers() pkill-9's EVERY
mavsdk_server on the box (destroys the other two drones), and it builds System() internally
so the gRPC port can't be injected. REIMPLEMENT vendored-with-fixes (convention 9), listing
every fix in the module docstring (convention 7):
- parameterized per-drone (sitl_address, mavsdk_grpc_port);
- targeted stale-server cleanup: pkill -9 -f "mavsdk_server.*-p <grpc_port>" (prove the
  targeting in the drill below);
- EKF/health poll WITH DEADLINE before arming (multi-instance load slows EKF settle;
  drone_control arms blind);
- telemetry-polled takeoff to >=0.9x target altitude (replaces the blind 20 s sleep);
- move(): read NED+yaw from the telemetry poller (get_position_with_task.py pattern), then
  _body_offset_to_ned(direction, distance_cm, yaw_deg) -> NED waypoint, then the PROVEN
  10 Hz setpoint loop (qualifier_run.py:268-331) WITH a hard deadline -> FlightTimeout
  naming drone, waypoint, elapsed, what-to-check;
- rotate(): the proven yaw PID + deadline; land(): poll in_air False + disarm + deadline;
- cm->m at this boundary; Telemetry altitude is UP-POSITIVE (negate down_m);
- _body_offset_to_ned is a PURE function importable WITHOUT mavsdk (keep mavsdk imports
  method-local so Windows pytest never needs the SDK).

TESTS: hypothesis property — for arbitrary yaw and every Direction, _body_offset_to_ned's
(dN,dE) == DeadReckoner's documented deltas under psi_NED = -yaw_deg — plus a hand-computed
grid (yaw 0/30/90/-120). Add `hypothesis` to requirements.txt (test-only). Config schema:
per-drone OPTIONAL sitl_address + mavsdk_grpc_port on DroneConfig; sitl profile with >1
drone validates DISTINCT addresses, ports, altitude_band_m; single drone falls back to
top-level sitl_address + 50051. Update configs/sitl.json _comment only. Verify the S4
factory constructs this adapter from (FinalsConfig, DroneConfig); widen it if S4 left it
narrower.

SMOKE:
- `python -m pytest finals/tests -q` green on Windows AND the VM.
- VM gate V1: push → pull on VM → `bash sim/launch_sitl.sh start 1` →
  `./run.sh -m finals.main --profile sitl --phases takeoff_demo` → takeoff → square → land.
  Paste: console, `tail -20` of the run's mission.jsonl, and the final telemetry-NED vs
  DeadReckoner-pose drift in metres (one number — it calibrates how honest DR is on a real
  flight path).
- Copy that run's mission.jsonl to finals/tests/fixtures/sim1_v1_square.jsonl and COMMIT it
  (runs_finals/ is gitignored; SIM-2's plotter is pytest-validated against this fixture).

DRILLS (paste evidence):
- Kill-PX4 mid-move: fresh V1 run; second terminal waits for the 2nd move event
  (`tail -f` the run's mission.jsonl) then `kill -9 $(cat sim/run/px4_0.pid)`. MUST:
  FlightTimeout with its actionable message within the command timeout (stopwatch it),
  emergency_land attempted + logged, process exits NONZERO, no hang.
- Stale-server: rerun V1 immediately after the kill with NO manual cleanup — connect()
  recovers via the targeted cleanup. Then prove targeting: start a dummy
  `mavsdk_server -p 50052` and show it SURVIVES alpha's cleanup.

DONE WHEN: V1 + drill evidence in sim_sessions.md, fixture committed, module_map S6 row
flipped to "✅ SIM-1 (3x = SIM-2)", ladder Status flipped, adversarial self-review +
mutation kill-check done, committed + pushed.
```

### SIM-2 — 3-drone headless swarm + replay tool (= S6 stretch) → HEADLESS SIM DONE

```text
You are doing SIM-2 = roadmap S6 stretch: the replay evidence tool, then the first 3-drone
headless swarm flight.

ORIENTATION (read in order): finals/docs/module_map.md; finals/docs/simulation.md Tier 0
(replay-plot spec) + Tier 1; finals/docs/sim_sessions.md (SIM-0 resource baseline, SIM-1
evidence + fixture); finals/flight/dead_reckon.py (the REAL math — the plotter IMPORTS it,
never reimplements); finals/events.py (read_events + the S4 origin event); finals/guards.py
+ the abort listener as landed in S5; finals/configs/sitl.json.

SCOPE, IN THIS ORDER (the evidence tool is proven BEFORE the flights it must judge):
1. finals/tools/__init__.py (empty) + finals/tools/replay_plot.py (~150 lines): parse a run
   dir's mission.jsonl; feed each drone's COMPLETED command events through
   finals.flight.dead_reckon.DeadReckoner; plot east-on-X / north-on-Y, set_aspect('equal'),
   yaw quivers, sighting points + bearing rays when present. Per-drone NED origins DIFFER
   (spawn poses) — one subplot per drone, or offset tracks by the spawn poses documented in
   the config _comment. matplotlib ONLY (finals/tools/ is inside the conventions scan; cv2
   and gz are forbidden there). CLI: python -m finals.tools.replay_plot <run_dir>
   [--save out.png].
2. pytest: plotter over finals/tests/fixtures/sim1_v1_square.jsonl → track closes (square
   within tolerance) and a PNG writes to tmp_path. A commanded right-turn square rendering
   as left turns is a sign bug — that is exactly what this test exists to catch.
3. finals/configs/sitl3.json: alpha/bravo/charlie; sitl_address udpin://0.0.0.0:14540/41/42;
   mavsdk_grpc_port 50051/52/53; altitude_band_m 1.2/1.7/2.2; phases takeoff_demo;
   frame_backend none; detector none; spawn poses ("0,0","0,1","0,2") in _comment matching
   sim/launch_sitl.sh.
4. The 3-drone run.

SMOKE:
- pytest green Windows + VM.
- `bash sim/launch_sitl.sh start 3` →
  `./run.sh -m finals.main --profile sitl --config finals/configs/sitl3.json` → all three
  agents complete. Paste: orchestrator summary, per-drone mission.jsonl tails, altitude
  bands visible in telemetry events. Replay plot → commit as
  finals/docs/evidence/sim2_3drone.png. Record RTF + wall time vs the SIM-0 baseline.

DRILLS (paste evidence):
- Kill instance #2 mid-mission: `kill -9 $(cat sim/run/px4_1.pid)` → bravo FAILED with
  exactly ONE emergency_land event (grep -c it), alpha + charlie COMPLETE, summary shows
  2 ok / 1 failed, exit code reflects partial failure.
- Headless abort: fresh 3x run, press 'q' → all three land, orderly shutdown + summary.
- Optional flavor: kill bravo's mavsdk_server instead (different error path — must still end
  FAILED, never a hang).

DONE WHEN: "HEADLESS SIM DONE" recorded in the evidence log with all of the above, PNG +
fixture-based test committed, module_map + ladder flipped, review + mutation check, pushed.

▎ ADDENDUM (facts from SIM-0/SIM-1 — supersede stale assumptions):
▎ - BEFORE STARTING: bump the VM to 4+ vCPUs (currently 2 — power it off in VMware, change
▎   settings, boot; NAT IP may drift: if `ssh bhvm` fails, read `hostname -I` in the VM
▎   console and update HostName in ~/.ssh/config). Record nproc in the evidence.
▎ - The VM is driven directly over `ssh bhvm` (key auth + passwordless sudo). Repo at
▎   ~/brainhack-2026-drone, venv `.venv` on python3.11 (lean set + hypothesis + matplotlib
▎   installed). Run pattern: `source .venv/bin/activate` then `./run.sh -m finals.main ...`.
▎ - PLOTTER MUST SEED DR FROM THE origin EVENT — EKF boot yaw is NOT 0 (the committed
▎   fixture's origin has yaw_deg=-95.97). Assuming yaw 0 renders the square rotated; the
▎   fixture test must still assert closure, which is yaw-invariant.
▎ - sitl3.json schema is ALREADY validated + tested: THREE_DRONES in
▎   finals/tests/test_sitl_adapter.py is the exact template (distinct addresses/ports/bands
▎   enforced). Carry command_timeout_s 30 over from sitl.json (recap §8).
▎ - The 'q' abort drill NEEDS A TTY: over plain ssh the AbortListener logs "stdin EOF —
▎   abort key disabled". Run that drill via `ssh -t bhvm` (or the VM console).
▎ - Kill-drill physics (SIM-1-proven): killing a px4 leaves its mavsdk_server ALIVE → the
▎   adapter's STALENESS detector fires (~1.2 s typed FlightError); killing the mavsdk_server
▎   instead → stream-END dead-flag path (the "optional flavor" drill). Killing px4_0 may
▎   take the gz server down with it — `start N` recovers state-aware.
▎ - VM pytest shows 1 pre-existing failure: test_budget_expiry_lands_all_and_exits_clean
▎   (S4-owned platform race, SIM-0/SIM-1 evidence) — the gate is "no NEW failures".
▎ - In ssh one-shot cleanliness checks use the bracket form `pgrep -fa 'p[x]4|g[z] sim'`
▎   (the plain pattern self-matches your own wrapper shell — sim/README.md documents it).
▎ - finals/docs/evidence/ does not exist yet — create it for sim2_3drone.png.
```

### SIM-3 — convoy world assets + detection check (= S8 assets; ∥ with S4–S7)

```text
You are doing SIM-3 = S8 world assets: a Gazebo world with the moving marker convoy —
gz-only, NO PX4, NO flight, NO finals imports. Parallelizable: it needs only SIM-0.

ORIENTATION (read in order): finals/docs/simulation.md Tier 2 (marker/convoy/camera recipes;
corrections #1 actors-DO-work and #4 evidence repos); finals/docs/sim_sessions.md (binding
recap §7 marker duality; SIM-0 rendering + ros2 probe verdicts — they pick your convoy
driver); finals/docs/module_map.md open questions (markers may be QR 20x20 cm; QR-vs-ArUco
UNCONFIRMED); the qualifier gz.transport13 subscriber pattern (root depth_receiver.py and
docs/quali/simulator-testing.md sensor topics).

SCOPE (all under sim/):
1. sim/gen_markers.py: --type aruco|qr --size-cm (default 20) --ids ... . ArUco:
   cv2.aruco.generateImageMarker(DICT_6X6_250, id, 800) + copyMakeBorder >=1-module white
   quiet zone. QR: cv2.QRCodeEncoder with the ID string as payload (fallback: the `qrcode`
   pip package if the encoder is missing from the cv2 build). Output one gz model dir per
   marker (model.config + model.sdf: STATIC plane sized to --size-cm at z=0.001, the
   pbr/metal/albedo_map pattern from simulation.md Tier 2). Generate convoy IDs in both
   types + 2 pad markers (separate --size-cm for pads).
2. sim/models/convoy_robot/: RoboMaster-ish chassis (~0.4x0.3x0.25 m) with a top-facing
   marker plane. Driver per the SIM-0 verdict:
   - ros_gz PRESENT: spawn via `ros2 run ros_gz_sim create`, drive with
     sim/convoy_driver.py — an rclpy node publishing Twist per robot along a configurable
     waypoint loop, bridged with `ros2 run ros_gz_bridge parameter_bridge
     /model/<n>/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist`.
   - ros_gz ABSENT: gz-native — VelocityControl plugin circles (initial_linear +
     initial_angular, 5 phase-offset spawns) and/or /world/<w>/set_pose scripting for
     deterministic scenarios.
3. sim/worlds/convoy.sdf: ground + sun + 5 convoy robots (phase-offset along the route) +
   2 landing-pad markers + 3 STATIC downward 640x480 cameras at 1.2 / 1.7 / 2.2 m above the
   route (the altitude bands — yields px-vs-distance data with zero flight variables).
4. sim/models/mono_cam_640/: stock mono_cam clone at 640x480 (keep HFOV, note its value in
   an SDF comment — SIM-4 copies it into config camera_hfov_deg).
5. sim/check_detection.py: gz.transport13-subscribe to the 3 static cameras; per frame run
   RAW cv2.aruco.detectMarkers AND cv2.QRCodeDetector.detectAndDecode (the finals detector
   wrapper arrives in S7 — this script validates the WORLD, not the package); log per-ID
   read counts + marker px sizes; save sample annotated frames.

SMOKE (paste into the evidence log):
- `gz sim -s -r sim/worlds/convoy.sdf` at the SIM-0-verdict rendering rung; >=1 full convoy
  lap observed per camera.
- ALL convoy + pad IDs read by at least one band camera (record which bands read which).
- px-vs-distance table PER MARKER TYPE (expect the QR decode floor to be much closer than
  ArUco — this table directly feeds the sentry-altitude/standoff decision and the
  QR-vs-ArUco question in module_map).
- FPS per rendering rung, including one REAL llvmpipe run; two-run determinism (same lap →
  same ID set).

DONE WHEN: assets + table + annotated frames committed (frames under
finals/docs/evidence/), an explicit "what this does NOT validate" note in the evidence block
(HULA HFOV, real-world read range, motion blur — the onsite list), `python -m pytest
finals/tests -q` STILL green and finals/ untouched, module_map + ladder flipped, pushed.
```

### SIM-4 — gazebo video + search phase, single-drone vision-in-the-loop (= S8 core)

```text
You are doing SIM-4 = roadmap S8 core: the Gazebo video backend + the search phase, then the
first vision-in-the-loop sim flight (gate V2a).

ORIENTATION (read in order): finals/docs/module_map.md (S8 row + conventions);
finals/docs/simulation.md Tier 2 (camera topics, channel order);
finals/docs/sim_sessions.md (SIM-1 V1 evidence, SIM-3 world + detection table);
qualifier_run.py:163-186 (RgbReceiver contract) + root depth_receiver.py (gz subscriber
pattern); finals/vision/video.py (ABC) + finals/vision/{aruco,perception}.py as landed in S7
(the detector seam is pluggable ArUco/QR); finals/guards.py lost-video guard (S5);
finals/configs/sitl.json.

SCOPE:
1. finals/vision/gazebo_video.py (gz import is whitelisted in SDK_ALLOWED; its tests
   skip-if-absent — Windows has no gz): VideoSource implementation; deterministic per-drone
   topic /world/<w>/model/<model>_<N>/link/camera_link/sensor/camera/image; R8G8B8 → the
   configured video_channel_order; staleness tracking + timeout_s per convention 2; NO
   silent auto-reconnect — surface the failure to the guard.
2. finals/mission/phases/search.py: SentryScan (hover → look → rotate step → repeat; all
   tunables from config: step_deg, dwell_s, cycles); OpenLoopLawnmower config-gated, OFF by
   default.
3. sim/: a PX4-spawnable x500 + mono_cam_640 model assembled from SIM-3 parts (verify
   PX4_SIM_MODEL spawning yields the deterministic camera topic name);
   sim/launch_sitl.sh gains --world/--model passthrough if SIM-0 didn't already include it.
4. configs/sitl.json: frame_backend "gazebo" for this run; camera_hfov_deg copied FROM the
   camera SDF (bearing math needs the true value).

SMOKE:
- pytest Windows + VM: search phase unit-tested over MockAdapter with canned Sightings;
  topic-name builder + channel conversion unit-tested; gz-dependent tests skip cleanly on
  Windows.
- V2a gate (VM): `bash sim/launch_sitl.sh start 1 --world convoy` →
  `./run.sh -m finals.main --profile sitl --phases search` → sightings.csv accumulates
  MOVING-marker reads across >=1 convoy lap. Paste: per-ID counts + `head` of
  sightings.csv. Replay plot with bearing rays → finals/docs/evidence/sim4_search.png.
- Record RTF (1 camera) vs the SIM-0/SIM-3 numbers.

DRILLS (paste evidence):
- Kill gz mid-search (kill the gz server PID): the lost-video guard fires within its
  deadline; the agent takes the configured response; NO hang.
- Empty-world control: same phases in a marker-less world → ZERO sightings, clean
  completion (no false positives from the rendered scene).

DONE WHEN: V2a + drill evidence + PNG committed, module_map S8 row part-flipped (SIM-4 done,
SIM-5 pending), ladder flipped, review + mutation check, pushed.
```

### SIM-5 — full rehearsal: 3 camera-drones, full mission → FULL SIM DONE

```text
You are doing SIM-5 = the full-sim rehearsal: 3 camera-drones flying the full mission in the
convoy world, all drills mandatory. The ladder ends here.

ORIENTATION (read in order): finals/docs/sim_sessions.md — ALL prior evidence blocks
(especially SIM-0 resource baseline + rendering verdict, SIM-3 FPS table, SIM-2/SIM-4
gates); finals/docs/simulation.md (rendering ladder, Tiers 1-2);
finals/docs/module_map.md (onsite hard rules — "tune config, not code" applies to sim
rehearsals too).

SCOPE:
1. finals/configs/sitl3_vision.json: sitl3.json + frame_backend "gazebo" + search phases +
   camera_hfov_deg + command_timeout_s SIZED FROM MEASURED RTF at the chosen rendering rung
   (recap §8: lockstep means slow ≠ wrong, but slow trips wall-clock timeouts — fix by
   config, never code).
2. Pick the highest rendering rung that holds 3 cameras (SIM-0/SIM-3 data decides; ladder in
   simulation.md: real GL → llvmpipe → --headless-rendering → WSL2).
3. Full mission: 3 drones take off to their bands → search the convoy world → sightings
   accumulate → land. Within budget_s.

SMOKE:
- Full run end-to-end: orchestrator summary 3 ok. Sightings review: EVERY convoy ID read at
  least once; repeats logged, never deduped. Replay plot, 3 tracks (spawn-pose offsets) →
  finals/docs/evidence/sim5_full.png. RTF + timeout audit: ZERO spurious FlightTimeouts (if
  any: resize command_timeout_s and rerun — config only; paste both runs).
- TWO consecutive full runs with no manual cleanup between (launch stop/start only).

DRILLS (ALL mandatory, paste evidence):
- 'q' abort mid-search → all three land, orderly exit.
- Kill instance #2 mid-search (`kill -9 $(cat sim/run/px4_1.pid)`) → bravo FAILED +
  exactly-once emergency_land; alpha + charlie complete the mission.
- Post-run cleanliness: `bash sim/launch_sitl.sh stop` → pgrep empty.

DONE WHEN: "FULL SIM DONE" recorded in the evidence log, PLUS a WRITTEN residual-gaps list:
start from simulation.md "what simulation can NEVER answer", confirm/expand from what these
runs showed — that list is the direct input to S10's docs/onsite_test_plan.md. Ladder +
module_map flipped, review + mutation check, pushed.
```

---

## Evidence log (each session appends under its heading; this doc is the logbook)

### SIM-0 — DONE 2026-06-07

**Deliverables**: `.gitattributes` (`*.sh text eol=lf`), `sim/README.md` (runbook),
`sim/launch_sitl.sh` (`start N [--world W] [--model M] | stop | status`; state-aware start =
the solo-relaunch path), `sim/sitl_smoke.py` (raw-MAVSDK, SIM-0-only per recap §5),
`sim/run/` gitignored. finals/ untouched — suite green on both OSes, zero finals/ code diff.

**VM env (one-time)**: deadsnakes PPA → python3.11.15 + python3.11-venv (jammy archive only
carries 3.11.0~rc1 — PPA mandatory) + mesa-utils. Repo cloned to `~/brainhack-2026-drone`;
venv `.venv` on 3.11 with the LEAN set (`pytest mavsdk "numpy<2" opencv-python matplotlib
scipy PyYAML Pillow pymavlink`) — torch/ultralytics/jupyter deliberately deferred (YOLO is
config-off, CannedDetector is torch-free, disk at 78%; decision OK'd by user). NOTE: pytest
is NOT in requirements.txt — the venv recipe in `sim/README.md` adds it explicitly.

**Pytest gate (VM, 3.11 venv)**: 424/425 stable; **one PRE-EXISTING platform-timing race
found** (not a SIM-0 regression — first failure occurred on the unmodified `5539c5c` clone;
SIM-0 changes zero finals/ code):
`test_orchestrator.py::test_budget_expiry_lands_all_and_exits_clean` — **Windows 10/10 pass
(isolation), VM Linux/3.11 0/10 fail (isolation)**, though it DID pass twice on the VM early
in the session (full run `425 passed in 15.11s` + one isolation pass) — flaky-then-sticky.
Failure mode: with `budget=0s` the Mock mission completes in ~0.2 s and agents reach DONE
before the orchestrator emits `budget_expired` (`ticks=7`, event list has the full
phase/action sequence, no `budget_expired`) — an agent-completion vs budget-evaluation
ordering race that faster Linux asyncio timing exposes. **→ owner: S4/orchestrator** (fix
the loop ordering or the test's race assumption; do NOT band-aid by widening budget).
Windows full suite: `425 passed in 23.72s`.

**Smoke matrix** (all on the VM):

```text
bash sim/launch_sitl.sh start 1
  gz server ready after 6s; instance 0 mavlink up (UDP 14580 bound)
python sim/sitl_smoke.py --instance 0
  instance 0: connected on 14540 (gRPC 50051) / health ready / altitude >= 1.2 m
  PASS instance 0                                              EXIT=0

DRILL stop-cleanliness:
bash sim/launch_sitl.sh stop                                   STOP_EXIT=0
pgrep -fa 'p[x]4|g[z] sim'  ->  (empty — clean); restart works (start 1 again: 5s)

bash sim/launch_sitl.sh start 3
  PX4-bound ports: 14580 14581 14582 18570 18571 18572
python sim/sitl_smoke.py --all 3
  PASS instance 0 + PASS instance 1 + PASS instance 2 (concurrent, gather)
  ALL 3 instances PASS                                         SMOKE_ALL3_EXIT=0
ss -uapn mid-run: UNCONN 0.0.0.0:1454x users:(("mavsdk_server",pid=...)) — the 1454x
  ports are bound by mavsdk_server for the whole client session (see port correction below)

DRILL kill instance 1:
kill -9 "$(cat sim/run/px4_1.pid)"
  python sim/sitl_smoke.py --instance 0  ->  PASS (EXIT=0)
  python sim/sitl_smoke.py --instance 2  ->  PASS (EXIT=0)
bash sim/launch_sitl.sh start 3            # the solo relaunch
  instance 0 already running — skipping
  model x500_1 already in the world — attaching instead of spawning   <- PX4_GZ_MODEL_NAME
  instance 1 mavlink up (UDP 14581 bound) after 4s
  instance 2 already running — skipping
  python sim/sitl_smoke.py --instance 1  ->  PASS (EXIT=0)
```

**Drill-earned fixes** (the drills caught both — they are the tests for `sim/`):
1. A name-based `pkill -9 -f "gz sim"` sweep in `stop` killed the operator's own ssh shell
   (any `bash -c` wrapper quoting the pattern matches it). `stop` now kills strictly by
   recorded PIDs; leftover checks use process-specific patterns (`bin/px4 -i|gz sim --`);
   one-shot ssh checks use the bracket form `pgrep -fa 'p[x]4|g[z] sim'` (README documents).
2. Solo relaunch must ATTACH, not respawn: a killed px4 leaves its `x500_<i>` model in the
   world; `start` now detects live gz + surviving model and uses `PX4_GZ_MODEL_NAME` (attach)
   instead of `PX4_GZ_MODEL_POSE` (spawn would name-collide).

**Probe verdicts** (consumed by later parts):

- **a. GL**: `DISPLAY=:0 glxinfo -B` → VMware SVGA3D, **OpenGL 4.3 core, Mesa 23.2.1,
  Accelerated: no** (LLVM-backed paravirt — software-class either way). Clears ogre2's
  GL 3.3 floor.
- **b. Camera rendering (rendering-ladder verdict: TOP RUNG HOLDS — VM "real GL"/SVGA3D)**:
  `start 1 --model gz_x500_vision --world roboverse` (the proven qualifier combo; world at
  `~/PX4-Autopilot/Tools/simulation/gz/worlds/roboverse.sdf`):
  IMX214 topic `/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image` →
  **29.8 FPS @ 1920×1080**, RTF 1.00 while rendering. Under `LIBGL_ALWAYS_SOFTWARE=1`
  (llvmpipe rung): **26.5 FPS @ 1920×1080**, RTF 0.94–1.18. Both rungs hold a 1080p camera at
  ~sensor rate with 1 drone; 3×640×480 (SIM-3's static-cam world) should be comfortably
  cheaper per-frame — SIM-3 measures it per rung as planned.
- **c. ros2 + ros_gz: PRESENT** (corrects the handover's "probably absent" guess —
  `which ros2` fails only unsourced): ROS 2 **Humble** at `/opt/ros/humble` (+ `~/ros2_ws`
  with open_vins), `ros-humble-ros-gzharmonic{,-bridge,-image,-interfaces,-sim}` 0.244.12
  installed; sourced `ros2 pkg list | grep ros_gz` → ros_gz, ros_gz_bridge, ros_gz_image,
  ros_gz_interfaces, ros_gz_sim(+demos). **SIM-3 convoy driver verdict: rclpy waypoint node
  via ros_gz** (the gzharmonic variant packages — note the names).
- **d. Resources**: nproc 2; RAM 7.7 GiB (1.2 used / 6.2 available with 3 instances —
  headless RAM is a non-issue); disk 11 GB free (78% used). **RTF ≈ 0.99–1.02 during the 3×
  concurrent flight** (from `/world/default/stats`; px4 logs carry no RTF lines), at load
  average 4.5–5.0 on 2 vCPU — lockstep absorbed the oversubscription headless. Still bump to
  4+ vCPU before SIM-2 (margin for orchestrator + 3 mission loops + plotting).

**Environment corrections pinned by SIM-0** (supersede earlier assumptions):
1. **Port scheme**: PX4 BINDS `14580+i` (offboard-local) + `18570+i` (GCS); it SENDS to
   `14540+i` — `udpin` clients (mavsdk_server) bind 1454x, so `ss -ulpn | grep 1454` is
   only non-empty while a client session runs. Verified in
   `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink:4-5`. The smoke-matrix
   line "ss shows 14540/41/42" is satisfied DURING `--all 3` (mavsdk_server UNCONN sockets).
2. **gz python on system 3.10 needs `PYTHONNOUSERSITE=1`**: a pip-user protobuf 7.34.1 in
   `~/.local` shadows apt protobuf 3.12 and breaks the apt-generated `_pb2` modules
   ("Descriptors cannot be created directly"). Binding for SIM-3's `check_detection.py`.
3. **IMX214 (x500_vision) is 1920×1080@30**, not 640×480 — SIM-3's `mono_cam_640` clone
   remains the right move for the pyhulax frame contract.
4. Stock worlds on the VM include `aruco.sdf` and `roboverse.sdf` (PX4 worlds dir);
   `~/worlds/` additionally has `aprilworld.sdf`/`apriltag.sdf` from the quali era.
5. System pip3 (3.10) already carries mavsdk 3.15.3 / numpy 1.26.4 / pytest 6.2.5 — that is
   how the qualifier flew (no venv existed). The finals stack ignores it; the 3.11 venv is
   the only sanctioned interpreter for repo code on the VM.

**Review bar**: adversarial self-review done on both scripts (every wait deadlined; failures
name WHAT/WHICH/CHECK; stop never touches mavsdk_server; PID lifecycle clean after kill -9 —
the two drill-earned fixes above came out of it). Mutation kill-check N/A: no new
pytest-covered code (sim/ is validated by the live drills, by design).

### SIM-1 — DONE 2026-06-07 (commit 5ee2bde + this evidence commit)

**Deliverables**: `finals/flight/sitl_adapter.py` (vendored-with-fixes, recap §2 honored — no
drone_control wrapping; every fix in the module docstring), `finals/config.py` per-drone
`sitl_address`/`mavsdk_grpc_port` + sitl multi-drone validation + `resolve_sitl_endpoint`,
`main._build_adapter` mavsdk_sitl branch, `configs/sitl.json` `command_timeout_s: 30`,
`tests/test_sitl_adapter.py` (52 tests), `hypothesis` in requirements.txt, whitelist widening
(conventions bullet 1 + test_conventions.py — reviewed), fixture
`finals/tests/fixtures/sim1_v1_square.jsonl` committed (SIM-2's plotter gate input).

**Design decisions (user-approved deviations from this handover)**: rotate() via
position-setpoint YAW, not the velocity PID (single setpoint type; PX4 slews yaw
MPC_YAWRAUTO_MAX; shortest-arc caveat documented; the vendored PID re-subscribed telemetry
every 0.1 s); verified fact that reshaped the design: **mavsdk_server auto-resends the last
setpoint at 20 Hz** → NO background streamer task; idle gaps cannot trip offboard-loss while
the server lives.

**Review bar**: adversarial review found 2 MAJORs, fixed pre-commit (unbounded
`System.connect()` could hang past the adapter's own deadline; takeoff returning at the 0.9×
poll threshold let the NEXT move re-anchor to the shortfall and fly the mission below band)
+ 8 minors (fixed: reconnect state reset, land-loop staleness, rate-setter deadline clamps,
stream-end fallthroughs, top-level sitl_address validation, emergency-land budget fit,
`_bounded` on every unary SDK await, NED-negation snapshot test). Mutation kill-check:
6/6 KILLED in a HEAD-cut worktree (sin flip, RIGHT/LEFT swap, cm→m drop, rotate sign,
fallback port, deleted distinct-ports validation).

**Pytest**: Windows 478 passed (no mavsdk installed — proves method-local imports).
VM 477 passed + the KNOWN pre-existing budget-expiry race (SIM-0 evidence; S4-owned;
no new failures). The race also reproduced ONCE on Windows under session load —
strengthens the S4 flag.

**V1 gate (VM)** — `bash sim/launch_sitl.sh start 1` →
`./run.sh -m finals.main --profile sitl --phases takeoff_demo`:

```text
[MavsdkSitlAdapter] alpha: connected in 1.7 s (gRPC 50051)
[MavsdkSitlAdapter] alpha: health ready in 1.2 s
[MavsdkSitlAdapter] alpha: airborne at 0.75 m in 12.9 s, offboard active
4 x [ move(FORWARD, 100 cm) 1.7-2.0 s | rotate(90 deg) 1.0-1.7 s ]
[MavsdkSitlAdapter] alpha: landed + disarmed in 4.5 s
MISSION SUMMARY  elapsed=34.0s  ticks=34   alpha DONE 1/1   exit 0
```

mission.jsonl: full action_start/action_complete stream (enums by NAME), Land routed
`route="safety"` (S5 controller), `agent_done` → `run_end exit_code 0`; fault.txt empty.
Run dir 20260607_134322 = the committed fixture. Health-ready was 1.2 s here (instance had
settled during launcher startup); the 30 s `command_timeout_s` covers the
freshly-booted-instance case (EKF settle 10–25 s) per recap §8 — keep, do not shrink.

**Battery scale verified**: heartbeat `battery_pct: 52.0` → mavsdk 3.15 `remaining_percent`
is 0–100 (not 0–1). SITL battery drains fast (52 % after ~3 missions) — irrelevant for
gates, relevant for long soak runs.

**Drift (DR honesty calibration)**: origin (measured) N=0.025 E=-0.042 yaw=-95.97° (EKF
boot heading ≠ 0 — the contract frame is per-boot; DR seeded from the origin event). DR
final == origin (the square closes exactly); passive one-shot MAVSDK read after the run
(measuring tape, not flight validation — recap §5): N=0.010 E=-0.030 →
**|measured − DR| = 0.019 m horizontal** over 11 actions. Final measured yaw −90.06° vs
−95.97° boot: ≈6° EKF heading shift across the flight (4 rotates at 2° tolerance + EKF) —
the number to beat in SIM-2's 3× runs.

**Kill drill (kill -9 px4_0 at the 2nd Move)**:
- Typed failure in **1.22 s** (≪ the 30 s timeout): `FlightError: move(FORWARD, 100 cm)
  aborted — telemetry is STALE (age 1.09 s > 1.00 s) — stream stalled; check ...` — the
  STALENESS detector, exactly as designed: killing PX4 leaves mavsdk_server alive, so
  streams go QUIET (no end, no exception); the dead-flag path covers server death instead.
- emergency_land EXACTLY ONCE (`grep -c` = 1), `hung: false`; its offboard.stop/land/disarm
  refusals each traceback-logged by the whitelisted swallows (mavsdk `TIMEOUT` ActionError —
  PX4 is gone). kill→exit 17.4 s (the bounded emergency tail), **exit code 1**, no hang.
- Observed: the gz server died with the px4_0 kill on this stack; the launcher's
  state-aware `start 1` took the fresh-server path and recovered cleanly.

**Stale-server drill (manufactured, run WITHOUT run.sh so the ADAPTER's cleanup is what's
proven)**: planted a squatter `mavsdk_server -p 50051 udpin://0.0.0.0:14540` (alpha's
ports) + a dummy `-p 50052`. Rerun V1 → connect()'s targeted
`pkill -9 -f "mavsdk_server.*-p 50051( |$)"` EVICTED the squatter (observed SIGKILL),
mission **DONE 1/1**; the 50052 dummy **SURVIVED** → targeting proven (recap §3 honored).
Also observed: after any finals process exit (clean or exit-1), its own mavsdk_server child
does NOT persist — the natural-stale case needs a SIGKILLed parent; the manufactured drill
covers it more strictly anyway.

**Notes for SIM-2**: bump VM to 4+ vCPU first (SIM-0 note stands); sitl3.json fields are
validated + tested already (THREE_DRONES shape in test_sitl_adapter.py is the template);
expect per-drone gRPC 50051/52/53 and spawn poses "0,0"/"0,1"/"0,2" documented in the
sitl.json _comment; the budget-expiry race remains S4's.

### SIM-2 — pending

### SIM-3 — pending

### SIM-4 — pending

### SIM-5 — pending
