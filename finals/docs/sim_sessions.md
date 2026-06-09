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
7. **Marker type — RESOLVED (user, 2026-06-09): ArUco, NOT QR** (the earlier "QR, 20×20 cm"
   was loose phrasing). `marker_backend` stays "aruco" (default); the QR path is a dormant
   seam. SIM-3 built+tested both and confirmed ArUco decodes at all sentry altitudes (QR did
   not) — so SIM-4/5 run the ArUco skin (`gen_markers.py --type aruco`, the committed default).
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
  will NOT import inside a 3.11 venv — irrelevant for headless SIM-0…2. **SOLVED in
  SIM-4**: a sidecar `sim/gz_camera_bridge.py` runs the gz subscriber under system 3.10
  and forwards raw RGB over a localhost TCP socket to `GazeboRgbSource` in the venv
  (no gz/cv2 in finals/). See the SIM-4 evidence block.
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
| SIM-2 | S6 stretch | `replay_plot.py` (proven on the SIM-1 fixture) → `sitl3.json` → 3× headless swarm + drills — **HEADLESS SIM DONE** | SIM-1, S5 | ✅ 2026-06-09 |
| SIM-3 | S8 assets | Convoy world: markers (ArUco+QR), robots, pads, band-altitude cams; detection check — **∥ with S4–S7** | SIM-0 | ✅ 2026-06-09 |
| SIM-4 | **S8** core | `gazebo_video.py` + `search.py`; **V2a**: single drone logs sightings of MOVING markers | SIM-1, SIM-3, S7, S5 | ✅ 2026-06-09 |
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

### SIM-2 — DONE 2026-06-09 (commit 2b5904c [tool/config] + this evidence commit [adapter fix + docs])

**Deliverables**: `finals/tools/replay_plot.py` + `finals/tools/__init__.py` + `finals/tests/test_replay_plot.py`
(34 tests) + `finals/configs/sitl3.json` (all in 2b5904c); `finals/docs/evidence/sim2_3drone.png`
(this commit); **`finals/flight/sitl_adapter.py` offboard-start race fix** + 6 new constructor tests
(this commit — see below).

**replay_plot design (reviewed + fixture-pinned)**: origin-seeded DR (EKF boot yaw ≠ 0 — the SIM-1
fixture booted −95.97°; assuming 0 renders every track rotated); per-drone subplots NEVER merged
(each drone's "north" is its OWN boot heading — overlaying frames is geometrically false; spawn-pose
offsets in the config `_comment` are documentation only); exact-field-set schema firewall
(`reconstruct_action` re-builds typed Actions via the real `DeadReckoner`, never reimplements the math —
unknown action/Direction/field-set → typed `ReplayPlotError`); null-origin leniency (PositionQuality.NONE
seeds 0 with a LOUD stderr warning, never errors — must still render real-hardware logs); signed shoelace
area **+1.0 m²** is the chirality+scale pin (CCW square, yaw-invariant). matplotlib lazy-imported with
`Agg` before pyplot when `--save` given (headless). All 34 tests + the conventions scan green.

**sitl3.json**: alpha/bravo/charlie, udpin 14540/41/42, gRPC 50051/52/53, bands 1.2/1.7/2.2 m
(= takeoff 120/170/220 cm via `takeoff_demo.from_config`), `command_timeout_s 30`,
`frame_backend "none"` (deliberate until SIM-5 wires gazebo frames). Schema THREE_DRONES-validated
(`test_sitl_adapter.py`). Spawn poses `0,i` = 1 m north apart (launcher PX4_GZ_MODEL_POSE).

**VM**: nproc **4** (bumped from 2 in VMware before this session — RTF margin per recap §8), reachable
`ssh bhvm` (192.168.174.128 unchanged), venv `.venv` (py3.11 + matplotlib). `git pull` → 2b5904c.

**Pytest**: Windows **512 passed** (deterministic, no mavsdk). VM: the full suite shows the
**S4-owned budget-expiry race** (`test_budget_expiry_lands_all_and_exits_clean`, SIM-0/SIM-1 evidence)
PLUS a **second newly-observed platform race**, `test_agent.py::test_stale_telemetry_fails_loud_before_acting`
— deterministic FakeClock test, passes **5/5 isolated** and 3/4 full-suite runs; fails only under full-suite
asyncio load. `agent.py` is UNTOUCHED by SIM-2 (the commit added only tool/test/config files → extra
collection load merely exposed a latent race). Same class as budget-expiry → **owner: S4**. Gate
("no NEW regressions") met. Post-fix `test_sitl_adapter.py` **58 passed** both OSes (52 + 6 new).

**⚠ THE 3× GATE FOUND A REAL BUG (the gate working as designed)** — `OffboardError: NO_SETPOINT_SET`
on takeoff. First fresh-instance 3× run: **all three FAILED** at `offboard.start()`; a re-run: alpha+bravo
DONE, charlie failed — intermittent + per-drone. Root cause: PX4 accepts `offboard.start()` only while
receiving a fresh setpoint stream; the 20 Hz mavsdk_server auto-resend covers an ALREADY-active session,
but before `start()` there is just the one priming setpoint, and under 3-instance gRPC contention it is not
always registered in time. Single-drone V1 (SIM-1) always had the headroom; the 3× swarm reliably tripped
≥1 drone. **Fix** (`sitl_adapter.py` takeoff offboard-entry): re-prime the hold setpoint + retry `start()`
on NO_SETPOINT_SET only, bounded by `offboard_start_tries` (default 5) AND the command deadline; every
other OffboardError/RpcError still fails loud on the first hit. Module docstring updated (convention 7).
Adversarial review + mutation kill-check: default-5 and the int≥1 validation killed by the 6 new
constructor tests; the "remove retry" mutant killed by the live gate (pre-fix all-3 NO_SETPOINT_SET vs
post-fix re-prime recovery). **No leak above the FlightAdapter — sim-only PX4 stand-in, pinned to SITL
mavsdk 3.15.3** (recap §1).

**3× gate (VM, fresh batteries, post-fix)** — `start 3` → `time ./run.sh -m finals.main --profile sitl
--config finals/configs/sitl3.json` (run dir `20260609_114513`):

```text
charlie/bravo/alpha: connected 1.5–1.8 s, health ready 0.6–1.0 s
[MavsdkSitlAdapter] charlie: offboard start NO_SETPOINT_SET — re-priming (attempt 1/5)
[MavsdkSitlAdapter] charlie: airborne at 2.05 m in 11.2 s, offboard active      <- 220 cm band
[MavsdkSitlAdapter] alpha:   offboard start NO_SETPOINT_SET — re-priming (attempt 1/5)
[MavsdkSitlAdapter] alpha:   airborne at 1.11 m ...                              <- 120 cm band
[MavsdkSitlAdapter] bravo:   airborne at 1.56 m ... (clean, no retry)            <- 170 cm band
4× [ move(FORWARD,100) | rotate(90) ] per drone, then land+disarm
MISSION SUMMARY  elapsed=42.4s  ticks=43   alpha DONE 1/1  bravo DONE 1/1  charlie DONE 1/1   EXIT=0
```

The log is the fix's own proof: the race STILL occurs under contention (charlie + alpha each hit it once)
and is transparently recovered on retry attempt 1. Takeoff `height_cm` 120/170/220 one drone each;
`grep -c '"event": "emergency_land"'` = **0**; wall **43.1 s**. **RTF ≈ 0.86–1.02** (mostly ~1.00,
one transient dip) from `/stats` during the flight — matches the SIM-0 baseline at the new 4 vCPU.

**Replay PNG** (`finals/docs/evidence/sim2_3drone.png`, generated ON the VM = headless Agg in situ,
scp'd as bytes): three per-drone subplots, each a CLOSED ~1 m square, CCW yaw quivers tangent to travel,
start/end markers coincident (closure), tilted ≈ −96° boot yaw, finals N±0.04 E±0.03 alt 0.00, 11 actions
each. Eyeballed ✓.

**Drill A — kill bravo's PX4 mid-mission** (`kill -9 $(cat sim/run/px4_1.pid)` after bravo's 2nd Move;
run dir `20260609_115711`):

```text
bravo FAILED: "Rotate failed: bravo: rotate(90 deg) aborted — telemetry is STALE (age 1.08 s > 1.00 s)
              — stream stalled; check the PX4 instance for udpin://0.0.0.0:14541 ..."
bravo events: action_start → action_failed → emergency_land → agent_failed → agent_disconnect
emergency_land  grep -c '"event": "emergency_land"'  = 1  (drone_bravo.jsonl AND mission.jsonl)
run_end exit_code 1  states {alpha DONE, bravo FAILED, charlie DONE}  (2 ok / 1 failed)
```

The STALENESS detector (kill px4 → mavsdk_server lives → streams go QUIET) fired at age 1.08 s, exactly
the SIM-1 physics. emergency_land's offboard.stop / land / disarm each TIMEOUT (PX4 gone) — traceback-logged
by the whitelisted swallows, latched exactly once, no hang. (First attempt at this drill was INVALID — a
stale-run-dir race in the harness killed px4 before bravo connected, yielding a connect-failure instead;
re-run parsing the run dir from run.sh's own stdout gave the clean mid-move kill above.)

**Drill B — 'q' abort, headless** (needs a TTY: driven via a Python `pty.openpty()` harness so
`isatty` is true and the AbortListener arms — `expect` is not installed; plain ssh gives "stdin EOF —
abort key disabled"):

```text
[AbortListener] abort key armed: press 'q' + Enter to LAND ALL drones
(all 3 airborne) → inject 'q\n'
[AbortListener] OPERATOR ABORT ('q'): landing ALL drones
[Orchestrator] OPERATOR ABORT (abort key): landing all drones cleanly
charlie/alpha/bravo: landed + disarmed
MISSION SUMMARY  elapsed=29.7s  alpha DONE 0/1  bravo DONE 0/1  charlie DONE 0/1   EXIT=0
```

Clean operator abort = land-all → all three DONE (phases incomplete, as expected — abort interrupts
mid-square), exit 0.

**Drill C — kill bravo's mavsdk_server** (`pkill -9 -f 'mavsdk_server.*-p 50052'`, targeted per recap §3,
at bravo's 2nd Move; run dir `20260609_120214`) — the DISTINCT dead-flag path (SIM-1 never exercised it
in a swarm):

```text
bravo FAILED: "Rotate failed: bravo: rotate(90 deg) aborted — in_air stream DIED: see stderr
              — PX4 instance dead? ..."   (poller_dead='in_air stream DIED', NOT the staleness message)
emergency_land = 1   run_end exit_code 1   {alpha DONE, bravo FAILED, charlie DONE}
```

Killing the server (not PX4) ENDS the gRPC streams → the adapter's stream-wrapper sets the dead-flag →
typed FlightError on the next command poll. Different trigger, same fail-loud outcome, no hang.

**Cleanliness**: `bash sim/launch_sitl.sh stop` → "no px4/gz processes remain";
`pgrep -fa 'p[x]4|g[z] sim'` → **CLEAN** (bracket form, recap §3 / sim/README self-match trap).

**Notes for SIM-3/4/5**: SITL battery drains across consecutive flights on the SAME instances
(SIM-1-known: ~52% after ~3 missions) → a 4th/5th back-to-back run trips "Battery unhealthy →
not armable → EKF-health timeout" on the most-flown instance. ALWAYS `stop` + `start 3` (fresh PX4 =
full battery) before each drill/run; never reuse instances across many missions. World stats topic is
`/stats`. The budget-expiry AND stale-telemetry full-suite races remain S4-owned (no NEW SIM-2 failures).

**HEADLESS SIM DONE**

### SIM-3 — convoy world assets ✅ 2026-06-09

**Scope shipped** (all under `sim/`, outside the conventions/SDK scan — raw cv2/gz/rclpy):
`gen_markers.py` (ArUco DICT_6X6_250 + QR v1/L PNG → per-marker gz model dir),
`models/{convoy_robot_<id>×5, pad_{100,101}, mono_cam_640}`, `worlds/convoy.sdf`,
`convoy_driver.py` (rclpy), `check_detection.py` (gz.transport13), `run_convoy.sh`.
**World**: 5 marker robots (VelocityControl, driven via rclpy→ros_gz→`/model/<n>/cmd_vel`)
orbiting a 2 m circle through the origin; a 3-camera down-tower (1.2/1.7/2.2 m sentry bands,
mono_cam_640 = stock mono_cam at 640×480, HFOV **1.74 rad = 99.69° → SIM-4 `camera_hfov_deg`**);
2 landing-pad markers just north of the route. `pytest finals/tests` green, `finals/` untouched.

**VM RENDER FINDING (supersedes SIM-0 "top GL rung holds")**: ogre2 + SVGA3D renders **BLANK**
camera frames on this VM — std=0 even on the stock `camera_sensor.sdf`. SIM-0's "29.8 FPS while
rendering" was measuring blank frames. Camera SENSOR geometry only renders under **llvmpipe**
(`LIBGL_ALWAYS_SOFTWARE=1`, std 59) or **ogre1** (`--render-engine ogre`, std 67). `run_convoy.sh`
defaults to llvmpipe + `DISPLAY=:0`. Headless EGL (`--headless-rendering`) is also blank here.
**SIM-4/5 must render under llvmpipe or ogre1, never the ogre2 default.**

**Two bugs fixed during bring-up**: (1) texture **basename collision** — every `marker.png`
collided in ogre2's resource cache so all markers rendered as the first-loaded id (7); textures
are now `<model>.png` (unique). (2) `gz sim` `$!` is the **wrapper** PID, not the render-server
child — `stop` now kills the server by world name.

**px-vs-distance table PER MARKER TYPE** (llvmpipe, run 1 of 2; convoy markers 20 cm on the
robot top z≈0.25 m so dist = band − 0.25; pads 40 cm on the ground so dist = band):

```text
ArUco (DICT_6X6_250) — DECODED on every band:
  band 1.2 m (dist 0.95):  ids 7,11,23,42,88   px 44/45/45
  band 1.7 m (dist 1.45):  ids 7,11,23,42,88   px 28/29/30
  band 2.2 m (dist 1.95):  ids 7,11,23,42,88   px 21/21/22 ; pads 100,101 (dist 2.20) px 38
QR (v1/L, payload=id) — LOCATED only, NEVER decoded:
  band 1.2 m:  QR located 75 reads  px 35/57/57   (full-code extent; ~1.7 px/module)
  band 1.7 m / 2.2 m:  QR not even located
```

**ArUco vs QR conclusion (feeds module_map's highest-value open question)**: ArUco (20 cm) decodes
at ALL three sentry altitudes; QR is barely *locatable* at 1.2 m and invisible at 1.7/2.2 m.
QR decode floor ≈ **4 px/module** (measured locally on cv2 4.11+QUIRC: decodes at ≥4, fails at 3),
i.e. ~132 px total for a 33-module code — the 57 px seen at 1.2 m is ~1.7 px/module, far below.
**→ QR is NOT viable for sentry detection at 1.2–2.2 m on 640 px; use ArUco** (or fly QR ≪1 m).
The VM's apt cv2 4.5.4 is also **not linked against QUIRC** (locates QR, never decodes — separate
real-world caveat); decode was confirmed on the laptop cv2 4.11.

**Coverage**: every convoy id (7,11,23,42,88) + both pads (100,101) decoded, **missing: NONE**.
Pads decode at band 2.2 m (centered in the wide FOV; near the frame edge at 1.2/1.7 m they fall
outside detection). A reproducible spurious id **157** (1–3 reads/run) is a raw single-frame ArUco
false positive — exactly why the finals perception layer confirms ids across frames.

**Determinism** (two fresh runs, seeded constant-Twist circle): identical decoded id SET
`{7,11,23,42,88,100,101}` both runs; per-band px identical (44/45 · 28/29/30 · 21/22). PASS.

**FPS per rendering rung** (3×640×480 cams @ 15 Hz, 4 vCPU):
```text
ogre2 (default)            : BLANK frames (std 0) — unusable for vision on this VM
llvmpipe (LIBGL_SW=1)      : ~14 fps/cam delivered, RTF ~0.9–1.0  [the working default + the smoke's REAL llvmpipe run]
ogre1 (--render-engine ogre): ~13.5 fps/cam, RTF 1.00 (renders + detects; faster RTF than llvmpipe)
```

**Annotated frames** (`finals/docs/evidence/`): `sim3_aruco_band120.png` (robot 7 detected on its
3D chassis + 2 distinct pads), `sim3_aruco_band220.png` (pads 100/101 + robot 88 all annotated),
`sim3_qr_band120.png` (40 cm pad QRs located [orange] but the 20 cm robot QR not even located).

**What this does NOT validate** (onsite-window jobs — `simulation.md`): HULA camera **HFOV** (we use
stock 1.74 rad; real unknown, `camera_hfov_deg: null`); **real-world read range** (rendered, noise-free,
flat-lit textures); **motion blur** (static cams + 0.4 m/s convoy ≈ none; real flight + rolling shutter
will degrade decode, QR worst). Out of scope by design: flight dynamics, PX4/HULA, the finals detector
wrapper (S7) / `gazebo_video.py` (SIM-4), bearing math, any `finals/` integration.

**Notes for SIM-4/5**: (a) render under **llvmpipe or ogre1** — ogre2 is blank here; (b) reuse the
camera topic `/world/convoy/model/cam_band_<NN>/link/camera_link/sensor/camera/image` and the
`PYTHONNOUSERSITE=1 python3` (system 3.10) interpreter for gz+cv2; (c) `gazebo_video.py` runs the SAME
gz.transport13 latest-frame pattern as `sim/check_detection.py` (mirrors root `depth_receiver.py`);
(d) marker skin is a one-key `gen_markers.py --type {aruco|qr}` reskin (type-agnostic model names);
(e) the 4th (0.7 m) camera was dropped — a 4-cam llvmpipe world starves one camera's stream.

### SIM-4 — gazebo video + search phase, single-drone vision-in-the-loop (2026-06-09) — V2a PASS

**Scope shipped**: `finals/vision/gazebo_video.py` (GazeboRgbSource), `finals/mission/phases/search.py`
(SentryScan + OpenLoopLawnmower), `main.py` gazebo wiring + `config.py` `gazebo_video_host/port`,
`configs/{mock_gazebo,sitl_vision}.json`, tests `test_vision_gazebo.py` + `test_search.py`; sim
assets `gz_camera_bridge.py`, `px4_models/x500_mono_cam_640`, `worlds/{convoy_px4,empty_cam}.sdf`,
`run_vision.sh`. `pytest finals/tests` green on Windows (636) AND the VM venv (708 incl. the new tests).

**THE FRAME-TRANSPORT SOLUTION (the deferred 3.11-venv problem, solved)**: `gz.transport13` is
compiled for system py3.10 and won't import in the finals 3.11 venv; the venv's pip opencv has no
GStreamer (so cv2 gstreamer pipelines are out) and PX4 gz Harmonic does not auto-stream RTP. So
finals imports NO gz/cv2 — a **sidecar TCP bridge** (`sim/gz_camera_bridge.py`, system py3.10 +
`PYTHONNOUSERSITE=1`) runs the PROVEN check_detection.py gz subscriber and forwards length-prefixed
raw RGB over a localhost TCP socket; `GazeboRgbSource` is a stdlib+numpy CLIENT (numpy lazy → the
module imports on a bare venv; an injected FakeFrameReceiver keeps the suite green). Wire frame:
`[u32 total_len][u64 frame_no][u32 w][u32 h][u8 ch][raw RGB]`, latest-drop, no auto-reconnect.

**Stage A (pipeline de-risk, no PX4)** — `run_vision.sh stageA`: convoy world (llvmpipe) + bridge on
the static `cam_band_170` + `finals --profile mock --config mock_gazebo.json`. The mock flight ends
instantly (MockAdapter consumes no wall-clock), but the brief window logged **3 real ArUco sightings**
(robot **7** + pads **100/101**) — proving bridge → GazeboRgbSource → PerceptionLoop → ArUco →
sightings.csv end-to-end with zero PX4.

**Stage B / gate V2a (full flight)** — `run_vision.sh stageB 110`: ONE PX4 x500 with the onboard
640×480 down-camera flying `--phases sentry_scan` over the driving convoy. Result: airborne at 1.57 m
(offboard NO_SETPOINT re-prime fired, as in SIM-2), 16 rotations over a **74.4 s** flight, landed +
disarmed, `alpha DONE`, **sightings = 1234**. Per marker_id (`runs_finals/20260609_175442`):

```text
ALL 5 MOVING convoy robots + both pads, every row bearing+yaw+MEASURED-position populated:
  7:149  11:117  23:114  42:59  88:89   (convoy)    100:588  101:117   (pads)
  157:1  (the SIM-3 single-frame ArUco false positive — exactly why perception confirms across frames)
  1234/1234 rows have bearing_deg + drone_yaw_deg; pos_quality=3 (MEASURED, SITL telemetry);
  drone_alt_m 1.67 at the sentry band; camera_hfov_deg 99.69 (= mono_cam_640 HFOV 1.74 rad) → bearing rays.
mission.jsonl mirrors 1234 'sighting' events.
sightings.csv head: alpha,…,aruco,aruco_100,100,153.0;208.0;401.0;455.0,1.0,480;640,113,169.88,…,176.58,3,0.41,-0.31,
```

**Drills (PASS)**:
- **Lost-video**: under a live GazeboRgbSource (`healthy=True`), `SIGTERM` the bridge → the reader
  thread logs "frame stream ended (ConnectionError: bridge closed) — no auto-reconnect", `healthy`
  flips **False in 0.2 s**, `get_frame()` still returns the last frame (no hang). This is the
  VideoWatchdog DEGRADE trigger (flight unaffected by design — a blind drone still flies home).
- **Empty-world**: `empty_cam.sdf` (ground + 1 down-cam, NO markers) → bridge → `finals mock_gazebo`
  → **sightings=0, alpha DONE** (clean exit; no hallucinated ids from the rendered scene).

**Two world-bring-up findings (the real Stage-B work)**:
1. **convoy.sdf has only the Sensors (camera) system** — PX4 in it reports Accel/Gyro/baro/compass
   "missing" (sensors never publish). `convoy_px4.sdf` adds the imu/air-pressure/air-speed/altimeter/
   magnetometer/navsat/forcetorque/contact systems + a `spherical_coordinates` GPS origin → the x500
   flies. PX4 must OWN the gz server (`PX4_GZ_WORLD=convoy_px4`, NO standalone) so lockstep drives the
   sensors; `LIBGL_ALWAYS_SOFTWARE=1` keeps the camera rendering (ogre2 = blank). convoy.sdf untouched.
2. **Spawn CLEAR of the robot starts**: spawning the drone at the origin (where `convoy_robot_7`
   starts) pinned it (takeoff reached 0.00 m). Spawn at `(1,0,0.2)` and start the convoy driving
   BEFORE the EKF settle so robot_7 vacates the origin.

**RTF / render**: with ONE camera (onboard only — `convoy_px4` drops the 3 tower cams; 4 cameras under
llvmpipe starve the EKF), `/clock` advances ~1 sim-s per wall-s → **RTF ≈ 1.0** on the 4-vCPU VM;
EKF "missing data" clears within the first ~60 s. Render rung = llvmpipe (`LIBGL_ALWAYS_SOFTWARE=1`),
never ogre2.

**Notes for SIM-5**: (a) `sitl3_vision.json` = 3× of `sitl_vision.json` with per-drone
`gazebo_video_port` (5600/5601/5602) + one `gz_camera_bridge.py` per drone on the per-instance camera
topic `x500_mono_cam_640_<i>`; (b) 3 onboard cameras under llvmpipe is the render-load question SIM-3
flagged (4 starved) — measure RTF, size `command_timeout_s` from it; (c) distinct `altitude_band_m` +
`sitl_address`/`mavsdk_grpc_port` per drone (config already validates this); (d) reuse `run_vision.sh`
stageB shape (PX4 owns the world, spawn each drone clear of robot starts, drive before settle).

### SIM-5 — pending
