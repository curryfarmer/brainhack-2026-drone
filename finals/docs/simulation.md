# Simulation strategy — simulating the physical flight

> Status 2026-06-06: written from a dedicated research pass (5 topics; every load-bearing
> claim adversarially fact-checked against primary sources — PX4 source/docs, gz-sim
> source/PRs, pyhulax GitHub/PyPI, vendor material). Rejected options are recorded WITH
> reasons — read this before proposing a new simulator so nothing gets re-litigated.

**The question**: the pytest/Mock pipeline covers logic, but can we simulate the *physical
drone movement* — (a) the whole 3-drone swarm mission end-to-end in Gazebo or other methods,
or (b) partwise: takeoff/landing/flight ops in Gazebo, coordinate logic in turtlesim or
alternatives?

**The answer**:

- **End-to-end: YES.** 3× PX4 SITL instances sharing one Gazebo Harmonic world in the
  existing qualifier VM, moving ArUco convoy included, per-drone 640×480 cameras feeding our
  REAL `cv2.aruco` detector. All stock, officially supported tooling (Tiers 1+2 below).
- **Partwise: YES, and cheaper.** Headless flight-only 3× SITL validates the whole C2
  orchestration in hours; a pure-Python kinematic layer + property tests + log-replay plot
  validates mission/frame logic on Windows with zero installs (Tier 0).
- **turtlesim: NO** — rejected with reasons (matrix below).
- **Nothing simulates the HULA/pyhulax specifics.** No vendor or community HULA simulator
  exists (verified — HighGreat's app "仿真" sim is Scratch-only with no SDK endpoint), and
  that gap is exactly what the onsite hardware window is for.

**Fidelity framing (binding)**: the finals drones are **HULA, not PX4** — our code speaks
*only* pyhulax over Wi-Fi; we never touch MAVLink on the real aircraft. PX4 SITL below is a
**physics stand-in**, useful solely because `MavsdkSitlAdapter` implements the SAME
relative-move FlightAdapter contract (so mission logic can't tell the backends apart — the
whole point of the seam). The x500's dynamics, controller, and telemetry are NOT the HULA's;
nothing PX4-specific may leak above the adapter, and no SITL result retires a HULA-specific
risk (units, dynamics, Wi-Fi, video — the onsite list above).

## What simulation can NEVER answer here (= the 2-hour onsite window's exclusive job)

- pyhulax `move()` **units** (docs say cm; `hula_connection.py` shows 0.5) — the "unit hop"
  preflight gate stays.
- Real `move_to()`/`get_position()` semantics on the UWB-positioned drones.
  `flight/adapter.py:17-19` records why the contract assumes NO honest goto — these exist in
  the pyhulax API surface, so *verify onsite*, but never plan on them.
- HULA camera **HFOV**, `.to_rgb()` channel order, real ArUco read-range vs marker size.
- Wi-Fi behavior with 3 simultaneous video streams; HULA controller overshoot/settle/drift.

Everything else should arrive at the rig pre-validated by the tiers below.

## Feasibility matrix

| Approach | What it validates | Effort | Verdict |
|---|---|---|---|
| 3× PX4 SITL + Gazebo Harmonic, cameras + moving ArUco convoy | whole mission end-to-end: 3 concurrent adapters, search vs MOVING targets, real detector on rendered frames, sightings, pad logic | ~2–3 days on the VM | **ADOPT — Tier 2 (S8)** |
| Flight-only 3× SITL, headless (no cameras) | all C2 orchestration: 3× adapter semantics, asyncio interleaving, timeouts/failsafes, SightingBus under real flight loops | hours (PX4 already built) | **ADOPT — Tier 1 (S6)** |
| Pure-Python kinematic sim + replay viz + hypothesis invariants | blocking-move contract in continuous time, 3-drone interleaving, coverage/FOV/frame math, fault injection | hours, zero installs, Windows-native | **ADOPT — Tier 0 (S3/S4)** |
| FakeDroneAPI at the pyhulax Python API layer | PyhulaxAdapter drives the real SDK surface correctly (signature contract tests) | hours | **ADOPT — Tier 3 (S9, already planned)** |
| Webots R2025a | Windows-native rendered-camera sim (Mavic/Crazyflie models, extern Python controllers) | 1–3 days | fallback ONLY if VM rendering AND WSL2 both fail |
| gym-pybullet-drones | quad dynamics + per-drone cameras in PyBullet | 1–2 days + build friction | plan-B only: pybullet 3.2.7 ships NO win/cp312 wheels (MSVC source build); Ubuntu-tested, Python 3.10 pin |
| turtlesim | nothing we need | days, for a toy | **REJECTED**: needs a full ROS 2 install (Jazzy Windows binaries: "Only Windows 10 is supported"); 2D x/y/θ in a THIRD frame convention (CCW from +x, bottom-left origin), no altitude/NED; the DRPose→turtle bridge is itself a new untested transform; zero independent ground truth |
| rviz2 + tf2 | frame visualization | week+ | REJECTED: an entire ROS 2 middleware stack for what matplotlib shows at a fraction of the cost; our stack is not ROS 2 |
| AirSim / Colosseum / Project AirSim | photorealism + UE dynamics | week+ (tens of GB of UE) | REJECTED: AirSim archived 2022; Colosseum main needs UE 5.6; Project AirSim is v0.2.0; photorealism adds nothing for ArUco on 640×480 |
| CrazySim / Flightmare / RotorS | — | — | REJECTED: wrong API (CFLib) / unmaintained ~2021 / EOL stack (ROS 1 + gazebo-classic) |
| Network-layer pyhulax fake (TCP 8888 dialect server) | the real pyhulax client stack in-loop | week+ | REJECTED: ack/heartbeat semantics documented only by source — high risk of faithfully emulating the WRONG behavior, unverifiable before the hardware window |
| gazebo-classic multi-vehicle (`sitl_multiple_run.sh`) | — | — | REJECTED: EOL Jan 2025, removed from PX4 main tooling |

## Tier 0 — pure-Python kinematic sim + invariants (S3/S4 scope; Windows, zero installs)

What the instant-teleport MockAdapter cannot exercise: continuous time/space, 3-drone asyncio
interleaving against a *moving* convoy, FOV geometry, timeout tuning. Prior art for the
pattern: Fireline-Science/tello_sim, DroneBlocksTelloSimulator, bobzwik/Quadcopter_SimCon.

- **KinematicSimAdapter** (extends the MockAdapter/DeadReckoner line, same FlightAdapter
  seam): `move()` integrates the body-frame cm offset at a configurable ~40–60 cm/s over
  `asyncio.sleep` slices (honoring the blocking contract and `timeout_s`), Gaussian noise +
  per-drone bias + command-drop fault injection; a `SimWorld` holds 3 drone poses + convoy
  waypoints on one clock. Vision seam: a geometric frustum test emitting detections first;
  optionally `cv2.warpPerspective` of a real DICT_6X6_250 PNG onto synthetic 640×480 frames
  so the REAL detector runs.
- **hypothesis property tests** over DeadReckoner (square closure at ARBITRARY yaw,
  FORWARD/BACK inverse, |Δ(north,east)| == distance, rotation equivariance: yaw+90 maps
  FORWARD onto LEFT) and — highest value — **cross-implementation agreement**: MockAdapter
  pose == DeadReckoner for arbitrary action sequences; at S6 add
  `_body_offset_to_ned` == DeadReckoner under ψ_NED = −yaw_deg; at S7 a bearing property
  settles the known `types.py:102` sign conflict BEFORE any flight. Use
  `st.floats(allow_nan=False, allow_infinity=False)` and `math.isclose` at ~1e-9 (chained
  trig exceeds the 1e-12 golden tolerance).
- **`finals/tools/replay_plot.py`** (~150 lines): parse `runs_finals/<ts>/mission.jsonl`,
  feed each drone's command events through the REAL `finals.flight.dead_reckon.DeadReckoner`
  (never reimplement the math in the plotter), plot east-on-X / north-on-Y with
  `set_aspect('equal')`, yaw quivers, sighting points + bearing rays. A commanded right-turn
  square rendering as left turns = sign bug, instantly visible. matplotlib is already in
  requirements.txt. **Prereq (S4)**: an initial-pose/origin event in the events schema —
  current `runs_finals/` content is demo output and carries none (see Corrections). From S7,
  `rerun-sdk` (pip, Windows wheels, no ROS/account) adds timeline scrubbing + synced frames.

## Tier 1 — headless flight-only 3-drone SITL (existing VM; S6)

Highest value-per-hour: validates 3 concurrent `MavsdkSitlAdapter`s, takeoff/move/rotate/land
semantics, timeout/failsafe paths, SightingBus under 3 real flight loops — no rendering at
all. Launch recipe (port scheme verified in PX4 source `ROMFS/.../px4-rc.mavlink`):

```bash
make px4_sitl                       # once
# terminal per drone, N = 0,1,2:
HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 \
  PX4_GZ_MODEL_POSE="0,N" ./build/px4_sitl_default/bin/px4 -i N
# instances after the first ALSO set PX4_GZ_STANDALONE=1 (join the running gz server)
```

- Offboard MAVLink UDP port is exactly **14540 + instance** → 14540/14541/14542;
  `MAV_SYS_ID = instance + 1`. Models auto-name `${PX4_SIM_MODEL}_<N>`.
- C2 side: three MAVSDK `System(port=50051/50052/50053)` (distinct gRPC ports MANDATORY — no
  auto-selection) + `connect("udpin://0.0.0.0:1454N")`. `configs/sitl.json` address format
  already matches.
- Lockstep guarantees an overloaded VM only slows real-time factor — physics stays correct.
- Gate V1 stays single-drone; 3× concurrent takeoff→square→land is the documented stretch
  check once V1 passes.

## Tier 2 — vision-in-the-loop SITL (S8)

Start single-drone, then scale: `make px4_sitl gz_x500_mono_cam_down` + the stock `aruco.sdf`
world (PX4-gazebo-models) is the smoke test; then the convoy world.

- **Markers**: clone the `arucotag` model pattern (static `<plane><size>0.5 0.5</size></plane>`
  at z=0.001, material `<pbr><metal><albedo_map>…png</albedo_map></metal></pbr>`). Generate
  PNGs with `cv2.aruco.generateImageMarker(getPredefinedDictionary(DICT_6X6_250), id, 800)` +
  `cv2.copyMakeBorder` white quiet zone (≥1 module — markers without it detect unreliably).
  ≥800 px sources avoid texture blur. One model dir per ID; `GZ_SIM_RESOURCE_PATH` points at
  them. Landing pads = the same model with pad IDs and real pad `<size>`. Avoid
  Gazebo-Classic marker repos — their OGRE `<script>` materials don't render in Harmonic.
- **Convoy motion** (5 robots driving a loop), three working options:
  1. **VelocityControl plugin** (recommended, zero runtime code): copy gz-sim's own
     `velocity_control.sdf` vehicle, `<initial_linear>/<initial_angular>` → endless circle;
     5 phase-offset spawn poses = a convoy. Runtime speed changes via Twist on
     `/model/<name>/cmd_vel` (we already have gz.transport13 code).
  2. **Box `<actor>` waypoint trajectories** — these DO work in Harmonic (see Corrections);
     kinematic + non-colliding, fine for marker targets, arbitrary polygon paths.
  3. `/world/<w>/set_pose` scripting from Python at ~5–10 Hz — best when the convoy path
     must be test-harness-controlled and seeded (deterministic pytest scenarios).
- **Cameras**: stock `mono_cam` is 1280×960 @30 Hz, HFOV 1.74 rad — clone it and set
  **640×480** to match the pyhulax frame contract. Per-drone topics are deterministic:
  `/world/<w>/model/x500_mono_cam_down_<N>/link/camera_link/sensor/camera/image`
  (gz Image is R8G8B8 → one `cvtColor` to BGR). RTP/H.264 alternative on UDP 5600/5601/5602.
- **Detection-range sanity to bake into tests**: at HFOV 1.74 rad / 640 px, a 0.5 m marker
  subtends ~135/d px (d = slant range, m) → ~4.5 m max range at a 30 px detection floor.
  Re-derive once the real marker size + HULA HFOV are known (open question in
  [`module_map.md`](module_map.md)); range stays a config value, never an assumption.
- **Rendering environment ladder**: existing VM with real GL 3.3 (ogre2) → VM with
  `LIBGL_ALWAYS_SOFTWARE=1` (llvmpipe, CPU-heavy but correct; fine for 1 camera, probe for 3)
  → `--headless-rendering` (EGL, no display) → **WSL2 + WSLg on the C2 laptop** (officially
  documented PX4 env; `.wslconfig networkingMode=mirrored` carries localhost UDP across the
  boundary) → Webots R2025a only if all of the above fail.

## Tier 3 — pyhulax API-layer fake (S9, as already planned — plus verified findings)

- FakeDroneAPI mirrors `pyhulax.DroneAPI` signatures; **pyhulax v0.2.0 pip-installs on
  Windows without hardware** (requires Python ≥3.11; compiled deps all ship win_amd64 wheels;
  import is I/O-free until `connect()`) — so `inspect.signature` contract tests against the
  REAL SDK are feasible. Mark them skip-if-absent to keep the S9 gate "tests green WITHOUT
  pyhulax installed". Import the real `Direction`/`VelocityLevel`/`CommandResult`/`DroneState`
  types in those tests so the fake cannot drift. Also fake `move_to()`/`get_position()` —
  they exist in the real surface (onsite-verify their semantics; the no-goto contract stays).
- **Licensing**: pyhulax is UNLICENSED source-available (no LICENSE file; license null on
  GitHub and PyPI) — all rights reserved by default. Fine to pip-install and test against;
  do NOT vendor or fork it into this repo.
- **Video path**: `pyhulax.video.RTSPStream` accepts any `rtsp://` URL → mediamtx + ffmpeg
  serving replay or Gazebo-rendered frames runs our REAL video pipeline code unchanged.

## Corrections pinned by verification (don't re-import the stale claims)

1. **"Box actors don't render in new gz-sim" is FALSE** since gz-sim PR #1947 (merged
   2023-04; in Fortress 6.14.0, Garden 7.4.0+, ALL Harmonic 8.x). Plain box actors follow
   waypoint trajectories; gz-sim ships the regression test (`test/worlds/actor_trajectory.sdf`,
   `ActorTrajectoryNoMesh`). Actors are kinematic/non-colliding — use VelocityControl or
   TrajectoryFollower when collision matters.
2. **pyhulax is not open source** — unlicensed source-available (see Tier 3).
3. **The run logs do NOT yet contain everything a replay tool needs**: existing
   `runs_finals/` dirs are hardcoded smoke-demo output; no initial-pose/origin or
   executed-action events are emitted anywhere yet, and `est_north_m`/`est_east_m`/
   `bearing_deg` are Optional and usually empty by design. Replay needs the S4 schema
   addition, not just a plotter.
4. ArUco-on-gz-sim evidence rests on SaxionMechatronics/ros2-gazebo-aruco (Garden),
   mohamedeyaad/aruco_visual_servoing (Harmonic), and the PX4 arucotag precision-landing
   ecosystem — NOT on the often-cited automaticaddison tutorial (that one is Gazebo Classic).
5. PX4 multi-vehicle: instances after the first must set `PX4_GZ_STANDALONE=1`; the
   PX4-user_guide repo is archived — canonical docs live in PX4-Autopilot/docs.

## Sources (primary)

- PX4 multi-vehicle simulation (new Gazebo): https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation.html
- Port scheme: PX4-Autopilot `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink` (14540+instance)
- MAVSDK-Python multi-system gRPC ports: https://mavsdk.mavlink.io/main/en/python/
- ArUco assets: https://github.com/PX4/PX4-gazebo-models (`models/arucotag`, `worlds/aruco.sdf`)
- Actor fix: https://github.com/gazebosim/gz-sim/pull/1947 ; VelocityControl example:
  gz-sim `examples/worlds/velocity_control.sdf`
- pyhulax: https://pyhulax.xenops.ae , https://github.com/XENOPSAE/pyhulax (v0.2.0,
  2026-04-21), PyPI `pyhulax`
- gz-sim ArUco precedents: https://github.com/SaxionMechatronics/ros2-gazebo-aruco ,
  https://github.com/mohamedeyaad/aruco_visual_servoing
- Kinematic-sim prior art: https://github.com/Fireline-Science/tello_sim ,
  https://github.com/bobzwik/Quadcopter_SimCon
