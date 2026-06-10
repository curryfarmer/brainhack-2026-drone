# sim/ — PX4-SITL VM runbook (SIM-0)

Scripts for running the finals sim ladder on the VMware Ubuntu 22.04 VM (`ssh bhvm`
from the C2 laptop). This directory is **outside** the `finals/` conventions scan and
SDK whitelist BY DESIGN (`finals/docs/sim_sessions.md` recap §4) — raw MAVSDK/cv2/gz
scripts live here. The fail-loud bar still applies (recap §6).

> `sim/sitl_smoke.py` is sanctioned for **SIM-0 environment validation only**. From
> SIM-1 on, every flight goes through `python -m finals.main --profile sitl` (recap §5).

## Quickstart — run the sims on ANY Ubuntu 22.04 VM (portable)

Self-contained: copy-paste these onto a fresh box (someone else's VM, a cloud
instance — anywhere). The repo is **public**, so the clone needs no credentials.

**No ssh needed — run everything INSIDE the VM.** Open a terminal on the VM's own
desktop (VMware / VirtualBox console window, or its GUI). Every command below runs
locally on the box; nothing is driven from a laptop. (The `ssh bhvm` flow in the
sections further down is the *other* path — laptop-driven — and is optional. This
Quickstart replaces it entirely.) Bonus: sitting at the VM's graphical desktop
means you already have a live `:0` display, so the camera sensor renders with no
extra setup and `land1-gui` shows up right there in the same desktop.

**Box must be:** Ubuntu 22.04, ≥4 vCPU / ≥8 GiB RAM / ≥15 GB free disk, with a
**graphical desktop session** (that gives the `:0` display the camera-sensor render
needs). A truly headless server VM works too but needs a virtual framebuffer — see
the headless note at the end.

### Step 0 — PX4-Autopilot built (skip if `~/PX4-Autopilot/build/px4_sitl_default/bin/px4` already exists)

Standard PX4 upstream setup (pulls **Gazebo Harmonic**; first build is slow). Our
rig is pinned to Gazebo Harmonic 8.11 — verify against the PX4 docs if it drifts:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh          # installs the toolchain + sim deps; log out/in after
make px4_sitl gz_x500                  # builds the SITL binary + fetches Gazebo Harmonic
```

### Step 1 — pull the repo

```bash
git clone https://github.com/curryfarmer/brainhack-2026-drone.git ~/brainhack-2026-drone
cd ~/brainhack-2026-drone
git checkout main                      # NAV landing is on main (bcdfc51); or a feature branch
# later, to refresh:  git -C ~/brainhack-2026-drone pull
```

ZIP fallback if git/creds break:
`wget https://github.com/curryfarmer/brainhack-2026-drone/archive/refs/heads/main.zip -O latest.zip && unzip -o latest.zip`

### Step 2 — Python 3.11 venv + deps (system 3.10 cannot run `finals/`)

`finals/guards.py` needs `asyncio.timeout()` (3.11+). Install 3.11 from deadsnakes,
build the venv, install the lean set:

```bash
sudo add-apt-repository -y ppa:deadsnakes/ppa && sudo apt update
sudo apt install -y python3.11 python3.11-venv
cd ~/brainhack-2026-drone
python3.11 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip wheel
pip install pytest hypothesis mavsdk "numpy<2" opencv-contrib-python matplotlib scipy PyYAML Pillow pymavlink
python -m pytest finals/tests -q       # GATE: green here = clean import, env good
```

`opencv-contrib-python` (not plain `opencv-python`) — it carries `cv2.aruco`, which
both the detector AND `run_landing.sh install` (marker-texture generation) need.
`torch`/`ultralytics` are deliberately NOT installed (the YOLO detector is config-off;
`CannedDetector` needs no torch).

### Step 3 — install the landing assets into PX4 (re-run after any model/world change)

```bash
bash sim/run_landing.sh install        # copies x500_mono_cam_640 + landing worlds into PX4; builds pad_102 texture
```

### Step 4 — run the Challenge-2A landing sim

```bash
source .venv/bin/activate              # if not already active
bash sim/run_landing.sh land1    [secs]   # L1: 1 drone, full takeoff->navigate->land_on_pad (headless, default 300 s)
bash sim/run_landing.sh viewtest [secs]   # ^ + records overview + onboard .mp4 (watchable footage)
bash sim/run_landing.sh land1-gui [secs]  # ^ + live 3D view on the VM :0 desktop
bash sim/run_landing.sh land3    [secs]   # L2: 3 drones, staggered launch + serialized landing (default 700 s)
bash sim/run_landing.sh abort3   [secs]   # drill: 'q' lands all (orderly)
bash sim/run_landing.sh kill3    [secs]   # drill: kill instance 2 mid-mission (isolation)
bash sim/run_landing.sh stop              # tear everything down (run this if a run dies dirty)
```

Artifacts (events, sightings, replay PNG, footage `.mp4`) land under
`~/brainhack-2026-drone/runs_finals/<latest>/` and `sim/run/`. The earlier sim
ladders run the same way: `sim/run_vision.sh` (search/convoy V2) and
`sim/run_convoy.sh` (gz-only marker render, SIM-3 below); raw env smoke is
`sim/launch_sitl.sh` + `sitl_smoke.py` (SIM-0, below). **Always invoke as
`bash sim/<script>.sh`** — exec bits don't survive Windows/ZIP transfers.

**Headless box (no desktop on `:0`)?** The scripts export `DISPLAY=:0` because the
gz camera sensor only renders under llvmpipe with a live X display. With no desktop,
start a virtual framebuffer first — `sudo apt install -y xvfb` then
`Xvfb :0 -screen 0 1280x720x24 &` (or wrap a command in `xvfb-run -a`). Confirm
frames aren't blank: a blank-camera render is the ogre2 gotcha (SIM-3 below) — the
scripts already force `LIBGL_ALWAYS_SOFTWARE=1` to avoid it.

## Code sync: clone/pull (primary), ZIP (fallback)

```bash
# one-time (done in SIM-0)
git clone https://github.com/curryfarmer/brainhack-2026-drone.git ~/brainhack-2026-drone

# per iteration: push from Windows, then on the VM
git -C ~/brainhack-2026-drone pull
```

Fallback if git/credentials break — the ZIP drop-in workflow from
`docs/quali/deployment.md`:

```bash
wget https://github.com/curryfarmer/brainhack-2026-drone/archive/refs/heads/main.zip -O latest.zip
unzip -o latest.zip && cd brainhack-2026-drone-main
```

Always invoke shell scripts as `bash sim/<script>.sh` — exec bits can be lost through
Windows/ZIP transfers. Line endings are pinned LF by the repo-root `.gitattributes`.

If `ssh bhvm` stops resolving, the NAT/DHCP guest IP drifted: read `hostname -I` in
the VM console and update the `HostName` in `~/.ssh/config` on the laptop. Keep the VM
RUNNING (not suspended) during sim sessions.

## Python environment (venv on 3.11 — system 3.10 cannot run finals/)

System Python is 3.10 but `finals/guards.py` needs `asyncio.timeout()` (3.11+).
SIM-0 installed `python3.11` + `python3.11-venv` from the deadsnakes PPA (the jammy
archive only carries 3.11.0~rc1) and built the venv:

```bash
cd ~/brainhack-2026-drone
python3.11 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip wheel
pip install pytest mavsdk "numpy<2" opencv-python matplotlib scipy PyYAML Pillow pymavlink
python -m pytest finals/tests -q   # the gate: green on the VM
```

This is the **lean set**, not the full `requirements.txt`: `torch`/`ultralytics`/
`jupyter`/`mss` are deliberately NOT installed (the YOLO detector is config-off in the
finals configs, `CannedDetector` needs no torch, and the VM disk is at 78%). Install
them later ONLY if real-YOLO-in-sim is ever wanted.

**gz bindings wrinkle**: the apt `gz.transport13` Python bindings are compiled for
system 3.10 and will NOT import inside the 3.11 venv. Headless work (SIM-0…2) never
needs them; gz-subscriber probes run under system `python3` — **with
`PYTHONNOUSERSITE=1`**: a pip `--user` protobuf 7.x in `~/.local` shadows the apt
protobuf 3.12 the gz `_pb2` modules were generated for ("Descriptors cannot be
created directly" without it). SIM-4 (`gazebo_video`) must solve the 3.11 story
properly and record the solution in `sim_sessions.md`.

## Launching SITL instances

```bash
bash sim/launch_sitl.sh start 3                    # instances 0,1,2 (gz_x500, default world)
bash sim/launch_sitl.sh start 1 --model gz_x500_vision --world aprilworld
bash sim/launch_sitl.sh status
bash sim/launch_sitl.sh stop
```

`start N` boots instance 0 first (it spawns the gz server), polls gz readiness with a
deadline, then starts instances 1+ with `PX4_GZ_STANDALONE=1` sequentially
(simultaneous launches flake). Logs: `sim/run/px4_<i>.log`. PIDs: `sim/run/px4_<i>.pid`
(+ `gz.pid` for the gz server) — they exist precisely so kill drills are scriptable.

`start` is state-aware: instances with a live PID are skipped, dead ones are
relaunched into the running gz world (attaching to the surviving `x500_<i>` model via
`PX4_GZ_MODEL_NAME` — a respawn would name-collide). So after killing instance 1,
`start 3` is the solo-relaunch: it restarts exactly the dead one. If px4 instances
are alive but the gz server is gone, `start` refuses — kill by PID and start fresh.

### Port map

| Instance i | PX4 binds (shows in `ss` after start) | MAVSDK client binds (`udpin://`) | mavsdk_server gRPC | MAV_SYS_ID |
|---|---|---|---|---|
| 0 | 14580, 18570 | 14540 | 50051 | 1 |
| 1 | 14581, 18571 | 14541 | 50052 | 2 |
| 2 | 14582, 18572 | 14542 | 50053 | 3 |

PX4 *sends* offboard MAVLink to `14540+i`; that port appears in `ss -ulpn` only while
a MAVSDK client is bound to it (e.g. during `sitl_smoke.py`). Source:
`~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink`.

### The global-pkill ban (sim_sessions.md recap §3)

**NEVER run `pkill -f mavsdk_server` while anything is flying** — each drone's MAVSDK
`System()` owns one `mavsdk_server`; a global pkill kills all three. Root `run.sh`
does it at launch time only (one process, no servers spawned yet) — that's fine as a
*launcher*, lethal mid-mission. Stale-server cleanup is always targeted:
`pkill -9 -f "mavsdk_server.*-p <grpc_port>"`. `launch_sitl.sh stop` never touches
mavsdk_server at all — it kills only the PIDs it recorded.

## Environment smoke (SIM-0 only)

```bash
source .venv/bin/activate
bash sim/launch_sitl.sh start 1
python sim/sitl_smoke.py --instance 0      # connect→health→arm→takeoff→land → PASS
bash sim/launch_sitl.sh stop

bash sim/launch_sitl.sh start 3
python sim/sitl_smoke.py --all 3           # three concurrent PASSes
```

## Kill-drill one-liners

```bash
kill -9 "$(cat sim/run/px4_1.pid)"     # murder instance 1; 0 and 2 must keep answering
pgrep -fa 'px4|gz sim'                 # post-stop cleanliness check (must be empty)
ss -ulpn | grep 1454                   # who is bound to the client-side MAVLink ports
tail -f sim/run/px4_0.log              # watch an instance boot / die
```

After a kill drill, `start N` clears stale PID files automatically; a full
`stop` → `start` cycle is the clean reset.

**pgrep self-match trap**: when you run the cleanliness check through `ssh bhvm "..."`
or any `bash -c` one-liner, the wrapping shell's command string itself contains
`gz sim` and pgrep will report it as a false leftover (and a `pkill` variant would
kill your own shell — a SIM-0 drill proved this the hard way). Use the bracket form
in one-shot commands: `pgrep -fa 'p[x]4|g[z] sim'`. Interactive terminals are immune.
`launch_sitl.sh stop` kills strictly by recorded PIDs and never by name.

## Resource notes (SIM-0 baseline)

2 vCPU / 7.7 GiB / 11 GB free disk. Lockstep means an overloaded VM only slows the
real-time factor — physics stays correct, but slow runs trip wall-clock `timeout_s`
spuriously (recap §8): fix by config, never code. **Bump the VM to 4+ vCPUs in VMware
settings before SIM-2's 3× swarm work.** RTF baseline numbers live in the SIM-0
evidence block in `finals/docs/sim_sessions.md`.

## SIM-3 — convoy world (gz-only; NO PX4, NO flight)

A Gazebo-Harmonic world (`worlds/convoy.sdf`) with 5 marker-carrying robots orbiting a
2 m circle through the origin, a 3-camera down-tower at the origin (1.2 / 1.7 / 2.2 m
sentry bands), and 2 landing-pad markers. Proves markers render and are readable at the
altitude bands → a px-vs-distance table **per marker type** that sets the sentry-altitude
/ ArUco-vs-QR decision. Assets + scripts live under `sim/` (outside the conventions scan):
`gen_markers.py`, `models/{convoy_robot_*,pad_*,mono_cam_640}`, `convoy_driver.py`
(rclpy), `check_detection.py` (gz.transport13).

**VM render gotcha (verified):** camera SENSOR geometry only renders under llvmpipe
(`LIBGL_ALWAYS_SOFTWARE=1`) or ogre1 on this VM — the default ogre2+SVGA3D path emits
BLANK camera frames (std 0 even on the stock `camera_sensor.sdf`). `run_convoy.sh`
defaults to llvmpipe + `DISPLAY=:0`. The apt system cv2 4.5.4 also lacks QUIRC, so QR is
LOCATED but not DECODED on the VM (QR non-viability at sentry altitude is shown by the
located px being far below the decode floor; see the SIM-3 evidence block).

Three interpreter contexts — **never crossed**: gz launch (any shell) · convoy_driver
(ROS sourced, system 3.10) · check_detection (`PYTHONNOUSERSITE=1 python3`, system 3.10
— gz bindings + apt cv2 4.5.4; NOT the .venv). Marker assets are generated on the
**.venv** (cv2 ≥4.7 for `generateImageMarker`/`QRCodeEncoder`) and committed (ArUco skin).

```bash
# one shot: start gz -> ros_gz bridge + rclpy driver -> detection check -> stop
bash sim/run_convoy.sh all 40             # 40 s capture at the top GL rung
bash sim/run_convoy.sh all 40 --sw        # the REAL llvmpipe rung (LIBGL_ALWAYS_SOFTWARE=1)

# QR pass (reskin the SAME model dirs, then restore the committed ArUco skin):
source .venv/bin/activate
python sim/gen_markers.py --type qr --kind robot --ids 7 11 23 42 88 --size-cm 20
python sim/gen_markers.py --type qr --kind plane --prefix pad --ids 100 101 --size-cm 40
deactivate
bash sim/run_convoy.sh start
bash sim/run_convoy.sh drive 65
PYTHONNOUSERSITE=1 python3 sim/check_detection.py --secs 40 --allow-empty   # QR=finding, not fault
bash sim/run_convoy.sh stop
git checkout -- sim/models                # restore the committed ArUco textures
```

`GZ_SIM_RESOURCE_PATH` is set by `run_convoy.sh` to `sim/models` (so `model://` albedo
paths and `<include>` resolve). `stop` uses the `pgrep -fa 'g[z] sim'` bracket form (the
ssh-wrapper self-match trap). Convoy motion is the SIM-0-sanctioned rclpy→ros_gz→cmd_vel
path into the VelocityControl plugins (kinematic, deterministic; two runs → same ID set).
