# Deployment guide — install, configure, run the mission

How to install, configure and launch the autonomous drone stack for the **RoboVerse Qualifier 2026** challenge (DSTA BrainHack). PX4 SITL + Gazebo Harmonic + MAVSDK + YOLO. Goal: find yellow (50 pt) and red (100 pt) fuel barrels in a 40 m × 40 m × 8 m GNSS-denied space port within 10 minutes.

Two mission entry points exist:

- **`qualifier_run.py`** — asyncio supervisor + lawnmower coverage + detection loop. This is the documented design; most of this guide assumes it.
- **`qualifier_main.py`** — DFS exploration over a cell grid (teammate's alternative, uploaded May 22; 2 m cells in the current copy). Outputs `barrels.json` + `visited_cells.json`. See [Running the mission](#running-the-mission) below.

For why the stack is built this way, see the [design rationale](design-rationale.md). For the file-by-file inventory, see the [codebase guide](codebase.md). The repo **root** is the code directory — every command in this guide runs from the top of the checkout.

---

## Environment

| Thing | Value |
|---|---|
| Python | 3.12 (venv at `.venv`) |
| Torch | 2.2.2 CPU (Mac). **CUDA build needed on Linux training VM** — install via `pip install torch --index-url https://download.pytorch.org/whl/cu121` |
| NumPy | pinned `<2` (torch 2.2 wheels were built against NumPy 1.x) |
| Gazebo Python | `gz.transport13`, `gz.msgs10` — **not on PyPI**. Install via system pkg: `brew install gz-harmonic` (macOS) / apt `gz-harmonic` from packages.osrfoundation.org (Ubuntu 24.04). |
| PX4 SITL | UDP `:14540`, NED frame, vehicle `x500_vision_0` |
| YOLO weights | Trained barrel weights **`best.pt`** (6.2 MB) now exist at repo root. **However** `model_config.json` still points at the COCO-pretrained `yolov10n.pt` placeholder — and since `Detector.py` reads it via the `config_path` parameter again (regressed in the May 22 upload, restored 2026-06-06), whatever it points at **will** be picked up. **Make sure it points at the intended weights** — or pass `--weights best.pt` explicitly for scored runs. |

---

## Install

### Get the code

Two ways. Pick whichever fits your workflow — both end up in a working folder you run everything from.

**Drop-in (recommended for the drone PC).** No git, no clone — just download a fresh ZIP every time you want the latest code. This is what the rig actually does. See [Drop-in workflow](#drop-in-workflow-runsh) for the full loop.

```bash
wget https://github.com/curryfarmer/brainhack-2026-drone/archive/refs/heads/main.zip
unzip main.zip
cd brainhack-2026-drone-main
chmod +x run.sh
```

**Clone (for development on your laptop).**

```bash
# HTTPS (uses a personal-access token or git credential manager)
git clone https://github.com/curryfarmer/brainhack-2026-drone.git
cd brainhack-2026-drone

# OR SSH (recommended once your key is on github.com/settings/keys)
git clone git@github.com:curryfarmer/brainhack-2026-drone.git
cd brainhack-2026-drone
```

If you don't have collaborator access yet, ask the repo owner (`curryfarmer`) to add you under **Settings → Collaborators**.

> Both the COCO-pretrained `yolov10n.pt` (5.6 MB) and the trained barrel weights `best.pt` (6.2 MB) are checked in, so a fresh clone is runnable as-is. Training data is **not** committed — see the [training pipeline](training-pipeline.md) to regenerate or extend it.

### Python environment

One-time setup (repeat on the competition rig):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip wheel setuptools
pip install -r requirements.txt
```

> **History:** the May 22 upload regressed `requirements.txt`, dropping three entries — `mss` (screen-capture fallback in `collect_yolo_data.py`), the `torch>=2.1` floor, and the `ultralytics<9` upper pin. All three are back as of 2026-06-06, so a fresh `pip install -r requirements.txt` is sufficient. (You still pick the platform-specific torch wheel per the next subsection.)

### Torch flavour

`requirements.txt` does not pin a torch wheel — install the right one for your platform.

```bash
# CPU dev (Mac, no GPU)
pip install torch torchvision

# CUDA training VM / competition rig
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

### Gazebo Harmonic

The Gazebo Python bindings (`gz.transport13`, `gz.msgs10`) are **not** on PyPI. Install Gazebo Harmonic from system packages:

```bash
brew install gz-harmonic                          # macOS
sudo apt install gz-harmonic                      # Ubuntu 24.04 (after adding packages.osrfoundation.org)
```

Without Gazebo Harmonic you can still import `qualifier_run` for unit testing — sensor/depth paths will refuse to start.

### PX4 SITL

The drone scripts all expect PX4 SITL on `udpin://0.0.0.0:14540`. On the competition rig PX4 is launched separately; for local dev:

```bash
# clone + build PX4 (one-time)
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_sitl gz_x500_vision       # x500 quad with depth + RGB cameras, Gazebo Harmonic
```

Use whatever world publishes `/depth_camera` and the IMX214 RGB topic (the vehicle name in the default topic strings is `x500_vision_0`). This boots PX4 on `:14540` and spawns the x500 model in Gazebo. Leave it running in its own terminal — every script in this repo will discover it.

---

## Running the mission

PX4 SITL must be running and publishing telemetry on `udpin://0.0.0.0:14540`. Gazebo Harmonic must publish the depth + RGB topics named in the config. Before any flight, run the no-hardware smoke tests in the [simulator testing guide](simulator-testing.md) — they catch pure-logic regressions without burning sim time.

```bash
source .venv/bin/activate

# Flight-only run, 60-second budget, no YOLO
python qualifier_run.py --no-detector --budget 60

# Full mission with the trained barrel weights
python qualifier_run.py --weights best.pt --device cuda

# Override altitude + display detections during dev
python qualifier_run.py --altitude 3.0 --display
```

Output goes to `runs/<timestamp>/`:
```
runs/20260518_140523/
├── barrels.csv         # crash-safe scoring log (rewritten on every sighting)
└── detections/         # YOLO-annotated frames (one per detection event)
```

### Alternative entry point: `qualifier_main.py`

`qualifier_main.py` is a second, independently developed mission main (uploaded May 22). Instead of pre-baked lawnmower lanes it runs a **DFS exploration over a cell grid** (2 m cells in the current copy — the file's docstring still says 1 m; see the [codebase guide](codebase.md)), and it writes its results as `barrels.json` (detections) and `visited_cells.json` (coverage record) at the repo root rather than `runs/<timestamp>/barrels.csv`.

The two mains are **not integrated** — pick one per scored attempt, and make sure the judges' scoring expects the output format you produce. The trade-offs between the two approaches are discussed in the [design rationale](design-rationale.md). Note that `avoid.py` (the old reactive-nav loop) is **no longer the current best entry point** — it has been superseded by both qualifier mains.

---

## Drop-in workflow (run.sh)

The drone PC runs the code by downloading the GitHub ZIP, unzipping it, and launching from inside the extracted folder. There is **no `git pull`** on the rig — every code change cycle is a fresh download. This section is the single source of truth for that flow.

### One-time setup (per drone PC)

These only need to be done once. They install Python deps and the Gazebo Python bindings into the rig's system Python.

```bash
# Python deps (covers mavsdk, ultralytics, opencv, mss, etc.)
pip install -r requirements.txt

# Gazebo Harmonic — provides gz.transport13 + gz.msgs10
sudo apt install gz-harmonic
```

If you need torch on a specific accelerator, see [Torch flavour](#torch-flavour) above.

### Every-run loop

```bash
# 1. Grab the latest code as a ZIP from GitHub. Refresh this each iteration.
wget https://github.com/curryfarmer/brainhack-2026-drone/archive/refs/heads/main.zip -O latest.zip
unzip -o latest.zip
cd brainhack-2026-drone-main

# 2. Make the launcher executable (first time only — the ZIP loses the bit).
chmod +x run.sh

# 3. Launch any script through run.sh. The launcher kills zombie mavsdk_server
#    processes and reports who else owns UDP :14540 before handing off to Python.
./run.sh collect_yolo_data.py
```

For other scripts:

```bash
./run.sh basic_offboard.py
./run.sh keyboardcontrol.py
./run.sh qualifier_run.py --no-detector --budget 60
```

### What `run.sh` does (and why)

The launcher is a five-step shell wrapper:

1. `pkill -9 -f mavsdk_server` — kill any subprocess from a previous crashed run that's still holding UDP `:14540`. Silent if nothing matches.
2. `sleep 0.5` — give the kernel a moment to release the bound port.
3. `ss -ulpn | grep :14540` — check who, if anyone, still owns the port.
4. If the port is owned by something we didn't just kill (e.g. PX4 itself), print a warning line naming the process plus the manual fix recipes.
5. `exec python3 <your script> <args...>` — replace the shell with Python so Ctrl-C still cleans up the way you expect.

> **History:** the May 22 upload regressed `drone_control.py`, dropping the `_kill_stale_servers()` helper that ran the same `pkill` internally at the top of `connect()`. It was restored 2026-06-06, so scripts are self-healing again even without the launcher — `run.sh` remains a fine belt-and-braces way to launch.

### If `run.sh` still reports a bind error

Two cases:

- **PX4 SITL is configured to *bind* :14540 itself.** Some PX4 launch scripts run `mavlink start -u 14540 ...` where `-u` means "bind UDP". When that happens, MAVSDK cannot also `udpin://` the same port — it has to `udpout://` (send to) instead. One-line fix: edit the `connect()` call in `drone_control.py` and change
  ```python
  await self.drone.connect(system_address="udpin://0.0.0.0:14540")
  ```
  to
  ```python
  await self.drone.connect(system_address="udpout://127.0.0.1:14540")
  ```
  Then re-zip / re-download and run again. The launcher's warning line will tell you whether PX4 is the binder — look at the `users:(("px4",...))` portion of the `ss` output.

- **Some unrelated process is holding :14540** (a stray QGroundControl, a leftover SITL from a previous reboot, etc.). Find the PID and kill it:
  ```bash
  sudo lsof -iUDP:14540
  sudo kill -9 <PID>
  ```

Once `ss -ulpn | grep :14540` reports an empty line, MAVSDK can bind cleanly.

### If the screen-capture fallback ever runs

If you see `[CAM] Screen-capture fallback enabled (mss).` in the console, the Gazebo camera topic was unavailable. Put the Gazebo window on the primary monitor so the recorded frames contain the simulated scene. If `mss` itself fails to import (`[CAM] screen fallback unavailable`), `pip install mss` and try again — it is back in `requirements.txt` as of 2026-06-06, so an import failure means your venv predates that restore (see the note under [Install](#install)).

---

## Configuration

Every tunable lives in `MissionConfig` (top of `qualifier_run.py`). Three ways to set values:

### CLI flags (overrides config)

| Flag | Effect |
|---|---|
| `--weights PATH` | YOLO weights file (defaults to `yolov10n.pt`, **untrained for barrels**) |
| `--device {cpu,cuda,mps}` | Torch device for inference |
| `--altitude M` | Cruise altitude in metres |
| `--budget S` | Wall-clock budget (defaults to 600 s) |
| `--no-detector` | Skip YOLO entirely (flight test) |
| `--display` | Show OpenCV window with detections |
| `--config PATH` | Load a JSON file with any of the fields below |

> Trained barrel weights exist as `best.pt` at the repo root, but the default stays at the COCO `yolov10n.pt` and `model_config.json` is still the placeholder — pass `--weights best.pt` explicitly.

### JSON config (`--config run_config.json`)

```json
{
  "origin_north": 0.0,
  "origin_east": 0.0,
  "arena_north_m": 40.0,
  "arena_east_m": 40.0,
  "cruise_altitude_m": 2.5,
  "lane_spacing_m": 3.5,
  "along_axis": "north",
  "cruise_speed_mps": 1.2,
  "yolo_weights": "best.pt",
  "yolo_device": "cuda",
  "yolo_conf": 0.45,
  "depth_topic": "/depth_camera",
  "rgb_topic": "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image",
  "detection_class_map": {
    "yellow_barrel": "yellow_barrel",
    "red_barrel": "red_barrel"
  }
}
```

`K` (camera intrinsics) can also be in the JSON as a 3×3 nested list — it's converted to numpy on load.

### Edit `MissionConfig` defaults in source

Permanent defaults — change `qualifier_run.py:MissionConfig` directly.

---

## Pre-run checklist

Run through this before any scored attempt.

- [ ] **Entry point decided** — `qualifier_run.py` (lawnmower + supervisor) or `qualifier_main.py` (DFS exploration). Don't mix.
- [ ] **YOLO weights** trained on yellow + red barrels (use the [training pipeline](training-pipeline.md)) and **explicitly wired in** via `--weights best.pt` — `model_config.json` is still the COCO placeholder and is not auto-read.
- [ ] `cfg.detection_class_map` updated to match the *exact* class names emitted by the model (check via `m.names` after loading).
- [ ] `cfg.depth_topic` and `cfg.rgb_topic` match the topics being published by the Gazebo world released for the run.
- [ ] `cfg.K`, `cfg.img_width`, `cfg.img_height` match the actual camera in the world (current defaults: 640×480, fx=fy=433).
- [ ] `cfg.origin_north`, `cfg.origin_east`, `cfg.arena_north_m`, `cfg.arena_east_m` set for the released arena layout.
- [ ] `cfg.cruise_altitude_m` chosen so the camera FOV covers both ground and elevated barrels (see R2 in the [risk register](design-rationale.md#9-risks)).
- [ ] **Pose-drift sanity check** — run `python basic_offboard.py` with PX4 GPS disabled (`param set GPS_1_CONFIG 0`); confirm NED pose doesn't drift more than ~1 m over 60 s of hover. (R1 in the [risk register](design-rationale.md#9-risks).)
- [ ] Wall-clock budget left at 600 s (10 min judge clock).
- [ ] `--display` **off** for the scored run (saves CPU and avoids GUI lockups).
- [ ] `keyboardcontrol.py` not launched — manual control = DQ.

---

## Troubleshooting

| Symptom | Likely cause | Check |
|---|---|---|
| `ModuleNotFoundError: gz` | Gazebo Harmonic not installed | `brew install gz-harmonic` |
| `No telemetry pose received within 10 s` | PX4 not running, or wrong UDP port | `cfg.px4_address`; `netstat -an \| grep 14540` |
| Drone drifts away from waypoints | EKF2 dead-reckoning without vision/mocap | See R1 in the [risk register](design-rationale.md#9-risks); configure an external odometry source |
| YOLO finds nothing | Wrong weights, wrong class map, low confidence threshold | Run with `--display`; check `cfg.detection_class_map` matches `model.names`; confirm you passed `--weights best.pt` (the default is COCO) |
| `Arm failed` on second attempt | Vehicle still armed from previous attempt | Restart PX4 SITL between attempts during dev; in flight, the emergency-land in `mission_loop`'s exception handler should disarm |
| Barrel double-counted | Pose drift > `dedup_radius` between sightings | Increase `BarrelLog(dedup_radius=...)`; default 2.0 m |
| Detector callback runs but no entries in CSV | `context["depth"]` was None when the YOLO frame ran — depth stream lagging | Verify depth receiver is publishing; check `cfg.depth_topic` |

### MAVSDK gRPC failures

The `mavsdk_server` subprocess (spawned by `System()`) is the most common silent failure point. Three diagnostics + one cleanup snippet cover ~all cases:

| gRPC error | Where it fires | Cause | Fix |
|---|---|---|---|
| `AioRpcError: Socket closed` (UNAVAILABLE) | first `drone.action.arm()` | Stale `mavsdk_server` holding UDP `:14540` from a previous crashed run. New server can't bind, exits, next RPC dies. | Run the cleanup below. |
| `AioRpcError: recvmsg:Connection reset by peer` (UNAVAILABLE) | first `drone.offboard.set_velocity_body` | `MAVSDK_ADDRESS` uses legacy `udp://:14540`. MAVSDK 2.x reads that as `udpout` and the server segfaults on the first outbound write. | Use `udpin://0.0.0.0:14540`. (Already fixed in this repo; regression-catcher.) |
| Health loop hangs forever (no traceback) | inside `connect()` | PX4 SITL not running, or running on a different port. | `pgrep -fa px4` and `ss -ulpn \| grep 14540` — confirm PX4 owns the port. |

**Cleanup snippet (run on the drone PC before relaunching the script):**

```bash
pkill -9 -f mavsdk_server
pkill -9 -f collect_yolo_data
sleep 1
ss -ulpn | grep 14540   # should now be empty or owned only by PX4
```

`collect_yolo_data.py` delegates the full flight lifecycle (connect, arm, takeoff, offboard prime, land) to `drone_control.Drone` — the wrapper `qualifier_run.py` uses in production. That wrapper has a proven `arm_and_takeoff()` sequence (arm → takeoff → sleep 20 → NED-prime → offboard.start) and there is no hand-rolled MAVSDK code in `collect_yolo_data.py`. (The wrapper auto-kills stale servers again via `_kill_stale_servers()` — regressed May 22, restored 2026-06-06 — so `run.sh` and the cleanup snippet are belt-and-braces.)

---

Back to the [docs index](../README.md) · [simulator testing](simulator-testing.md) · [codebase guide](codebase.md) · [training pipeline](training-pipeline.md) · [design rationale](design-rationale.md) · [finals docs](../finals/README.md)
