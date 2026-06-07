# sim/ — PX4-SITL VM runbook (SIM-0)

Scripts for running the finals sim ladder on the VMware Ubuntu 22.04 VM (`ssh bhvm`
from the C2 laptop). This directory is **outside** the `finals/` conventions scan and
SDK whitelist BY DESIGN (`finals/docs/sim_sessions.md` recap §4) — raw MAVSDK/cv2/gz
scripts live here. The fail-loud bar still applies (recap §6).

> `sim/sitl_smoke.py` is sanctioned for **SIM-0 environment validation only**. From
> SIM-1 on, every flight goes through `python -m finals.main --profile sitl` (recap §5).

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
