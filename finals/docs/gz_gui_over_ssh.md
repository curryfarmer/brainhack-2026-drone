# Live Gazebo 3D GUI over ssh (watch all 3 drones)

Goal: while driving the sim from a laptop over ssh, see the **3D Gazebo window**
— all 3 drones + the convoy, 3rd-person — so you can actually watch the flight
instead of reading `sightings.csv` after the fact.

This is **sim-only debug/demo**. The finals stack never opens a window; onsite and
in headless cron there is no display.

## The key fact

The drones already run in Gazebo, but **headless**: every PX4 instance is launched
with `HEADLESS=1`, so instance 0 starts the gz **server** (`gz sim -s`, physics +
sensors) with **no window**. The picture is being computed; nothing paints it.

`gz sim` splits into two processes that talk over gz-transport:
- **server** (`-s`) — physics, sensors, lockstep. This is what PX4 owns.
- **GUI client** (`-g`) — a pure viewer. Subscribes to the scene + pose stream and
  renders. **Adds no camera sensors**, so it does **not** add frames to the
  single-threaded gz lockstep that caps SIM-5 RTF — only desktop GL cost.

So the workflow is: **leave the server headless, attach a `-g` client to it, and
paint that client onto the VM's own desktop (`:0`)** — which you watch in the
VMware console window. Nothing is forwarded back to the laptop; ssh only launches
the run.

## The three env vars that make it work on this VM

| var | value | why |
|---|---|---|
| `DISPLAY` | `:0` | the VM's real GNOME/Wayland desktop (gdm3). NOT an ssh-forwarded display — you see it in the VMware console window. |
| `QT_QPA_PLATFORM` | `xcb` | the gz GUI is Qt; `:0` is Wayland, so Qt rides **XWayland**. Without this Qt tries native Wayland and aborts. |
| `LIBGL_ALWAYS_SOFTWARE` | `1` | the VM (VMware SVGA3D) has no real GPU; force llvmpipe. (Same reason the camera sensors render under llvmpipe, never ogre2.) |

Plus `GZ_SIM_RESOURCE_PATH` must include `sim/models` + the PX4 gz models so the
client can resolve meshes/materials for the scene it is told to draw.

## Run it (one command)

After `git pull` on the VM, from a single blocking ssh session:

```bash
# Workstream A (hover + photograph), live 3D:
bash sim/run_vision.sh lanes3-gui 300

# Workstream B (active chase), live 3D:
bash sim/run_vision.sh track3-gui 360
```

`*-gui` just sets `GZ_GUI=1` and calls the normal `lanes3`/`track3`. The GUI is
attached **in-process, after `launch3`** (so the server is already up), and
`stageB3_stop` reaps it at the end — one process tree, no orphans. Watch the
**VMware console window**, not the ssh terminal.

`GZ_GUI=1` is a plain env gate, so it composes with any 3-instance mode:
```bash
GZ_GUI=1 bash sim/run_vision.sh stageB3 360      # the SIM-5 circle world, live
GZ_GUI=1 GZ_GUI_DISPLAY=:1 bash sim/run_vision.sh track3-gui 360   # other display
```

## Gotchas (learned the hard way)

- **Run it as ONE blocking ssh command.** Backgrounding the flight in one ssh
  call and attaching the GUI in a second call orphans the flight: the first
  session's process group gets SIGHUP'd on disconnect and the run is torn down
  under you. The `*-gui` modes launch the GUI inside the single run for exactly
  this reason. Use `ssh -o ServerAliveInterval=15` so the long silent EKF settle
  (120 s) does not drop the connection.
- **Absolute log path.** The GUI logs to `$RUN/gz_gui.log` (absolute). An earlier
  attempt redirected to a relative path and the GUI silently failed to exec
  (wrong CWD) — the window never appeared and there was no error in the terminal.
- **Never `pkill -f "gz sim"` from your ssh shell.** That pattern matches the
  killer shell's **own argv** → it kills your session (ssh exits 255, no output).
  Teardown uses the bracket form `g[z] sim -g` / `g[z] sim`, which the literal
  `g[z]...` in the running command does not match. Same rule for `bin/px4`.
- **Blank window?** `tail $RUN/gz_gui.log`. Usual causes: the `:0` desktop is not
  logged in (gdm sitting at the greeter — log into the GNOME session in the VMware
  console first), or `GZ_SIM_RESOURCE_PATH` is missing a model dir.
- **The window opens on the VM, not the laptop.** Have the VMware console visible
  before you start. There is no X-forwarding here (software GL + Qt over a
  forwarded X channel would be unusably slow anyway).

## What the script does (reference)

`sim/run_vision.sh`:
- `gz_gui_start()` — no-op unless `GZ_GUI=1`; else launches
  `DISPLAY=$disp QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 GZ_SIM_RESOURCE_PATH=... setsid gz sim -g`
  detached, records the wrapper PID in `$RUN/gz_gui.pid`.
- `stageB3` calls `gz_gui_start` right after `launch3` (server up).
- `stageB3_stop` kills `$RUN/gz_gui.pid` then `pkill -9 -f 'g[z] sim -g'`.
- Cases `lanes3-gui` / `track3-gui` set `GZ_GUI=1` and call `lanes3` / `track3`.
