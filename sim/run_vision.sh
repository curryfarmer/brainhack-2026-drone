#!/usr/bin/env bash
# SIM-4 vision-in-the-loop orchestration (VM only).
#
#   bash sim/run_vision.sh install-model        # copy x500_mono_cam_640 into PX4
#   bash sim/run_vision.sh bridge --topic T [--port P]   # gz->TCP bridge (PID + ready gate)
#   bash sim/run_vision.sh stop-bridge          # kill the recorded bridge PID
#   bash sim/run_vision.sh stageA [secs]        # Stage A de-risk: static cam + mock flight
#
# Interpreter contexts (NEVER crossed, sim/README): gz launch (any shell) |
# convoy driver (ROS sourced, system 3.10) | gz_camera_bridge (PYTHONNOUSERSITE=1
# python3, system 3.10 — gz bindings) | finals (.venv, python 3.11). The bridge is
# the ONLY new crossing: it reads gz under system 3.10 and serves frames to the
# venv over localhost TCP. Fail-loud: every wait has a deadline.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN="$REPO/sim/run"; mkdir -p "$RUN"
PX4_MODELS="$HOME/PX4-Autopilot/Tools/simulation/gz/models"
BRIDGE_READY_DEADLINE_S=60
CAM_BAND_170="/world/convoy/model/cam_band_170/link/camera_link/sensor/camera/image"

install_model() {
  [ -d "$PX4_MODELS" ] || { echo "FAIL [install-model]: $PX4_MODELS missing — is PX4-Autopilot built?" >&2; return 2; }
  cp -r "$REPO/sim/px4_models/x500_mono_cam_640" "$PX4_MODELS/"
  echo "[install-model] x500_mono_cam_640 -> $PX4_MODELS (PX4_SIM_MODEL=gz_x500_mono_cam_640)"
}

# Launch the gz->TCP bridge under system python3 (PYTHONNOUSERSITE=1), record its
# PID, and block until it prints BRIDGE READY (first gz frame) or the deadline.
bridge() {
  local topic="" port="5600"
  while (( $# )); do
    case "$1" in
      --topic) topic="${2:-}"; shift 2 ;;
      --port)  port="${2:-}";  shift 2 ;;
      *) echo "usage: bridge --topic T [--port P]" >&2; return 64 ;;
    esac
  done
  [ -n "$topic" ] || { echo "FAIL [bridge]: --topic required" >&2; return 64; }

  # Per-port filenames so 3 concurrent bridges (SIM-5) never clobber each
  # other's log/ready/pid (stageA/stageB still use exactly one, on 5600).
  local log="$RUN/gz_bridge_${port}.log" ready="$RUN/gz_bridge_${port}.ready"
  local pidf="$RUN/gz_bridge_${port}.pid"
  rm -f "$ready"
  echo "[bridge] gz_camera_bridge topic=$topic port=$port -> $log"
  PYTHONNOUSERSITE=1 python3 "$REPO/sim/gz_camera_bridge.py" \
    --topic "$topic" --port "$port" --ready-file "$ready" > "$log" 2>&1 &
  echo $! > "$pidf"

  local t0=$SECONDS pid; pid="$(cat "$pidf")"
  until [ -f "$ready" ] || grep -q "BRIDGE READY" "$log" 2>/dev/null; do
    kill -0 "$pid" 2>/dev/null || { echo "FAIL [bridge]: PID $pid died before READY — CHECK: tail $log" >&2; return 3; }
    if (( SECONDS - t0 > BRIDGE_READY_DEADLINE_S )); then
      echo "FAIL [bridge]: no BRIDGE READY within ${BRIDGE_READY_DEADLINE_S}s — WHY: no gz frame on $topic" >&2
      echo "  CHECK: gz topic -l | grep image ; world launched under llvmpipe? ; tail $log" >&2
      return 4
    fi
    sleep 1
  done
  echo "[bridge] ready after $((SECONDS - t0))s (PID $pid, port $port)"
}

# Tear down EVERY recorded bridge (1 for stageA/B, 3 for stageB3). Iterates the
# per-port pid files; also reaps the legacy single-pid file from older runs.
stop_bridge() {
  local f pid found=0
  for f in "$RUN"/gz_bridge_*.pid "$RUN/gz_bridge.pid"; do
    [ -e "$f" ] || continue
    found=1
    pid="$(cat "$f")"
    kill -TERM "$pid" 2>/dev/null && echo "[stop-bridge] TERM -> PID $pid ($(basename "$f"))"
    sleep 1
    kill -0 "$pid" 2>/dev/null && { kill -KILL "$pid" 2>/dev/null; echo "[stop-bridge] KILL -> PID $pid"; }
    rm -f "$f"
  done
  (( found )) || echo "[stop-bridge] no recorded bridge PID"
}

# Stage A: convoy world (llvmpipe) + driving convoy + bridge on the STATIC
# cam_band_170 + finals mock-flight (NO PX4) -> sightings.csv. Proves the whole
# transport + perception + search + CSV path before the PX4 onboard-camera flight.
stageA() {
  local secs="${1:-40}"
  echo "[stageA] convoy world (llvmpipe) + bridge(cam_band_170) + finals mock_gazebo ${secs}s"
  bash "$REPO/sim/run_convoy.sh" start --sw || { echo "FAIL [stageA]: world start" >&2; return 2; }
  bash "$REPO/sim/run_convoy.sh" drive "$((secs + 30))" || { bash "$REPO/sim/run_convoy.sh" stop; return 2; }
  bridge --topic "$CAM_BAND_170" --port 5600 || { bash "$REPO/sim/run_convoy.sh" stop; return 3; }
  local rc=0
  ( cd "$REPO" && .venv/bin/python -m finals.main --profile mock \
      --config finals/configs/mock_gazebo.json --budget "$secs" ) || rc=$?
  stop_bridge
  bash "$REPO/sim/run_convoy.sh" stop
  echo "[stageA] finals rc=$rc — sightings.csv under $REPO/runs_finals/<latest>/"
  return $rc
}

ONBOARD_TOPIC="/world/convoy_px4/model/x500_mono_cam_640_0/link/camera_link/sensor/camera/image"

stageB_stop() {
  stop_bridge
  if [ -f "$RUN/px4_vision.pid" ]; then
    kill -9 "$(cat "$RUN/px4_vision.pid")" 2>/dev/null; rm -f "$RUN/px4_vision.pid"
  fi
  pkill -9 -f 'bin/px4 -i' 2>/dev/null
  bash "$REPO/sim/run_convoy.sh" stop 2>/dev/null   # stops the ros bridge + driver
  pkill -9 -f 'convoy_px4' 2>/dev/null               # PX4's gz server (this world)
  pkill -9 -f 'g[z] sim' 2>/dev/null                 # last resort (bracket = no self-match)
  sleep 1
}

# Stage B (gate V2a): PX4 OWNS the gz server (lockstep -> sensors/EKF healthy)
# running convoy_px4 under llvmpipe (1 camera renders), spawns the x500 with the
# onboard mono_cam_640, the convoy drives, the bridge serves the ONBOARD camera,
# and finals flies sentry_scan logging moving-marker sightings.
stageB() {
  local secs="${1:-120}"
  install_model
  cp "$REPO/sim/worlds/convoy_px4.sdf" "$HOME/PX4-Autopilot/Tools/simulation/gz/worlds/" 2>/dev/null
  export DISPLAY="${DISPLAY:-:0}"
  export GZ_SIM_RESOURCE_PATH="$REPO/sim/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"

  echo "[stageB] launching PX4 (convoy_px4, llvmpipe, lockstep) -> $RUN/px4_vision.log"
  # Spawn the drone CLEAR of the convoy robot start poses (robot_7 starts at the
  # origin) so they don't interpenetrate during the EKF settle and pin the drone
  # (a t=0 overlap made takeoff reach 0.00 m). (1,0) is >=1 m from every robot
  # start; at 1.7 m the camera footprint still covers the origin crossing.
  ( cd "$HOME/PX4-Autopilot" && \
    LIBGL_ALWAYS_SOFTWARE=1 HEADLESS=1 PX4_SYS_AUTOSTART=4001 \
      PX4_SIM_MODEL=gz_x500_mono_cam_640 PX4_GZ_WORLD=convoy_px4 \
      PX4_GZ_MODEL_POSE="1.0,0,0.2,0,0,0" \
      setsid ./build/px4_sitl_default/bin/px4 -i 0 -d > "$RUN/px4_vision.log" 2>&1 & \
    echo $! > "$RUN/px4_vision.pid" )

  local t0=$SECONDS
  until gz topic -l 2>/dev/null | grep -q "x500_mono_cam_640_0/link/camera_link/sensor/camera/image"; do
    if (( SECONDS - t0 > 90 )); then
      echo "FAIL [stageB]: onboard camera topic never appeared — CHECK: tail $RUN/px4_vision.log" >&2
      stageB_stop; return 3
    fi
    sleep 2
  done
  echo "[stageB] camera topic up after $((SECONDS - t0))s"

  # Drive the convoy BEFORE the settle so robot_7 vacates the origin and the
  # convoy is mid-lap by takeoff. Duration covers settle + flight.
  echo "[stageB] driving the convoy"
  bash "$REPO/sim/run_convoy.sh" drive "$((secs + 90))" || echo "[stageB] WARN convoy drive failed — markers will be static" >&2

  echo "[stageB] settling EKF 45s (convoy moving)"
  sleep 45

  bridge --topic "$ONBOARD_TOPIC" --port 5600 || { stageB_stop; return 4; }

  echo "[stageB] finals sitl_vision (sentry_scan) budget=${secs}s"
  local rc=0
  ( cd "$REPO" && .venv/bin/python -m finals.main --profile sitl \
      --config finals/configs/sitl_vision.json --budget "$secs" ) || rc=$?
  stageB_stop
  echo "[stageB] finals rc=$rc — sightings.csv under $REPO/runs_finals/<latest>/"
  return $rc
}

# ============================================================
# SIM-5: 3 PX4 camera-drones (FULL SIM)
# ============================================================
# Onboard camera topic for instance i (model x500_mono_cam_640_<i>).
cam_topic_n() { echo "/world/${VWORLD}/model/x500_mono_cam_640_$1/link/camera_link/sensor/camera/image"; }

# World + finals config + spawn poses launch3/stageB3 use. Defaults = the SIM-5
# circle world; lanes3/track3 (S11) override these globals to convoy_3lane.
VWORLD="${VWORLD:-convoy_px4}"
VCONFIG="${VCONFIG:-finals/configs/sitl3_vision.json}"

# Spawn poses (ENU x,y,z,r,p,yaw), >=1.2 m from every robot start (robot_7 @ origin):
# alpha (1.2,0.2) E, bravo (-1.2,0.2) W, charlie (0,-2) = convoy-circle CENTRE.
SIM5_POSES=( "1.2,0.2,0.2,0,0,0" "-1.2,0.2,0.2,0,0,0" "0,-2,0.2,0,0,0" )
# S11 3-lane drone poses: alpha over car_7 (4.2,0), bravo over car_23 (-2,3.5),
# charlie over car_88 (-2,-3.5) — each >=1.2 m from its car spawn (see
# convoy_3lane.sdf). VM-TUNE these against the measured nadir footprints.
LANES3_POSES=( "4.2,0,0.2,0,0,0" "-2.0,3.5,0.2,0,0,0" "-2.0,-3.5,0.2,0,0,0" )
# The poses launch3 actually spawns; lanes3/track3 swap in LANES3_POSES.
LAUNCH_POSES=( "${SIM5_POSES[@]}" )

stageB3_stop() {
  stop_bridge
  # The optional live GUI client (GZ_GUI=1) first — it is render-only, but leave
  # it and it survives the run holding a stale window. Bracket pattern = the
  # killer shell's own argv ('g[z] sim -g') does NOT self-match (sim/README).
  if [ -f "$RUN/gz_gui.pid" ]; then
    kill -9 "$(cat "$RUN/gz_gui.pid")" 2>/dev/null; rm -f "$RUN/gz_gui.pid"
  fi
  pkill -9 -f 'g[z] sim -g' 2>/dev/null
  local i
  for i in 0 1 2; do
    if [ -f "$RUN/px4_vision_$i.pid" ]; then
      kill -9 "$(cat "$RUN/px4_vision_$i.pid")" 2>/dev/null; rm -f "$RUN/px4_vision_$i.pid"
    fi
  done
  pkill -9 -f 'bin/px4 -i' 2>/dev/null
  bash "$REPO/sim/run_convoy.sh" stop 2>/dev/null   # ros bridge + driver
  pkill -9 -f "$VWORLD" 2>/dev/null                  # PX4's gz server (this world)
  pkill -9 -f 'g[z] sim' 2>/dev/null                 # last resort (bracket = no self-match)
  sleep 1
}

# Launch 3 PX4 instances into ONE convoy_px4 world. Instance 0 OWNS the gz server
# (renders all 3 cams -> needs llvmpipe); 1 and 2 JOIN via PX4_GZ_STANDALONE=1
# (the normal multi-vehicle pattern — joining a PX4-lockstep world, unlike SIM-4's
# failed join of a plain non-PX4 gz world). Each instance is gated on its OWN
# camera topic (proves the model spawned AND its sensor publishes). PIDs recorded
# for scriptable kill drills.
launch3() {
  install_model
  cp "$REPO/sim/worlds/${VWORLD}.sdf" "$HOME/PX4-Autopilot/Tools/simulation/gz/worlds/" 2>/dev/null
  export DISPLAY="${DISPLAY:-:0}"
  export GZ_SIM_RESOURCE_PATH="$REPO/sim/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"

  local i
  for i in 0 1 2; do
    local log="$RUN/px4_vision_$i.log" pidf="$RUN/px4_vision_$i.pid"
    echo "[launch3] instance $i world=$VWORLD pose=${LAUNCH_POSES[$i]} -> $log"
    if (( i == 0 )); then
      ( cd "$HOME/PX4-Autopilot" && \
        LIBGL_ALWAYS_SOFTWARE=1 HEADLESS=1 PX4_SYS_AUTOSTART=4001 \
          PX4_SIM_MODEL=gz_x500_mono_cam_640 PX4_GZ_WORLD=$VWORLD \
          PX4_GZ_MODEL_POSE="${LAUNCH_POSES[$i]}" \
          setsid ./build/px4_sitl_default/bin/px4 -i "$i" -d > "$log" 2>&1 & \
        echo $! > "$pidf" )
    else
      ( cd "$HOME/PX4-Autopilot" && \
        HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1 \
          PX4_SIM_MODEL=gz_x500_mono_cam_640 PX4_GZ_WORLD=$VWORLD \
          PX4_GZ_MODEL_POSE="${LAUNCH_POSES[$i]}" \
          setsid ./build/px4_sitl_default/bin/px4 -i "$i" -d > "$log" 2>&1 & \
        echo $! > "$pidf" )
    fi

    local t0=$SECONDS pid; pid="$(cat "$pidf")"
    until gz topic -l 2>/dev/null | grep -q "x500_mono_cam_640_$i/link/camera_link/sensor/camera/image"; do
      kill -0 "$pid" 2>/dev/null || { echo "FAIL [launch3]: instance $i (PID $pid) died before its camera topic — CHECK: tail $log" >&2; stageB3_stop; return 3; }
      if (( SECONDS - t0 > 120 )); then
        echo "FAIL [launch3]: instance $i camera topic never appeared in 120s — WHY: gz server (i0) or EKF/spawn — CHECK: tail $log" >&2
        stageB3_stop; return 4
      fi
      sleep 2
    done
    echo "[launch3] instance $i camera up after $((SECONDS - t0))s"
  done
  echo "[launch3] all 3 instances up (cams 0/1/2)"
}

# Read sim-time milliseconds from one /clock message (the `sim` block only).
# NB: anchor sec/nsec — an unanchored /sec:/ ALSO matches `nsec:` lines; and
# print as a float (%.0f) so a large sec*1000 never overflows awk's 32-bit %d.
_clock_sim_ms() {
  gz topic -e -t /clock -n 1 2>/dev/null | awk '
    /^sim \{/{insim=1; next}
    insim && /^[ \t]*sec:/  {s=$2}
    insim && /^[ \t]*nsec:/ {printf "%.0f", s*1000 + $2/1000000; exit}'
}

# probe3: bring up 3 instances + 3 cams, measure RTF + per-cam fps over a window,
# tear down. NO flight. THE render-load gate (SIM-3: 4 cams starve a stream on
# this 4-vCPU VM; 3 is the open question). WARN -> drop x500_mono_cam_640
# update_rate 15->10 and size command_timeout_s from the RTF.
probe3() {
  local secs="${1:-15}"
  launch3 || return $?
  echo "[probe3] measuring RTF + per-cam fps over ${secs}s (3 cams, llvmpipe)..."

  # RTF from /clock (tiny messages); per-cam fps from 3 count-mode bridges — the
  # SAME gz subscriber stageB3 runs, so this loads the render exactly like the
  # real flight (NOT `gz topic -e`, which would stream full 640x480 frames as
  # text and itself starve the box).
  local m0 w0; m0="$(_clock_sim_ms)"; w0=$SECONDS
  local i
  for i in 0 1 2; do
    ( PYTHONNOUSERSITE=1 python3 "$REPO/sim/gz_camera_bridge.py" \
        --topic "$(cam_topic_n "$i")" --count-secs "$secs" \
        > "$RUN/probe3_cam$i.log" 2>&1 ) &
  done
  wait
  local m1 w1; m1="$(_clock_sim_ms)"; w1=$SECONDS

  local wall=$(( w1 - w0 )); (( wall < 1 )) && wall=1
  local rtf="n/a"
  if [ -n "$m0" ] && [ -n "$m1" ]; then
    rtf="$(awk -v a="$m0" -v b="$m1" -v w="$wall" 'BEGIN{printf "%.2f", (b-a)/1000.0/w}')"
  fi
  echo "[probe3] --- RTF=${rtf} (sim ${m0:-?}->${m1:-?} ms over ${wall}s wall) ---"
  local verdict="PASS"
  for i in 0 1 2; do
    local line fps
    line="$(grep -m1 'BRIDGE FPS' "$RUN/probe3_cam$i.log" 2>/dev/null)"
    if [ -z "$line" ]; then
      echo "[probe3] cam $i: NO FRAMES — $(tail -1 "$RUN/probe3_cam$i.log" 2>/dev/null)"
      verdict="WARN"; continue
    fi
    fps="$(echo "$line" | sed -n 's/.*fps=\([0-9.]*\).*/\1/p')"
    echo "[probe3] cam $i: ${line#BRIDGE FPS }"
    awk -v f="${fps:-0}" 'BEGIN{exit !(f < 8.0)}' && verdict="WARN"
  done
  awk -v r="$rtf" 'BEGIN{ if (r=="n/a") exit 1; exit !(r+0 < 0.9) }' && verdict="WARN"
  echo "[probe3] VERDICT: $verdict"
  [ "$verdict" = "WARN" ] && echo "[probe3]  -> a cam <8 fps / no frames / RTF<0.9: drop x500_mono_cam_640 update_rate 15->10 and re-probe; size command_timeout_s + mission_budget_s in sitl3_vision.json from the RTF (slow != broken)." >&2
  stageB3_stop
}

# Optional LIVE 3D GUI (GZ_GUI=1) — the gz-GUI-over-ssh workflow. Instance 0 runs
# the gz server HEADLESS (no window). This attaches a SEPARATE gz GUI client to
# that already-running server and paints it onto the VM's own :0 desktop, so the
# VMware console window shows all 3 drones + the convoy in 3D, 3rd-person.
#
#   - The GUI is render-ONLY: it subscribes to the scene/pose stream, it is NOT a
#     camera sensor and adds NO frames to the single-threaded gz lockstep (which
#     is the SIM-5 RTF ceiling) — only desktop GL load. Safe to run during flight.
#   - DISPLAY :0 = the VM's real GNOME/Wayland desktop (NOT forwarded to the
#     laptop — you watch it in the VMware console window). Qt rides XWayland, so
#     QT_QPA_PLATFORM=xcb. LIBGL_ALWAYS_SOFTWARE=1: the VM has no real GPU.
#   - In-process inside ONE blocking run so stageB3_stop reaps it; setsid shields
#     it from SIGHUP if the ssh session that launched the run drops.
#   - SIM-ONLY: never onsite / in headless cron (no :0 there). gz must already be
#     up (call AFTER launch3), else the client paints an empty world.
gz_gui_start() {
  [ "${GZ_GUI:-0}" = "1" ] || return 0
  local disp="${GZ_GUI_DISPLAY:-:0}"
  echo "[gui] attaching gz GUI client on DISPLAY=$disp (software GL, XWayland) -> $RUN/gz_gui.log"
  DISPLAY="$disp" QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 \
    GZ_SIM_RESOURCE_PATH="$REPO/sim/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}" \
    setsid gz sim -g > "$RUN/gz_gui.log" 2>&1 &
  echo $! > "$RUN/gz_gui.pid"
  echo "[gui] launched (wrapper PID $(cat "$RUN/gz_gui.pid")) — LOOK AT THE VMware CONSOLE WINDOW."
  echo "[gui]   window blank? -> tail $RUN/gz_gui.log ; is the :0 desktop logged in?"
}

# stageB3 (gate V2 full): 3 PX4 camera-drones fly sentry_scan over the moving
# convoy, each reading its OWN onboard cam via its OWN bridge (5600/5601/5602).
stageB3() {
  local secs="${1:-180}" mode="${2:-normal}"   # mode: normal | abort | kill
  launch3 || return $?
  gz_gui_start                                  # no-op unless GZ_GUI=1 (live 3D view)

  echo "[stageB3] driving the convoy"
  bash "$REPO/sim/run_convoy.sh" drive "$((secs + 150))" || echo "[stageB3] WARN convoy drive failed — markers static" >&2

  echo "[stageB3] settling EKF 120s (3-instance lockstep, convoy moving; the"
  echo "          standalone-joining instances 1/2 converge slowest under render load)"
  sleep 120

  local i
  for i in 0 1 2; do
    bridge --topic "$(cam_topic_n "$i")" --port "$((5600 + i))" || { stageB3_stop; return 5; }
  done

  # DRILL kill3: background a killer that -9's instance 2 (charlie) mid-scan so
  # the swarm proves single-drone-loss isolation (charlie FAILED + exactly one
  # emergency_land, alpha+bravo complete, exit 1).
  if [ "$mode" = "kill" ]; then
    ( sleep "${KILL_AFTER:-45}"
      # The recorded px4_vision_2.pid is a setsid wrapper, not the live px4
      # (setsid detaches px4 into its own session) -> kill the REAL process by
      # command-line pattern. Killing px4 (not its mavsdk_server) exercises the
      # staleness path: px4 dies, mavsdk_server lives, telemetry goes QUIET.
      kp="$(pgrep -f 'bin/px4 -i 2' | tr '\n' ' ')"
      echo "[stageB3] DRILL kill3: kill -9 instance 2 (charlie) px4 -i 2 pid(s)=${kp:-NONE}" >&2
      pkill -9 -f 'bin/px4 -i 2' 2>/dev/null ) &
  fi

  echo "[stageB3] finals --config $VCONFIG budget=${secs}s mode=${mode}"
  local rc=0
  if [ "$mode" = "abort" ]; then
    # DRILL abort3: run finals under a PTY so the AbortListener arms, inject 'q'
    # once all 3 are airborne (orderly land-all -> all DONE, exit 0).
    ( cd "$REPO" && PYTHONNOUSERSITE=1 python3 sim/pty_q_harness.py \
        --trigger-regex 'offboard active' --trigger-count 3 --fallback-secs 80 -- \
        .venv/bin/python -m finals.main --profile sitl \
        --config "$VCONFIG" --budget "$secs" ) || rc=$?
  else
    ( cd "$REPO" && .venv/bin/python -m finals.main --profile sitl \
        --config "$VCONFIG" --budget "$secs" ) || rc=$?
  fi
  stageB3_stop
  echo "[stageB3] finals rc=$rc — sightings.csv under $REPO/runs_finals/<latest>/"
  return $rc
}

# ============================================================
# S11: 3 PX4 drones over the convoy_3lane world (3 diverging straight lanes).
# Reuses the full stageB3 flow (launch3 gate + 120 s settle + 3 bridges +
# finals) with VWORLD/VCONFIG/LAUNCH_POSES overridden and a STRAIGHT convoy
# (CONVOY_ANGULAR=0) on the 3 lane robots only.
#
# CONVOY_DELAY (150 s): the convoy HOLDS at its spawns through the EKF settle +
# takeoff and only starts driving once the scan is live. Each drone spawns
# DIRECTLY OVER its car's spawn (LANES3_POSES, in-footprint), so detection is
# guaranteed the moment the scan starts (car sitting still under the nadir cam),
# and the post-delay motion shows the 3 diverging directions. Without the hold
# the cars (driving from t=0) would leave the footprints before takeoff.
# CONVOY_LINEAR slow (0.05 m/s) so each car lingers in / is easily chased by its
# drone. Both env-overridable for VM tuning.
# ============================================================
lanes3() {                 # Workstream A: hover + photograph (sentry_scan)
  VWORLD=convoy_3lane
  VCONFIG=finals/configs/sitl3_lanes_vision.json
  LAUNCH_POSES=( "${LANES3_POSES[@]}" )
  export CONVOY_IDS="7 23 88" CONVOY_ANGULAR=0 \
         CONVOY_LINEAR="${CONVOY_LINEAR:-0.05}" CONVOY_DELAY="${CONVOY_DELAY:-150}"
  stageB3 "${1:-300}" normal
}

track3() {                 # Workstream B: active chase (track_convoy)
  VWORLD=convoy_3lane
  VCONFIG=finals/configs/sitl3_track_vision.json
  LAUNCH_POSES=( "${LANES3_POSES[@]}" )
  # CONVOY_LINEAR 0.08 (faster than lanes3's 0.05): track_convoy holds over a
  # centred car (the nadir deadband), so a near-still car looks like 3 parked
  # drones — the cars need to actually DRIVE for the chase to read as a chase,
  # while staying slow enough to stay inside the smallest footprint (alpha 1.2).
  export CONVOY_IDS="7 23 88" CONVOY_ANGULAR=0 \
         CONVOY_LINEAR="${CONVOY_LINEAR:-0.08}" CONVOY_DELAY="${CONVOY_DELAY:-150}"
  stageB3 "${1:-360}" normal
}

# ============================================================
# WS-5: DYNAMIC self-assignment (the user's sim #3). Identical flight rig to
# track3, but NO drone is told which car to chase — each runs track_convoy in
# dynamic mode (track_marker_ids null) and claims whatever ArUco id it sees that
# the shared C2 ConvoyRegistry still allows. The registry's single-winner CAS
# dedups; convoy_ids seeds the known set + the serviced tally. Logic is pure-
# proven (WS-1/2); these runs are gz INTEGRATION evidence.
#   dyn3  = clean case: convoy_3lane, 3 drones over 3 diverging cars (7/23/88).
#           Expect 3 DISTINCT owners + serviced 3/3 in the heartbeat.
#   dyn5  = contention: convoy_px4, 3 drones over 5 cars (7/11/23/42/88). Expect
#           3 distinct claims + remaining_ids = the 2 unclaimed. dyn5-kill -9's
#           charlie's px4 mid-run -> its car frees to LOST -> a free drone may
#           re-claim (LOST->re-claim). Which 3 of 5 get serviced is gate-F tuning.
# ============================================================
dyn3() {
  VWORLD=convoy_3lane
  VCONFIG=finals/configs/sitl3_dyn3_vision.json
  LAUNCH_POSES=( "${LANES3_POSES[@]}" )
  export CONVOY_IDS="7 23 88" CONVOY_ANGULAR=0 \
         CONVOY_LINEAR="${CONVOY_LINEAR:-0.08}" CONVOY_DELAY="${CONVOY_DELAY:-150}"
  stageB3 "${1:-360}" normal
}

dyn5() {                   # mode: normal | kill (LOST->re-claim drill)
  local secs="${1:-420}" mode="${2:-normal}"
  VWORLD=convoy_px4
  VCONFIG=finals/configs/sitl3_dyn5_vision.json
  LAUNCH_POSES=( "${SIM5_POSES[@]}" )
  # All 5 cars drive (the 2 unclaimed still move through the arena). Slow + held
  # at spawn through the EKF settle (same nadir-footprint discipline as track3).
  export CONVOY_IDS="${CONVOY_IDS:-7 11 23 42 88}" \
         CONVOY_LINEAR="${CONVOY_LINEAR:-0.08}" CONVOY_DELAY="${CONVOY_DELAY:-150}"
  stageB3 "$secs" "$mode"
}

# ============================================================
# WS-7A: SOFT-ZONE HANDOVER (the user's stretch). Identical dynamic flight rig
# to dyn3 (convoy_3lane, 3 drones, track_convoy dynamic, shared ConvoyRegistry)
# but the config adds per-drone advisory SECTORS (sitl3_handover_vision.json +
# arena sitl_handover). The demo: a car drives a slow ARC that carries it OUT of
# its owner's wedge and INTO an idle neighbour's wedge mid-run. Expected
# behaviour in the heartbeat (convoys block) + mission.jsonl:
#   1. the owner FLAGS the convoy exited_zone (it KEEPS tracking) — snapshot
#      "exited_zone": [<id>];
#   2. the orchestrator matcher logs convoy_handover_offered (from->to) and the
#      snapshot shows "offered": {"<id>": "<neighbour>"};
#   3. the idle neighbour claims it -> ownership in "in_flight" moves to the
#      neighbour; the original owner re-acquires (its give-up line counts a
#      handover, not a loss).
# If NO neighbour is idle, the convoy stays flagged with its owner (still
# tracked) — the 'keep tracking but flagged' path.
#
# CONVOY_ROUTE is a GLOBAL 'dur,v,w; ...' path (v = m/s forward, w = rad/s yaw)
# applied to every driven car (sim/convoy_driver.py --route). The default below
# = drive straight ~30 s (cars settle in their own wedges, each drone claims its
# lane car), then a sustained LEFT yaw (w>0 = CCW) that swings the cars' heading
# so a car crosses a wedge boundary into the neighbour's sector, then straight
# again. SLOW so track_convoy keeps the car centred through the turn. The exact
# turn that crosses a boundary depends on the gz spawn bearings + the wedge
# centres in sitl3_handover_vision.json — VM-TUNE the route + the sector_deg
# wedges together at gate F (config only, never code).
# ============================================================
handover3() {
  VWORLD=convoy_3lane
  VCONFIG=finals/configs/sitl3_handover_vision.json
  LAUNCH_POSES=( "${LANES3_POSES[@]}" )
  export CONVOY_IDS="7 23 88" \
         CONVOY_LINEAR="${CONVOY_LINEAR:-0.06}" CONVOY_DELAY="${CONVOY_DELAY:-150}" \
         CONVOY_ROUTE="${CONVOY_ROUTE:-30,0.06,0.0; 40,0.05,0.18; 60,0.06,0.0}"
  stageB3 "${1:-360}" normal
}

# ============================================================
# LAWNMOWER COVERAGE (the user's "implement lawnmower first"). Each drone flies
# the OpenLoopLawnmower phase (search.py, name=lawnmower) — a blind precomputed
# body-frame boustrophedon — and the parallel PerceptionLoop decodes EVERY ArUco
# that crosses the nadir footprint (save_marker_frames stamps a JPEG). NO chase,
# NO registry, NO trained CV model: ArUco decode is classical, the marker IS the
# id. lawn3 is the headline (3 drones read all 5 ids over convoy_px4); lawn1 is
# the cheap 1-cam warm-up (proves the sweep flies + reads at all).
# ============================================================

# Fail LOUD if the just-finished run logged NO sightings. The lawnmower phase is
# a blind plan emitter — unlike track_convoy (min_sightings_to_pass) it cannot
# self-check that it ever saw a marker — so a coverage run that reads nothing
# exits 0 on clean Dones, which would read as success. This is the harness-side
# mirror of that guard: count data rows in the latest run's sightings.csv.
assert_sightings() {
  local label="$1"
  local latest; latest="$(ls -1dt "$REPO"/runs_finals/*/ 2>/dev/null | head -1)"
  if [ -z "$latest" ]; then
    echo "FAIL [$label]: no runs_finals/<run> dir — WHY: finals never created a run dir — CHECK: did the mission start? re-read the finals output above" >&2
    return 6
  fi
  local csv="${latest%/}/sightings.csv"
  if [ ! -f "$csv" ]; then
    echo "FAIL [$label]: $csv missing — WHY: perception logged no sightings file — CHECK: bridge frames (sim/run/gz_bridge_*.log) + marker_backend in $VCONFIG" >&2
    return 6
  fi
  # grep -c . counts non-empty lines (and DOES count a final unterminated line),
  # so a header-only file = 1; >1 means >=1 data row.
  local nonempty; nonempty="$(grep -c . "$csv" 2>/dev/null || echo 0)"
  if (( nonempty <= 1 )); then
    echo "FAIL [$label]: sightings.csv has 0 data rows — WHY: the lawnmower swept but read NO ArUco — CHECK: does the sweep height/lanes cover the cars? cars driving (CONVOY_DELAY)? cam frames arriving (gz_bridge_*.log)? -> tune the sweep (config) BEFORE reaching for a detector" >&2
    return 7
  fi
  echo "[$label] sightings.csv OK: $((nonempty - 1)) data row(s) in $csv"
}

# Launch ONE PX4 instance (instance 0, owns the gz server under llvmpipe lockstep)
# into $VWORLD at LAUNCH_POSES[0], gated on its camera topic. The 1-cam analog of
# launch3 for the cheap single-drone warm-up — leaves launch3/stageB3 untouched.
# PID recorded as px4_vision_0.pid so stageB3_stop reaps it.
launch1() {
  install_model
  cp "$REPO/sim/worlds/${VWORLD}.sdf" "$HOME/PX4-Autopilot/Tools/simulation/gz/worlds/" 2>/dev/null
  export DISPLAY="${DISPLAY:-:0}"
  export GZ_SIM_RESOURCE_PATH="$REPO/sim/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
  local log="$RUN/px4_vision_0.log" pidf="$RUN/px4_vision_0.pid"
  echo "[launch1] instance 0 world=$VWORLD pose=${LAUNCH_POSES[0]} -> $log"
  ( cd "$HOME/PX4-Autopilot" && \
    LIBGL_ALWAYS_SOFTWARE=1 HEADLESS=1 PX4_SYS_AUTOSTART=4001 \
      PX4_SIM_MODEL=gz_x500_mono_cam_640 PX4_GZ_WORLD=$VWORLD \
      PX4_GZ_MODEL_POSE="${LAUNCH_POSES[0]}" \
      setsid ./build/px4_sitl_default/bin/px4 -i 0 -d > "$log" 2>&1 & \
    echo $! > "$pidf" )
  local t0=$SECONDS pid; pid="$(cat "$pidf")"
  until gz topic -l 2>/dev/null | grep -q "x500_mono_cam_640_0/link/camera_link/sensor/camera/image"; do
    kill -0 "$pid" 2>/dev/null || { echo "FAIL [launch1]: instance 0 (PID $pid) died before its camera topic — CHECK: tail $log" >&2; stageB3_stop; return 3; }
    if (( SECONDS - t0 > 120 )); then
      echo "FAIL [launch1]: instance 0 camera topic never appeared in 120s — CHECK: tail $log" >&2
      stageB3_stop; return 4
    fi
    sleep 2
  done
  echo "[launch1] instance 0 camera up after $((SECONDS - t0))s"
}

# lawn1: 1 drone flies the lawnmower over convoy_3lane while ONE car (id 7) is
# driven straight + slow. Cheapest rig (1 cam -> RTF ~1.0). Spawns over car_7
# (LANES3_POSES[0]) so the first read is guaranteed, then the sweep translates
# away. CONVOY_DELAY holds the car at spawn through the EKF settle.
lawn1() {
  VWORLD=convoy_3lane
  VCONFIG=finals/configs/sitl1_lawn_vision.json
  LAUNCH_POSES=( "${LANES3_POSES[0]}" )
  export CONVOY_IDS="7" CONVOY_ANGULAR=0 \
         CONVOY_LINEAR="${CONVOY_LINEAR:-0.05}" CONVOY_DELAY="${CONVOY_DELAY:-150}"
  local secs="${1:-300}" rc=0
  launch1 || return $?
  gz_gui_start                                   # no-op unless GZ_GUI=1 (live 3D view)
  echo "[lawn1] driving the convoy"
  bash "$REPO/sim/run_convoy.sh" drive "$((secs + 150))" || echo "[lawn1] WARN convoy drive failed — marker will be static" >&2
  echo "[lawn1] settling EKF 90s (single instance)"
  sleep 90
  bridge --topic "$(cam_topic_n 0)" --port 5600 || { stageB3_stop; return 5; }
  echo "[lawn1] finals --config $VCONFIG budget=${secs}s"
  ( cd "$REPO" && .venv/bin/python -m finals.main --profile sitl \
      --config "$VCONFIG" --budget "$secs" ) || rc=$?
  stageB3_stop
  echo "[lawn1] finals rc=$rc — sightings.csv under $REPO/runs_finals/<latest>/"
  (( rc == 0 )) && { assert_sightings lawn1 || rc=$?; }
  return $rc
}

# lawn3: 3 drones each sweep a disjoint strip over convoy_px4 (5 cars present) so
# all 5 ids get read across the run — the read-all-5 coverage proof. Reuses the
# full 3-instance stageB3 flow (the lawn config has 3 drones matching the rig);
# the post-run assert fails loud if coverage read nothing.
lawn3() {
  VWORLD=convoy_px4
  VCONFIG=finals/configs/sitl3_lawn_vision.json
  LAUNCH_POSES=( "${SIM5_POSES[@]}" )
  export CONVOY_IDS="${CONVOY_IDS:-7 11 23 42 88}" \
         CONVOY_LINEAR="${CONVOY_LINEAR:-0.08}" CONVOY_DELAY="${CONVOY_DELAY:-150}"
  local secs="${1:-420}" rc=0
  stageB3 "$secs" normal || rc=$?
  (( rc == 0 )) && { assert_sightings lawn3 || rc=$?; }
  return $rc
}

# read5: READ-AND-RELEASE coverage (sitl3_read5_vision.json). Same convoy_px4
# rig as lawn3/dyn5 (5 cars), but the 3 drones run track_convoy in COVERAGE mode
# (disengage_on_read + search_when_idle + registry dedup): each chases an unread
# car only until it has 2 consistent reads, marks it SERVICED, and peels off to
# the next; when all 5 are SERVICED the orchestrator EARLY-STOPS and lands all.
# Watch the summary's CONVOY COVERAGE serviced N/5 + the coverage_complete event.
read5() {
  VWORLD=convoy_px4
  VCONFIG=finals/configs/sitl3_read5_vision.json
  LAUNCH_POSES=( "${SIM5_POSES[@]}" )
  export CONVOY_IDS="${CONVOY_IDS:-7 11 23 42 88}" \
         CONVOY_LINEAR="${CONVOY_LINEAR:-0.08}" CONVOY_DELAY="${CONVOY_DELAY:-150}"
  local secs="${1:-420}" rc=0
  stageB3 "$secs" normal || rc=$?
  (( rc == 0 )) && { assert_sightings read5 || rc=$?; }
  return $rc
}

case "${1:-}" in
  install-model) install_model ;;
  bridge)        shift; bridge "$@" ;;
  stop-bridge)   stop_bridge ;;
  stageA)        shift; stageA "$@" ;;
  stageB)        shift; stageB "$@" ;;
  stageB-stop)   stageB_stop ;;
  probe3)        shift; probe3 "$@" ;;
  stageB3)       shift; stageB3 "${1:-360}" normal ;;
  abort3)        shift; stageB3 "${1:-360}" abort ;;
  kill3)         shift; stageB3 "${1:-360}" kill ;;
  stageB3-stop)  stageB3_stop ;;
  lanes3)        shift; lanes3 "${1:-300}" ;;
  track3)        shift; track3 "${1:-360}" ;;
  lanes3-gui)    shift; export GZ_GUI=1; lanes3 "${1:-300}" ;;   # +live 3D view on :0
  track3-gui)    shift; export GZ_GUI=1; track3 "${1:-360}" ;;   # +live 3D view on :0
  dyn3)          shift; dyn3 "${1:-360}" ;;                      # WS-5 dynamic 3-lane
  dyn3-gui)      shift; export GZ_GUI=1; dyn3 "${1:-360}" ;;
  dyn5)          shift; dyn5 "${1:-420}" normal ;;               # WS-5 5-car contention
  dyn5-kill)     shift; dyn5 "${1:-420}" kill ;;                 # WS-5 LOST->re-claim drill
  handover3)     shift; handover3 "${1:-360}" ;;                 # WS-7A soft-zone handover
  handover3-gui) shift; export GZ_GUI=1; handover3 "${1:-360}" ;;
  lawn1)         shift; lawn1 "${1:-300}" ;;                     # lawnmower 1-drone warm-up
  lawn1-gui)     shift; export GZ_GUI=1; lawn1 "${1:-300}" ;;
  lawn3)         shift; lawn3 "${1:-420}" ;;                     # lawnmower 3-drone read-all-5
  lawn3-gui)     shift; export GZ_GUI=1; lawn3 "${1:-420}" ;;
  read5)         shift; read5 "${1:-420}" ;;                     # read-and-release chase coverage
  read5-gui)     shift; export GZ_GUI=1; read5 "${1:-420}" ;;
  *) echo "usage: $0 {install-model|bridge --topic T [--port P]|stop-bridge|stageA [secs]|stageB [secs]|probe3 [secs]|stageB3 [secs]|abort3 [secs]|kill3 [secs]|stageB3-stop|lanes3 [secs]|track3 [secs]|lanes3-gui [secs]|track3-gui [secs]|dyn3 [secs]|dyn3-gui [secs]|dyn5 [secs]|dyn5-kill [secs]|handover3 [secs]|handover3-gui [secs]|lawn1 [secs]|lawn1-gui [secs]|lawn3 [secs]|lawn3-gui [secs]|read5 [secs]|read5-gui [secs]}" >&2; exit 64 ;;
esac
