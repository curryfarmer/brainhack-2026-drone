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
cam_topic_n() { echo "/world/convoy_px4/model/x500_mono_cam_640_$1/link/camera_link/sensor/camera/image"; }

# Spawn poses (ENU x,y,z,r,p,yaw), >=1.2 m from every robot start (robot_7 @ origin):
# alpha (1.2,0.2) E, bravo (-1.2,0.2) W, charlie (0,-2) = convoy-circle CENTRE.
SIM5_POSES=( "1.2,0.2,0.2,0,0,0" "-1.2,0.2,0.2,0,0,0" "0,-2,0.2,0,0,0" )

stageB3_stop() {
  stop_bridge
  local i
  for i in 0 1 2; do
    if [ -f "$RUN/px4_vision_$i.pid" ]; then
      kill -9 "$(cat "$RUN/px4_vision_$i.pid")" 2>/dev/null; rm -f "$RUN/px4_vision_$i.pid"
    fi
  done
  pkill -9 -f 'bin/px4 -i' 2>/dev/null
  bash "$REPO/sim/run_convoy.sh" stop 2>/dev/null   # ros bridge + driver
  pkill -9 -f 'convoy_px4' 2>/dev/null               # PX4's gz server (this world)
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
  cp "$REPO/sim/worlds/convoy_px4.sdf" "$HOME/PX4-Autopilot/Tools/simulation/gz/worlds/" 2>/dev/null
  export DISPLAY="${DISPLAY:-:0}"
  export GZ_SIM_RESOURCE_PATH="$REPO/sim/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"

  local i
  for i in 0 1 2; do
    local log="$RUN/px4_vision_$i.log" pidf="$RUN/px4_vision_$i.pid"
    echo "[launch3] instance $i pose=${SIM5_POSES[$i]} -> $log"
    if (( i == 0 )); then
      ( cd "$HOME/PX4-Autopilot" && \
        LIBGL_ALWAYS_SOFTWARE=1 HEADLESS=1 PX4_SYS_AUTOSTART=4001 \
          PX4_SIM_MODEL=gz_x500_mono_cam_640 PX4_GZ_WORLD=convoy_px4 \
          PX4_GZ_MODEL_POSE="${SIM5_POSES[$i]}" \
          setsid ./build/px4_sitl_default/bin/px4 -i "$i" -d > "$log" 2>&1 & \
        echo $! > "$pidf" )
    else
      ( cd "$HOME/PX4-Autopilot" && \
        HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1 \
          PX4_SIM_MODEL=gz_x500_mono_cam_640 PX4_GZ_WORLD=convoy_px4 \
          PX4_GZ_MODEL_POSE="${SIM5_POSES[$i]}" \
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

# Read sim-time milliseconds from one /clock message (sim block only).
_clock_sim_ms() {
  gz topic -e -t /clock -n 1 2>/dev/null | awk '
    /^sim \{/{insim=1; next}
    insim && /sec:/ {s=$2}
    insim && /nsec:/{printf "%d", s*1000 + int($2/1000000); exit}'
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

# stageB3 (gate V2 full): 3 PX4 camera-drones fly sentry_scan over the moving
# convoy, each reading its OWN onboard cam via its OWN bridge (5600/5601/5602).
stageB3() {
  local secs="${1:-180}"
  launch3 || return $?

  echo "[stageB3] driving the convoy"
  bash "$REPO/sim/run_convoy.sh" drive "$((secs + 150))" || echo "[stageB3] WARN convoy drive failed — markers static" >&2

  echo "[stageB3] settling EKF 60s (3-instance lockstep, convoy moving)"
  sleep 60

  local i
  for i in 0 1 2; do
    bridge --topic "$(cam_topic_n "$i")" --port "$((5600 + i))" || { stageB3_stop; return 5; }
  done

  echo "[stageB3] finals sitl3_vision (sentry_scan x3) budget=${secs}s"
  local rc=0
  ( cd "$REPO" && .venv/bin/python -m finals.main --profile sitl \
      --config finals/configs/sitl3_vision.json --budget "$secs" ) || rc=$?
  stageB3_stop
  echo "[stageB3] finals rc=$rc — sightings.csv under $REPO/runs_finals/<latest>/"
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
  stageB3)       shift; stageB3 "$@" ;;
  stageB3-stop)  stageB3_stop ;;
  *) echo "usage: $0 {install-model|bridge --topic T [--port P]|stop-bridge|stageA [secs]|stageB [secs]|probe3 [secs]|stageB3 [secs]|stageB3-stop}" >&2; exit 64 ;;
esac
