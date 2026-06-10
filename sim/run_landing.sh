#!/usr/bin/env bash
# NAV-9 LANDING SITL orchestration (VM only) — the Challenge-2A rehearsal rig.
#
#   bash sim/run_landing.sh install            # x500 cam model + pad_102 texture + copy worlds into PX4
#   bash sim/run_landing.sh land1   [secs]     # gate L1: 1 drone, full [takeoff,navigate,land_on_pad]
#   bash sim/run_landing.sh land3   [secs]     # gate L2: 3 drones, staggered + serialized landing
#   bash sim/run_landing.sh abort3  [secs]     # drill: 'q' lands all (orderly)
#   bash sim/run_landing.sh kill3   [secs]     # drill: kill instance 2 mid-mission (isolation)
#   bash sim/run_landing.sh viewtest [secs]    # WATCHABLE: 1 drone in landing_view + record overview+onboard mp4
#   bash sim/run_landing.sh stop               # tear everything down
#
# WHAT THIS IS (and is NOT): a PX4+Gazebo SITL rehearsal of the BACKEND-AGNOSTIC
# navigation LOGIC (planner -> navigate -> land_on_pad -> deconfliction). The REAL
# drone is HULA via finals' pyhulax adapter (configs/landing_real.json,
# flight_backend=pyhulax) — NOT PX4. The nav phases sit ABOVE the FlightAdapter
# boundary and emit the same Action vocabulary (Takeoff/Rotate/Move/Hover/Land)
# regardless of backend, so SITL proves the ALGORITHM; the HULA-specific constants
# (move-cm calibration, yaw trust, camera HFOV, ArUco decode range) are deferred to
# the onsite bench gates (B2/B3/P0/E/F/M in onsite_test_plan.md). SITL is a logic
# rehearsal, not a hardware twin.
#
# Forked from sim/run_vision.sh (SIM-4/5 — same bridge/lockstep/llvmpipe pattern).
# Interpreter contexts (sim/README): gz launch (any shell) | gz_camera_bridge +
# gz_video_record (PYTHONNOUSERSITE=1 system python3, gz bindings + cv2) | finals
# (.venv python 3.11). NO convoy driver here (landing has no convoy). Fail-loud:
# every wait has a deadline.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN="$REPO/sim/run"; mkdir -p "$RUN"
PX4="$HOME/PX4-Autopilot"
PX4_MODELS="$PX4/Tools/simulation/gz/models"
PX4_WORLDS="$PX4/Tools/simulation/gz/worlds"
VENV_PY="$REPO/.venv/bin/python"
BRIDGE_READY_DEADLINE_S=60
SETTLE_1=45         # EKF settle, single instance
SETTLE_3=120        # EKF settle, 3-instance lockstep (standalone-joiners converge slowest)

# Spawn poses (ENU x=E,y=N,z,r,p,yaw). alpha AT the C2 origin (0,0) so its plan
# (from c2_origin) has zero start-offset = the cleanest avoidance+land demo; bravo
# W, charlie E, >=1 m apart (x500 ~0.5 m wide) so they don't interpenetrate at t=0.
POSES=( "0,0,0.2,0,0,0" "-1.0,0,0.2,0,0,0" "1.0,0,0.2,0,0,0" )

cam_topic() { echo "/world/$1/model/x500_mono_cam_640_$2/link/camera_link/sensor/camera/image"; }

# --------------------------------------------------------------------------- #
install() {
  [ -d "$PX4_MODELS" ] || { echo "FAIL [install]: $PX4_MODELS missing — is PX4-Autopilot built?" >&2; return 2; }
  cp -r "$REPO/sim/px4_models/x500_mono_cam_640" "$PX4_MODELS/"
  echo "[install] x500_mono_cam_640 -> $PX4_MODELS"
  # pad_102 has hand-authored model.sdf/.config in the repo; generate ONLY its
  # ArUco texture (the binary png is a build artifact, not committed). pad_100/101
  # textures already SHIP in the repo, so leave them untouched. gen_markers falls
  # back to the legacy drawMarker on the VM's cv2 4.5.4 (no generateImageMarker).
  if [ -f "$REPO/sim/models/pad_102/materials/textures/pad_102.png" ]; then
    echo "[install] pad_102 texture already present"
  elif python3 "$REPO/sim/gen_markers.py" --type aruco --kind plane --prefix pad \
        --ids 102 --size-cm 40 ; then
    echo "[install] pad_102 ArUco texture generated"
  else
    echo "FAIL [install]: gen_markers (pad_102) failed — WHY: cv2.aruco missing? — CHECK: pip install opencv-contrib-python 'numpy<2'" >&2
    return 3
  fi
  mkdir -p "$PX4_WORLDS"
  cp "$REPO/sim/worlds/landing_px4.sdf"  "$PX4_WORLDS/" && echo "[install] landing_px4.sdf  -> $PX4_WORLDS"
  cp "$REPO/sim/worlds/landing_view.sdf" "$PX4_WORLDS/" && echo "[install] landing_view.sdf -> $PX4_WORLDS"
}

# Launch one gz->TCP bridge (PID + ready gate), per-port files (3 concurrent).
bridge() {
  local topic="$1" port="$2"
  local log="$RUN/bridge_${port}.log" ready="$RUN/bridge_${port}.ready" pidf="$RUN/bridge_${port}.pid"
  rm -f "$ready"
  PYTHONNOUSERSITE=1 python3 "$REPO/sim/gz_camera_bridge.py" --topic "$topic" --port "$port" \
      --ready-file "$ready" > "$log" 2>&1 &
  echo $! > "$pidf"
  local t0=$SECONDS pid; pid="$(cat "$pidf")"
  until [ -f "$ready" ] || grep -q "BRIDGE READY" "$log" 2>/dev/null; do
    kill -0 "$pid" 2>/dev/null || { echo "FAIL [bridge]: PID $pid died before READY — CHECK: tail $log" >&2; return 3; }
    (( SECONDS - t0 > BRIDGE_READY_DEADLINE_S )) && { echo "FAIL [bridge]: no READY in ${BRIDGE_READY_DEADLINE_S}s on $topic — CHECK: tail $log" >&2; return 4; }
    sleep 1
  done
  echo "[bridge] ready after $((SECONDS - t0))s (PID $pid, port $port)"
}

# Launch a recorder (mp4) on a topic; record for `secs` then it self-exits.
recorder() {
  local topic="$1" out="$2" secs="$3" fps="${4:-10}"
  local base; base="$(basename "$out" .mp4)"
  PYTHONNOUSERSITE=1 python3 "$REPO/sim/gz_video_record.py" --topic "$topic" \
      --out "$out" --secs "$secs" --fps "$fps" > "$RUN/record_${base}.log" 2>&1 &
  echo $! > "$RUN/record_${base}.pid"
  echo "[recorder] $topic -> $out (${secs}s) PID $(cat "$RUN/record_${base}.pid")"
}

# Launch PX4 instance i into `world`. i0 OWNS the gz server (llvmpipe renders all
# cams); 1/2 JOIN via PX4_GZ_STANDALONE=1. Gated on instance i's camera topic.
launch_instance() {
  local world="$1" i="$2"
  local log="$RUN/px4_${i}.log" pidf="$RUN/px4_${i}.pid"
  echo "[launch] $world instance $i pose=${POSES[$i]} -> $log"
  if (( i == 0 )); then
    ( cd "$PX4" && \
      LIBGL_ALWAYS_SOFTWARE=1 HEADLESS=1 PX4_SYS_AUTOSTART=4001 \
        PX4_SIM_MODEL=gz_x500_mono_cam_640 PX4_GZ_WORLD="$world" \
        PX4_GZ_MODEL_POSE="${POSES[$i]}" \
        setsid ./build/px4_sitl_default/bin/px4 -i "$i" -d > "$log" 2>&1 & \
      echo $! > "$pidf" )
  else
    ( cd "$PX4" && \
      HEADLESS=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_STANDALONE=1 \
        PX4_SIM_MODEL=gz_x500_mono_cam_640 PX4_GZ_WORLD="$world" \
        PX4_GZ_MODEL_POSE="${POSES[$i]}" \
        setsid ./build/px4_sitl_default/bin/px4 -i "$i" -d > "$log" 2>&1 & \
      echo $! > "$pidf" )
  fi
  local t0=$SECONDS pid; pid="$(cat "$pidf")"
  until gz topic -l 2>/dev/null | grep -q "x500_mono_cam_640_$i/link/camera_link/sensor/camera/image"; do
    kill -0 "$pid" 2>/dev/null || { echo "FAIL [launch]: instance $i (PID $pid) died before its camera topic — CHECK: tail $log" >&2; return 3; }
    (( SECONDS - t0 > 120 )) && { echo "FAIL [launch]: instance $i camera topic never appeared in 120s — CHECK: tail $log" >&2; return 4; }
    sleep 2
  done
  echo "[launch] instance $i camera up after $((SECONDS - t0))s"
}

stop() {
  # recorders + bridges first (clean mp4 finalize on SIGTERM)
  local f pid
  for f in "$RUN"/record_*.pid "$RUN"/bridge_*.pid; do
    [ -e "$f" ] || continue
    pid="$(cat "$f")"; kill -TERM "$pid" 2>/dev/null; rm -f "$f"
  done
  sleep 2
  for f in "$RUN"/px4_*.pid; do
    [ -e "$f" ] || continue
    kill -9 "$(cat "$f")" 2>/dev/null; rm -f "$f"
  done
  pkill -9 -f 'bin/px4 -i' 2>/dev/null
  pkill -9 -f 'landing_px4\|landing_view' 2>/dev/null   # PX4's gz server (these worlds)
  pkill -9 -f 'g[z] sim' 2>/dev/null                     # last resort (bracket = no self-match)
  sleep 1
  echo "[stop] torn down"
}

# --------------------------------------------------------------------------- #
# Gate L1: 1 drone, full landing pipeline, headless.
land1() {
  local secs="${1:-300}"
  install
  export DISPLAY="${DISPLAY:-:0}"
  export GZ_SIM_RESOURCE_PATH="$REPO/sim/models:$PX4_MODELS:${GZ_SIM_RESOURCE_PATH:-}"
  launch_instance landing_px4 0 || { stop; return 3; }
  echo "[land1] settling EKF ${SETTLE_1}s"; sleep "$SETTLE_1"
  bridge "$(cam_topic landing_px4 0)" 5600 || { stop; return 4; }
  echo "[land1] finals sitl1_landing budget=${secs}s"
  local rc=0
  ( cd "$REPO" && "$VENV_PY" -m finals.main --profile sitl \
      --config finals/configs/sitl1_landing.json --budget "$secs" ) || rc=$?
  stop
  echo "[land1] finals rc=$rc — logs under $REPO/runs_finals/<latest>/"
  return $rc
}

# Gates L2 / drills: 3 drones. mode: normal | abort | kill.
land3() {
  local secs="${1:-700}" mode="${2:-normal}"
  install
  export DISPLAY="${DISPLAY:-:0}"
  export GZ_SIM_RESOURCE_PATH="$REPO/sim/models:$PX4_MODELS:${GZ_SIM_RESOURCE_PATH:-}"
  local i
  for i in 0 1 2; do launch_instance landing_px4 "$i" || { stop; return 3; }; done
  echo "[land3] all 3 instances up"
  echo "[land3] settling EKF ${SETTLE_3}s (3-instance lockstep)"; sleep "$SETTLE_3"
  for i in 0 1 2; do bridge "$(cam_topic landing_px4 "$i")" "$((5600 + i))" || { stop; return 4; }; done

  if [ "$mode" = "kill" ]; then
    ( sleep "${KILL_AFTER:-90}"
      kp="$(pgrep -f 'bin/px4 -i 2' | tr '\n' ' ')"
      echo "[land3] DRILL kill3: kill -9 instance 2 (charlie) px4 -i 2 pid(s)=${kp:-NONE}" >&2
      pkill -9 -f 'bin/px4 -i 2' 2>/dev/null ) &
  fi

  echo "[land3] finals sitl3_landing (3x landing) budget=${secs}s mode=${mode}"
  local rc=0
  if [ "$mode" = "abort" ]; then
    ( cd "$REPO" && PYTHONNOUSERSITE=1 python3 sim/pty_q_harness.py \
        --trigger-regex 'offboard active' --trigger-count 3 --fallback-secs 120 -- \
        "$VENV_PY" -m finals.main --profile sitl \
        --config finals/configs/sitl3_landing.json --budget "$secs" ) || rc=$?
  else
    ( cd "$REPO" && "$VENV_PY" -m finals.main --profile sitl \
        --config finals/configs/sitl3_landing.json --budget "$secs" ) || rc=$?
  fi
  stop
  echo "[land3] finals rc=$rc — logs under $REPO/runs_finals/<latest>/"
  return $rc
}

# WATCHABLE footage: 1 drone in landing_view (= landing_px4 + overhead overview
# cam). Records the overview (third-person) AND the onboard down-cam to mp4 while
# the drone flies takeoff -> navigate (crate detour) -> land_on_pad. 2 cameras
# total (onboard + overview) so the lockstep stays healthy on the 4-vCPU VM.
viewtest() {
  local secs="${1:-300}"
  install
  export DISPLAY="${DISPLAY:-:0}"
  export GZ_SIM_RESOURCE_PATH="$REPO/sim/models:$PX4_MODELS:${GZ_SIM_RESOURCE_PATH:-}"
  launch_instance landing_view 0 || { stop; return 3; }
  echo "[viewtest] settling EKF ${SETTLE_1}s"; sleep "$SETTLE_1"
  # Record for the whole flight window (budget + a tail margin).
  local rec_secs=$(( secs + 60 ))
  recorder "/world/landing_view/model/overview_cam/link/link/sensor/camera/image" \
           "$RUN/landing_overview.mp4" "$rec_secs" 12
  recorder "$(cam_topic landing_view 0)" "$RUN/landing_onboard_alpha.mp4" "$rec_secs" 6
  bridge "$(cam_topic landing_view 0)" 5600 || { stop; return 4; }
  echo "[viewtest] finals sitl1_landing budget=${secs}s (footage recording)"
  local rc=0
  ( cd "$REPO" && "$VENV_PY" -m finals.main --profile sitl \
      --config finals/configs/sitl1_landing.json --budget "$secs" ) || rc=$?
  echo "[viewtest] finals done rc=$rc — letting recorders flush 5s"; sleep 5
  stop
  echo "[viewtest] FOOTAGE:"
  echo "  third-person : $RUN/landing_overview.mp4"
  echo "  onboard cam  : $RUN/landing_onboard_alpha.mp4"
  echo "  (replay track PNG under $REPO/runs_finals/<latest>/ if replay_plot ran)"
  return $rc
}

case "${1:-}" in
  install)  install ;;
  land1)    shift; land1 "$@" ;;
  land3)    shift; land3 "${1:-700}" normal ;;
  abort3)   shift; land3 "${1:-700}" abort ;;
  kill3)    shift; land3 "${1:-700}" kill ;;
  viewtest) shift; viewtest "$@" ;;
  stop)     stop ;;
  *) echo "usage: $0 {install|land1 [secs]|land3 [secs]|abort3 [secs]|kill3 [secs]|viewtest [secs]|stop}" >&2; exit 64 ;;
esac
