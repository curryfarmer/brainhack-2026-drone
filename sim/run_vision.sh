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

  local log="$RUN/gz_bridge.log" ready="$RUN/gz_bridge.ready"
  rm -f "$ready"
  echo "[bridge] gz_camera_bridge topic=$topic port=$port -> $log"
  PYTHONNOUSERSITE=1 python3 "$REPO/sim/gz_camera_bridge.py" \
    --topic "$topic" --port "$port" --ready-file "$ready" > "$log" 2>&1 &
  echo $! > "$RUN/gz_bridge.pid"

  local t0=$SECONDS pid; pid="$(cat "$RUN/gz_bridge.pid")"
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

stop_bridge() {
  if [ -f "$RUN/gz_bridge.pid" ]; then
    local pid; pid="$(cat "$RUN/gz_bridge.pid")"
    kill -TERM "$pid" 2>/dev/null && echo "[stop-bridge] TERM -> PID $pid"
    sleep 1
    kill -0 "$pid" 2>/dev/null && { kill -KILL "$pid" 2>/dev/null; echo "[stop-bridge] KILL -> PID $pid"; }
    rm -f "$RUN/gz_bridge.pid"
  else
    echo "[stop-bridge] no recorded bridge PID"
  fi
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

case "${1:-}" in
  install-model) install_model ;;
  bridge)        shift; bridge "$@" ;;
  stop-bridge)   stop_bridge ;;
  stageA)        shift; stageA "$@" ;;
  *) echo "usage: $0 {install-model|bridge --topic T [--port P]|stop-bridge|stageA [secs]}" >&2; exit 64 ;;
esac
