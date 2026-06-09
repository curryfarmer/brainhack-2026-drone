#!/usr/bin/env bash
# SIM-3 convoy-world orchestration (VM only; gz-harmonic + ros_gz).
#
#   bash sim/run_convoy.sh all [secs] [--sw]   # start -> bridge+drive -> check -> stop
#   bash sim/run_convoy.sh start [--sw]        # gz server only (--sw = llvmpipe rung)
#   bash sim/run_convoy.sh check [secs]        # detection check against a running world
#   bash sim/run_convoy.sh stop                # kill recorded PIDs + cleanliness check
#
# Interpreter contexts (never crossed): gz launch (any shell) | bridge+driver (ROS
# sourced) | check_detection (PYTHONNOUSERSITE=1 python3, system 3.10). Fail-loud: every
# wait has a deadline; missing camera topic / driver exits nonzero with WHAT/WHY/CHECK.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN="$REPO/sim/run"; mkdir -p "$RUN"
WORLD="$REPO/sim/worlds/convoy.sdf"
ROS_SETUP="/opt/ros/humble/setup.bash"
IDS=(7 11 23 42 88)
TOPIC_DEADLINE_S=40

export GZ_SIM_RESOURCE_PATH="$REPO/sim/models:${GZ_SIM_RESOURCE_PATH:-}"

start() {
  local sw=""; [ "${1:-}" = "--sw" ] && sw="LIBGL_ALWAYS_SOFTWARE=1"
  echo "[start] gz server ${sw:+(llvmpipe) }$WORLD"
  env $sw gz sim -s -r "$WORLD" > "$RUN/gz_convoy.log" 2>&1 &
  echo $! > "$RUN/gz_convoy.pid"
  local t0=$SECONDS
  until gz topic -l 2>/dev/null | grep -q cam_band_120; do
    if (( SECONDS - t0 > TOPIC_DEADLINE_S )); then
      echo "FAIL [start]: no /world/convoy/.../cam_band_120/.../image within ${TOPIC_DEADLINE_S}s" >&2
      echo "  WHY: Sensors plugin missing, GZ_SIM_RESOURCE_PATH wrong, or render stall" >&2
      echo "  CHECK: tail $RUN/gz_convoy.log ; gz topic -l" >&2
      return 3
    fi
    sleep 1
  done
  echo "[start] camera topics up after $((SECONDS - t0))s"
}

drive() {
  local secs="${1:-65}"
  # shellcheck disable=SC1090
  source "$ROS_SETUP" || { echo "FAIL [drive]: cannot source $ROS_SETUP" >&2; return 2; }
  local bridge_args=()
  for i in "${IDS[@]}"; do
    bridge_args+=("/model/convoy_robot_${i}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist")
  done
  echo "[drive] ros_gz bridge (${#IDS[@]} robots) + convoy_driver ${secs}s"
  ros2 run ros_gz_bridge parameter_bridge "${bridge_args[@]}" > "$RUN/bridge.log" 2>&1 &
  echo $! > "$RUN/bridge.pid"
  sleep 2
  python3 "$REPO/sim/convoy_driver.py" --duration-s "$secs" > "$RUN/driver.log" 2>&1 &
  echo $! > "$RUN/driver.pid"
}

check() {
  local secs="${1:-40}"
  echo "[check] detection check ${secs}s (PYTHONNOUSERSITE=1 system python3)"
  PYTHONNOUSERSITE=1 python3 "$REPO/sim/check_detection.py" --secs "$secs"
}

stop() {
  echo "[stop] killing recorded PIDs"
  for p in driver bridge gz_convoy; do
    [ -f "$RUN/$p.pid" ] && kill "$(cat "$RUN/$p.pid")" 2>/dev/null && rm -f "$RUN/$p.pid"
  done
  sleep 2
  # bracket form: a plain 'gz sim' pattern self-matches the wrapping shell (sim/README)
  local leftover; leftover="$(pgrep -fa 'g[z] sim' || true)"
  if [ -n "$leftover" ]; then
    echo "[stop] WARN leftover gz processes:"; echo "$leftover"
  else
    echo "[stop] clean (no g[z] sim leftovers)"
  fi
}

case "${1:-all}" in
  start) shift; start "$@";;
  drive) shift; drive "$@";;
  check) shift; check "$@";;
  stop)  stop;;
  all)
    shift
    secs="40"; sw=""
    for a in "$@"; do [ "$a" = "--sw" ] && sw="--sw" || secs="$a"; done
    start $sw || exit $?
    drive "$((secs + 25))" || { stop; exit 2; }
    rc=0; check "$secs" || rc=$?
    stop
    exit $rc
    ;;
  *) echo "usage: $0 {all [secs] [--sw]|start [--sw]|drive [secs]|check [secs]|stop}" >&2; exit 64;;
esac
