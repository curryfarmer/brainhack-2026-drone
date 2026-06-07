#!/usr/bin/env bash
# sim/launch_sitl.sh — PX4-SITL multi-instance launcher for the BrainHack finals VM.
#
# Usage:
#   bash sim/launch_sitl.sh start N [--world W] [--model M]
#   bash sim/launch_sitl.sh stop
#   bash sim/launch_sitl.sh status
#
# Port map (instance i):
#   - PX4 BINDS UDP 14580+i (offboard-local) and 18570+i (GCS-local) — these are what
#     `ss -ulpn` shows after start. Verified in
#     ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink.
#   - PX4 SENDS offboard MAVLink to 14540+i; a MAVSDK client binds it via
#     `udpin://0.0.0.0:1454<i>`, so 1454x appears in `ss` only WHILE a client runs.
#   - mavsdk_server gRPC: 50051+i (client side). MAV_SYS_ID = i+1.
#
# PID files in sim/run/ exist precisely so kill drills are scriptable:
#   kill -9 "$(cat sim/run/px4_1.pid)"
#
# BANNED: global `pkill -f mavsdk_server` while anything runs (sim_sessions.md recap §3)
# — it kills ALL drones' servers, not just the broken one. `stop` kills ONLY the PIDs it
# recorded (px4 instances + the gz server tree instance 0 spawned).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="$SCRIPT_DIR/run"
PX4_DIR="$HOME/PX4-Autopilot"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"

# Deadlines sized for a 2-vCPU VM (sim_sessions.md recap §8: slow != wrong, but every
# wait still needs a hard deadline — recap §6).
GZ_READY_DEADLINE_S=90
INST_READY_DEADLINE_S=60
STOP_TERM_GRACE_S=10

die()  { echo "launch_sitl: ERROR: $*" >&2; exit 1; }
info() { echo "launch_sitl: $*"; }

pid_alive() { kill -0 "$1" 2>/dev/null; }

# Wait for the gz server that instance 0 spawns (a /clock topic appears once the world
# is running). Instances 1+ flake if started before this — hence the hard gate.
wait_gz_ready() {
    local log="$1" t0=$SECONDS pid
    info "waiting for gz server (deadline ${GZ_READY_DEADLINE_S}s)..."
    while (( SECONDS - t0 < GZ_READY_DEADLINE_S )); do
        if gz topic -l 2>/dev/null | grep -q "/clock"; then
            info "gz server ready after $(( SECONDS - t0 ))s"
            return 0
        fi
        pid="$(cat "$RUN_DIR/px4_0.pid")"
        pid_alive "$pid" || die "instance 0 (PID $pid) exited before the gz server came up — CHECK: tail $log"
        sleep 1
    done
    die "gz server not ready after ${GZ_READY_DEADLINE_S}s (no /clock topic) — instance 0 failed to boot Gazebo; CHECK: tail $log"
}

# Wait for instance i's PX4-bound offboard-local port 14580+i to appear in ss — that is
# the proof the instance's mavlink came up (14540+i only binds when a client connects).
wait_inst_ready() {
    local i="$1" log="$2" port=$((14580 + i)) t0=$SECONDS
    while (( SECONDS - t0 < INST_READY_DEADLINE_S )); do
        if ss -ulpn 2>/dev/null | grep -q ":$port "; then
            info "instance $i mavlink up (UDP $port bound) after $(( SECONDS - t0 ))s"
            return 0
        fi
        local pid; pid="$(cat "$RUN_DIR/px4_$i.pid")"
        pid_alive "$pid" || die "instance $i (PID $pid) exited before binding UDP $port — CHECK: tail $log"
        sleep 1
    done
    die "instance $i did not bind UDP $port within ${INST_READY_DEADLINE_S}s — CHECK: tail $log; ss -ulpn | grep $port"
}

cmd_start() {
    local n="${1:-}"; shift || true
    [[ "${n:-}" =~ ^[0-9]+$ ]] && (( n >= 1 )) || die "start needs an instance count >= 1 (got '${n:-}') — usage: start N [--world W] [--model M]"
    local world="" model="gz_x500"
    while (( $# )); do
        case "$1" in
            --world) world="${2:-}"; [[ -n "$world" ]] || die "--world needs a value"; shift 2 ;;
            --model) model="${2:-}"; [[ -n "$model" ]] || die "--model needs a value"; shift 2 ;;
            *) die "unknown start option '$1' (known: --world W, --model M)" ;;
        esac
    done

    [[ -x "$PX4_BIN" ]] || die "PX4 binary not found at $PX4_BIN — build it: cd $PX4_DIR && make px4_sitl"
    mkdir -p "$RUN_DIR"

    # State-aware start: a live gz.pid means our server is up — live instances are
    # skipped and dead ones relaunched into the running world (the kill-drill recovery
    # path). Without a live server, any surviving px4 is an unknown state: refuse.
    local server_up=0 f pid
    if [[ -e "$RUN_DIR/gz.pid" ]] && pid_alive "$(head -1 "$RUN_DIR/gz.pid")"; then
        server_up=1
    else
        for f in "$RUN_DIR"/px4_*.pid; do
            [[ -e "$f" ]] || continue
            pid="$(cat "$f")"
            pid_alive "$pid" && die "$(basename "$f") is alive (PID $pid) but our gz server is gone — kill it by PID, then rerun"
            rm -f "$f"  # stale leftover from a crash / kill drill
        done
        rm -f "$RUN_DIR/gz.pid"
    fi

    cd "$PX4_DIR"  # instance rootfs dirs resolve relative to here (official multi-vehicle pattern)

    local i
    for (( i = 0; i < n; i++ )); do
        local log="$RUN_DIR/px4_$i.log" pidf="$RUN_DIR/px4_$i.pid"
        if [[ -e "$pidf" ]] && pid_alive "$(cat "$pidf")"; then
            info "instance $i already running (PID $(cat "$pidf")) — skipping"
            continue
        fi
        rm -f "$pidf"

        local -a env_kv=(
            HEADLESS=1
            PX4_SYS_AUTOSTART=4001
            "PX4_SIM_MODEL=$model"
        )
        [[ -n "$world" ]] && env_kv+=("PX4_GZ_WORLD=$world")
        if (( server_up )); then
            env_kv+=(PX4_GZ_STANDALONE=1)  # join the running gz server
            # A killed px4 leaves its model in the world; a fresh spawn would collide
            # on the name — ATTACH to the surviving model instead (PX4_GZ_MODEL_NAME).
            local mname="${model#gz_}_$i"
            if gz model --list 2>/dev/null | grep -qE " $mname\$"; then
                info "model $mname already in the world — attaching instead of spawning"
                env_kv+=("PX4_GZ_MODEL_NAME=$mname")
            else
                env_kv+=("PX4_GZ_MODEL_POSE=0,$i")
            fi
        else
            env_kv+=("PX4_GZ_MODEL_POSE=0,$i")
        fi

        info "starting instance $i (model=$model${world:+ world=$world}) -> $log"
        env "${env_kv[@]}" nohup "$PX4_BIN" -i "$i" -d >"$log" 2>&1 &
        echo $! >"$pidf"

        if (( ! server_up )); then
            wait_gz_ready "$log"
            # Record the gz server PID (px4 spawns exactly one `gz sim --... -s <world>`
            # process) so stop and kill drills stay targeted — never a global pkill.
            # `gz sim --` is specific enough not to match shells quoting "gz sim".
            pgrep -f "gz sim --" >"$RUN_DIR/gz.pid" || die "instance $i is up but no 'gz sim --' server process found — CHECK: tail $log"
            server_up=1
        fi
        wait_inst_ready "$i" "$log"
    done
    info "start $n: all requested instances up"
    info "PX4-bound ports: $(ss -ulpn 2>/dev/null | grep -oE ':(1458[0-9]|1857[0-9])' | sort -u | tr -d ':' | tr '\n' ' ')"
    info "client udpin targets: $(for (( i = 0; i < n; i++ )); do printf '%s ' $((14540 + i)); done)(bind these from MAVSDK)"
}

cmd_stop() {
    local f pid killed=0
    for f in "$RUN_DIR"/px4_*.pid; do
        [[ -e "$f" ]] || continue
        pid="$(cat "$f")"
        if pid_alive "$pid"; then
            info "stopping $(basename "$f" .pid) (PID $pid)"
            kill -TERM "$pid" 2>/dev/null || true
            local t0=$SECONDS
            while pid_alive "$pid" && (( SECONDS - t0 < STOP_TERM_GRACE_S )); do sleep 1; done
            if pid_alive "$pid"; then
                info "$(basename "$f" .pid) ignored TERM after ${STOP_TERM_GRACE_S}s — sending KILL"
                kill -KILL "$pid" 2>/dev/null || true
            fi
            killed=1
        fi
        rm -f "$f"
    done

    # Kill the gz server we recorded at start — strictly by PID, NEVER a name-based
    # pkill: a loose `pkill -f "gz sim"` matches any shell whose command string merely
    # contains that text (a kill drill proved it by murdering the operator's ssh shell).
    if [[ -e "$RUN_DIR/gz.pid" ]]; then
        while read -r pid; do
            pid_alive "$pid" && { info "stopping gz server (PID $pid)"; kill -KILL "$pid" 2>/dev/null || true; }
        done <"$RUN_DIR/gz.pid"
        rm -f "$RUN_DIR/gz.pid"
        killed=1
    fi

    sleep 1
    # Leftover check: patterns specific to the real processes (`bin/px4 -i`, `gz sim --`)
    # so a caller shell quoting this script's name or a pgrep one-liner never matches.
    # Anything found is reported for MANUAL kill-by-PID — auto-killing by name is banned.
    local leftovers
    leftovers="$(pgrep -fa 'bin/px4 -i|gz sim --' || true)"
    if [[ -n "$leftovers" ]]; then
        die "stop finished but px4/gz processes remain — kill them by PID and investigate:
$leftovers"
    fi
    (( killed )) && info "stopped — no px4/gz processes remain" || info "nothing was running"
}

cmd_status() {
    echo "--- pgrep -fa 'bin/px4 -i|gz sim --' ---"
    pgrep -fa 'bin/px4 -i|gz sim --' || echo "(none)"
    echo "--- ss -ulpn | grep -E ':1454|:1458|:1857' ---"
    ss -ulpn 2>/dev/null | grep -E ':1454|:1458|:1857' || echo "(no sitl ports bound; 1454x binds only while a MAVSDK client runs)"
    echo "--- PID files in $RUN_DIR ---"
    local f pid
    for f in "$RUN_DIR"/px4_*.pid "$RUN_DIR"/gz.pid; do
        [[ -e "$f" ]] || continue
        pid="$(head -1 "$f")"
        if pid_alive "$pid"; then echo "$(basename "$f"): $pid (alive)"; else echo "$(basename "$f"): $pid (STALE)"; fi
    done
    [[ -d "$RUN_DIR" ]] && ls "$RUN_DIR"/px4_*.pid >/dev/null 2>&1 || echo "(no PID files)"
}

case "${1:-}" in
    start)  shift; cmd_start "$@" ;;
    stop)   cmd_stop ;;
    status) cmd_status ;;
    *) die "usage: bash sim/launch_sitl.sh start N [--world W] [--model M] | stop | status" ;;
esac
