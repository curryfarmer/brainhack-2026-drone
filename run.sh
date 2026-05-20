#!/usr/bin/env bash
# Edited by Claude — drop-in launcher for the no-git ZIP-download workflow.
# Usage: ./run.sh collect_yolo_data.py     (any script in this folder)
# Kills zombie mavsdk_server, names whoever else owns UDP :14540, then runs
# the script. See README §12.

set -eu

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <script.py> [script args...]" >&2
  exit 2
fi

pkill -9 -f mavsdk_server 2>/dev/null || true
sleep 0.5

PORT_OWNER="$(ss -ulpn 2>/dev/null | awk '$5 ~ /:14540$/ {print $0}' || true)"
if [[ -n "$PORT_OWNER" ]]; then
  echo "[run.sh] WARN: UDP :14540 already owned by another process:"
  echo "         $PORT_OWNER"
  echo "[run.sh] If this is PX4 itself, mavsdk needs udpout://, not udpin://."
  echo "[run.sh] If this is a stale process you cannot identify, run:"
  echo "         sudo lsof -iUDP:14540   (then kill its PID)"
fi

exec python3 "$@"
