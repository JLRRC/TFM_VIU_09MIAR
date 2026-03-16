#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/status_panel_v2.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="$WS_DIR/log/ros2_launch.pid"
PANEL_CONTROLLER_MANAGER="${PANEL_CONTROLLER_MANAGER:-/controller_manager}"
echo "[STATUS_PANEL_V2] PANEL_CONTROLLER_MANAGER=${PANEL_CONTROLLER_MANAGER}"

if [[ ! -f "$PID_FILE" ]]; then
  echo "[STATUS_PANEL_V2] No PID file at $PID_FILE"
  exit 1
fi

pid="$(cat "$PID_FILE" 2>/dev/null || true)"
if [[ -z "$pid" ]]; then
  echo "[STATUS_PANEL_V2] Empty PID file: $PID_FILE"
  exit 1
fi

if kill -0 "$pid" 2>/dev/null; then
  echo "[STATUS_PANEL_V2] PID $pid is running"
  ps -p "$pid" -o pid,ppid,stat,cmd
else
  echo "[STATUS_PANEL_V2] PID $pid not running"
  exit 1
fi

echo "[STATUS_PANEL_V2] Zombie processes (ros2/tee):"
ps -e -o pid,ppid,stat,cmd | awk '$3 ~ /Z/ && ($4 ~ /ros2/ || $4 ~ /tee/) {print}'
