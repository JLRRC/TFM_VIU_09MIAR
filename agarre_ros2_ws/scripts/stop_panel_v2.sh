#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/stop_panel_v2.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="$WS_DIR/log/ros2_launch.pid"
PANEL_CONTROLLER_MANAGER="${PANEL_CONTROLLER_MANAGER:-/controller_manager}"
echo "[STOP_PANEL_V2] PANEL_CONTROLLER_MANAGER=${PANEL_CONTROLLER_MANAGER}"

if [[ ! -f "$PID_FILE" ]]; then
  echo "[STOP_PANEL_V2] No PID file at $PID_FILE"
  echo "[STOP_PANEL_V2] Parada idempotente: intentando detener procesos residuales."
  pkill -f "ros2 launch ur5_bringup" 2>/dev/null || true
  pkill -f "ur5_qt_panel.*panel_v2|panel_v2.py" 2>/dev/null || true
  exit 0
fi

pid="$(cat "$PID_FILE" 2>/dev/null || true)"
if [[ -z "$pid" ]]; then
  echo "[STOP_PANEL_V2] Empty PID file: $PID_FILE"
  exit 1
fi

if kill -0 "$pid" 2>/dev/null; then
  echo "[STOP_PANEL_V2] Stopping PID $pid"
  kill "$pid"
  sleep 0.5
else
  echo "[STOP_PANEL_V2] PID $pid not running"
fi

rm -f "$PID_FILE"
pkill -f "ur5_qt_panel.*panel_v2|panel_v2.py" 2>/dev/null || true
