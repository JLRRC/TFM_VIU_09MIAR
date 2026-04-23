#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/stop_panel_v2.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="$WS_DIR/log/ros2_launch.pid"
PANEL_CONTROLLER_MANAGER="${PANEL_CONTROLLER_MANAGER:-/controller_manager}"
echo "[STOP_PANEL_V2] PANEL_CONTROLLER_MANAGER=${PANEL_CONTROLLER_MANAGER}"

stop_residual_processes() {
  local sig="${1:-TERM}"
  # Kill by broad launch/process patterns first.
  pkill "-$sig" -f "ros2 launch ur5_bringup" 2>/dev/null || true
  pkill "-$sig" -f "ur5_qt_panel.*panel_v2|panel_v2.py|panel_v2_moveit_publisher|ur5_qt_panel" 2>/dev/null || true
  pkill "-$sig" -f "move_group" 2>/dev/null || true
  pkill "-$sig" -f "ur5_moveit_bridge" 2>/dev/null || true
  pkill "-$sig" -f "controller_manager|ros2_control_node" 2>/dev/null || true
  pkill "-$sig" -f "controller_bootstrap" 2>/dev/null || true
  pkill "-$sig" -f "system_state_manager|release_objects_service" 2>/dev/null || true
  pkill "-$sig" -f "gripper_attach_backend|planning_scene_sync" 2>/dev/null || true
  pkill "-$sig" -f "gz_pose_bridge|gz_ros_control_guard|world_tf_publisher" 2>/dev/null || true
  pkill "-$sig" -f "ros_gz_bridge|parameter_bridge" 2>/dev/null || true
  pkill "-$sig" -f "robot_state_publisher" 2>/dev/null || true
  pkill "-$sig" -f "gz sim|gzserver|gzclient|ign gazebo" 2>/dev/null || true
}

drain_residual_processes() {
  # Some ROS/Gazebo children exit with delay; run a few TERM/KILL sweeps.
  for _ in 1 2 3; do
    stop_residual_processes TERM
    sleep 1
    stop_residual_processes KILL
    sleep 1
  done
}

refresh_ros_graph_cache() {
  if [[ "${PANEL_RESET_ROS_DAEMON_ON_STOP:-1}" != "1" ]]; then
    return 0
  fi
  if ! command -v ros2 >/dev/null 2>&1; then
    return 0
  fi
  # Clears stale graph cache entries so post-stop checks are deterministic.
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 0.5
  ros2 daemon start >/dev/null 2>&1 || true
  sleep 0.5
}

if [[ ! -f "$PID_FILE" ]]; then
  echo "[STOP_PANEL_V2] No PID file at $PID_FILE"
  echo "[STOP_PANEL_V2] Parada idempotente: intentando detener procesos residuales."
  drain_residual_processes
  refresh_ros_graph_cache
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
drain_residual_processes
refresh_ros_graph_cache
