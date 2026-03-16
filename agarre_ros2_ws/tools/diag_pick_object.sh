#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/tools/diag_pick_object.sh
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
set -u

echo "[DIAG] pick_object diagnostics started: $(date -Iseconds)"
echo "[DIAG] cwd=$(pwd)"

run_cmd() {
  local title="$1"
  shift
  echo
  echo "===== ${title} ====="
  echo "+ $*"
  "$@"
  local rc=$?
  echo "[RC] ${rc}"
}

run_cmd "ROS Nodes (moveit/bridge/panel/controllers)" \
  bash -lc 'ros2 node list | grep -E "move_group|moveit|bridge|panel|controller_manager" || true'

run_cmd "Topic /desired_grasp info" \
  ros2 topic info /desired_grasp -v

run_cmd "Topic /desired_grasp/result info" \
  ros2 topic info /desired_grasp/result -v

run_cmd "Controllers list" \
  ros2 control list_controllers

if [[ -f "log/ros2_launch.log" ]]; then
  run_cmd "Validate latest PICK Objeto from log" \
    python3 tools/validate_pick_object_log.py --log log/ros2_launch.log
else
  echo
  echo "===== Validate latest PICK Objeto from log ====="
  echo "[SKIP] log/ros2_launch.log not found"
fi

echo
echo "[DIAG] pick_object diagnostics finished: $(date -Iseconds)"
