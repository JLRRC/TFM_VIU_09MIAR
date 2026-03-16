#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/diag_gazebo_camera_ctrl.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -u

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_GZ="${WS_DIR}/log/gz_server.log"

echo "[DIAG] Workspace: ${WS_DIR}"
echo "[DIAG] Date: $(date -Iseconds)"

echo
echo "[DIAG] Camera topics"
ros2 topic list | grep -E '^/camera' || echo "[WARN] No /camera_* topics listed"

echo
echo "[DIAG] /clock hz (5s)"
timeout 6s ros2 topic hz /clock || echo "[WARN] /clock hz failed or timeout"

echo
echo "[DIAG] Camera hz (prefer /camera_overhead/image)"
if ros2 topic list | grep -qx '/camera_overhead/image'; then
  timeout 6s ros2 topic hz /camera_overhead/image || echo "[WARN] /camera_overhead/image hz timeout"
elif ros2 topic list | grep -qx '/camera_overhead/image_raw'; then
  timeout 6s ros2 topic hz /camera_overhead/image_raw || echo "[WARN] /camera_overhead/image_raw hz timeout"
else
  echo "[WARN] No overhead camera topic found"
fi

echo
echo "[DIAG] Gazebo-ready gate for list_controllers"
if ros2 service list | grep -qx '/controller_manager/list_controllers'; then
  timeout 5s ros2 control list_controllers || echo "[WARN] list_controllers timeout"
else
  echo "[INFO] /controller_manager/list_controllers not available (gazebo_not_ready?)"
fi

echo
echo "[DIAG] use_sim_time (best-effort)"
for n in /robot_state_publisher /move_group /controller_manager /ur5_qt_panel; do
  ros2 param get "${n}" use_sim_time 2>/dev/null || echo "[INFO] ${n} unavailable"
done

echo
echo "[DIAG] tf quick check world->base_link (3s)"
timeout 3s ros2 run tf2_ros tf2_echo world base_link || echo "[WARN] tf2_echo timeout/fail"

echo
echo "[DIAG] gz_server tail"
if [[ -f "${LOG_GZ}" ]]; then
  tail -n 80 "${LOG_GZ}"
else
  echo "[WARN] ${LOG_GZ} not found"
fi

echo
echo "[DIAG] Done"
