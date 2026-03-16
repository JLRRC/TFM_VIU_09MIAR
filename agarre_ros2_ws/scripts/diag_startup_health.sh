#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/diag_startup_health.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="/home/laboratorio/TFM/agarre_ros2_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$WS_DIR/reports/diag_startup/$STAMP"
LOG_FILE="$OUT_DIR/health.log"
mkdir -p "$OUT_DIR"

cd "$WS_DIR"
set +u
source install/setup.bash
set -u

tf_echo_clean() {
  local target="$1"
  local source="$2"
  local tmp
  tmp="$(mktemp)"
  timeout 5s ros2 run tf2_ros tf2_echo "$target" "$source" >"$tmp" 2>&1 || true
  if grep -q '^At time' "$tmp"; then
    awk 'BEGIN{p=0} /^At time/{p=1} p{print}' "$tmp"
  else
    cat "$tmp"
  fi
  rm -f "$tmp"
}

CAM_REQUIRED="${PANEL_CAMERA_REQUIRED:-0}"

{
  echo "[HEALTH] stamp=$STAMP"
  echo "[HEALTH] cwd=$(pwd)"
  echo "[HEALTH] camera_required=$CAM_REQUIRED"

  echo "[HEALTH] 1) /clock --once (timeout 5s)"
  timeout 5s ros2 topic echo --once /clock || true

  echo "[HEALTH] 2) /joint_states --once (timeout 5s)"
  timeout 5s ros2 topic echo --once /joint_states || true

  echo "[HEALTH] 3) /tf hz (3s)"
  timeout 3s ros2 topic hz /tf || true

  echo "[HEALTH] 4) move_group node"
  ros2 node list | grep -E '^/move_group$' || true

  echo "[HEALTH] 5) TF chain checks"
  tf_echo_clean world base_link
  tf_echo_clean base_link rg2_tcp

  echo "[HEALTH] 6) bridge pose/info topic"
  ros2 topic list | grep -E '^/world/.*/pose/info$' || true

  echo "[HEALTH] 7) camera topics"
  ros2 topic list | grep -E '^/camera' || true

  echo "[HEALTH] 8) critical node list (subset)"
  ros2 node list | grep -E '/(ros_gz_bridge_main|world_tf_publisher|controller_bootstrap|controller_manager|system_state_manager|panel_superpro|panel_tf_helper)' || true
} 2>&1 | tee "$LOG_FILE"

echo "[HEALTH] log=$LOG_FILE"
