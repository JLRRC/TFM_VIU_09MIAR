#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/monitor_attach_publishers.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-}"
export COLCON_TRACE="${COLCON_TRACE:-}"
export COLCON_PREFIX_PATH="${COLCON_PREFIX_PATH:-}"
export COLCON_PYTHON_EXECUTABLE="${COLCON_PYTHON_EXECUTABLE:-}"

ROOT="/home/laboratorio/TFM"
source /opt/ros/jazzy/setup.bash
source "${ROOT}/agarre_ros2_ws/install/setup.bash"

TOPICS=(
  /drop_anchor/box_red/attach
  /drop_anchor/box_blue/attach
  /drop_anchor/box_green/attach
  /drop_anchor/box_lightblue/attach
  /drop_anchor/box_yellow/attach
  /drop_anchor/cross_cyan/attach
  /drop_anchor/cyl_gray/attach
  /drop_anchor/cyl_green/attach
  /drop_anchor/cyl_orange/attach
  /drop_anchor/cyl_purple/attach
  /drop_anchor/box_red/detach
  /drop_anchor/box_blue/detach
  /drop_anchor/box_green/detach
  /drop_anchor/box_lightblue/detach
  /drop_anchor/box_yellow/detach
  /drop_anchor/cross_cyan/detach
  /drop_anchor/cyl_gray/detach
  /drop_anchor/cyl_green/detach
  /drop_anchor/cyl_orange/detach
  /drop_anchor/cyl_purple/detach
)

echo "Monitoring drop_anchor attach/detach publishers (CTRL+C to stop)"
while true; do
  stamp="$(date +%Y-%m-%dT%H:%M:%S)"
  for t in "${TOPICS[@]}"; do
    info="$(ros2 topic info -v "${t}" 2>/dev/null | awk '/Publisher count/ {print $3}' || true)"
    if [[ -n "${info}" && "${info}" != "0" ]]; then
      echo "[${stamp}] ${t} publishers=${info}"
      ros2 topic info -v "${t}" | sed 's/^/  /'
    fi
  done
  sleep 0.5
done
