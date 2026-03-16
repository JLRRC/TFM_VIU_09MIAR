#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/start_panel_v2_venv.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="/home/laboratorio/TFM/agarre_ros2_ws"
VISION_DIR="/home/laboratorio/TFM/agarre_inteligente"

VENV_DIR="${PANEL_VENV_DIR:-}"
if [[ -z "$VENV_DIR" ]]; then
  for candidate in \
    "/home/laboratorio/TFM/agarre_inteligente/.venv-tfm" \
    "/home/laboratorio/TFM/agarre_inteligente/venv" \
    "/home/laboratorio/TFM/agarre_inteligente/.venv"; do
    if [[ -f "$candidate/bin/activate" ]]; then
      VENV_DIR="$candidate"
      break
    fi
  done
fi

if [ ! -d "$VENV_DIR" ]; then
  echo "[START_PANEL_V2_VENV] ERROR: venv no encontrado: $VENV_DIR" >&2
  exit 1
fi

source "$VENV_DIR/bin/activate"
set +u
source "$WS_DIR/install/setup.bash"
set -u
export VISION_DIR="$VISION_DIR"

exec "$WS_DIR/scripts/start_panel_v2.sh"
