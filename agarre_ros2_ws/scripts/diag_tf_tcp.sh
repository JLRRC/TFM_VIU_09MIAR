#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/diag_tf_tcp.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="/home/laboratorio/TFM/agarre_ros2_ws"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$WS_DIR/reports/diag_tf/$STAMP"
mkdir -p "$OUT_DIR"
LOG_FILE="$OUT_DIR/diag.log"

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

{
  echo "[DIAG] stamp=$STAMP"
  echo "[DIAG] cwd=$(pwd)"
  echo "[DIAG] 1) tf2_echo base_link rg2_tcp (5s)"
  tf_echo_clean base_link rg2_tcp
  echo "[DIAG] 2) tf2_echo world base_link (5s)"
  tf_echo_clean world base_link
  echo "[DIAG] 3) /joint_states one sample"
  timeout 5s ros2 topic echo --once /joint_states || true
} 2>&1 | tee "$LOG_FILE"

echo "[DIAG] log=$LOG_FILE"
