#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/run_touch_tuner_front_right.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="/home/laboratorio/TFM/agarre_ros2_ws"
STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
OUT_DIR="$WS_DIR/reports/touch_tuner/${STAMP}_front_right"
mkdir -p "$OUT_DIR"
LOG_FILE="$OUT_DIR/run.log"

cd "$WS_DIR"
export COLCON_TRACE="${COLCON_TRACE:-0}"
set +u
source install/setup.bash
set -u

{
  echo "[RUN] touch tuner FRONT_RIGHT"
  echo "[RUN] timestamp=$STAMP"
  echo "[RUN] out_dir=$OUT_DIR"

  python3 "$WS_DIR/tools/autonomous_touch_probe.py" \
    --base_frame base_link \
    --ee_frame rg2_tcp \
    --target_name FRONT_RIGHT \
    --tol_xy 0.02 \
    --tol_z 0.02 \
    --max_iters 50 \
    --z_start_offset 0.12 \
    --z_step 0.01 \
    --cartesian_descent true \
    --constraint_orientation true \
    --allowed_collision_table false \
    --reports_dir "$OUT_DIR"
} 2>&1 | tee "$LOG_FILE"

echo "[RUN] log=$LOG_FILE"
echo "[RUN] csv=$OUT_DIR/tuning.csv"
