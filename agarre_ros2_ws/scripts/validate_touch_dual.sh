#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/validate_touch_dual.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="/home/laboratorio/TFM/agarre_ros2_ws"
STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
OUT_DIR="$WS_DIR/reports/touch_tuner/${STAMP}_dual_validation"
mkdir -p "$OUT_DIR"

cd "$WS_DIR"
export COLCON_TRACE="${COLCON_TRACE:-0}"
set +u
source install/setup.bash
set -u

run_target() {
  local target="$1"
  local subdir="$OUT_DIR/${target}"
  local log_file="$subdir/run.log"
  mkdir -p "$subdir"

  echo "[RUN] target=$target out=$subdir"

  set +e
  python3 "$WS_DIR/tools/autonomous_touch_probe.py" \
    --base_frame base_link \
    --ee_frame rg2_tcp \
    --target_name "$target" \
    --tol_xy 0.02 \
    --tol_z 0.02 \
    --max_iters 50 \
    --z_start_offset 0.12 \
    --z_step 0.01 \
    --cartesian_descent true \
    --constraint_orientation true \
    --allowed_collision_table false \
    --reports_dir "$subdir" 2>&1 | tee "$log_file" >&2
  local rc=${PIPESTATUS[0]}
  set -e

  local verdict="FAIL"
  if [[ $rc -eq 0 ]]; then
    verdict="PASS"
  fi

  local last_line
  last_line="$(grep -E "\[SUCCESS\]|\[FAIL\]" "$log_file" | tail -n 1 || true)"
  {
    echo "target=$target"
    echo "rc=$rc"
    echo "verdict=$verdict"
    echo "summary=${last_line:-N/A}"
    echo "log=$log_file"
    echo "csv=$subdir/tuning.csv"
  } > "$subdir/result.env"

  printf "%s" "$verdict"
  return $rc
}

echo "[RUN] dual touch validation"
echo "[RUN] out_dir=$OUT_DIR"

left_verdict="$(run_target FRONT_LEFT || true)"
right_verdict="$(run_target FRONT_RIGHT || true)"

SUMMARY_FILE="$OUT_DIR/summary.txt"
{
  echo "DUAL_TOUCH_VALIDATION"
  echo "timestamp=$STAMP"
  echo "front_left=$left_verdict"
  echo "front_right=$right_verdict"
  echo "front_left_result=$OUT_DIR/FRONT_LEFT/result.env"
  echo "front_right_result=$OUT_DIR/FRONT_RIGHT/result.env"
} > "$SUMMARY_FILE"

echo "[RUN] summary=$SUMMARY_FILE"

if [[ "$left_verdict" == "PASS" && "$right_verdict" == "PASS" ]]; then
  echo "[SUCCESS] dual validation PASS"
  exit 0
fi

echo "[FAIL] dual validation FAILED"
exit 2
