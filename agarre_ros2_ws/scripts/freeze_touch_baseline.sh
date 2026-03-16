#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/freeze_touch_baseline.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="/home/laboratorio/TFM/agarre_ros2_ws"
BASE_DIR="$WS_DIR/reports/touch_tuner/baselines"
mkdir -p "$BASE_DIR"

cd "$WS_DIR"

if [[ $# -ge 1 ]]; then
  VALIDATION_DIR="$1"
else
  VALIDATION_DIR="$(ls -1dt reports/touch_tuner/*_dual_validation | head -n 1 || true)"
fi

if [[ -z "${VALIDATION_DIR:-}" || ! -d "$VALIDATION_DIR" ]]; then
  echo "[ERROR] no dual validation directory found"
  exit 2
fi

SUMMARY_FILE="$VALIDATION_DIR/summary.txt"
if [[ ! -f "$SUMMARY_FILE" ]]; then
  echo "[ERROR] summary not found: $SUMMARY_FILE"
  exit 3
fi

LEFT_ENV="$VALIDATION_DIR/FRONT_LEFT/result.env"
RIGHT_ENV="$VALIDATION_DIR/FRONT_RIGHT/result.env"
if [[ ! -f "$LEFT_ENV" || ! -f "$RIGHT_ENV" ]]; then
  echo "[ERROR] missing result.env files in $VALIDATION_DIR"
  exit 4
fi

stamp="$(date +%Y-%m-%d_%H-%M-%S)"
OUT_FILE="$BASE_DIR/touch_baseline_${stamp}.env"

{
  echo "created_at=$stamp"
  echo "validation_dir=$VALIDATION_DIR"
  echo "summary_file=$SUMMARY_FILE"
  echo "left_result=$LEFT_ENV"
  echo "right_result=$RIGHT_ENV"
  echo ""
  echo "# summary.txt"
  cat "$SUMMARY_FILE"
  echo ""
  echo "# FRONT_LEFT/result.env"
  cat "$LEFT_ENV"
  echo ""
  echo "# FRONT_RIGHT/result.env"
  cat "$RIGHT_ENV"
} > "$OUT_FILE"

ln -sfn "$OUT_FILE" "$BASE_DIR/latest_touch_baseline.env"

echo "[SUCCESS] baseline frozen"
echo "[BASELINE] file=$OUT_FILE"
echo "[BASELINE] latest=$BASE_DIR/latest_touch_baseline.env"
