#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/show_latest_startup_evidence.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="/home/laboratorio/TFM/agarre_ros2_ws"
BASE_DIR="$WS_DIR/reports/evidence_startup_ready"
LATEST_LINK="$BASE_DIR/latest"

if [[ ! -e "$LATEST_LINK" ]]; then
  echo "[LATEST] no hay evidencia en $BASE_DIR"
  exit 2
fi

LATEST_DIR="$(readlink -f "$LATEST_LINK")"
RESULT_ENV="$LATEST_DIR/result.env"
SUMMARY_LOG="$LATEST_DIR/summary.log"

if [[ ! -f "$RESULT_ENV" ]]; then
  echo "[LATEST] falta result.env en $LATEST_DIR"
  exit 3
fi

# shellcheck disable=SC1090
source "$RESULT_ENV"

echo "[LATEST] dir=$LATEST_DIR"
echo "[LATEST] stamp=${stamp:-unknown}"
echo "[LATEST] cycles=${cycles:-unknown}"
echo "[LATEST] result=${result:-unknown}"
echo "[LATEST] summary=${summary_log:-$SUMMARY_LOG}"

echo "--- summary tail ---"
tail -n 20 "$SUMMARY_LOG" || true
