#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/validate_startup_repro.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="/home/laboratorio/TFM/agarre_ros2_ws"
CYCLES="${1:-3}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$WS_DIR/reports/repro_startup/$STAMP"
mkdir -p "$OUT_DIR"

cd "$WS_DIR"

echo "[REPRO] ws=$WS_DIR" | tee -a "$OUT_DIR/summary.log"
echo "[REPRO] cycles=$CYCLES" | tee -a "$OUT_DIR/summary.log"
echo "[REPRO] out=$OUT_DIR" | tee -a "$OUT_DIR/summary.log"

fails=0

for i in $(seq 1 "$CYCLES"); do
  echo "" | tee -a "$OUT_DIR/summary.log"
  echo "=== CYCLE $i START ===" | tee -a "$OUT_DIR/summary.log"

  ./scripts/stop_panel_v2.sh > "$OUT_DIR/cycle${i}_stop.log" 2>&1 || true
  ./scripts/start_panel_v2.sh --bg > "$OUT_DIR/cycle${i}_start.log" 2>&1

  sleep 10

  ./check_pipeline.sh > "$OUT_DIR/cycle${i}_pipeline.log" 2>&1 || true

  if grep -q 'RESULTADO: PASS' "$OUT_DIR/cycle${i}_pipeline.log"; then
    echo "[REPRO] cycle=$i PASS" | tee -a "$OUT_DIR/summary.log"
  else
    echo "[REPRO] cycle=$i FAIL" | tee -a "$OUT_DIR/summary.log"
    fails=$((fails+1))
  fi

  tail -n 8 "$OUT_DIR/cycle${i}_pipeline.log" >> "$OUT_DIR/summary.log"
done

echo "" | tee -a "$OUT_DIR/summary.log"
if [[ "$fails" -eq 0 ]]; then
  echo "[REPRO] RESULT=PASS fails=0/$CYCLES" | tee -a "$OUT_DIR/summary.log"
  exit 0
fi

echo "[REPRO] RESULT=FAIL fails=$fails/$CYCLES" | tee -a "$OUT_DIR/summary.log"
exit 2
