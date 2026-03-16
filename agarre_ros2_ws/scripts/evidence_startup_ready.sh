#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/evidence_startup_ready.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="/home/laboratorio/TFM/agarre_ros2_ws"
CYCLES="${1:-3}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$WS_DIR/reports/evidence_startup_ready/$STAMP"
mkdir -p "$OUT_DIR"

cd "$WS_DIR"

log(){ echo "[EVIDENCE] $*" | tee -a "$OUT_DIR/summary.log"; }

log "ws=$WS_DIR"
log "cycles=$CYCLES"
log "out=$OUT_DIR"

log "1) pipeline actual"
./check_pipeline.sh > "$OUT_DIR/01_pipeline.log" 2>&1 || true
if grep -q 'RESULTADO: PASS' "$OUT_DIR/01_pipeline.log"; then
  log "pipeline=PASS"
else
  log "pipeline=FAIL"
fi

log "2) reproducibilidad startup"
./scripts/validate_startup_repro.sh "$CYCLES" > "$OUT_DIR/02_repro.log" 2>&1 || true
if grep -q '\[REPRO\] RESULT=PASS' "$OUT_DIR/02_repro.log"; then
  log "repro=PASS"
else
  log "repro=FAIL"
fi

log "3) diagnósticos TF + health"
./scripts/diag_tf_tcp.sh > "$OUT_DIR/03_diag_tf.log" 2>&1 || true
./scripts/diag_startup_health.sh > "$OUT_DIR/04_diag_health.log" 2>&1 || true

log "3b) validación flujo panel"
set +e
./scripts/validate_panel_flow.sh > "$OUT_DIR/04b_validate_panel_flow.log" 2>&1
panel_flow_rc=$?
set -e
echo "panel_flow_rc=$panel_flow_rc" > "$OUT_DIR/04b_validate_panel_flow.rc"

log "4) extracto de ros2_launch.log"
grep -nE 'STATE READY|STATE ERROR_FATAL|MoveIt timeout|\[AUTO\]\[WAIT\].*FAIL|bootstrap recuperado|autostart diferido|NOT_READY|PANEL_FATAL_STOPS_ALL' \
  log/ros2_launch.log > "$OUT_DIR/05_launch_extract.log" || true

# Eliminar línea informativa conocida (no es fallo real):
grep -v 'PANEL_FATAL_STOPS_ALL=0 .*ERROR_FATAL no hace stop_all' "$OUT_DIR/05_launch_extract.log" \
  > "$OUT_DIR/05_launch_extract.filtered.log" || true

result="PASS"
if ! grep -q 'RESULTADO: PASS' "$OUT_DIR/01_pipeline.log"; then
  result="FAIL"
fi
if ! grep -q '\[REPRO\] RESULT=PASS' "$OUT_DIR/02_repro.log"; then
  result="FAIL"
fi
if [[ "${panel_flow_rc:-1}" -ne 0 ]]; then
  result="FAIL"
fi
if grep -qE 'STATE ERROR_FATAL|MoveIt timeout|\[AUTO\]\[WAIT\].*FAIL' "$OUT_DIR/05_launch_extract.filtered.log"; then
  result="FAIL"
fi

RESULT_ENV="$OUT_DIR/result.env"
{
  echo "stamp=$STAMP"
  echo "cycles=$CYCLES"
  echo "out_dir=$OUT_DIR"
  echo "pipeline_log=$OUT_DIR/01_pipeline.log"
  echo "repro_log=$OUT_DIR/02_repro.log"
  echo "diag_tf_log=$OUT_DIR/03_diag_tf.log"
  echo "diag_health_log=$OUT_DIR/04_diag_health.log"
  echo "panel_flow_log=$OUT_DIR/04b_validate_panel_flow.log"
  echo "panel_flow_rc_file=$OUT_DIR/04b_validate_panel_flow.rc"
  echo "launch_extract=$OUT_DIR/05_launch_extract.log"
  echo "launch_extract_filtered=$OUT_DIR/05_launch_extract.filtered.log"
  echo "summary_log=$OUT_DIR/summary.log"
  echo "result=$result"
} > "$RESULT_ENV"

ln -sfn "$OUT_DIR" "$WS_DIR/reports/evidence_startup_ready/latest"

log "RESULT=$result"
log "summary_file=$OUT_DIR/summary.log"
log "result_env=$RESULT_ENV"
log "latest_link=$WS_DIR/reports/evidence_startup_ready/latest"

if [[ "$result" == "PASS" ]]; then
  exit 0
fi
exit 2
