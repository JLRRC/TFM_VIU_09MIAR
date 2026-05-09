#!/usr/bin/env bash
# run_directo_batch_validation.sh — Ejecuta 5–10 corridas DIRECTO y resume artefactos.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

RUNS="${RUNS:-5}"
BATCH_ROOT="${BATCH_ROOT:-$WS_DIR/../auditoria/directo_batch_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "$BATCH_ROOT"

echo "[BATCH] Ejecutando ${RUNS} corridas en $BATCH_ROOT"

for idx in $(seq 1 "$RUNS"); do
  RUN_OUT="$BATCH_ROOT/directo_validation_$(date +%Y%m%d_%H%M%S)_run$(printf '%02d' "$idx")"
  echo "[BATCH] Corrida $idx/$RUNS -> $RUN_OUT"
  OUT_DIR="$RUN_OUT" "$SCRIPT_DIR/run_directo_validation.sh" || true
done

python3 "$SCRIPT_DIR/summarize_directo_batch.py" "$BATCH_ROOT" | tee "$BATCH_ROOT/batch_summary.stdout.json"
echo "[BATCH] Resumen disponible en $BATCH_ROOT/batch_summary.json"
python3 - "$BATCH_ROOT/batch_summary.json" <<'PY'
import json
import sys

summary_path = sys.argv[1]
with open(summary_path, "r", encoding="utf-8") as handle:
    payload = json.load(handle)
failed = int(payload.get("failed", 0))
runs = int(payload.get("runs", 0))
passed = int(payload.get("passed", 0))
print(f"[BATCH] Resultado agregado: passed={passed} failed={failed} runs={runs}")
raise SystemExit(0 if failed == 0 else 1)
PY
