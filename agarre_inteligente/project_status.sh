#!/bin/bash
# ============================================================================
# project_status.sh
# ============================================================================
# Ruta y propósito: /home/laboratorio/TFM/agarre_inteligente/project_status.sh
# Muestra estado global del proyecto y cobertura de artefactos TFM.
# ============================================================================

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

echo "== Estado general de agarre_inteligente =="
echo "Ruta: $ROOT_DIR"
echo "Fecha: $(date '+%Y-%m-%d %H:%M:%S')"
echo

echo "[1] Estructura clave"
for d in src scripts config data experiments reports docs; do
  [[ -d "$d" ]] && echo "  OK  $d" || echo "  MISS $d"
done

echo ""
echo "[2] Experimentos"
for e in EXP1_SIMPLE_RGB EXP2_SIMPLE_RGBD EXP3_RESNET18_RGB_AUGMENT EXP4_RESNET18_RGBD; do
  if [[ -d "experiments/$e" ]]; then
    seeds=$(find "experiments/$e" -maxdepth 1 -type d -name 'seed_*' | wc -l)
    echo "  $e -> seeds=$seeds"
  else
    echo "  $e -> no ejecutado"
  fi
done

echo ""
echo "[3] Artefactos de reportes"
for f in reports/tables/summary_results.csv reports/tables/table_metrics_final.csv reports/figures/val_success_by_epoch.png reports/bench/latency_results.csv; do
  [[ -f "$f" ]] && echo "  OK  $f" || echo "  MISS $f"
done

echo ""
echo "[4] Registro maestro"
if [[ -f docs/REGISTRO_RECONSTRUCCION.md ]]; then
  lines=$(wc -l < docs/REGISTRO_RECONSTRUCCION.md)
  echo "  OK docs/REGISTRO_RECONSTRUCCION.md ($lines lineas)"
else
  echo "  MISS docs/REGISTRO_RECONSTRUCCION.md"
fi
