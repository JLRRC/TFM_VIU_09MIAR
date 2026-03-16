#!/bin/bash
# ============================================================================
# regenerate_tfm_block.sh
# ============================================================================
# Ruta y propósito: /home/laboratorio/TFM/agarre_inteligente/scripts/regenerate_tfm_block.sh
# Regenera el bloque experimental del TFM: entrenamiento -> resumen -> tablas -> figuras.
# ============================================================================

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

ALLOW_SYNTHETIC="${ALLOW_SYNTHETIC:-0}"

configs=(
  "config/exp1_simple_rgb.yaml"
  "config/exp2_simple_rgbd.yaml"
  "config/exp3_resnet18_rgb_augment.yaml"
  "config/exp4_resnet18_rgbd.yaml"
)

echo "[1/6] Ejecutando entrenamiento por experimento..."
for cfg in "${configs[@]}"; do
  if [[ "$ALLOW_SYNTHETIC" == "1" ]]; then
    python3 scripts/run_experiment.py --config "$cfg" --allow-synthetic
  else
    python3 scripts/run_experiment.py --config "$cfg"
  fi

done

echo "[2/6] Seleccionando best_epoch por experimento..."
for exp_dir in experiments/EXP*; do
  [[ -d "$exp_dir" ]] || continue
  python3 scripts/select_best_epoch.py --experiment-dir "$exp_dir"
done

echo "[3/6] Generando resumen global..."
python3 scripts/summarize_results.py --experiments-root experiments --output reports/tables/summary_results.csv

echo "[4/6] Generando figuras..."
python3 scripts/generate_figures.py --experiments-root experiments --summary reports/tables/summary_results.csv --out-dir reports/figures

echo "[5/6] Generando tablas..."
python3 scripts/generate_tables.py --summary reports/tables/summary_results.csv --out-dir reports/tables

echo "[6/6] Validación de artefactos..."
python3 scripts/validate_artifacts.py --strict

echo "[OK] Bloque TFM regenerado"
