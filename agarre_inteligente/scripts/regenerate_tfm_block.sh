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
official_experiments=(
  "experiments/EXP1_SIMPLE_RGB"
  "experiments/EXP2_SIMPLE_RGBD"
  "experiments/EXP3_RESNET18_RGB_AUGMENT"
  "experiments/EXP4_RESNET18_RGBD"
)
for exp_dir in "${official_experiments[@]}"; do
  [[ -d "$exp_dir" ]] || continue
  python3 scripts/select_best_epoch.py --experiment-dir "$exp_dir"
done

echo "[3/6] Generando resumen global..."
python3 scripts/summarize_results.py --experiments-root experiments --output report/tables/summary_results.csv

echo "[4/6] Generando figuras..."
python3 scripts/generate_figures.py --experiments-root experiments --summary report/tables/summary_results.csv --out-dir report/figures

echo "[5/7] Validando scope oficial..."
python3 scripts/validate_official_scope.py --summary report/tables/summary_results.csv --results-by-seed report/tables/results_by_seed.csv

echo "[6/7] Generando tablas..."
python3 scripts/generate_tables.py --summary report/tables/summary_results.csv --out-dir report/tables

echo "[7/7] Validación de artefactos..."
python3 scripts/validate_artifacts.py --strict

echo "[OK] Bloque TFM regenerado"
