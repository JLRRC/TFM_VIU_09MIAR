#!/usr/bin/env bash

set -euo pipefail

ROOT="/home/laboratorio/TFM"
PYTHON_BIN="${PYTHON_BIN:-/home/laboratorio/.venv-tfm/bin/python}"
TMP_DIR="$ROOT/auditoria/tmp/recreated_tmp"
LOSS_TMP="$TMP_DIR/loss_shared_ylim"
BOUNDARY_TMP="$TMP_DIR/boundary_failure"

mkdir -p "$LOSS_TMP" "$BOUNDARY_TMP" "$ROOT/reports/figures/cap5" "$ROOT/reports/evidence/chapter5"

"$PYTHON_BIN" "$ROOT/agarre_inteligente/scripts/generar_ilustraciones_tfm_5_1.py" \
  --out-dir "$LOSS_TMP" \
  --log-path "$LOSS_TMP/LOG_regeneracion_curvas_loss_shared_ylim.md" \
  --skip-best-epoch-comparisons

cp -a "$LOSS_TMP/ilustracion_5_2_curvas_loss_exp1_simple_rgb.png" \
  "$ROOT/reports/figures/cap5/Ilustracion_5-1_curvas_de_perdida_train_loss_y_val_loss_para_exp1.png"
cp -a "$LOSS_TMP/ilustracion_5_3_curvas_loss_exp2_simple_rgbd.png" \
  "$ROOT/reports/figures/cap5/Ilustracion_5-2_curvas_de_perdida_train_loss_y_val_loss_para_exp2.png"
cp -a "$LOSS_TMP/ilustracion_5_4_curvas_loss_exp3_resnet18_rgb_augment.png" \
  "$ROOT/reports/figures/cap5/Ilustracion_5-3_curvas_de_perdida_train_loss_y_val_loss_para_exp3.png"
cp -a "$LOSS_TMP/ilustracion_5_5_curvas_loss_exp4_resnet18_rgbd.png" \
  "$ROOT/reports/figures/cap5/Ilustracion_5-4_curvas_de_perdida_train_loss_y_val_loss_para_exp4.png"

"$PYTHON_BIN" "$ROOT/agarre_inteligente/scripts/generate_boundary_failure_figure.py" \
  --output-dir "$BOUNDARY_TMP" \
  --base-name "fig_5_15_false_negative_plausible"

cp -a "$BOUNDARY_TMP/fig_5_15_false_negative_plausible.png" \
  "$ROOT/reports/figures/cap5/Ilustracion_5-15_caso_limite_de_falso_negativo_visualmente_plausible.png"
cp -a "$BOUNDARY_TMP/fig_5_15_false_negative_plausible.pdf" \
  "$ROOT/reports/figures/cap5/Ilustracion_5-15_caso_limite_de_falso_negativo_visualmente_plausible.pdf"
cp -a "$BOUNDARY_TMP/fig_5_15_false_negative_plausible.png" \
  "$ROOT/reports/evidence/chapter5/fig_5_15_false_negative_plausible.png"
cp -a "$BOUNDARY_TMP/fig_5_15_false_negative_plausible.pdf" \
  "$ROOT/reports/evidence/chapter5/fig_5_15_false_negative_plausible.pdf"
cp -a "$BOUNDARY_TMP/fig_5_15_false_negative_plausible_candidates.csv" \
  "$ROOT/reports/evidence/chapter5/fig_5_15_false_negative_plausible_candidates.csv"

"$PYTHON_BIN" "$ROOT/agarre_inteligente/scripts/regenerate_chapter5_post_retrain.py"

echo "Artefactos regenerados en reports/figures/cap5 y reports/evidence/chapter5."
