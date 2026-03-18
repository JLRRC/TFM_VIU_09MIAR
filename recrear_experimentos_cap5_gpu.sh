#!/usr/bin/env bash

set -euo pipefail

ROOT="/home/laboratorio/TFM"
PROJECT_ROOT="$ROOT/agarre_inteligente"
PYTHON_BIN="${PYTHON_BIN:-/home/laboratorio/.venv-tfm/bin/python}"
STAMP="${STAMP:-$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$ROOT/report/logs/training"
LOG_PATH="$LOG_DIR/retrain_cap5_gpu_${STAMP}.log"

CONFIGS=(
  "config/exp1_simple_rgb.yaml"
  "config/exp2_simple_rgbd.yaml"
  "config/exp3_resnet18_rgb_augment.yaml"
  "config/exp4_resnet18_rgbd.yaml"
)

mkdir -p "$LOG_DIR"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "[ERROR] Python CUDA no encontrado en $PYTHON_BIN" >&2
  exit 1
fi

if [[ $# -gt 0 ]]; then
  CONFIGS=("$@")
fi

{
  echo "[INFO] Inicio de relanzamiento capítulo 5: $(date --iso-8601=seconds)"
  echo "[INFO] Proyecto: $PROJECT_ROOT"
  echo "[INFO] Python: $PYTHON_BIN"
  echo "[INFO] Configs: ${CONFIGS[*]}"
  "$PYTHON_BIN" -c "import torch; print(f'[INFO] torch={torch.__version__} cuda_available={torch.cuda.is_available()} device_count={torch.cuda.device_count()}')"
} | tee "$LOG_PATH"

cd "$PROJECT_ROOT"

for config_path in "${CONFIGS[@]}"; do
  {
    echo
    echo "[INFO] Ejecutando $(date --iso-8601=seconds) -> $config_path"
    "$PYTHON_BIN" scripts/run_experiment.py --config "$config_path"
    echo "[INFO] Finalizado $(date --iso-8601=seconds) -> $config_path"
  } 2>&1 | tee -a "$LOG_PATH"
done

echo "[INFO] Relanzamiento completado: $(date --iso-8601=seconds)" | tee -a "$LOG_PATH"
echo "[INFO] Log: $LOG_PATH" | tee -a "$LOG_PATH"
