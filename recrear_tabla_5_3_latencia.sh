#!/usr/bin/env bash

set -euo pipefail

ROOT="/home/laboratorio/TFM"
PROJECT_ROOT="$ROOT/agarre_inteligente"
PYTHON_BIN="${PYTHON_BIN:-/home/laboratorio/.venv-tfm/bin/python}"
DEVICES="${DEVICES:-cpu,cuda}"
WARMUP="${WARMUP:-5}"
REPEATS="${REPEATS:-20}"
STAMP="${STAMP:-$(date +%Y%m%d_%H%M%S)}"

TMP_CSV="$ROOT/report/bench/latency_results_${STAMP}.csv"
OFFICIAL_CSV="$ROOT/report/tables/cap5/Tabla_5-3_medicion_de_latencia_de_inferencia_por_experimento_y_dispositivo.csv"
TRACE_TSV="$ROOT/report/logs/reproducibility/tabla_5_3_latencia_${STAMP}.tsv"

CONFIGS=(
  "EXP1_SIMPLE_RGB config/exp1_simple_rgb.yaml"
  "EXP2_SIMPLE_RGBD config/exp2_simple_rgbd.yaml"
  "EXP3_RESNET18_RGB_AUGMENT config/exp3_resnet18_rgb_augment.yaml"
  "EXP4_RESNET18_RGBD config/exp4_resnet18_rgbd.yaml"
)

mkdir -p "$ROOT/report/bench" "$ROOT/report/tables/cap5" "$ROOT/report/logs/reproducibility"
rm -f "$TMP_CSV"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "[ERROR] Python no encontrado en $PYTHON_BIN" >&2
  exit 1
fi

IFS=',' read -r -a DEVICE_LIST <<< "$DEVICES"

{
  echo -e "timestamp\tevent\tdetail"
  echo -e "$(date --iso-8601=seconds)\tstart\tdevices=$DEVICES warmup=$WARMUP repeats=$REPEATS"
} > "$TRACE_TSV"

for item in "${CONFIGS[@]}"; do
  exp_name="${item%% *}"
  config_path="${item#* }"
  summary_csv="$PROJECT_ROOT/experiments/$exp_name/best_epoch_summary.csv"

  if [[ ! -f "$summary_csv" ]]; then
    echo "[ERROR] Falta $summary_csv" >&2
    exit 1
  fi

  seed="$("$PYTHON_BIN" - <<PY
import pandas as pd
df = pd.read_csv("$summary_csv").sort_values("val_success", ascending=False)
print(int(df.iloc[0]["seed"]))
PY
)"
  checkpoint_path="$PROJECT_ROOT/experiments/$exp_name/seed_${seed}/checkpoints/best.pth"

  if [[ ! -f "$checkpoint_path" ]]; then
    echo "[ERROR] Falta $checkpoint_path" >&2
    exit 1
  fi

  echo -e "$(date --iso-8601=seconds)\tcheckpoint\t$exp_name seed=$seed path=$checkpoint_path" >> "$TRACE_TSV"

  for device in "${DEVICE_LIST[@]}"; do
    echo "[INFO] Benchmark $exp_name seed=$seed device=$device"
    "$PYTHON_BIN" "$PROJECT_ROOT/scripts/benchmark_latency.py" \
      --config "$PROJECT_ROOT/$config_path" \
      --checkpoint "$checkpoint_path" \
      --device "$device" \
      --warmup "$WARMUP" \
      --repeats "$REPEATS" \
      --output "$TMP_CSV"
    echo -e "$(date --iso-8601=seconds)\tbenchmark\t$exp_name seed=$seed device=$device" >> "$TRACE_TSV"
  done
done

"$PYTHON_BIN" - <<PY
import pandas as pd

tmp_csv = "$TMP_CSV"
official_csv = "$OFFICIAL_CSV"
trace_tsv = "$TRACE_TSV"

df = pd.read_csv(tmp_csv)
order_exp = [
    "EXP1_SIMPLE_RGB",
    "EXP2_SIMPLE_RGBD",
    "EXP3_RESNET18_RGB_AUGMENT",
    "EXP4_RESNET18_RGBD",
]
order_device = {"cpu": 0, "cuda": 1}

df["exp_order"] = df["experiment"].map({k: i for i, k in enumerate(order_exp)})
df["device_order"] = df["device"].map(order_device)
df = df.sort_values(["exp_order", "device_order"]).drop(columns=["exp_order", "device_order"])

df.to_csv(official_csv, index=False)

with open(trace_tsv, "a", encoding="utf-8") as handle:
    handle.write(f"{pd.Timestamp.now().isoformat()}\tgenerated\t{official_csv}\n")
    handle.write(f"{pd.Timestamp.now().isoformat()}\trows\t{len(df)}\n")
PY

echo "[OK] Tabla oficial regenerada en $OFFICIAL_CSV"
echo "[OK] Traza: $TRACE_TSV"
