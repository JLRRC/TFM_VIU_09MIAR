#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/prune_startup_evidence.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

WS_DIR="/home/laboratorio/TFM/agarre_ros2_ws"
BASE_DIR="$WS_DIR/reports/evidence_startup_ready"
KEEP="${1:-10}"

if [[ ! -d "$BASE_DIR" ]]; then
  echo "[PRUNE] no existe $BASE_DIR"
  exit 0
fi

mapfile -t dirs < <(find "$BASE_DIR" -maxdepth 1 -mindepth 1 -type d -printf '%f\n' | sort -r)
count="${#dirs[@]}"

echo "[PRUNE] base=$BASE_DIR"
echo "[PRUNE] total=$count keep=$KEEP"

if [[ "$count" -le "$KEEP" ]]; then
  echo "[PRUNE] nada que eliminar"
  exit 0
fi

for idx in "${!dirs[@]}"; do
  if [[ "$idx" -lt "$KEEP" ]]; then
    continue
  fi
  old_dir="$BASE_DIR/${dirs[$idx]}"
  if [[ -L "$BASE_DIR/latest" ]] && [[ "$(readlink -f "$BASE_DIR/latest")" == "$(readlink -f "$old_dir")" ]]; then
    echo "[PRUNE] skip latest: $old_dir"
    continue
  fi
  rm -rf "$old_dir"
  echo "[PRUNE] removed $old_dir"
done

echo "[PRUNE] done"
