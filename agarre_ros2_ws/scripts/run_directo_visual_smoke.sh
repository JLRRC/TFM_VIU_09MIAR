#!/usr/bin/env bash
# run_directo_visual_smoke.sh — Corrida única con artefactos visuales mínimos.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
OUT_DIR="${OUT_DIR:-$WS_DIR/../auditoria/directo_visual_smoke_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "$OUT_DIR"

OUT_DIR="$OUT_DIR" "$SCRIPT_DIR/run_directo_validation.sh"

echo "[SMOKE] Revisa:"
echo "[SMOKE]   $OUT_DIR/visual_smoke"
echo "[SMOKE]   $OUT_DIR/camera_frames"
echo "[SMOKE]   $OUT_DIR/helper.log"
