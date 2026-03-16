#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPORTS_DIR="$ROOT_DIR/reports"
AI_DIR="$ROOT_DIR/agarre_inteligente"
ROS2_DIR="$ROOT_DIR/agarre_ros2_ws"
AI_VENV="$AI_DIR/venv"

log() {
  echo "[ACTUALIZAR_REPORTS] $*"
}

sync_tree() {
  local src="$1"
  local dst="$2"
  if [[ ! -e "$src" ]]; then
    return 0
  fi
  mkdir -p "$dst"
  if command -v rsync >/dev/null 2>&1; then
    rsync -a --delete "$src"/ "$dst"/
  else
    rm -rf "$dst"
    mkdir -p "$(dirname "$dst")"
    cp -a "$src" "$dst"
  fi
}

copy_if_exists() {
  local src="$1"
  local dst="$2"
  if [[ -f "$src" ]]; then
    mkdir -p "$(dirname "$dst")"
    cp -f "$src" "$dst"
  fi
}

ensure_structure() {
  mkdir -p \
    "$REPORTS_DIR/archive" \
    "$REPORTS_DIR/bench" \
    "$REPORTS_DIR/capitulos" \
    "$REPORTS_DIR/cornell_audit" \
    "$REPORTS_DIR/docs/workspace" \
    "$REPORTS_DIR/figures" \
    "$REPORTS_DIR/panel_audit" \
    "$REPORTS_DIR/panel_logs" \
    "$REPORTS_DIR/tables" \
    "$REPORTS_DIR/tfm_figuras_cap5_1" \
    "$REPORTS_DIR/tfm_ros_gazebo_results" \
    "$REPORTS_DIR/tfm_visual_revision" \
    "$REPORTS_DIR/validation"
}

regenerate_ai_reports() {
  if [[ ! -f "$AI_VENV/bin/activate" ]]; then
    log "Aviso: no existe $AI_VENV. Se omite regeneracion de reportes de agarre_inteligente."
    return 0
  fi
  log "Regenerando tablas y figuras del bloque experimental..."
  # shellcheck disable=SC1090
  source "$AI_VENV/bin/activate"
  pushd "$AI_DIR" >/dev/null
  python3 scripts/summarize_results.py \
    --experiments-root experiments \
    --output ../reports/tables/summary_results.csv
  python3 scripts/generate_figures.py \
    --experiments-root experiments \
    --summary ../reports/tables/summary_results.csv \
    --out-dir ../reports/figures
  python3 scripts/generate_tables.py \
    --summary ../reports/tables/summary_results.csv \
    --latency ../reports/bench/latency_results.csv \
    --out-dir ../reports/tables
  if [[ -f scripts/generar_ilustraciones_tfm_5_1.py ]]; then
    python3 scripts/generar_ilustraciones_tfm_5_1.py || log "Aviso: no se pudieron regenerar las ilustraciones 5.x; se conserva el contenido existente."
  fi
  popd >/dev/null
}

sync_legacy_reports() {
  log "Sincronizando artefactos historicos o generados fuera de la raiz..."
  for name in bench cornell_audit figures tables tfm_figuras_cap5_1 tfm_ros_gazebo_results tfm_visual_revision; do
    if [[ -d "$AI_DIR/reports/$name" ]]; then
      sync_tree "$AI_DIR/reports/$name" "$REPORTS_DIR/$name"
    fi
  done
  if [[ -d "$ROS2_DIR/reports" ]]; then
    sync_tree "$ROS2_DIR/reports" "$REPORTS_DIR/ros2_ws"
  fi
  copy_if_exists "$AI_DIR/reports/VALIDATION_REPORT_2026_03_11.txt" "$REPORTS_DIR/validation/VALIDATION_REPORT_2026_03_11.txt"
}

update_workspace_docs() {
  log "Actualizando documentacion consolidada del workspace..."
  mkdir -p "$REPORTS_DIR/docs/workspace"
}

generate_inventory_md() {
  log "Regenerando inventario maestro de artefactos y vista por capitulos..."
  python3 "$AI_DIR/scripts/generate_tfm_chapter_reports.py"
}

cleanup_nested_reports() {
  log "Eliminando carpetas reports fuera de la raiz..."
  rm -rf "$AI_DIR/reports" "$ROS2_DIR/reports"
}

main() {
  ensure_structure
  regenerate_ai_reports
  sync_legacy_reports
  update_workspace_docs
  generate_inventory_md
  cleanup_nested_reports
  log "Reports consolidados y actualizados en $REPORTS_DIR"
}

main "$@"
