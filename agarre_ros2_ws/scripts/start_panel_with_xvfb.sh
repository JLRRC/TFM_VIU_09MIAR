#!/bin/bash
# Ruta/archivo: agarre_ros2_ws/scripts/start_panel_with_xvfb.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
# start_panel_with_xvfb.sh
# Wrapper que lanza el panel con Xvfb para renderizado virtual de cámaras
# Uso: ./scripts/start_panel_with_xvfb.sh [opts]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

log() { echo "[START_PANEL_XVFB] $*"; }

# Detectar si ya hay DISPLAY
if [[ -n "${DISPLAY:-}" ]]; then
  log "DISPLAY ya disponible ($DISPLAY); lanzando panel directamente sin Xvfb"
  exec "$SCRIPT_DIR/start_panel_v2.sh" "$@"
fi

# Setear ambiente para xvfb-run
export XVFB_RUN=1
log "lanzando panel con xvfb-run (para renderizado de cámaras)..."

# Usar xvfb-run para crear display virtual
# -a: buscar display disponible automáticamente
# -s: argumentos del servidor Xvfb
exec xvfb-run \
  -a \
  -s "-screen 0 1024x768x24 -ac" \
  "$SCRIPT_DIR/start_panel_v2.sh" "$@"

