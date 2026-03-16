#!/usr/bin/env bash
# lanzar_panelv2.sh
# Script para lanzar el panelv2 asegurando entorno gráfico y plugins Qt correctos

set -e

# Comprobar entorno gráfico
if [[ -z "$DISPLAY" && "${PANEL_FORCE_OFFSCREEN:-0}" != "1" && "${QT_QPA_PLATFORM:-}" != "offscreen" ]]; then
  echo "[ERROR] No está definida la variable DISPLAY. Abre una terminal gráfica (no TTY/SSH) y vuelve a intentarlo."
  echo "[INFO] Alternativa sin GUI: export PANEL_FORCE_OFFSCREEN=1"
  exit 1
fi

# Configurar plugins Qt del sistema
export QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins
export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms

# Activar entorno virtual si existe
for venv_dir in \
  "/home/laboratorio/TFM/agarre_inteligente/.venv-tfm" \
  "/home/laboratorio/TFM/agarre_inteligente/venv" \
  "/home/laboratorio/TFM/agarre_inteligente/.venv"; do
  if [[ -f "$venv_dir/bin/activate" ]]; then
    source "$venv_dir/bin/activate"
    break
  fi
done

# Lanzar el panelv2
cd /home/laboratorio/TFM/agarre_ros2_ws
./scripts/start_panel_v2.sh
