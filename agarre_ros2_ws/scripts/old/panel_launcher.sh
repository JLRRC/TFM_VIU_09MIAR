#!/bin/bash
# Wrapper que exporta variables de entorno correctas para el panel
# Usado por ur5_stack.launch.py para asegurar que ros2 run hereda el entorno

# Exportar variables ANTES de ros2 run
export PANEL_AUTO_BRIDGE="${PANEL_AUTO_BRIDGE:-1}"
export PANEL_AUTO_BRIDGE_DELAY_MS="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}"
export PANEL_AUTO_BRIDGE_MAX_RETRIES="${PANEL_AUTO_BRIDGE_MAX_RETRIES:-30}"
export PANEL_MANAGED="${PANEL_MANAGED:-0}"
export PANEL_CAMERA_REQUIRED="${PANEL_CAMERA_REQUIRED:-true}"
export PANEL_CONTROLLER_MANAGER="${PANEL_CONTROLLER_MANAGER:-/controller_manager}"

# Lanzar el panel
exec ros2 run ur5_qt_panel panel_v2
