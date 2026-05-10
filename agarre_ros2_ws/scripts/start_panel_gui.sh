#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/start_panel_gui.sh
# F-audit (2026-05-10): wrapper sobre start_panel_v2.sh que fuerza
# Gazebo GUI visible + cámaras activas, recuperando la sesión gráfica
# local (:1 / :0) cuando se ejecuta desde un SSH remoto.
#
# Uso:
#     ./scripts/start_panel_gui.sh
#
# Tras la ejecución verás:
#   * Ventana de Gazebo Sim con UR5+RG2 + objetos
#   * Panel Qt con imagen overhead
#
# Requisitos:
#   * Sesión gráfica del usuario `laboratorio` activa (ej. login en
#     pantalla física → DISPLAY=:1 con XAUTHORITY válido).
#   * El script detecta automáticamente la sesión gráfica buscando
#     procesos gnome-shell / gnome-session-binary.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── 1. Recuperar DISPLAY local (en caso de SSH) ───────────────────────────
recover_local_display() {
  local pid env_file cd cx
  for pid in $(pgrep -u "$(id -u)" -f 'gnome-shell|gnome-session-binary' 2>/dev/null); do
    env_file="/proc/$pid/environ"
    [[ -r "$env_file" ]] || continue
    cd=$(tr '\0' '\n' < "$env_file" | sed -n 's/^DISPLAY=//p' | head -n1)
    cx=$(tr '\0' '\n' < "$env_file" | sed -n 's/^XAUTHORITY=//p' | head -n1)
    [[ -n "$cd" ]] || continue
    if env DISPLAY="$cd" XAUTHORITY="${cx:-${HOME}/.Xauthority}" xdpyinfo >/dev/null 2>&1; then
      export DISPLAY="$cd"
      [[ -n "$cx" ]] && export XAUTHORITY="$cx"
      return 0
    fi
  done
  return 1
}

# Si DISPLAY actual no funciona (típico SSH), intenta recuperar local.
if ! xdpyinfo >/dev/null 2>&1; then
  if recover_local_display; then
    echo "[GUI] Recuperado DISPLAY=$DISPLAY (XAUTHORITY=$XAUTHORITY)"
  else
    echo "[ERROR] No hay DISPLAY usable. Inicia sesión gráfica en la pantalla local."
    exit 1
  fi
else
  echo "[GUI] DISPLAY=$DISPLAY funcional"
fi

# ── 2. Forzar GUI + cámaras + render moderno ──────────────────────────────
export PANEL_GZ_GUI=1
export PANEL_GZ_HEADLESS=0
export PANEL_FORCE_OFFSCREEN=0
unset QT_QPA_PLATFORM 2>/dev/null || true   # quita "offscreen" si estaba
export PANEL_CAMERA_REQUIRED=1
export PANEL_KEEP_CAMERAS=1
export PANEL_START_STACK=1                  # arranca todo el stack
export PANEL_MANAGED=1
# OGRE clásico (no OGRE2): default validado del proyecto. OGRE2 + GUI
# cliente segfaultea (SIGSEGV) en este sistema durante el arranque del
# `gz sim -g`. Si prefieres OGRE2, exporta GZ_RENDER_ENGINE=ogre2 antes
# de lanzar (asume tu PC lo soporta).
export GZ_RENDER_ENGINE="${GZ_RENDER_ENGINE:-ogre}"
# Mitigaciones GLX/Mesa por si hay problemas de driver en la GUI:
# (descomentar si gz sim -g sigue segfaultándose)
# export LIBGL_ALWAYS_SOFTWARE=1
# export __GLX_VENDOR_LIBRARY_NAME=mesa

# ── 3. Limpieza preventiva de procesos pegados ────────────────────────────
echo "[GUI] Limpiando procesos zombi del stack anterior…"
pkill -KILL -f "gz sim|gz-sim|panel_v2|ur5_qt_panel|ros2 launch|move_group|ros_gz_bridge|gz_pose_bridge|controller_manager|controller_bootstrap|world_tf_publisher|gripper_attach_backend|release_objects_service|tf_geometry_service|object_pose_resolver|pick_orchestrator|plan_to_pose_server|planning_scene_sync|robot_state_publisher|system_state_manager|joint_state_broadcaster|joint_trajectory_controller|gripper_controller|start_panel_v2" 2>/dev/null || true
sleep 2
echo "[GUI] Limpieza OK"

# ── 4. Lanzar el script estándar con todo configurado ─────────────────────
echo "[GUI] Lanzando start_panel_v2.sh con PANEL_GZ_GUI=1 PANEL_CAMERA_REQUIRED=1"
echo "[GUI] DISPLAY=$DISPLAY  XAUTHORITY=${XAUTHORITY:-<default>}"
exec "$SCRIPT_DIR/start_panel_v2.sh" "$@"
