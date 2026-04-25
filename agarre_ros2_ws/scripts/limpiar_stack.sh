#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/limpiar_stack.sh
# Uso breve: Reset completo del stack (procesos, SHM, daemon ROS, PID files).
#            Equivale a cold-boot sin arrancar nada nuevo.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PID_FILE="$WS_DIR/log/ros2_launch.pid"
LIMPIAR_CLEAN_LOGS="${LIMPIAR_CLEAN_LOGS:-0}"  # 1 = borra también ros2_launch.log

log()  { echo "[LIMPIAR] $*"; }
warn() { echo "[LIMPIAR][WARN] $*" >&2; }

# ── 1. Parada graceful vía PID file (si existe) ──────────────────────────────
if [[ -f "$PID_FILE" ]]; then
  pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    log "enviando SIGINT a launch PID $pid..."
    kill -INT "$pid" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.2
    done
    if kill -0 "$pid" 2>/dev/null; then
      log "aún vivo — SIGTERM..."
      kill -TERM "$pid" 2>/dev/null || true
      sleep 1
    fi
  fi
  rm -f "$PID_FILE"
  log "PID file eliminado."
fi

# ── 2. Barrido TERM de todos los patrones conocidos ───────────────────────────
log "barrido TERM..."
pkill -TERM -f "ros2 launch ur5_bringup"              2>/dev/null || true
pkill -TERM -f "ur5_qt_panel.*panel_v2|panel_v2\.py"  2>/dev/null || true
pkill -TERM -f "move_group"                            2>/dev/null || true
pkill -TERM -f "ur5_moveit_bridge"                     2>/dev/null || true
pkill -TERM -f "controller_manager|ros2_control_node"  2>/dev/null || true
pkill -TERM -f "controller_bootstrap"                  2>/dev/null || true
pkill -TERM -f "system_state_manager|release_objects_service" 2>/dev/null || true
pkill -TERM -f "gripper_attach_backend|planning_scene_sync"   2>/dev/null || true
pkill -TERM -f "gz_pose_bridge|gz_ros_control_guard|world_tf_publisher" 2>/dev/null || true
pkill -TERM -f "ros_gz_bridge|parameter_bridge"        2>/dev/null || true
pkill -TERM -f "robot_state_publisher"                 2>/dev/null || true
pkill -TERM -f "gz sim|gz-sim|gz-sim-server|gz-sim-gui|gzserver|gzclient|ign gazebo" 2>/dev/null || true
pkill -TERM -f "ros2 bag record"                       2>/dev/null || true
pkill -TERM -f "Xvfb"                                  2>/dev/null || true

_any_running() {
  pgrep -af "gz sim|gz-sim|gzserver|gzclient|ign gazebo|ros_gz_bridge|parameter_bridge|\
ros2 launch ur5_bringup|ros2_control_node|robot_state_publisher|world_tf_publisher|\
controller_manager|move_group|ur5_moveit_bridge|panel_v2\.py|ur5_qt_panel|\
system_state_manager|gripper_attach_backend|planning_scene_sync|gz_pose_bridge" \
    >/dev/null 2>&1
}

for _ in {1..25}; do
  _any_running || break
  sleep 0.2
done

# ── 3. Barrido KILL si aún quedan procesos ────────────────────────────────────
if _any_running; then
  log "procesos resistentes — barrido KILL..."
  pkill -KILL -f "ros2 launch ur5_bringup"              2>/dev/null || true
  pkill -KILL -f "ur5_qt_panel.*panel_v2|panel_v2\.py"  2>/dev/null || true
  pkill -KILL -f "move_group"                            2>/dev/null || true
  pkill -KILL -f "ur5_moveit_bridge"                     2>/dev/null || true
  pkill -KILL -f "controller_manager|ros2_control_node"  2>/dev/null || true
  pkill -KILL -f "controller_bootstrap"                  2>/dev/null || true
  pkill -KILL -f "system_state_manager|release_objects_service" 2>/dev/null || true
  pkill -KILL -f "gripper_attach_backend|planning_scene_sync"   2>/dev/null || true
  pkill -KILL -f "gz_pose_bridge|gz_ros_control_guard|world_tf_publisher" 2>/dev/null || true
  pkill -KILL -f "ros_gz_bridge|parameter_bridge"        2>/dev/null || true
  pkill -KILL -f "robot_state_publisher"                 2>/dev/null || true
  pkill -KILL -f "gz sim|gz-sim|gz-sim-server|gz-sim-gui|gzserver|gzclient|ign gazebo" 2>/dev/null || true
  pkill -KILL -f "ros2 bag record"                       2>/dev/null || true
  pkill -KILL -f "Xvfb"                                  2>/dev/null || true
  sleep 1
fi

# ── 4. Limpieza FastDDS shared-memory ────────────────────────────────────────
if [[ -d /dev/shm ]]; then
  log "limpiando FastDDS SHM (/dev/shm/fastdds*, /dev/shm/ros*)..."
  rm -rf /dev/shm/fastdds* /dev/shm/ros* 2>/dev/null || true
fi

# ── 5. Reset daemon ROS 2 (limpia caché del grafo) ───────────────────────────
if command -v ros2 >/dev/null 2>&1; then
  log "reiniciando ros2 daemon..."
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 0.5
  ros2 daemon start >/dev/null 2>&1 || true
fi

# ── 6. Limpieza opcional de logs de lanzamiento ──────────────────────────────
if [[ "$LIMPIAR_CLEAN_LOGS" == "1" ]]; then
  log "LIMPIAR_CLEAN_LOGS=1: borrando $WS_DIR/log/ros2_launch.log..."
  rm -f "$WS_DIR/log/ros2_launch.log" 2>/dev/null || true
fi

# ── 7. Informe final ─────────────────────────────────────────────────────────
if _any_running; then
  warn "aún quedan procesos del stack:"
  pgrep -af "gz sim|gz-sim|gzserver|ros_gz_bridge|parameter_bridge|ros2 launch ur5_bringup|\
ros2_control_node|robot_state_publisher|controller_manager|move_group|panel_v2\.py" 2>/dev/null || true
  exit 1
fi

log "OK — stack limpio. Listo para arrancar con start_panel_v2.sh."
