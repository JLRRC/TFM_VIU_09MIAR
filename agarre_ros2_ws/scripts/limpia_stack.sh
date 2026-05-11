#!/usr/bin/env bash
# agarre_ros2_ws/scripts/limpia_stack.sh
# Reset completo del stack ROS 2 / Gazebo / MoveIt 2.
# Idempotente, no bloqueante, siempre devuelve exit 0.
#
# Uso:
#   ./limpia_stack.sh                    # limpieza normal
#   ./limpia_stack.sh 103569 103652      # matar PIDs concretos primero
#
# Variables de entorno opcionales:
#   LIMPIAR_CLEAN_LOGS=1   borra también ws/log/ros2_launch.log
set -uo pipefail

log()  { echo "[LIMPIA_STACK] $*"; }
warn() { echo "[LIMPIA_STACK][WARN] $*" >&2; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PID_FILE="$WS_DIR/log/ros2_launch.pid"
LIMPIAR_CLEAN_LOGS="${LIMPIAR_CLEAN_LOGS:-0}"

# Asegurarse de que ros2 está en PATH (necesario para daemon y node list)
# Se sourcea solo si ros2 no está ya disponible en el entorno actual.
# CRITICO: setup.bash referencia AMENT_TRACE_SETUP_FILES sin fallback, asi
# que con `set -u` activo el source falla y aborta el script. Desactivamos
# `set -u` solo durante el source (mismo patron que lanzar_panelv2.sh).
if ! command -v ros2 >/dev/null 2>&1; then
  set +u
  [[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash || true
  [[ -f "$WS_DIR/install/setup.bash" ]] && source "$WS_DIR/install/setup.bash" || true
  set -u
fi

# ── 0. PIDs explícitos pasados como argumentos ───────────────────────────────
# Ejemplo: ./limpia_stack.sh 103569 103652
# Permite matar procesos colgados conocidos antes de la limpieza general.
if [[ $# -gt 0 ]]; then
  log "matando PIDs explícitos: $*"
  for explicit_pid in "$@"; do
    if [[ "$explicit_pid" =~ ^[0-9]+$ ]]; then
      if kill -0 "$explicit_pid" 2>/dev/null; then
        log "  SIGTERM → PID $explicit_pid"
        kill -TERM "$explicit_pid" 2>/dev/null || true
      else
        log "  PID $explicit_pid ya no existe — omitiendo"
      fi
    else
      warn "  '$explicit_pid' no es un PID numérico válido — omitiendo"
    fi
  done
  sleep 1
  # SIGKILL para los que sobrevivan al TERM
  for explicit_pid in "$@"; do
    if [[ "$explicit_pid" =~ ^[0-9]+$ ]]; then
      if kill -0 "$explicit_pid" 2>/dev/null; then
        log "  SIGKILL → PID $explicit_pid (no respondió a SIGTERM)"
        kill -KILL "$explicit_pid" 2>/dev/null || true
      fi
    fi
  done
fi

# ── 1. Matar comandos ros2 CLI colgados (SIGKILL directo) ────────────────────
# CRÍTICO: comandos como `ros2 node list` sin --no-daemon o sin timeout pueden
# bloquear indefinidamente cuando el daemon DDS no responde (bug conocido en
# FastDDS con RMW sobre localhost). Se matan con SIGKILL antes de cualquier
# otra operación para que los pasos siguientes no hereden el bloqueo.
log "matando comandos ros2 CLI colgados..."
pkill -KILL -f "ros2 node list"    2>/dev/null || true
pkill -KILL -f "ros2 topic list"   2>/dev/null || true
pkill -KILL -f "ros2 service list" 2>/dev/null || true
pkill -KILL -f "ros2 action list"  2>/dev/null || true
pkill -KILL -f "ros2 param list"   2>/dev/null || true
# El proceso _ros2_daemon puede quedar colgado aunque su cliente ya terminó
pkill -KILL -f "_ros2_daemon"      2>/dev/null || true

# ── 2. Parada graceful vía PID file (si existe) ──────────────────────────────
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

# ── 3. Barrido TERM de todos los patrones conocidos del stack ────────────────
log "barrido TERM del stack..."
# Wrappers de lanzamiento (script bash y scripts auxiliares).
# CRITICO: si quedan vivos relanzan el stack tras matarlo o ejecutan
# cleanup agresivo en paralelo y se matan entre si (visto 2026-05-11:
# 4 instancias paralelas de lanzar_panelv2.sh causaban kill mutuo).
# Excluir el PID actual y su shell padre para no auto-matarse si esta
# limpieza se ejecuta DESDE uno de esos wrappers.
_self_pid=$$
_parent_pid=${PPID:-0}
for _wrapper_pat in "lanzar_panelv2\.sh" "lanzar_panelc2\.sh" "start_panel_v2\.sh" "start_panel_c2\.sh"; do
  while IFS= read -r _wp; do
    [[ -z "$_wp" ]] && continue
    if [[ "$_wp" == "$_self_pid" || "$_wp" == "$_parent_pid" ]]; then
      continue
    fi
    kill -TERM "$_wp" 2>/dev/null || true
  done < <(pgrep -f "$_wrapper_pat" 2>/dev/null || true)
done
# Launches (por nombre de paquete y por ruta directa al fichero)
pkill -TERM -f "ros2 launch ur5_bringup"               2>/dev/null || true
pkill -TERM -f "ur5_stack.launch.py"                   2>/dev/null || true
pkill -TERM -f "ur5_moveit_bringup"                    2>/dev/null || true
# Panel Qt y nodos de inferencia
pkill -TERM -f "ur5_qt_panel.*panel_v2|panel_v2\.py"   2>/dev/null || true
pkill -TERM -f "ur5_qt_panel.*main_panel|main_panel\.py" 2>/dev/null || true
pkill -TERM -f "grasp_inference"                       2>/dev/null || true
# Aux nodos F4/F5 (contact monitor + static aux joint state) — vistos
# como zombis tras barrido 2026-05-11 (no estaban listados aqui).
pkill -TERM -f "contact_monitor_node|contact_monitor\b" 2>/dev/null || true
pkill -TERM -f "static_aux_joint_state_publisher"      2>/dev/null || true
# MoveIt 2
pkill -TERM -f "move_group"                            2>/dev/null || true
pkill -TERM -f "ur5_moveit_bridge"                     2>/dev/null || true
pkill -TERM -f "ur5_moveit_py"                         2>/dev/null || true
pkill -TERM -f "planning_scene_sync"                   2>/dev/null || true
# ros2_control
pkill -TERM -f "controller_manager|ros2_control_node"  2>/dev/null || true
pkill -TERM -f "controller_bootstrap"                  2>/dev/null || true
pkill -TERM -f "spawner"                               2>/dev/null || true
# Nodos auxiliares
pkill -TERM -f "system_state_manager|release_objects_service" 2>/dev/null || true
pkill -TERM -f "gripper_attach_backend"                2>/dev/null || true
pkill -TERM -f "evidence_logger"                       2>/dev/null || true
pkill -TERM -f "tf_probe|clock_probe"                  2>/dev/null || true
# F5 (orchestrator + microservicios) — añadidos sprint B-iter1..14
pkill -TERM -f "pick_orchestrator_lifecycle"           2>/dev/null || true
pkill -TERM -f "pick_orchestrator_node\|tfm_orchestrator/pick_orchestrator\b" 2>/dev/null || true
pkill -TERM -f "plan_to_pose_server"                   2>/dev/null || true
pkill -TERM -f "object_pose_resolver_service"          2>/dev/null || true
pkill -TERM -f "tf_geometry_service"                   2>/dev/null || true
# F6 + F9 (audit 2026-05-10): LifecycleNodes nuevos.
pkill -TERM -f "panel_launch_control_node"             2>/dev/null || true
pkill -TERM -f "panel_backend_node"                    2>/dev/null || true
pkill -TERM -f "controller_health_monitor_node"        2>/dev/null || true
pkill -TERM -f "simulation_reset_service"              2>/dev/null || true
# Bridges y publishers
pkill -TERM -f "gz_pose_bridge|gz_ros_control_guard|world_tf_publisher" 2>/dev/null || true
pkill -TERM -f "ros_gz_bridge|parameter_bridge"        2>/dev/null || true
pkill -TERM -f "robot_state_publisher"                 2>/dev/null || true
# Gazebo Sim / Harmonic (todos los nombres conocidos)
pkill -TERM -f "gz sim|gz-sim|gz-sim-server|gz-sim-gui|gzserver|gzclient|ign gazebo" 2>/dev/null || true
# Misc
pkill -TERM -f "ros2 bag record"                       2>/dev/null || true
pkill -TERM -f "Xvfb"                                  2>/dev/null || true

# Comprobación de procesos residuales usando pgrep (sin ros2, no puede bloquearse)
_any_running() {
  # NOTA: los wrappers (lanzar_panelv2.sh, start_panel_v2.sh) NO se incluyen
  # aqui porque la limpieza puede invocarse desde dentro de uno y dispararia
  # falsos positivos infinitos. Su muerte se comprueba indirectamente al
  # verificar que no relancen procesos del stack.
  pgrep -af "gz sim|gz-sim|gzserver|gzclient|ign gazebo|\
ros_gz_bridge|parameter_bridge|robot_state_publisher|world_tf_publisher|\
gz_pose_bridge|gz_ros_control_guard|\
ros2 launch ur5_bringup|ur5_stack.launch.py|ur5_moveit_bringup|\
ros2_control_node|controller_manager|controller_bootstrap|spawner|\
move_group|ur5_moveit_bridge|ur5_moveit_py|\
planning_scene_sync|gripper_attach_backend|release_objects_service|system_state_manager|\
panel_v2\.py|ur5_qt_panel|grasp_inference|main_panel|\
pick_orchestrator|plan_to_pose_server|object_pose_resolver_service|\
tf_geometry_service|evidence_logger|tf_probe|clock_probe|\
panel_launch_control_node|panel_backend_node|\
controller_health_monitor_node|simulation_reset_service|\
contact_monitor_node|static_aux_joint_state_publisher" \
    >/dev/null 2>&1
}

# Dar tiempo a los procesos para que respondan al TERM antes del KILL
for _ in {1..25}; do
  _any_running || break
  sleep 0.2
done

# ── 4. Barrido KILL si aún quedan procesos resistentes ───────────────────────
if _any_running; then
  log "procesos resistentes — barrido KILL..."
  # Wrappers (excluyendo este script y su padre)
  for _wrapper_pat in "lanzar_panelv2\.sh" "lanzar_panelc2\.sh" "start_panel_v2\.sh" "start_panel_c2\.sh"; do
    while IFS= read -r _wp; do
      [[ -z "$_wp" ]] && continue
      if [[ "$_wp" == "$_self_pid" || "$_wp" == "$_parent_pid" ]]; then
        continue
      fi
      kill -KILL "$_wp" 2>/dev/null || true
    done < <(pgrep -f "$_wrapper_pat" 2>/dev/null || true)
  done
  pkill -KILL -f "ros2 launch ur5_bringup"               2>/dev/null || true
  pkill -KILL -f "ur5_stack.launch.py"                   2>/dev/null || true
  pkill -KILL -f "ur5_moveit_bringup"                    2>/dev/null || true
  pkill -KILL -f "ur5_qt_panel.*panel_v2|panel_v2\.py"   2>/dev/null || true
  pkill -KILL -f "ur5_qt_panel.*main_panel|main_panel\.py" 2>/dev/null || true
  pkill -KILL -f "grasp_inference"                       2>/dev/null || true
  pkill -KILL -f "contact_monitor_node|contact_monitor\b" 2>/dev/null || true
  pkill -KILL -f "static_aux_joint_state_publisher"      2>/dev/null || true
  pkill -KILL -f "move_group"                            2>/dev/null || true
  pkill -KILL -f "ur5_moveit_bridge"                     2>/dev/null || true
  pkill -KILL -f "ur5_moveit_py"                         2>/dev/null || true
  pkill -KILL -f "planning_scene_sync"                   2>/dev/null || true
  pkill -KILL -f "controller_manager|ros2_control_node"  2>/dev/null || true
  pkill -KILL -f "controller_bootstrap"                  2>/dev/null || true
  pkill -KILL -f "spawner"                               2>/dev/null || true
  pkill -KILL -f "system_state_manager|release_objects_service" 2>/dev/null || true
  pkill -KILL -f "gripper_attach_backend"                2>/dev/null || true
  pkill -KILL -f "evidence_logger"                       2>/dev/null || true
  pkill -KILL -f "tf_probe|clock_probe"                  2>/dev/null || true
  # F5 (orchestrator + microservicios) — añadidos sprint B-iter1..14
  pkill -KILL -f "pick_orchestrator_lifecycle"           2>/dev/null || true
  pkill -KILL -f "pick_orchestrator_node\|tfm_orchestrator/pick_orchestrator\b" 2>/dev/null || true
  pkill -KILL -f "plan_to_pose_server"                   2>/dev/null || true
  pkill -KILL -f "object_pose_resolver_service"          2>/dev/null || true
  pkill -KILL -f "tf_geometry_service"                   2>/dev/null || true
  # F6 + F9 (audit 2026-05-10): LifecycleNodes nuevos.
  pkill -KILL -f "panel_launch_control_node"             2>/dev/null || true
  pkill -KILL -f "panel_backend_node"                    2>/dev/null || true
  pkill -KILL -f "controller_health_monitor_node"        2>/dev/null || true
  pkill -KILL -f "simulation_reset_service"              2>/dev/null || true
  pkill -KILL -f "gz_pose_bridge|gz_ros_control_guard|world_tf_publisher" 2>/dev/null || true
  pkill -KILL -f "ros_gz_bridge|parameter_bridge"        2>/dev/null || true
  pkill -KILL -f "robot_state_publisher"                 2>/dev/null || true
  pkill -KILL -f "gz sim|gz-sim|gz-sim-server|gz-sim-gui|gzserver|gzclient|ign gazebo" 2>/dev/null || true
  pkill -KILL -f "ros2 bag record"                       2>/dev/null || true
  pkill -KILL -f "Xvfb"                                  2>/dev/null || true
  sleep 1
fi

# ── 5. Parar el daemon ROS 2 de forma segura ─────────────────────────────────
# CRÍTICO: sin timeout, `ros2 daemon stop` se bloquea indefinidamente si el
# daemon DDS no responde (el cliente RPC no tiene deadline interno). Con
# timeout 5s el peor caso es 5s de espera, no un cuelgue permanente.
# --no-daemon no aplica aquí porque queremos parar el daemon, no saltarlo.
if command -v ros2 >/dev/null 2>&1; then
  log "parando ros2 daemon (timeout 5s)..."
  timeout 5s ros2 daemon stop >/dev/null 2>&1 || true
  # Matar el proceso _ros2_daemon si sobrevive al stop
  pkill -KILL -f "_ros2_daemon" 2>/dev/null || true
  sleep 0.3
  log "reiniciando ros2 daemon (timeout 5s)..."
  timeout 5s ros2 daemon start >/dev/null 2>&1 || true
fi

# ── 6. Limpieza memoria compartida DDS / FastDDS ─────────────────────────────
# FastDDS deja segmentos SHM y semáforos POSIX en /dev/shm con prefijos
# fastrtps_* / fastdds_* según la versión. CycloneDDS usa cyclonedds*.
# Sin esta limpieza el siguiente proceso intenta attach a SHM huérfana,
# lo que produce bloqueos en la fase de discovery o RuntimeError en init.
if [[ -d /dev/shm ]]; then
  log "limpiando SHM DDS (/dev/shm/fastrtps_*, fastdds_*, ros*, cyclonedds*)..."
  rm -f  /dev/shm/fastrtps_*     2>/dev/null || true
  rm -f  /dev/shm/sem.fastrtps_* 2>/dev/null || true
  rm -f  /dev/shm/fastdds_*      2>/dev/null || true
  rm -f  /dev/shm/sem.fastdds_*  2>/dev/null || true
  rm -rf /dev/shm/ros*            2>/dev/null || true
  rm -f  /dev/shm/cyclonedds*    2>/dev/null || true
fi

# ── 7. Limpieza opcional de logs de lanzamiento ──────────────────────────────
if [[ "$LIMPIAR_CLEAN_LOGS" == "1" ]]; then
  log "LIMPIAR_CLEAN_LOGS=1: borrando $WS_DIR/log/ros2_launch.log..."
  rm -f "$WS_DIR/log/ros2_launch.log" 2>/dev/null || true
fi

# ── 8. Verificación final no bloqueante ──────────────────────────────────────
# Se usa ps aux en lugar de ros2 node list para no depender del daemon.
# La llamada ros2 node list usa --no-daemon (consulta directa sin intermediario)
# y timeout para garantizar que nunca se bloquea aunque el daemon esté caído.
log "verificación final (no bloqueante)..."
log "--- procesos residuales del stack ---"
ps aux | grep -E \
  "gz.sim|gz-sim|ros_gz_bridge|parameter_bridge|\
robot_state_publisher|world_tf_publisher|\
move_group|ur5_moveit_bridge|controller_manager|\
gripper_attach_backend|system_state_manager|\
panel_v2|main_panel|grasp_inference|\
pick_orchestrator|plan_to_pose_server|object_pose_resolver_service|\
tf_geometry_service|evidence_logger|\
panel_launch_control_node|panel_backend_node|\
controller_health_monitor_node|simulation_reset_service|\
contact_monitor_node|static_aux_joint_state_publisher|\
lanzar_panelv2|lanzar_panelc2|start_panel_v2|start_panel_c2" \
  | grep -v grep || true
# Reaper de zombis: si el shell padre del stack ya murió, los hijos quedan
# como <defunct>. Forzamos un wait sobre cualquier hijo del PID 1 con
# nombre conocido (no se pueden "matar" pero se loguean para diagnostico).
log "--- procesos zombi (<defunct>) ---"
ps -eo pid,ppid,stat,comm | awk '$3 ~ /Z/ {print}' | head -20 || true
log "--- ros2 node list (timeout 5s, --no-daemon) ---"
# --no-daemon: consulta la red DDS directamente, evita el daemon como proxy.
# timeout 5s: garantiza que no se bloquea aunque DDS no responda.
timeout 5s ros2 node list --no-daemon 2>/dev/null || log "(sin nodos visibles o timeout — normal si el stack está apagado)"
log "--- fin verificación ---"

log "OK — stack limpio. Listo para arrancar con ./scripts/start_panel_v2.sh"
exit 0
