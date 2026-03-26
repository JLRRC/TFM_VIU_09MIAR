#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/start_panel_v2.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -euo pipefail

# =========================
# start_panel_v2.sh
# Workspace: ~/TFM/agarre_ros2_ws
# =========================

# --- helpers ---
log() { echo "[START_PANEL_V2] $*"; }
err() { echo "[START_PANEL_V2] ERROR: $*" >&2; }
cleanup_ran=0
cleanup() {
  if [[ "$cleanup_ran" == "1" ]]; then
    return
  fi
  cleanup_ran=1
  log "cerrando panel y procesos hijos..."
  kill -INT -- -$$ >/dev/null 2>&1 || true
  sleep 0.5
  kill -TERM -- -$$ >/dev/null 2>&1 || true
}
on_tstp() {
  log "SIGTSTP recibido; cerrando en vez de suspender."
  cleanup
  exit 0
}
setup_traps() {
  trap cleanup EXIT INT TERM HUP
  trap on_tstp TSTP
}

runtime_sanity_check() {
  local strict_sanity="${PANEL_STRICT_RUNTIME_SANITY:-1}"
  if [[ "$strict_sanity" != "1" ]]; then
    return 0
  fi
  local foreign_ros
  local foreign_gz
  foreign_ros="$(ps -ef | grep -E 'ros2 launch .*simple_arm|simple_arm_bringup|agarre_ros2_ws_moveit2' | grep -v grep || true)"
  foreign_gz="$(ps -ef | grep -E 'gz sim' | grep -E 'simple_world\.sdf|agarre_ros2_ws_moveit2' | grep -v grep || true)"
  if [[ -n "$foreign_ros" || -n "$foreign_gz" ]]; then
    err "Preflight runtime fallido: se detectaron procesos de otro workspace/robot activos."
    if [[ -n "$foreign_ros" ]]; then
      err "Procesos ROS externos detectados:"
      echo "$foreign_ros" >&2
    fi
    if [[ -n "$foreign_gz" ]]; then
      err "Procesos Gazebo externos detectados:"
      echo "$foreign_gz" >&2
    fi
    err "Detén esos procesos o exporta PANEL_STRICT_RUNTIME_SANITY=0 bajo tu responsabilidad."
    exit 3
  fi
}

# Detectar WS_DIR (raíz del repo)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
export WS_DIR

# Config (puedes exportar estas vars antes de lanzar)
: "${ROS_DISTRO:=jazzy}"
: "${PANEL_COLD_BOOT:=1}"          # 1 = mata procesos antes de arrancar
: "${PANEL_V2_PREFER_INSTALLED:=1}"# 1 = usa install/.../panel_v2 si existe
: "${PANEL_MODE:=auto}"            # auto=stack completo por defecto (usar manual para modo legacy)
: "${PANEL_START_STACK:=1}"        # 1 = autoarranca RSP+Gazebo
: "${PANEL_MANAGED:=0}"            # 1 = panel guiado por /system_state
: "${PANEL_CONTROLLER_MANAGER:=/controller_manager}" # namespace del controller_manager
: "${PANEL_WRITE_PID:=0}"          # 1 = ejecutar en background y crear pidfile
: "${PANEL_PROMPT_PID:=0}"         # 1 = preguntar si se quiere pidfile (solo TTY)
: "${PANEL_VENV_AUTO:=1}"          # 1 = activa venv automaticamente si existe
: "${PANEL_VENV_DIR:=}"
: "${PANEL_STRICT_PHYSICS_MODE:=0}" # 1 = desactiva attach asistido y usa self-collision mas estricta
: "${PANEL_STRICT_RUNTIME_SANITY:=1}" # 1 = bloquea arranque si detecta otro stack ROS activo
: "${PANEL_MOVEIT_MODE:=auto}"        # auto|move_group|bridge
: "${PANEL_ROS_EXECUTOR_THREADS:=3}"  # executor del RosWorker para no bloquear /clock durante servicios largos
: "${PANEL_GRIPPER_CLOSED_RAD:=0.60}" # RG2 en este modelo suele cerrar alrededor de ~0.60 rad
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"
if [[ -z "${PANEL_GZ_GUI+x}" ]]; then
  if [[ -n "${DISPLAY:-}" && "${PANEL_FORCE_OFFSCREEN:-0}" != "1" && "${QT_QPA_PLATFORM:-}" != "offscreen" ]]; then
    PANEL_GZ_GUI="1"
  else
    PANEL_GZ_GUI="0"
  fi
fi
export RMW_IMPLEMENTATION
export PANEL_CONTROLLER_MANAGER
export PANEL_ROS_EXECUTOR_THREADS
export PANEL_GRIPPER_CLOSED_RAD
export PANEL_GZ_GUI

if [[ -z "${PANEL_VENV_DIR}" ]]; then
  for candidate in \
    "/home/laboratorio/TFM/agarre_inteligente/.venv-tfm" \
    "/home/laboratorio/TFM/agarre_inteligente/venv" \
    "/home/laboratorio/TFM/agarre_inteligente/.venv"; do
    if [[ -f "$candidate/bin/activate" ]]; then
      PANEL_VENV_DIR="$candidate"
      break
    fi
  done
fi
if [[ -z "${PANEL_VENV_DIR}" ]]; then
  PANEL_VENV_DIR="/home/laboratorio/TFM/agarre_inteligente/.venv-tfm"
fi

# --- argumentos opcionales ---
while [[ $# -gt 0 ]]; do
  case "$1" in
    --bg|--pid)
      PANEL_WRITE_PID=1
      shift
      ;;
    --prompt)
      PANEL_PROMPT_PID=1
      shift
      ;;
    --fg)
      PANEL_WRITE_PID=0
      shift
      ;;
    *)
      err "Argumento desconocido: $1"
      exit 2
      ;;
  esac
done

log "WS_DIR=$WS_DIR"
log "PANEL_CONTROLLER_MANAGER=${PANEL_CONTROLLER_MANAGER}"

# --- activar venv si procede ---
if [[ "$PANEL_VENV_AUTO" == "1" && -z "${VIRTUAL_ENV:-}" ]]; then
  if [[ -f "$PANEL_VENV_DIR/bin/activate" ]]; then
    log "activando venv: $PANEL_VENV_DIR"
    set +u
    # shellcheck disable=SC1090
    source "$PANEL_VENV_DIR/bin/activate"
    set -u
  fi
fi
if [[ -n "${VIRTUAL_ENV:-}" && -x "$PANEL_VENV_DIR/bin/python" ]]; then
  export PANEL_PYTHON="$PANEL_VENV_DIR/bin/python"
fi
if [[ -z "${VISION_DIR:-}" && -d "/home/laboratorio/TFM/agarre_inteligente" ]]; then
  export VISION_DIR="/home/laboratorio/TFM/agarre_inteligente"
fi

# --- cold boot: limpieza de procesos ---
if [[ "$PANEL_COLD_BOOT" == "1" ]]; then
  log "cold boot: matando procesos previos..."
  # Limpieza FastDDS (opcional pero útil)
  if [[ -d /dev/shm ]]; then
    rm -rf /dev/shm/fastdds* /dev/shm/ros* 2>/dev/null || true
  fi

  # Procesos típicos del stack
  pkill -f "gz sim"            2>/dev/null || true
  pkill -f "gzserver"          2>/dev/null || true
  pkill -f "gzclient"          2>/dev/null || true
  pkill -f "ign gazebo"        2>/dev/null || true
  pkill -f "ros_gz_bridge"     2>/dev/null || true
  pkill -f "parameter_bridge"  2>/dev/null || true
  pkill -f "robot_state_publisher" 2>/dev/null || true
  pkill -f "ros2_control_node" 2>/dev/null || true
  pkill -f "controller_manager" 2>/dev/null || true
  pkill -f "spawner" 2>/dev/null || true
  pkill -f "release_objects_service" 2>/dev/null || true
  pkill -f "system_state_manager" 2>/dev/null || true
  pkill -f "ros2 launch ur5_bringup" 2>/dev/null || true

  # Paneles previos (ajusta patrones si lo necesitas)
  pkill -f "ur5_qt_panel"      2>/dev/null || true
  pkill -f "panel_v2.py"       2>/dev/null || true
  pkill -f "main_panel.py"     2>/dev/null || true
  pkill -f "ros2 run ur5_qt_panel panel_v2" 2>/dev/null || true

  # Eliminar procesos zombis (procesos bash huérfanos pausados)
  log "limpiando procesos zombis..."
  ps aux | awk '$8 == "Z" {print $2}' | xargs -r kill -9 2>/dev/null || true
  ps aux | awk '$8 == "T" {print $2}' | xargs -r kill -9 2>/dev/null || true

  any_running() {
    pgrep -af "ros2 bag record|ros_gz_bridge|parameter_bridge|gz sim|gz-sim|gzserver|gzclient|ign gazebo|ros2 launch ur5_bringup|ros2_control_node|robot_state_publisher|world_tf_publisher|controller_manager|spawner|move_group|ur5_qt_panel|panel_v2.py|main_panel.py" >/dev/null 2>&1
  }

  # Verificación: si quedan procesos, intentar cierre forzado.
  for _ in {1..20}; do
    if any_running; then
      sleep 0.2
    else
      break
    fi
  done
  if any_running; then
    log "cold boot: procesos aún activos, forzando cierre..."
    pkill -KILL -f "ros2 bag record" >/dev/null 2>&1 || true
    pkill -KILL -f "ros_gz_bridge" >/dev/null 2>&1 || true
    pkill -KILL -f "parameter_bridge" >/dev/null 2>&1 || true
    pkill -KILL -f "gz sim" >/dev/null 2>&1 || true
    pkill -KILL -f "gz-sim" >/dev/null 2>&1 || true
    pkill -KILL -f "gzserver" >/dev/null 2>&1 || true
    pkill -KILL -f "gzclient" >/dev/null 2>&1 || true
    pkill -KILL -f "ign gazebo" >/dev/null 2>&1 || true
    pkill -KILL -f "ros2 launch ur5_bringup" >/dev/null 2>&1 || true
    pkill -KILL -f "ros2_control_node" >/dev/null 2>&1 || true
    pkill -KILL -f "robot_state_publisher" >/dev/null 2>&1 || true
    pkill -KILL -f "world_tf_publisher" >/dev/null 2>&1 || true
    pkill -KILL -f "controller_manager" >/dev/null 2>&1 || true
    pkill -KILL -f "spawner" >/dev/null 2>&1 || true
    pkill -KILL -f "move_group" >/dev/null 2>&1 || true
    pkill -KILL -f "release_objects_service" >/dev/null 2>&1 || true
    pkill -KILL -f "system_state_manager" >/dev/null 2>&1 || true
    pkill -KILL -f "ur5_qt_panel" >/dev/null 2>&1 || true
    pkill -KILL -f "panel_v2.py" >/dev/null 2>&1 || true
    pkill -KILL -f "main_panel.py" >/dev/null 2>&1 || true
    pkill -KILL -f "ros2 run ur5_qt_panel panel_v2" >/dev/null 2>&1 || true
  fi
fi

# --- cargar entorno ROS2 + overlay ---
# --- cargar entorno ROS2 + overlay ---
log "cargando entorno ROS 2 ${ROS_DISTRO} ($WS_DIR)"

ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  err "No existe $ROS_SETUP. ¿Seguro que estás en ROS 2 ${ROS_DISTRO}?"
  exit 1
fi

# 🔑 ROS 2 Jazzy NO soporta `set -u` durante source
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"

if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "$WS_DIR/install/setup.bash"
else
  log "Aviso: no existe install/setup.bash (¿falta colcon build?). Continuo igualmente."
fi
set -u

# --- Recursos Gazebo y modo headless ---
export GZ_SIM_RESOURCE_PATH="$WS_DIR/models:$WS_DIR/worlds:$WS_DIR/install${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export PANEL_AUTO_BRIDGE_DELAY_MS="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}"
export RMW_FASTRTPS_USE_SHM="${RMW_FASTRTPS_USE_SHM:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$WS_DIR/scripts/fastdds_no_shm.xml}"
export USE_SIM_TIME="${USE_SIM_TIME:-1}"
log "GZ_SIM_RESOURCE_PATH set to: $GZ_SIM_RESOURCE_PATH"

if [[ -z "${LIBGL_ALWAYS_SOFTWARE:-}" ]]; then
  # En GUI priorizamos estabilidad de gz-gui (evitar segfaults OpenGL/QtQuick).
  if [[ "${PANEL_GZ_GUI:-0}" == "1" ]]; then
    export LIBGL_ALWAYS_SOFTWARE=1
  else
    export LIBGL_ALWAYS_SOFTWARE=0
  fi
fi
export QT_XCB_GL_INTEGRATION=none
if [[ "${PANEL_GZ_GUI:-0}" == "1" ]]; then
  export QT_QUICK_BACKEND="${QT_QUICK_BACKEND:-software}"
  export QSG_RENDER_LOOP="${QSG_RENDER_LOOP:-basic}"
fi
export PANEL_SKIP_CLEANUP="${PANEL_SKIP_CLEANUP:-0}"
export PANEL_KILL_STALE="${PANEL_KILL_STALE:-1}"

# Configurar rendering para Gazebo cameras (EGL offscreen)
# Esto permite que las cámaras virtuales se rendericen incluso sin display
export LIBGL_INDIRECT_RENDERING=1
if [[ -z "${DISPLAY:-}" || "${PANEL_FORCE_OFFSCREEN:-0}" == "1" ]]; then
  export MESA_GL_VERSION_OVERRIDE=4.5
  export LIBGLVND_DRIVERS_PATH="/usr/lib/x86_64-linux-gnu/GL"
fi

if [[ -n "${PANEL_QT_PLATFORM:-}" ]]; then
  export QT_QPA_PLATFORM="$PANEL_QT_PLATFORM"
elif [[ -z "${DISPLAY:-}" || "${PANEL_FORCE_OFFSCREEN:-0}" == "1" ]]; then
  export QT_QPA_PLATFORM=offscreen
fi
export PANEL_STALE_GRACE_SEC="${PANEL_STALE_GRACE_SEC:-20}"
# Vision en este stack es critica: por defecto requerimos camara y preservamos modelos de camara.
if [[ -z "${PANEL_CAMERA_REQUIRED:-}" ]]; then
  export PANEL_CAMERA_REQUIRED=1
fi
export PANEL_KEEP_CAMERAS="${PANEL_KEEP_CAMERAS:-${PANEL_CAMERA_REQUIRED}}"
if [[ "${PANEL_CAMERA_REQUIRED}" == "1" && "${PANEL_KEEP_CAMERAS}" != "1" ]]; then
  log "Forzando PANEL_KEEP_CAMERAS=1 porque PANEL_CAMERA_REQUIRED=1"
  export PANEL_KEEP_CAMERAS=1
fi
export PANEL_SETTINGS_YAML="${PANEL_SETTINGS_YAML:-$WS_DIR/src/ur5_qt_panel/config/panel_settings.yaml}"
export INFER_ROI_SIZE="${INFER_ROI_SIZE:-96}"
export PANEL_CRITICAL_POSE_TIMEOUT_SEC="${PANEL_CRITICAL_POSE_TIMEOUT_SEC:-60.0}"
export PANEL_CRITICAL_CLOCK_TIMEOUT_SEC="${PANEL_CRITICAL_CLOCK_TIMEOUT_SEC:-5.0}"
export PANEL_PICK_OBJECT_EXTRA_DOWN_Z="${PANEL_PICK_OBJECT_EXTRA_DOWN_Z:-0.020}"
export PANEL_PICK_OBJECT_CONTACT_DOWN_Z_M="${PANEL_PICK_OBJECT_CONTACT_DOWN_Z_M:-0.020}"
export PANEL_PICK_OBJECT_POST_LIFT_MAX_DIST_M="${PANEL_PICK_OBJECT_POST_LIFT_MAX_DIST_M:-0.18}"
export PANEL_PICK_OBJECT_POST_LIFT_MIN_CONSECUTIVE="${PANEL_PICK_OBJECT_POST_LIFT_MIN_CONSECUTIVE:-1}"
export PANEL_PICK_OBJECT_CARRY_JOINT_TIME_SEC="${PANEL_PICK_OBJECT_CARRY_JOINT_TIME_SEC:-12.0}"
export PANEL_PICK_OBJECT_MOVEIT_EXCLUSIVE="${PANEL_PICK_OBJECT_MOVEIT_EXCLUSIVE:-1}"
export PANEL_PICK_OBJECT_PREFLIGHT_MODE="${PANEL_PICK_OBJECT_PREFLIGHT_MODE:-home}"
export PANEL_PICK_OBJECT_ALLOW_JOINT_PREFLIGHT_IN_MOVEIT="${PANEL_PICK_OBJECT_ALLOW_JOINT_PREFLIGHT_IN_MOVEIT:-0}"
export PANEL_PICK_OBJECT_PICK_IMAGE_PREFLIGHT_TOL_RAD="${PANEL_PICK_OBJECT_PICK_IMAGE_PREFLIGHT_TOL_RAD:-0.05}"
export PANEL_PICK_OBJECT_PICK_IMAGE_PREFLIGHT_REQUIRE_REACHED="${PANEL_PICK_OBJECT_PICK_IMAGE_PREFLIGHT_REQUIRE_REACHED:-1}"
export PANEL_PICK_OBJECT_APPROACH_JOINT_FALLBACK="${PANEL_PICK_OBJECT_APPROACH_JOINT_FALLBACK:-1}"
export PANEL_PICK_OBJECT_PRE_GRASP_JOINT_FALLBACK="${PANEL_PICK_OBJECT_PRE_GRASP_JOINT_FALLBACK:-1}"
export PANEL_PICK_OBJECT_DETERMINISTIC_JOINT_AFTER_APPROACH="${PANEL_PICK_OBJECT_DETERMINISTIC_JOINT_AFTER_APPROACH:-0}"
export PANEL_PICK_OBJECT_DETERMINISTIC_CARRY_GATE_MAX_M="${PANEL_PICK_OBJECT_DETERMINISTIC_CARRY_GATE_MAX_M:-0.80}"
export PANEL_PICK_OBJECT_DETERMINISTIC_CARRY_GATE_MIN_CONSECUTIVE="${PANEL_PICK_OBJECT_DETERMINISTIC_CARRY_GATE_MIN_CONSECUTIVE:-1}"
export PANEL_PICK_OBJECT_APPROACH_SKIP_RETRY_ON_FALLBACK="${PANEL_PICK_OBJECT_APPROACH_SKIP_RETRY_ON_FALLBACK:-1}"
export PANEL_PICK_OBJECT_APPROACH_TOL_M="${PANEL_PICK_OBJECT_APPROACH_TOL_M:-0.10}"
export PANEL_PICK_OBJECT_PRE_GRASP_TOL_M="${PANEL_PICK_OBJECT_PRE_GRASP_TOL_M:-0.10}"
export PANEL_PICK_OBJECT_PRE_GRASP_RECENTER_TOL_M="${PANEL_PICK_OBJECT_PRE_GRASP_RECENTER_TOL_M:-0.10}"
export PANEL_PICK_OBJECT_PRE_GRASP_ABORT_RECOVERY_TOL_M="${PANEL_PICK_OBJECT_PRE_GRASP_ABORT_RECOVERY_TOL_M:-0.10}"
export PANEL_PICK_OBJECT_PRE_GRASP_ALIGN_XY_TOL_M="${PANEL_PICK_OBJECT_PRE_GRASP_ALIGN_XY_TOL_M:-0.045}"
export PANEL_PICK_OBJECT_PRE_GRASP_RECENTER_ENABLE="${PANEL_PICK_OBJECT_PRE_GRASP_RECENTER_ENABLE:-1}"
export PANEL_PICK_OBJECT_GRASP_JOINT_FALLBACK="${PANEL_PICK_OBJECT_GRASP_JOINT_FALLBACK:-0}"
export PANEL_PICK_OBJECT_GRASP_Z_REACH_TOL_FALLBACK_M="${PANEL_PICK_OBJECT_GRASP_Z_REACH_TOL_FALLBACK_M:-0.160}"
export PANEL_PICK_OBJECT_GRASP_STEP_TOL_M="${PANEL_PICK_OBJECT_GRASP_STEP_TOL_M:-0.040}"
export PANEL_PICK_OBJECT_STEP_TOL_M="${PANEL_PICK_OBJECT_STEP_TOL_M:-0.040}"
export PANEL_PICK_OBJECT_OBJECT_XY_GATE_TOL_M="${PANEL_PICK_OBJECT_OBJECT_XY_GATE_TOL_M:-0.030}"
export PANEL_PICK_OBJECT_TCP_OBJ_DIST_MAX_M="${PANEL_PICK_OBJECT_TCP_OBJ_DIST_MAX_M:-0.055}"
export PANEL_PICK_OBJECT_TRANSPORT_JOINT_FALLBACK="${PANEL_PICK_OBJECT_TRANSPORT_JOINT_FALLBACK:-0}"
export PANEL_PICK_OBJECT_TRANSPORT_JOINT_ONLY="${PANEL_PICK_OBJECT_TRANSPORT_JOINT_ONLY:-0}"
export PANEL_PICK_OBJECT_RETURN_TO_MESA="${PANEL_PICK_OBJECT_RETURN_TO_MESA:-0}"
export PANEL_PICK_OBJECT_HOME_BEFORE_CESTA="${PANEL_PICK_OBJECT_HOME_BEFORE_CESTA:-0}"
# La ruta canónica TFM -> MoveIt debe planificar desde el estado vivo actual.
# HOME_START sigue disponible para diagnóstico, pero fuera del camino crítico.
export PANEL_PICK_OBJECT_FORCE_HOME_START="${PANEL_PICK_OBJECT_FORCE_HOME_START:-0}"
export PANEL_PICK_OBJECT_HOME_TOL_RAD="${PANEL_PICK_OBJECT_HOME_TOL_RAD:-0.08}"
export PANEL_PICK_OBJECT_HOME_START_READY_WAIT_SEC="${PANEL_PICK_OBJECT_HOME_START_READY_WAIT_SEC:-60.0}"
export PANEL_PICK_OBJECT_HOME_START_DUR_SEC="${PANEL_PICK_OBJECT_HOME_START_DUR_SEC:-8.0}"
export PANEL_PICK_OBJECT_ORIENTATION_XYZW="${PANEL_PICK_OBJECT_ORIENTATION_XYZW:-0.70710678,0.0,0.70710678,0.0}"
export PANEL_PICK_OBJECT_APPROACH_CARTESIAN="${PANEL_PICK_OBJECT_APPROACH_CARTESIAN:-0}"
export PANEL_PICK_OBJECT_GRASP_CARTESIAN="${PANEL_PICK_OBJECT_GRASP_CARTESIAN:-0}"
export PANEL_PICK_OBJECT_LIFT_CARTESIAN="${PANEL_PICK_OBJECT_LIFT_CARTESIAN:-0}"
export PANEL_PICK_OBJECT_TRANSPORT_CARTESIAN="${PANEL_PICK_OBJECT_TRANSPORT_CARTESIAN:-0}"
export PANEL_PICK_OBJECT_STAGED_LIFT_ALWAYS="${PANEL_PICK_OBJECT_STAGED_LIFT_ALWAYS:-1}"
export PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_STEP_M="${PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_STEP_M:-0.005}"
export PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_MAX_DIST_M="${PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_MAX_DIST_M:-0.270}"
export PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC="${PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC:-25.0}"
export PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC="${PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC:-80.0}"
export PANEL_MOVEIT_BRIDGE_JOINT_STATE_TIMEOUT_SEC="${PANEL_MOVEIT_BRIDGE_JOINT_STATE_TIMEOUT_SEC:-6.0}"
export PANEL_MOVEIT_BRIDGE_JOINT_STATE_MAX_AGE_SEC="${PANEL_MOVEIT_BRIDGE_JOINT_STATE_MAX_AGE_SEC:-2.5}"
export PANEL_MOVEIT_BRIDGE_FORCE_FJT_DIRECT="${PANEL_MOVEIT_BRIDGE_FORCE_FJT_DIRECT:-0}"
export PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TIME_TOL_SEC="${PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TIME_TOL_SEC:-45.0}"
export PANEL_MOVEIT_BRIDGE_CONTROLLER_EXPECTED_GOAL_TIME_SEC="${PANEL_MOVEIT_BRIDGE_CONTROLLER_EXPECTED_GOAL_TIME_SEC:-45.0}"
export PANEL_MOVEIT_BRIDGE_CONTROLLER_PATH_TOL_RAD="${PANEL_MOVEIT_BRIDGE_CONTROLLER_PATH_TOL_RAD:-4.0}"
export PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TOL_RAD="${PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TOL_RAD:-0.20}"
export PANEL_PICK_OBJECT_MOVEIT_WAIT_SEC="${PANEL_PICK_OBJECT_MOVEIT_WAIT_SEC:-420.0}"
export PANEL_PICK_OBJECT_MOVEIT_WAIT_RECOVERED_SEC="${PANEL_PICK_OBJECT_MOVEIT_WAIT_RECOVERED_SEC:-420.0}"
export PANEL_TFM_MOVEIT_RESULT_TIMEOUT_SEC="${PANEL_TFM_MOVEIT_RESULT_TIMEOUT_SEC:-420.0}"
export PANEL_PICK_OBJECT_MOVEIT_ACTIVE_REQUEST_GRACE_SEC="${PANEL_PICK_OBJECT_MOVEIT_ACTIVE_REQUEST_GRACE_SEC:-480.0}"
export PANEL_TFM_MOVEIT_ACTIVE_REQUEST_GRACE_SEC="${PANEL_TFM_MOVEIT_ACTIVE_REQUEST_GRACE_SEC:-480.0}"
export PANEL_SELECT_OBJECT_SERVICE_TIMEOUT_SEC="${PANEL_SELECT_OBJECT_SERVICE_TIMEOUT_SEC:-10.0}"
export PANEL_TFM_INFER_SERVICE_TIMEOUT_SEC="${PANEL_TFM_INFER_SERVICE_TIMEOUT_SEC:-30.0}"
export PANEL_TFM_EXECUTE_SERVICE_TIMEOUT_SEC="${PANEL_TFM_EXECUTE_SERVICE_TIMEOUT_SEC:-480.0}"
export PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE="${PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE:-0.15}"
export PANEL_MOVEIT_BRIDGE_ACCEL_SCALE="${PANEL_MOVEIT_BRIDGE_ACCEL_SCALE:-0.15}"
# Perfil de defensa: priorizar trayectorias pose-based y arranque asentado del controlador.
export PANEL_TFM_GRASP_CARTESIAN="${PANEL_TFM_GRASP_CARTESIAN:-0}"
export PANEL_TFM_CANONICAL_USE_PICK_OBJECT="${PANEL_TFM_CANONICAL_USE_PICK_OBJECT:-1}"
export PANEL_TFM_CANONICAL_USE_GRASP_YAW="${PANEL_TFM_CANONICAL_USE_GRASP_YAW:-1}"
export PANEL_TFM_CANONICAL_GRASP_XY_MAX_DELTA_M="${PANEL_TFM_CANONICAL_GRASP_XY_MAX_DELTA_M:-0.080}"
# Perfil canónico: la selección del objeto manda en XY; el TFM aporta yaw.
export PANEL_TFM_CANONICAL_SNAP_SELECTED_XY="${PANEL_TFM_CANONICAL_SNAP_SELECTED_XY:-1}"
export PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_TIMEOUT_SEC="${PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_TIMEOUT_SEC:-3.0}"
export PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_STABLE_SEC="${PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_STABLE_SEC:-0.45}"
export PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_TOL_RAD="${PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_TOL_RAD:-0.015}"
export PANEL_MOVEIT_BRIDGE_START_LEAD_GAIN="${PANEL_MOVEIT_BRIDGE_START_LEAD_GAIN:-0.60}"
export PANEL_MOVEIT_BRIDGE_START_LEAD_MAX_SEC="${PANEL_MOVEIT_BRIDGE_START_LEAD_MAX_SEC:-4.0}"
export PANEL_MOVEIT_BRIDGE_START_BLEND_ERR_RAD="${PANEL_MOVEIT_BRIDGE_START_BLEND_ERR_RAD:-0.80}"
export PANEL_MOVEIT_BRIDGE_START_BLEND_RATIO="${PANEL_MOVEIT_BRIDGE_START_BLEND_RATIO:-0.55}"
export PANEL_CONTROLLER_READY_TIMEOUT_SEC="${PANEL_CONTROLLER_READY_TIMEOUT_SEC:-60.0}"
export ATTACH_BACKEND_MAX_POSE_AGE_SEC="${ATTACH_BACKEND_MAX_POSE_AGE_SEC:-2.5}"
export ATTACH_BACKEND_FOLLOW_RATE_HZ="${ATTACH_BACKEND_FOLLOW_RATE_HZ:-10.0}"
export ATTACH_BACKEND_FOLLOW_BREAK_DIST_M="${ATTACH_BACKEND_FOLLOW_BREAK_DIST_M:-0.45}"
export ATTACH_BACKEND_MODE="${ATTACH_BACKEND_MODE:-follow_tcp}"
PYQT_PLUGINS_DIR=""
if [[ -n "${PANEL_VENV_DIR:-}" ]]; then
  for pyqt_candidate in \
    "$PANEL_VENV_DIR/lib/python3.12/site-packages/PyQt5/Qt5/plugins" \
    "$PANEL_VENV_DIR/lib64/python3.12/site-packages/PyQt5/Qt5/plugins"; do
    if [[ -d "$pyqt_candidate" ]]; then
      PYQT_PLUGINS_DIR="$pyqt_candidate"
      break
    fi
  done
fi
if [[ -z "$PYQT_PLUGINS_DIR" || ! -d "$PYQT_PLUGINS_DIR" ]]; then
  PYQT_PLUGINS_DIR=""
fi
if [[ -n "$PYQT_PLUGINS_DIR" ]]; then
  export QT_PLUGIN_PATH="$PYQT_PLUGINS_DIR"
  export QT_QPA_PLATFORM_PLUGIN_PATH="$PYQT_PLUGINS_DIR/platforms"
else
  if [[ -z "${QT_PLUGIN_PATH:-}" && -d "/usr/lib/x86_64-linux-gnu/qt5/plugins" ]]; then
    export QT_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/qt5/plugins"
  fi
  if [[ -z "${QT_QPA_PLATFORM_PLUGIN_PATH:-}" && -d "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms" ]]; then
    export QT_QPA_PLATFORM_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"
  fi
fi

# Preparar modelo runtime con params reales para gz_ros2_control (panel lanza gz sim directamente).
runtime_models_root="$WS_DIR/log/gz_models"
runtime_ur5_model="$runtime_models_root/ur5_rg2"
rm -rf "$runtime_ur5_model" 2>/dev/null || true
mkdir -p "$runtime_ur5_model"
cp -a "$WS_DIR/models/ur5_rg2/." "$runtime_ur5_model/" 2>/dev/null || true
controllers_yaml="$(python3 - <<'PY'
from ament_index_python.packages import get_package_share_directory
import os
print(os.path.join(get_package_share_directory("ur5_description"), "config", "ur5_controllers.yaml"))
PY
)"
if [[ -n "$controllers_yaml" && -f "$runtime_ur5_model/model.sdf" ]]; then
  python3 - <<PY
import re
path = r"$runtime_ur5_model/model.sdf"
params = r"$controllers_yaml"
with open(path, "r", encoding="utf-8") as f:
    text = f.read()
pat = re.compile(r'(<plugin filename="gz_ros2_control-system"[^>]*>)(.*?)(</plugin>)', re.DOTALL)
m = pat.search(text)
if m:
    header, body, footer = m.groups()
    if "<parameters>" in body:
        body = re.sub(r"<parameters>.*?</parameters>", f"<parameters>{params}</parameters>", body, flags=re.DOTALL)
    else:
        body = body + f"\n            <parameters>{params}</parameters>\n"
    text = text[:m.start()] + header + body + footer + text[m.end():]
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
PY
  export GZ_SIM_RESOURCE_PATH="$runtime_models_root:$GZ_SIM_RESOURCE_PATH"
fi

log "Nota: start_panel_v2.sh es wrapper; usa ros2 launch ur5_bringup ur5_stack.launch.py"

PANEL_START_ROS2_CONTROL="${PANEL_START_ROS2_CONTROL:-0}"
PANEL_LAUNCH_BRIDGE="${PANEL_LAUNCH_BRIDGE:-$PANEL_START_STACK}"
PANEL_LAUNCH_WORLD_TF="${PANEL_LAUNCH_WORLD_TF:-1}"
PANEL_LAUNCH_SYSTEM_STATE="${PANEL_LAUNCH_SYSTEM_STATE:-1}"
PANEL_LAUNCH_MOVEIT="${PANEL_LAUNCH_MOVEIT:-$PANEL_START_STACK}"
export PANEL_ALLOW_UNSETTLED_ON_TIMEOUT="${PANEL_ALLOW_UNSETTLED_ON_TIMEOUT:-1}"
export PANEL_AUTO_RELEASE_DROP_OBJECTS="${PANEL_AUTO_RELEASE_DROP_OBJECTS:-1}"
export DEBUG_LOGS_TO_STDOUT="${DEBUG_LOGS_TO_STDOUT:-0}"
export PANEL_AUTO_EXIT_ON_PANEL="${PANEL_AUTO_EXIT_ON_PANEL:-1}"

if [[ "${PANEL_MODE}" == "manual" ]]; then
  PANEL_START_STACK="0"
  PANEL_START_ROS2_CONTROL="0"
  PANEL_LAUNCH_BRIDGE="0"
  PANEL_LAUNCH_WORLD_TF="0"
  PANEL_LAUNCH_SYSTEM_STATE="0"
  PANEL_LAUNCH_MOVEIT="0"
  PANEL_MANAGED="0"
fi
HEADLESS="true"
if [[ "${PANEL_GZ_GUI:-0}" == "1" ]]; then
  HEADLESS="false"
fi

if [[ -z "${GZ_RENDER_ENGINE:-}" ]]; then
  if [[ "$HEADLESS" == "true" ]]; then
    # En headless priorizamos estabilidad del simulador para evitar caidas durante pick.
    export GZ_RENDER_ENGINE="ogre"
  else
    export GZ_RENDER_ENGINE="ogre"
  fi
fi
log "PANEL_GZ_GUI=${PANEL_GZ_GUI} HEADLESS=${HEADLESS}"

LAUNCH_GZ="false"
LAUNCH_RSP="false"
if [[ "${PANEL_START_STACK}" == "1" ]]; then
  LAUNCH_GZ="true"
  LAUNCH_RSP="true"
  PANEL_MANAGED="1"
fi

LAUNCH_BRIDGE="false"
if [[ "${PANEL_LAUNCH_BRIDGE}" == "1" ]]; then
  LAUNCH_BRIDGE="true"
fi

LAUNCH_ROS2_CONTROL="false"
if [[ "${PANEL_START_ROS2_CONTROL}" == "1" ]]; then
  LAUNCH_ROS2_CONTROL="true"
fi

LAUNCH_WORLD_TF="false"
if [[ "${PANEL_LAUNCH_WORLD_TF}" == "1" ]]; then
  LAUNCH_WORLD_TF="true"
fi

LAUNCH_SYSTEM_STATE="false"
if [[ "${PANEL_LAUNCH_SYSTEM_STATE}" == "1" ]]; then
  LAUNCH_SYSTEM_STATE="true"
fi
LAUNCH_MOVEIT="false"
if [[ "${PANEL_LAUNCH_MOVEIT}" == "1" ]]; then
  LAUNCH_MOVEIT="true"
fi

MOVEIT_MODE="$(echo "${PANEL_MOVEIT_MODE}" | tr '[:upper:]' '[:lower:]' | xargs)"
case "$MOVEIT_MODE" in
  auto|move_group|bridge)
    ;;
  *)
    log "PANEL_MOVEIT_MODE desconocido (${PANEL_MOVEIT_MODE}); usando auto"
    MOVEIT_MODE="auto"
    ;;
esac
if [[ "$MOVEIT_MODE" == "move_group" ]]; then
  LAUNCH_MOVEIT="true"
  export PANEL_AUTO_BRIDGE="0"
elif [[ "$MOVEIT_MODE" == "bridge" ]]; then
  LAUNCH_MOVEIT="false"
fi
export PANEL_MOVEIT_MODE="$MOVEIT_MODE"

runtime_sanity_check

if [[ "$LAUNCH_GZ" == "true" || "$LAUNCH_RSP" == "true" || "$LAUNCH_BRIDGE" == "true" || "$LAUNCH_ROS2_CONTROL" == "true" || "$LAUNCH_WORLD_TF" == "true" || "$LAUNCH_SYSTEM_STATE" == "true" ]]; then
  PANEL_MANAGED="1"
fi

LAUNCH_FILE_PKG="ur5_stack.launch.py"
LAUNCH_FILE_INSTALLED="$WS_DIR/install/ur5_bringup/share/ur5_bringup/$LAUNCH_FILE_PKG"
LAUNCH_FILE_SRC="$WS_DIR/src/ur5_bringup/launch/$LAUNCH_FILE_PKG"
LAUNCH_TARGET="ur5_bringup $LAUNCH_FILE_PKG"
if [[ -f "$LAUNCH_FILE_INSTALLED" ]]; then
  LAUNCH_TARGET="ur5_bringup $LAUNCH_FILE_PKG"
elif [[ -f "$LAUNCH_FILE_SRC" ]]; then
  LAUNCH_TARGET="$LAUNCH_FILE_SRC"
else
  err "no encuentro ur5_stack.launch.py en install o src"
  err " - $LAUNCH_FILE_INSTALLED"
  err " - $LAUNCH_FILE_SRC"
  err "Solución típica: colcon build --symlink-install"
  exit 1
fi

if [[ "${DEBUG_LOGS_TO_STDOUT}" == "1" ]]; then
  ros2 launch $LAUNCH_TARGET \
    headless:="$HEADLESS" \
    launch_panel:=true \
    launch_gazebo:="$LAUNCH_GZ" \
    launch_rsp:="$LAUNCH_RSP" \
    launch_bridge:="$LAUNCH_BRIDGE" \
    launch_ros2_control:="$LAUNCH_ROS2_CONTROL" \
    launch_world_tf:="$LAUNCH_WORLD_TF" \
    launch_system_state:="$LAUNCH_SYSTEM_STATE" \
    launch_moveit:="$LAUNCH_MOVEIT" \
    moveit_mode:="$MOVEIT_MODE" \
    strict_physics_mode:="$PANEL_STRICT_PHYSICS_MODE" \
    panel_managed:="$PANEL_MANAGED" \
    camera_required:="$PANEL_CAMERA_REQUIRED" \
    panel_auto_bridge:="${PANEL_AUTO_BRIDGE:-0}" \
    panel_auto_bridge_delay_ms:="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}"
  exit 0
fi

launch_log="$WS_DIR/log/ros2_launch.log"
mkdir -p "$WS_DIR/log"
pid_file="$WS_DIR/log/ros2_launch.pid"
PANEL_LOG_FILTER="${PANEL_LOG_FILTER:-0}"

if [[ "${PANEL_PROMPT_PID}" == "1" && -t 0 && -t 1 ]]; then
  read -r -p "[START_PANEL_V2] ¿Ejecutar en background y crear PID para stop_panel_v2.sh? [y/N] " reply
  case "${reply}" in
    y|Y|s|S)
      PANEL_WRITE_PID=1
      ;;
    *)
      PANEL_WRITE_PID=0
      ;;
  esac
fi

if [[ "${PANEL_WRITE_PID}" != "1" ]]; then
  setup_traps
fi

if [[ "${PANEL_WRITE_PID}" == "1" ]]; then
  rm -f "$pid_file" 2>/dev/null || true
  if [[ "$PANEL_LOG_FILTER" == "1" ]]; then
    log "PANEL_LOG_FILTER ignorado en background (logs en $launch_log)."
  fi
  nohup ros2 launch $LAUNCH_TARGET \
    headless:="$HEADLESS" \
    launch_panel:=true \
    launch_gazebo:="$LAUNCH_GZ" \
    launch_rsp:="$LAUNCH_RSP" \
    launch_bridge:="$LAUNCH_BRIDGE" \
    launch_ros2_control:="$LAUNCH_ROS2_CONTROL" \
    launch_world_tf:="$LAUNCH_WORLD_TF" \
    launch_system_state:="$LAUNCH_SYSTEM_STATE" \
    launch_moveit:="$LAUNCH_MOVEIT" \
    moveit_mode:="$MOVEIT_MODE" \
    strict_physics_mode:="$PANEL_STRICT_PHYSICS_MODE" \
    panel_managed:="$PANEL_MANAGED" \
    camera_required:="$PANEL_CAMERA_REQUIRED" \
    panel_auto_bridge:="${PANEL_AUTO_BRIDGE:-0}" \
    panel_auto_bridge_delay_ms:="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}" \
    > "$launch_log" 2>&1 < /dev/null &
  launch_pid=$!
  disown "$launch_pid" 2>/dev/null || true
  echo "${launch_pid}" > "$pid_file"
  log "ros2 launch PID=${launch_pid} (pidfile: $pid_file)"
  log "Para detener: ./scripts/stop_panel_v2.sh"
  exit 0
fi

if [[ "${PANEL_AUTO_EXIT_ON_PANEL}" == "1" ]]; then
  if [[ "$PANEL_LOG_FILTER" == "1" ]]; then
    log "PANEL_LOG_FILTER ignorado con PANEL_AUTO_EXIT_ON_PANEL=1."
  fi
  ros2 launch $LAUNCH_TARGET \
    headless:="$HEADLESS" \
    launch_panel:=true \
    launch_gazebo:="$LAUNCH_GZ" \
    launch_rsp:="$LAUNCH_RSP" \
    launch_bridge:="$LAUNCH_BRIDGE" \
    launch_ros2_control:="$LAUNCH_ROS2_CONTROL" \
    launch_world_tf:="$LAUNCH_WORLD_TF" \
    launch_system_state:="$LAUNCH_SYSTEM_STATE" \
    launch_moveit:="$LAUNCH_MOVEIT" \
    moveit_mode:="$MOVEIT_MODE" \
    strict_physics_mode:="$PANEL_STRICT_PHYSICS_MODE" \
    panel_managed:="$PANEL_MANAGED" \
    camera_required:="$PANEL_CAMERA_REQUIRED" \
    panel_auto_bridge:="${PANEL_AUTO_BRIDGE:-0}" \
    panel_auto_bridge_delay_ms:="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}" \
    > >(tee "$launch_log") 2> >(tee -a "$launch_log" >&2) &
  launch_pid=$!
  panel_seen=0
  for _ in {1..120}; do
    if pgrep -f "panel_v2.py|ur5_qt_panel.*panel_v2" >/dev/null 2>&1; then
      panel_seen=1
      break
    fi
    sleep 0.5
  done
  while kill -0 "$launch_pid" >/dev/null 2>&1; do
    if [[ "$panel_seen" == "1" ]]; then
      if ! pgrep -f "panel_v2.py|ur5_qt_panel.*panel_v2" >/dev/null 2>&1; then
        log "panel cerrado; deteniendo ros2 launch..."
        kill -INT "$launch_pid" >/dev/null 2>&1 || true
        break
      fi
    fi
    sleep 0.5
  done
  wait "$launch_pid" 2>/dev/null || true
  exit 0
fi

if [[ "$PANEL_LOG_FILTER" == "1" ]]; then
  stdbuf -oL -eL ros2 launch $LAUNCH_TARGET \
    headless:="$HEADLESS" \
    launch_panel:=true \
    launch_gazebo:="$LAUNCH_GZ" \
    launch_rsp:="$LAUNCH_RSP" \
    launch_bridge:="$LAUNCH_BRIDGE" \
    launch_ros2_control:="$LAUNCH_ROS2_CONTROL" \
    launch_world_tf:="$LAUNCH_WORLD_TF" \
    launch_system_state:="$LAUNCH_SYSTEM_STATE" \
    launch_moveit:="$LAUNCH_MOVEIT" \
    moveit_mode:="$MOVEIT_MODE" \
    strict_physics_mode:="$PANEL_STRICT_PHYSICS_MODE" \
    panel_managed:="$PANEL_MANAGED" \
    camera_required:="$PANEL_CAMERA_REQUIRED" \
    panel_auto_bridge:="${PANEL_AUTO_BRIDGE:-0}" \
    panel_auto_bridge_delay_ms:="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}" \
    2>&1 \
    | tee "$launch_log" \
    | awk '
        /\[STARTUP\]/ || /\[BTN\]/ { print > "/dev/stderr"; fflush("/dev/stderr") }
        /\[ERROR\]/ || /Traceback/ || /Exception/ || /FATAL/ { print > "/dev/stderr"; fflush("/dev/stderr") }
      '
else
  stdbuf -oL -eL ros2 launch $LAUNCH_TARGET \
    headless:="$HEADLESS" \
    launch_panel:=true \
    launch_gazebo:="$LAUNCH_GZ" \
    launch_rsp:="$LAUNCH_RSP" \
    launch_bridge:="$LAUNCH_BRIDGE" \
    launch_ros2_control:="$LAUNCH_ROS2_CONTROL" \
    launch_world_tf:="$LAUNCH_WORLD_TF" \
    launch_system_state:="$LAUNCH_SYSTEM_STATE" \
    launch_moveit:="$LAUNCH_MOVEIT" \
    moveit_mode:="$MOVEIT_MODE" \
    strict_physics_mode:="$PANEL_STRICT_PHYSICS_MODE" \
    panel_managed:="$PANEL_MANAGED" \
    camera_required:="$PANEL_CAMERA_REQUIRED" \
    panel_auto_bridge:="${PANEL_AUTO_BRIDGE:-0}" \
    panel_auto_bridge_delay_ms:="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}" \
    2>&1 \
    | tee "$launch_log"
fi
