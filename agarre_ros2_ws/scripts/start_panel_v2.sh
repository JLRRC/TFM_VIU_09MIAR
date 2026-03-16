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
: "${RMW_IMPLEMENTATION:=rmw_fastrtps_cpp}"
export RMW_IMPLEMENTATION
export PANEL_CONTROLLER_MANAGER

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
  export LIBGL_ALWAYS_SOFTWARE=0
fi
export QT_XCB_GL_INTEGRATION=none
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
export PANEL_CRITICAL_POSE_TIMEOUT_SEC="${PANEL_CRITICAL_POSE_TIMEOUT_SEC:-60.0}"
export PANEL_CRITICAL_CLOCK_TIMEOUT_SEC="${PANEL_CRITICAL_CLOCK_TIMEOUT_SEC:-5.0}"
export PANEL_PICK_OBJECT_EXTRA_DOWN_Z="${PANEL_PICK_OBJECT_EXTRA_DOWN_Z:-0.060}"
export PANEL_PICK_OBJECT_CONTACT_DOWN_Z_M="${PANEL_PICK_OBJECT_CONTACT_DOWN_Z_M:-0.060}"
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

# ESTRATEGIA: Headless-rendering SIEMPRE para máxima estabilidad
HEADLESS="true"
if [[ "${PANEL_GZ_GUI:-0}" == "1" ]]; then
  HEADLESS="false"
fi

if [[ -z "${GZ_RENDER_ENGINE:-}" ]]; then
  if [[ "$HEADLESS" == "true" ]]; then
    export GZ_RENDER_ENGINE="ogre2"
  else
    export GZ_RENDER_ENGINE="ogre"
  fi
fi

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
    panel_managed:="$PANEL_MANAGED" \
    panel_auto_bridge:="${PANEL_AUTO_BRIDGE:-0}" \
    panel_auto_bridge_delay_ms:="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}" \
    > "$launch_log" 2>&1 &
  launch_pid=$!
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
    panel_managed:="$PANEL_MANAGED" \
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
    panel_managed:="$PANEL_MANAGED" \
    camera_required:="$PANEL_CAMERA_REQUIRED" \
    panel_auto_bridge:="${PANEL_AUTO_BRIDGE:-0}" \
    panel_auto_bridge_delay_ms:="${PANEL_AUTO_BRIDGE_DELAY_MS:-1200}" \
    2>&1 \
    | tee "$launch_log"
fi
