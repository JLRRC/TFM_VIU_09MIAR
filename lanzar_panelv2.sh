#!/usr/bin/env bash
# lanzar_panelv2.sh — Implementación del lanzador canónico. El entrypoint para el usuario es ./lanzar_panelc2.sh.
# Uso recomendado: ./lanzar_panelc2.sh

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/agarre_ros2_ws"
LOG_DIR="$SCRIPT_DIR/historico"
RUNTIME_PROFILE="$WS_DIR/scripts/panel_runtime_validated.env"
mkdir -p "$LOG_DIR"

recover_local_display() {
  local pid env_file candidate_display candidate_xauth candidate_session
  local current_uid
  current_uid="$(id -u)"
  for pid in $(pgrep -u "$current_uid" -f 'gnome-shell|gnome-session-binary|gnome-terminal-server|ptyxis|tilix|terminator' 2>/dev/null); do
    env_file="/proc/$pid/environ"
    [[ -r "$env_file" ]] || continue
    candidate_display="$(tr '\0' '\n' < "$env_file" | sed -n 's/^DISPLAY=//p' | head -n1)"
    [[ -n "$candidate_display" ]] || continue
    candidate_xauth="$(tr '\0' '\n' < "$env_file" | sed -n 's/^XAUTHORITY=//p' | head -n1)"
    candidate_session="$(tr '\0' '\n' < "$env_file" | sed -n 's/^XDG_SESSION_TYPE=//p' | head -n1)"
    if env DISPLAY="$candidate_display" XAUTHORITY="${candidate_xauth:-${XAUTHORITY:-}}" xdpyinfo >/dev/null 2>&1; then
      export DISPLAY="$candidate_display"
      if [[ -n "$candidate_xauth" ]]; then
        export XAUTHORITY="$candidate_xauth"
      fi
      if [[ -n "$candidate_session" ]]; then
        export XDG_SESSION_TYPE="$candidate_session"
      fi
      echo "[LAUNCH] Recuperado display local DISPLAY=$DISPLAY desde PID=$pid"
      return 0
    fi
  done
  return 1
}

# ── Entorno gráfico ───────────────────────────────────────────────────────────
# Detectar headless por DISPLAY, no por SSH_CONNECTION (que puede estar
# definida en terminales locales que iniciaron sesión SSH previamente).
if [[ "${PANEL_FORCE_OFFSCREEN:-0}" != "1" ]] && { [[ -z "${DISPLAY:-}" ]] || ! xdpyinfo >/dev/null 2>&1; }; then
  recover_local_display || true
fi
if [[ -n "${DISPLAY:-}" && "${PANEL_FORCE_OFFSCREEN:-0}" != "1" ]] && xdpyinfo >/dev/null 2>&1; then
  export HEADLESS=false
  export PANEL_GZ_GUI=1
else
  export HEADLESS=true
  export PANEL_GZ_GUI=0
fi

if [[ "${HEADLESS}" == "true" && "${PANEL_FORCE_OFFSCREEN:-0}" != "1" && \
      "${QT_QPA_PLATFORM:-}" != "offscreen" ]]; then
  echo "[ERROR] No está definida la variable DISPLAY. Abre una terminal gráfica (no TTY/SSH)."
  echo "[INFO]  Alternativa sin GUI: export PANEL_FORCE_OFFSCREEN=1"
  exit 1
fi

export QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins
export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms

# Si el DISPLAY sigue sin ser usable, abortamos con un mensaje claro.
# El modo offscreen debe pedirse explícitamente con PANEL_FORCE_OFFSCREEN=1.
if [[ "${HEADLESS}" == "true" && "${PANEL_FORCE_OFFSCREEN:-0}" != "1" ]]; then
  echo "[ERROR] DISPLAY=${DISPLAY:-<vacío>} no es usable desde esta sesión."
  echo "[INFO]  Abre una terminal gráfica local del escritorio y ejecuta ./lanzar_panelc2.sh"
  echo "[INFO]  Alternativa explícita sin GUI: export PANEL_FORCE_OFFSCREEN=1"
  exit 1
fi

# ── Activar venv ──────────────────────────────────────────────────────────────
for venv_dir in \
  "/home/laboratorio/.venv-tfm" \
  "/home/laboratorio/TFM/agarre_inteligente/.venv-tfm" \
  "/home/laboratorio/TFM/agarre_inteligente/venv" \
  "/home/laboratorio/TFM/agarre_inteligente/.venv"; do
  if [[ -f "$venv_dir/bin/activate" ]]; then
    # shellcheck disable=SC1090
    source "$venv_dir/bin/activate"
    break
  fi
done

# ── Cargar ROS 2 y overlay del workspace ──────────────────────────────────────
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
fi
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$WS_DIR/install/setup.bash"
fi

# ── DDS: mismo perfil que los nodos del stack ─────────────────────────────────
# Los nodos (move_group, gz, controllers) arrancan con fastdds_no_shm.xml.
# Sin esta exportación, ros2 node list usa SHM y no descubre ningún nodo.
export FASTRTPS_DEFAULT_PROFILES_FILE="$WS_DIR/scripts/fastdds_no_shm.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_SHM=0

# ── Limpiar procesos residuales ───────────────────────────────────────────────
echo "[LAUNCH] Limpiando procesos residuales..."
# SIGTERM primero
pkill -f "gz sim"              2>/dev/null || true
pkill -f "gz_server"           2>/dev/null || true
pkill -f "ros_gz_bridge"       2>/dev/null || true
pkill -f "parameter_bridge"    2>/dev/null || true
pkill -f "ros2_control_node"   2>/dev/null || true
pkill -f "controller_manager"  2>/dev/null || true
pkill -f "spawner"             2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "gripper_attach_backend" 2>/dev/null || true
pkill -f "move_group"          2>/dev/null || true
pkill -f "world_tf_publisher"  2>/dev/null || true
pkill -f "system_state_manager" 2>/dev/null || true
pkill -f "ur5_moveit_bridge"   2>/dev/null || true
pkill -f "start_panel_v2"      2>/dev/null || true
pkill -f "pick_demo_panel"     2>/dev/null || true
pkill -f "ur5_stack"           2>/dev/null || true
pkill -f "run_directo_button_offscreen" 2>/dev/null || true
pkill -f "pick_demo_panel"     2>/dev/null || true
sleep 3
# SIGKILL para procesos que sobrevivan (incluidos los suspendidos con Ctrl+Z)
pkill -9 -f "gz sim"           2>/dev/null || true
pkill -9 -f "gz_server"        2>/dev/null || true
pkill -9 -f "move_group"       2>/dev/null || true
pkill -9 -f "gripper_attach_backend" 2>/dev/null || true
pkill -9 -f "world_tf_publisher" 2>/dev/null || true
pkill -9 -f "system_state_manager" 2>/dev/null || true
pkill -9 -f "ur5_moveit_bridge" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge"    2>/dev/null || true
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
echo "[LAUNCH] Limpieza completada"

# ── GZ_PARTITION único para esta sesión ──────────────────────────────────────
export GZ_PARTITION="ur5pro_manual_$(date +%s)"
echo "[LAUNCH] GZ_PARTITION=$GZ_PARTITION"
mkdir -p "$WS_DIR/log"
echo "$GZ_PARTITION" > "$WS_DIR/log/gz_partition.txt"

# ── Variables exportadas al panel ─────────────────────────────────────────────
export PANEL_MANAGED=1
export PANEL_CANONICAL_ENTRYPOINT=1
export PANEL_START_STACK=0
export PANEL_AUTO_BRIDGE=1
export PANEL_MOVEIT_REQUIRED=1
export PANEL_MOVEIT_MODE=move_group
export PANEL_SKIP_CLEANUP=1
export PANEL_COLD_BOOT=0          # el stack ya está arriba: no matar al abrir panel
export PANEL_DIRECT_DEBUG_ROOT="$LOG_DIR"
# El stack lanza world_tf y system_state: no duplicar en el panel.
export PANEL_LAUNCH_WORLD_TF=0
export PANEL_LAUNCH_SYSTEM_STATE=0

if [[ ! -f "$RUNTIME_PROFILE" ]]; then
  echo "[ERROR] Falta el perfil runtime validado: $RUNTIME_PROFILE"
  exit 1
fi
# shellcheck disable=SC1090
source "$RUNTIME_PROFILE"
export PANEL_RUNTIME_PROFILE_PATH="$RUNTIME_PROFILE"
echo "[LAUNCH] Perfil runtime validado: $PANEL_RUNTIME_VALIDATED_PROFILE ($RUNTIME_PROFILE)"

# ── Lanzar stack en background ────────────────────────────────────────────────
STACK_LOG="$LOG_DIR/stack_manual_$(date +%Y%m%d_%H%M%S).log"
echo "[LAUNCH] Lanzando stack (Gazebo + MoveIt2 + controllers)..."
echo "[LAUNCH] Stack log: $STACK_LOG"

# En SSH/headless no hay GPU para renderizar cámaras; en local sí.
if [[ "${HEADLESS}" == "true" ]]; then
    EXTRA_LAUNCH_ARGS="headless:=true camera_required:=false"
    export PANEL_CAMERA_REQUIRED=0
else
    # FIX: forzar headless:=true para Gazebo aunque tengamos display local.
    # La GUI de Gazebo (gz-4) falla con QSGRenderLoop::handleContextCreationFailure
    # porque Qt5 Quick no puede crear contexto OpenGL en este entorno.
    # El servidor de Gazebo (gz-3) corre correctamente en headless; el panel
    # accede a la simulación via ros_gz_bridge, que no depende de la GUI de gz.
    EXTRA_LAUNCH_ARGS="headless:=true"
fi

# shellcheck disable=SC2086
ros2 launch ur5_bringup ur5_stack.launch.py \
    $EXTRA_LAUNCH_ARGS \
    launch_panel:=false \
    launch_moveit:=true \
    moveit_mode:=move_group \
    launch_moveit_bridge:=false \
    >"$STACK_LOG" 2>&1 &
STACK_PID=$!
echo "[LAUNCH] Stack PID=$STACK_PID"

# Matar el stack cuando el script termine (cierre del panel, Ctrl+C o error)
trap 'echo "[LAUNCH] Cerrando stack PID=$STACK_PID..."; kill "$STACK_PID" 2>/dev/null || true; wait "$STACK_PID" 2>/dev/null || true; echo "[LAUNCH] Stack cerrado."' EXIT

# ── Esperar move_group ────────────────────────────────────────────────────────
# FIX: reiniciar el daemon ROS 2 para evitar caché stale de sesiones anteriores
# que impide detectar /move_group aunque el proceso esté vivo.
echo "[LAUNCH] Reiniciando daemon ROS 2 (flush caché de nodos)..."
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start 2>/dev/null || true
sleep 2

MOVEIT_WAIT_SEC=300
MOVEIT_WAIT_START=$(date +%s)
echo "[LAUNCH] Esperando /move_group (timeout=${MOVEIT_WAIT_SEC}s)..."
until ros2 node list --no-daemon 2>/dev/null | grep -q "/move_group"; do
    NOW=$(date +%s)
    ELAPSED=$(( NOW - MOVEIT_WAIT_START ))
    if (( ELAPSED >= MOVEIT_WAIT_SEC )); then
        echo "[ERROR] Timeout: /move_group no apareció en ${MOVEIT_WAIT_SEC}s. Abortando."
        exit 1
    fi
    sleep 3
done
echo "[LAUNCH] /move_group listo ($(( $(date +%s) - MOVEIT_WAIT_START ))s)"

# ── Esperar pose/info activo ──────────────────────────────────────────────────
# Usamos ros2 topic list (no bloqueante) en lugar de ros2 topic hz
WORLD="${GZ_WORLD:-ur5_mesa_objetos}"
POSE_TOPIC="/world/${WORLD}/pose/info"
GAZEBO_WAIT_SEC=120
GAZEBO_WAIT_START=$(date +%s)
echo "[LAUNCH] Esperando $POSE_TOPIC en ros2 topic list (timeout=${GAZEBO_WAIT_SEC}s)..."
until ros2 topic list --no-daemon 2>/dev/null | grep -q "pose/info"; do
    NOW=$(date +%s)
    ELAPSED=$(( NOW - GAZEBO_WAIT_START ))
    if (( ELAPSED >= GAZEBO_WAIT_SEC )); then
        echo "[WARN]  Gazebo pose/info sin datos tras ${GAZEBO_WAIT_SEC}s — continuando"
        break
    fi
    sleep 5
done
echo "[LAUNCH] Gazebo pose/info activo ($(( $(date +%s) - GAZEBO_WAIT_START ))s)"

# ── Verificar nodos críticos ──────────────────────────────────────────────────
NODE_WAIT_SEC=30
NODE_WAIT_START=$(date +%s)
echo "[LAUNCH] Verificando nodos críticos..."
for NODE in "/gripper_attach_backend" "/world_tf_publisher" "/system_state_manager"; do
    until ros2 node list --no-daemon 2>/dev/null | grep -q "$NODE"; do
        NOW=$(date +%s)
        ELAPSED=$(( NOW - NODE_WAIT_START ))
        if (( ELAPSED >= NODE_WAIT_SEC )); then
            echo "[ERROR] Nodo $NODE no apareció tras ${NODE_WAIT_SEC}s. Abortando."
            exit 1
        fi
        sleep 2
    done
    echo "[LAUNCH] $NODE OK"
done

# Esperar a que system_state_manager declare READY tras pasar TF, cámaras y self-check geométrico.
# Con LAUNCH_STRICT_READY=1: abortar si READY no llega (comportamiento original).
# Sin LAUNCH_STRICT_READY: advertir y continuar si los nodos críticos siguen vivos.
LAUNCH_STRICT_READY="${LAUNCH_STRICT_READY:-0}"
STATE_WAIT_SEC=180
STATE_WAIT_START=$(date +%s)
echo "[LAUNCH] Esperando /system_state=READY (timeout=${STATE_WAIT_SEC}s, strict=${LAUNCH_STRICT_READY})..."
while true; do
    if ! kill -0 "$STACK_PID" 2>/dev/null; then
        echo "[ERROR] El stack terminó antes de publicar READY. Revisa $STACK_LOG"
        exit 1
    fi
    STATE_SAMPLE="$(
        timeout 6s ros2 topic echo --once /system_state 2>/dev/null \
        | sed -n 's/^data: //p' \
        | tr -d '"' \
        | tr -d "'" \
        | head -n1
    )"
    if [[ "$STATE_SAMPLE" == "READY" ]]; then
        break
    fi
    if [[ "$STATE_SAMPLE" == "ERROR_FATAL" ]]; then
        echo "[ERROR] system_state_manager publicó ERROR_FATAL. Revisa $STACK_LOG"
        exit 1
    fi
    NOW=$(date +%s)
    ELAPSED=$(( NOW - STATE_WAIT_START ))
    if (( ELAPSED >= STATE_WAIT_SEC )); then
        if [[ "$LAUNCH_STRICT_READY" == "1" ]]; then
            echo "[ERROR] LAUNCH_STRICT_READY=1: timeout esperando /system_state=READY."
            echo "[ERROR] Último estado='${STATE_SAMPLE:-none}'. Revisa $STACK_LOG"
            exit 1
        fi
        echo "[WARN]  Timeout esperando /system_state=READY (${STATE_WAIT_SEC}s)."
        echo "[WARN]  Último estado='${STATE_SAMPLE:-none}'. Los nodos críticos están vivos."
        echo "[WARN]  Abriendo panel en estado degradado. Log: $STACK_LOG"
        echo "[WARN]  Para abortar en este caso: export LAUNCH_STRICT_READY=1"
        break
    fi
    sleep 2
done
echo "[LAUNCH] /system_state listo ($(( $(date +%s) - STATE_WAIT_START ))s, estado='${STATE_SAMPLE:-READY}')"

# Matar zombis/detenidos antes de abrir el panel (PANEL_COLD_BOOT=0 omite esto en start_panel_v2.sh)
# Los zombis no se pueden matar directamente (ya están muertos); hay que matar al padre.
echo "[LAUNCH] Limpiando procesos zombis/detenidos..."
ps aux | awk '$8 ~ /T/ {print $2}' | xargs -r kill -9 2>/dev/null || true
ps -eo pid,ppid,stat | awk '$3 ~ /Z/ {print $2}' | sort -u | xargs -r kill -SIGCHLD 2>/dev/null || true

echo "[LAUNCH] Entorno listo. Abriendo panel (GZ_PARTITION=$GZ_PARTITION)..."

# ── Abrir panel ───────────────────────────────────────────────────────────────
cd "$WS_DIR"
./scripts/start_panel_v2.sh "$@"
