#!/usr/bin/env bash
# lanzar_panelv2.sh — Limpia residuales, lanza el stack completo y abre el panel.
# Uso: ./lanzar_panelv2.sh

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/agarre_ros2_ws"
LOG_DIR="$SCRIPT_DIR/historico"
mkdir -p "$LOG_DIR"

# ── Entorno gráfico ───────────────────────────────────────────────────────────
# Detectar headless por DISPLAY, no por SSH_CONNECTION (que puede estar
# definida en terminales locales que iniciaron sesión SSH previamente).
if [[ -n "${DISPLAY:-}" && "${PANEL_FORCE_OFFSCREEN:-0}" != "1" ]]; then
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

# ── Activar venv ──────────────────────────────────────────────────────────────
for venv_dir in \
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
export PANEL_START_STACK=0
export PANEL_AUTO_BRIDGE=1
export PANEL_MOVEIT_REQUIRED=1
export PANEL_MOVEIT_MODE=move_group
export PANEL_SKIP_CLEANUP=1
export PANEL_COLD_BOOT=0          # el stack ya está arriba: no matar al abrir panel
export PANEL_DIRECT_DEBUG_ROOT="$LOG_DIR"

# Timeouts de movimiento: sin estos el APPROACH_COARSE expira en 4.5s (insuficiente)
export PANEL_PICK_DEMO_MOVE_SEC=15
export PANEL_PICK_DEMO_STEP_TIMEOUT_EXTRA_SEC=60
export PANEL_TF_INIT_GRACE_SEC=300
export PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL=0.035
export PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL=0.035
export PANEL_PICK_DEMO_IK_SEED_JOINTS="0.0,-1.5708,0.0,-1.5708,0.0,-3.07"

# ── Lanzar stack en background ────────────────────────────────────────────────
STACK_LOG="$LOG_DIR/stack_manual_$(date +%Y%m%d_%H%M%S).log"
echo "[LAUNCH] Lanzando stack (Gazebo + MoveIt2 + controllers)..."
echo "[LAUNCH] Stack log: $STACK_LOG"

# En SSH/headless no hay GPU para renderizar cámaras; en local sí.
if [[ "${HEADLESS}" == "true" ]]; then
    EXTRA_LAUNCH_ARGS="headless:=true camera_required:=false"
    export PANEL_CAMERA_REQUIRED=0
else
    EXTRA_LAUNCH_ARGS="headless:=false"
fi

# shellcheck disable=SC2086
ros2 launch ur5_bringup ur5_stack.launch.py \
    $EXTRA_LAUNCH_ARGS \
    launch_panel:=false \
    launch_moveit:=true \
    moveit_mode:=move_group \
    >"$STACK_LOG" 2>&1 &
STACK_PID=$!
echo "[LAUNCH] Stack PID=$STACK_PID"

# Matar el stack cuando el script termine (cierre del panel, Ctrl+C o error)
trap 'echo "[LAUNCH] Cerrando stack PID=$STACK_PID..."; kill "$STACK_PID" 2>/dev/null || true; wait "$STACK_PID" 2>/dev/null || true; echo "[LAUNCH] Stack cerrado."' EXIT

# ── Esperar move_group ────────────────────────────────────────────────────────
MOVEIT_WAIT_SEC=300
MOVEIT_WAIT_START=$(date +%s)
echo "[LAUNCH] Esperando /move_group (timeout=${MOVEIT_WAIT_SEC}s)..."
until ros2 node list 2>/dev/null | grep -q "/move_group"; do
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
until ros2 topic list 2>/dev/null | grep -q "pose/info"; do
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
for NODE in "/gripper_attach_backend" "/world_tf_publisher"; do
    until ros2 node list 2>/dev/null | grep -q "$NODE"; do
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

# Extra grace: system_state_manager necesita tiempo para publicar GAZEBO_READY
sleep 5

# Matar zombis/detenidos antes de abrir el panel (PANEL_COLD_BOOT=0 omite esto en start_panel_v2.sh)
# Los zombis no se pueden matar directamente (ya están muertos); hay que matar al padre.
echo "[LAUNCH] Limpiando procesos zombis/detenidos..."
ps aux | awk '$8 ~ /T/ {print $2}' | xargs -r kill -9 2>/dev/null || true
ps -eo pid,ppid,stat | awk '$3 ~ /Z/ {print $2}' | sort -u | xargs -r kill -SIGCHLD 2>/dev/null || true

echo "[LAUNCH] Entorno listo. Abriendo panel (GZ_PARTITION=$GZ_PARTITION)..."

# ── Abrir panel ───────────────────────────────────────────────────────────────
cd "$WS_DIR"
./scripts/start_panel_v2.sh "$@"
