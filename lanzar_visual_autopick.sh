#!/usr/bin/env bash
# =============================================================================
# lanzar_visual_autopick.sh
# Lanza visual_autopick_ros_ws completo: Gazebo + robot + controladores + panel simple.
# No depende de agarre_ros2_ws. Todo arranca con un solo launch.
#
# Logs en: visual_autopick_ros_ws/log/runtime/<timestamp>/
# =============================================================================

set -e

# =============================================================================
# CONFIGURACION
# =============================================================================
AUTOPICK_WS=/home/laboratorio/TFM/visual_autopick_ros_ws
USE_MOVEIT=false          # true para arrancar move_group (mas lento)
USE_ATTACH=false          # true para attach fisico al cerrar gripper

# =============================================================================
# Directorio de logs
# =============================================================================
TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
LOG_DIR=$AUTOPICK_WS/log/runtime/$TIMESTAMP
mkdir -p "$LOG_DIR"

LATEST_LINK=$AUTOPICK_WS/log/runtime/latest
rm -f "$LATEST_LINK"
ln -s "$LOG_DIR" "$LATEST_LINK"

LAUNCH_LOG=$LOG_DIR/launch.log
ln -sf "$LAUNCH_LOG" /tmp/visual_autopick_panel.log

# =============================================================================
# Colores
# =============================================================================
RED='\033[0;31m'; YEL='\033[1;33m'; GRN='\033[0;32m'; NC='\033[0m'
_info()    { echo -e "${GRN}[INFO]${NC}  $*" | tee -a "$LAUNCH_LOG"; }
_warn()    { echo -e "${YEL}[WARN]${NC}  $*" | tee -a "$LAUNCH_LOG"; }
_error()   { echo -e "${RED}[ERROR]${NC} $*" | tee -a "$LAUNCH_LOG"; }
_section() { echo ""
             echo "=========================================="
             echo "  $*"
             echo "=========================================="
             echo ""; }

_section "UR5 Visual Autopick — workspace independiente"
_info "Workspace : $AUTOPICK_WS"
_info "Log dir   : $LOG_DIR"
_info "Log live  : tail -f $LAUNCH_LOG"

# =============================================================================
# Limpieza previa del stack (siempre, sin confirmación)
# =============================================================================
_section "Limpieza previa del stack"
LIMPIAR_SCRIPT=/home/laboratorio/TFM/limpiar_stack.sh
if [ -x "$LIMPIAR_SCRIPT" ]; then
    "$LIMPIAR_SCRIPT" --force --quiet | tee -a "$LAUNCH_LOG"
else
    _warn "No se encontró $LIMPIAR_SCRIPT — omitiendo limpieza"
fi

# =============================================================================
# Source ROS 2
# =============================================================================
source /opt/ros/jazzy/setup.bash

# =============================================================================
# Verificar workspace compilado
# =============================================================================
if [ ! -f "$AUTOPICK_WS/install/setup.bash" ]; then
    _error "Workspace no compilado. Ejecuta primero:"
    echo "    cd $AUTOPICK_WS"
    echo "    source /opt/ros/jazzy/setup.bash"
    echo "    colcon build --symlink-install"
    exit 1
fi
source "$AUTOPICK_WS/install/setup.bash"
_info "Workspace sourced OK."

# =============================================================================
# INSTRUCCIONES
# =============================================================================
_section "Instrucciones"
echo "  Arrancando: Gazebo + UR5+RG2 + controladores + panel simple"
echo ""
echo "  Cuando el panel abra:"
echo "    - Espera que la camara muestre imagen (~10s)"
echo "    - Pulsa DIRECTO para pick directo por joints"
echo "    - Pulsa MOVEIT para pick via MoveIt (requiere USE_MOVEIT=true)"
echo ""
echo "  Control manual desde otra terminal:"
echo "    ros2 topic pub --once /visual_autopick/step_command std_msgs/msg/String \"{data: 'DIRECTO:HOME'}\""
echo "    ros2 topic pub --once /visual_autopick/step_command std_msgs/msg/String \"{data: 'DIRECTO:APPROACH'}\""
echo "    ros2 topic pub --once /visual_autopick/step_command std_msgs/msg/String \"{data: 'DIRECTO:DOWN'}\""
echo "    ros2 topic pub --once /visual_autopick/step_command std_msgs/msg/String \"{data: 'DIRECTO:CLOSE'}\""
echo "    ros2 topic pub --once /visual_autopick/step_command std_msgs/msg/String \"{data: 'DIRECTO:LIFT'}\""
echo ""
echo "  Log en tiempo real:"
echo "    tail -f $LAUNCH_LOG"
echo ""

# =============================================================================
# LANZAR
# =============================================================================
CMD="ros2 launch visual_autopick_bringup visual_autopick.launch.py \
  use_moveit:=$USE_MOVEIT \
  use_panel:=true \
  panel_mode:=autopick \
  use_attach:=$USE_ATTACH"

_info "Ejecutando: $CMD"
echo ""

exec $CMD 2>&1 | tee "$LAUNCH_LOG"
