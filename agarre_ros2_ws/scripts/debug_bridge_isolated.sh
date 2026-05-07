#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/debug_bridge_isolated.sh
# Contenido: script de diagnóstico aislado del bug bridge MoveIt-controller.
# Uso breve:
#   bash agarre_ros2_ws/scripts/debug_bridge_isolated.sh
#
# Reproduce el bug bridge SIN orchestrator y SIN panel. Solo:
#   1. Lanza el stack mínimo (Gazebo + MoveIt + controller).
#   2. Espera a que MoveIt esté ready.
#   3. Envía 5 goals consecutivos a /move_action con poses simples.
#   4. Mide tiempo y resultado de cada goal.
#
# Esto permite reproducir el flakiness sin las capas del orchestrator
# (sim_time vs wall_time), aislando la causa raíz al bridge MoveIt-controller.

set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$WS_DIR/log"
DEBUG_LOG="$LOG_DIR/bridge_debug_$(date +%Y%m%d_%H%M%S).log"

echo "=== Bridge Debug Aislado — $(date) ==="
echo "WS_DIR: $WS_DIR"
echo "LOG: $DEBUG_LOG"
echo ""

# --- Cleanup previo ---
echo "[1/5] Limpiando procesos previos..."
pkill -9 -f "gz sim|move_group|controller_manager|panel_v2|pick_orchestrator|tf_geometry|object_pose|gripper_attach|world_tf|gz_pose|plan_to_pose|robot_state_publisher|parameter_bridge|ros2 launch" 2>/dev/null || true
sleep 5
rm -rf "$WS_DIR/log/gz_models" "$WS_DIR/log/ros2_launch.log" 2>/dev/null || true

# --- Source ROS env ---
echo "[2/5] Cargando ROS Jazzy + workspace..."
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"

# --- Lanzar stack mínimo en background ---
echo "[3/5] Lanzando stack mínimo (Gazebo + MoveIt + controller)..."
ros2 launch ur5_bringup ur5_stack.launch.py \
    headless:=true \
    launch_panel:=false \
    launch_gazebo:=true \
    launch_rsp:=true \
    launch_bridge:=false \
    launch_ros2_control:=false \
    launch_world_tf:=true \
    launch_system_state:=true \
    launch_release_service:=false \
    launch_attach_backend:=false \
    launch_scene_sync:=false \
    launch_tf_geometry_service:=false \
    launch_moveit:=true \
    launch_moveit_bridge:=false \
    moveit_mode:=move_group \
    panel_managed:=0 \
    camera_required:=0 \
    > "$DEBUG_LOG" 2>&1 &
LAUNCH_PID=$!
echo "  Launch PID: $LAUNCH_PID"

# --- Esperar a que MoveIt esté ready ---
echo "[4/5] Esperando a que /move_action esté disponible..."
DEADLINE=$(($(date +%s) + 120))
while [ "$(date +%s)" -lt "$DEADLINE" ]; do
    if ros2 action list 2>/dev/null | grep -q "/move_action"; then
        echo "  /move_action listo."
        break
    fi
    sleep 2
done

if ! ros2 action list 2>/dev/null | grep -q "/move_action"; then
    echo "ERROR: /move_action no apareció en 120s"
    kill -9 "$LAUNCH_PID" 2>/dev/null || true
    exit 1
fi

# Esperar a que el robot esté en home (joint_states publicando).
sleep 10

# --- Enviar 5 goals consecutivos ---
echo "[5/5] Enviando 5 goals consecutivos a /move_action..."
echo "" >> "$DEBUG_LOG"
echo "=== DEBUG GOALS START $(date) ===" >> "$DEBUG_LOG"

for i in 1 2 3 4 5; do
    echo ""
    echo "--- Goal $i/5 ---"
    GOAL_START=$(date +%s)
    cat > /tmp/move_goal.yaml <<EOF
{request: {group_name: 'manipulator',
  num_planning_attempts: 5,
  allowed_planning_time: 25.0,
  max_velocity_scaling_factor: 0.3,
  max_acceleration_scaling_factor: 0.3,
  goal_constraints: [{
    position_constraints: [{
      header: {frame_id: 'base_link'},
      link_name: 'rg2_tcp',
      target_point_offset: {x: 0.0, y: 0.0, z: 0.0},
      constraint_region: {
        primitives: [{type: 2, dimensions: [0.05]}],
        primitive_poses: [{position: {x: 0.4, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}]
      },
      weight: 1.0
    }]
  }]
}}
EOF
    timeout 120 ros2 action send_goal /move_action moveit_msgs/action/MoveGroup -f -p "$(cat /tmp/move_goal.yaml)" \
        2>&1 | tail -20 | tee -a "$DEBUG_LOG" || echo "Goal $i timeout/error" | tee -a "$DEBUG_LOG"
    GOAL_END=$(date +%s)
    GOAL_DUR=$((GOAL_END - GOAL_START))
    echo "Goal $i duración: ${GOAL_DUR}s"
    sleep 3
done

echo ""
echo "=== Cleanup ==="
kill -9 "$LAUNCH_PID" 2>/dev/null || true
pkill -9 -f "gz sim|move_group|controller_manager" 2>/dev/null || true

echo ""
echo "Log completo: $DEBUG_LOG"
echo ""
echo "=== ANÁLISIS ==="
echo "Si todos los goals completan en <60s con error_code=1: bridge funcionando."
echo "Si hay goals que tardan >60s o dan error_code != 1: bug bridge confirmado."
echo "Comparar duración entre goals: si la varianza es alta, hay flakiness."
