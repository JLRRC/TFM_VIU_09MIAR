#!/usr/bin/env bash
# limpiar_stack.sh — Limpieza completa del stack ROS 2/Gazebo/MoveIt/panel
# Uso: ./limpiar_stack.sh [--force] [--quiet]
#   --force  : salta la confirmación interactiva
#   --quiet  : suprime salida de detalle por PID
#
# Seguro para ejecutar con el stack corriendo o ya caído.
# NO modifica código, URDF, SDF, launch ni configuración.
# NO borra logs ni reports.

set -euo pipefail

FORCE=0
QUIET=0
for arg in "$@"; do
  case "$arg" in
    --force) FORCE=1 ;;
    --quiet) QUIET=1 ;;
  esac
done

# ── colores ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; YEL='\033[1;33m'; GRN='\033[0;32m'; CYA='\033[0;36m'
BLD='\033[1m'; RST='\033[0m'

log()  { echo -e "${BLD}[CLEAN]${RST} $*"; }
ok()   { echo -e "${GRN}[OK]${RST}    $*"; }
warn() { echo -e "${YEL}[WARN]${RST}  $*"; }
info() { [[ $QUIET -eq 0 ]] && echo -e "${CYA}[INFO]${RST}  $*" || true; }

# ── helpers ───────────────────────────────────────────────────────────────────
count_pattern() { ps -eo cmd | grep -c "$1" 2>/dev/null | grep -v grep || echo 0; }

kill_pattern_term() {
  local pattern="$1"
  local pids
  pids=$(ps -eo pid,cmd | grep -E "$pattern" | grep -v grep | awk '{print $1}' || true)
  [[ -z "$pids" ]] && return 0
  for pid in $pids; do
    kill -TERM "$pid" 2>/dev/null && info "  TERM $pid ($pattern)" || true
  done
}

kill_pattern_kill() {
  local pattern="$1"
  local pids
  pids=$(ps -eo pid,cmd | grep -E "$pattern" | grep -v grep | awk '{print $1}' || true)
  [[ -z "$pids" ]] && return 0
  for pid in $pids; do
    kill -KILL "$pid" 2>/dev/null && info "  KILL $pid ($pattern)" || true
  done
}

# ── snapshot inicial ──────────────────────────────────────────────────────────
echo ""
echo -e "${BLD}╔══════════════════════════════════════════════════════════╗${RST}"
echo -e "${BLD}║        LIMPIEZA STACK ROS 2 / Gazebo / Panel             ║${RST}"
echo -e "${BLD}╚══════════════════════════════════════════════════════════╝${RST}"
echo ""
log "Fecha:   $(date)"
log "Host:    $(hostname)"
log "Uptime:  $(uptime | sed 's/.*up /up /')"
echo ""

# Contar instancias relevantes
N_PANEL=$(ps -eo cmd | grep -c "panel_v2" 2>/dev/null || echo 0)
N_GZ=$(ps -eo cmd | grep -c "gz sim" 2>/dev/null || echo 0)
N_CB=$(ps -eo cmd | grep -c "controller_bootstrap" 2>/dev/null || echo 0)
N_PSS=$(ps -eo cmd | grep -c "planning_scene_sync" 2>/dev/null || echo 0)
N_GPB=$(ps -eo cmd | grep -c "gz_pose_bridge" 2>/dev/null || echo 0)
N_MG=$(ps -eo cmd | grep -c "move_group" 2>/dev/null || echo 0)
N_ZOM=$(ps -eo stat | grep -c "^Z" 2>/dev/null || echo 0)

echo -e "  panel_v2              : ${BLD}${N_PANEL}${RST} instancias"
echo -e "  gz sim                : ${BLD}${N_GZ}${RST} instancias"
echo -e "  move_group            : ${BLD}${N_MG}${RST} instancias"
echo -e "  controller_bootstrap  : ${BLD}${N_CB}${RST} instancias"
echo -e "  planning_scene_sync   : ${BLD}${N_PSS}${RST} instancias"
echo -e "  gz_pose_bridge        : ${BLD}${N_GPB}${RST} instancias"
echo -e "  zombis (estado Z)     : ${BLD}${N_ZOM}${RST}"
echo ""

TOTAL=$((N_PANEL + N_GZ + N_CB + N_PSS + N_GPB + N_MG))
if [[ $TOTAL -eq 0 ]]; then
  ok "No hay procesos del stack activos. Sistema ya limpio."
  echo ""
  exit 0
fi

# ── confirmación ──────────────────────────────────────────────────────────────
if [[ $FORCE -eq 0 ]]; then
  echo -e "${YEL}Se cerrarán TODOS los procesos del stack ROS 2/Gazebo/panel.${RST}"
  read -r -p "  ¿Continuar? [s/N] " resp
  case "$resp" in
    [sS]|[sS][iI]) : ;;
    *) log "Cancelado por el usuario."; exit 0 ;;
  esac
  echo ""
fi

# ── FASE 1: SIGTERM a lanzadores raíz ─────────────────────────────────────────
log "FASE 1 — SIGTERM a lanzadores raíz..."

# lanzar_panelc2.sh / start_panel_v2.sh
kill_pattern_term "lanzar_panelc2"
kill_pattern_term "start_panel_v2"

# ros2 launch ur5_bringup (stack principal)
kill_pattern_term "ros2 launch ur5_bringup"
kill_pattern_term "ros2 launch.*ur5_stack"

# ros2 run panel
kill_pattern_term "ros2 run ur5_qt_panel"

sleep 3

# ── FASE 2: SIGTERM a nodos del workspace ─────────────────────────────────────
log "FASE 2 — SIGTERM a nodos del workspace..."

TERM_PATTERNS=(
  "panel_v2"
  "move_group"
  "robot_state_publisher"
  "controller_bootstrap"
  "planning_scene_sync"
  "gz_pose_bridge"
  "gripper_attach_backend"
  "tf_probe"
  "world_tf_publisher"
  "system_state_manager"
  "release_objects_service"
  "ur5_moveit_bridge"
  "parameter_bridge"
  "rviz2"
  "gz sim"
  "gz_sim"
)

for pat in "${TERM_PATTERNS[@]}"; do
  kill_pattern_term "$pat"
done

sleep 5

# ── FASE 3: SIGKILL a supervivientes ─────────────────────────────────────────
log "FASE 3 — SIGKILL a supervivientes..."

KILL_PATTERNS=(
  "panel_v2"
  "move_group"
  "robot_state_publisher"
  "controller_bootstrap"
  "planning_scene_sync"
  "gz_pose_bridge"
  "gripper_attach_backend"
  "tf_probe"
  "world_tf_publisher"
  "system_state_manager"
  "release_objects_service"
  "ur5_moveit_bridge"
  "parameter_bridge"
  "rviz2"
  "gz sim"
  "gz_sim"
  "gz-transport-topic"
  "gz-transport-service"
  "ur5_rsp"
  "ur5_bringup"
)

for pat in "${KILL_PATTERNS[@]}"; do
  kill_pattern_kill "$pat"
done

# tee pipes colgados del log
TPIDS=$(ps -eo pid,cmd | grep 'tee.*ros2_launch' | grep -v grep | awk '{print $1}' || true)
for pid in $TPIDS; do
  kill -KILL "$pid" 2>/dev/null && info "  KILL tee $pid" || true
done

# ros2 topic/hz/echo y service call colgados del stack
MISC_PIDS=$(ps -eo pid,cmd | grep -E "ros2 topic (echo|hz).*ur5_mesa|ros2 service call.*panel|ros2 topic hz /clock" \
  | grep -v grep | awk '{print $1}' || true)
for pid in $MISC_PIDS; do
  kill -KILL "$pid" 2>/dev/null && info "  KILL ros2 cli colgado $pid" || true
done

sleep 2

# ── FASE 4: verificación ──────────────────────────────────────────────────────
log "FASE 4 — Verificación..."
echo ""

REMAINING=$(ps -eo pid,ppid,stat,pcpu,cmd | grep -E \
  "panel_v2|gz sim|move_group|robot_state_publisher|controller_bootstrap|planning_scene_sync|gz_pose_bridge|gripper_attach_backend|tf_probe|world_tf_publisher|system_state_manager|release_objects_service|ur5_moveit_bridge|parameter_bridge|rviz2" \
  | grep -v grep || true)

ZOMBIES=$(ps -eo pid,ppid,stat,etimes,cmd | awk '$3 ~ /Z/ {print}' || true)

LOAD1=$(awk '{print $1}' /proc/loadavg)

echo -e "  Load 1m: ${BLD}${LOAD1}${RST}"
echo ""

if [[ -z "$REMAINING" ]]; then
  ok "Procesos del stack: NINGUNO"
else
  warn "Quedan procesos (puede ser normal si acaban de iniciarse):"
  echo "$REMAINING"
fi

if [[ -z "$ZOMBIES" ]]; then
  ok "Zombis: NINGUNO"
else
  warn "Zombis detectados:"
  echo "$ZOMBIES"
fi

echo ""

# ── veredicto final ───────────────────────────────────────────────────────────
if [[ -z "$REMAINING" && -z "$ZOMBIES" ]]; then
  echo -e "${GRN}${BLD}╔══════════════════════════════════════════════════════════╗${RST}"
  echo -e "${GRN}${BLD}║  LIMPIO — puedes relanzar el panel                       ║${RST}"
  echo -e "${GRN}${BLD}╚══════════════════════════════════════════════════════════╝${RST}"
  echo ""
  echo -e "  ${BLD}Comando para relanzar:${RST}"
  echo -e "    cd /home/laboratorio/TFM && ./lanzar_panelc2.sh"
else
  echo -e "${YEL}${BLD}╔══════════════════════════════════════════════════════════╗${RST}"
  echo -e "${YEL}${BLD}║  ATENCIÓN — quedan procesos residuales                   ║${RST}"
  echo -e "${YEL}${BLD}╚══════════════════════════════════════════════════════════╝${RST}"
  echo ""
  warn "Revisa la lista de arriba. Si persisten, considera reiniciar la sesión."
fi

echo ""
