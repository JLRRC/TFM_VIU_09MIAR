#!/usr/bin/env bash
# lanzar_panelc2.sh — entrypoint operativo canónico del panel.
# La repetición manual final de las pruebas debe ejecutarse con este launcher.
#
# Modos de uso:
#   ./lanzar_panelc2.sh                    Detección automática: fast-path si READY, cold-boot si no
#   PANEL_FORCE_COLD_BOOT=1 ./...          Siempre arrancar desde cero (mata todo, relanza stack)
#   PANEL_FAST_ONLY=1 ./...               Solo abrir panel si stack ya está READY; salir si no
#   PANEL_ALLOW_DEGRADED_STACK=1 ./...    Abrir panel aunque /system_state no sea READY
#   LAUNCH_STRICT_READY=1 ./...           En cold-boot: abortar si READY no llega a tiempo
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/agarre_ros2_ws"
LOG_DIR="$SCRIPT_DIR/historico"
mkdir -p "$LOG_DIR"

# ── Cargar ROS 2 para poder consultar estado del stack ───────────────────────
# shellcheck disable=SC1091
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash || true
# shellcheck disable=SC1090
[[ -f "$WS_DIR/install/setup.bash" ]] && source "$WS_DIR/install/setup.bash" || true

export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$WS_DIR/scripts/fastdds_no_shm.xml}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export RMW_FASTRTPS_USE_SHM="${RMW_FASTRTPS_USE_SHM:-0}"

PANEL_FORCE_COLD_BOOT="${PANEL_FORCE_COLD_BOOT:-0}"
PANEL_FAST_ONLY="${PANEL_FAST_ONLY:-0}"
PANEL_ALLOW_DEGRADED_STACK="${PANEL_ALLOW_DEGRADED_STACK:-0}"

# ── Helpers de detección de estado ───────────────────────────────────────────
stack_is_ready() {
  local state
  state="$(timeout 6s ros2 topic echo --once /system_state 2>/dev/null \
    | sed -n 's/^data: //p' | tr -d '"' | tr -d "'" | head -n1)"
  [[ "${state:-}" == "READY" ]]
}

critical_nodes_alive() {
  local nl
  nl="$(ros2 node list --no-daemon 2>/dev/null || true)"
  grep -q "/move_group" <<< "$nl" \
    && grep -q "/gripper_attach_backend" <<< "$nl" \
    && grep -q "/system_state_manager" <<< "$nl"
}

_setup_fast_panel_env() {
  local gz_file="$WS_DIR/log/gz_partition.txt"
  if [[ -f "$gz_file" ]]; then
    export GZ_PARTITION
    GZ_PARTITION="$(cat "$gz_file")"
    echo "[LAUNCH] GZ_PARTITION recuperado: $GZ_PARTITION"
  fi
  export PANEL_COLD_BOOT=0
  export PANEL_SKIP_CLEANUP=1
  export PANEL_START_STACK=0
  export PANEL_MANAGED=1
  export PANEL_AUTO_BRIDGE=1
  export PANEL_MOVEIT_REQUIRED=1
  export PANEL_MOVEIT_MODE=move_group
  export PANEL_LAUNCH_WORLD_TF=0
  export PANEL_LAUNCH_SYSTEM_STATE=0
  export PANEL_CANONICAL_ENTRYPOINT=1
  export PANEL_DIRECT_DEBUG_ROOT="$LOG_DIR"
}

echo "[LAUNCH] Detectando estado del stack..."

# ── PANEL_FORCE_COLD_BOOT: siempre arrancar desde cero ───────────────────────
if [[ "$PANEL_FORCE_COLD_BOOT" == "1" ]]; then
  echo "[LAUNCH] PANEL_FORCE_COLD_BOOT=1 — arrancando desde cero."
  exec "$SCRIPT_DIR/lanzar_panelv2.sh" "$@"
fi

# ── Fast path: stack ya READY → abrir panel directamente ─────────────────────
if stack_is_ready 2>/dev/null; then
  echo "[LAUNCH] Stack READY detectado — abriendo panel (fast path, sin reiniciar stack)."
  _setup_fast_panel_env
  exec "$WS_DIR/scripts/start_panel_v2.sh" "$@"
fi

# ── Degraded path: nodos críticos vivos pero sin READY ───────────────────────
if [[ "$PANEL_ALLOW_DEGRADED_STACK" == "1" ]] && critical_nodes_alive 2>/dev/null; then
  echo "[WARN]  /system_state no es READY pero los nodos críticos están vivos."
  echo "[WARN]  PANEL_ALLOW_DEGRADED_STACK=1 — abriendo panel con posibles limitaciones."
  _setup_fast_panel_env
  exec "$WS_DIR/scripts/start_panel_v2.sh" "$@"
fi

# ── PANEL_FAST_ONLY: solo abrir si ya está listo, nunca arrancar stack ────────
if [[ "$PANEL_FAST_ONLY" == "1" ]]; then
  echo "[ERROR] PANEL_FAST_ONLY=1 pero el stack no está READY. Saliendo sin lanzar nada."
  echo "[INFO]  Para arrancar desde cero: ./lanzar_panelc2.sh"
  echo "[INFO]  Para stack degradado:    PANEL_ALLOW_DEGRADED_STACK=1 ./lanzar_panelc2.sh"
  exit 1
fi

# ── Cold boot: stack no detectado → lanzar stack completo ────────────────────
echo "[LAUNCH] Stack no detectado — arrancando stack completo (cold boot)..."
exec "$SCRIPT_DIR/lanzar_panelv2.sh" "$@"
