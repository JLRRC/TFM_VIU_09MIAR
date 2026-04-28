#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/fresh_build.sh
# Contenido: Rebuild limpio para evitar el modo de fallo "install desincronizado".
# Uso breve:
#   ./scripts/fresh_build.sh                   # rebuild ur5_qt_panel + ur5_tools (más editados)
#   ./scripts/fresh_build.sh ur5_qt_panel      # rebuild solo el paquete indicado
#   ./scripts/fresh_build.sh --all             # rebuild todos los paquetes
#   FRESH_BUILD_KILL=1 ./scripts/fresh_build.sh   # mata stack/panel antes de rebuilear
#
# Por qué existe: colcon con detección incremental puede dejar el install/
# obsoleto frente a src/ aunque "el build vaya OK" (los mtimes no detectan
# cambios). Con esta sesión perdimos > 1 h por eso (world_tf_publisher y
# panel_tf_discovery quedaron viejos en install y la simulación corría con
# bugs ya arreglados en src). Este script borra build/ + install/ del paquete
# y reconstruye desde cero, garantizando que install == src.

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

# ── Argumentos ─────────────────────────────────────────────────────────────
PACKAGES=()
BUILD_ALL=0
for arg in "$@"; do
  case "$arg" in
    --all)
      BUILD_ALL=1
      ;;
    --help|-h)
      sed -n '2,16p' "$0" | sed 's/^# //'
      exit 0
      ;;
    *)
      PACKAGES+=("$arg")
      ;;
  esac
done

# Default: ur5_qt_panel y ur5_tools (los paquetes que más se editan).
if [[ "$BUILD_ALL" == "0" && "${#PACKAGES[@]}" == "0" ]]; then
  PACKAGES=(ur5_qt_panel ur5_tools)
fi

echo "[FRESH] workspace=$ROOT_DIR"
if [[ "$BUILD_ALL" == "1" ]]; then
  echo "[FRESH] modo=ALL"
else
  echo "[FRESH] paquetes=${PACKAGES[*]}"
fi

# ── Matar procesos del stack si lo piden (FRESH_BUILD_KILL=1) ──────────────
if [[ "${FRESH_BUILD_KILL:-0}" == "1" ]]; then
  echo "[FRESH] matando stack/panel previos..."
  pkill -INT -f "panel_v2|gz sim|move_group|gripper_attach|world_tf_publisher|release_objects_service|gz_pose_bridge|controller_bootstrap|parameter_bridge|robot_state_publisher|planning_scene_sync|gz_ros_control_guard|system_state_manager|pytest.*pick" 2>/dev/null || true
  sleep 2
  pkill -9   -f "panel_v2|gz sim|move_group|gripper_attach|world_tf_publisher|release_objects_service|gz_pose_bridge|controller_bootstrap|parameter_bridge|robot_state_publisher|planning_scene_sync|gz_ros_control_guard|system_state_manager|pytest.*pick" 2>/dev/null || true
  sleep 1
fi

# ── Borrar install/ y build/ de los paquetes seleccionados ─────────────────
if [[ "$BUILD_ALL" == "1" ]]; then
  echo "[FRESH] borrando install/ y build/ enteros..."
  rm -rf install build
else
  for pkg in "${PACKAGES[@]}"; do
    if [[ -d "install/$pkg" || -d "build/$pkg" ]]; then
      echo "[FRESH] borrando install/$pkg + build/$pkg"
      rm -rf "install/$pkg" "build/$pkg"
    else
      echo "[FRESH] $pkg: sin install/build previos (OK)"
    fi
  done
fi

# ── Source ROS ─────────────────────────────────────────────────────────────
set +u
# shellcheck disable=SC1091
[[ -f /opt/ros/jazzy/setup.bash ]] && source /opt/ros/jazzy/setup.bash || {
  echo "[FRESH][ERROR] /opt/ros/jazzy/setup.bash no existe" >&2
  exit 1
}
set -u

# ── Build ───────────────────────────────────────────────────────────────────
echo "[FRESH] colcon build $(date -Iseconds)"
if [[ "$BUILD_ALL" == "1" ]]; then
  colcon build --event-handlers console_cohesion+
else
  colcon build --packages-select "${PACKAGES[@]}" --event-handlers console_cohesion+
fi

# ── Verificación post-build: install != vacío para cada paquete ────────────
FAIL=0
for pkg in "${PACKAGES[@]}"; do
  if [[ "$BUILD_ALL" == "1" || ${#PACKAGES[@]} -gt 0 ]]; then
    inst="install/$pkg"
    if [[ ! -d "$inst" ]]; then
      echo "[FRESH][FAIL] $pkg: install/ no se generó"
      FAIL=$((FAIL + 1))
    else
      echo "[FRESH][OK]   $pkg installed at $inst"
    fi
  fi
done

if [[ "$FAIL" -gt 0 ]]; then
  echo "[FRESH] resultado=FAIL ($FAIL paquete(s))"
  exit 1
fi

echo "[FRESH] resultado=OK"
echo "[FRESH] siguiente paso: source install/setup.bash && ./lanzar_panelc2.sh"
