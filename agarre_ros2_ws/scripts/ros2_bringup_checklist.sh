#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/ros2_bringup_checklist.sh
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
set -Eeuo pipefail

WS_DIR="${WS_DIR:-$HOME/TFM/agarre_ros2_ws}"
LOG_DIR="$WS_DIR/log/checklist"
TS="$(date +%Y%m%d_%H%M%S)"

mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/bringup_checklist_${TS}.log"

{
  echo "[CHECKLIST] $(date)"
  echo "WS_DIR=$WS_DIR"
  echo

  echo "[1] Source ROS2 + workspace"
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
    export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-$(command -v python3 || true)}"
    set +u
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    set -u
  else
    echo "[WARN] /opt/ros/jazzy/setup.bash no encontrado"
  fi
  if [[ -f "$WS_DIR/install/setup.bash" ]]; then
    export COLCON_TRACE="${COLCON_TRACE:-}"
    set +u
    # shellcheck source=/dev/null
    source "$WS_DIR/install/setup.bash"
    set -u
  else
    echo "[WARN] install/setup.bash no encontrado (ejecuta colcon build)"
  fi

  echo
  echo "[2] Validate panel flow"
  if [[ -x "$WS_DIR/scripts/validate_panel_flow.sh" ]]; then
    bash "$WS_DIR/scripts/validate_panel_flow.sh" || true
  else
    echo "[WARN] validate_panel_flow.sh no ejecutable"
  fi

  echo
  echo "[3] System state"
  if command -v ros2 >/dev/null 2>&1; then
    timeout 5s ros2 topic echo /system_state --once || true
    timeout 5s ros2 topic echo /system_diag --once || true
  else
    echo "[WARN] ros2 no disponible en PATH"
  fi

  echo
  echo "[4] Probes (opcionales, no bloqueantes)"
  if command -v ros2 >/dev/null 2>&1; then
    timeout 5s ros2 run ur5_tools tf_probe --ros-args -p log_every_sec:=1.0 &
    PID_TF=$!
    sleep 2
    kill "$PID_TF" >/dev/null 2>&1 || true

    timeout 5s ros2 run ur5_tools clock_probe --ros-args -p warn_every_sec:=1.0 &
    PID_CLK=$!
    sleep 2
    kill "$PID_CLK" >/dev/null 2>&1 || true
  fi

  echo
  echo "[5] Smoke test TFM"
  if [[ -x "$WS_DIR/scripts/tfm_smoketest.py" ]]; then
    python3 "$WS_DIR/scripts/tfm_smoketest.py" || true
  else
    echo "[WARN] tfm_smoketest.py no ejecutable"
  fi

  echo
  echo "[DONE] Checklist completado"
} 2>&1 | tee "$LOG"

echo "[LOG] $LOG"
