#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/test_moveit_regression.sh
# Contenido: regresion offline de PlanToPose, FJT directo y wiring MoveIt.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

set +u
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi
if [[ -f "$ROOT_DIR/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
fi
set -u

echo "[MOVEIT_REGRESSION] workspace=$ROOT_DIR"
python3 -m py_compile src/ur5_tools/ur5_tools/plan_to_pose_server.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/ur5_tools/test/test_plan_to_pose_server_refactor_t15.py \
  src/ur5_tools/test/test_plan_to_pose_moveit_direct.py \
  src/ur5_tools/test/test_fjt_direct_helpers.py \
  -q

echo "[MOVEIT_REGRESSION] OK"