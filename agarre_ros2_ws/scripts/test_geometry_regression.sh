#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/test_geometry_regression.sh
# Contenido: regresion estatica de geometria URDF/SDF y launch.
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

echo "[GEOMETRY_REGRESSION] workspace=$ROOT_DIR"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/ur5_tools/test/test_gripper_geometry.py \
  src/ur5_tools/test/test_urdf_sdf_parity.py \
  src/ur5_description/test/test_urdf_xacro_parses.py \
  src/ur5_bringup/test/test_launch_factories.py \
  -q

echo "[GEOMETRY_REGRESSION] OK"