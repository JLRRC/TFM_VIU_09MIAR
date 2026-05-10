#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/run_tfm_regression_suite.sh
# Contenido: suite de regresion TFM para geometria, MoveIt y pick fisico opt-in.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

echo "[TFM_REGRESSION] workspace=$ROOT_DIR"
bash scripts/test_geometry_regression.sh
bash scripts/test_moveit_regression.sh
bash scripts/test_pick_physics_regression.sh
echo "[TFM_REGRESSION] OK"