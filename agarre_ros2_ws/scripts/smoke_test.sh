#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/smoke_test.sh
# Contenido: Smoke test rápido del workspace — sin ROS/Gazebo.
# Uso breve: ./scripts/smoke_test.sh [--fast]
#   --fast: omit colcon build, only run lint + unit tests.
#
# Devuelve 0 si todos los checks pasan, 1 en caso contrario.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

FAST=0
for arg in "$@"; do
  [[ "$arg" == "--fast" ]] && FAST=1
done

PASS=0
FAIL=0
ERRORS=()

_ok()   { echo "[SMOKE][OK]   $*"; PASS=$((PASS + 1)); }
_fail() { echo "[SMOKE][FAIL] $*"; FAIL=$((FAIL + 1)); ERRORS+=("$*"); }
_skip() { echo "[SMOKE][SKIP] $*"; }
_sep()  { echo ""; echo "--- $* ---"; }

echo "[SMOKE] workspace=$ROOT_DIR  fast=$FAST  $(date -Iseconds)"

# ---------------------------------------------------------------------------
# 1. Python parse check: all modules in ur5_qt_panel and ur5_tools
# ---------------------------------------------------------------------------
_sep "AST parse"
PY_FILES_QT=$(find src/ur5_qt_panel/ur5_qt_panel -name "*.py" 2>/dev/null | sort)
PY_FILES_UT=$(find src/ur5_tools/ur5_tools -name "*.py" 2>/dev/null | sort)
PY_FILES_BU=$(find src/ur5_bringup/launch -name "*.py" 2>/dev/null | sort)
parse_errors=0
for f in $PY_FILES_QT $PY_FILES_UT $PY_FILES_BU; do
  if ! python3 -c "import ast; ast.parse(open('$f').read())" 2>/dev/null; then
    _fail "Syntax error in $f"
    parse_errors=$((parse_errors + 1))
  fi
done
if [ "$parse_errors" -eq 0 ]; then
  count=$(echo "$PY_FILES_QT $PY_FILES_UT $PY_FILES_BU" | wc -w)
  _ok "All $count Python files parse cleanly"
fi

# ---------------------------------------------------------------------------
# 2. F401 unused imports (flake8)
# ---------------------------------------------------------------------------
_sep "F401 unused imports"
if command -v python3 -m flake8 >/dev/null 2>&1 || python3 -c "import flake8" 2>/dev/null; then
  f401_out=$(python3 -m flake8 \
    src/ur5_qt_panel/ur5_qt_panel \
    src/ur5_qt_panel/test \
    src/ur5_tools/ur5_tools \
    src/ur5_tools/test \
    src/ur5_bringup/launch \
    src/ur5_bringup/test \
    --select=F401 2>&1 || true)
  if [[ -n "$f401_out" ]]; then
    _fail "F401 violations:\n$f401_out"
  else
    _ok "Zero F401 unused import violations"
  fi
else
  _skip "flake8 not available"
fi

# ---------------------------------------------------------------------------
# 3. Unit tests — directo_geometry and directo_gate_evaluator (no ROS needed)
# ---------------------------------------------------------------------------
_sep "Unit tests (ur5_qt_panel — no ROS)"
ut1_ok=0
# Strip the ROS Python path so the conftest stubs don't clash with rclpy C
# extensions when this script is sourced from a ROS-active shell. Tests are
# designed to run without ROS, so the isolated PYTHONPATH is correct here.
if PYTHONPATH="src/ur5_qt_panel:src/ur5_tools" python3 -m pytest \
     src/ur5_qt_panel/test/test_directo_geometry.py \
     src/ur5_qt_panel/test/test_directo_gate_evaluator.py \
     src/ur5_qt_panel/test/test_panel_config.py \
     src/ur5_qt_panel/test/test_panel_utils.py \
     src/ur5_qt_panel/test/test_panel_settings_defaults.py \
     src/ur5_qt_panel/test/test_step_pipeline.py \
     src/ur5_qt_panel/test/test_panel_tfm_apply_gate.py \
     src/ur5_qt_panel/test/test_state_machine.py \
     src/ur5_qt_panel/test/test_panel_pick_demo_basket_confirmation.py \
     src/ur5_qt_panel/test/test_panel_pick_demo_direct_grasp_z.py \
     src/ur5_qt_panel/test/test_panel_pick_demo_live_pose.py \
     src/ur5_qt_panel/test/test_panel_pick_demo_transport_follow.py \
     src/ur5_qt_panel/test/test_panel_pick_object_moveit_init.py \
     src/ur5_qt_panel/test/test_pick_geometry.py \
     src/ur5_qt_panel/test/test_attach_gate_evaluator.py \
     src/ur5_qt_panel/test/test_ur5_kinematics.py \
     src/ur5_qt_panel/test/test_logging_utils.py \
     -q --tb=short 2>&1; then
  ut1_ok=1
fi

_sep "Unit tests (ur5_tools + ur5_bringup — no ROS)"
ut2_ok=0
if python3 -m pytest \
     src/ur5_tools/test/test_moveit_bridge_utils.py \
     src/ur5_tools/test/test_moveit_bridge_plan_success.py \
     src/ur5_tools/test/test_cycle_logger.py \
     src/ur5_tools/test/test_gripper_geometry.py \
     src/ur5_tools/test/test_system_state_machine.py \
     src/ur5_tools/test/test_param_utils.py \
     src/ur5_bringup/test/test_launch_helpers.py \
     -q --tb=short 2>&1; then
  ut2_ok=1
fi

if [ "$ut1_ok" -eq 1 ] && [ "$ut2_ok" -eq 1 ]; then
  _ok "All unit tests PASS"
else
  _fail "Unit tests FAIL — see output above"
fi

# ---------------------------------------------------------------------------
# 4. Import check for zero-dependency modules
# ---------------------------------------------------------------------------
_sep "Zero-dep import check"
PYTHONPATH="src/ur5_qt_panel:${PYTHONPATH:-}" \
  python3 -c "from ur5_qt_panel.directo_geometry import angle_shortest_diff_rad, pick_demo_target_semantics; print('directo_geometry OK')" \
  && _ok "directo_geometry imports without dependencies" \
  || _fail "directo_geometry failed to import"

# ---------------------------------------------------------------------------
# 5. Key file existence checks
# ---------------------------------------------------------------------------
_sep "File existence"
REQUIRED_FILES=(
  "src/ur5_qt_panel/ur5_qt_panel/directo_geometry.py"
  "src/ur5_qt_panel/ur5_qt_panel/directo_gate_evaluator.py"
  "src/ur5_qt_panel/ur5_qt_panel/panel_config.py"
  "src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py"
  "src/ur5_qt_panel/ur5_qt_panel/panel_v2.py"
  "src/ur5_qt_panel/ur5_qt_panel/step_pipeline_helpers.py"
  "src/ur5_qt_panel/ur5_qt_panel/ur5_kinematics.py"
  "src/ur5_tools/ur5_tools/gripper_geometry.py"
  "src/ur5_tools/ur5_tools/moveit_bridge_utils.py"
  "src/ur5_tools/ur5_tools/cycle_logger.py"
)
for f in "${REQUIRED_FILES[@]}"; do
  if [[ -f "$f" ]]; then
    _ok "exists: $f"
  else
    _fail "MISSING: $f"
  fi
done

# ---------------------------------------------------------------------------
# 6. colcon build (skip in --fast mode)
# ---------------------------------------------------------------------------
if ((FAST == 0)); then
  _sep "colcon build"
  if command -v colcon >/dev/null 2>&1; then
    if colcon build --symlink-install --packages-select ur5_qt_panel ur5_tools \
        2>&1 | tail -5; then
      _ok "colcon build OK"
    else
      _fail "colcon build FAIL"
    fi
  else
    _skip "colcon not in PATH"
  fi
else
  _skip "colcon build (--fast)"
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "========================================"
echo "[SMOKE] PASS=$PASS  FAIL=$FAIL"
if [ "$FAIL" -gt 0 ]; then
  echo "[SMOKE] FAILURES:"
  for err in "${ERRORS[@]}"; do
    echo "  - $err"
  done
  echo "[SMOKE] RESULT: FAILED"
  exit 1
fi
echo "[SMOKE] RESULT: OK  $(date -Iseconds)"
