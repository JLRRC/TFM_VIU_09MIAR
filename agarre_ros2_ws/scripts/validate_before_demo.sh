#!/usr/bin/env bash
# Ruta/archivo: agarre_ros2_ws/scripts/validate_before_demo.sh
# Contenido: Checklist pre-demo — verifica que el stack está listo para ejecutar DIRECTO.
# Uso breve: ./scripts/validate_before_demo.sh
#   Requiere ROS 2 sourced y stack arrancado (Gazebo + MoveIt + panel).
#
# Devuelve 0 si todos los checks OBLIGATORIOS pasan, 1 en caso contrario.
set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PASS=0
FAIL=0
WARN=0
ERRORS=()
WARNINGS=()

_ok()   { echo "[PRE_DEMO][OK]   $*"; PASS=$((PASS + 1)); }
_fail() { echo "[PRE_DEMO][FAIL] $*"; FAIL=$((FAIL + 1)); ERRORS+=("$*"); }
_warn() { echo "[PRE_DEMO][WARN] $*"; WARN=$((WARN + 1)); WARNINGS+=("$*"); }
_sep()  { echo ""; echo "--- $* ---"; }

echo "[PRE_DEMO] workspace=$ROOT_DIR  $(date -Iseconds)"

# ---------------------------------------------------------------------------
# 0. Smoke test (static checks, no ROS needed)
# ---------------------------------------------------------------------------
_sep "Smoke test (static)"
if bash "$ROOT_DIR/scripts/smoke_test.sh" --fast > /tmp/smoke_test_out.txt 2>&1; then
  _ok "smoke_test.sh --fast PASS"
else
  _fail "smoke_test.sh --fast FAIL — run: bash scripts/smoke_test.sh --fast"
  tail -20 /tmp/smoke_test_out.txt
fi

# ---------------------------------------------------------------------------
# 1. ROS 2 environment
# ---------------------------------------------------------------------------
_sep "ROS 2 environment"
if [[ -z "${ROS_DISTRO:-}" ]]; then
  _fail "ROS_DISTRO not set — source /opt/ros/jazzy/setup.bash"
else
  _ok "ROS_DISTRO=$ROS_DISTRO"
fi

if [[ -z "${GZ_PARTITION:-}" ]]; then
  _warn "GZ_PARTITION not set — Gazebo topics may not resolve"
else
  _ok "GZ_PARTITION=$GZ_PARTITION"
fi

# ---------------------------------------------------------------------------
# 2. ROS 2 nodes
# ---------------------------------------------------------------------------
_sep "ROS 2 nodes"
if ! command -v ros2 >/dev/null 2>&1; then
  _fail "ros2 CLI not found — source ROS 2"
else
  NODE_LIST=$(timeout 5 ros2 node list 2>/dev/null || true)
  for expected_node in \
    "/move_group" \
    "/robot_state_publisher" \
    "/gz_ros2_control"; do
    if echo "$NODE_LIST" | grep -q "$expected_node"; then
      _ok "node found: $expected_node"
    else
      _fail "node MISSING: $expected_node"
    fi
  done
fi

# ---------------------------------------------------------------------------
# 3. TF frames
# ---------------------------------------------------------------------------
_sep "TF frames"
REQUIRED_FRAMES=(
  "world"
  "base_link"
  "tool0"
  "rg2_pinch_center"
)
for frame in "${REQUIRED_FRAMES[@]}"; do
  if timeout 5 ros2 run tf2_ros tf2_echo world "$frame" 2>/dev/null | grep -q "Translation"; then
    _ok "TF frame reachable: $frame"
  else
    _warn "TF frame not confirmed: $frame (may be normal if robot just started)"
  fi
done

# ---------------------------------------------------------------------------
# 4. Gazebo object topic
# ---------------------------------------------------------------------------
_sep "Gazebo object bridge"
if timeout 5 ros2 topic echo /world/ur5_world/dynamic_pose/info \
   --once 2>/dev/null | grep -q "name"; then
  _ok "/world/ur5_world/dynamic_pose/info responding"
else
  _warn "/world/ur5_world/dynamic_pose/info not responding — check gz_bridge"
fi

# ---------------------------------------------------------------------------
# 5. MoveIt action server
# ---------------------------------------------------------------------------
_sep "MoveIt action server"
if timeout 5 ros2 action list 2>/dev/null | grep -q "move_action"; then
  _ok "MoveIt move_action found"
else
  _warn "MoveIt move_action not found — MOVEIT pick may not work"
fi

# ---------------------------------------------------------------------------
# 6. ros2_control joint state
# ---------------------------------------------------------------------------
_sep "Joint state"
if timeout 5 ros2 topic echo /joint_states --once 2>/dev/null | grep -q "position"; then
  _ok "/joint_states topic responding"
else
  _fail "/joint_states not responding — robot_state_publisher or controller may be down"
fi

# ---------------------------------------------------------------------------
# 7. Gripper geometry constants
# ---------------------------------------------------------------------------
_sep "Gripper geometry"
if python3 -c "
from ur5_tools.gripper_geometry import load_gripper_geometry, RG2_PINCH_CENTER_FRAME
g = load_gripper_geometry()
xyz = g.xyz_for_frame(RG2_PINCH_CENTER_FRAME)
assert xyz is not None, 'xyz_for_frame returned None'
print(f'  rg2_pinch_center xyz={xyz}')
" 2>/dev/null; then
  _ok "gripper_geometry loads correctly"
else
  _warn "gripper_geometry import failed — workspace may not be built"
fi

# ---------------------------------------------------------------------------
# 8. Table calibration file
# ---------------------------------------------------------------------------
_sep "Table calibration"
CALIB_CANDIDATES=(
  "$HOME/.config/ur5_panel/table_calib.json"
  "$ROOT_DIR/table_calib.json"
  "/tmp/table_calib.json"
)
calib_found=0
for f in "${CALIB_CANDIDATES[@]}"; do
  if [[ -f "$f" ]]; then
    _ok "table_calib found: $f"
    calib_found=1
    break
  fi
done
[ "$calib_found" -eq 0 ] && _warn "No table_calib.json found — pixel-to-world mapping will use defaults"

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "========================================"
echo "[PRE_DEMO] PASS=$PASS  FAIL=$FAIL  WARN=$WARN"
if [ "$FAIL" -gt 0 ]; then
  echo "[PRE_DEMO] BLOCKING FAILURES:"
  for err in "${ERRORS[@]}"; do
    echo "  !! $err"
  done
fi
if [ "$WARN" -gt 0 ]; then
  echo "[PRE_DEMO] WARNINGS (non-blocking):"
  for w in "${WARNINGS[@]}"; do
    echo "  ~~ $w"
  done
fi
if [ "$FAIL" -gt 0 ]; then
  echo "[PRE_DEMO] RESULT: NOT READY"
  exit 1
fi
echo "[PRE_DEMO] RESULT: READY FOR DEMO  $(date -Iseconds)"
