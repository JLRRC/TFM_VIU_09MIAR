#!/bin/bash
##############################################################################
# Fase 1 - Validación Post-Instalación
# Verifica que ROS2 Jazzy y todas las dependencias se instalaron correctamente
# Uso: ./01_validate_installation.sh
##############################################################################

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
VALIDATE_LOG="${WS_DIR}/validate_phase1_$(date +%s).txt"

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

PASS=0
FAIL=0
WARN=0

log_result() {
  local status=$1
  local test=$2
  local detail=$3
  
  case $status in
    PASS)
      echo -e "${GREEN}✓ PASS${NC}: $test"
      PASS=$((PASS + 1))
      echo "[✓] $test" >> "$VALIDATE_LOG"
      ;;
    FAIL)
      echo -e "${RED}✗ FAIL${NC}: $test"
      FAIL=$((FAIL + 1))
      echo "[✗] $test - ${detail}" >> "$VALIDATE_LOG"
      ;;
    WARN)
      echo -e "${YELLOW}⚠ WARN${NC}: $test"
      WARN=$((WARN + 1))
      echo "[⚠] $test - ${detail}" >> "$VALIDATE_LOG"
      ;;
  esac
}

# Header
clear
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  Fase 1 - Post-Installation Validation                         ║"
echo "║  ROS2 Jazzy & Dependencies Verification                        ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "Log: ${VALIDATE_LOG}"
echo ""

{
  echo "═══════════════════════════════════════════════════════════════"
  echo "ROS2 JAZZY POST-INSTALLATION VALIDATION"
  echo "Timestamp: $(date)"
  echo "═══════════════════════════════════════════════════════════════"
  echo ""
} > "$VALIDATE_LOG"

# ============================================================
# CORE ROS2 CHECKS
# ============================================================
echo "▸ CORE ROS2 CHECKS:"
echo ""

# Check 1: ROS2 setup.bash
if [ -f /opt/ros/jazzy/setup.bash ]; then
  log_result PASS "ROS2 setup.bash exists" ""
else
  log_result FAIL "ROS2 setup.bash exists" "File not found at /opt/ros/jazzy/setup.bash"
fi

# Check 2: Source ROS2 environment
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
if [ ! -z "$ROS_DISTRO" ]; then
  if [ "$ROS_DISTRO" = "jazzy" ]; then
    log_result PASS "ROS_DISTRO is 'jazzy'" ""
  else
    log_result WARN "ROS_DISTRO is 'jazzy'" "Found: ${ROS_DISTRO}"
  fi
else
  log_result FAIL "ROS_DISTRO environment variable set" "Not set after sourcing setup.bash"
fi

# Check 3: ros2 CLI
if command -v ros2 &> /dev/null; then
  ROS2_VERSION=$(ros2 --version 2>&1 | grep -oP 'version \K[0-9a-zA-Z.]+' || echo "unknown")
  log_result PASS "ros2 CLI available" "Version: ${ROS2_VERSION}"
else
  log_result FAIL "ros2 CLI available" "Command not found"
fi

# Check 4: ROS2 packages core
ROS2_CORE_PKGS="ros-core std-msgs geometry-msgs"
for pkg in $ROS2_CORE_PKGS; do
  if dpkg -l | grep -q "ros-jazzy-${pkg}" 2>/dev/null; then
    log_result PASS "ROS2 package: ${pkg}" ""
  else
    log_result WARN "ROS2 package: ${pkg}" "Not found in package list"
  fi
done

echo ""
echo "▸ BUILD TOOLS:"
echo ""

# Check 5: colcon
if command -v colcon &> /dev/null; then
  COLCON_VER=$(colcon --help 2>&1 | head -1 || true)
  log_result PASS "colcon build tool" "${COLCON_VER}"
else
  log_result FAIL "colcon build tool" "Command not found"
fi

# Check 6: colcon extensions
if python3 -c "import colcon_cmake; import colcon_python_cmake_build_type" 2>/dev/null; then
  log_result PASS "colcon CMake extensions" ""
else
  log_result WARN "colcon CMake extensions" "Some colcon extensions may not be installed"
fi

echo ""
echo "▸ DEPENDENCY MANAGEMENT:"
echo ""

# Check 7: rosdep
if command -v rosdep &> /dev/null; then
  log_result PASS "rosdep dependency manager" ""
else
  log_result FAIL "rosdep dependency manager" "Command not found"
fi

# Check 8: rosdep sources initialized
if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || [ -f "$HOME/.ros/rosdep/sources.cache.d/index.yaml" ]; then
  log_result PASS "rosdep sources initialized" ""
else
  log_result WARN "rosdep sources initialized" "Default sources may need initialization (rosdep update)"
fi

echo ""
echo "▸ SYSTEM PACKAGES:"
echo ""

# Check 9: Python3 packages
PYTHON_PKGS="python3-pip python3-venv"
for pkg in $PYTHON_PKGS; do
  if dpkg -l | grep -q "^ii  $pkg" 2>/dev/null; then
    log_result PASS "System package: ${pkg}" ""
  else
    log_result FAIL "System package: ${pkg}" "Not installed"
  fi
done

# Check 10: Build tools chain
BUILD_CHAIN="gcc g++ make cmake"
for tool in $BUILD_CHAIN; do
  if command -v $tool &> /dev/null; then
    log_result PASS "Build tool: ${tool}" ""
  else
    log_result FAIL "Build tool: ${tool}" "Not found in PATH"
  fi
done

echo ""
echo "▸ OPTIONAL SIMULATION TOOLS:"
echo ""

# Check 11: Gazebo
if command -v gz &> /dev/null; then
  GZ_VER=$(gz --version 2>&1 | head -1)
  log_result PASS "Gazebo simulator" "${GZ_VER}"
else
  log_result WARN "Gazebo simulator" "Not installed (needed for simulation phases, install separately if needed)"
fi

# Check 12: MoveIt
if dpkg -l | grep -q "ros-jazzy-moveit-core" 2>/dev/null; then
  log_result PASS "MoveIt motion planning" ""
else
  log_result WARN "MoveIt motion planning" "Not installed (optional, install if needed for planning tasks)"
fi

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  VALIDATION SUMMARY                                            ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo -e "${GREEN}✓ Passed: ${PASS}${NC} | ${YELLOW}⚠ Warnings: ${WARN}${NC} | ${RED}✗ Failed: ${FAIL}${NC}"
echo ""

# Determine pass/fail
{
  echo ""
  echo "═══════════════════════════════════════════════════════════════"
  echo "VALIDATION RESULT"
  echo "═══════════════════════════════════════════════════════════════"
  echo "✓ Passed: ${PASS}"
  echo "⚠ Warnings: ${WARN}"
  echo "✗ Failed: ${FAIL}"
  echo ""
} >> "$VALIDATE_LOG"

if [[ $FAIL -eq 0 ]]; then
  echo -e "${GREEN}✓ VALIDATION PASSED${NC} - Fase 1 ROS2 Installation Successful"
  echo "Status: ✓ PASS" >> "$VALIDATE_LOG"
  echo ""
  echo "You can now:"
  echo "  1. Ensure ROS2 is sourced: source /opt/ros/jazzy/setup.bash"
  echo "  2. Proceed to Fase 2 (Workspace Setup): run colcon build"
  echo "  3. Optional: Install Gazebo and other simulation tools"
  exit 0
else
  echo -e "${RED}✗ VALIDATION FAILED${NC} - Some components are missing"
  echo "Status: ✗ FAIL" >> "$VALIDATE_LOG"
  echo ""
  echo "Failed checks:"
  grep "^\[✗\]" "$VALIDATE_LOG" | sed 's/^\[✗\]/  /'
  echo ""
  echo "Please fix the above issues and re-run this validation."
  exit 1
fi
