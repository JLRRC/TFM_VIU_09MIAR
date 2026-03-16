#!/bin/bash
##############################################################################
# Fase 1 - Pre-installation toolkit
# Orchestrates all Phase 1 checks and creates environment for Fase 2
# Usage: ./01_run_all_preflight.sh
##############################################################################

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  Fase 1 - Full Phase 1 Runner                                   ║"
echo "║  Ejecuta todos los chequeos y instalaciones de Fase 1          ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Make scripts executable
chmod +x "${SCRIPT_DIR}/01_preflight_check.sh" 2>/dev/null || true
chmod +x "${SCRIPT_DIR}/01_install_ros2_jazzy.sh" 2>/dev/null || true
chmod +x "${SCRIPT_DIR}/01_validate_installation.sh" 2>/dev/null || true

# Run preflight
echo "▸ STEP 1/3: Running preflight checks..."
echo ""
"${SCRIPT_DIR}/01_preflight_check.sh"
PREFLIGHT_RESULT=$?

if [[ $PREFLIGHT_RESULT -ne 0 ]]; then
  echo -e "${RED}✗ Preflight checks failed or require attention${NC}"
  echo "Please fix the issues and try again."
  exit 1
fi

echo ""
echo -e "${GREEN}✓ Preflight checks passed${NC}"
echo ""
read -p "Proceed with ROS2 Jazzy installation? This may take 10-15 minutes. (y/N): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
  echo "Installation cancelled."
  exit 0
fi

# Run installation
echo ""
echo "▸ STEP 2/3: Installing ROS2 Jazzy..."
echo ""
"${SCRIPT_DIR}/01_install_ros2_jazzy.sh"
INSTALL_RESULT=$?

if [[ $INSTALL_RESULT -ne 0 ]]; then
  echo -e "${RED}✗ ROS2 installation failed${NC}"
  exit 1
fi

echo ""
echo -e "${GREEN}✓ ROS2 Jazzy installation completed${NC}"
echo ""

# Run validation
echo "▸ STEP 3/3: Validating installation..."
echo ""
"${SCRIPT_DIR}/01_validate_installation.sh"
VALIDATE_RESULT=$?

if [[ $VALIDATE_RESULT -ne 0 ]]; then
  echo -e "${RED}✗ Validation failed${NC}"
  exit 1
fi

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo -e "║  ${GREEN}✓ FASE 1 COMPLETED SUCCESSFULLY${NC}                              ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "Next steps:"
echo "  1. Source ROS2: source /opt/ros/jazzy/setup.bash"
echo "  2. See Fase 2 Plan: cat docs/rebuild_plan.md"
echo "  3. Start workspace build: colcon build --symlink-install"
echo ""
