#!/bin/bash
##############################################################################
# Fase 1 - Preflight Check
# Verifica que el sistema cumple requisitos mínimos antes de reconstrucción
# Uso: ./01_preflight_check.sh
##############################################################################

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
REPORT_FILE="${WS_DIR}/preflight_report_$(date +%s).txt"

# Colores para salida
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Contadores
PASS=0
FAIL=0
WARN=0

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  Preflight Check - agarre_ros2_ws Reconstruction (Fase 1)      ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "Sistema: $(uname -a | cut -d' ' -f1-3)"
echo "Fecha: $(date)"
echo "Workspace: ${WS_DIR}"
echo "Report será guardado en: ${REPORT_FILE}"
echo ""

{
  echo "╔════════════════════════════════════════════════════════════════╗"
  echo "║  PREFLIGHT REPORT - $(date)                ║"
  echo "╚════════════════════════════════════════════════════════════════╝"
  echo ""
} > "$REPORT_FILE"

# ============================================================
# CHECK 1: OS Version
# ============================================================
echo -n "▸ Comprobando SO (Ubuntu 24.04)... "
if grep -q "24.04" /etc/lsb-release 2>/dev/null; then
  echo -e "${GREEN}✓ PASS${NC}"
  PASS=$((PASS + 1))
  echo "  ✓ Ubuntu 24.04 LTS detectado" >> "$REPORT_FILE"
else
  OS_VERSION=$(grep DISTRIB_RELEASE /etc/lsb-release 2>/dev/null | cut -d= -f2)
  echo -e "${YELLOW}⚠ WARN${NC} (Ubuntu ${OS_VERSION})"
  WARN=$((WARN + 1))
  echo "  ⚠ Ubuntu ${OS_VERSION} detectado (se espera 24.04, pero puede funcionar)" >> "$REPORT_FILE"
fi

# ============================================================
# CHECK 2: Python Version
# ============================================================
echo -n "▸ Comprobando Python (>= 3.10)... "
if command -v python3 &> /dev/null; then
  PY_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
  PY_MAJOR=$(echo $PY_VERSION | cut -d. -f1)
  PY_MINOR=$(echo $PY_VERSION | cut -d. -f2)
  if [[ $PY_MAJOR -gt 3 ]] || [[ $PY_MAJOR -eq 3 && $PY_MINOR -ge 10 ]]; then
    echo -e "${GREEN}✓ PASS${NC} (${PY_VERSION})"
    PASS=$((PASS + 1))
    echo "  ✓ Python ${PY_VERSION} detectado" >> "$REPORT_FILE"
  else
    echo -e "${RED}✗ FAIL${NC} (${PY_VERSION})"
    FAIL=$((FAIL + 1))
    echo "  ✗ Python ${PY_VERSION} (se requiere >= 3.10)" >> "$REPORT_FILE"
  fi
else
  echo -e "${RED}✗ FAIL${NC}"
  FAIL=$((FAIL + 1))
  echo "  ✗ Python3 no instalado" >> "$REPORT_FILE"
fi

# ============================================================
# CHECK 3: Build Tools
# ============================================================
echo -n "▸ Comprobando herramientas build (build-essential, cmake)... "
MISSING_BUILD_TOOLS=""
for tool in gcc g++ make cmake; do
  if ! command -v $tool &> /dev/null; then
    MISSING_BUILD_TOOLS="${MISSING_BUILD_TOOLS} $tool"
  fi
done
if [[ -z "$MISSING_BUILD_TOOLS" ]]; then
  echo -e "${GREEN}✓ PASS${NC}"
  PASS=$((PASS + 1))
  echo "  ✓ Todas las herramientas build disponibles" >> "$REPORT_FILE"
else
  echo -e "${YELLOW}⚠ WARN${NC} (faltan:${MISSING_BUILD_TOOLS})"
  WARN=$((WARN + 1))
  echo "  ⚠ Herramientas faltantes:${MISSING_BUILD_TOOLS}" >> "$REPORT_FILE"
fi

# ============================================================
# CHECK 4: Git
# ============================================================
echo -n "▸ Comprobando Git... "
if command -v git &> /dev/null; then
  GIT_VERSION=$(git --version | awk '{print $3}')
  echo -e "${GREEN}✓ PASS${NC} (${GIT_VERSION})"
  PASS=$((PASS + 1))
  echo "  ✓ Git ${GIT_VERSION} disponible" >> "$REPORT_FILE"
else
  echo -e "${RED}✗ FAIL${NC}"
  FAIL=$((FAIL + 1))
  echo "  ✗ Git no instalado" >> "$REPORT_FILE"
fi

# ============================================================
# CHECK 5: ROS2 (Jazzy)
# ============================================================
echo -n "▸ Comprobando ROS2 Jazzy... "
if [ -f /opt/ros/jazzy/setup.bash ]; then
  echo -e "${GREEN}✓ PASS${NC}"
  PASS=$((PASS + 1))
  echo "  ✓ ROS2 Jazzy setup.bash encontrado" >> "$REPORT_FILE"
elif command -v ros2 &> /dev/null; then
  ROS_DISTRO=$(. /opt/ros/*/setup.bash 2>/dev/null; echo $ROS_DISTRO 2>/dev/null || echo "desconocido")
  echo -e "${YELLOW}⚠ WARN${NC} (distro: ${ROS_DISTRO})"
  WARN=$((WARN + 1))
  echo "  ⚠ ROS2 ${ROS_DISTRO} detectado (se espera Jazzy)" >> "$REPORT_FILE"
else
  echo -e "${RED}✗ FAIL${NC}"
  FAIL=$((FAIL + 1))
  echo "  ✗ ROS2 no instalado" >> "$REPORT_FILE"
fi

# ============================================================
# CHECK 6: colcon
# ============================================================
echo -n "▸ Comprobando colcon... "
if command -v colcon &> /dev/null; then
  COLCON_VERSION=$(colcon --version | awk '{print $NF}')
  echo -e "${GREEN}✓ PASS${NC} (${COLCON_VERSION})"
  PASS=$((PASS + 1))
  echo "  ✓ colcon ${COLCON_VERSION} disponible" >> "$REPORT_FILE"
else
  echo -e "${RED}✗ FAIL${NC}"
  FAIL=$((FAIL + 1))
  echo "  ✗ colcon no instalado (necesario para compilar workspace)" >> "$REPORT_FILE"
fi

# ============================================================
# CHECK 7: Gazebo
# ============================================================
echo -n "▸ Comprobando Gazebo... "
if command -v gz &> /dev/null; then
  GZ_VERSION=$(gz --version 2>&1 | head -1)
  echo -e "${GREEN}✓ PASS${NC}"
  PASS=$((PASS + 1))
  echo "  ✓ Gazebo disponible: ${GZ_VERSION}" >> "$REPORT_FILE"
else
  echo -e "${RED}✗ FAIL${NC}"
  FAIL=$((FAIL + 1))
  echo "  ✗ Gazebo no instalado" >> "$REPORT_FILE"
fi

# ============================================================
# CHECK 8: Workspace Structure
# ============================================================
echo -n "▸ Comprobando estructura workspace... "
if [ -d "${WS_DIR}/src" ] && [ -d "${WS_DIR}/scripts" ]; then
  echo -e "${GREEN}✓ PASS${NC}"
  PASS=$((PASS + 1))
  echo "  ✓ Estructura base del workspace presente" >> "$REPORT_FILE"
else
  echo -e "${RED}✗ FAIL${NC}"
  FAIL=$((FAIL + 1))
  echo "  ✗ Estructura workspace incompleta (falta src/ o scripts/)" >> "$REPORT_FILE"
fi

# ============================================================
# CHECK 9: Disk Space
# ============================================================
echo -n "▸ Comprobando espacio disco (>= 20GB disponibles)... "
AVAILABLE_SPACE=$(df -BG "${WS_DIR}" | tail -1 | awk '{print $4}' | sed 's/G//')
if [[ $AVAILABLE_SPACE -ge 20 ]]; then
  echo -e "${GREEN}✓ PASS${NC} (${AVAILABLE_SPACE}GB)"
  PASS=$((PASS + 1))
  echo "  ✓ Espacio disponible: ${AVAILABLE_SPACE}GB" >> "$REPORT_FILE"
else
  echo -e "${RED}✗ FAIL${NC} (${AVAILABLE_SPACE}GB)"
  FAIL=$((FAIL + 1))
  echo "  ✗ Espacio insuficiente: ${AVAILABLE_SPACE}GB (se requieren 20GB mínimo)" >> "$REPORT_FILE"
fi

# ============================================================
# CHECK 10: venv
# ============================================================
echo -n "▸ Comprobando venv para aislamiento Python... "
if python3 -m venv --help &> /dev/null; then
  echo -e "${GREEN}✓ PASS${NC}"
  PASS=$((PASS + 1))
  echo "  ✓ venv disponible" >> "$REPORT_FILE"
else
  echo -e "${RED}✗ FAIL${NC}"
  FAIL=$((FAIL + 1))
  echo "  ✗ venv no instalado (python3-venv)" >> "$REPORT_FILE"
fi

# ============================================================
# Summary
# ============================================================
echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  RESUMEN                                                       ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${GREEN}✓ Passed: ${PASS}${NC} | ${YELLOW}⚠ Warnings: ${WARN}${NC} | ${RED}✗ Failed: ${FAIL}${NC}"

{
  echo ""
  echo "╔════════════════════════════════════════════════════════════════╗"
  echo "║  RESUMEN FINAL                                                 ║"
  echo "╚════════════════════════════════════════════════════════════════╝"
  echo "✓ Passed: ${PASS}"
  echo "⚠ Warnings: ${WARN}"
  echo "✗ Failed: ${FAIL}"
  echo ""
  
  if [[ $FAIL -eq 0 ]]; then
    echo "RESULTADO: ✓ PASS - Sistema listo para Fase 1 (Instalación)"
  elif [[ $FAIL -le 2 ]]; then
    echo "RESULTADO: ⚠ PROCEED WITH CAUTION - Algunos requisitos faltantes (vea detalles arriba)"
  else
    echo "RESULTADO: ✗ FAIL - Sistema no cumple requisitos mínimos (vea detalles arriba)"
  fi
  echo ""
  echo "Próximos pasos:"
  echo "  1. Si PASS: ejecutar 01_install_ros2_jazzy.sh"
  echo "  2. Si warnings: revisar detalles en este reporte"
  echo "  3. Si FAIL: instalar componentes faltantes antes de continuar"
} >> "$REPORT_FILE"

echo ""
if [[ $FAIL -eq 0 ]]; then
  echo -e "${GREEN}✓ PASS${NC} - Sistema listo para Fase 1 (Instalación de dependencias)"
  exit 0
elif [[ $FAIL -le 2 ]]; then
  echo -e "${YELLOW}⚠ PROCEED WITH CAUTION${NC} - Algunos requisitos faltantes"
  echo "Revise: ${REPORT_FILE}"
  exit 0
else
  echo -e "${RED}✗ FAIL${NC} - Sistema no cumple requisitos mínimos"
  echo "Revise: ${REPORT_FILE}"
  exit 1
fi
