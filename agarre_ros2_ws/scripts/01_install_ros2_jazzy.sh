#!/bin/bash
##############################################################################
# Fase 1 - Instalación de ROS2 Jazzy y Dependencias
# Instala software base y ROS2 Jazzy en Ubuntu 24.04
# Uso: ./01_install_ros2_jazzy.sh
# Requiere: ejecutar 01_preflight_check.sh primero (PASS o WARN)
##############################################################################

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
INSTALL_LOG="${WS_DIR}/install_ros2_jazzy_$(date +%s).log"

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$INSTALL_LOG"
}

info() {
  echo -e "${BLUE}ℹ${NC} $1" | tee -a "$INSTALL_LOG"
}

success() {
  echo -e "${GREEN}✓${NC} $1" | tee -a "$INSTALL_LOG"
}

error() {
  echo -e "${RED}✗${NC} $1" | tee -a "$INSTALL_LOG"
}

warning() {
  echo -e "${YELLOW}⚠${NC} $1" | tee -a "$INSTALL_LOG"
}

run_step() {
  local description="$1"
  shift
  log "$description"
  if "$@" >> "$INSTALL_LOG" 2>&1; then
    success "$description"
  else
    error "$description"
    error "Revisa el log: $INSTALL_LOG"
    exit 1
  fi
}

# Header
clear
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  Fase 1 - ROS2 Jazzy Installation                             ║"
echo "║  Log: ${INSTALL_LOG}                ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

log "Inicializando instalación de ROS2 Jazzy..."

# Remove stale ROS2 source entries from previous failed attempts.
if [ -f /etc/apt/sources.list.d/ros2.list ]; then
  if grep -q "repo.ros2.org/ubuntu" /etc/apt/sources.list.d/ros2.list 2>/dev/null; then
    warning "Detectado repositorio ROS2 obsoleto (repo.ros2.org). Eliminando para evitar errores 404..."
    sudo rm -f /etc/apt/sources.list.d/ros2.list >> "$INSTALL_LOG" 2>&1 || true
  fi
fi

# ============================================================
# STEP 1: Update system
# ============================================================
info "STEP 1/8: Actualizando gestor de paquetes..."
run_step "Actualización de paquetes del sistema" sudo apt-get update

# Ensure universe repo is enabled on Ubuntu for dependency availability.
if command -v add-apt-repository >/dev/null 2>&1; then
  run_step "Repositorio 'universe' habilitado" sudo add-apt-repository -y universe
  run_step "apt-get update tras habilitar universe" sudo apt-get update
fi

# ============================================================
# STEP 2: Install build tools
# ============================================================
info "STEP 2/8: Instalando herramientas de compilación..."
BUILD_TOOLS="build-essential cmake git wget curl python3-pip python3-venv"
log "Instalando: ${BUILD_TOOLS}"
run_step "Herramientas de compilación instaladas" sudo apt-get install -y $BUILD_TOOLS software-properties-common

# ============================================================
# STEP 3: Setup ROS2 keyring
# ============================================================
info "STEP 3/8: Configurando keyring de ROS2..."
run_step "Keyring ROS2 configurado" sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# ============================================================
# STEP 4: Add ROS2 repo
# ============================================================
info "STEP 4/8: Agregando repositorio ROS2..."
run_step "Repositorio ROS2 agregado" bash -lc "echo 'deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main' | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"

# ============================================================
# STEP 5: Update from ROS2 repo
# ============================================================
info "STEP 5/8: Actualizando desde repositorio ROS2..."
run_step "Repositorio ROS2 actualizado" sudo apt-get update

# ============================================================
# STEP 6: Install ROS2 Jazzy
# ============================================================
info "STEP 6/8: Instalando ROS2 Jazzy (full desktop)..."
log "Esto puede tomar varios minutos..."
run_step "ROS2 Jazzy instalado" sudo apt-get install -y ros-jazzy-desktop

# ============================================================
# STEP 7: Install development tools
# ============================================================
info "STEP 7/8: Instalando herramientas de desarrollo ROS2..."
log "Instalando: colcon, rosdoc, etc."
ROS2_DEV_TOOLS="python3-colcon-common-extensions python3-rosdep ros-dev-tools"
if sudo apt-get install -y $ROS2_DEV_TOOLS >> "$INSTALL_LOG" 2>&1; then
  success "Herramientas de desarrollo instaladas"
else
  warning "Instalación parcial de herramientas dev; reintentando mínimo requerido (colcon + rosdep)"
  run_step "Herramientas mínimas de desarrollo instaladas" sudo apt-get install -y python3-colcon-common-extensions python3-rosdep
fi

# ============================================================
# STEP 8: Initialize rosdep
# ============================================================
info "STEP 8/8: Inicializando rosdep..."
if sudo rosdep init >> "$INSTALL_LOG" 2>&1; then
  success "rosdep init completado"
else
  warning "rosdep init puede ya estar inicializado; continuando"
fi
run_step "rosdep actualizado" rosdep update

# ============================================================
# Verification
# ============================================================
echo ""
info "Verificando instalación..."

# Check ROS2 setup
if [ -f /opt/ros/jazzy/setup.bash ]; then
  success "setup.bash de ROS2 Jazzy encontrado"
else
  error "setup.bash no encontrado en /opt/ros/jazzy/"
  exit 1
fi

# Check colcon
if command -v colcon &> /dev/null; then
  COLCON_VER=$(colcon --version || echo "desconocida")
  success "colcon disponible: ${COLCON_VER}"
else
  error "colcon no disponible"
  exit 1
fi

# Check rosdep
if command -v rosdep &> /dev/null; then
  success "rosdep disponible"
else
  error "rosdep no disponible"
  exit 1
fi

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  ✓ INSTALACIÓN COMPLETADA                                      ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "Próximos pasos:"
echo "  1. Para usar ROS2 en esta sesión:"
echo "     source /opt/ros/jazzy/setup.bash"
echo ""
echo "  2. Para agregar automáticamente a ~/.bashrc:"
echo "     echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc"
echo "     source ~/.bashrc"
echo ""
echo "  3. Ejecutar siguiente fase:"
echo "     ./01_validate_installation.sh"
echo ""
echo "Log guardado en: ${INSTALL_LOG}"
