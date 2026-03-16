#!/bin/bash
# ============================================================================
# check_system.sh - Verificación de requisitos del sistema
# ============================================================================
# Descripción: Verifica que el sistema tiene todos los requisitos para el proyecto
# Uso: ./check_system.sh
# ============================================================================

set -u

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║       VERIFICACIÓN DE REQUISITOS DEL SISTEMA                 ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

ERRORS=0
WARNINGS=0

# ============================================================================
# 1. SISTEMA OPERATIVO
# ============================================================================
echo -e "${YELLOW}[1] Sistema Operativo${NC}"
echo "─────────────────────────────────────────────"

if [ -f /etc/os-release ]; then
    source /etc/os-release
    echo -e "OS: ${GREEN}$PRETTY_NAME${NC}"
    echo -e "Kernel: ${GREEN}$(uname -r)${NC}"
else
    echo -e "${RED}✗ No se pudo detectar el sistema operativo${NC}"
    ((ERRORS++))
fi

echo -e "Usuario: ${GREEN}$(whoami)${NC}"
echo -e "Hostname: ${GREEN}$(hostname)${NC}"
echo ""

# ============================================================================
# 2. HARDWARE
# ============================================================================
echo -e "${YELLOW}[2] Hardware${NC}"
echo "─────────────────────────────────────────────"

# CPU
CPU_MODEL=$(lscpu | grep "Model name" | cut -d':' -f2 | xargs)
CPU_CORES=$(nproc)
echo -e "CPU: ${GREEN}$CPU_MODEL${NC}"
echo -e "Cores: ${GREEN}$CPU_CORES${NC}"

# RAM
TOTAL_RAM=$(free -h | awk '/^Mem:/{print $2}')
AVAIL_RAM=$(free -h | awk '/^Mem:/{print $7}')
echo -e "RAM Total: ${GREEN}$TOTAL_RAM${NC}"
echo -e "RAM Disponible: ${GREEN}$AVAIL_RAM${NC}"

# Disco
DISK_TOTAL=$(df -h ~ | awk 'NR==2{print $2}')
DISK_AVAIL=$(df -h ~ | awk 'NR==2{print $4}')
DISK_USE=$(df -h ~ | awk 'NR==2{print $5}')
echo -e "Disco Total: ${GREEN}$DISK_TOTAL${NC}"
echo -e "Disco Disponible: ${GREEN}$DISK_AVAIL${NC} (uso: $DISK_USE)"

# GPU
if command -v nvidia-smi &> /dev/null; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    echo -e "GPU: ${GREEN}$GPU_NAME${NC}"
    GPU_MEMORY=$(nvidia-smi --query-gpu=memory.total --format=csv,noheader 2>/dev/null | head -1)
    echo -e "GPU Memory: ${GREEN}$GPU_MEMORY${NC}"
else
    echo -e "GPU: ${YELLOW}⚠ No detectada (entrenamiento será en CPU)${NC}"
    ((WARNINGS++))
fi

echo ""

# ============================================================================
# 3. PYTHON Y PIP
# ============================================================================
echo -e "${YELLOW}[3] Python y Pip${NC}"
echo "─────────────────────────────────────────────"

if command -v python3 &> /dev/null; then
    PY_VERSION=$(python3 --version)
    echo -e "Python: ${GREEN}✓ $PY_VERSION${NC}"
    
    PY_MAJOR=$(python3 -c 'import sys; print(sys.version_info.major)')
    PY_MINOR=$(python3 -c 'import sys; print(sys.version_info.minor)')
    
    if [ "$PY_MAJOR" -eq 3 ] && [ "$PY_MINOR" -ge 8 ]; then
        echo -e "Versión: ${GREEN}✓ Compatible (≥3.8)${NC}"
    else
        echo -e "Versión: ${RED}✗ Requiere Python ≥3.8${NC}"
        ((ERRORS++))
    fi
else
    echo -e "Python: ${RED}✗ No encontrado${NC}"
    ((ERRORS++))
fi

if command -v pip3 &> /dev/null; then
    PIP_VERSION=$(pip3 --version | cut -d' ' -f2)
    echo -e "Pip: ${GREEN}✓ $PIP_VERSION${NC}"
else
    echo -e "Pip: ${RED}✗ No encontrado${NC}"
    echo "  Instalar: sudo apt install python3-pip"
    ((ERRORS++))
fi

if python3 -c "import venv" 2>/dev/null; then
    echo -e "venv: ${GREEN}✓ Disponible${NC}"
else
    echo -e "venv: ${RED}✗ No disponible${NC}"
    echo "  Instalar: sudo apt install python3-venv"
    ((ERRORS++))
fi

echo ""

# ============================================================================
# 4. HERRAMIENTAS DE DESARROLLO
# ============================================================================
echo -e "${YELLOW}[4] Herramientas de Desarrollo${NC}"
echo "─────────────────────────────────────────────"

check_tool() {
    if command -v "$1" &> /dev/null; then
        VERSION=$($1 --version 2>/dev/null | head -1)
        echo -e "$1: ${GREEN}✓ $VERSION${NC}"
    else
        echo -e "$1: ${RED}✗ No encontrado${NC}"
        ((ERRORS++))
    fi
}

check_tool "git"
check_tool "gcc"
check_tool "g++"
check_tool "make"
check_tool "cmake"
check_tool "pkg-config"
check_tool "tar"
check_tool "unzip"
check_tool "jq"
check_tool "rsync"
check_tool "curl"
check_tool "wget"

if command -v tree &> /dev/null; then
    echo -e "tree: ${GREEN}✓ $(tree --version | head -1)${NC}"
else
    echo -e "tree: ${YELLOW}⚠ No encontrado (opcional)${NC}"
    ((WARNINGS++))
fi

echo ""

# ============================================================================
# 4.1 HERRAMIENTAS ROS/GAZEBO (NO BLOQUEANTES)
# ============================================================================
echo -e "${YELLOW}[4.1] ROS2/Gazebo (dependencia externa)${NC}"
echo "─────────────────────────────────────────────"

for tool in ros2 colcon gz gazebo; do
    if command -v "$tool" &> /dev/null; then
        echo -e "$tool: ${GREEN}✓ Disponible${NC}"
    else
        echo -e "$tool: ${YELLOW}⚠ No instalado${NC}"
    fi
done

echo "Nota: para la parte agarra_inteligente (ML) no es bloqueante;"
echo "ROS2/Gazebo se requiere para el demostrador en agarra_ros2_ws."
echo ""

# ============================================================================
# 5. ENTORNO VIRTUAL
# ============================================================================
echo -e "${YELLOW}[5] Entorno Virtual${NC}"
echo "─────────────────────────────────────────────"

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -d "$PROJECT_ROOT/venv" ]; then
    echo -e "venv: ${GREEN}✓ Existe${NC}"
    
    if [ -n "${VIRTUAL_ENV:-}" ]; then
        echo -e "Estado: ${GREEN}✓ Activado${NC}"
        echo -e "Path: ${BLUE}$VIRTUAL_ENV${NC}"
    else
        echo -e "Estado: ${YELLOW}⚠ No activado${NC}"
        echo "  Activar: source venv/bin/activate"
        ((WARNINGS++))
    fi
else
    echo -e "venv: ${RED}✗ No existe${NC}"
    echo "  Crear: ./bootstrap.sh"
    ((ERRORS++))
fi

echo ""

# ============================================================================
# 6. DEPENDENCIAS PYTHON
# ============================================================================
echo -e "${YELLOW}[6] Dependencias Python${NC}"
echo "─────────────────────────────────────────────"

if [ -n "${VIRTUAL_ENV:-}" ]; then
    check_python_package() {
        if python3 -c "import $1" 2>/dev/null; then
            VERSION=$(python3 -c "import $1; print($1.__version__)" 2>/dev/null || echo "")
            echo -e "$2: ${GREEN}✓ $VERSION${NC}"
        else
            echo -e "$2: ${RED}✗ No instalado${NC}"
            ((ERRORS++))
        fi
    }
    
    check_python_package "torch" "PyTorch"
    check_python_package "torchvision" "torchvision"
    check_python_package "cv2" "OpenCV"
    check_python_package "PIL" "Pillow"
    check_python_package "numpy" "NumPy"
    check_python_package "matplotlib" "Matplotlib"
    check_python_package "yaml" "PyYAML"
    check_python_package "sklearn" "scikit-learn"
    check_python_package "tensorboard" "TensorBoard"
else
    echo -e "${YELLOW}⚠ Entorno virtual no activado. Active para verificar dependencias.${NC}"
    ((WARNINGS++))
fi

echo ""

# ============================================================================
# RESUMEN
# ============================================================================
echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                     RESUMEN                                  ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo -e "${GREEN}✓ TODOS LOS REQUISITOS CUMPLIDOS${NC}"
    echo ""
    echo "El sistema está listo para ejecutar el proyecto."
    exit 0
elif [ $ERRORS -eq 0 ]; then
    echo -e "${YELLOW}⚠ SISTEMA FUNCIONAL CON ADVERTENCIAS${NC}"
    echo ""
    echo -e "Advertencias: ${YELLOW}$WARNINGS${NC}"
    echo "El sistema puede ejecutar el proyecto, pero considere resolver las advertencias."
    exit 0
else
    echo -e "${RED}✗ FALTAN REQUISITOS CRÍTICOS${NC}"
    echo ""
    echo -e "Errores: ${RED}$ERRORS${NC}"
    echo -e "Advertencias: ${YELLOW}$WARNINGS${NC}"
    echo ""
    echo "Ejecute ./bootstrap.sh para configurar el proyecto automáticamente."
    exit 1
fi
