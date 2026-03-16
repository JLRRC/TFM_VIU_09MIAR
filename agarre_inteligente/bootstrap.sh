#!/bin/bash
# ============================================================================
# bootstrap.sh - Script de configuración inicial del proyecto
# ============================================================================
# Descripción: Configura el entorno completo del proyecto desde cero
# Uso: ./bootstrap.sh
# ============================================================================

set -e  # Exit on error

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Directorio del proyecto
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_ROOT"

echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║       AGARRE INTELIGENTE - Bootstrap Setup                   ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

# ============================================================================
# 1. VERIFICAR REQUISITOS DEL SISTEMA
# ============================================================================
echo -e "${YELLOW}[1/6] Verificando requisitos del sistema...${NC}"

if ! command -v python3 &> /dev/null; then
    echo -e "${RED}✗ Python 3 no encontrado${NC}"
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
echo -e "${GREEN}✓ Python ${PYTHON_VERSION} encontrado${NC}"

if ! command -v pip3 &> /dev/null; then
    echo -e "${RED}✗ pip3 no encontrado${NC}"
    echo "Instale con: sudo apt install python3-pip"
    exit 1
fi
echo -e "${GREEN}✓ pip3 encontrado${NC}"

if ! command -v git &> /dev/null; then
    echo -e "${YELLOW}⚠ Git no encontrado (opcional para desarrollo)${NC}"
else
    echo -e "${GREEN}✓ Git encontrado${NC}"
fi

# ============================================================================
# 2. CREAR ENTORNO VIRTUAL
# ============================================================================
echo ""
echo -e "${YELLOW}[2/6] Configurando entorno virtual...${NC}"

if [ -d "venv" ]; then
    echo -e "${YELLOW}⚠ Entorno virtual ya existe. ¿Desea recrearlo? (s/n)${NC}"
    read -r RECREATE
    if [ "$RECREATE" = "s" ] || [ "$RECREATE" = "S" ]; then
        echo "Eliminando venv existente..."
        rm -rf venv
    else
        echo "Usando venv existente."
        source venv/bin/activate
        echo -e "${GREEN}✓ Entorno virtual activado${NC}"
    fi
fi

if [ ! -d "venv" ]; then
    echo "Creando entorno virtual..."
    python3 -m venv venv
    if [ $? -ne 0 ]; then
        echo -e "${RED}✗ Error al crear entorno virtual${NC}"
        echo "Instale python3-venv: sudo apt install python3-venv"
        exit 1
    fi
    echo -e "${GREEN}✓ Entorno virtual creado${NC}"
    
    source venv/bin/activate
    echo -e "${GREEN}✓ Entorno virtual activado${NC}"
fi

# ============================================================================
# 3. ACTUALIZAR PIP
# ============================================================================
echo ""
echo -e "${YELLOW}[3/6] Actualizando pip...${NC}"
pip install --upgrade pip > /dev/null 2>&1
echo -e "${GREEN}✓ pip actualizado a versión $(pip --version | cut -d' ' -f2)${NC}"

# ============================================================================
# 4. INSTALAR DEPENDENCIAS
# ============================================================================
echo ""
echo -e "${YELLOW}[4/6] Instalando dependencias Python...${NC}"
echo "Esto puede tomar varios minutos..."

if [ ! -f "requirements.txt" ]; then
    echo -e "${RED}✗ requirements.txt no encontrado${NC}"
    exit 1
fi

pip install -r requirements.txt

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Dependencias instaladas correctamente${NC}"
else
    echo -e "${RED}✗ Error instalando dependencias${NC}"
    exit 1
fi

# ============================================================================
# 5. VERIFICAR CUDA/GPU (opcional)
# ============================================================================
echo ""
echo -e "${YELLOW}[5/6] Verificando disponibilidad de GPU...${NC}"

python3 << EOF
import torch
if torch.cuda.is_available():
    print(f"\033[0;32m✓ GPU detectada: {torch.cuda.get_device_name(0)}\033[0m")
    print(f"\033[0;32m✓ CUDA versión: {torch.version.cuda}\033[0m")
else:
    print("\033[1;33m⚠ GPU no disponible. Entrenamiento usará CPU (será más lento)\033[0m")
EOF

# ============================================================================
# 6. CREAR DIRECTORIOS SI NO EXISTEN
# ============================================================================
echo ""
echo -e "${YELLOW}[6/6] Verificando estructura de directorios...${NC}"

DIRS=(
    "data/raw"
    "data/processed"
    "data/external"
    "experiments/runs"
    "experiments/checkpoints"
    "experiments/results"
    "models"
    "reports/figures"
    "reports/tables"
)

for dir in "${DIRS[@]}"; do
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        echo "  Creado: $dir"
    fi
done
echo -e "${GREEN}✓ Estructura de directorios verificada${NC}"

# ============================================================================
# RESUMEN FINAL
# ============================================================================
echo ""
echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                  ✓ SETUP COMPLETADO                          ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${GREEN}El proyecto está listo para usar.${NC}"
echo ""
echo -e "${YELLOW}Próximos pasos:${NC}"
echo "  1. Activar entorno virtual:"
echo "     ${BLUE}source venv/bin/activate${NC}"
echo ""
echo "  2. Verificar instalación:"
echo "     ${BLUE}./check_system.sh${NC}"
echo "     ${BLUE}./check_project.sh${NC}"
echo ""
echo "  3. Preparar datos en: ${BLUE}data/processed/${NC}"
echo ""
echo "  4. Entrenar modelo:"
echo "     ${BLUE}python scripts/train.py --config config/exp1_simple_rgb.yaml${NC}"
echo ""
echo -e "${YELLOW}Documentación:${NC}"
echo "  - README.md - Guía general del proyecto"
echo "  - docs/EXPERIMENTS.md - Registro de experimentos"
echo "  - docs/API.md - Documentación del código"
echo ""
echo -e "${GREEN}¡Éxito!${NC}"
