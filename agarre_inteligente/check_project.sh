#!/bin/bash
# ============================================================================
# check_project.sh - Verificación de integridad del proyecto
# ============================================================================
# Descripción: Verifica que la estructura y archivos del proyecto sean correctos
# Uso: ./check_project.sh
# ============================================================================

set -u

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔══════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║         VERIFICACIÓN DE INTEGRIDAD DEL PROYECTO              ║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_ROOT"

ERRORS=0
WARNINGS=0

# ============================================================================
# 1. ESTRUCTURA DE DIRECTORIOS
# ============================================================================
echo -e "${YELLOW}[1] Estructura de Directorios${NC}"
echo "─────────────────────────────────────────────"

REQUIRED_DIRS=(
    "src"
    "src/models"
    "src/data"
    "src/training"
    "src/evaluation"
    "src/utils"
    "scripts"
    "config"
    "data"
    "data/raw"
    "data/processed"
    "data/external"
    "experiments"
    "models"
    "reports"
    "reports/cornell_audit"
    "reports/figures"
    "reports/tables"
    "docs"
    "tests"
)

for dir in "${REQUIRED_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        echo -e "  ✓ ${GREEN}$dir${NC}"
    else
        echo -e "  ✗ ${RED}$dir (falta)${NC}"
        ((ERRORS++))
    fi
done

echo ""

# ============================================================================
# 2. ARCHIVOS RAÍZ ESENCIALES
# ============================================================================
echo -e "${YELLOW}[2] Archivos Raíz Esenciales${NC}"
echo "─────────────────────────────────────────────"

REQUIRED_FILES=(
    "README.md"
    "requirements.txt"
    ".gitignore"
    "bootstrap.sh"
    "check_system.sh"
    "check_project.sh"
    "check_python_deps.sh"
    "project_status.sh"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ]; then
        SIZE=$(du -h "$file" | cut -f1)
        echo -e "  ✓ ${GREEN}$file${NC} ($SIZE)"
    else
        echo -e "  ✗ ${RED}$file (falta)${NC}"
        ((ERRORS++))
    fi
done

# Verificar ejecutables
for script in bootstrap.sh check_system.sh check_project.sh check_python_deps.sh project_status.sh; do
    if [ -x "$script" ]; then
        echo -e "  ✓ ${GREEN}$script es ejecutable${NC}"
    else
        echo -e "  ⚠ ${YELLOW}$script no es ejecutable${NC}"
        echo "    Corregir: chmod +x $script"
        ((WARNINGS++))
    fi
done

echo ""

# ============================================================================
# 3. MÓDULOS PYTHON
# ============================================================================
echo -e "${YELLOW}[3] Módulos Python${NC}"
echo "─────────────────────────────────────────────"

PYTHON_INIT_FILES=(
    "src/__init__.py"
    "src/models/__init__.py"
    "src/data/__init__.py"
    "src/training/__init__.py"
    "src/evaluation/__init__.py"
    "src/utils/__init__.py"
    "tests/__init__.py"
)

for file in "${PYTHON_INIT_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo -e "  ✓ ${GREEN}$file${NC}"
    else
        echo -e "  ✗ ${RED}$file (falta)${NC}"
        ((ERRORS++))
    fi
done

echo ""

# ============================================================================
# 4. ARCHIVOS DE CONFIGURACIÓN
# ============================================================================
echo -e "${YELLOW}[4] Archivos de Configuración${NC}"
echo "─────────────────────────────────────────────"

CONFIG_FILES=(
    "config/default.yaml"
    "config/exp1_simple_rgb.yaml"
    "config/exp2_simple_rgbd.yaml"
    "config/exp3_resnet18_rgb_augment.yaml"
    "config/exp4_resnet18_rgbd.yaml"
)

for file in "${CONFIG_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo -e "  ✓ ${GREEN}$file${NC}"
    else
        echo -e "  ✗ ${RED}$file (falta)${NC}"
        ((ERRORS++))
    fi
done

# Validar YAML syntax
if command -v python3 &> /dev/null && [ -n "${VIRTUAL_ENV:-}" ]; then
    echo ""
    echo "Validando sintaxis YAML..."
    for file in "${CONFIG_FILES[@]}"; do
        if [ -f "$file" ]; then
            if python3 << EOF
import yaml
try:
    with open('$file', 'r') as f:
        yaml.safe_load(f)
    print("  ✓ \033[0;32m$file (sintaxis válida)\033[0m")
except Exception as e:
    print(f"  ✗ \033[0;31m$file (error: {e})\033[0m")
    exit(1)
EOF
            then
                :
            else
                ((ERRORS++))
            fi
        fi
    done
fi

echo ""

# ============================================================================
# 5. DOCUMENTACIÓN
# ============================================================================
echo -e "${YELLOW}[5] Documentación${NC}"
echo "─────────────────────────────────────────────"

DOC_FILES=(
    "docs/REGISTRO_RECONSTRUCCION.md"
    "docs/EXPERIMENTS.md"
    "docs/API.md"
    "data/README.md"
    "experiments/README.md"
    "models/README.md"
)

for file in "${DOC_FILES[@]}"; do
    if [ -f "$file" ]; then
        LINES=$(wc -l < "$file")
        echo -e "  ✓ ${GREEN}$file${NC} ($LINES líneas)"
    else
        echo -e "  ⚠ ${YELLOW}$file (falta)${NC}"
        ((WARNINGS++))
    fi
done

echo ""

# ============================================================================
# 6. ARCHIVOS .gitkeep
# ============================================================================
echo -e "${YELLOW}[6] Archivos .gitkeep${NC}"
echo "─────────────────────────────────────────────"

GITKEEP_FILES=(
    "data/raw/.gitkeep"
    "data/processed/.gitkeep"
    "data/external/.gitkeep"
    "experiments/.gitkeep"
    "models/.gitkeep"
    "reports/figures/.gitkeep"
    "reports/tables/.gitkeep"
)

GITKEEP_COUNT=0
for file in "${GITKEEP_FILES[@]}"; do
    if [ -f "$file" ]; then
        GITKEEP_COUNT=$((GITKEEP_COUNT + 1))
    fi
done

echo -e "  Encontrados: ${GREEN}$GITKEEP_COUNT/${#GITKEEP_FILES[@]}${NC}"
if [ $GITKEEP_COUNT -lt ${#GITKEEP_FILES[@]} ]; then
    echo -e "  ${YELLOW}⚠ Algunos .gitkeep faltan (no crítico)${NC}"
    ((WARNINGS++))
fi

echo ""

# ============================================================================
# 7. VALIDACIÓN GIT (si aplica)
# ============================================================================
echo -e "${YELLOW}[7] Repositorio Git${NC}"
echo "─────────────────────────────────────────────"

if [ -d ".git" ]; then
    echo -e "  ✓ ${GREEN}Repositorio Git inicializado${NC}"
    
    if [ -f ".gitignore" ]; then
        echo -e "  ✓ ${GREEN}.gitignore presente${NC}"
    else
        echo -e "  ✗ ${RED}.gitignore falta${NC}"
        ((ERRORS++))
    fi
    
    BRANCH=$(git branch --show-current 2>/dev/null || echo "sin rama")
    echo -e "  Rama actual: ${BLUE}$BRANCH${NC}"
    
    COMMITS=$(git rev-list --count HEAD 2>/dev/null || echo "0")
    echo -e "  Commits: ${BLUE}$COMMITS${NC}"
else
    echo -e "  ${YELLOW}⚠ No es un repositorio Git${NC}"
    echo "    Inicializar: git init"
    ((WARNINGS++))
fi

echo ""

# ============================================================================
# 8. DATOS Y MODELOS
# ============================================================================
echo -e "${YELLOW}[8] Datos y Modelos${NC}"
echo "─────────────────────────────────────────────"

# Contar archivos en data/
DATA_RAW_COUNT=$(find data/raw -type f ! -name '.gitkeep' 2>/dev/null | wc -l)
DATA_PROC_COUNT=$(find data/processed -type f ! -name '.gitkeep' 2>/dev/null | wc -l)
DATA_EXT_COUNT=$(find data/external -type f ! -name '.gitkeep' 2>/dev/null | wc -l)

echo -e "  Datos raw: ${BLUE}$DATA_RAW_COUNT archivos${NC}"
echo -e "  Datos processed: ${BLUE}$DATA_PROC_COUNT archivos${NC}"
echo -e "  Datos external: ${BLUE}$DATA_EXT_COUNT archivos${NC}"

if [ $DATA_PROC_COUNT -eq 0 ]; then
    echo -e "  ${YELLOW}⚠ No hay datos procesados. Preparar antes de entrenar.${NC}"
    ((WARNINGS++))
fi

# Contar modelos
MODEL_COUNT=$(find models -name "*.pth" -o -name "*.pt" 2>/dev/null | wc -l)
echo -e "  Modelos guardados: ${BLUE}$MODEL_COUNT${NC}"

# Contar checkpoints
CKPT_COUNT=$(find experiments -path "*/checkpoints/*" \( -name "*.pth" -o -name "*.pt" \) 2>/dev/null | wc -l)
echo -e "  Checkpoints: ${BLUE}$CKPT_COUNT${NC}"

if [ -d "reports/bench" ]; then
    BENCH_STATUS="presente"
else
    BENCH_STATUS="pendiente"
    ((WARNINGS++))
fi
echo -e "  Benchmark latencia: ${BLUE}$BENCH_STATUS${NC}"

echo ""

# ============================================================================
# 9. SCRIPTS DE EJECUCIÓN (opcional, pendientes de crear)
# ============================================================================
echo -e "${YELLOW}[9] Scripts de Ejecución${NC}"
echo "─────────────────────────────────────────────"

SCRIPT_FILES=(
    "scripts/train.py"
    "scripts/evaluate.py"
    "scripts/predict.py"
    "scripts/run_experiment.py"
    "scripts/select_best_epoch.py"
    "scripts/summarize_results.py"
    "scripts/generate_tables.py"
    "scripts/generate_figures.py"
    "scripts/generate_qualitative_overlays.py"
    "scripts/benchmark_latency.py"
    "scripts/validate_artifacts.py"
    "scripts/regenerate_tfm_block.sh"
)

SCRIPT_COUNT=0
for file in "${SCRIPT_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo -e "  ✓ ${GREEN}$file${NC}"
        SCRIPT_COUNT=$((SCRIPT_COUNT + 1))
    else
        echo -e "  ⚠ ${YELLOW}$file (pendiente)${NC}"
    fi
done

if [ $SCRIPT_COUNT -lt 6 ]; then
    echo -e "  ${YELLOW}⚠ Scripts TFM incompletos${NC}"
    ((WARNINGS++))
fi

echo ""

# ============================================================================
# 10. TRAZABILIDAD TFM
# ============================================================================
echo -e "${YELLOW}[10] Trazabilidad TFM${NC}"
echo "─────────────────────────────────────────────"

if [ -f "docs/TRAZABILIDAD_TFM.md" ]; then
    echo -e "  ✓ ${GREEN}docs/TRAZABILIDAD_TFM.md${NC}"
else
    echo -e "  ✗ ${RED}docs/TRAZABILIDAD_TFM.md (falta)${NC}"
    ((ERRORS++))
fi

if [ -f "data/processed/cornell/splits/object_wise/train.csv" ] && [ -f "data/processed/cornell/splits/object_wise/val.csv" ]; then
    echo -e "  ✓ ${GREEN}CSV split object-wise detectados${NC}"
else
    echo -e "  ⚠ ${YELLOW}Split CSV object-wise no preparado${NC}"
    ((WARNINGS++))
fi

if [ -f "reports/cornell_audit/clean_idx_train_v2.txt" ] && [ -f "reports/cornell_audit/clean_idx_val.txt" ]; then
    echo -e "  ✓ ${GREEN}Archivos de auditoria Cornell presentes${NC}"
else
    echo -e "  ⚠ ${YELLOW}Faltan archivos de auditoria Cornell${NC}"
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
    echo -e "${GREEN}✓ PROYECTO ÍNTEGRO Y COMPLETO${NC}"
    echo ""
    echo "Todos los componentes del proyecto están en su lugar."
    echo "El proyecto está listo para desarrollo."
    exit 0
elif [ $ERRORS -eq 0 ]; then
    echo -e "${YELLOW}⚠ PROYECTO FUNCIONAL CON ADVERTENCIAS${NC}"
    echo ""
    echo -e "Advertencias: ${YELLOW}$WARNINGS${NC}"
    echo ""
    echo "La estructura básica está completa, pero hay elementos opcionales pendientes."
    exit 0
else
    echo -e "${RED}✗ PROYECTO INCOMPLETO${NC}"
    echo ""
    echo -e "Errores: ${RED}$ERRORS${NC}"
    echo -e "Advertencias: ${YELLOW}$WARNINGS${NC}"
    echo ""
    echo "Algunos componentes críticos faltan."
    echo "Ejecute ./bootstrap.sh para reconfigurar el proyecto."
    exit 1
fi
