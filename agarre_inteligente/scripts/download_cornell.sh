#!/bin/bash
# Script para descargar y preparar el Cornell Grasping Dataset
# Autor: Reconstrucción TFM agarre_inteligente
# Fecha: 2026-03-06

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

RAW_DIR="data/raw/cornell"
TEMP_DIR="/tmp/cornell_download"
PROCESSED_DIR="data/processed/cornell"

echo "=========================================="
echo "  Descarga Cornell Grasping Dataset"
echo "=========================================="
echo ""

# Colores
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Función para imprimir con color
print_status() {
    echo -e "${GREEN}[OK]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 1. Crear directorios
print_status "Creando directorios..."
mkdir -p "$RAW_DIR"
mkdir -p "$TEMP_DIR"
mkdir -p "$PROCESSED_DIR/splits/object_wise"

# 2. Descargar dataset
DATASET_URL="http://pr.cs.cornell.edu/grasping/rect_data/data.php"
DOWNLOAD_FILE="$TEMP_DIR/cornell_grasping.zip"

if [ -f "$DOWNLOAD_FILE" ]; then
    print_warning "Archivo ya descargado: $DOWNLOAD_FILE"
else
    print_status "Descargando Cornell Grasping Dataset..."
    echo "URL: $DATASET_URL"
    echo ""
    
    # El dataset está en un archivo comprimido
    # URL real del archivo (puede variar, verificar en la web oficial)
    DIRECT_URL="http://www.cs.cornell.edu/~asaxena/grasping/rectdata/data.php"
    
    print_warning "NOTA: Cornell dataset requiere descarga manual desde:"
    echo "      http://pr.cs.cornell.edu/grasping/rect_data/data.php"
    echo ""
    echo "Opciones:"
    echo "  1) Descarga manual y coloca en: $TEMP_DIR/cornell_grasping.zip"
    echo "  2) Intento de descarga automática (puede fallar por protección web)"
    echo ""
    
    read -p "¿Intentar descarga automática? (s/N): " -n 1 -r
    echo ""
    
    if [[ $REPLY =~ ^[Ss]$ ]]; then
        # Intentar con wget
        if command -v wget &> /dev/null; then
            wget -O "$DOWNLOAD_FILE" \
                --user-agent="Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36" \
                "http://www.cs.cornell.edu/~asaxena/grasping/rectdata/data.php" \
                || print_error "Descarga falló. Prueba descarga manual."
        # Intentar con curl
        elif command -v curl &> /dev/null; then
            curl -L -o "$DOWNLOAD_FILE" \
                -A "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36" \
                "http://www.cs.cornell.edu/~asaxena/grasping/rectdata/data.php" \
                || print_error "Descarga falló. Prueba descarga manual."
        else
            print_error "No se encuentra wget ni curl. Instala: sudo apt install wget"
            exit 1
        fi
    else
        print_warning "Coloca manualmente el archivo descargado en:"
        echo "  $DOWNLOAD_FILE"
        echo ""
        read -p "Presiona ENTER cuando el archivo esté listo..." dummy
    fi
fi

# Verificar que el archivo existe
if [ ! -f "$DOWNLOAD_FILE" ]; then
    print_error "No se encuentra el archivo: $DOWNLOAD_FILE"
    print_warning "Descarga manualmente desde: $DATASET_URL"
    exit 1
fi

# 3. Extraer dataset
print_status "Extrayendo archivos..."
cd "$TEMP_DIR"

if [ "${DOWNLOAD_FILE##*.}" = "zip" ]; then
    unzip -q "$DOWNLOAD_FILE" -d extracted/ || print_error "Error al extraer ZIP"
elif [ "${DOWNLOAD_FILE##*.}" = "gz" ] || [ "${DOWNLOAD_FILE##*.}" = "tgz" ]; then
    tar -xzf "$DOWNLOAD_FILE" -C extracted/ || print_error "Error al extraer TAR.GZ"
else
    print_error "Formato desconocido: $DOWNLOAD_FILE"
    exit 1
fi

# 4. Organizar archivos
print_status "Organizando estructura de archivos..."

# El dataset Cornell suele venir con esta estructura:
# - Carpetas numeradas (01, 02, ...) con imágenes pcd<NUM>r.png (RGB) y pcd<NUM>d.png (depth)
# - Archivos pcd<NUM>cpos.txt con anotaciones

cd "$PROJECT_ROOT"

# Buscar directorio extraído
EXTRACTED_BASE=$(find "$TEMP_DIR/extracted" -maxdepth 2 -type d -name "[0-9][0-9]" | head -1 | xargs dirname)

if [ -z "$EXTRACTED_BASE" ]; then
    print_error "No se encontró la estructura esperada del dataset"
    print_warning "Contenido de $TEMP_DIR/extracted:"
    ls -la "$TEMP_DIR/extracted/"
    exit 1
fi

print_status "Dataset encontrado en: $EXTRACTED_BASE"

# Copiar archivos al directorio raw
cp -r "$EXTRACTED_BASE"/* "$RAW_DIR/"

print_status "Archivos copiados a: $RAW_DIR"

# 5. Contar archivos
NUM_OBJECTS=$(find "$RAW_DIR" -maxdepth 1 -type d -name "[0-9][0-9]" | wc -l)
NUM_IMAGES=$(find "$RAW_DIR" -type f -name "*r.png" | wc -l)

print_status "Dataset Cornell descargado:"
echo "  - Objetos: $NUM_OBJECTS"
echo "  - Imágenes RGB: $NUM_IMAGES"

# 6. Ejecutar script de procesamiento
print_status "Ejecutando script de procesamiento..."

if [ -f "scripts/prepare_cornell_csv.py" ]; then
    source venv/bin/activate
    python3 scripts/prepare_cornell_csv.py --raw-dir "$RAW_DIR" --out-dir "$PROCESSED_DIR"
    print_status "CSVs generados en: $PROCESSED_DIR/splits/object_wise/"
else
    print_warning "Script prepare_cornell_csv.py no encontrado. Ejecutar manualmente."
fi

# 7. Limpiar temporales
print_status "Limpiando archivos temporales..."
read -p "¿Eliminar archivos temporales de $TEMP_DIR? (s/N): " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Ss]$ ]]; then
    rm -rf "$TEMP_DIR"
    print_status "Temporales eliminados"
fi

echo ""
echo "=========================================="
echo "  Descarga completada"
echo "=========================================="
print_status "Dataset Cornell disponible en: $RAW_DIR"
print_status "CSVs procesados en: $PROCESSED_DIR/splits/object_wise/"
echo ""
print_status "Siguiente paso: ejecutar experimentos con datos reales"
echo "  ./scripts/run_experiment.py --config config/exp1_simple_rgb.yaml"
echo ""
