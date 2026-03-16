#!/bin/bash
# Script de monitoreo del entrenamiento
# Uso: ./monitor_training.sh

LOG_FILE=$(ls -t logs/training_*.log 2>/dev/null | head -1)

if [ -z "$LOG_FILE" ]; then
    echo "❌ No se encontró archivo de log"
    exit 1
fi

echo "=========================================="
echo "MONITOREO DE ENTRENAMIENTO"
echo "=========================================="
echo "Archivo de log: $LOG_FILE"
echo ""

# Ver últimas líneas del log
echo "--- Últimas 20 líneas del log ---"
tail -20 "$LOG_FILE"
echo ""

# Contar experimentos completados
echo "--- Progreso ---"
if [ -d "experiments" ]; then
    total_seeds=$(find experiments -type f -name "metrics.csv" 2>/dev/null | wc -l)
    echo "Seeds completados: $total_seeds / 10"
    
    for exp in experiments/EXP*; do
        if [ -d "$exp" ]; then
            exp_name=$(basename "$exp")
            seeds=$(find "$exp" -type f -name "metrics.csv" 2>/dev/null | wc -l)
            echo "  - $exp_name: $seeds seeds"
        fi
    done
else
    echo "Aún no se han creado carpetas de experimentos"
fi

echo ""
echo "--- Procesos activos ---"
pgrep -fa "run_experiment.py\|train.py" | head -5 || echo "No hay procesos de entrenamiento activos"

echo ""
echo "=========================================="
echo "Para ver el log en tiempo real:"
echo "  tail -f $LOG_FILE"
echo ""
echo "Para verificar uso de GPU:"
echo "  nvidia-smi"
echo "=========================================="
