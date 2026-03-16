#!/bin/bash

# Script de prueba automatizada para AUTO TUNE y PICK MESA
# Simula clics en los botones del panel Qt

set -x

LOG_FILE="/home/laboratorio/TFM/agarre_ros2_ws/log/automation_test.log"
PANEL_LOG="/home/laboratorio/TFM/agarre_ros2_ws/log/panel_phase_test.log"

{
    echo "[AUTOMATION] Iniciando pruebas automáticas..."
    date
    
    # Esperar a que el panel esté visible (5 segundos)
    sleep 5
    
    # Obtener ID de la ventana del panel
    WINDOW_ID=$(xdotool search --name "UR5" --name "Panel" 2>/dev/null | head -1)
    
    if [ -z "$WINDOW_ID" ]; then
        echo "[ERROR] No se encontró ventana del panel"
        exit 1
    fi
    
    echo "[OK] Ventana encontrada: $WINDOW_ID"
    
    # Activar ventana
    xdotool windowactivate $WINDOW_ID
    sleep 1
    
    # --- PRUEBA 1: AUTO TUNE ---
    echo ""
    echo "[TEST 1] AUTO TUNE (MoveIt) - Intento 1"
    echo "Buscando botón AUTO TUNE..."
    
    # Intentar hacer clic en el botón (aproximadamente centrado)
    xdotool search --name "AUTO TUNE" windowfocus click 1 2>/dev/null || \
    xdotool windowfocus $WINDOW_ID && sleep 0.5 && xdotool key Tab && sleep 0.2 && xdotool key Return
    
    sleep 120
    
    # Registrar resultado
    if grep -q "AUTO_TUNE.*PASS\|FRONT_LEFT.*PASS.*FRONT_RIGHT.*PASS" "$PANEL_LOG"; then
        echo "[PASS] AUTO TUNE completado exitosamente"
    else
        echo "[FAIL] AUTO TUNE falló o no completó correctamente"
    fi
    
    echo ""
    echo "[TEST 2] PICK MESA"
    echo "Presionando botón PICK MESA..."
    
    xdotool windowfocus $WINDOW_ID && sleep 0.5 && xdotool key Tab && sleep 0.2 && xdotool key Return
    
    sleep 60
    
    # Registrar resultado
    if grep -q "PICK.*completado\|RELEASED" "$PANEL_LOG"; then
        echo "[PASS] PICK MESA completado exitosamente"
    else
        echo "[FAIL] PICK MESA falló"
    fi
    
    echo ""
    echo "[AUTOMATION] Pruebas completadas"
    date
    
} | tee -a "$LOG_FILE"

echo "[SUMMARY] Log guardado en: $LOG_FILE"
