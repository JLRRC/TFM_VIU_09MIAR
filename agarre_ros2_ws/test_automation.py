#!/usr/bin/env python3
"""
Script de prueba automatizada para AUTO TUNE y PICK MESA.
Usa ROS2 para disparar acciones desde línea de comando sin necesidad de GUI.
"""

import sys
import time
import subprocess
from pathlib import Path

# Agregar workspace a path de Python
sys.path.insert(0, '/home/laboratorio/TFM/agarre_ros2_ws/install/lib/python3.12/site-packages')

def log(msg, level="INFO"):
    """Log con timestamp."""
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] [{level}] {msg}", flush=True)

def wait_for_ready(timeout_sec=60):
    """Espera a que el panel esté en STATE READY."""
    log("Esperando que el sistema esté listo...", "WAIT")
    start = time.time()
    panel_log = Path("/home/laboratorio/TFM/agarre_ros2_ws/log/panel_phase_test.log")
    
    while time.time() - start < timeout_sec:
        try:
            if panel_log.exists():
                with open(panel_log) as f:
                    content = f.read()
                    if "STATE READY" in content:
                        log("Sistema en STATE READY ✓", "OK")
                        return True
        except Exception as e:
            pass
        time.sleep(1)
    
    log(f"Timeout esperando STATE READY después de {timeout_sec}s", "ERROR")
    return False

def run_test_suite():
    """Ejecuta suite de pruebas."""
    panel_log = Path("/home/laboratorio/TFM/agarre_ros2_ws/log/panel_phase_test.log")
    
    # Limpiar log anterior
    log("Limpiando logs de panel...", "INFO")
    if panel_log.exists():
        panel_log.write_text("")
    
    # Esperar a que panel esté listo
    if not wait_for_ready():
        return False
    
    log("=" * 60, "BANNER")
    log("INICIANDO SUITE DE PRUEBAS", "BANNER")
    log("=" * 60, "BANNER")
    
    # Prueba 1: AUTO TUNE
    log("", "INFO")
    log("[TEST 1] AUTO TUNE (MoveIt) - Intento 1", "TEST")
    log("Disparando AUTO TUNE via ROS2...", "ACTION")
    
    # Usar ros2 service para disparar si existe, sino usar xte o similar
    try:
        # Intentar disparar via xdotool si GUI está disponible
        result = subprocess.run(
            ["bash", "-c", "xdotool search --name 'Panel' windowactivate 2>/dev/null"],
            timeout=5,
            capture_output=True
        )
        
        if result.returncode == 0:
            log("Panel GUI encontrado, usando xdotool", "INFO")
            # Enviar keystroke para Tab (navegación) y Return (click)
            subprocess.run(["xdotool", "key", "Tab", "Tab", "Return"], timeout=5)
    except Exception as e:
        log(f"xdotool no disponible: {e}", "WARN")
    
    # Esperar a que AUTO TUNE se complete (~100-150 segundos máximo)
    log("Esperando que AUTO TUNE complete (hasta 150s)...", "WAIT")
    start = time.time()
    timeout = 150
    test1_pass = False
    
    while time.time() - start < timeout:
        try:
            with open(panel_log) as f:
                content = f.read()
                # Buscar indicios de finalización
                if "TEST ROBOT completado" in content and "FRONT_LEFT.*PASS" in content:
                    log("AUTO TUNE completado ✓", "OK")
                    test1_pass = True
                    break
                elif "TEST_FAILED" in content:
                    log("AUTO TUNE falló", "FAIL")
                    break
        except:
            pass
        time.sleep(2)
    
    if not test1_pass:
        log("Timeout o fallo en AUTO TUNE", "FAIL")
    
    # Esperar un poco antes de prueba 2
    time.sleep(5)
    
    # Prueba 2: PICK MESA
    log("", "INFO")
    log("[TEST 2] PICK MESA",  "TEST")
    log("Disparando PICK MESA...", "ACTION")
    
    try:
        # Enviar keystroke para disparar siguiente botón
        subprocess.run(["xdotool", "key", "Tab", "Return"], timeout=5)
    except:
        pass
    
    log("Esperando que PICK MESA complete (hasta 90s)...", "WAIT")
    start = time.time()
    timeout = 90
    test2_pass = False
    
    while time.time() - start < timeout:
        try:
            with open(panel_log) as f:
                content = f.read()
                if "RELEASED" in content and "HOME_FINAL" in content:
                    log("PICK MESA completado ✓", "OK")
                    test2_pass = True
                    break
                elif "[PICK]" in content and "Paso" in content:
                    # Show progress
                    lines = content.split('\n')
                    latest_pick = [l for l in lines if "[PICK] Paso" in l]
                    if latest_pick:
                        log(f"Progreso: {latest_pick[-1]}", "PROGRESS")
        except:
            pass
        time.sleep(3)
    
    if not test2_pass:
        log("Timeout o fallo en PICK MESA", "FAIL")
    
    # Resumen
    log("", "INFO")
    log("=" * 60, "BANNER")
    log("RESUMEN DE PRUEBAS", "BANNER")
    log("=" * 60, "BANNER")
    log(f"[TEST 1] AUTO TUNE: {'PASS ✓' if test1_pass else 'FAIL ✗'}", "SUMMARY")
    log(f"[TEST 2] PICK MESA: {'PASS ✓' if test2_pass else 'FAIL ✗'}", "SUMMARY")
    log(f"Total: {'TODO PASADO ✓✓' if (test1_pass and test2_pass) else 'ALGUNOS FALLARON'}", "SUMMARY")
    log("=" * 60, "BANNER")
    
    return test1_pass and test2_pass

if __name__ == '__main__':
    success = run_test_suite()
    sys.exit(0 if success else 1)
