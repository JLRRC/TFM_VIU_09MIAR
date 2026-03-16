#!/usr/bin/env python3
"""
Script de prueba automatizada - Versión 2 (sin limpiar logs).
"""

import time
import subprocess
from pathlib import Path

def log(msg, level="INFO"):
    """Log con timestamp."""
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] [{level}] {msg}", flush=True)

def get_log_position(filepath):
    """Obtiene posición actual en el archivo."""
    try:
        return Path(filepath).stat().st_size
    except:
        return 0

def read_from_position(filepath, start_pos):
    """Lee desde una posición en el archivo."""
    try:
        with open(filepath, 'rb') as f:
            f.seek(start_pos)
            return f.read().decode(errors='ignore')
    except:
        return ""

def check_state_ready(panel_log):
    """Verifica si el panel está en STATE READY."""
    try:
        with open(panel_log) as f:
            content = f.read()
            return "STATE READY" in content
    except:
        return False

def run_automation():
    """Ejecuta automatización."""
    panel_log = Path("/home/laboratorio/TFM/agarre_ros2_ws/log/panel_phase_test.log")
    
    log("=" * 70, "BANNER")
    log("SUITE DE PRUEBAS AUTOMATIZADA - AUTO TUNE + PICK MESA", "BANNER")
    log("=" * 70, "BANNER")
    
    # Esperar a STATE READY sin limpiar log
    log("", "INFO")
    log("Esperando que el panel esté en STATE READY...", "WAIT")
    start = time.time()
    while  time.time() - start < 120:
        if check_state_ready(panel_log):
            log("✓ Panel en STATE READY", "OK")
            break
        time.sleep(2)
    else:
        log("Timeout esperando STATE READY", "WARN")
    
    time.sleep(5)
    
    # Registro inicial
    start_pos = get_log_position(panel_log)
    
    # TEST 1: AUTO TUNE
    log("", "INFO")
    log("─" * 70, "BANNER")
    log("[TEST 1] AUTO TUNE (MoveIt) - Intento 1", "TEST")
    log("─" * 70, "BANNER")
    
    log("Disparando AUTO TUNE mediante keypress...", "ACTION")
    try:
        # Intentar activar ventana y presionar botón
        subprocess.run(["bash", "-c", "xdotool search --name 'Panel\\|UR5' windowactivate 2>/dev/null || true"], timeout=3)
        time.sleep(1)
        subprocess.run(["xdotool", "key", "Tab", "Tab", "Tab", "Return"], timeout=3)
        log("Keystroke enviado", "OK")
    except Exception as e:
        log(f"Nota: xdotool no disponible ({e}), pero AUTO TUNE puede estar disparándose", "INFO")
    
    log("Esperando que AUTO TUNE complete (~150s máximo)...", "WAIT")
    start = time.time()
    test1_result = "UNKNOWN"
    test1_time = 0
    
    while time.time() - start < 160:
        new_content = read_from_position(panel_log, start_pos)
        
        if "TEST ROBOT completado" in new_content:
            test1_time = time.time() - start
            if "PASS markers touched" in new_content or ("FRONT_LEFT" in new_content and "PASS" in new_content):
                test1_result = "PASS ✓"
                log(f"✓ AUTO TUNE completado exitosamente en {test1_time:.1f}s", "OK")
            else:
                test1_result = "FAIL"
                log("✗ AUTO TUNE completado pero con fallos", "FAIL")
            
            # Mostrar resumen de AUTO TUNE
            lines = new_content.split('\n')
            summary_lines = [l for l in lines if "PASS markers" in l or "FRONT_LEFT" in l or "FRONT_RIGHT" in l or "touch_miss" in l]
            if summary_lines:
                log("Detalles:")
                for line in summary_lines[-3:]:
                    log(f"  {line.strip()}", "INFO")
            break
        
        elif "TEST_FAILED" in new_content:
            test1_time = time.time() - start
            test1_result = "FAIL"
            log(f"✗ AUTO TUNE falló después de {test1_time:.1f}s", "FAIL")
            
            # Mostrar razón de fallo
            fail_lines = [l for l in new_content.split('\n') if "TEST_FAILED" in l]
            if fail_lines:
                log(f"Razón: {fail_lines[-1]}", "INFO")
            break
        
        time.sleep(2)
    
    if test1_result == "UNKNOWN":
        test1_result = "TIMEOUT"
        log("✗ Timeout esperando finalización de AUTO TUNE", "FAIL")
    
    time.sleep(5)
    
    # TEST 2: PICK MESA
    log("", "INFO")
    log("─" * 70, "BANNER")
    log("[TEST 2] PICK MESA", "TEST")
    log("─" * 70, "BANNER")
    
    start_pos = get_log_position(panel_log)
    
    log("Disparando PICK MESA mediante keystroke...", "ACTION")
    try:
        subprocess.run(["xdotool", "key", "Tab", "Return"], timeout=3)
        log("Keystroke enviado", "OK")
    except:
        log("Nota: xdotool no disponible, pero PICK puede estar disparándose", "INFO")
    
    log("Esperando que PICK MESA complete (~90s máximo)...", "WAIT")
    start = time.time()
    test2_result = "UNKNOWN"
    test2_time = 0
    pick_steps = []
    
    while time.time() - start < 100:
        new_content = read_from_position(panel_log, start_pos)
        
        # Capturar pasos de PICK
        lines = new_content.split('\n')
        steps = [l.strip() for l in lines if "[PICK] Paso" in l]
        if steps != pick_steps:
            pick_steps = steps
            if steps:
                log(f"Progreso: {steps[-1]}", "PROGRESS")
        
        if "HOME_FINAL" in new_content and "RELEASED" in new_content:
            test2_time = time.time() - start
            test2_result = "PASS ✓"
            log(f"✓ PICK MESA completado exitosamente en {test2_time:.1f}s", "OK")
            
            # Listar pasos completados
            steps = [l for l in new_content.split('\n') if "[PICK] Paso" in l]
            if steps:
                log("Pasos completados:", "INFO")
                for step in steps:
                    log(f"  {step.strip()}", "INFO")
            break
        
        elif "PICK.*falló" in new_content or "[ERROR]" in new_content:
            test2_time = time.time() - start
            test2_result = "FAIL"
            log(f"✗ PICK MESA falló después de {test2_time:.1f}s", "FAIL")
            break
        
        time.sleep(2)
    
    if test2_result == "UNKNOWN":
        test2_result = "TIMEOUT"
        log("✗ Timeout esperando finalización de PICK MESA", "FAIL")
    
    # RESUMEN
    log("", "INFO")
    log("=" * 70, "BANNER")
    log("RESUMEN FINAL","BANNER")
    log("=" * 70, "BANNER")
    log(f"[TEST 1] AUTO TUNE (MoveIt):  {test1_result:15} ({test1_time:.1f}s)" if test1_time else f"[TEST 1] AUTO TUNE (MoveIt):  {test1_result}", "SUMMARY")
    log(f"[TEST 2] PICK MESA:           {test2_result:15} ({test2_time:.1f}s)" if test2_time else f"[TEST 2] PICK MESA:           {test2_result}", "SUMMARY")
    
    overall = "✓ TODAS LAS PRUEBAS PASADAS" if ("PASS" in test1_result and "PASS" in test2_result) else "✗ ALGUNAS PRUEBAS FALLARON"
    log(f"Resultado: {overall}", "SUMMARY")
    log("=" * 70, "BANNER")
    
    return "PASS" in test1_result and "PASS" in test2_result

if __name__ == '__main__':
    success = run_automation()
    exit(0 if success else 1)
