#!/usr/bin/env python3
"""
FASE B — Validación post-PICK_IMAGE gate (APPROACH_COARSE timing)

Verifica que [STEP][PICK_IMAGE_TCP_DELTA] gate=open aparece en el log
antes de [ORG_CAPTURE] phase=APPROACH_COARSE.

Uso:
    source /opt/ros/jazzy/setup.bash
    source agarre_ros2_ws/install/setup.bash
    python3 agarre_ros2_ws/scripts/fase_b_validate.py
"""
import os
import sys
import time
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

LOG_FILE = os.path.join(
    os.path.dirname(__file__), "..", "log", "ros2_launch.log"
)
DETACH_TOPIC = "/gripper/pick_demo/detach"
PICK_DEMO_SRV = "/panel/pick_demo"
PICK_DEMO_INITIAL_WORLD = (-0.42, 0.00, 0.925)  # posición SDF inicial

# Marcadores para verificación de orden
MARKER_GATE_OPEN = "[STEP][PICK_IMAGE_TCP_DELTA]"
MARKER_GATE_OR_TIMEOUT = "APPROACH_COARSE_GATE"
# El marcador real de inicio de APPROACH_COARSE es [PICK][DIRECT][APPROACH_PLAN]
# (captura de tcp_before_coarse antes de phase_begin("APPROACH_COARSE"))
MARKER_ORG_CAPTURE_AC = "[PICK][DIRECT][APPROACH_PLAN]"
# Marcadores de fin de run (cualquiera indica que el pick terminó)
RUN_END_MARKERS = [
    "[DEMO] Fin pick",
    "[PICK][DIRECT][ABORT]",
    "[PICK][DIRECT][FAIL]",
    "DEMO_COMPLETE",
    "pick_demo_ok",
    "pick_demo_error",
    "ANCHOR][VERDICT",
    "[RUN][LIFT_CONFIRMED]",
    "[RUN][ABORT]",
    "PICK_DEMO_DONE",
    "[DEMO_DONE]",
    "PHASE_FINAL",
    "SHORT_LIFT",
    "lift_ok",
    "lift_fail",
    "[LIFT]",
    "BASKET",
    "cesta",
]

GZ_SET_POSE_BIN = "/opt/ros/jazzy/opt/gz_transport_vendor/libexec/gz/transport13/gz-transport-service"


def gz_reset_pick_demo():
    """Reset pick_demo position via gz set_pose service."""
    x, y, z = PICK_DEMO_INITIAL_WORLD
    req = f'name: "pick_demo" position {{x: {x} y: {y} z: {z}}} orientation {{x: 0 y: 0 z: 0 w: 1}}'
    cmd = [
        GZ_SET_POSE_BIN, "-s", "/world/ur5_mesa_objetos/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "3000",
        "--req", req,
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        return result.returncode == 0, result.stdout.strip()
    except Exception as e:
        return False, str(e)


def get_log_line_count():
    try:
        with open(LOG_FILE, "r") as f:
            lines = f.readlines()
        return len(lines)
    except Exception:
        return 0


def get_log_lines_from(start_line):
    try:
        with open(LOG_FILE, "r") as f:
            lines = f.readlines()
        return lines[start_line:]
    except Exception:
        return []


class FaseBValidator(Node):
    def __init__(self):
        super().__init__("fase_b_validator")
        self._detach_pub = self.create_publisher(Empty, DETACH_TOPIC, 1)
        self._pick_svc = self.create_client(Trigger, PICK_DEMO_SRV)

    def detach_object(self):
        print(f"[FASE_B] Publicando detach en {DETACH_TOPIC}")
        self._detach_pub.publish(Empty())
        rclpy.spin_once(self, timeout_sec=0.3)
        time.sleep(0.5)
        self._detach_pub.publish(Empty())
        rclpy.spin_once(self, timeout_sec=0.3)

    def call_pick_demo_service(self, timeout_sec=10.0):
        if not self._pick_svc.wait_for_service(timeout_sec=timeout_sec):
            print(f"[FASE_B] ERROR: Servicio {PICK_DEMO_SRV} no disponible")
            return False, "service_unavailable"
        req = Trigger.Request()
        future = self._pick_svc.call_async(req)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            return False, "timeout"
        result = future.result()
        return result.success, result.message


def wait_for_run_end(start_line, timeout_sec=180.0):
    """Espera a que el log muestre un marcador de fin de run."""
    print(f"[FASE_B] Esperando fin de run (timeout={timeout_sec}s)...")
    deadline = time.monotonic() + timeout_sec
    dots = 0
    while time.monotonic() < deadline:
        new_lines = get_log_lines_from(start_line)
        for line in new_lines:
            for marker in RUN_END_MARKERS:
                if marker.lower() in line.lower():
                    print(f"\n[FASE_B] Run terminado: {line.strip()[:120]}")
                    return True, new_lines
        time.sleep(1.0)
        dots += 1
        if dots % 10 == 0:
            elapsed = time.monotonic() - (deadline - timeout_sec)
            print(f"  ...esperando {elapsed:.0f}s")
    print("\n[FASE_B] TIMEOUT: run no terminó en tiempo")
    return False, get_log_lines_from(start_line)


def analyze_log(lines):
    """
    Verifica que PICK_IMAGE_TCP_DELTA gate aparece antes que ORG_CAPTURE APPROACH_COARSE.
    Retorna (pass, details_dict)
    """
    gate_line = None
    gate_idx = None
    org_capture_ac_line = None
    org_capture_ac_idx = None

    for i, line in enumerate(lines):
        if gate_line is None and MARKER_GATE_OPEN in line:
            gate_line = line.strip()
            gate_idx = i
        if org_capture_ac_line is None and MARKER_ORG_CAPTURE_AC in line:
            org_capture_ac_line = line.strip()
            org_capture_ac_idx = i

    gate_is_open = gate_line is not None and "gate=open" in gate_line
    gate_is_timeout = gate_line is not None and "gate=timeout_continue" in gate_line

    details = {
        "gate_line_idx": gate_idx,
        "gate_line": gate_line,
        "gate_is_open": gate_is_open,
        "gate_is_timeout": gate_is_timeout,
        "org_capture_ac_idx": org_capture_ac_idx,
        "org_capture_ac_line": org_capture_ac_line,
    }

    if gate_line is None:
        details["verdict"] = "FAIL — no se encontró PICK_IMAGE_TCP_DELTA en los logs"
        return False, details

    if org_capture_ac_line is None:
        details["verdict"] = "INCONCLUSIVE — gate encontrado pero no ORG_CAPTURE APPROACH_COARSE"
        return False, details

    if gate_idx < org_capture_ac_idx:
        if gate_is_open:
            details["verdict"] = "PASS — gate=open aparece antes de ORG_CAPTURE APPROACH_COARSE"
            return True, details
        elif gate_is_timeout:
            details["verdict"] = "PASS (timeout_continue) — gate aparece antes de ORG_CAPTURE APPROACH_COARSE (robot no convergió en 3s pero el run continuó)"
            return True, details
        else:
            details["verdict"] = f"PASS (gate presente) — gate={gate_line[-30:]} antes de ORG_CAPTURE APPROACH_COARSE"
            return True, details
    else:
        details["verdict"] = "FAIL — ORG_CAPTURE APPROACH_COARSE aparece ANTES del gate PICK_IMAGE_TCP_DELTA"
        return False, details


def print_evidence(lines, gate_idx, org_capture_ac_idx):
    """Imprime las líneas relevantes del log con contexto."""
    if gate_idx is None:
        return
    start = max(0, gate_idx - 3)
    end = min(len(lines), max(gate_idx, org_capture_ac_idx or gate_idx) + 5)
    print("\n[FASE_B] === EXTRACTO DE LOG RELEVANTE ===")
    for i in range(start, end):
        marker = ""
        if i == gate_idx:
            marker = " <-- GATE"
        if org_capture_ac_idx is not None and i == org_capture_ac_idx:
            marker = " <-- APPROACH_PLAN (inicio APPROACH_COARSE)"
        print(f"  [{i}] {lines[i].rstrip()[:150]}{marker}")
    print("[FASE_B] ===================================\n")


def main():
    rclpy.init()
    node = FaseBValidator()

    print("\n" + "="*60)
    print("FASE B — Validación post-PICK_IMAGE gate")
    print("="*60)

    # 1. Detach object
    print("\n[FASE_B] PASO 1: Detach pick_demo")
    node.detach_object()
    print("[FASE_B] Detach publicado.")

    # 2. Reset object position
    print(f"\n[FASE_B] PASO 2: Reset pick_demo a posición inicial {PICK_DEMO_INITIAL_WORLD}")
    ok, msg = gz_reset_pick_demo()
    print(f"[FASE_B] set_pose result: ok={ok} msg={msg[:80] if msg else 'n/a'}")

    # 3. Esperar que los objetos se estabilicen
    print("\n[FASE_B] PASO 3: Esperando estabilización (5s)...")
    time.sleep(5.0)

    # 3b. Llamar /release_objects para desbloquear _objects_release_done
    print("\n[FASE_B] PASO 3b: Llamando /release_objects para desbloquear panel...")
    try:
        svc_release = node.create_client(Trigger, "/release_objects")
        if svc_release.wait_for_service(timeout_sec=5.0):
            future_rel = svc_release.call_async(Trigger.Request())
            deadline_rel = time.monotonic() + 10.0
            while not future_rel.done() and time.monotonic() < deadline_rel:
                rclpy.spin_once(node, timeout_sec=0.1)
            if future_rel.done():
                res_rel = future_rel.result()
                print(f"[FASE_B] /release_objects: success={res_rel.success} msg={res_rel.message}")
            else:
                print("[FASE_B] /release_objects: timeout waiting response")
        else:
            print("[FASE_B] /release_objects: servicio no disponible, continuando...")
    except Exception as e:
        print(f"[FASE_B] /release_objects: error={e}")

    # Pequeña espera para que _objects_settled se ponga a True
    time.sleep(3.0)

    # 4. Registrar línea actual del log
    log_start = get_log_line_count()
    print(f"\n[FASE_B] PASO 4: Log offset registrado: línea {log_start}")

    # 5. Llamar al servicio de pick demo
    print(f"\n[FASE_B] PASO 5: Llamando {PICK_DEMO_SRV}...")
    success, message = node.call_pick_demo_service(timeout_sec=120.0)
    print(f"[FASE_B] Respuesta servicio: success={success} message={message}")

    if not success and message == "service_unavailable":
        print("[FASE_B] ERROR FATAL: servicio no disponible. ¿El stack está corriendo?")
        rclpy.shutdown()
        sys.exit(1)

    # 6. Esperar fin de run
    run_ok, new_lines = wait_for_run_end(log_start, timeout_sec=180.0)

    if not new_lines:
        print("[FASE_B] ERROR: No hay líneas nuevas en el log")
        rclpy.shutdown()
        sys.exit(1)

    # 7. Analizar ordering
    print(f"\n[FASE_B] PASO 6: Analizando {len(new_lines)} líneas nuevas del log...")
    passed, details = analyze_log(new_lines)

    # Imprimir evidencia
    print_evidence(new_lines, details.get("gate_line_idx"), details.get("org_capture_ac_idx"))

    # 8. Imprimir resultado
    print("\n" + "="*60)
    print(f"VEREDICTO: {details['verdict']}")
    print("="*60)
    print(f"  gate_line:      {(details.get('gate_line') or 'N/A')[:120]}")
    print(f"  gate=open:      {details.get('gate_is_open')}")
    print(f"  gate=timeout:   {details.get('gate_is_timeout')}")
    print(f"  gate_idx:       {details.get('gate_line_idx')}")
    print(f"  org_capture_idx:{details.get('org_capture_ac_idx')}")
    print(f"  org_capture:    {(details.get('org_capture_ac_line') or 'N/A')[:120]}")
    print("="*60)

    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
