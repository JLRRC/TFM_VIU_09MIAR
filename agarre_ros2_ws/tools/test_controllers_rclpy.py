#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_controllers_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Check required ros2_control controllers via rclpy service call."""

from __future__ import annotations

import argparse
import os
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rclpy.parameter import Parameter


REQUIRED = ("joint_trajectory_controller", "gripper_controller")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--service", default="/controller_manager/list_controllers")
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = Node("test_controllers_rclpy")
    if not node.has_parameter("use_sim_time"):
        node.declare_parameter("use_sim_time", True)
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

    rc = 1
    try:
        client = node.create_client(ListControllers, args.service)
        if not client.wait_for_service(timeout_sec=max(0.2, args.timeout)):
            print(f"[FAIL] servicio no disponible: {args.service}")
            return 1
        req = ListControllers.Request()
        fut = client.call_async(req)
        deadline = time.monotonic() + max(0.2, args.timeout)
        while rclpy.ok() and (time.monotonic() < deadline) and (not fut.done()):
            rclpy.spin_once(node, timeout_sec=0.1)

        if not fut.done():
            print(f"[FAIL] timeout esperando respuesta de {args.service}")
            return 1

        try:
            res = fut.result()
        except Exception as exc:
            print(f"[FAIL] llamada a {args.service} falló: {exc}")
            return 1

        states = {}
        for c in list(getattr(res, "controller", []) or []):
            name = str(getattr(c, "name", "") or "")
            state = str(getattr(c, "state", "") or "")
            if name:
                states[name] = state

        print("[INFO] controllers:")
        for name in sorted(states):
            print(f"  - {name}: {states[name]}")

        missing = [name for name in REQUIRED if name not in states]
        inactive = [name for name in REQUIRED if states.get(name, "") != "active"]
        if missing:
            print(f"[FAIL] faltan controladores requeridos: {','.join(missing)}")
            return 1
        if inactive:
            detail = ",".join(f"{name}:{states.get(name, 'n/a')}" for name in inactive)
            print(f"[FAIL] controladores no activos: {detail}")
            return 1

        print("[PASS] controladores requeridos activos")
        rc = 0
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:
        pass
    os._exit(rc)


if __name__ == "__main__":
    raise SystemExit(main())
