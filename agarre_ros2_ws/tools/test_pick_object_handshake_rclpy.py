#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_pick_object_handshake_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Run 3x request/result handshake checks for PICK object contract."""

from __future__ import annotations

import os
import subprocess
import sys


def main() -> int:
    cmd = [
        sys.executable,
        os.path.join(os.path.dirname(__file__), "test_bridge_contract_rclpy.py"),
        "--requests",
        "3",
        "--start-request-id",
        "101",
        "--timeout",
        "15.0",
    ]
    print("[INFO] running:", " ".join(cmd))
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
