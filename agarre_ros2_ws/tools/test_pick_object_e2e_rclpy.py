#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_pick_object_e2e_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""E2E probe for PICK object PRE_GRASP using existing rclpy reach test."""

from __future__ import annotations

import os
import subprocess
import sys


def main() -> int:
    cmd = [
        sys.executable,
        os.path.join(os.path.dirname(__file__), "test_pick_pregrasp_reach_rclpy.py"),
        "--discover-timeout",
        "8.0",
        "--result-timeout",
        "8.0",
        "--reach-timeout",
        "15.0",
        "--tol",
        "0.03",
    ]
    print("[INFO] running:", " ".join(cmd))
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
