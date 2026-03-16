#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_attach_range_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Validate attach range geometry with rclpy (no ros2 CLI)."""

from __future__ import annotations

import argparse
import math
import sys
from typing import Tuple

import rclpy
from rclpy.node import Node


def _compute_attach_delta(
    *,
    tcp: Tuple[float, float, float],
    obj_center: Tuple[float, float, float],
    obj_height: float,
    z_ref_mode: str,
    z_clearance: float,
) -> Tuple[float, float, float, float, float]:
    obj_center_z = float(obj_center[2])
    obj_top_z = obj_center_z + (max(0.0, float(obj_height)) * 0.5)
    mode = str(z_ref_mode or "center").strip().lower()
    if mode not in ("center", "top"):
        mode = "center"
    obj_ref_z = (obj_center_z if mode == "center" else obj_top_z) + float(z_clearance)
    dx = float(obj_center[0]) - float(tcp[0])
    dy = float(obj_center[1]) - float(tcp[1])
    dz = float(obj_ref_z) - float(tcp[2])
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    return dx, dy, dz, dist, obj_ref_z


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("synthetic",), default="synthetic")
    parser.add_argument("--xy-tol", type=float, default=0.02)
    parser.add_argument("--z-tol", type=float, default=0.03)
    parser.add_argument("--z-ref-mode", choices=("center", "top"), default="center")
    parser.add_argument("--z-clearance", type=float, default=0.0)
    parser.add_argument("--tcp-x", type=float, default=0.430)
    parser.add_argument("--tcp-y", type=float, default=0.000)
    parser.add_argument("--tcp-z", type=float, default=0.080)
    parser.add_argument("--obj-x", type=float, default=0.430)
    parser.add_argument("--obj-y", type=float, default=0.000)
    parser.add_argument("--obj-z", type=float, default=0.070)
    parser.add_argument("--obj-height", type=float, default=0.10)
    args = parser.parse_args()

    rclpy.init(args=None)
    node: Node = rclpy.create_node("test_attach_range_rclpy")
    try:
        tcp = (args.tcp_x, args.tcp_y, args.tcp_z)
        obj_center = (args.obj_x, args.obj_y, args.obj_z)
        dx, dy, dz, dist, obj_ref_z = _compute_attach_delta(
            tcp=tcp,
            obj_center=obj_center,
            obj_height=args.obj_height,
            z_ref_mode=args.z_ref_mode,
            z_clearance=args.z_clearance,
        )
        in_range = (
            abs(dx) <= float(args.xy_tol)
            and abs(dy) <= float(args.xy_tol)
            and abs(dz) <= float(args.z_tol)
        )
        obj_top_z = float(args.obj_z) + (max(0.0, float(args.obj_height)) * 0.5)
        print(
            "[ATTACH_RANGE] "
            f"mode={args.mode} z_ref_mode={args.z_ref_mode} z_clearance={args.z_clearance:.3f} "
            f"tcp=({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f}) "
            f"obj_center=({obj_center[0]:.3f},{obj_center[1]:.3f},{obj_center[2]:.3f}) "
            f"obj_top_z={obj_top_z:.3f} obj_ref_z={obj_ref_z:.3f} "
            f"dx={dx:.3f} dy={dy:.3f} dz={dz:.3f} dist={dist:.3f} "
            f"thresh=(xy={args.xy_tol:.3f},z={args.z_tol:.3f}) in_range={str(in_range).lower()}"
        )
        if in_range:
            print("[PASS] attach range check is consistent and in-range")
            return 0
        print("[FAIL] attach range check out-of-range")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

