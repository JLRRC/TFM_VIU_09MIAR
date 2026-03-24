#!/usr/bin/env python3
"""
Monitor en tiempo real de la punta logica (rg2_tcp) respecto a la mesa.

Regla clave:
- La distancia vertical a la superficie solo se considera valida cuando la
  proyeccion XY del TCP cae dentro de la huella superior del tablero.
- Si el TCP esta fuera de esa huella, se informa "N/A" y cuanto falta para
  entrar en X/Y.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from dataclasses import dataclass

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


@dataclass
class TableGeometry:
    center_x: float
    center_y: float
    top_z: float
    size_x: float
    size_y: float

    @property
    def half_x(self) -> float:
        return 0.5 * self.size_x

    @property
    def half_y(self) -> float:
        return 0.5 * self.size_y

    def inside_top(self, x: float, y: float) -> bool:
        return abs(x - self.center_x) <= self.half_x and abs(y - self.center_y) <= self.half_y

    def outside_margins(self, x: float, y: float) -> tuple[float, float]:
        dx = max(0.0, abs(x - self.center_x) - self.half_x)
        dy = max(0.0, abs(y - self.center_y) - self.half_y)
        return dx, dy


class TipTableMonitor(Node):
    def __init__(self, tip_frame: str, world_frame: str) -> None:
        super().__init__("tip_table_clearance_monitor")
        self._tip_frame = tip_frame
        self._world_frame = world_frame
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

    def tip_world_xyz(self, timeout_sec: float = 0.1) -> tuple[float, float, float] | None:
        try:
            if not self._tf_buffer.can_transform(
                self._world_frame,
                self._tip_frame,
                Time(),
                timeout=Duration(seconds=timeout_sec),
            ):
                return None
            tr = self._tf_buffer.lookup_transform(self._world_frame, self._tip_frame, Time())
            t = tr.transform.translation
            return float(t.x), float(t.y), float(t.z)
        except Exception:
            return None


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Monitor de distancia punta-mesa")
    p.add_argument("--tip-frame", default="rg2_tcp", help="Frame TCP a monitorizar")
    p.add_argument("--world-frame", default="world", help="Frame global")
    p.add_argument("--hz", type=float, default=5.0, help="Frecuencia de impresion")
    p.add_argument("--duration", type=float, default=0.0, help="Duracion en segundos (0=infinito)")
    p.add_argument("--table-cx", type=float, default=-0.17, help="Centro X de mesa (world)")
    p.add_argument("--table-cy", type=float, default=0.0, help="Centro Y de mesa (world)")
    p.add_argument("--table-top-z", type=float, default=0.85, help="Cota Z superficie mesa (world)")
    p.add_argument("--table-size-x", type=float, default=0.768, help="Ancho X tablero")
    p.add_argument("--table-size-y", type=float, default=0.80, help="Ancho Y tablero")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if args.hz <= 0.0:
        print("ERROR: --hz debe ser > 0", file=sys.stderr)
        return 2

    table = TableGeometry(
        center_x=float(args.table_cx),
        center_y=float(args.table_cy),
        top_z=float(args.table_top_z),
        size_x=float(args.table_size_x),
        size_y=float(args.table_size_y),
    )

    rclpy.init()
    node = TipTableMonitor(tip_frame=args.tip_frame, world_frame=args.world_frame)
    stop = {"flag": False}

    def _stop_handler(_sig: int, _frame: object) -> None:
        stop["flag"] = True

    signal.signal(signal.SIGINT, _stop_handler)
    signal.signal(signal.SIGTERM, _stop_handler)

    period = 1.0 / args.hz
    start = time.time()
    next_tick = start

    print(
        "[TIP_MONITOR] "
        f"tip={args.tip_frame} world={args.world_frame} "
        f"table_center=({table.center_x:.3f},{table.center_y:.3f}) "
        f"table_size=({table.size_x:.3f},{table.size_y:.3f}) top_z={table.top_z:.3f}"
    )
    print(
        "[TIP_MONITOR] "
        "dist_superficie_valida solo cuando inside_top=true (TCP encima de la huella de mesa)."
    )

    try:
        while not stop["flag"]:
            now = time.time()
            if args.duration > 0.0 and (now - start) >= args.duration:
                break

            rclpy.spin_once(node, timeout_sec=0.02)
            if now < next_tick:
                continue
            next_tick = now + period

            xyz = node.tip_world_xyz(timeout_sec=0.05)
            if xyz is None:
                print("[TIP_MONITOR] tcp_tf=unavailable")
                continue

            x, y, z = xyz
            inside = table.inside_top(x, y)
            dx_out, dy_out = table.outside_margins(x, y)
            d_vertical = z - table.top_z

            if inside:
                print(
                    "[TIP_MONITOR] "
                    f"tcp=({x:.3f},{y:.3f},{z:.3f}) "
                    f"inside_top=true dist_superficie={d_vertical:+.4f}m ({d_vertical*100.0:+.2f}cm)"
                )
            else:
                edge = math.hypot(dx_out, dy_out)
                print(
                    "[TIP_MONITOR] "
                    f"tcp=({x:.3f},{y:.3f},{z:.3f}) "
                    "inside_top=false dist_superficie=N/A "
                    f"outside_x={dx_out:.4f}m outside_y={dy_out:.4f}m outside_xy={edge:.4f}m "
                    f"vertical_vs_plane={d_vertical:+.4f}m"
                )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
