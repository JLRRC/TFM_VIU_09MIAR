#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_bridge_heartbeat_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Check ur5_moveit_bridge heartbeat without ros2 CLI."""

from __future__ import annotations

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class HeartbeatProbe(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("test_bridge_heartbeat_rclpy")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.topic = topic
        self.seq = 0
        self.last_mono = 0.0
        self.sub = self.create_subscription(Bool, topic, self._cb, qos)

    def _cb(self, _msg: Bool) -> None:
        self.seq += 1
        self.last_mono = time.monotonic()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/ur5_moveit_bridge/heartbeat")
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--max-age", type=float, default=1.2)
    args = parser.parse_args()

    rclpy.init()
    node = HeartbeatProbe(args.topic)
    rc = 1
    try:
        deadline = time.monotonic() + max(0.5, float(args.timeout))
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.seq > 0:
                break
        age = float("inf")
        if node.last_mono > 0.0:
            age = max(0.0, time.monotonic() - node.last_mono)
        print(
            f"[INFO] topic={args.topic} heartbeat_count={node.seq} "
            f"last_age={('inf' if age == float('inf') else f'{age:.2f}s')}"
        )
        if node.seq <= 0:
            print(f"[FAIL] no heartbeat on {args.topic}")
            return 1
        if age > float(args.max_age):
            print(
                f"[FAIL] stale heartbeat on {args.topic} age={age:.2f}s "
                f"max_age={float(args.max_age):.2f}s"
            )
            return 1
        print("[PASS] bridge heartbeat OK")
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
