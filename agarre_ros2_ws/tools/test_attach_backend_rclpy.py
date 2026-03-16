#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_attach_backend_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Validate attach backend availability and relay behavior."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty


class AttachProbe(Node):
    def __init__(self, obj_name: str):
        super().__init__("test_attach_backend_rclpy")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.obj_name = obj_name
        self.attach_topic = f"/gripper/{obj_name}/attach"
        self.drop_detach_topic = f"/drop_anchor/{obj_name}/detach"
        self.qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.attach_pub = self.create_publisher(Empty, self.attach_topic, self.qos)
        self.drop_sub = self.create_subscription(Empty, self.drop_detach_topic, self._on_drop_detach, self.qos)
        self.drop_msgs = 0

    def _on_drop_detach(self, _msg: Empty) -> None:
        self.drop_msgs += 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--object", default="cyl_gray")
    parser.add_argument("--discover-timeout", type=float, default=3.0)
    parser.add_argument("--relay-timeout", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = AttachProbe(args.object)
    rc = 1
    try:
        deadline = time.monotonic() + max(0.5, args.discover_timeout)
        sub_count = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            sub_count = int(node.count_subscribers(node.attach_topic))
            if sub_count > 0:
                break

        print(
            f"[INFO] attach_topic={node.attach_topic} attach_subscribers={sub_count}"
        )
        if sub_count <= 0:
            print(f"[FAIL] attach backend missing on {node.attach_topic}")
            return 1

        baseline = int(node.drop_msgs)
        node.attach_pub.publish(Empty())
        print(
            f"[INFO] published attach trigger topic={node.attach_topic} expecting relay={node.drop_detach_topic}"
        )

        deadline = time.monotonic() + max(0.5, args.relay_timeout)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if int(node.drop_msgs) > baseline:
                break

        relayed = int(node.drop_msgs) > baseline
        print(
            f"[CHECK] relay_received={str(relayed).lower()} drop_detach_msgs={node.drop_msgs - baseline} "
            f"topic={node.drop_detach_topic}"
        )
        if not relayed:
            print("[FAIL] attach backend did not relay to drop_anchor detach")
            return 1

        print("[PASS] attach backend present and relay works")
        rc = 0
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
