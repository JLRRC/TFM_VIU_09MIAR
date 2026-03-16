#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_attach_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Attach pipe probe without ros2 CLI.

PASS (strict): if panel log confirms carried/attached transition.
PASS (diagnostic): if attach publish succeeds and --publish-only-ok is set.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty


class AttachProbe(Node):
    def __init__(self, attach_topic: str) -> None:
        super().__init__("test_attach_rclpy")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.attach_topic = attach_topic
        self.attach_pub = self.create_publisher(Empty, attach_topic, qos)


def _read_text_slice(path: Path, start: int) -> str:
    try:
        with path.open("rb") as fh:
            fh.seek(max(0, start))
            data = fh.read()
        return data.decode("utf-8", errors="ignore")
    except Exception:
        return ""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--object", required=True, help="Object model name (e.g. cyl_gray)")
    parser.add_argument("--attach-prefix", default="/gripper")
    parser.add_argument("--discover-timeout", type=float, default=3.0)
    parser.add_argument("--timeout", type=float, default=4.0)
    parser.add_argument("--log", default="log/ros2_launch.log")
    parser.add_argument(
        "--publish-only-ok",
        action="store_true",
        help="Allow PASS when attach message is published and topic has subscribers.",
    )
    parser.add_argument(
        "--allow-no-subscriber-fallback",
        action="store_true",
        help="PASS in diagnostic mode when no attach subscriber exists (logical attach fallback path).",
    )
    args = parser.parse_args()

    obj = str(args.object or "").strip()
    if not obj:
        print("[FAIL] object vacío")
        return 1
    attach_topic = f"{str(args.attach_prefix).rstrip('/')}/{obj}/attach"
    log_path = Path(args.log)
    if not log_path.is_absolute():
        log_path = Path(__file__).resolve().parents[1] / log_path

    baseline_size = 0
    if log_path.exists():
        try:
            baseline_size = log_path.stat().st_size
        except Exception:
            baseline_size = 0

    rclpy.init()
    node = AttachProbe(attach_topic)
    rc = 1
    try:
        deadline = time.monotonic() + max(0.5, float(args.discover_timeout))
        subs = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            subs = int(node.count_subscribers(attach_topic))
            if subs > 0:
                break
        pubs = int(node.count_publishers(attach_topic))
        print(f"[INFO] topic={attach_topic} pub_count={pubs} sub_count={subs}")
        if subs <= 0:
            if args.allow_no_subscriber_fallback:
                print("[WARN] attach topic sin subscriptores (fallback lógico habilitado)")
                print("[PASS] no-subscriber fallback diagnostic")
                return 0
            print("[FAIL] attach topic sin subscriptores")
            return 1

        node.attach_pub.publish(Empty())
        print(f"[INFO] published attach object={obj}")

        carried_re = re.compile(
            rf"\[OBJECTS\]\s+state\s+{re.escape(obj)}\s+->\s+(CARRIED|GRASPED)",
            re.IGNORECASE,
        )
        attach_re = re.compile(
            rf"\[ATTACH\].*attach_name={re.escape(obj)}",
            re.IGNORECASE,
        )

        saw_carried = False
        saw_attach = False
        wait_deadline = time.monotonic() + max(0.2, float(args.timeout))
        while time.monotonic() < wait_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if log_path.exists():
                chunk = _read_text_slice(log_path, baseline_size)
                if chunk:
                    if carried_re.search(chunk):
                        saw_carried = True
                    if attach_re.search(chunk):
                        saw_attach = True
                    if saw_carried:
                        break
            time.sleep(0.05)

        print(f"[CHECK] saw_attach_log={str(saw_attach).lower()}")
        print(f"[CHECK] saw_carried_log={str(saw_carried).lower()}")

        if saw_carried:
            print("[PASS] attach confirmado por transición de estado (CARRIED/GRASPED)")
            rc = 0
            return rc

        if args.publish_only_ok:
            print("[PASS] attach publish OK (modo diagnóstico)")
            rc = 0
            return rc

        print("[FAIL] no se confirmó attach en logs")
        return 1
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    return rc


if __name__ == "__main__":
    sys.exit(main())
