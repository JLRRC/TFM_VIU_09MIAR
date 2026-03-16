#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_bridge_contract_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Validate request/result contract on /desired_grasp without ros2 CLI."""

from __future__ import annotations

import argparse
import ast
import json
import os
import sys
import time
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class ContractProbe(Node):
    def __init__(self, pose_topic: str, result_topic: str) -> None:
        super().__init__("test_bridge_contract_rclpy")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pose_topic = pose_topic
        self.result_topic = result_topic
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, qos)
        self.result_sub = self.create_subscription(String, result_topic, self._on_result, qos)
        self.last_payload = {}
        self.result_count = 0
        self.result_msgs: list[dict] = []

    def _on_result(self, msg: String) -> None:
        self.result_count += 1
        payload = {}
        text = str(msg.data or "")
        try:
            payload = json.loads(text)
        except Exception:
            try:
                dec = ast.literal_eval(text)
                if isinstance(dec, str):
                    payload = json.loads(dec)
            except Exception:
                payload = {}
        self.last_payload = payload if isinstance(payload, dict) else {}
        if isinstance(self.last_payload, dict):
            self.result_msgs.append(dict(self.last_payload))

    def wait_ros_time(self, timeout_sec: float) -> int:
        deadline = time.monotonic() + max(0.2, timeout_sec)
        while time.monotonic() < deadline:
            now_ns = int(self.get_clock().now().nanoseconds)
            if now_ns > 0:
                return now_ns
            rclpy.spin_once(self, timeout_sec=0.05)
        return int(self.get_clock().now().nanoseconds)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-topic", default="/desired_grasp")
    parser.add_argument("--result-topic", default="/desired_grasp/result")
    parser.add_argument("--discover-timeout", type=float, default=8.0)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--x", type=float, default=0.43)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.075)
    parser.add_argument("--requests", type=int, default=1)
    parser.add_argument("--start-request-id", type=int, default=101)
    args = parser.parse_args()

    rclpy.init()
    node = ContractProbe(args.pose_topic, args.result_topic)
    rc = 1
    try:
        deadline = time.monotonic() + max(1.0, args.discover_timeout)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.count_subscribers(args.pose_topic) > 0 and node.count_publishers(args.result_topic) > 0:
                break
        pose_subs = int(node.count_subscribers(args.pose_topic))
        result_pubs = int(node.count_publishers(args.result_topic))
        print(f"[INFO] pose_subs={pose_subs} result_pubs={result_pubs}")
        if pose_subs <= 0:
            print("[FAIL] bridge no conectado: /desired_grasp sin subscriptores")
            return 1
        if result_pubs <= 0:
            print("[FAIL] bridge no conectado: /desired_grasp/result sin publishers")
            return 1

        baseline_req = int(node.last_payload.get("request_id", -1) or -1) if node.last_payload else -1
        matched_ids: list[int] = []
        requests = max(1, int(args.requests))

        for idx in range(requests):
            before_count = int(node.result_count)
            before_req = int(matched_ids[-1]) if matched_ids else int(baseline_req)

            pose = PoseStamped()
            stamp_ns = int(node.wait_ros_time(2.0))
            expected_req_id = int(args.start_request_id) + idx
            expected_uuid = uuid.uuid4().hex
            pose.header.frame_id = f"base_link|rid={expected_req_id}|uid={expected_uuid}"
            pose.header.stamp.sec = stamp_ns // 1_000_000_000
            pose.header.stamp.nanosec = stamp_ns % 1_000_000_000
            pose.pose.position.x = float(args.x)
            pose.pose.position.y = float(args.y)
            pose.pose.position.z = float(args.z + 0.002 * idx)
            pose.pose.orientation.y = 0.7071068
            pose.pose.orientation.z = 0.7071068
            pose.pose.orientation.w = 0.0
            node.pose_pub.publish(pose)
            print(
                f"[INFO] send[{idx+1}/{requests}] stamp_ns={stamp_ns} "
                f"z={pose.pose.position.z:.3f} baseline_req={before_req} "
                f"expected_request_id={expected_req_id} expected_uuid={expected_uuid}"
            )

            deadline = time.monotonic() + max(0.2, args.timeout)
            matched = {}
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.result_count <= before_count:
                    continue
                for payload in node.result_msgs[before_count:]:
                    if not payload:
                        continue
                    req = int(payload.get("request_id", -1) or -1)
                    if req == expected_req_id:
                        matched = dict(payload)
                        break
                if matched:
                    break

            if not matched:
                print(
                    f"[FAIL] no result matching request_id for send[{idx+1}] "
                    f"(expected={expected_req_id})"
                )
                return 1

            req = int(matched.get("request_id", -1) or -1)
            got_stamp = int(matched.get("target_stamp_ns", 0) or 0)
            got_uuid = str(matched.get("request_uuid", "") or "")
            print(
                "[INFO] result "
                f"send[{idx+1}] request_id={req} success={matched.get('success')} "
                f"plan_ok={matched.get('plan_ok')} exec_ok={matched.get('exec_ok')} "
                f"stamp={got_stamp} request_uuid={got_uuid or 'n/a'}"
            )
            if req != expected_req_id:
                print(
                    f"[FAIL] request_id mismatch en send[{idx+1}]: req={req} "
                    f"expected={expected_req_id}"
                )
                return 1
            if got_uuid != expected_uuid:
                print(
                    f"[FAIL] request_uuid mismatch en send[{idx+1}]: "
                    f"got={got_uuid or 'n/a'} expected={expected_uuid}"
                )
                return 1
            matched_ids.append(req)

        if len(set(matched_ids)) != len(matched_ids):
            print(f"[FAIL] request_id duplicado en resultados: {matched_ids}")
            return 1
        print(
            "[PASS] contrato bridge request/result OK "
            f"(requests={requests}, ids={matched_ids})"
        )
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
