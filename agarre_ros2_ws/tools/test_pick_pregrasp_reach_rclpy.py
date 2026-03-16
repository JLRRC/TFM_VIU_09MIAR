#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_pick_pregrasp_reach_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Validate PRE_GRASP reaches target distance using MoveIt bridge + TF."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class PregraspProbe(Node):
    def __init__(self, pose_topic: str, result_topic: str, target_frame: str, ee_link: str) -> None:
        super().__init__("test_pick_pregrasp_reach_rclpy")
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
        self.target_frame = target_frame
        self.ee_link = ee_link
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, qos)
        self.result_sub = self.create_subscription(String, result_topic, self._on_result, qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_result: dict = {}
        self.result_count = 0

    def _on_result(self, msg: String) -> None:
        self.result_count += 1
        try:
            payload = json.loads(str(msg.data or ""))
        except Exception:
            payload = {}
        self.last_result = payload if isinstance(payload, dict) else {}

    def wait_ros_time_ns(self, timeout_sec: float) -> int:
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
    parser.add_argument("--request-id", type=int, default=201)
    parser.add_argument("--discover-timeout", type=float, default=8.0)
    parser.add_argument("--result-timeout", type=float, default=8.0)
    parser.add_argument("--reach-timeout", type=float, default=8.0)
    parser.add_argument("--target-frame", default="base_link")
    parser.add_argument("--ee-link", default="rg2_tcp")
    parser.add_argument("--x", type=float, default=0.43)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.075)
    parser.add_argument("--tol", type=float, default=0.03)
    args = parser.parse_args()

    rclpy.init()
    node = PregraspProbe(args.pose_topic, args.result_topic, args.target_frame, args.ee_link)
    rc = 1
    try:
        deadline = time.monotonic() + max(0.5, float(args.discover_timeout))
        pose_subs = 0
        result_pubs = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            pose_subs = int(node.count_subscribers(args.pose_topic))
            result_pubs = int(node.count_publishers(args.result_topic))
            if pose_subs > 0 and result_pubs > 0:
                break
        print(f"[INFO] pose_subs={pose_subs} result_pubs={result_pubs}")
        if pose_subs <= 0 or result_pubs <= 0:
            print("[FAIL] bridge no conectado para PRE_GRASP")
            return 1

        before_count = int(node.result_count)
        rid = int(args.request_id)
        stamp_ns = int(node.wait_ros_time_ns(2.0))
        pose = PoseStamped()
        pose.header.frame_id = f"{args.target_frame}|rid={rid}"
        pose.header.stamp.sec = stamp_ns // 1_000_000_000
        pose.header.stamp.nanosec = stamp_ns % 1_000_000_000
        pose.pose.position.x = float(args.x)
        pose.pose.position.y = float(args.y)
        pose.pose.position.z = float(args.z)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.7071068
        pose.pose.orientation.z = 0.7071068
        pose.pose.orientation.w = 0.0
        node.pose_pub.publish(pose)
        print(
            "[INFO] publish PRE_GRASP "
            f"request_id={rid} stamp_ns={stamp_ns} "
            f"target=({args.x:.3f},{args.y:.3f},{args.z:.3f})"
        )

        result = {}
        deadline = time.monotonic() + max(0.5, float(args.result_timeout))
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.result_count <= before_count:
                continue
            payload = dict(node.last_result)
            if not payload:
                continue
            if int(payload.get("request_id", -1) or -1) != rid:
                continue
            result = payload
            break
        if not result:
            print(f"[FAIL] no result for request_id={rid}")
            return 1
        print(
            "[INFO] result "
            f"request_id={result.get('request_id')} success={result.get('success')} "
            f"plan_ok={result.get('plan_ok')} exec_ok={result.get('exec_ok')} "
            f"msg={result.get('message')}"
        )
        if not bool(result.get("exec_ok", False)):
            print("[FAIL] exec_ok=false en PRE_GRASP")
            return 1

        target = (float(args.x), float(args.y), float(args.z))
        deadline = time.monotonic() + max(0.5, float(args.reach_timeout))
        best_dist = float("inf")
        last_dist = float("inf")
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                tf_msg = node.tf_buffer.lookup_transform(args.target_frame, args.ee_link, Time())
            except Exception:
                continue
            tx = float(tf_msg.transform.translation.x)
            ty = float(tf_msg.transform.translation.y)
            tz = float(tf_msg.transform.translation.z)
            dx = target[0] - tx
            dy = target[1] - ty
            dz = target[2] - tz
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5
            last_dist = dist
            best_dist = min(best_dist, dist)
            tf_stamp_ns = (
                int(getattr(tf_msg.header.stamp, "sec", 0) or 0) * 1_000_000_000
                + int(getattr(tf_msg.header.stamp, "nanosec", 0) or 0)
            )
            ros_now_ns = int(node.get_clock().now().nanoseconds)
            tf_age = (
                (ros_now_ns - tf_stamp_ns) / 1_000_000_000.0
                if ros_now_ns > 0 and tf_stamp_ns > 0
                else float("nan")
            )
            print(
                "[TF_CHECK] "
                f"dist={dist:.3f} best={best_dist:.3f} tol={float(args.tol):.3f} "
                f"tf_age={('n/a' if tf_age != tf_age else f'{tf_age:.3f}s')} "
                f"frame={args.target_frame} ee_link={args.ee_link}"
            )
            if dist < float(args.tol):
                print("[PASS] PRE_GRASP reached within tolerance")
                rc = 0
                break
        if rc != 0:
            print(
                f"[FAIL] pos_not_reached best={best_dist:.3f} "
                f"last={last_dist:.3f} tol={float(args.tol):.3f}"
            )
            return 1
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
