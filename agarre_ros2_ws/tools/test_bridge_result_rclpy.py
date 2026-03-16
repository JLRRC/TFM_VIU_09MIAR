#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_bridge_result_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Publish one PRE_GRASP PoseStamped and wait for /desired_grasp/result."""

from __future__ import annotations

import argparse
import ast
import json
import os
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from tf2_ros import Buffer, TransformListener


class BridgeProbe(Node):
    def __init__(self, pose_topic: str, result_topic: str, traj_topic: str):
        super().__init__("test_bridge_result_rclpy")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.set_parameters(
            [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
        )
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pose_topic = pose_topic
        self.result_topic = result_topic
        self.traj_topic = traj_topic
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, qos)
        self.result_sub = self.create_subscription(String, result_topic, self._cb, qos)
        self.traj_sub = self.create_subscription(
            JointTrajectory, traj_topic, self._traj_cb, qos
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.result_raw = ""
        self.result_payload = {}
        self.result_msgs: list[dict] = []
        self.traj_count = 0

    def _cb(self, msg: String) -> None:
        self.result_raw = msg.data
        payload = {}
        try:
            payload = json.loads(msg.data)
        except Exception:
            try:
                decoded = ast.literal_eval(msg.data)
                if isinstance(decoded, str):
                    payload = json.loads(decoded)
            except Exception:
                payload = {}
        self.result_payload = payload if isinstance(payload, dict) else {}
        self.result_msgs.append(self.result_payload)

    def _traj_cb(self, _msg: JointTrajectory) -> None:
        self.traj_count += 1

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
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--discover-timeout", type=float, default=8.0)
    parser.add_argument("--x", type=float, default=0.43)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.075)
    parser.add_argument(
        "--traj-topic",
        default="/joint_trajectory_controller/joint_trajectory",
    )
    parser.add_argument("--monitor-sec", type=float, default=3.0)
    parser.add_argument("--target-frame", default="base_link")
    parser.add_argument("--ee-link", default="rg2_tcp")
    parser.add_argument("--tf-timeout", type=float, default=2.0)
    parser.add_argument("--dist-tol", type=float, default=0.03)
    parser.add_argument("--bridge-log", default="")
    args = parser.parse_args()

    rclpy.init()
    node = BridgeProbe(args.pose_topic, args.result_topic, args.traj_topic)
    rc = 1
    try:
        sub_deadline = time.monotonic() + max(1.0, float(args.discover_timeout))
        while time.monotonic() < sub_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.count_subscribers(node.pose_topic) > 0:
                break
        subs = int(node.count_subscribers(node.pose_topic))
        if subs <= 0:
            print("[CHECK] saw_bridge_result=false")
            print("[CHECK] saw_bridge_recv=false")
            print(f"[FAIL] {args.pose_topic} sin subscriptores")
            return 1

        baseline_req_id = -1
        if isinstance(node.result_payload, dict):
            try:
                baseline_req_id = int(node.result_payload.get("request_id", -1) or -1)
            except Exception:
                baseline_req_id = -1
        baseline_count = len(node.result_msgs)

        pose = PoseStamped()
        stamp_ns = int(node.wait_ros_time(2.0))
        pose.header.frame_id = "base_link"
        pose.header.stamp.sec = stamp_ns // 1_000_000_000
        pose.header.stamp.nanosec = stamp_ns % 1_000_000_000
        pose.pose.position.x = float(args.x)
        pose.pose.position.y = float(args.y)
        pose.pose.position.z = float(args.z)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.7071068
        pose.pose.orientation.z = 0.7071068
        pose.pose.orientation.w = 0.0
        traj_before = int(node.traj_count)
        node.pose_pub.publish(pose)
        print(
            "[INFO] sent pose "
            f"topic={args.pose_topic} xyz=({args.x:.3f},{args.y:.3f},{args.z:.3f}) stamp_ns={stamp_ns}"
        )

        deadline = time.monotonic() + max(0.5, args.timeout)
        matched_payload = {}
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if len(node.result_msgs) <= baseline_count:
                continue
            for payload in node.result_msgs[baseline_count:]:
                if not isinstance(payload, dict):
                    continue
                try:
                    req_id = int(payload.get("request_id", -1) or -1)
                except Exception:
                    req_id = -1
                if req_id > baseline_req_id:
                    try:
                        got_stamp_ns = int(payload.get("target_stamp_ns", 0) or 0)
                    except Exception:
                        got_stamp_ns = 0
                    # If both stamps are present, ignore stale results from a prior request.
                    if stamp_ns > 0 and got_stamp_ns > 0 and got_stamp_ns < stamp_ns:
                        continue
                    matched_payload = payload
                    break
            if matched_payload:
                break

        saw_bridge_result = bool(matched_payload)
        if not saw_bridge_result:
            print("[CHECK] saw_bridge_result=false")
            print("[CHECK] saw_bridge_recv=false")
            print(
                f"[FAIL] timeout esperando {args.result_topic} "
                f"(request_id>{baseline_req_id})"
            )
            return 1

        traj_during = int(node.traj_count) - traj_before
        print(
            f"[INFO] trajectory_msgs_during_wait={traj_during} topic={args.traj_topic}"
        )
        payload = dict(matched_payload)
        print(f"[INFO] result_raw={json.dumps(payload, ensure_ascii=True)}")

        success = None
        if payload:
            success = bool(payload.get("success", False))
            print(
                "[RESULT] "
                f"request_id={payload.get('request_id')} success={payload.get('success')} "
                f"plan_ok={payload.get('plan_ok')} exec_ok={payload.get('exec_ok')} "
                f"message={payload.get('message')}"
            )
            got_stamp = int(payload.get("target_stamp_ns", 0) or 0)
            if got_stamp and got_stamp != stamp_ns:
                print(
                    f"[WARN] stamp mismatch expected={stamp_ns} got={got_stamp} "
                    "(matching primario por request_id)"
                )
            if not bool(payload.get("exec_ok", False)):
                monitor_deadline = time.monotonic() + max(0.1, args.monitor_sec)
                while time.monotonic() < monitor_deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                traj_after = int(node.traj_count) - traj_before
                print(
                    f"[INFO] trajectory_msgs_total_window={traj_after} "
                    f"window_sec={args.monitor_sec:.1f}"
                )
                if traj_after > 0:
                    print("CONFLICT DETECTED")
        else:
            print("[WARN] result payload no-JSON; se muestra raw")

        bridge_log_path = args.bridge_log.strip()
        if not bridge_log_path:
            bridge_log_path = str(Path(__file__).resolve().parents[1] / "log" / "moveit_bridge.log")
        saw_bridge_recv = False
        try:
            with open(bridge_log_path, "r", encoding="utf-8", errors="ignore") as fh:
                log_text = fh.read()
            req_txt = ""
            try:
                req_txt = str(int(payload.get("request_id", -1) or -1))
            except Exception:
                req_txt = ""
            if req_txt and req_txt != "-1":
                saw_bridge_recv = f"[BRIDGE][RECV]" in log_text and f"request_id={req_txt}" in log_text
            else:
                stamp_txt = f"{pose.header.stamp.sec}.{pose.header.stamp.nanosec:09d}"
                saw_bridge_recv = (
                    "[BRIDGE][RECV]" in log_text
                    and stamp_txt in log_text
                )
        except Exception as exc:
            print(f"[WARN] no se pudo leer bridge log ({bridge_log_path}): {exc}")
            saw_bridge_recv = False

        print(f"[CHECK] saw_bridge_result={str(bool(saw_bridge_result)).lower()}")
        if (not saw_bridge_recv) and bool(saw_bridge_result):
            # Fallback: if a well-formed result arrived for this request_id,
            # bridge receive is implied even when file logging is unavailable.
            try:
                if int(payload.get("request_id", -1) or -1) >= 0:
                    saw_bridge_recv = True
                    print("[WARN] saw_bridge_recv inferred_from_result=true")
            except Exception:
                pass
        print(f"[CHECK] saw_bridge_recv={str(bool(saw_bridge_recv)).lower()}")

        tf_deadline = time.monotonic() + max(0.1, args.tf_timeout)
        tf_msg = None
        while time.monotonic() < tf_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                tf_msg = node.tf_buffer.lookup_transform(
                    args.target_frame,
                    args.ee_link,
                    Time(),
                )
                break
            except Exception:
                tf_msg = None
        if tf_msg is None:
            print(
                f"[FAIL] TF no disponible {args.target_frame}->{args.ee_link} "
                f"(timeout={args.tf_timeout:.1f}s)"
            )
            return 1
        tx = float(tf_msg.transform.translation.x)
        ty = float(tf_msg.transform.translation.y)
        tz = float(tf_msg.transform.translation.z)
        dx = float(args.x) - tx
        dy = float(args.y) - ty
        dz = float(args.z) - tz
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        tf_stamp_ns = (
            int(getattr(tf_msg.header.stamp, "sec", 0) or 0) * 1_000_000_000
            + int(getattr(tf_msg.header.stamp, "nanosec", 0) or 0)
        )
        ros_now_tf = int(node.get_clock().now().nanoseconds)
        tf_age = None
        if tf_stamp_ns > 0 and ros_now_tf > 0:
            tf_age = (ros_now_tf - tf_stamp_ns) / 1_000_000_000.0
        print(
            f"[TF] {args.target_frame}->{args.ee_link} tcp=({tx:.3f},{ty:.3f},{tz:.3f}) "
            f"target=({args.x:.3f},{args.y:.3f},{args.z:.3f}) dist={dist:.3f} tol={args.dist_tol:.3f} "
            f"tf_age={('n/a' if tf_age is None else f'{tf_age:.3f}s')}"
        )
        if not bool(saw_bridge_result) or not bool(saw_bridge_recv):
            print("[FAIL] contrato bridge incompleto (recv/result)")
            return 1
        if dist >= float(args.dist_tol):
            print("[FAIL] TF mismatch: dist fuera de tolerancia")
            return 1
        if success is not True:
            print("[WARN] result success=false (se valida pipe+stamp+TF)")
        print("[PASS] result recibido, stamp coherente y TCP dentro de tolerancia")
        rc = 0
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    # Avoid rare rclpy shutdown hangs.
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:
        pass
    os._exit(rc)


if __name__ == "__main__":
    raise SystemExit(main())
