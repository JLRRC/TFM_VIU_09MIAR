#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_result_topic_stability_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Monitor /desired_grasp/result publisher stability and result delivery."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import String


class StabilityProbe(Node):
    def __init__(self, pose_topic: str, result_topic: str, heartbeat_topic: str) -> None:
        super().__init__("test_result_topic_stability_rclpy")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.pose_topic = pose_topic
        self.result_topic = result_topic
        self.heartbeat_topic = heartbeat_topic
        self.qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, self.qos)
        self.result_sub = self.create_subscription(String, self.result_topic, self._on_result, self.qos)
        self.hb_sub = self.create_subscription(Bool, self.heartbeat_topic, self._on_hb, self.qos)
        self.result_payloads: list[dict] = []
        self.result_raw = ""
        self.hb_count = 0
        self.hb_last_mono = 0.0

    def _on_result(self, msg: String) -> None:
        self.result_raw = str(msg.data or "")
        try:
            payload = json.loads(self.result_raw)
        except Exception:
            payload = {}
        if isinstance(payload, dict):
            self.result_payloads.append(payload)

    def _on_hb(self, _msg: Bool) -> None:
        self.hb_count += 1
        self.hb_last_mono = time.monotonic()

    def ros_now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def wait_ros_time(self, timeout_sec: float) -> int:
        deadline = time.monotonic() + max(0.2, timeout_sec)
        while time.monotonic() < deadline:
            now_ns = int(self.get_clock().now().nanoseconds)
            if now_ns > 0:
                return now_ns
            rclpy.spin_once(self, timeout_sec=0.05)
        return int(self.get_clock().now().nanoseconds)


def _wait_connected(node: StabilityProbe, timeout_sec: float) -> tuple[bool, int, int]:
    deadline = time.monotonic() + max(0.5, timeout_sec)
    pose_subs = 0
    result_pubs = 0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        pose_subs = int(node.count_subscribers(node.pose_topic))
        result_pubs = int(node.count_publishers(node.result_topic))
        if pose_subs > 0 and result_pubs > 0:
            return True, pose_subs, result_pubs
    return False, pose_subs, result_pubs


def _start_bridge(cmd: str, log_path: str) -> subprocess.Popen | None:
    try:
        logf = open(log_path, "a", encoding="utf-8")
        proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            stdout=logf,
            stderr=logf,
            preexec_fn=os.setsid,
        )
        logf.close()
        print(f"[INFO] bridge_started pid={proc.pid} log={log_path}")
        return proc
    except Exception as exc:
        print(f"[FAIL] no se pudo lanzar bridge: {exc}")
        return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-topic", default="/desired_grasp")
    parser.add_argument("--result-topic", default="/desired_grasp/result")
    parser.add_argument("--discover-timeout", type=float, default=8.0)
    parser.add_argument("--monitor-sec", type=float, default=10.0)
    parser.add_argument("--result-timeout", type=float, default=5.0)
    parser.add_argument("--heartbeat-topic", default="/ur5_moveit_bridge/heartbeat")
    parser.add_argument("--heartbeat-max-age", type=float, default=1.2)
    parser.add_argument("--x", type=float, default=0.43)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.075)
    parser.add_argument("--auto-start-bridge", action="store_true")
    parser.add_argument("--bridge-log", default="/tmp/test_result_topic_stability_bridge.log")
    parser.add_argument(
        "--bridge-cmd",
        default=(
            "source /opt/ros/jazzy/setup.bash && "
            "source /home/laboratorio/TFM/agarre_ros2_ws/install/setup.bash && "
            "ros2 run ur5_tools ur5_moveit_bridge --ros-args "
            "-p backend:=auto -p ee_frame:=rg2_tcp -p result_topic:=/desired_grasp/result "
            "-p request_timeout_sec:=2.0 -p execute_timeout_sec:=1.5 "
            "-p use_sim_time:=true -p moveit_py_use_sim_time:=true"
        ),
    )
    args = parser.parse_args()

    rclpy.init()
    node = StabilityProbe(args.pose_topic, args.result_topic, args.heartbeat_topic)
    bridge_proc = None
    rc = 1
    try:
        ok, pose_subs, result_pubs = _wait_connected(node, args.discover_timeout)
        if (not ok) and args.auto_start_bridge:
            bridge_proc = _start_bridge(args.bridge_cmd, args.bridge_log)
            if bridge_proc is not None:
                ok, pose_subs, result_pubs = _wait_connected(node, args.discover_timeout)
        print(
            "[INFO] bridge_path "
            f"connected={str(ok).lower()} pose_subs={pose_subs} result_pubs={result_pubs} "
            f"pose_topic={args.pose_topic} result_topic={args.result_topic} "
            f"heartbeat_topic={args.heartbeat_topic}"
        )
        if not ok:
            print("[FAIL] bridge no conectado para test de estabilidad")
            return 1

        baseline_req = -1
        if node.result_payloads:
            try:
                baseline_req = int(node.result_payloads[-1].get("request_id", -1) or -1)
            except Exception:
                baseline_req = -1

        stamp_ns = max(0, node.wait_ros_time(2.0))
        pose = PoseStamped()
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

        node.pose_pub.publish(pose)
        print(
            "[INFO] published_request "
            f"stamp_ns={stamp_ns} target=({args.x:.3f},{args.y:.3f},{args.z:.3f})"
        )

        monitor_deadline = time.monotonic() + max(1.0, args.monitor_sec)
        result_deadline = time.monotonic() + max(1.0, args.result_timeout)
        saw_result = False
        matched_req = -1
        had_result_publisher = False
        publisher_lost = False
        publisher_lost_elapsed = 0.0
        publisher_lost_with_heartbeat = False
        publisher_lost_hb_age = float("inf")
        while time.monotonic() < monitor_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = max(0.0, args.monitor_sec - (monitor_deadline - time.monotonic()))
            result_pub_count = int(node.count_publishers(node.result_topic))
            hb_age = (
                float("inf")
                if node.hb_last_mono <= 0.0
                else max(0.0, time.monotonic() - node.hb_last_mono)
            )
            hb_recent = hb_age <= float(args.heartbeat_max_age)
            if result_pub_count > 0:
                had_result_publisher = True
            elif had_result_publisher and not publisher_lost:
                publisher_lost = True
                publisher_lost_elapsed = elapsed
                publisher_lost_with_heartbeat = hb_recent
                publisher_lost_hb_age = hb_age

            if (not saw_result) and time.monotonic() <= result_deadline:
                for payload in node.result_payloads:
                    try:
                        req_id = int(payload.get("request_id", -1) or -1)
                    except Exception:
                        req_id = -1
                    if req_id > baseline_req:
                        matched_req = req_id
                        saw_result = True
                        break

        print(
            "[CHECK] result_delivery "
            f"saw_result={str(saw_result).lower()} matched_request_id={matched_req} baseline_request_id={baseline_req}"
        )
        print(
            "[CHECK] publisher_stability "
            f"had_result_publisher={str(had_result_publisher).lower()} "
            f"publisher_lost={str(publisher_lost).lower()} "
            f"lost_elapsed={publisher_lost_elapsed:.1f}s"
        )
        hb_last_age_txt = (
            "inf"
            if node.hb_last_mono <= 0.0
            else f"{max(0.0, time.monotonic() - node.hb_last_mono):.2f}s"
        )
        print(
            "[CHECK] heartbeat "
            f"count={node.hb_count} last_age={hb_last_age_txt} "
            f"max_age={float(args.heartbeat_max_age):.2f}s"
        )

        if not saw_result:
            print("[FAIL] no se recibio /desired_grasp/result dentro de ventana")
            return 1
        if publisher_lost_with_heartbeat:
            print(
                "[FAIL] publisher de /desired_grasp/result desaparecio "
                "mientras heartbeat seguia vivo "
                f"(hb_age={publisher_lost_hb_age:.2f}s)"
            )
            return 1
        if publisher_lost:
            print("[FAIL] publisher de /desired_grasp/result desaparecio durante la monitorizacion")
            return 1

        print("[PASS] result topic estable y resultado recibido")
        rc = 0
    finally:
        try:
            if bridge_proc is not None and bridge_proc.poll() is None:
                os.killpg(os.getpgid(bridge_proc.pid), signal.SIGTERM)
        except Exception:
            pass
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
