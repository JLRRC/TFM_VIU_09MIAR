#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_result_pipe_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Test real de tuberia /desired_grasp -> /desired_grasp/result con rclpy."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def qos_summary(qos: QoSProfile) -> str:
    rel = "RELIABLE" if qos.reliability == ReliabilityPolicy.RELIABLE else "BEST_EFFORT"
    dur = "VOLATILE" if qos.durability == DurabilityPolicy.VOLATILE else "TRANSIENT_LOCAL"
    hist = "KEEP_LAST" if qos.history == HistoryPolicy.KEEP_LAST else "KEEP_ALL"
    return f"{rel}/{dur}/{hist}@depth={qos.depth}"


def endpoint_dump(node: Node, topic: str) -> str:
    try:
        pubs = node.get_publishers_info_by_topic(topic) or []
    except Exception:
        pubs = []
    try:
        subs = node.get_subscriptions_info_by_topic(topic) or []
    except Exception:
        subs = []

    lines: list[str] = [f"topic={topic} pubs={len(pubs)} subs={len(subs)}"]
    for idx, ep in enumerate(pubs):
        q = getattr(ep, "qos_profile", None)
        lines.append(
            f"  pub[{idx}] node={getattr(ep, 'node_name', '?')} "
            f"ns={getattr(ep, 'node_namespace', '?')} type={getattr(ep, 'topic_type', '?')} "
            f"qos={qos_summary(q) if q else 'n/a'}"
        )
    for idx, ep in enumerate(subs):
        q = getattr(ep, "qos_profile", None)
        lines.append(
            f"  sub[{idx}] node={getattr(ep, 'node_name', '?')} "
            f"ns={getattr(ep, 'node_namespace', '?')} type={getattr(ep, 'topic_type', '?')} "
            f"qos={qos_summary(q) if q else 'n/a'}"
        )
    return "\n".join(lines)


def tail_lines(path: str, n: int = 120) -> list[str]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = f.readlines()
    except Exception:
        return []
    return lines[-max(1, n) :]


class PipeProbe(Node):
    def __init__(self, pose_topic: str, result_topic: str, qos: QoSProfile):
        super().__init__("test_result_pipe_rclpy")
        self.pose_topic = pose_topic
        self.result_topic = result_topic
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, qos)
        self.result_sub = self.create_subscription(String, result_topic, self._on_result, qos)
        self.qos = qos
        self.last_result = ""
        self.last_payload: dict = {}
        self.result_count = 0

    def _on_result(self, msg: String) -> None:
        self.last_result = str(msg.data or "")
        self.result_count += 1
        try:
            self.last_payload = json.loads(self.last_result)
        except Exception:
            self.last_payload = {}


def start_bridge(cmd: str, log_path: str) -> Optional[subprocess.Popen]:
    try:
        logf = open(log_path, "a", encoding="utf-8")
        proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            stdout=logf,
            stderr=logf,
            preexec_fn=os.setsid,
        )
        logf.close()
        print(f"[INFO] bridge started pid={proc.pid} log={log_path}")
        return proc
    except Exception as exc:
        print(f"[FAIL] no se pudo arrancar bridge: {exc}")
        return None


def wait_connected(node: PipeProbe, timeout_sec: float) -> bool:
    deadline = time.monotonic() + max(0.5, timeout_sec)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        pose_subs = int(node.count_subscribers(node.pose_topic))
        result_pubs = int(node.count_publishers(node.result_topic))
        if pose_subs > 0 and result_pubs > 0:
            print(f"[INFO] connected pose_subs={pose_subs} result_pubs={result_pubs}")
            return True
    print(f"[WARN] not_connected pose_subs={node.count_subscribers(node.pose_topic)} result_pubs={node.count_publishers(node.result_topic)}")
    return False


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-topic", default="/desired_grasp")
    parser.add_argument("--result-topic", default="/desired_grasp/result")
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--x", type=float, default=0.43)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.075)
    parser.add_argument("--bridge-log", default="/tmp/test_result_pipe_bridge.log")
    parser.add_argument(
        "--bridge-cmd",
        default=(
            "ros2 run ur5_tools ur5_moveit_bridge --ros-args "
            "-p backend:=auto -p ee_frame:=rg2_tcp -p result_topic:=/desired_grasp/result "
            "-p request_timeout_sec:=2.0 -p execute_timeout_sec:=1.5"
        ),
    )
    parser.add_argument("--auto-start-bridge", action="store_true")
    args = parser.parse_args()

    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    rclpy.init()
    node = PipeProbe(args.pose_topic, args.result_topic, qos)
    bridge_proc: Optional[subprocess.Popen] = None
    rc = 1
    try:
        try:
            os.remove(args.bridge_log)
        except Exception:
            pass

        connected = wait_connected(node, timeout_sec=2.0)
        if not connected and args.auto_start_bridge:
            bridge_proc = start_bridge(args.bridge_cmd, args.bridge_log)
            if bridge_proc is None:
                return 1
            connected = wait_connected(node, timeout_sec=8.0)

        before_log = tail_lines(args.bridge_log, n=120)

        stamp_ns = time.time_ns()
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
        seq_before = int(node.result_count)
        node.pose_pub.publish(pose)
        print(
            "[INFO] sent request "
            f"topic={args.pose_topic} stamp_ns={stamp_ns} qos={qos_summary(qos)} "
            f"xyz=({args.x:.3f},{args.y:.3f},{args.z:.3f})"
        )

        deadline = time.monotonic() + max(0.5, args.timeout)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if int(node.result_count) > seq_before:
                break

        after_log = tail_lines(args.bridge_log, n=220)
        recv_seen = any("[BRIDGE][RECV]" in ln for ln in after_log[len(before_log) :]) or any(
            "[BRIDGE][RECV]" in ln for ln in after_log
        )
        pub_seen = any("[BRIDGE][PUB_RESULT]" in ln for ln in after_log[len(before_log) :]) or any(
            "[BRIDGE][PUB_RESULT]" in ln for ln in after_log
        )

        print("[INFO] local_sub type=std_msgs/msg/String qos=" + qos_summary(qos))
        print("[INFO] endpoint /desired_grasp")
        print(endpoint_dump(node, args.pose_topic))
        print("[INFO] endpoint /desired_grasp/result")
        print(endpoint_dump(node, args.result_topic))
        print(
            f"[INFO] bridge_log_flags recv_seen={str(recv_seen).lower()} pub_seen={str(pub_seen).lower()}"
        )

        if int(node.result_count) <= seq_before:
            print("[FAIL] no llego ningun result en el timeout")
            if after_log:
                print("[DIAG] bridge_log_tail:")
                print("".join(after_log[-50:]))
            return 1

        print(f"[INFO] result_raw={node.last_result}")
        payload = node.last_payload or {}
        print(
            "[RESULT] "
            f"request_id={payload.get('request_id')} success={payload.get('success')} "
            f"plan_ok={payload.get('plan_ok')} exec_ok={payload.get('exec_ok')} "
            f"msg={payload.get('message')}"
        )
        print("[PASS] result pipe OK: request enviado y result recibido")
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
    os._exit(rc)


if __name__ == "__main__":
    raise SystemExit(main())
