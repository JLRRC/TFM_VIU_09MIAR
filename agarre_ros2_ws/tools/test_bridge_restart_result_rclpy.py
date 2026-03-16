#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_bridge_restart_result_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Simula caida/restart del bridge y verifica que el siguiente request recibe result."""

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
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RestartProbe(Node):
    def __init__(self, pose_topic: str, result_topic: str) -> None:
        super().__init__("test_bridge_restart_result_rclpy")
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
        self.last_result = ""
        self.last_seq = 0

    def _on_result(self, msg: String) -> None:
        self.last_result = str(msg.data or "")
        self.last_seq += 1

    def wait_ros_time(self, timeout_sec: float) -> int:
        deadline = time.monotonic() + max(0.2, timeout_sec)
        while time.monotonic() < deadline:
            now_ns = int(self.get_clock().now().nanoseconds)
            if now_ns > 0:
                return now_ns
            rclpy.spin_once(self, timeout_sec=0.05)
        return int(self.get_clock().now().nanoseconds)

    def wait_bridge_connected(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.5, timeout_sec)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            pose_subs = int(self.count_subscribers(self.pose_topic))
            result_pubs = int(self.count_publishers(self.result_topic))
            if pose_subs > 0 and result_pubs > 0:
                print(
                    f"[INFO] bridge_connected pose_subs={pose_subs} result_pubs={result_pubs}"
                )
                return True
        pose_subs = int(self.count_subscribers(self.pose_topic))
        result_pubs = int(self.count_publishers(self.result_topic))
        print(
            f"[FAIL] bridge_not_connected pose_subs={pose_subs} result_pubs={result_pubs}"
        )
        return False

    def wait_bridge_disconnected(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.5, timeout_sec)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            pose_subs = int(self.count_subscribers(self.pose_topic))
            result_pubs = int(self.count_publishers(self.result_topic))
            if pose_subs == 0 and result_pubs == 0:
                print("[INFO] bridge_disconnected confirmed")
                return True
        pose_subs = int(self.count_subscribers(self.pose_topic))
        result_pubs = int(self.count_publishers(self.result_topic))
        print(
            f"[WARN] bridge_disconnect_not_confirmed pose_subs={pose_subs} result_pubs={result_pubs}"
        )
        return False

    def send_and_wait(self, *, x: float, y: float, z: float, timeout_sec: float) -> bool:
        before_seq = int(self.last_seq)
        stamp_ns = int(self.wait_ros_time(2.0))
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp.sec = stamp_ns // 1_000_000_000
        pose.header.stamp.nanosec = stamp_ns % 1_000_000_000
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.7071068
        pose.pose.orientation.z = 0.7071068
        pose.pose.orientation.w = 0.0
        self.pose_pub.publish(pose)
        print(
            "[INFO] sent request "
            f"stamp_ns={stamp_ns} xyz=({x:.3f},{y:.3f},{z:.3f}) seq={before_seq}"
        )
        deadline = time.monotonic() + max(0.5, timeout_sec)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if int(self.last_seq) > before_seq and self.last_result:
                print(f"[INFO] got result seq={self.last_seq} raw={self.last_result}")
                try:
                    payload = json.loads(self.last_result)
                except Exception:
                    payload = {}
                if payload:
                    print(
                        "[RESULT] "
                        f"request_id={payload.get('request_id')} success={payload.get('success')} "
                        f"plan_ok={payload.get('plan_ok')} exec_ok={payload.get('exec_ok')} "
                        f"message={payload.get('message')}"
                    )
                return True
        print(f"[FAIL] timeout esperando result (<{timeout_sec:.1f}s)")
        return False


def _kill_bridge_processes() -> None:
    try:
        out = subprocess.check_output(["pgrep", "-f", "ur5_moveit_bridge"], text=True).strip()
    except Exception:
        out = ""
    pids = []
    for line in out.splitlines():
        try:
            pids.append(int(line.strip()))
        except Exception:
            continue
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except Exception:
            pass
    if pids:
        print(f"[INFO] killed bridge pids={pids}")
    else:
        print("[INFO] no existing bridge pids found")


def _start_bridge(cmd: str, log_path: str) -> Optional[subprocess.Popen]:
    try:
        logf = open(log_path, "a", encoding="utf-8")
        proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            stdout=logf,
            stderr=logf,
            preexec_fn=os.setsid,
        )
        logf.close()
        print(f"[INFO] started bridge pid={proc.pid} log={log_path}")
        return proc
    except Exception as exc:
        print(f"[FAIL] cannot start bridge: {exc}")
        return None


def _tail_file(path: str, lines: int = 40) -> str:
    try:
        with open(path, "r", encoding="utf-8") as f:
            content = f.readlines()
    except Exception:
        return ""
    return "".join(content[-max(1, lines) :])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-topic", default="/desired_grasp")
    parser.add_argument("--result-topic", default="/desired_grasp/result")
    parser.add_argument("--x", type=float, default=0.43)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.075)
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--bridge-cmd",
        default=(
            "ros2 run ur5_tools ur5_moveit_bridge --ros-args "
            "-p backend:=auto -p ee_frame:=rg2_tcp -p result_topic:=/desired_grasp/result "
            "-p request_timeout_sec:=2.0 -p execute_timeout_sec:=1.5"
        ),
    )
    parser.add_argument("--bridge-log", default="/tmp/test_bridge_restart_moveit_bridge.log")
    args = parser.parse_args()

    rclpy.init()
    node = RestartProbe(args.pose_topic, args.result_topic)
    spawned_proc: Optional[subprocess.Popen] = None
    rc = 1
    try:
        try:
            os.remove(args.bridge_log)
        except Exception:
            pass
        if not node.wait_bridge_connected(timeout_sec=4.0):
            spawned_proc = _start_bridge(args.bridge_cmd, args.bridge_log)
            if spawned_proc is None:
                return 1
            if not node.wait_bridge_connected(timeout_sec=8.0):
                return 1

        print("[STEP] request before restart")
        if not node.send_and_wait(x=args.x, y=args.y, z=args.z, timeout_sec=args.timeout):
            return 1

        print("[STEP] restart bridge")
        _kill_bridge_processes()
        node.wait_bridge_disconnected(timeout_sec=3.0)
        spawned_proc = _start_bridge(args.bridge_cmd, args.bridge_log)
        if spawned_proc is None:
            return 1
        if not node.wait_bridge_connected(timeout_sec=8.0):
            return 1

        print("[STEP] request after restart")
        if not node.send_and_wait(x=args.x, y=args.y, z=args.z, timeout_sec=args.timeout):
            tail = _tail_file(args.bridge_log)
            if tail:
                print("[DIAG] bridge_log_tail:")
                print(tail)
            return 1

        print("[PASS] bridge restart path returns result before and after restart")
        rc = 0
    finally:
        try:
            if spawned_proc is not None and spawned_proc.poll() is None:
                os.killpg(os.getpgid(spawned_proc.pid), signal.SIGTERM)
        except Exception:
            pass
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    # Avoid rclpy teardown hangs in CI/headless.
    os._exit(rc)


if __name__ == "__main__":
    raise SystemExit(main())
