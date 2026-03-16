#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/verify_attach_motion.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Verify that a gripper attach causes real object motion in Gazebo."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Empty
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


UR5_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class VerifyAttachMotion(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("verify_attach_motion")
        self._object = args.object
        self._tcp_frame = args.tcp_frame
        self._world_frame = args.world_frame
        self._pose_topic = args.pose_topic
        self._duration = float(args.duration)
        self._threshold = float(args.threshold)
        self._nudge = bool(args.nudge)
        self._nudge_delta = float(args.nudge_delta)
        self._nudge_sec = float(args.nudge_sec)
        self._pose: Dict[str, Tuple[float, float, float]] = {}
        self._joint_state: Optional[JointState] = None
        self._tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        self._pose_sub = self.create_subscription(
            TFMessage, self._pose_topic, self._on_pose, 10
        )
        self._joint_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10
        )
        self._attach_pub = self.create_publisher(
            Empty, f"/gripper/{self._object}/attach", 10
        )
        self._detach_pub = self.create_publisher(
            Empty, f"/gripper/{self._object}/detach", 10
        )
        self._traj_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

    def _on_pose(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            name = str(tf.child_frame_id or "").strip()
            if not name:
                continue
            tr = tf.transform.translation
            p = (float(tr.x), float(tr.y), float(tr.z))
            self._pose[name] = p
            if "::" in name:
                model = name.split("::")[0].strip()
                if model:
                    self._pose[model] = p

    def _on_joint_states(self, msg: JointState) -> None:
        self._joint_state = msg

    def _lookup_pose(self, name: str) -> Optional[Tuple[float, float, float]]:
        if name in self._pose:
            return self._pose[name]
        alt = name.lstrip("/")
        if alt in self._pose:
            return self._pose[alt]
        scoped = f"ur5_rg2::{alt}"
        if scoped in self._pose:
            return self._pose[scoped]
        suffix = f"::{alt}"
        for key, pose in self._pose.items():
            if key.endswith(suffix):
                return pose
        return None

    def _lookup_tcp_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame,
                self._tcp_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            tr = tf.transform.translation
            return (float(tr.x), float(tr.y), float(tr.z))
        except Exception:
            return self._lookup_pose(self._tcp_frame)

    @staticmethod
    def _dist(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _wait_pose(self, key: str, timeout: float) -> Optional[Tuple[float, float, float]]:
        deadline = time.time() + max(0.2, timeout)
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pose = self._lookup_pose(key)
            if pose is not None:
                return pose
        return None

    def _send_attach(self) -> None:
        self._attach_pub.publish(Empty())
        self.get_logger().info(f"[VERIFY] attach published topic=/gripper/{self._object}/attach")

    def _send_detach(self) -> None:
        self._detach_pub.publish(Empty())
        self.get_logger().info(f"[VERIFY] detach published topic=/gripper/{self._object}/detach")

    def _send_nudge_trajectory(self) -> bool:
        msg = self._joint_state
        if msg is None:
            self.get_logger().warn("[VERIFY] nudge skipped: no joint_states")
            return False
        names = list(msg.name)
        pos = list(msg.position)
        index = {n: i for i, n in enumerate(names)}
        if not all(j in index for j in UR5_JOINTS):
            self.get_logger().warn("[VERIFY] nudge skipped: missing UR5 joints in joint_states")
            return False
        traj = JointTrajectory()
        traj.joint_names = list(UR5_JOINTS)
        base = [float(pos[index[j]]) for j in UR5_JOINTS]
        moved = list(base)
        moved[0] += self._nudge_delta
        p1 = JointTrajectoryPoint()
        p1.positions = base
        p1.time_from_start.sec = 0
        p1.time_from_start.nanosec = 200_000_000
        p2 = JointTrajectoryPoint()
        p2.positions = moved
        p2.time_from_start.sec = int(max(1.0, self._nudge_sec))
        p2.time_from_start.nanosec = int((max(1.0, self._nudge_sec) % 1.0) * 1e9)
        traj.points = [p1, p2]
        self._traj_pub.publish(traj)
        self.get_logger().info(
            "[VERIFY] nudge trajectory published "
            f"delta_joint0={self._nudge_delta:.3f}rad t={self._nudge_sec:.2f}s"
        )
        return True

    def run(self) -> int:
        object_before = self._wait_pose(self._object, 4.0)
        tcp_before = None
        deadline = time.time() + 4.0
        while time.time() < deadline and tcp_before is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            tcp_before = self._lookup_tcp_pose()
        if object_before is None:
            print(f"[FAIL] object pose missing object={self._object}")
            return 1
        if tcp_before is None:
            print(f"[FAIL] tcp pose missing tcp_frame={self._tcp_frame}")
            return 1
        print(
            "[INFO] before "
            f"object={self._object}@({object_before[0]:.3f},{object_before[1]:.3f},{object_before[2]:.3f}) "
            f"tcp={self._tcp_frame}@({tcp_before[0]:.3f},{tcp_before[1]:.3f},{tcp_before[2]:.3f})"
        )
        self._send_attach()
        time.sleep(0.25)
        if self._nudge:
            wait_js_deadline = time.time() + 3.0
            while self._joint_state is None and time.time() < wait_js_deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            self._send_nudge_trajectory()
        start = time.time()
        max_obj_disp = 0.0
        max_tcp_disp = 0.0
        last_obj = object_before
        last_tcp = tcp_before
        while (time.time() - start) < self._duration:
            rclpy.spin_once(self, timeout_sec=0.05)
            obj_now = self._lookup_pose(self._object)
            tcp_now = self._lookup_tcp_pose()
            if obj_now is not None:
                last_obj = obj_now
                max_obj_disp = max(max_obj_disp, self._dist(obj_now, object_before))
            if tcp_now is not None:
                last_tcp = tcp_now
                max_tcp_disp = max(max_tcp_disp, self._dist(tcp_now, tcp_before))
        self._send_detach()
        print(
            "[INFO] after "
            f"object=({last_obj[0]:.3f},{last_obj[1]:.3f},{last_obj[2]:.3f}) "
            f"tcp=({last_tcp[0]:.3f},{last_tcp[1]:.3f},{last_tcp[2]:.3f}) "
            f"obj_disp={max_obj_disp:.3f}m tcp_disp={max_tcp_disp:.3f}m threshold={self._threshold:.3f}m"
        )
        if max_obj_disp >= self._threshold:
            print("[PASS] attach motion detected")
            return 0
        print("[FAIL] attach motion not detected")
        return 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify that attached object moves physically in Gazebo."
    )
    parser.add_argument("--object", default="cyl_gray", help="Object model name")
    parser.add_argument("--pose-topic", default="/world/ur5_mesa_objetos/pose/info")
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--tcp-frame", default="rg2_tcp")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--threshold", type=float, default=0.02)
    parser.add_argument("--nudge", action="store_true", help="Publish a small joint nudge")
    parser.add_argument("--nudge-delta", type=float, default=0.12)
    parser.add_argument("--nudge-sec", type=float, default=1.8)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = VerifyAttachMotion(args)
    code = 1
    try:
        code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
