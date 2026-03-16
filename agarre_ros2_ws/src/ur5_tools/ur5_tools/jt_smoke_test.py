#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/jt_smoke_test.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/jt_smoke_test.py
# Summary: Minimal JointTrajectory smoke test with joint state verification.
"""Publish a minimal JointTrajectory and verify joint state change."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Dict, List

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .param_utils import read_float_param, read_str_list_param, read_str_param

DEFAULT_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


@dataclass
class JointSnapshot:
    positions: Dict[str, float]
    stamp_ns: int


class JointTrajectorySmokeTest(Node):
    """Send a single-point JointTrajectory and confirm state changes."""

    def __init__(self) -> None:
        super().__init__("jt_smoke_test")
        self.declare_parameter(
            "traj_topic", "/joint_trajectory_controller/joint_trajectory"
        )
        self.declare_parameter(
            "traj_action", "/joint_trajectory_controller/follow_joint_trajectory"
        )
        self.declare_parameter("joints", DEFAULT_JOINTS)
        self.declare_parameter("delta_rad", 0.2)
        self.declare_parameter("move_sec", 2.0)
        self.declare_parameter("timeout_sec", 5.0)
        self.declare_parameter("min_change_rad", 0.02)

        self._traj_topic = read_str_param(
            self,
            "traj_topic",
            "/joint_trajectory_controller/joint_trajectory",
        )
        self._traj_action = read_str_param(
            self,
            "traj_action",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self._joints = read_str_list_param(self, "joints", default=DEFAULT_JOINTS)
        self._delta = read_float_param(self, "delta_rad", 0.2)
        self._move_sec = read_float_param(self, "move_sec", 2.0, min_value=0.1)
        self._timeout_sec = read_float_param(self, "timeout_sec", 5.0, min_value=0.5)
        self._min_change = read_float_param(
            self, "min_change_rad", 0.02, min_value=0.001
        )

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._pub = self.create_publisher(JointTrajectory, self._traj_topic, qos)
        self._action = ActionClient(
            self,
            FollowJointTrajectory,
            self._traj_action,
        )
        self._sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self._timer = self.create_timer(0.1, self._on_timer)

        self._initial: JointSnapshot | None = None
        self._last_state: JointSnapshot | None = None
        self._sent_state: JointSnapshot | None = None
        self._sent = False
        self._goal_done = False
        self._goal_status = ""
        self._sent_time: Time | None = None
        self._sent_wall_s: float | None = None

        self.get_logger().info(
            f"JT smoke test ready (topic={self._traj_topic}, action={self._traj_action}, joints={len(self._joints)})"
        )

    def _now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _on_joint_state(self, msg: JointState) -> None:
        positions = {}
        for name, pos in zip(msg.name, msg.position):
            positions[name] = pos
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        snapshot = JointSnapshot(positions=positions, stamp_ns=stamp_ns)
        self._last_state = snapshot
        if self._initial is None and self._has_all_joints(snapshot):
            self._initial = snapshot

    def _has_all_joints(self, snapshot: JointSnapshot) -> bool:
        return all(joint in snapshot.positions for joint in self._joints)

    def _build_target(self, snapshot: JointSnapshot) -> List[float]:
        positions = [snapshot.positions[joint] for joint in self._joints]
        if positions:
            positions[0] += self._delta
        return positions

    def _on_goal_result(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._goal_status = f"result_error:{exc}"
            self.get_logger().warn(f"Action result error: {exc}")
            return
        status = int(getattr(result, "status", 0))
        self._goal_done = True
        self._goal_status = str(status)
        self.get_logger().info(f"FollowJointTrajectory result status={status}")

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._goal_status = f"goal_response_error:{exc}"
            self.get_logger().warn(f"Action goal response error: {exc}")
            return
        if not goal_handle.accepted:
            self._goal_done = True
            self._goal_status = "rejected"
            self.get_logger().warn("FollowJointTrajectory goal rejected")
            return
        goal_handle.get_result_async().add_done_callback(self._on_goal_result)

    def _publish_trajectory(self) -> None:
        if self._initial is None:
            return
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(self._joints)
        point = JointTrajectoryPoint()
        point.positions = self._build_target(self._initial)
        point.time_from_start.sec = int(self._move_sec)
        point.time_from_start.nanosec = int((self._move_sec % 1.0) * 1e9)
        traj.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        if self._action.wait_for_server(timeout_sec=2.0):
            self._action.send_goal_async(goal).add_done_callback(self._on_goal_response)
            self.get_logger().info("FollowJointTrajectory goal sent")
        else:
            self._pub.publish(traj)
            self.get_logger().warn(
                "FollowJointTrajectory action unavailable; fallback a topic publish"
            )
        self._sent = True
        self._sent_time = self.get_clock().now()
        self._sent_wall_s = time.monotonic()
        self._sent_state = self._last_state

    def _detect_movement(self) -> bool:
        baseline = self._sent_state or self._initial
        if baseline is None or self._last_state is None:
            return False
        if not self._has_all_joints(self._last_state) or not self._has_all_joints(
            baseline
        ):
            return False
        for joint in self._joints:
            before = baseline.positions[joint]
            after = self._last_state.positions[joint]
            if abs(after - before) >= self._min_change:
                return True
        return False

    def _on_timer(self) -> None:
        if not self._sent:
            if self._initial is None:
                return
            self._publish_trajectory()
            return

        if self._detect_movement():
            self.get_logger().info("JointTrajectory executed (joint_states changed)")
            rclpy.shutdown()
            return

        if self._sent_wall_s is not None:
            elapsed = time.monotonic() - self._sent_wall_s
            if elapsed > self._timeout_sec:
                if self._goal_status:
                    self.get_logger().warn(
                        f"JointTrajectory timeout: no joint_states change (goal_status={self._goal_status})"
                    )
                else:
                    self.get_logger().warn(
                        "JointTrajectory timeout: no joint_states change"
                    )
                rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = JointTrajectorySmokeTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
