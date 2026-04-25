"""Gripper client: publishes Float64MultiArray to /gripper_controller/commands.

The existing stack uses /gripper_controller/commands (std_msgs/Float64MultiArray)
with data=[joint1_pos, joint2_pos].  open=1.0 rad, closed=0.0 rad, joint2_sign=1.0.

Validation uses /joint_states (sensor_msgs/JointState) to measure finger positions.
"""
from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT1 = "rg2_finger_joint1"
JOINT2 = "rg2_finger_joint2"
OPEN_POS = 1.0
CLOSED_POS = 0.0
JOINT2_SIGN = 1.0
CMD_TOPIC = "/gripper_controller/commands"
JS_TOPIC = "/joint_states"


class GripperClient:
    """Sends open/close commands to the gripper and reads finger joint states."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._lock = threading.Lock()
        self._joint_pos: dict = {}
        self._joint_wall = 0.0

        self._pub = node.create_publisher(Float64MultiArray, CMD_TOPIC, 10)
        self._js_sub = node.create_subscription(
            JointState, JS_TOPIC, self._js_callback, 20
        )
        node.get_logger().info(
            f"[VISUAL_AUTOPICK][GRIPPER_CLIENT] cmd={CMD_TOPIC} js={JS_TOPIC}"
        )

    def _js_callback(self, msg: JointState) -> None:
        with self._lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._joint_pos[name] = float(msg.position[i])
            self._joint_wall = time.time()

    def _get_finger_sum(self) -> Optional[float]:
        with self._lock:
            p1 = self._joint_pos.get(JOINT1)
            p2 = self._joint_pos.get(JOINT2)
        if p1 is None or p2 is None:
            return None
        return abs(p1) + abs(p2)

    def _send_command(self, target: float) -> None:
        msg = Float64MultiArray()
        msg.data = [float(target), float(target) * JOINT2_SIGN]
        self._pub.publish(msg)

    def open_gripper(self) -> bool:
        self._send_command(OPEN_POS)
        self._node.get_logger().info(
            f"[VISUAL_AUTOPICK][GRIPPER] command=open target={OPEN_POS:.3f} rad"
        )
        return True

    def close_gripper(
        self,
        timeout_sec: float = 4.0,
        min_delta_sum: float = 0.01,
    ) -> Tuple[bool, float, float, float]:
        """Close the gripper.

        Returns (success, pre_sum, post_sum, delta).
        """
        pre_sum = self._get_finger_sum()
        if pre_sum is None:
            pre_sum = 0.0
            self._node.get_logger().warning(
                "[VISUAL_AUTOPICK][GRIPPER] no joint_states yet; pre_sum=0"
            )

        self._send_command(CLOSED_POS)
        self._node.get_logger().info(
            f"[VISUAL_AUTOPICK][GRIPPER] command=close target={CLOSED_POS:.3f} rad "
            f"pre_sum={pre_sum:.4f}"
        )

        # Wait for fingers to move
        deadline = time.monotonic() + timeout_sec
        post_sum = pre_sum
        while time.monotonic() < deadline:
            time.sleep(0.1)
            s = self._get_finger_sum()
            if s is not None:
                post_sum = s

        delta = pre_sum - post_sum  # positive means fingers closed (sum decreased)
        success = delta >= min_delta_sum

        verdict = "CLOSED" if success else "FAILED"
        self._node.get_logger().info(
            f"[VISUAL_AUTOPICK][GRIPPER] command=close "
            f"pre_opening_sum={pre_sum:.4f} post_opening_sum={post_sum:.4f} "
            f"delta={delta:.4f} verdict={verdict}"
        )
        return success, pre_sum, post_sum, delta

    def get_finger_opening_sum(self) -> Optional[float]:
        """Return sum of absolute finger joint positions (proxy for opening)."""
        return self._get_finger_sum()
