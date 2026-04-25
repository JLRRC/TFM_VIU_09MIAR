"""MoveIt TCP client: sends PoseStamped to /desired_grasp and waits for result.

The existing ur5_moveit_bridge subscribes to /desired_grasp (PoseStamped)
and publishes JSON results to /desired_grasp/result (std_msgs/String).

This client does NOT import any Python module from agarre_ros2_ws.
It communicates exclusively via ROS topics.

Result JSON schema (from ur5_moveit_bridge):
  {
    "request_id": int,
    "request_uuid": str,
    "success": bool,
    "plan_ok": bool,
    "exec_ok": bool,
    "message": str,
    "backend": str,
    "cartesian": bool,
    "frame_id": str,
    "ee_link": str,
    ...
  }
"""
from __future__ import annotations

import json
import threading
import time
import uuid
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String


class MoveItTcpClient:
    """Publishes PoseStamped to the bridge and waits for the JSON result.

    Heartbeat topic is monitored to detect bridge liveness.
    """

    GRASP_TOPIC = "/desired_grasp"
    RESULT_TOPIC = "/desired_grasp/result"
    HEARTBEAT_TOPIC = "/ur5_moveit_bridge/heartbeat"

    def __init__(self, node: Node) -> None:
        self._node = node
        self._lock = threading.Lock()
        self._latest_result: Optional[Dict] = None
        self._result_event = threading.Event()
        self._heartbeat_last_wall = 0.0

        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = node.create_publisher(PoseStamped, self.GRASP_TOPIC, qos_cmd)
        self._result_sub = node.create_subscription(
            String, self.RESULT_TOPIC, self._result_callback, qos_cmd
        )
        self._heartbeat_sub = node.create_subscription(
            Bool, self.HEARTBEAT_TOPIC, self._heartbeat_callback, 10
        )
        node.get_logger().info(
            f"[VISUAL_AUTOPICK][MOVEIT_CLIENT] "
            f"cmd={self.GRASP_TOPIC} result={self.RESULT_TOPIC} hb={self.HEARTBEAT_TOPIC}"
        )

    def _result_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._latest_result = data
        self._result_event.set()

    def _heartbeat_callback(self, msg: Bool) -> None:
        self._heartbeat_last_wall = time.time()

    def is_bridge_alive(self, max_age_sec: float = 3.0) -> bool:
        """True if a heartbeat was received within max_age_sec."""
        age = time.time() - self._heartbeat_last_wall
        return self._heartbeat_last_wall > 0.0 and age < max_age_sec

    def move_tcp(
        self,
        x: float,
        y: float,
        z: float,
        frame_id: str = "base_link",
        timeout_sec: float = 10.0,
    ) -> Tuple[bool, str]:
        """Send a move request and block until result or timeout.

        Returns (success, message).
        """
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # Neutral orientation (pointing down): w=1 = identity → bridge will handle
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        # Clear previous result
        with self._lock:
            self._latest_result = None
        self._result_event.clear()

        self._pub.publish(msg)
        self._node.get_logger().info(
            f"[VISUAL_AUTOPICK][MOVEIT_CLIENT][SEND] "
            f"frame={frame_id} pos=({x:.3f},{y:.3f},{z:.3f})"
        )

        # Wait for result
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            fired = self._result_event.wait(timeout=0.1)
            if fired:
                with self._lock:
                    result = dict(self._latest_result or {})
                success = bool(result.get("success", False))
                message = str(result.get("message", ""))
                self._node.get_logger().info(
                    f"[VISUAL_AUTOPICK][MOVEIT_CLIENT][RESULT] "
                    f"success={success} msg={message}"
                )
                return success, message
            self._result_event.clear()

        return False, f"timeout after {timeout_sec:.1f}s waiting for bridge result"
