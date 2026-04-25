"""ROS 2 client for the step_pick_server."""
from __future__ import annotations

from std_msgs.msg import String


class StepPickClient:
    """Publishes commands to /visual_autopick/step_command and reads /visual_autopick/status."""

    CMD_TOPIC = "/visual_autopick/step_command"
    STATUS_TOPIC = "/visual_autopick/status"

    def __init__(self, ros_node, on_status=None):
        self._node = ros_node
        self._pub = ros_node.create_publisher(String, self.CMD_TOPIC, 10)
        self._last_status = "IDLE"
        if on_status:
            ros_node.create_subscription(String, self.STATUS_TOPIC, on_status, 10)

    def send_command(self, mode: str, phase: str) -> None:
        msg = String()
        msg.data = f"{mode.upper()}:{phase.upper()}"
        self._pub.publish(msg)
        self._node.get_logger().info(f"[CMD] {msg.data}")

    def send_direct(self, phase: str) -> None:
        self.send_command("DIRECTO", phase)

    def send_moveit(self, phase: str) -> None:
        self.send_command("MOVEIT", phase)

    def abort(self, mode: str = "DIRECTO") -> None:
        self.send_command(mode, "ABORT")
