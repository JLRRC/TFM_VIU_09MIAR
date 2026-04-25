#!/usr/bin/env python3
"""Read pick_demo pose from Gazebo pose/info and publish as PoseStamped."""
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage


class ObjectPoseProvider(Node):
    def __init__(self) -> None:
        super().__init__("object_pose_provider")
        self.declare_parameter("world_name", "visual_autopick_world")
        self.declare_parameter("object_name", "pick_demo")
        self.declare_parameter("world_frame", "world")

        self._world_name = str(self.get_parameter("world_name").value)
        self._object_name = str(self.get_parameter("object_name").value)
        self._world_frame = str(self.get_parameter("world_frame").value)

        topic = f"/world/{self._world_name}/pose/info"
        self._sub = self.create_subscription(
            TFMessage, topic, self._on_pose, qos_profile_sensor_data
        )
        self._pub = self.create_publisher(PoseStamped, "/visual_autopick/object_pose", 10)
        self.get_logger().info(
            f"ObjectPoseProvider: watching '{self._object_name}' on {topic}"
        )

    def _on_pose(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            child = getattr(tf, "child_frame_id", "") or ""
            if self._object_name not in child:
                continue
            t = tf.transform.translation
            r = tf.transform.rotation
            if abs(t.x) < 1e-6 and abs(t.y) < 1e-6 and abs(t.z) < 1e-6:
                continue
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = self._world_frame
            ps.pose.position.x = t.x
            ps.pose.position.y = t.y
            ps.pose.position.z = t.z
            ps.pose.orientation.x = r.x
            ps.pose.orientation.y = r.y
            ps.pose.orientation.z = r.z
            ps.pose.orientation.w = r.w
            self._pub.publish(ps)
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectPoseProvider()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    node.destroy_node()
    try:
        rclpy.try_shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
