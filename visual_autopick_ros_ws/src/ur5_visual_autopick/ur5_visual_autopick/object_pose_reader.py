"""Object pose reader: subscribes to gz_pose_bridge TFMessage and extracts object poses."""
from __future__ import annotations

import threading
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


class ObjectPoseReader:
    """Reads object world poses from /world/<world>/pose/info (TFMessage from gz_pose_bridge).

    Each TransformStamped in the TFMessage has:
      header.frame_id = "world"
      child_frame_id  = object name (e.g. "pick_demo")
      transform.translation = world position
    """

    def __init__(self, node: Node, topic: str) -> None:
        self._node = node
        self._topic = topic
        self._lock = threading.Lock()
        # name -> (x, y, z, wall_time)
        self._poses: Dict[str, Tuple[float, float, float, float]] = {}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = node.create_subscription(TFMessage, topic, self._callback, qos)
        node.get_logger().info(f"[VISUAL_AUTOPICK][POSE_READER] subscribed topic={topic}")

    def _callback(self, msg: TFMessage) -> None:
        now = time.time()
        with self._lock:
            for tf in msg.transforms:
                name = tf.child_frame_id
                if not name:
                    continue
                t = tf.transform.translation
                self._poses[name] = (t.x, t.y, t.z, now)

    def get_object_pose(
        self, name: str, max_age_sec: float = 1.0
    ) -> Tuple[Optional[Tuple[float, float, float]], float, str]:
        """Return ((x,y,z), age_sec, error_str).  error_str='' on success."""
        with self._lock:
            entry = self._poses.get(name)

        if entry is None:
            return None, 0.0, f"object '{name}' not seen yet"

        x, y, z, stamp = entry
        age_sec = time.time() - stamp
        if age_sec > max_age_sec:
            return None, age_sec, f"STALE age_sec={age_sec:.3f} > max={max_age_sec}"

        return (x, y, z), age_sec, ""

    def known_names(self) -> list:
        with self._lock:
            return list(self._poses.keys())
