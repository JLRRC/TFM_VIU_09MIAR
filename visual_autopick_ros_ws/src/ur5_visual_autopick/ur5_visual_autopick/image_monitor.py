"""Image monitor: subscribes to a camera topic and tracks received frames."""
from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


class ImageMonitor:
    """Thread-safe monitor for an image topic.

    Tracks frame count, last timestamp, and latest Image message.
    """

    def __init__(self, node: Node, topic: str) -> None:
        self._node = node
        self._topic = topic
        self._lock = threading.Lock()
        self._frame_count = 0
        self._last_stamp_wall = 0.0
        self._last_msg: Optional[Image] = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = node.create_subscription(Image, topic, self._callback, qos)
        node.get_logger().info(f"[VISUAL_AUTOPICK][IMAGE_MONITOR] subscribed topic={topic}")

    def _callback(self, msg: Image) -> None:
        with self._lock:
            self._frame_count += 1
            self._last_stamp_wall = time.time()
            self._last_msg = msg

    def get_status(self) -> dict:
        """Return dict with: received, frame_count, width, height, age_sec."""
        with self._lock:
            count = self._frame_count
            stamp = self._last_stamp_wall
            msg = self._last_msg

        if msg is None or count == 0:
            return {"received": False, "frame_count": 0, "width": 0, "height": 0, "age_sec": None}

        age_sec = time.time() - stamp
        return {
            "received": True,
            "frame_count": count,
            "width": msg.width,
            "height": msg.height,
            "age_sec": age_sec,
        }

    def is_valid(self, min_frames: int = 3, max_age_sec: float = 1.0) -> bool:
        s = self.get_status()
        if not s["received"]:
            return False
        if s["frame_count"] < min_frames:
            return False
        age = s["age_sec"]
        if age is None or age > max_age_sec:
            return False
        return True

    def get_latest_image(self) -> Optional[Image]:
        with self._lock:
            return self._last_msg
