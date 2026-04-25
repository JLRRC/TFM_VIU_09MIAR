#!/usr/bin/env python3
"""Publish world->base_link TF from Gazebo pose/info topic."""
from __future__ import annotations

import math
import os
import re
import time
from typing import Optional
import xml.etree.ElementTree as ET

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class WorldTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__("world_tf_publisher")
        self.declare_parameter("world_name", "visual_autopick_world")
        self.declare_parameter("model_name", "ur5_rg2")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("world_file", "")
        self.declare_parameter("static_grace_sec", -1.0)
        self.declare_parameter("wait_for_clock", True)
        self.declare_parameter("clock_ready_min_sec", 0.5)
        self.declare_parameter("pose_timeout_sec", 10.0)
        self.declare_parameter("clock_timeout_sec", 10.0)

        self._world_name = str(self.get_parameter("world_name").value)
        self._model_name = str(self.get_parameter("model_name").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._world_file = str(self.get_parameter("world_file").value)
        self._static_grace = float(self.get_parameter("static_grace_sec").value)
        self._wait_for_clock = bool(self.get_parameter("wait_for_clock").value)
        self._clock_ready_min_ns = int(
            float(self.get_parameter("clock_ready_min_sec").value) * 1e9
        )
        self._pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
        self._clock_timeout = float(self.get_parameter("clock_timeout_sec").value)

        self._topic = f"/world/{self._world_name}/pose/info"
        self._tf_pub = TransformBroadcaster(self)
        self._static_tf_pub = StaticTransformBroadcaster(self)

        self._last_pose: Optional[tuple] = None
        self._last_pose_time = 0.0
        self._last_warn = 0.0
        self._last_source = ""
        self._start_time = time.monotonic()
        self._pose_deadline = time.monotonic() + max(0.0, self._pose_timeout)
        self._clock_deadline = time.monotonic() + max(0.0, self._clock_timeout)
        self._clock_ready = not self._wait_for_clock
        self._clock_last_ns = 0
        self._clock_last_log = 0.0
        self._static_pose: Optional[tuple] = None
        self._static_ready_at = time.monotonic() + max(0.0, self._static_grace)
        self._static_used = False

        self._sub = self.create_subscription(
            TFMessage, self._topic, self._on_pose_info, qos_profile_sensor_data
        )
        self._timer = self.create_timer(0.1, self._publish_timer)

        self.get_logger().info(
            f"WorldTfPublisher: topic={self._topic} model={self._model_name}"
        )

        self._static_pose = self._load_static_pose()
        if self._static_grace < 0.0:
            self._publish_static_prewarm()

    # ── Static pre-warm ────────────────────────────────────────────────────────

    def _publish_static_prewarm(self) -> None:
        if self._static_pose is None or self._is_identity_pose(self._static_pose):
            self.get_logger().warn("No static pose available for pre-warm; skipping.")
            return
        tx, ty, tz, rx, ry, rz, rw = self._static_pose
        out = TransformStamped()
        out.header.stamp.sec = 0
        out.header.stamp.nanosec = 0
        out.header.frame_id = self._world_frame or "world"
        out.child_frame_id = self._base_frame
        out.transform.translation.x = tx
        out.transform.translation.y = ty
        out.transform.translation.z = tz
        out.transform.rotation.x = rx
        out.transform.rotation.y = ry
        out.transform.rotation.z = rz
        out.transform.rotation.w = rw
        self._static_tf_pub.sendTransform(out)
        self.get_logger().info(
            f"[PRE-WARM] {self._world_frame}->{self._base_frame} "
            f"x={tx:.3f} y={ty:.3f} z={tz:.3f}"
        )
        self._static_used = True

    # ── SDF parsing ────────────────────────────────────────────────────────────

    def _rpy_to_quat(self, roll: float, pitch: float, yaw: float):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def _parse_pose_text(self, text: str) -> Optional[tuple]:
        try:
            parts = [float(v) for v in text.split()]
            if len(parts) < 6:
                return None
            x, y, z, rr, pp, yy = parts[:6]
            if abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
                return None
            qx, qy, qz, qw = self._rpy_to_quat(rr, pp, yy)
            return (x, y, z, qx, qy, qz, qw)
        except Exception:
            return None

    def _load_static_pose(self) -> Optional[tuple]:
        world_file = self._world_file
        if not world_file or not os.path.isfile(world_file):
            self.get_logger().warn(
                f"world_file not found: '{world_file}'. Static pre-warm disabled."
            )
            return None
        try:
            with open(world_file, "r", encoding="utf-8") as f:
                content = f.read()
        except Exception as e:
            self.get_logger().warn(f"Cannot read world_file: {e}")
            return None
        return self._parse_sdf_for_model(content)

    def _parse_sdf_for_model(self, content: str) -> Optional[tuple]:
        try:
            root = ET.fromstring(content)
            for elem in root.iter():
                if "}" in elem.tag:
                    elem.tag = elem.tag.split("}", 1)[1]
            for model in root.iter("model"):
                if model.get("name") != self._model_name:
                    continue
                pose_el = model.find("pose")
                if pose_el is not None and pose_el.text:
                    parsed = self._parse_pose_text(pose_el.text.strip())
                    if parsed is not None:
                        return parsed
        except Exception:
            pass
        # regex fallback
        pattern = (
            rf"<model\s+name=\"{re.escape(self._model_name)}\"[^>]*>.*?"
            r"<pose>\s*([^<]+)\s*</pose>"
        )
        m = re.search(pattern, content, flags=re.DOTALL)
        if m:
            return self._parse_pose_text(m.group(1).strip())
        return None

    # ── Subscription callback ─────────────────────────────────────────────────

    def _name_from_tf(self, tf: TransformStamped) -> str:
        child = getattr(tf, "child_frame_id", "") or ""
        if child:
            return child
        header = getattr(tf, "header", None)
        return getattr(header, "frame_id", "") if header else ""

    def _score_name(self, name: str) -> int:
        if name == f"{self._model_name}::{self._base_frame}":
            return 120
        if name == self._base_frame:
            return 110
        if name.endswith("::base_link"):
            return 100
        if name.endswith(f"::{self._base_frame}"):
            return 95
        if name.endswith(self._base_frame):
            return 85
        if name == self._model_name:
            return 50
        return 0

    def _is_identity(self, tf: TransformStamped) -> bool:
        t = tf.transform.translation
        r = tf.transform.rotation
        return (
            abs(t.x) < 1e-4 and abs(t.y) < 1e-4 and abs(t.z) < 1e-4
            and abs(r.x) < 1e-4 and abs(r.y) < 1e-4
            and abs(r.z) < 1e-4 and abs(r.w - 1.0) < 1e-4
        )

    @staticmethod
    def _is_identity_pose(pose: tuple) -> bool:
        tx, ty, tz, rx, ry, rz, rw = pose
        return (
            abs(tx) < 1e-4 and abs(ty) < 1e-4 and abs(tz) < 1e-4
            and abs(rx) < 1e-4 and abs(ry) < 1e-4
            and abs(rz) < 1e-4 and abs(rw - 1.0) < 1e-4
        )

    def _on_pose_info(self, msg: TFMessage) -> None:
        best = None
        best_score = -1
        for tf in msg.transforms:
            name = self._name_from_tf(tf)
            score = self._score_name(name)
            if score <= 0 or self._is_identity(tf):
                continue
            if score > best_score:
                best = tf
                best_score = score
        if best is None:
            return
        t = best.transform.translation
        r = best.transform.rotation
        self._last_pose = (t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        self._last_pose_time = self.get_clock().now().nanoseconds * 1e-9

    # ── Timer: publish TF ──────────────────────────────────────────────────────

    def _publish_timer(self) -> None:
        if self._wait_for_clock and not self._clock_ready:
            now_ns = self.get_clock().now().nanoseconds
            wall_now = time.monotonic()
            if now_ns <= 0:
                if wall_now - self._clock_last_log > 2.0:
                    self.get_logger().info("Waiting for /clock...")
                    self._clock_last_log = wall_now
                return
            if self._clock_last_ns == 0:
                self._clock_last_ns = now_ns
                return
            if now_ns > self._clock_last_ns and (now_ns - self._clock_last_ns) >= self._clock_ready_min_ns:
                self._clock_ready = True
                self.get_logger().info("/clock ready.")
            self._clock_last_ns = now_ns
            if not self._clock_ready:
                return

        if self._last_pose is None and self._static_pose is not None:
            if time.monotonic() >= self._static_ready_at and not self._is_identity_pose(self._static_pose):
                self._last_pose = self._static_pose
                if not self._static_used:
                    self.get_logger().warn("Using static pose from world file (Gazebo pose not yet received).")
                    self._static_used = True

        if self._last_pose is None:
            now = time.monotonic()
            if now - self._last_warn > 2.0:
                self.get_logger().warn(
                    f"No valid pose for '{self._model_name}' yet on {self._topic}"
                )
                self._last_warn = now
            return

        if self._is_identity_pose(self._last_pose):
            return

        tx, ty, tz, rx, ry, rz, rw = self._last_pose
        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._world_frame or "world"
        out.child_frame_id = self._base_frame
        out.transform.translation.x = tx
        out.transform.translation.y = ty
        out.transform.translation.z = tz
        out.transform.rotation.x = rx
        out.transform.rotation.y = ry
        out.transform.rotation.z = rz
        out.transform.rotation.w = rw
        self._tf_pub.sendTransform(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldTfPublisher()
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
