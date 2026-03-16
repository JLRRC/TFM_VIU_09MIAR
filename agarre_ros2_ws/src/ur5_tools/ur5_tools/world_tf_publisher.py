#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py
# Summary: Publishes world->base_link TF from Gazebo pose/info.
"""Publish world->base_link using Gazebo model pose bridged to ROS."""

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
from tf2_ros import TransformBroadcaster


class WorldTfPublisher(Node):
    """Publish world->base_link from /world/<world>/pose/info."""

    def __init__(self) -> None:
        super().__init__("world_tf_publisher")
        self.declare_parameter("world_name", "ur5_mesa_objetos")
        self.declare_parameter("model_name", "ur5_rg2")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("world_file", "")
        self.declare_parameter("static_grace_sec", 0.0)
        self.declare_parameter("wait_for_clock", True)
        self.declare_parameter("clock_ready_min_sec", 0.1)
        self.declare_parameter("pose_timeout_sec", 5.0)
        self.declare_parameter("clock_timeout_sec", 5.0)

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
        if self._pose_timeout <= 0.0:
            self._pose_timeout = 5.0
        if self._clock_timeout <= 0.0:
            self._clock_timeout = 5.0

        self._topic = f"/world/{self._world_name}/pose/info"
        self._tf_pub = TransformBroadcaster(self)

        self._last_stamp = None
        self._last_warn = 0.0
        self._last_source = ""
        self._last_pose = None
        self._last_pose_time = 0.0
        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        self._pose_deadline = time.monotonic() + max(0.0, self._pose_timeout)
        self._clock_deadline = time.monotonic() + max(0.0, self._clock_timeout)
        self._clock_ready = not self._wait_for_clock
        self._clock_last_ns = 0
        self._clock_last_log = 0.0
        self._static_pose = None
        self._static_ready_at = time.monotonic() + max(0.0, self._static_grace)
        self._static_used = False
        self._fatal = False
        self._fatal_reason = ""
        self._sub = self.create_subscription(
            TFMessage,
            self._topic,
            self._on_pose_info,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(0.1, self._publish_timer)

        self.get_logger().info(
            f"WorldTfPublisher listening on {self._topic} "
            f"(model={self._model_name}, base={self._base_frame})"
        )
        if self._wait_for_clock:
            self.get_logger().info("Waiting for /clock before publishing TF.")
        if self._static_grace >= 0.0:
            self._static_pose = self._load_static_pose()

    def _name_from_tf(self, tf: TransformStamped) -> str:
        child = getattr(tf, "child_frame_id", "") or ""
        if child:
            return child
        header = getattr(tf, "header", None)
        return getattr(header, "frame_id", "") if header else ""

    def _rpy_to_quat(
        self,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> tuple[float, float, float, float]:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def _parse_pose_text(
        self,
        text: str,
    ) -> Optional[tuple[float, float, float, float, float, float, float]]:
        try:
            parts = [float(v) for v in text.split()]
            if len(parts) < 6:
                return None
            x, y, z, rr, pp, yy = parts[:6]
            qx, qy, qz, qw = self._rpy_to_quat(rr, pp, yy)
            if abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
                return None
            return (x, y, z, qx, qy, qz, qw)
        except Exception:
            return None

    def _strip_ns(self, tag: str) -> str:
        if "}" in tag:
            return tag.split("}", 1)[1]
        return tag

    def _load_static_pose_xml(
        self,
        content: str,
    ) -> Optional[tuple[float, float, float, float, float, float, float]]:
        try:
            root = ET.fromstring(content)
        except Exception:
            return None

        for elem in root.iter():
            elem.tag = self._strip_ns(elem.tag)

        for include in root.iter("include"):
            name_el = include.find("name")
            if name_el is None or (name_el.text or "").strip() != self._model_name:
                continue
            pose_el = include.find("pose")
            if pose_el is None or not pose_el.text:
                continue
            parsed = self._parse_pose_text(pose_el.text.strip())
            if parsed is not None:
                return parsed

        for model in root.iter("model"):
            if model.get("name") != self._model_name:
                continue
            pose_el = model.find("pose")
            if pose_el is None or not pose_el.text:
                continue
            parsed = self._parse_pose_text(pose_el.text.strip())
            if parsed is not None:
                return parsed
        return None

    def _load_static_pose(
        self,
    ) -> Optional[tuple[float, float, float, float, float, float, float]]:
        world_file = self._world_file
        if not world_file:
            ws_dir = os.environ.get(
                "WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws")
            )
            world_file = os.path.join(ws_dir, "worlds", f"{self._world_name}.sdf")
        if not os.path.isfile(world_file):
            return None
        try:
            with open(world_file, "r", encoding="utf-8") as f:
                content = f.read()
        except Exception:
            return None
        parsed = self._load_static_pose_xml(content)
        if parsed is not None:
            return parsed
        pattern = (
            rf"<include>.*?<name>\s*{re.escape(self._model_name)}\s*</name>.*?"
            r"<pose>\s*([^<]+)\s*</pose>.*?</include>"
        )
        match = re.search(pattern, content, flags=re.DOTALL)
        if not match:
            pattern = (
                rf"<model\s+name=\"{re.escape(self._model_name)}\"[^>]*>.*?"
                r"<pose>\s*([^<]+)\s*</pose>.*?</model>"
            )
            match = re.search(pattern, content, flags=re.DOTALL)
        if not match:
            return None
        return self._parse_pose_text(match.group(1).strip())

    def _is_identity(self, tf: TransformStamped) -> bool:
        t = tf.transform.translation
        r = tf.transform.rotation
        if abs(t.x) > 1e-4 or abs(t.y) > 1e-4 or abs(t.z) > 1e-4:
            return False
        if (
            abs(r.x) > 1e-4
            or abs(r.y) > 1e-4
            or abs(r.z) > 1e-4
            or abs(r.w - 1.0) > 1e-4
        ):
            return False
        return True

    @staticmethod
    def _is_zero_translation(
        pose: tuple[float, float, float, float, float, float, float],
    ) -> bool:
        tx, ty, tz, _, _, _, _ = pose
        return abs(tx) < 1e-4 and abs(ty) < 1e-4 and abs(tz) < 1e-4

    @staticmethod
    def _is_identity_pose(
        pose: tuple[float, float, float, float, float, float, float],
    ) -> bool:
        tx, ty, tz, rx, ry, rz, rw = pose
        if abs(tx) > 1e-4 or abs(ty) > 1e-4 or abs(tz) > 1e-4:
            return False
        if abs(rx) > 1e-4 or abs(ry) > 1e-4 or abs(rz) > 1e-4 or abs(rw - 1.0) > 1e-4:
            return False
        return True

    def _score_name(self, name: str) -> int:
        if name == self._model_name:
            return 100
        if name.endswith("::base_link"):
            return 95
        if name == self._base_frame:
            return 90
        if self._model_name and self._model_name in name:
            return 80
        if name.endswith(self._base_frame):
            return 70
        if self._base_frame and self._base_frame in name:
            return 60
        return 0

    def _select_transform(self, msg: TFMessage) -> Optional[TransformStamped]:
        names: dict[str, TransformStamped] = {}
        for tf in msg.transforms:
            name = self._name_from_tf(tf)
            if name:
                names[name] = tf
        best = None
        best_score = -1
        for name, tf in names.items():
            score = self._score_name(name)
            if score <= 0:
                continue
            if self._is_identity(tf):
                continue
            if score > best_score:
                best = (name, tf)
                best_score = score
        if best is None:
            return None
        self._last_source = best[0]
        return best[1]

    def _stamp_valid(self, tf: TransformStamped) -> bool:
        stamp = getattr(tf, "header", None)
        if stamp is None:
            return False
        try:
            sec = int(stamp.stamp.sec)
            nsec = int(stamp.stamp.nanosec)
        except Exception:
            return False
        return sec > 0 or nsec > 0

    def _on_pose_info(self, msg: TFMessage) -> None:
        available = []
        for tf in msg.transforms:
            name = self._name_from_tf(tf)
            if name:
                available.append(name)
        tf = self._select_transform(msg)
        if tf is None:
            now = time.monotonic()
            if (now - self._last_warn) > 2.0:
                sample = ", ".join(available[:12])
                suffix = "..." if len(available) > 12 else ""
                self.get_logger().warn(
                    f"No pose for model '{self._model_name}' yet on {self._topic}. "
                    f"Available: {sample}{suffix}"
                )
                self._last_warn = now
            return
        if self._is_identity(tf):
            now = time.monotonic()
            if (now - self._last_warn) > 2.0:
                self.get_logger().warn(
                    "Pose info devolvió identidad para el UR5; esperando pose real."
                )
                self._last_warn = now
            return
        pose = (
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        )
        self._last_pose = pose
        self._last_pose_time = self.get_clock().now().nanoseconds * 1e-9
        self._last_stamp = tf.header.stamp if self._stamp_valid(tf) else None

    def _abort(self, reason: str) -> None:
        if self._fatal:
            return
        self._fatal = True
        self._fatal_reason = reason
        ctx = f"(world={self._world_name} model={self._model_name} base={self._base_frame})"
        self.get_logger().error(f"{reason} {ctx}")
        try:
            self.destroy_node()
        except Exception:
            pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass

    def _publish_timer(self) -> None:
        if self._fatal:
            return
        if self._wait_for_clock and not self._clock_ready:
            now_ns = self.get_clock().now().nanoseconds
            now = now_ns * 1e-9
            wall_now = time.monotonic()
            if wall_now > self._clock_deadline:
                self._abort("Clock no disponible; abortando world_tf_publisher.")
                return
            if now_ns <= 0:
                if (now - self._clock_last_log) > 2.0:
                    self.get_logger().warn("Esperando /clock (tiempo=0); TF bloqueado.")
                    self._clock_last_log = now
                return
            if self._clock_last_ns == 0:
                self._clock_last_ns = now_ns
                return
            if now_ns <= self._clock_last_ns:
                self._clock_last_ns = now_ns
                return
            if (now_ns - self._clock_last_ns) < self._clock_ready_min_ns:
                self._clock_last_ns = now_ns
                return
            self._clock_ready = True
            self.get_logger().info("/clock listo; publicando TF.")

        if self._last_pose is None and self._static_pose is not None:
            if time.monotonic() >= self._static_ready_at and not self._is_identity_pose(
                self._static_pose
            ):
                self._last_pose = self._static_pose
                self._last_pose_time = self.get_clock().now().nanoseconds * 1e-9
                if not self._static_used:
                    ctx = (
                        f"(world={self._world_name} "
                        f"model={self._model_name} "
                        f"base={self._base_frame})"
                    )
                    self.get_logger().warn(
                        "Pose/info no disponible; usando pose estática del world file para TF. "
                        f"{ctx}"
                    )
                    self._static_used = True

        if self._last_pose is None:
            now = time.monotonic()
            if now > self._pose_deadline:
                self._abort("Pose/info no disponible; abortando world_tf_publisher.")
                return
            if (now - self._last_warn) > 2.0:
                self.get_logger().warn("Sin pose válida del UR5; no se publica TF.")
                self._last_warn = now
            return
        tx, ty, tz, rx, ry, rz, rw = self._last_pose
        if self._is_identity_pose(self._last_pose):
            return
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
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.try_shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
