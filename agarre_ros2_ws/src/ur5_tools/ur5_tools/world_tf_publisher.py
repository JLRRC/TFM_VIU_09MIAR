#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/world_tf_publisher.py
# Summary: Publishes world->base_link TF from Gazebo pose/info.
"""Publish world->base_link using Gazebo model pose bridged to ROS.

F13 (2026-05-01): este nodo migró a ``LifecycleNode``. Para preservar
el comportamiento de los launch existentes, el constructor declara
parámetros y el parámetro ``auto_activate`` (default True) hace que
``main()`` dispare ``configure → activate`` automáticamente. Si se
pasa ``auto_activate:=false`` el nodo queda en ``UNCONFIGURED``
esperando control externo (``ros2 lifecycle set ...``).
"""

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
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class WorldTfPublisher(LifecycleNode):
    """Publish world->base_link from /world/<world>/pose/info.

    LifecycleNode con auto-activate (F13). Recursos:
      - ``on_configure``: lee parámetros, crea broadcasters, carga
        pose estática del world file si aplica.
      - ``on_activate``: crea suscripción al topic Gazebo y timer.
      - ``on_deactivate``: cancela timer y suscripción.
      - ``on_cleanup``: libera broadcasters y resetea estado.
    """

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
        # F13 lifecycle: auto-activate por defecto preserva
        # backward-compat con los launch que no transicionan el nodo.
        self.declare_parameter("auto_activate", True)

        self._tf_pub: Optional[TransformBroadcaster] = None
        self._static_tf_pub: Optional[StaticTransformBroadcaster] = None
        self._sub = None
        self._timer = None
        self._topic = ""
        self._world_name = ""
        self._model_name = ""
        self._base_frame = ""
        self._world_frame = ""
        self._world_file = ""
        self._static_grace = 0.0
        self._wait_for_clock = True
        self._clock_ready_min_ns = 0
        self._pose_timeout = 0.0
        self._clock_timeout = 0.0

        self._last_stamp = None
        self._last_warn = 0.0
        self._last_source = ""
        self._last_pose = None
        self._last_pose_time = 0.0
        self._start_time = 0.0
        self._pose_deadline = 0.0
        self._clock_deadline = 0.0
        self._clock_ready = False
        self._clock_last_ns = 0
        self._clock_last_log = 0.0
        self._static_pose = None
        self._static_ready_at = 0.0
        self._static_used = False
        self._fatal = False
        self._fatal_reason = ""

    # ------------------------------------------------------------------
    # Lifecycle transitions (F13)
    # ------------------------------------------------------------------

    def on_configure(self, _state) -> TransitionCallbackReturn:
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
        # Static broadcaster used only for the pre-warm transform when static_grace_sec < 0.
        # Published once on /tf_static so tf2 consumers have world->base_link immediately,
        # before Gazebo and /clock are ready.  Overridden by dynamic TF once Gazebo is live.
        self._static_tf_pub = StaticTransformBroadcaster(self)

        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        self._pose_deadline = time.monotonic() + max(0.0, self._pose_timeout)
        self._clock_deadline = time.monotonic() + max(0.0, self._clock_timeout)
        self._clock_ready = not self._wait_for_clock
        self._clock_last_ns = 0
        self._clock_last_log = 0.0
        self._static_ready_at = time.monotonic() + max(0.0, self._static_grace)
        self._static_used = False
        self._fatal = False
        self._fatal_reason = ""

        if self._static_grace >= 0.0:
            self._static_pose = self._load_static_pose()
        else:
            # static_grace_sec < 0: publish world->base_link immediately on /tf_static
            # without waiting for /clock or Gazebo. Eliminates TF-null window at startup.
            self._static_pose = self._load_static_pose()
            self._publish_static_prewarm()

        self.get_logger().info(
            f"[LIFECYCLE] WorldTfPublisher configured (topic={self._topic}, "
            f"model={self._model_name}, base={self._base_frame})"
        )
        if self._wait_for_clock:
            self.get_logger().info("Waiting for /clock before publishing TF.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state) -> TransitionCallbackReturn:
        self._sub = self.create_subscription(
            TFMessage,
            self._topic,
            self._on_pose_info,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(0.1, self._publish_timer)
        self.get_logger().info(
            f"[LIFECYCLE] WorldTfPublisher activated; listening on {self._topic}"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state) -> TransitionCallbackReturn:
        if self._timer is not None:
            try:
                self._timer.cancel()
                self.destroy_timer(self._timer)
            except Exception:
                pass
            self._timer = None
        if self._sub is not None:
            try:
                self.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None
        self.get_logger().info("[LIFECYCLE] WorldTfPublisher deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state) -> TransitionCallbackReturn:
        # Por si on_deactivate no se llamó por alguna razón.
        if self._timer is not None:
            try:
                self.destroy_timer(self._timer)
            except Exception:
                pass
            self._timer = None
        if self._sub is not None:
            try:
                self.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None
        self._tf_pub = None
        self._static_tf_pub = None
        self._last_pose = None
        self._last_stamp = None
        self._static_pose = None
        self.get_logger().info("[LIFECYCLE] WorldTfPublisher cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state) -> TransitionCallbackReturn:
        return self.on_cleanup(_state)

    def _publish_static_prewarm(self) -> None:
        """Publish world->base_link on /tf_static immediately, before clock/Gazebo are ready.

        Uses stamp=0 (the tf2 convention for "always valid") so tf2 consumers can look up
        world->base_link from the very first second of startup.  The dynamic TF published
        by _publish_timer will automatically take precedence once Gazebo is live.
        """
        if self._static_pose is None or self._is_identity_pose(self._static_pose):
            self.get_logger().warn(
                "static_grace_sec<0 pero no hay pose estática disponible en el world file; "
                "pre-warm /tf_static omitido."
            )
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
            f"[PRE-WARM] Publicado {self._world_frame}->{self._base_frame} en /tf_static "
            f"(pose estática del world file: x={tx:.3f} y={ty:.3f} z={tz:.3f}). "
            "Se actualizará con la pose dinámica de Gazebo cuando esté disponible."
        )
        self._static_used = True

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
        if name == f"{self._model_name}::{self._base_frame}":
            return 120
        if name == self._base_frame:
            return 110
        if name.endswith("::base_link"):
            return 100
        if name.endswith(f"::{self._base_frame}"):
            return 95
        # Do NOT match by bare suffix `endswith(base_frame)` without a "::" or "/"
        # separator: that would accidentally pick links like `rg2_base_link`
        # (the gripper's base) when base_frame=="base_link", because
        # "rg2_base_link".endswith("base_link") is True. Bug observed
        # 2026-04-27: pose/info had `rg2_base_link` with -90deg X rotation;
        # the publisher used it as world->base_link, producing unreachable
        # IK targets in panel_pick_demo (pos_err_m≈0.487).
        if "/" in name and name.split("/")[-1] == self._base_frame:
            return 85
        if self._model_name and name == self._model_name:
            # model-level pose accepted as fallback when link-level (::base_link) unavailable
            return 50
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
        # En LifecycleNode preferimos no hacer destroy_node aquí: deactivate
        # libera timer/subscription. El proceso queda vivo para diagnóstico
        # externo. Si se quiere terminar, hacer trigger_shutdown desde fuera.
        try:
            self.trigger_deactivate()
        except Exception:
            pass

    def _publish_timer(self) -> None:
        if self._fatal:
            return
        if self._wait_for_clock and not self._clock_ready:
            now_ns = self.get_clock().now().nanoseconds
            wall_now = time.monotonic()
            if wall_now > self._clock_deadline:
                if (wall_now - self._clock_last_log) > 2.0:
                    self.get_logger().warn(
                        "Clock no disponible todavia; continuando espera sin abortar."
                    )
                    self._clock_last_log = wall_now
            if now_ns <= 0:
                if (wall_now - self._clock_last_log) > 2.0:
                    self.get_logger().warn("Esperando /clock (tiempo=0); TF bloqueado.")
                    self._clock_last_log = wall_now
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
                if (now - self._last_warn) > 2.0:
                    self.get_logger().warn(
                        "Pose/info no disponible todavia; continuando espera sin abortar."
                    )
                    self._last_warn = now
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
    # F13: auto-activate por defecto preserva el comportamiento previo
    # (los launch existentes no transicionan el nodo). Si auto_activate=false,
    # queda en UNCONFIGURED y se controla externamente vía ros2 lifecycle.
    if bool(node.get_parameter("auto_activate").value):
        try:
            node.trigger_configure()
            node.trigger_activate()
        except Exception as exc:
            node.get_logger().error(f"[LIFECYCLE] auto_activate failed: {exc}")
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
