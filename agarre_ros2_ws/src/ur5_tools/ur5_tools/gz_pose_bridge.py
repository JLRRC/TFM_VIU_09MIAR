#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/gz_pose_bridge.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
"""Bridge Gazebo /world/<world>/pose/info to ROS TFMessage with names."""

from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from typing import Dict, List, Optional

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from tf2_msgs.msg import TFMessage


class GzPoseBridge(LifecycleNode):
    """Read gz pose/info and publish a TFMessage with named frames.

    F13c (2026-05-02): segregación de recursos a Pleno.

    * ``__init__`` solo declara parámetros y reserva atributos.
    * ``on_configure``: lee parámetros y crea el publisher.
    * ``on_activate``: arranca el hilo gz topic + watchdog timer.
    * ``on_deactivate``: detiene hilo y timer (deja publisher creado
      para reuso rápido sin reservar topic).
    * ``on_cleanup``: destruye publisher y reset.
    """

    def __init__(self) -> None:
        super().__init__("gz_pose_bridge")
        if not self.has_parameter("auto_activate"):
            self.declare_parameter("auto_activate", True)
        self.declare_parameter("world_name", "ur5_mesa_objetos")
        self.declare_parameter("gz_topic", "")
        self.declare_parameter("ros_topic", "")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("startup_timeout_sec", 5.0)

        # Recursos creados en lifecycle, no en __init__.
        self._world_name = ""
        self._gz_topic = ""
        self._ros_topic = ""
        self._world_frame = "world"
        self._startup_timeout = 5.0
        self._pub = None
        self._stop = threading.Event()
        self._last_valid = 0.0
        self._start_wall = 0.0
        self._proc = None
        self._proc_lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._timer = None

    def _watchdog(self) -> None:
        if self._last_valid > 0.0:
            return
        if (time.monotonic() - self._start_wall) >= self._startup_timeout:
            self._abort("No pose/info with names received; aborting.")

    def _reader_loop(self) -> None:
        cmd = [
            "gz",
            "topic",
            "-e",
            "-t",
            self._gz_topic,
            "--json-output",
        ]
        env = os.environ.copy()
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=env,
            )
            with self._proc_lock:
                self._proc = proc
        except Exception as exc:
            self._abort(f"Failed to start gz topic: {exc}")
            return

        try:
            assert proc.stdout is not None
            for line in proc.stdout:
                if self._stop.is_set():
                    break
                line = line.strip()
                if not line:
                    continue
                msg = self._parse_json(line)
                if msg is None:
                    continue
                tf_msg = self._to_tf(msg)
                if tf_msg is None:
                    continue
                self._pub.publish(tf_msg)
                self._last_valid = time.monotonic()
        finally:
            self._terminate_proc()
            if not self._stop.is_set() and self._last_valid <= 0.0:
                self._abort("gz topic exited before receiving pose/info.")

    def _abort(self, reason: str) -> None:
        if self._stop.is_set():
            return
        self.get_logger().error(reason)
        self._stop.set()
        self._terminate_proc()
        # F13c: en LifecycleNode delegamos en deactivate; el supervisor
        # decide shutdown.
        try:
            self.trigger_deactivate()
        except Exception:
            pass

    def _terminate_proc(self) -> None:
        with self._proc_lock:
            proc = self._proc
            self._proc = None
        if proc is None:
            return
        try:
            proc.terminate()
        except Exception:
            pass
        try:
            proc.wait(timeout=1.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

    @staticmethod
    def _parse_json(line: str) -> Optional[Dict[str, object]]:
        try:
            return json.loads(line)
        except json.JSONDecodeError:
            return None

    def _to_tf(self, msg: Dict[str, object]) -> Optional[TFMessage]:
        poses = msg.get("pose")
        if not isinstance(poses, list):
            return None
        stamp = self._extract_stamp(msg)
        transforms: List[TransformStamped] = []
        for pose in poses:
            if not isinstance(pose, dict):
                continue
            name = pose.get("name")
            if not isinstance(name, str) or not name:
                continue
            pos = pose.get("position") or {}
            ori = pose.get("orientation") or {}
            if not isinstance(pos, dict) or not isinstance(ori, dict):
                continue
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self._world_frame
            tf.child_frame_id = name
            tf.transform.translation.x = float(pos.get("x", 0.0))
            tf.transform.translation.y = float(pos.get("y", 0.0))
            tf.transform.translation.z = float(pos.get("z", 0.0))
            tf.transform.rotation.x = float(ori.get("x", 0.0))
            tf.transform.rotation.y = float(ori.get("y", 0.0))
            tf.transform.rotation.z = float(ori.get("z", 0.0))
            tf.transform.rotation.w = float(ori.get("w", 1.0))
            transforms.append(tf)
        if not transforms:
            return None
        return TFMessage(transforms=transforms)

    def _extract_stamp(self, msg: Dict[str, object]):
        header = msg.get("header") or {}
        if isinstance(header, dict):
            stamp = header.get("stamp") or {}
            if isinstance(stamp, dict):
                sec = int(stamp.get("sec", 0))
                nsec = int(stamp.get("nsec", 0))
                if sec or nsec:
                    return rclpy.time.Time(seconds=sec, nanoseconds=nsec).to_msg()
        return self.get_clock().now().to_msg()


    # ------------------------------------------------------------------
    # Lifecycle transitions (F13c — Pleno, segregación real de recursos)
    # ------------------------------------------------------------------

    def on_configure(self, _state) -> TransitionCallbackReturn:
        self._world_name = str(self.get_parameter("world_name").value)
        self._gz_topic = (
            str(self.get_parameter("gz_topic").value)
            or f"/world/{self._world_name}/pose/info"
        )
        self._ros_topic = (
            str(self.get_parameter("ros_topic").value)
            or f"/world/{self._world_name}/pose/info"
        )
        self._world_frame = (
            str(self.get_parameter("world_frame").value) or "world"
        )
        try:
            self._startup_timeout = float(
                self.get_parameter("startup_timeout_sec").value
            )
        except Exception:
            self._startup_timeout = 5.0
        if self._startup_timeout <= 0.0:
            self._startup_timeout = 5.0

        self._pub = self.create_publisher(TFMessage, self._ros_topic, 10)
        self.get_logger().info(
            f"[LIFECYCLE] GzPoseBridge configured "
            f"(gz={self._gz_topic} -> ros={self._ros_topic})"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state) -> TransitionCallbackReturn:
        self._stop.clear()
        self._last_valid = 0.0
        self._start_wall = time.monotonic()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        self._timer = self.create_timer(0.5, self._watchdog)
        self.get_logger().info(
            "[LIFECYCLE] GzPoseBridge activated; reader thread + watchdog up"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state) -> TransitionCallbackReturn:
        self._stop.set()
        self._terminate_proc()
        if self._timer is not None:
            try:
                self._timer.cancel()
                self.destroy_timer(self._timer)
            except Exception:
                pass
            self._timer = None
        if self._thread is not None:
            try:
                self._thread.join(timeout=2.0)
            except Exception:
                pass
            self._thread = None
        self.get_logger().info("[LIFECYCLE] GzPoseBridge deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state) -> TransitionCallbackReturn:
        # Por si on_deactivate no se ejecutó.
        self._stop.set()
        self._terminate_proc()
        if self._timer is not None:
            try:
                self.destroy_timer(self._timer)
            except Exception:
                pass
            self._timer = None
        if self._pub is not None:
            try:
                self.destroy_publisher(self._pub)
            except Exception:
                pass
            self._pub = None
        self._thread = None
        self.get_logger().info("[LIFECYCLE] GzPoseBridge cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state) -> TransitionCallbackReturn:
        return self.on_cleanup(_state)


def main() -> None:
    rclpy.init()
    node = GzPoseBridge()
    # F13b lifecycle: auto-activate por defecto preserva backward-compat.
    if bool(node.get_parameter("auto_activate").value):
        try:
            node.trigger_configure()
            node.trigger_activate()
        except Exception as exc:
            node.get_logger().error(f"[LIFECYCLE] auto_activate failed: {exc}")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
