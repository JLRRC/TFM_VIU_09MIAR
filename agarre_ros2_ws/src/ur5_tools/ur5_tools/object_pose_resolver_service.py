#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/object_pose_resolver_service.py
# Contenido: F5-step4 — LifecycleNode con server ResolveObjectPoseWorld.
"""Server LifecycleNode para ``ResolveObjectPoseWorld`` (F5-step4).

Suscribe al topic donde ``gz_pose_bridge`` publica las poses Gazebo
(TFMessage con un TransformStamped por objeto) y mantiene un cache
indexado por nombre. Cuando el orchestrator (o cualquier cliente)
invoca el service, devuelve la pose si está fresca; si no, success=False
con detalle.

Diseño:

* LifecycleNode (compatible con resto del stack: configure → activate
  → ... → cleanup → shutdown).
* on_configure declara params (world_name, max_pose_age_sec, gz_topic,
  service_name, world_frame).
* on_activate crea sub al TFMessage topic + service server.
* on_deactivate destruye ambos.
* on_cleanup libera params y cache.
* La lógica pura (cache + lookup + conversores) vive en
  ``object_pose_cache.py`` — testeable sin ROS.

Uso:

    ros2 run ur5_tools object_pose_resolver_service
    ros2 lifecycle set /object_pose_resolver_service configure
    ros2 lifecycle set /object_pose_resolver_service activate
    ros2 service call /orchestrator/resolve_object_pose_world \\
        ur5_panel_interfaces/srv/ResolveObjectPoseWorld \\
        "{object_name: 'box_red'}"
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from tf2_msgs.msg import TFMessage

from ur5_panel_interfaces.srv import ResolveObjectPoseWorld

from .object_pose_cache import ObjectPoseCache, tfmessage_to_updates


_DEFAULT_SERVICE_NAME = "/orchestrator/resolve_object_pose_world"
_DEFAULT_WORLD_NAME = "test_world"
_DEFAULT_WORLD_FRAME = "world"
_DEFAULT_MAX_POSE_AGE_SEC = 1.5


class ObjectPoseResolverService(LifecycleNode):
    """Server `/orchestrator/resolve_object_pose_world` con ciclo de vida managed."""

    def __init__(self) -> None:
        super().__init__("object_pose_resolver_service")
        self._cb_group = MutuallyExclusiveCallbackGroup()
        self._cache = ObjectPoseCache()
        self._sub = None
        self._service = None
        self._service_name = _DEFAULT_SERVICE_NAME
        self._world_name = _DEFAULT_WORLD_NAME
        self._world_frame = _DEFAULT_WORLD_FRAME
        self._max_pose_age_sec = _DEFAULT_MAX_POSE_AGE_SEC
        self._gz_topic = ""
        self.get_logger().info(
            "[OBJ_POSE_RESOLVER] instantiated (UNCONFIGURED)"
        )

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        try:
            self.declare_parameter("world_name", _DEFAULT_WORLD_NAME)
            self.declare_parameter("world_frame", _DEFAULT_WORLD_FRAME)
            self.declare_parameter("max_pose_age_sec", _DEFAULT_MAX_POSE_AGE_SEC)
            self.declare_parameter("gz_pose_topic", "")
            self.declare_parameter("service_name", _DEFAULT_SERVICE_NAME)
        except Exception:
            # Re-configure: ya declarados.
            pass
        self._world_name = str(self.get_parameter("world_name").value)
        self._world_frame = str(self.get_parameter("world_frame").value or _DEFAULT_WORLD_FRAME)
        try:
            self._max_pose_age_sec = float(
                self.get_parameter("max_pose_age_sec").value
            )
        except Exception:
            self._max_pose_age_sec = _DEFAULT_MAX_POSE_AGE_SEC
        gz_topic_param = str(self.get_parameter("gz_pose_topic").value or "").strip()
        self._gz_topic = (
            gz_topic_param
            or f"/world/{self._world_name}/pose/info"
        )
        self._service_name = str(
            self.get_parameter("service_name").value or _DEFAULT_SERVICE_NAME
        )
        self._cache.clear()
        self.get_logger().info(
            f"[OBJ_POSE_RESOLVER] on_configure OK "
            f"world={self._world_name} gz_topic={self._gz_topic} "
            f"service={self._service_name} max_age={self._max_pose_age_sec:.2f}s"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state: State) -> TransitionCallbackReturn:
        self._sub = self.create_subscription(
            TFMessage,
            self._gz_topic,
            self._on_tfmessage,
            10,
            callback_group=self._cb_group,
        )
        self._service = self.create_service(
            ResolveObjectPoseWorld,
            self._service_name,
            self._on_service_request,
            callback_group=self._cb_group,
        )
        self.get_logger().info(
            f"[OBJ_POSE_RESOLVER] on_activate OK — sub={self._gz_topic}, "
            f"srv={self._service_name}"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state: State) -> TransitionCallbackReturn:
        if self._sub is not None:
            try:
                self.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None
        if self._service is not None:
            try:
                self.destroy_service(self._service)
            except Exception:
                pass
            self._service = None
        self.get_logger().info("[OBJ_POSE_RESOLVER] on_deactivate OK")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        if self._sub is not None:
            try:
                self.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None
        if self._service is not None:
            try:
                self.destroy_service(self._service)
            except Exception:
                pass
            self._service = None
        self._cache.clear()
        self.get_logger().info("[OBJ_POSE_RESOLVER] on_cleanup OK")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        self._cache.clear()
        self.get_logger().info("[OBJ_POSE_RESOLVER] on_shutdown OK")
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Sub callback + Service callback
    # ------------------------------------------------------------------

    def _on_tfmessage(self, msg: TFMessage) -> None:
        now_sec = time.monotonic()
        for name, pose7, stamp_sec, frame_id in tfmessage_to_updates(msg, now_sec):
            self._cache.update(name, pose7, stamp_sec, frame_id)

    def _on_service_request(
        self,
        request: ResolveObjectPoseWorld.Request,
        response: ResolveObjectPoseWorld.Response,
    ) -> ResolveObjectPoseWorld.Response:
        name = str(request.object_name or "").strip()
        if not name:
            response.success = False
            response.detail = "object_name_empty"
            return response
        cached, reason = self._cache.lookup(
            name,
            now_sec=time.monotonic(),
            max_age_sec=self._max_pose_age_sec,
        )
        if cached is None:
            response.success = False
            response.detail = (
                reason or "lookup_failed"
            ) + f" known={','.join(self._cache.known_objects()[:10]) or 'none'}"
            return response
        x, y, z, qx, qy, qz, qw = cached.pose7
        response.pose_world = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        )
        response.success = True
        response.detail = (
            f"frame_id={cached.frame_id} age={time.monotonic() - cached.stamp_sec:.3f}s"
        )
        return response


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ObjectPoseResolverService()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
