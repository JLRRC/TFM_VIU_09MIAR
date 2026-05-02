#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/tf_geometry_service.py
# Contenido: F16 (2026-05-01) — nodo ROS 2 que aloja servicios geométricos TF.
"""``tf_geometry_service`` — nodo ROS 2 con servicios TF geométricos.

F16 (2026-05-01) crea un microservicio dedicado que centraliza los
cálculos TF que actualmente están duplicados entre
``world_tf_publisher``, panel y ``panel_state_methods``. Aloja:

* ``/tf_geometry/world_to_base`` (``WorldToBase.srv``) — convierte
  un punto en frame ``world`` al frame ``base_link`` vía TF live.
* ``/tf_geometry/compute_approach_pose`` (``ComputeApproachPose.srv``)
  — devuelve la pose de approach con clearance vertical sobre un
  objeto dado en ``base_link``.

Es ``LifecycleNode`` con auto-activate por defecto (preserva
backward-compat con launches que no transicionan el nodo).

La lógica matemática vive en ``tf_geometry_logic.py`` (puro,
testeable offline). Este módulo solo ata el nodo ROS al backend
puro y al ``tf2_ros.Buffer`` para queries TF live.

Uso::

    ros2 run ur5_tools tf_geometry_service
    ros2 service call /tf_geometry/world_to_base \
      ur5_panel_interfaces/srv/WorldToBase \
      "{world_xyz: {x: 1.0, y: 0.0, z: 0.0}}"
"""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

try:
    from geometry_msgs.msg import Point, Pose, Quaternion
    from tf2_ros import Buffer, TransformListener
except Exception:  # pragma: no cover
    Point = None  # type: ignore
    Pose = None  # type: ignore
    Quaternion = None  # type: ignore
    Buffer = None  # type: ignore
    TransformListener = None  # type: ignore

try:
    from ur5_panel_interfaces.srv import (
        ComputeApproachPose,
        WorldToBase,
    )
except Exception:  # pragma: no cover
    ComputeApproachPose = None  # type: ignore
    WorldToBase = None  # type: ignore

from .tf_geometry_logic import (
    apply_world_to_base_transform,
    compute_approach_pose,
)


class TfGeometryService(LifecycleNode):
    """LifecycleNode que aloja los servicios geométricos TF.

    Asume:

    * ``WorldToBase.srv`` y ``ComputeApproachPose.srv`` disponibles
      en ``ur5_panel_interfaces``.
    * Un publisher TF activo (``world_tf_publisher``) que mantiene
      ``world → base_link`` actualizado.

    Recursos por estado:

    * ``on_configure``: declara parámetros, crea TF buffer/listener,
      reserva atributos para los servicios.
    * ``on_activate``: crea los dos services y los anuncia.
    * ``on_deactivate``: destruye los services (rechazan peticiones
      nuevas).
    * ``on_cleanup``: libera TF buffer/listener.
    """

    def __init__(self) -> None:
        super().__init__("tf_geometry_service")
        self.declare_parameter("auto_activate", True)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.2)

        self._world_frame = ""
        self._base_frame = ""
        self._tf_timeout = 0.2
        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener: Optional[TransformListener] = None
        self._world_to_base_srv = None
        self._approach_pose_srv = None

    # ------------------------------------------------------------------
    # Lifecycle transitions
    # ------------------------------------------------------------------

    def on_configure(self, _state) -> TransitionCallbackReturn:
        if WorldToBase is None or ComputeApproachPose is None:
            self.get_logger().error(
                "[TF_GEOM] ur5_panel_interfaces no disponible — "
                "el nodo no puede configurarse."
            )
            return TransitionCallbackReturn.FAILURE
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        try:
            self._tf_timeout = float(self.get_parameter("tf_timeout_sec").value)
        except Exception:
            self._tf_timeout = 0.2
        if self._tf_timeout <= 0.0:
            self._tf_timeout = 0.2
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.get_logger().info(
            f"[LIFECYCLE] tf_geometry_service configured "
            f"(world={self._world_frame}, base={self._base_frame}, "
            f"timeout={self._tf_timeout:.2f}s)"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state) -> TransitionCallbackReturn:
        self._world_to_base_srv = self.create_service(
            WorldToBase,
            "/tf_geometry/world_to_base",
            self._on_world_to_base,
        )
        self._approach_pose_srv = self.create_service(
            ComputeApproachPose,
            "/tf_geometry/compute_approach_pose",
            self._on_compute_approach_pose,
        )
        self.get_logger().info(
            "[LIFECYCLE] tf_geometry_service activated; services available: "
            "/tf_geometry/world_to_base, /tf_geometry/compute_approach_pose"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state) -> TransitionCallbackReturn:
        for attr in ("_world_to_base_srv", "_approach_pose_srv"):
            srv = getattr(self, attr, None)
            if srv is not None:
                try:
                    self.destroy_service(srv)
                except Exception:
                    pass
                setattr(self, attr, None)
        self.get_logger().info("[LIFECYCLE] tf_geometry_service deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state) -> TransitionCallbackReturn:
        for attr in ("_world_to_base_srv", "_approach_pose_srv"):
            srv = getattr(self, attr, None)
            if srv is not None:
                try:
                    self.destroy_service(srv)
                except Exception:
                    pass
                setattr(self, attr, None)
        self._tf_buffer = None
        self._tf_listener = None
        self.get_logger().info("[LIFECYCLE] tf_geometry_service cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state) -> TransitionCallbackReturn:
        return self.on_cleanup(_state)

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _on_world_to_base(self, request, response):
        if self._tf_buffer is None:
            response.success = False
            response.detail = "tf_buffer_not_initialized"
            return response
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._world_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._tf_timeout),
            )
        except Exception as exc:
            response.success = False
            response.detail = f"tf_lookup_failed:{type(exc).__name__}:{exc}"
            return response

        # tf is base_link from world's perspective. We want to apply the
        # inverse of (world->base_link) i.e. base_link as expressed in world,
        # but the lookup we asked for already provides base_link←world
        # composition. tf2 returns it as the transform that takes a point
        # expressed in world and gives it in base_link directly:
        # base_xyz = R * world_xyz + t.
        t = tf.transform.translation
        q = tf.transform.rotation
        # Aplicación directa con apply_world_to_base_transform requiere los
        # valores de la TF inversa (de world_to_base — el origen de
        # base_link en world). tf2 los entrega como transform from
        # `target_frame` (base_link) to `source_frame` (world), por lo
        # que ya es ese sentido. Reusamos el helper puro:
        try:
            from .tf_geometry_logic import rotate_vector_by_quat
            world_xyz = (
                float(request.world_xyz.x),
                float(request.world_xyz.y),
                float(request.world_xyz.z),
            )
            quat = (float(q.x), float(q.y), float(q.z), float(q.w))
            rotated = rotate_vector_by_quat(world_xyz, quat)
            base_x = rotated[0] + float(t.x)
            base_y = rotated[1] + float(t.y)
            base_z = rotated[2] + float(t.z)
        except Exception as exc:
            response.success = False
            response.detail = f"compute_failed:{type(exc).__name__}:{exc}"
            return response

        response.base_xyz = Point(x=base_x, y=base_y, z=base_z)
        response.success = True
        response.detail = "ok"
        return response

    def _on_compute_approach_pose(self, request, response):
        try:
            obj = request.object_pose_base
            pos = (float(obj.position.x), float(obj.position.y), float(obj.position.z))
            quat = (
                float(obj.orientation.x),
                float(obj.orientation.y),
                float(obj.orientation.z),
                float(obj.orientation.w),
            )
            clearance = float(request.z_clearance_m)
        except Exception as exc:
            response.success = False
            response.detail = f"invalid_request:{exc}"
            return response
        if clearance <= 0.0:
            response.success = False
            response.detail = "z_clearance_must_be_positive"
            return response
        approach_pos, approach_quat = compute_approach_pose(
            (pos, quat), clearance
        )
        response.approach_pose_base = Pose(
            position=Point(
                x=approach_pos[0], y=approach_pos[1], z=approach_pos[2]
            ),
            orientation=Quaternion(
                x=approach_quat[0],
                y=approach_quat[1],
                z=approach_quat[2],
                w=approach_quat[3],
            ),
        )
        response.success = True
        response.detail = "ok"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfGeometryService()
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
    node.destroy_node()
    try:
        rclpy.try_shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
