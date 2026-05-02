#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tf_geometry_client.py
# Contenido: F16-step3 (2026-05-02) — cliente Python del tf_geometry_service.
"""Cliente Python del ``tf_geometry_service`` (F16) para uso desde el panel.

F16-step3 cierra la transición arquitectónica: provee una clase
``TfGeometryClient`` que wrappea los services
``/tf_geometry/world_to_base`` y
``/tf_geometry/compute_approach_pose`` con timeouts, fallback graceful
y validación de inputs. El panel y ``panel_state_methods`` pueden
sustituir sus cálculos TF locales por llamadas a este cliente sin
duplicar la lógica matemática (que vive en
``ur5_tools/tf_geometry_logic.py``).

Diseño:

* La parte ROS (ServiceClient, await_call) vive en este módulo.
* La lógica matemática está en ``tf_geometry_logic`` (puro).
* Los timeouts y fallback están parametrizados.
* Si el servicio no está disponible (no rclpy / no spin / timeout),
  el cliente devuelve ``None`` con razón estructurada para que el
  caller decida si caer al cálculo local o abortar.

Uso desde el panel (cuando el usuario decida migrar consumidores)::

    from .panel_tf_geometry_client import TfGeometryClient

    client = TfGeometryClient(node)
    base_xyz, reason = client.world_to_base(
        world_xyz=(0.5, 0.0, 1.0), timeout_sec=0.2
    )
    if base_xyz is None:
        # Fallback al cálculo local + log
        ...
"""

from __future__ import annotations

from typing import Any, Optional, Tuple

try:
    import rclpy
    from rclpy.duration import Duration
except Exception:  # pragma: no cover
    rclpy = None  # type: ignore
    Duration = None  # type: ignore

try:
    from geometry_msgs.msg import Point, Pose, Quaternion
except Exception:  # pragma: no cover
    Point = None  # type: ignore
    Pose = None  # type: ignore
    Quaternion = None  # type: ignore

try:
    from ur5_panel_interfaces.srv import (
        ComputeApproachPose,
        WorldToBase,
    )
except Exception:  # pragma: no cover
    ComputeApproachPose = None  # type: ignore
    WorldToBase = None  # type: ignore


Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]
PoseTuple = Tuple[Vec3, Quat]


def is_available() -> bool:
    """True si el cliente puede instanciarse (rclpy + msgs disponibles)."""
    return all(
        x is not None
        for x in (rclpy, Point, Pose, Quaternion, WorldToBase, ComputeApproachPose)
    )


class TfGeometryClient:
    """Wrapper Python sobre los services del ``tf_geometry_service``.

    Args:
        node: rclpy.node.Node (o LifecycleNode) propietario del cliente.
            Necesita estar siendo spinneado para que las respuestas lleguen.
        world_to_base_srv_name: nombre del service (default canonical).
        approach_srv_name: nombre del service (default canonical).
    """

    DEFAULT_WORLD_TO_BASE = "/tf_geometry/world_to_base"
    DEFAULT_APPROACH_POSE = "/tf_geometry/compute_approach_pose"

    def __init__(
        self,
        node,
        *,
        world_to_base_srv_name: str = DEFAULT_WORLD_TO_BASE,
        approach_srv_name: str = DEFAULT_APPROACH_POSE,
    ) -> None:
        if not is_available():
            raise RuntimeError(
                "tf_geometry_client unavailable: rclpy or interfaces missing"
            )
        self._node = node
        self._world_to_base_srv_name = world_to_base_srv_name
        self._approach_srv_name = approach_srv_name
        self._world_to_base_client = node.create_client(
            WorldToBase, world_to_base_srv_name
        )
        self._approach_client = node.create_client(
            ComputeApproachPose, approach_srv_name
        )

    def wait_for_services(self, timeout_sec: float = 2.0) -> bool:
        """Espera a que ambos services estén anunciados.

        Devuelve True si los dos están listos antes del timeout.
        """
        try:
            ok1 = self._world_to_base_client.wait_for_service(timeout_sec=timeout_sec)
            ok2 = self._approach_client.wait_for_service(timeout_sec=timeout_sec)
            return bool(ok1 and ok2)
        except Exception:
            return False

    def world_to_base(
        self,
        world_xyz: Vec3,
        *,
        timeout_sec: float = 0.2,
    ) -> Tuple[Optional[Vec3], str]:
        """Llama al service /tf_geometry/world_to_base.

        Returns:
            (base_xyz, "ok") si éxito.
            (None, reason) si fallo (timeout, service unavailable, error
            en el server, etc).
        """
        try:
            req = WorldToBase.Request()
            req.world_xyz = Point(
                x=float(world_xyz[0]),
                y=float(world_xyz[1]),
                z=float(world_xyz[2]),
            )
        except Exception as exc:
            return None, f"invalid_request:{exc}"

        if not self._world_to_base_client.service_is_ready():
            if not self._world_to_base_client.wait_for_service(
                timeout_sec=timeout_sec
            ):
                return None, "service_unavailable"

        try:
            future = self._world_to_base_client.call_async(req)
        except Exception as exc:
            return None, f"call_failed:{type(exc).__name__}:{exc}"

        # spin_until_future_complete bloquea el ejecutor del node hasta
        # que la respuesta llega o expira el timeout.
        try:
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=float(timeout_sec)
            )
        except Exception as exc:
            return None, f"spin_failed:{type(exc).__name__}:{exc}"

        if not future.done():
            return None, "timeout"
        try:
            response = future.result()
        except Exception as exc:
            return None, f"result_failed:{type(exc).__name__}:{exc}"
        if response is None:
            return None, "no_response"
        if not bool(getattr(response, "success", False)):
            return None, str(getattr(response, "detail", "")) or "service_failed"

        try:
            return (
                (
                    float(response.base_xyz.x),
                    float(response.base_xyz.y),
                    float(response.base_xyz.z),
                ),
                "ok",
            )
        except Exception as exc:
            return None, f"unpack_failed:{exc}"

    def compute_approach_pose(
        self,
        object_pose_base: PoseTuple,
        z_clearance_m: float,
        *,
        timeout_sec: float = 0.2,
    ) -> Tuple[Optional[PoseTuple], str]:
        """Llama al service /tf_geometry/compute_approach_pose.

        Args:
            object_pose_base: ((x, y, z), (qx, qy, qz, qw)) en frame base_link.
            z_clearance_m: clearance vertical (m). Debe ser > 0 — el server
                rechaza valores no positivos.

        Returns:
            (approach_pose_tuple, "ok") si éxito.
            (None, reason) si fallo.
        """
        try:
            (px, py, pz), (qx, qy, qz, qw) = object_pose_base
            req = ComputeApproachPose.Request()
            req.object_pose_base = Pose(
                position=Point(x=float(px), y=float(py), z=float(pz)),
                orientation=Quaternion(
                    x=float(qx), y=float(qy), z=float(qz), w=float(qw)
                ),
            )
            req.z_clearance_m = float(z_clearance_m)
        except Exception as exc:
            return None, f"invalid_request:{exc}"

        if not self._approach_client.service_is_ready():
            if not self._approach_client.wait_for_service(timeout_sec=timeout_sec):
                return None, "service_unavailable"

        try:
            future = self._approach_client.call_async(req)
        except Exception as exc:
            return None, f"call_failed:{type(exc).__name__}:{exc}"

        try:
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=float(timeout_sec)
            )
        except Exception as exc:
            return None, f"spin_failed:{type(exc).__name__}:{exc}"

        if not future.done():
            return None, "timeout"
        try:
            response = future.result()
        except Exception as exc:
            return None, f"result_failed:{type(exc).__name__}:{exc}"
        if response is None:
            return None, "no_response"
        if not bool(getattr(response, "success", False)):
            return None, str(getattr(response, "detail", "")) or "service_failed"

        try:
            ap = response.approach_pose_base
            return (
                (
                    (float(ap.position.x), float(ap.position.y), float(ap.position.z)),
                    (
                        float(ap.orientation.x),
                        float(ap.orientation.y),
                        float(ap.orientation.z),
                        float(ap.orientation.w),
                    ),
                ),
                "ok",
            )
        except Exception as exc:
            return None, f"unpack_failed:{exc}"

    def destroy(self) -> None:
        """Libera los clientes ROS."""
        for attr in ("_world_to_base_client", "_approach_client"):
            client = getattr(self, attr, None)
            if client is not None:
                try:
                    self._node.destroy_client(client)
                except Exception:
                    pass
                setattr(self, attr, None)
