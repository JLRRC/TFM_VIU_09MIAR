#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_moveit_publishers.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""MoveIt publisher helpers for the panel (ROS node + pose publishers)."""
from __future__ import annotations

from typing import Optional, Tuple

from .panel_config import ROS_AVAILABLE

try:
    import rclpy
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import PoseStamped
except Exception:  # pragma: no cover
    rclpy = None  # type: ignore
    Parameter = None  # type: ignore
    PoseStamped = None  # type: ignore


def init_moveit_publishers(
    *,
    node: Optional[object],
    pose_pub: Optional[object],
    pose_pub_cartesian: Optional[object],
    use_sim_time: bool,
    pose_topic: str,
    cartesian_topic: str,
    logger,
) -> Tuple[Optional[object], Optional[object], Optional[object]]:
    """Ensure MoveIt pose publishers exist; return updated (node, pose_pub, pose_pub_cartesian)."""
    if not ROS_AVAILABLE:
        logger("[Panel] ROS no disponible; MoveIt publisher deshabilitado.")
        return node, pose_pub, pose_pub_cartesian
    if PoseStamped is None:
        logger("[Panel] ROS no disponible; MoveIt publisher deshabilitado.")
        return node, pose_pub, pose_pub_cartesian
    try:
        if rclpy is not None and not rclpy.ok():
            rclpy.init(args=None)
    except Exception as exc:
        logger(f"[Panel] Advertencia: rclpy.init falló ({exc}), se intentará de nuevo.")
    try:
        if node is None and rclpy is not None:
            overrides = None
            if Parameter is not None:
                overrides = [Parameter("use_sim_time", Parameter.Type.BOOL, bool(use_sim_time))]
            node_kwargs = {}
            if overrides:
                node_kwargs["parameter_overrides"] = overrides
            node = rclpy.create_node("panel_v2_moveit_publisher", **node_kwargs)
        if pose_pub is None and node is not None:
            pose_pub = node.create_publisher(PoseStamped, pose_topic, 10)
        if pose_pub_cartesian is None and node is not None:
            pose_pub_cartesian = node.create_publisher(PoseStamped, cartesian_topic, 10)
        if pose_pub:
            logger(f"[Panel] MoveIt publisher listo en {pose_topic}")
        if pose_pub_cartesian:
            logger(f"[Panel] MoveIt publisher listo en {cartesian_topic}")
    except Exception as exc:
        logger(f"[Panel] ERROR creando publisher MoveIt: {exc}")
        pose_pub = None
        pose_pub_cartesian = None
    return node, pose_pub, pose_pub_cartesian
