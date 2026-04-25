#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_ros_publishers.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""ROS publisher helpers for the panel."""
from __future__ import annotations

from typing import Dict, Optional, Tuple

try:
    from trajectory_msgs.msg import JointTrajectory
    from std_msgs.msg import Float64MultiArray, Empty
except Exception:  # pragma: no cover
    JointTrajectory = None  # type: ignore
    Float64MultiArray = None  # type: ignore
    Empty = None  # type: ignore


def get_traj_publisher(
    node: Optional[object],
    pub: Optional[object],
    current_topic: str,
    topic: str,
    log_fn,
) -> Tuple[Optional[object], str]:
    if node is None or JointTrajectory is None:
        return pub, current_topic
    if pub is None or current_topic != topic:
        try:
            pub = node.create_publisher(JointTrajectory, topic, 10)
            current_topic = topic
        except Exception as exc:
            log_fn(f"[Panel] ERROR creando publisher JointTrajectory: {exc}")
            pub = None
    return pub, current_topic


def get_gripper_publisher(
    node: Optional[object],
    pub: Optional[object],
    current_topic: str,
    topic: str,
    log_fn,
) -> Tuple[Optional[object], str]:
    if node is None or Float64MultiArray is None:
        return pub, current_topic
    if pub is None or current_topic != topic:
        try:
            pub = node.create_publisher(Float64MultiArray, topic, 10)
            current_topic = topic
        except Exception as exc:
            log_fn(f"[Panel] ERROR creando publisher gripper: {exc}")
            pub = None
    return pub, current_topic


def get_attach_publisher(
    node: Optional[object],
    pubs: Dict[str, object],
    topic: str,
    log_fn,
) -> Optional[object]:
    if node is None or Empty is None:
        return None
    pub = pubs.get(topic)
    if pub is not None:
        return pub
    try:
        pub = node.create_publisher(Empty, topic, 10)
        pubs[topic] = pub
        return pub
    except Exception as exc:
        log_fn(f"[Panel] ERROR creando publisher attach: {exc}")
        return None
