#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_robot_presets.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Robot pose presets and helpers for the UR5 panel."""
from __future__ import annotations

import math
from typing import Dict, Tuple

from geometry_msgs.msg import PoseStamped


def _make_pose_data(
    position: Tuple[float, float, float],
    orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    frame: str = "base_link",
) -> Dict[str, object]:
    return {"position": position, "orientation": orientation, "frame": frame}


def _build_pose_stamped(data: Dict[str, object]) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = data.get("frame", "base_link")
    stamp_ns = int(data.get("stamp_ns", 0) or 0)
    if stamp_ns > 0:
        pose.header.stamp.sec = stamp_ns // 1_000_000_000
        pose.header.stamp.nanosec = stamp_ns % 1_000_000_000
    position = data.get("position", (0.0, 0.0, 0.0))
    orientation = data.get("orientation", (0.0, 0.0, 0.0, 1.0))
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
    (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w,
    ) = orientation
    return pose


POSE_HOME_DATA = _make_pose_data((0.18, 0.0, 0.35))
POSE_TABLE_DATA = _make_pose_data((0.30, -0.35, 0.28))
POSE_BASKET_DATA = _make_pose_data((0.45, 0.28, 0.32))
JOINT_TABLE_POSE_RAD = [
    math.radians(-2.7),
    math.radians(-0.1),
    math.radians(90.0),
    math.radians(-5.5),
    math.radians(-90.7),
    math.radians(0.9),
]
JOINT_BASKET_POSE_RAD = [
    math.radians(167.6),
    math.radians(-4.8),
    math.radians(110.9),
    math.radians(-21.1),
    math.radians(-92.0),
    math.radians(0.9),
]
JOINT_BASKET_DEMO_RELEASE_POSE_RAD = [
    math.radians(167.6),
    math.radians(-11.9071),
    math.radians(116.8468),
    math.radians(-19.9396),
    math.radians(-92.0),
    math.radians(0.9),
]
JOINT_HOME_POSE_RAD = [
    math.radians(0.0),
    math.radians(0.0),
    math.radians(0.0),
    math.radians(0.0),
    math.radians(0.0),
    math.radians(0.0),
]
JOINT_PICK_IMAGE_POSE_RAD = [
    math.radians(-15.7),
    math.radians(-1.3),
    math.radians(110.9),
    math.radians(-17.5),
    math.radians(-89.3),
    math.radians(0.9),
]
JOINT_GRASP_DOWN_POSE_RAD = [
    math.radians(-16.6),
    math.radians(0.5),
    math.radians(110.0),
    math.radians(-17.5),
    math.radians(-88.8),
    math.radians(0.9),
]
PRE_GRASP_POSE_DATA = _make_pose_data((0.28, -0.10, 0.35))
GRASP_POSE_DATA = _make_pose_data((0.28, -0.10, 0.20))
TRANSPORT_POSE_DATA = _make_pose_data((0.48, 0.10, 0.40))
DROP_POSE_DATA = _make_pose_data((0.48, 0.20, 0.33))
PICK_DEMO_OBJECT_NAME = "pick_demo"
PICK_SEQUENCE = [
    ("PRE_GRASP", PRE_GRASP_POSE_DATA, 0.6),
    ("GRASP", GRASP_POSE_DATA, 0.8),
    ("TRANSPORT", TRANSPORT_POSE_DATA, 0.6),
    ("DROP", DROP_POSE_DATA, 0.8),
]
