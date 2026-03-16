#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/tf_pose_utils.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Unified TF pose helpers for base_link business logic."""

from __future__ import annotations

import math
import time
from typing import Callable, Dict, Optional, Tuple

from geometry_msgs.msg import PoseStamped

from .panel_tf import get_tf_helper


LoggerFn = Optional[Callable[[str], None]]

# FASE 1: Throttle para evitar spam de logs TF repetitivos.
_TF_LOG_THROTTLE: Dict[str, float] = {}
_TF_LOG_THROTTLE_SEC = 5.0


def _log(logger: LoggerFn, message: str) -> None:
    if logger is None:
        return
    try:
        logger(message)
    except Exception:
        pass


def _log_throttled(logger: LoggerFn, key: str, message: str) -> None:
    """Log message at most once per _TF_LOG_THROTTLE_SEC for a given key."""
    if logger is None:
        return
    now = time.monotonic()
    last = _TF_LOG_THROTTLE.get(key, 0.0)
    if (now - last) < _TF_LOG_THROTTLE_SEC:
        return
    _TF_LOG_THROTTLE[key] = now
    try:
        logger(message)
    except Exception:
        pass


def _quat_to_rpy_deg(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def get_transform(
    parent: str,
    child: str,
    timeout: float = 0.2,
    *,
    logger: LoggerFn = None,
):
    helper = get_tf_helper()
    if helper is None:
        reason = f"TF_UNAVAILABLE helper_none parent={parent} child={child}"
        _log(logger, f"[TF_POSE] {reason}")
        return None, reason
    tf = helper.lookup_transform(str(parent), str(child), timeout_sec=float(max(0.01, timeout)))
    if tf is None:
        reason = f"TF_LOOKUP_FAIL parent={parent} child={child} timeout={timeout:.2f}s"
        _log_throttled(logger, f"get_transform:{parent}:{child}", f"[TF_POSE] {reason}")
        return None, reason
    return tf, "ok"


def transform_pose(
    pose_stamped: PoseStamped,
    target_frame: str,
    timeout: float = 0.2,
    *,
    logger: LoggerFn = None,
):
    helper = get_tf_helper()
    if helper is None:
        reason = f"TF_UNAVAILABLE transform target={target_frame}"
        _log(logger, f"[TF_POSE] {reason}")
        return None, reason
    out = helper.transform_pose(pose_stamped, str(target_frame), timeout_sec=float(max(0.01, timeout)))
    if out is None:
        src = str(getattr(getattr(pose_stamped, "header", None), "frame_id", "") or "")
        reason = f"POSE_TRANSFORM_FAIL {src}->{target_frame} timeout={timeout:.2f}s"
        # FASE 1: Throttle para no generar spam en cada ciclo de trace.
        _log_throttled(logger, f"transform_pose:{src}:{target_frame}", f"[TF_POSE] {reason}")
        return None, reason
    return out, "ok"


def get_tcp_in_base(
    base_frame: str = "base_link",
    ee_frame: str = "rg2_tcp",
    timeout: float = 0.2,
    *,
    logger: LoggerFn = None,
):
    tf, reason = get_transform(base_frame, ee_frame, timeout=timeout, logger=logger)
    if tf is None:
        return None, None, reason
    out = PoseStamped()
    out.header.frame_id = str(tf.header.frame_id or base_frame)
    out.header.stamp = tf.header.stamp
    out.pose.position.x = float(tf.transform.translation.x)
    out.pose.position.y = float(tf.transform.translation.y)
    out.pose.position.z = float(tf.transform.translation.z)
    out.pose.orientation.x = float(tf.transform.rotation.x)
    out.pose.orientation.y = float(tf.transform.rotation.y)
    out.pose.orientation.z = float(tf.transform.rotation.z)
    out.pose.orientation.w = float(tf.transform.rotation.w)
    rpy_deg = _quat_to_rpy_deg(
        out.pose.orientation.x,
        out.pose.orientation.y,
        out.pose.orientation.z,
        out.pose.orientation.w,
    )
    return out, rpy_deg, "ok"


def world_pose_to_base(
    p_world_pose,
    world_frame: str = "world",
    base_frame: str = "base_link",
    timeout: float = 0.2,
    *,
    logger: LoggerFn = None,
):
    if isinstance(p_world_pose, PoseStamped):
        pose = p_world_pose
        if not str(pose.header.frame_id or "").strip():
            pose.header.frame_id = str(world_frame)
    else:
        pose = PoseStamped()
        pose.header.frame_id = str(world_frame)
        try:
            if isinstance(p_world_pose, dict):
                pos = p_world_pose.get("position", p_world_pose)
                x = float(pos.get("x", 0.0))
                y = float(pos.get("y", 0.0))
                z = float(pos.get("z", 0.0))
            else:
                x = float(p_world_pose[0])
                y = float(p_world_pose[1])
                z = float(p_world_pose[2])
        except Exception:
            reason = "WORLD_POSE_INVALID"
            _log(logger, f"[TF_POSE] {reason}")
            return None, reason
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

    out, reason = transform_pose(pose, target_frame=base_frame, timeout=timeout, logger=logger)
    if out is None:
        return None, reason
    out.header.frame_id = str(base_frame)
    return out, "ok"
