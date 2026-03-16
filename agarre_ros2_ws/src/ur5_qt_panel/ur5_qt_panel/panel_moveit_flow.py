#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_moveit_flow.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""MoveIt publish flow helpers for the panel."""
from __future__ import annotations

import time
from typing import Callable, Dict, Optional, Tuple

try:
    from rclpy.time import Time
except Exception:  # pragma: no cover
    Time = None  # type: ignore


def _block_moveit(
    *,
    emit_log: Callable[[str], None],
    set_status: Callable[[str, bool], None],
    log_msg: str,
    status_msg: str,
    error: bool,
) -> None:
    emit_log(log_msg)
    set_status(status_msg, error)


def publish_moveit_pose(
    *,
    label: str,
    pose_data: Dict[str, object],
    cartesian: bool,
    calibrating: bool,
    calib_grid_until: float,
    moveit_required: bool,
    managed_mode: bool,
    state_ready_moveit: bool,
    system_state_reason: str,
    system_state_value: str,
    moveit_ready: bool,
    moveit_state_reason: str,
    moveit_block_reason: Optional[str],
    moveit_node: Optional[object],
    moveit_pose_pub: Optional[object],
    moveit_pose_pub_cartesian: Optional[object],
    pose_topic: str,
    cartesian_topic: str,
    build_pose_stamped: Callable[[Dict[str, object]], object],
    emit_log: Callable[[str], None],
    set_status: Callable[[str, bool], None],
    log_warning: Callable[[str], None],
    logger_info: Callable[[str], None],
    log_exception: Callable[[str, Exception], None],
) -> Tuple[Optional[str], bool]:
    """Publish a MoveIt pose with the same gating/behavior as the panel."""
    if calibrating or (time.time() <= calib_grid_until):
        _block_moveit(
            emit_log=emit_log,
            set_status=set_status,
            log_msg=f"[MOVEIT] Bloqueado: {label} (calibración activa)",
            status_msg="Calibración activa: MoveIt bloqueado",
            error=False,
        )
        return moveit_block_reason, False
    if not moveit_required:
        _block_moveit(
            emit_log=emit_log,
            set_status=set_status,
            log_msg=f"[MOVEIT] Bloqueado: {label} (MoveIt deshabilitado)",
            status_msg=f"MoveIt deshabilitado; bloqueando {label}",
            error=True,
        )
        return moveit_block_reason, False
    if managed_mode and not state_ready_moveit:
        reason = system_state_reason or system_state_value
        _block_moveit(
            emit_log=emit_log,
            set_status=set_status,
            log_msg=f"[MOVEIT] Bloqueado: {label} (state={system_state_value})",
            status_msg=f"Sistema no listo; esperando {label} ({reason})",
            error=False,
        )
        return moveit_block_reason, False
    if not moveit_ready:
        reason = moveit_state_reason or "MoveIt no listo"
        if reason != moveit_block_reason:
            _block_moveit(
                emit_log=emit_log,
                set_status=set_status,
                log_msg=f"[MOVEIT] Bloqueado: {label} ({reason})",
                status_msg=f"MoveIt no listo; esperando {label}",
                error=False,
            )
            moveit_block_reason = reason
        return moveit_block_reason, False
    moveit_block_reason = None
    if moveit_node is None:
        log_warning(f"[Panel] LEGACY: MoveIt publisher no inicializado - no se envió pose {label}.")
        return moveit_block_reason, False
    pub = moveit_pose_pub_cartesian if cartesian else moveit_pose_pub
    if pub is None:
        log_warning(f"[Panel] MoveIt publisher no inicializado - no se envió pose {label}.")
        return moveit_block_reason, False
    pose = build_pose_stamped(pose_data)
    has_stamp = False
    try:
        has_stamp = bool(
            int(getattr(pose.header.stamp, "sec", 0)) != 0
            or int(getattr(pose.header.stamp, "nanosec", 0)) != 0
        )
    except Exception:
        has_stamp = False
    if Time is not None and not has_stamp:
        try:
            if moveit_node is not None and hasattr(moveit_node, "get_clock"):
                pose.header.stamp = moveit_node.get_clock().now().to_msg()
            else:
                pose.header.stamp = Time().to_msg()
        except Exception as exc:
            log_exception("build MoveIt pose stamp", exc)
            pose.header.stamp.sec = 0
            pose.header.stamp.nanosec = 0
    pub.publish(pose)
    topic = cartesian_topic if cartesian else pose_topic
    logger_info(f"[Panel] Sent {label} pose to MoveIt on {topic}; MoveIt is responsible for the motion.")
    return moveit_block_reason, True
