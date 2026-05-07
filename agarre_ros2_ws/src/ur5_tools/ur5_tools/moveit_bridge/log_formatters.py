#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/moveit_bridge/log_formatters.py
# Contenido: F3 — formatters puros de logs del UR5MoveItBridge.
"""Builders puros de strings de log para UR5MoveItBridge.

Extraídos de ``_pose_callback`` y ``_plan_worker``. Cada función
recibe los inputs como argumentos y devuelve el string final, sin
imprimir ni acceder a self.

Diseño:
* Cero dependencia ROS / time / random.
* Compatible con cualquier dict-like para `pos`/`stamp` (acepta
  geometry_msgs/Pose o tuples/dicts equivalentes).
* Tests offline garantizan estabilidad del formato exacto.
"""

from __future__ import annotations

from typing import Optional, Tuple

Vec3 = Tuple[float, float, float]


def _pos_xyz(pose) -> Vec3:
    """Acepta Pose ROS, dict {x,y,z} o tuple3."""
    if pose is None:
        return (0.0, 0.0, 0.0)
    if isinstance(pose, dict):
        return (
            float(pose.get("x", 0.0)),
            float(pose.get("y", 0.0)),
            float(pose.get("z", 0.0)),
        )
    if isinstance(pose, (tuple, list)):
        return (float(pose[0]), float(pose[1]), float(pose[2]))
    return (float(pose.x), float(pose.y), float(pose.z))


def format_pose_xyz(pose) -> str:
    """Format pose como ``(x.xxx,y.yyy,z.zzz)`` con 3 decimales."""
    x, y, z = _pos_xyz(pose)
    return f"({x:.3f},{y:.3f},{z:.3f})"


def format_rx_log(
    *,
    ts_us: int,
    request_id: int,
    request_uuid: str,
    frame_id: str,
    pose,
    accepted: bool,
    reason: Optional[str] = None,
) -> str:
    """Construye log ``[MOVEIT_BRIDGE][RX]``.

    Si ``accepted=False`` y ``reason`` es no vacío, se incluye
    ``reason=<reason>``.
    """
    base = (
        "[MOVEIT_BRIDGE][RX] "
        f"ts_us={int(ts_us)} req_id={int(request_id)} "
        f"req_uuid={request_uuid or 'n/a'} "
        f"frame={frame_id or 'n/a'} "
        f"pose={format_pose_xyz(pose)} "
        f"accepted={'true' if accepted else 'false'}"
    )
    if not accepted and reason:
        return f"{base} reason={reason}"
    return base


def format_recv_log(
    *,
    label: str,  # "POSE" | "CARTESIAN"
    topic_name: str,
    request_id: int,
    frame_id: str,
    pose,
    stamp_sec: int,
    stamp_nanosec: int,
    frame_raw: str,
    request_uuid: str,
    ee_target_tol_m: Optional[float],
    phase_label: Optional[str],
) -> str:
    """Construye log ``[BRIDGE][RECV]``."""
    tol_str = (
        str(float(ee_target_tol_m))
        if ee_target_tol_m is not None
        else "n/a"
    )
    return (
        "[BRIDGE][RECV] "
        f"label={label} topic={topic_name or 'n/a'} request_id={int(request_id)} "
        f"frame={frame_id} pos={format_pose_xyz(pose)} "
        f"stamp={int(stamp_sec)}.{int(stamp_nanosec):09d} "
        f"frame_raw={frame_raw or 'n/a'} request_uuid={request_uuid or 'n/a'} "
        f"ee_target_tol_m={tol_str} "
        f"phase={phase_label or 'n/a'}"
    )


def format_pick_request_log(
    *,
    ts_us: int,
    request_id: int,
    request_uuid: str,
    frame_id: str,
    pose,
    cartesian: bool,
    phase_label: Optional[str],
    ee_target_tol_m: Optional[float],
    accepted: bool = True,
) -> str:
    """Construye log ``[PICK][MOVEIT][REQUEST]``."""
    tol_str = (
        str(float(ee_target_tol_m))
        if ee_target_tol_m is not None
        else "n/a"
    )
    return (
        "[PICK][MOVEIT][REQUEST] "
        f"ts_us={int(ts_us)} request_id={int(request_id)} "
        f"request_uuid={request_uuid or 'n/a'} "
        f"frame={frame_id or 'n/a'} pose={format_pose_xyz(pose)} "
        f"cartesian={'true' if cartesian else 'false'} phase={phase_label or 'n/a'} "
        f"ee_target_tol_m={tol_str} "
        f"accepted={'true' if accepted else 'false'}"
    )


def format_exec_start_log(
    *,
    request_id: int,
    cartesian: bool,
    frame_id: str,
    ee_frame: str,
    base_frame: str,
    pose,
) -> str:
    """Construye log ``[BRIDGE][EXEC_START]``."""
    return (
        "[BRIDGE][EXEC_START] "
        f"request_id={int(request_id)} "
        f"label={'CARTESIAN' if cartesian else 'POSE'} "
        f"frame={frame_id} ee_link={ee_frame} base_frame={base_frame} "
        f"pos={format_pose_xyz(pose)}"
    )


def format_target_log(
    *,
    request_id: int,
    request_uuid: str,
    phase_label: Optional[str],
    frame_id: str,
    pose,
    cartesian: bool,
    ee_frame: str,
) -> str:
    """Construye log ``[PICK][MOVEIT][TARGET]``."""
    return (
        "[PICK][MOVEIT][TARGET] "
        f"request_id={int(request_id)} request_uuid={request_uuid or 'n/a'} "
        f"phase={phase_label or 'n/a'} frame={frame_id or 'n/a'} "
        f"pose={format_pose_xyz(pose)} "
        f"cartesian={'true' if cartesian else 'false'} "
        f"ee_frame={ee_frame or 'n/a'}"
    )


def format_busy_message(
    *,
    active_request_id: int,
    active_request_uuid: str,
    active_age_sec: float,
) -> str:
    """Construye string ``bridge_busy:active_request_id=...;..``."""
    return (
        "bridge_busy:"
        f"active_request_id={int(active_request_id)};"
        f"active_request_uuid={active_request_uuid or 'n/a'};"
        f"active_age={float(active_age_sec):.2f}s"
    )
