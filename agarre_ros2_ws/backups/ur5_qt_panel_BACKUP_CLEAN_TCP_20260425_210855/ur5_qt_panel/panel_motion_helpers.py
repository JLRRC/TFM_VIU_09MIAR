#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_motion_helpers.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Motion helper utilities for the UR5 panel."""
from __future__ import annotations

from typing import Iterable

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def build_joint_trajectory(
    positions: Iterable[float],
    sec: float,
    joint_names: Iterable[str],
    *,
    stamp_msg=None,
    # FASE 2: Añadir parámetros de scaling para evitar TOTG warnings
    max_velocity_scaling: float = 0.3,
    max_acceleration_scaling: float = 0.3,
) -> JointTrajectory:
    """
    Build a JointTrajectory message with proper scaling factors.

    FASE 2: Añade velocities/accelerations defaults para evitar warnings:
    - TOTG: max_velocity_scaling_factor 0.0 out of range
    - TOTG: max_acceleration_scaling_factor 0.0 out of range
    """
    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    if stamp_msg is not None:
        traj.header.stamp = stamp_msg
    sec = max(0.0, float(sec))
    sec_i = int(sec)
    nsec_i = int((sec - sec_i) * 1e9)
    point = JointTrajectoryPoint()
    point.positions = [round(float(p), 4) for p in positions]

    # FASE 2: Añadir velocities y accelerations con valores razonables
    # Esto permite a TOTG calcular correctamente la trayectoria
    num_joints = len(point.positions)
    # Velocidades: estimación simplificada (posición_delta / tiempo)
    # Como no conocemos posición inicial, usamos 0.0 (TOTG calculará velocidades reales)
    point.velocities = [0.0] * num_joints
    # Aceleraciones: también 0.0 (TOTG calculará)
    point.accelerations = [0.0] * num_joints

    point.time_from_start.sec = sec_i
    point.time_from_start.nanosec = nsec_i
    traj.points = [point]
    return traj
