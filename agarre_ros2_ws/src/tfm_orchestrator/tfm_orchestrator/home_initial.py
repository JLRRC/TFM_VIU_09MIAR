#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/home_initial.py
# Contenido: B-iter6 (2026-05-03) — captura real de HOME_INITIAL en orchestrator.
"""B-iter6 — HOME_INITIAL real del PickOrchestrator.

Reemplaza el scaffold (B-iter2) por un movimiento real a la pose HOME del
UR5 vía cliente directo a ``/joint_trajectory_controller/follow_joint_trajectory``.

Esto rompe la dependencia panel→ HOME_INITIAL (que antes requería que el
panel mandara la trayectoria HOME). Ahora el orchestrator es autónomo:
construye la JointTrajectory + send_goal a la action del controller.

API pure/inyectable:

* ``build_home_joint_trajectory(home_positions, joint_names, duration_sec)``
  → ``trajectory_msgs.msg.JointTrajectory`` con un único punto (HOME) al
  tiempo ``duration_sec``.

* ``build_follow_joint_trajectory_goal(jt, position_tol_rad, goal_time_tol_sec)``
  → ``control_msgs.action.FollowJointTrajectory.Goal`` listo para
  ``send_goal_async``.

* ``parse_fjt_result(result_wrapper)`` → ``(success, reason)`` decodifica
  ``error_code`` del result.

Constantes UR5:

* HOME pose: ``(0.0, -π/2, 0.0, -π/2, 0.0, 0.0)`` — coincide con la pose
  de inicialización en ``controller_bootstrap`` (validada vía logs live).

* Joint names en orden SRDF canónico:
  ``shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3``.
"""

from __future__ import annotations

import math
from typing import Any, Sequence, Tuple


# Constantes del UR5.
UR5_HOME_POSITIONS_RAD: Tuple[float, ...] = (
    0.0, -math.pi / 2.0, 0.0, -math.pi / 2.0, 0.0, 0.0,
)
UR5_JOINT_NAMES_CANONICAL: Tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def build_home_joint_trajectory(
    home_positions: Sequence[float] = UR5_HOME_POSITIONS_RAD,
    joint_names: Sequence[str] = UR5_JOINT_NAMES_CANONICAL,
    duration_sec: float = 5.0,
) -> Any:
    """Construye un ``trajectory_msgs.msg.JointTrajectory`` con 1 punto HOME.

    Parameters:
        home_positions: posiciones target (rad). Default: HOME UR5.
        joint_names: orden SRDF de los 6 joints UR5.
        duration_sec: tiempo desde el inicio para alcanzar HOME (>=0.5s).

    Raises:
        ValueError: si len(home_positions) != len(joint_names).

    Imports lazy: no requiere ROS al importar el módulo.
    """
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    if len(home_positions) != len(joint_names):
        raise ValueError(
            f"home_positions/joint_names len mismatch: "
            f"{len(home_positions)} vs {len(joint_names)}"
        )

    duration = max(0.5, float(duration_sec))
    jt = JointTrajectory()
    jt.joint_names = list(joint_names)
    point = JointTrajectoryPoint()
    point.positions = [float(p) for p in home_positions]
    point.velocities = [0.0] * len(joint_names)
    point.accelerations = [0.0] * len(joint_names)
    sec = int(duration)
    nsec = int(round((duration - sec) * 1_000_000_000.0))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    point.time_from_start.sec = sec
    point.time_from_start.nanosec = nsec
    jt.points = [point]
    return jt


def build_follow_joint_trajectory_goal(
    jt: Any,
    *,
    position_tol_rad: float = 0.10,
    goal_time_tol_sec: float = 5.0,
) -> Any:
    """Construye un ``control_msgs.action.FollowJointTrajectory.Goal``.

    Setea path_tolerance y goal_tolerance por joint a ``position_tol_rad``,
    y goal_time_tolerance a ``goal_time_tol_sec``.

    Parameters:
        jt: JointTrajectory (output de build_home_joint_trajectory).
        position_tol_rad: tolerancia angular por joint (m=path == goal).
        goal_time_tol_sec: tolerancia temporal de finalización.
    """
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTolerance

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = jt
    pos_tol = float(max(0.01, position_tol_rad))
    goal.path_tolerance = [
        JointTolerance(name=str(jn), position=pos_tol)
        for jn in jt.joint_names
    ]
    goal.goal_tolerance = [
        JointTolerance(name=str(jn), position=pos_tol)
        for jn in jt.joint_names
    ]
    total = float(max(0.0, goal_time_tol_sec))
    sec = int(total)
    nsec = int(round((total - sec) * 1_000_000_000.0))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    goal.goal_time_tolerance.sec = sec
    goal.goal_time_tolerance.nanosec = nsec
    return goal


# FollowJointTrajectory.Result.error_code mapping. Sólo los más comunes.
_FJT_ERROR_NAMES = {
    0: "SUCCESSFUL",
    -1: "INVALID_GOAL",
    -2: "INVALID_JOINTS",
    -3: "OLD_HEADER_TIMESTAMP",
    -4: "PATH_TOLERANCE_VIOLATED",
    -5: "GOAL_TOLERANCE_VIOLATED",
}


def parse_fjt_result(result_wrapper: Any) -> Tuple[bool, str]:
    """Decodifica result de FollowJointTrajectory en (success, reason).

    El ``result_wrapper`` es lo que devuelve ``goal_handle.get_result_async()
    .result()`` — su atributo ``.result`` contiene
    ``FollowJointTrajectory.Result`` con ``error_code`` y ``error_string``.

    Devuelve (True, "fjt:SUCCESSFUL") si error_code==0, o
    (False, "fjt_err:<NAME>:<error_string>") en otro caso.
    """
    if result_wrapper is None:
        return False, "no_result"
    payload = getattr(result_wrapper, "result", result_wrapper)
    if payload is None:
        return False, "no_payload"
    err_code = int(getattr(payload, "error_code", -1))
    err_string = str(getattr(payload, "error_string", "") or "")
    name = _FJT_ERROR_NAMES.get(err_code, f"unknown:val={err_code}")
    if err_code == 0:
        return True, f"fjt:{name}"
    if err_string:
        return False, f"fjt_err:{name}:{err_string}"
    return False, f"fjt_err:{name}"
