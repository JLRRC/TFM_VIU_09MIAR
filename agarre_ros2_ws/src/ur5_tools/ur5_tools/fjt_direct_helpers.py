#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/fjt_direct_helpers.py
"""F1.24 / H9 LIVE (2026-05-08) — Helpers puros para bypass MoveIt en APPROACH.

Objetivo: evitar el bug ``BUG_CONTROLLER_FEEDBACK_HANG`` que ocurre en el
path MoveIt → simple_controller_manager → joint_trajectory_controller.
HOME_INITIAL bypassa ese path (envía FJT directo al controller) y siempre
funciona — extender ese patrón a APPROACH.

Flujo:
1. Llamar a ``/compute_ik`` (síncrono, sin pasar por
   simple_controller_manager) para obtener joint_positions de la pose XYZ.
2. Construir un JointTrajectory de 2 puntos (current → target).
3. Enviar al ``/joint_trajectory_controller/follow_joint_trajectory`` action
   directamente.
4. Esperar el "Goal reached, success!" (ese path no tiene el bug).

Esta función puede ser fallback cuando el modo MOVEIT_DIRECT cuelga, o
ser el modo principal si la trayectoria no necesita collision avoidance
compleja (que es el caso del APPROACH típico: HOME → encima del objeto).

API pura (sin ROS al importar):
- ``build_ik_request(target_xyz, target_quat_xyzw, ee_frame, base_frame,
   group_name, current_joints, joint_names, timeout_sec)`` → PositionIKRequest
- ``build_fjt_trajectory_two_point(joint_names, start_positions,
   target_positions, duration_sec)`` → JointTrajectory
- ``parse_ik_result(response)`` → (success, reason, joint_positions)

Tests offline en ``test/test_fjt_direct_helpers.py``.
"""

from __future__ import annotations

from typing import Any, List, Optional, Sequence, Tuple


def build_ik_request(
    *,
    target_xyz: Tuple[float, float, float],
    target_quat_xyzw: Tuple[float, float, float, float],
    ee_frame: str,
    base_frame: str,
    group_name: str,
    current_joints: Sequence[float],
    joint_names: Sequence[str],
    timeout_sec: float = 1.0,
    avoid_collisions: bool = False,
) -> Any:
    """Construye un GetPositionIK.Request.

    Args:
        target_xyz / target_quat_xyzw: pose target del ee_frame en base_frame.
        ee_frame: link del end effector (ik_link_name, e.g. "rg2_pinch_center").
        base_frame: frame de referencia.
        group_name: planning group SRDF (e.g. "manipulator").
        current_joints: posiciones actuales (seed para IK).
        joint_names: nombres de los joints en el orden de current_joints.
        timeout_sec: timeout del solver IK.
        avoid_collisions: si True, IK evita colisiones (más lento). Default
            False — para APPROACH a un target XYZ libre, no se necesita.

    Returns:
        moveit_msgs.srv.GetPositionIK.Request listo para call_async.

    Imports lazy: no requiere ROS al importar este módulo.
    """
    from moveit_msgs.srv import GetPositionIK
    from moveit_msgs.msg import PositionIKRequest, RobotState
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
    from builtin_interfaces.msg import Duration

    req = GetPositionIK.Request()

    ik_req = PositionIKRequest()
    ik_req.group_name = str(group_name)
    ik_req.ik_link_name = str(ee_frame)
    ik_req.avoid_collisions = bool(avoid_collisions)

    # Pose target stamped en base_frame.
    ps = PoseStamped()
    ps.header.frame_id = str(base_frame)
    ps.pose = Pose(
        position=Point(
            x=float(target_xyz[0]),
            y=float(target_xyz[1]),
            z=float(target_xyz[2]),
        ),
        orientation=Quaternion(
            x=float(target_quat_xyzw[0]),
            y=float(target_quat_xyzw[1]),
            z=float(target_quat_xyzw[2]),
            w=float(target_quat_xyzw[3]),
        ),
    )
    ik_req.pose_stamped = ps

    # Seed: estado actual del robot.
    rs = RobotState()
    js = JointState()
    js.name = list(str(n) for n in joint_names)
    js.position = list(float(p) for p in current_joints)
    rs.joint_state = js
    ik_req.robot_state = rs

    # Timeout del solver.
    timeout_total = max(0.05, float(timeout_sec))
    sec = int(timeout_total)
    nsec = int(round((timeout_total - sec) * 1_000_000_000.0))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    ik_req.timeout = Duration(sec=sec, nanosec=nsec)

    req.ik_request = ik_req
    return req


def build_fjt_trajectory_two_point(
    *,
    joint_names: Sequence[str],
    start_positions: Sequence[float],
    target_positions: Sequence[float],
    duration_sec: float = 5.0,
) -> Any:
    """Construye un JointTrajectory de 2 puntos (current → target).

    Args:
        joint_names: nombres en el orden esperado por el controller.
        start_positions: posiciones actuales.
        target_positions: posiciones target del IK.
        duration_sec: duración total de la trayectoria.

    Returns:
        trajectory_msgs.msg.JointTrajectory.

    Notas: el primer punto está en t=0 (current state); el segundo en
    t=duration_sec (target). El controller interpola linealmente.
    """
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    if len(joint_names) != len(start_positions) or len(joint_names) != len(target_positions):
        raise ValueError(
            f"joint_names ({len(joint_names)}), start ({len(start_positions)}) "
            f"y target ({len(target_positions)}) deben tener misma longitud"
        )

    jt = JointTrajectory()
    jt.joint_names = list(str(n) for n in joint_names)

    # Punto inicial t=0
    p0 = JointTrajectoryPoint()
    p0.positions = list(float(v) for v in start_positions)
    p0.velocities = [0.0] * len(start_positions)
    p0.time_from_start = Duration(sec=0, nanosec=0)
    jt.points.append(p0)

    # Punto final t=duration
    total = max(0.1, float(duration_sec))
    sec = int(total)
    nsec = int(round((total - sec) * 1_000_000_000.0))
    if nsec >= 1_000_000_000:
        sec += 1
        nsec -= 1_000_000_000
    p1 = JointTrajectoryPoint()
    p1.positions = list(float(v) for v in target_positions)
    p1.velocities = [0.0] * len(target_positions)
    p1.time_from_start = Duration(sec=sec, nanosec=nsec)
    jt.points.append(p1)

    return jt


def parse_ik_result(
    response: Any,
    joint_names: Sequence[str],
) -> Tuple[bool, str, Optional[List[float]]]:
    """Decodifica un GetPositionIK.Response.

    Returns:
        Tuple[success, reason, joint_positions]:
        - success: True si error_code.val == 1 (SUCCESS).
        - reason: descripción human-readable.
        - joint_positions: list[float] alineado con joint_names si SUCCESS,
          None en caso contrario.
    """
    if response is None:
        return False, "ik_no_response", None
    err = getattr(response, "error_code", None)
    if err is None:
        return False, "ik_no_error_code", None
    val = int(getattr(err, "val", 0))
    if val != 1:
        # Códigos comunes: -31 NO_IK_SOLUTION, -1 PLANNING_FAILED.
        reason_map = {
            -1: "PLANNING_FAILED",
            -10: "START_STATE_IN_COLLISION",
            -12: "GOAL_IN_COLLISION",
            -15: "INVALID_GROUP_NAME",
            -16: "INVALID_GOAL_CONSTRAINTS",
            -18: "INVALID_LINK_NAME",
            -31: "NO_IK_SOLUTION",
        }
        name = reason_map.get(val, f"err_val={val}")
        return False, f"ik:{name}", None

    sol = getattr(response, "solution", None)
    if sol is None:
        return False, "ik_no_solution", None
    js = getattr(sol, "joint_state", None)
    if js is None:
        return False, "ik_no_joint_state", None
    sol_names = list(getattr(js, "name", []) or [])
    sol_positions = list(getattr(js, "position", []) or [])
    if not sol_names or not sol_positions:
        return False, "ik_empty_joint_state", None

    # Alinear a joint_names esperados.
    name_to_pos = {str(n): float(p) for n, p in zip(sol_names, sol_positions)}
    ordered = []
    for name in joint_names:
        if name not in name_to_pos:
            return False, f"ik_joint_missing:{name}", None
        ordered.append(name_to_pos[name])
    return True, "ik:SUCCESS", ordered
