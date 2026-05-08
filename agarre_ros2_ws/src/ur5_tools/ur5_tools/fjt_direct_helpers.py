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


def normalize_joint_to_pi(angle: float) -> float:
    """F1.24 H10 (2026-05-08): normaliza un ángulo joint al rango [-π, π].

    El IK de MoveIt puede devolver joints con wraps angulares (e.g. -3.387 rad
    cuando una solución equivalente sería +2.896 rad). Para joints UR5 que
    tienen rangos ±2π, los wraps son válidos pero el controller intenta el
    movimiento literal (giro de 3π = 540°) que excede límites físicos o
    requiere wall time excesivo.

    Esta función mapea cualquier ángulo al rango canónico [-π, π] módulo 2π,
    preservando la pose final (mismo punto en el círculo unitario) pero
    eliminando los wraps innecesarios.
    """
    import math
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def build_fjt_trajectory_multi_point(
    *,
    joint_names: Sequence[str],
    start_positions: Sequence[float],
    target_positions: Sequence[float],
    num_intermediate_points: int = 8,
    total_duration_sec: float = 20.0,
) -> Any:
    """F1.24 H11 (2026-05-08): JointTrajectory con N puntos interpolados.

    A diferencia de build_fjt_trajectory_two_point (start, target con
    velocidades cero), esta versión genera N+2 waypoints linealmente
    interpolados con velocidades intermedias no-cero. Esto da al
    joint_trajectory_controller una rampa suave que evita
    path_tolerance_violation por aceleración brusca o tracking error
    en trayectorias largas (TRANSPORT ~1m).

    Patrón usado en HOME_INITIAL exitoso: interpolación lineal en joint
    space + velocidades estimadas como diferencia central.

    Args:
        joint_names: orden esperado por el controller.
        start_positions: posiciones actuales.
        target_positions: target del IK (normalizado).
        num_intermediate_points: cuántos puntos entre start y target.
            Default 8 → 10 puntos totales (start + 8 + target).
        total_duration_sec: duración total. Velocidad media = delta/total.

    Returns:
        trajectory_msgs.msg.JointTrajectory con num_intermediate+2 puntos.
    """
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    if len(joint_names) != len(start_positions) or len(joint_names) != len(target_positions):
        raise ValueError(
            f"joint_names ({len(joint_names)}), start ({len(start_positions)}) "
            f"y target ({len(target_positions)}) deben tener misma longitud"
        )
    n_inter = max(0, int(num_intermediate_points))
    total_pts = n_inter + 2  # start + intermediates + target
    total_t = max(0.5, float(total_duration_sec))
    n = len(joint_names)
    deltas = [float(target_positions[i]) - float(start_positions[i]) for i in range(n)]
    # Velocidad media aproximada (positiva o negativa según signo del delta).
    v_mid = [d / total_t for d in deltas]

    jt = JointTrajectory()
    jt.joint_names = list(str(name) for name in joint_names)

    for k in range(total_pts):
        t_norm = float(k) / float(total_pts - 1)  # 0 .. 1
        positions = [
            float(start_positions[i]) + t_norm * deltas[i] for i in range(n)
        ]
        # Velocidades: 0 en extremos (k=0 y k=last), v_mid en intermedios.
        if k == 0 or k == total_pts - 1:
            velocities = [0.0] * n
        else:
            velocities = list(v_mid)
        time_sec_total = t_norm * total_t
        sec = int(time_sec_total)
        nsec = int(round((time_sec_total - sec) * 1_000_000_000.0))
        if nsec >= 1_000_000_000:
            sec += 1
            nsec -= 1_000_000_000
        p = JointTrajectoryPoint()
        p.positions = positions
        p.velocities = velocities
        p.time_from_start = Duration(sec=sec, nanosec=nsec)
        jt.points.append(p)
    return jt


def parse_ik_result(
    response: Any,
    joint_names: Sequence[str],
    *,
    normalize_joints: bool = True,
) -> Tuple[bool, str, Optional[List[float]]]:
    """Decodifica un GetPositionIK.Response.

    Args:
        response: GetPositionIK.Response del servicio.
        joint_names: nombres en el orden esperado.
        normalize_joints: F1.24 H10 (2026-05-08): si True (default),
            aplica normalize_joint_to_pi a cada joint para eliminar wraps
            angulares que el controller no puede ejecutar dentro de límites
            UR5. Mismo punto cinemático, joints "limpios" en [-π, π].

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
    ordered: List[float] = []
    for name in joint_names:
        if name not in name_to_pos:
            return False, f"ik_joint_missing:{name}", None
        joint_val: float = name_to_pos[name]
        if normalize_joints:
            joint_val = float(normalize_joint_to_pi(joint_val))
        ordered.append(joint_val)
    return True, "ik:SUCCESS", ordered


def build_fjt_path_tolerances(
    *,
    joint_names: Sequence[str],
    position_tolerance_rad: float,
    velocity_tolerance_rad_s: float = 0.0,
    acceleration_tolerance_rad_s2: float = 0.0,
) -> List[Any]:
    """F1.24 H14 (2026-05-08): construye lista de JointTolerance para el
    campo ``path_tolerance`` del FollowJointTrajectory.Goal.

    Mitiga el flakiness observado en T35 × 5 stress: tras stop+restart del
    stack, la trayectoria multi-waypoint generada por H11 puede cruzar
    tracking errors transitorios que el controller interpreta como
    ``path_tolerance_violation`` y aborta el goal sin "Goal reached".

    Setting de ``position`` generoso (default 0.3 rad ≈ 17°) absorbe esos
    tracking errors transitorios sin permitir desviaciones realmente
    peligrosas. ``velocity``/``acceleration`` a 0.0 = no chequear esas
    tolerancias.

    Args:
        joint_names: orden esperado por el controller (mismos que los del
            JointTrajectory ya construido).
        position_tolerance_rad: tolerancia angular por joint durante el
            seguimiento de la trayectoria (no al final).
        velocity_tolerance_rad_s: idem para velocidad. 0.0 = no chequear.
        acceleration_tolerance_rad_s2: idem para aceleración.

    Returns:
        Lista de control_msgs.msg.JointTolerance, una por joint.

    Imports lazy: no requiere ROS al importar este módulo.
    """
    from control_msgs.msg import JointTolerance

    tolerances: List[Any] = []
    pos_tol = float(position_tolerance_rad)
    vel_tol = float(velocity_tolerance_rad_s)
    acc_tol = float(acceleration_tolerance_rad_s2)
    for name in joint_names:
        jt = JointTolerance()
        jt.name = str(name)
        jt.position = pos_tol
        jt.velocity = vel_tol
        jt.acceleration = acc_tol
        tolerances.append(jt)
    return tolerances
