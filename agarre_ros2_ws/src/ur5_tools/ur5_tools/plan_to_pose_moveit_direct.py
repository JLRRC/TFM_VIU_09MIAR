#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/plan_to_pose_moveit_direct.py
# Contenido: B-iter3 (2026-05-03) — modo MOVEIT_DIRECT del plan_to_pose_server.
"""B-iter3 — Modo MOVEIT_DIRECT del PlanToPose action server.

Bypassa el bridge ``/desired_grasp → panel`` (modo REAL_BRIDGE original)
llamando directamente al action server ``/move_action`` que expone el
``move_group`` node de MoveIt 2.

Esto convierte el orchestrator en INDEPENDIENTE del panel para las fases
APPROACH/LIFT/TRANSPORT que dependen de plan_to_pose:

    Antes (REAL_BRIDGE):
        orchestrator → /pick_place → plan_to_pose_server → topic /desired_grasp
                                                               ↓
                                                          panel (atascado)
                                                               ↓
                                                       timeout 60s → FAIL

    Después (MOVEIT_DIRECT):
        orchestrator → /pick_place → plan_to_pose_server → /move_action
                                                               ↓
                                                          MoveGroup (move_group node)
                                                               ↓
                                                          plan + execute → SUCCESS

API:

* ``build_move_group_goal(target_xyz, target_quat_xyzw, ee_frame, base_frame,
  group_name="ur5_arm", planner_id="", planning_time_sec=5.0,
  position_tol_m=0.005, orientation_tol_rad=0.05)`` → ``MoveGroup.Goal``
  con MotionPlanRequest construido.

* ``parse_move_group_result(result_wrapper)`` → ``(success: bool, reason: str)``
  decodifica el ``MoveItErrorCodes`` del result en una razón textual.

Sin estado propio: 100% pure functions con tests offline.
"""

from __future__ import annotations

from typing import Any, NamedTuple, Tuple


class PhaseTuning(NamedTuple):
    """Tuning per-fase (F1.18). Devuelto por classify_phase_by_target_z."""

    velocity_scaling: float
    acceleration_scaling: float
    first_attempt_timeout_sec: float
    phase_label: str  # "TRANSPORT" | "OTHER"


# F1.18 (2026-05-08): heurística per-fase. Umbral en target Z (base_link).
# TRANSPORT (drop pose) está a Z_world≈0.85 ≈ base_link Z=0 → < 0.05.
# Otras fases (APPROACH/GRASP_DOWN/LIFT) tienen Z > 0.05 base_link.
_TRANSPORT_Z_THRESHOLD_M = 0.05
_TRANSPORT_VEL_SCALING = 0.5
_TRANSPORT_ACC_SCALING = 0.5
_TRANSPORT_FIRST_ATTEMPT_TIMEOUT_SEC = 240.0
_DEFAULT_VEL_SCALING = 0.25
_DEFAULT_ACC_SCALING = 0.25
_DEFAULT_FIRST_ATTEMPT_TIMEOUT_SEC = 120.0


def classify_phase_by_target_z(
    target_xyz: Tuple[float, float, float],
    *,
    transport_z_threshold_m: float = _TRANSPORT_Z_THRESHOLD_M,
) -> PhaseTuning:
    """Clasifica la fase por la coordenada Z del target en base_link (F1.18).

    Heurística: el drop pose de TRANSPORT está a Z_world≈0.85 ≈ base_link Z=0
    (la base del UR5 está a Z_world=0.85). Cualquier otra fase del pick demo
    (APPROACH, GRASP_DOWN, LIFT) opera con target Z > 0.05 base_link.

    Returns:
        PhaseTuning con (velocity_scaling, acceleration_scaling,
        first_attempt_timeout_sec, phase_label).

    Justificación: TRANSPORT (~1m de recorrido) con scaling=0.25 produce
    trayectorias > 120s wall que disparan first_attempt_timeout. Subir a
    scaling=0.5 baja la duración a ~50% y timeout 240s da margen para retry.
    """
    z = float(target_xyz[2])
    if z < float(transport_z_threshold_m):
        return PhaseTuning(
            velocity_scaling=_TRANSPORT_VEL_SCALING,
            acceleration_scaling=_TRANSPORT_ACC_SCALING,
            first_attempt_timeout_sec=_TRANSPORT_FIRST_ATTEMPT_TIMEOUT_SEC,
            phase_label="TRANSPORT",
        )
    return PhaseTuning(
        velocity_scaling=_DEFAULT_VEL_SCALING,
        acceleration_scaling=_DEFAULT_ACC_SCALING,
        first_attempt_timeout_sec=_DEFAULT_FIRST_ATTEMPT_TIMEOUT_SEC,
        phase_label="OTHER",
    )


def build_move_group_goal(
    target_xyz: Tuple[float, float, float],
    target_quat_xyzw: Tuple[float, float, float, float],
    *,
    ee_frame: str,
    base_frame: str,
    group_name: str = "manipulator",
    planner_id: str = "",
    planning_time_sec: float = 5.0,
    position_tol_m: float = 0.005,
    orientation_tol_rad: float = 0.05,
    velocity_scaling_factor: float = 0.25,
    acceleration_scaling_factor: float = 0.25,
) -> Any:
    """Construye un ``moveit_msgs.action.MoveGroup.Goal`` para target pose.

    Parameters:
        target_xyz: posición destino del end effector (x, y, z) en base_frame.
        target_quat_xyzw: orientación destino (x, y, z, w).
        ee_frame: link del end effector (e.g. "rg2_pinch_center", "tool0").
        base_frame: frame de referencia (e.g. "base_link").
        group_name: planning group de SRDF (default "ur5_arm").
        planner_id: planner específico (vacío = default OMPL).
        planning_time_sec: timeout del planner.
        position_tol_m: tolerancia posicional (m) para PositionConstraint.
        orientation_tol_rad: tolerancia angular (rad) para OrientationConstraint.

    Returns:
        moveit_msgs.action.MoveGroup.Goal listo para ``send_goal_async``.

    Imports lazy: no requiere ROS al importar este módulo (sólo al llamar).
    """
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (
        BoundingVolume,
        Constraints,
        OrientationConstraint,
        PlanningOptions,
        PositionConstraint,
    )
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Point, Pose, Quaternion

    goal = MoveGroup.Goal()

    # MotionPlanRequest
    req = goal.request
    req.group_name = str(group_name or "ur5_arm")
    req.planner_id = str(planner_id or "")
    req.allowed_planning_time = float(planning_time_sec)
    req.num_planning_attempts = 5
    # 2026-05-07: bajado 0.5→0.3 para que la trayectoria sea más lenta y el
    # controller en gz_ros2_control la siga sin path_tolerance_violation.
    # F1.11 audit-v4 (2026-05-08): bajado 0.3 → 0.1. Validación v6 mostró
    # FJT goal_time_tolerance abort 3/3 ciclos a ~400s. RTF de Gazebo Sim
    # bajo + scaling 0.3 = trayectoria demasiado rápida para tracking.
    # Con 0.1, la trayectoria nominal triplica su duración, dando tiempo
    # al PID para converger antes del goal_time_tolerance.
    # F1.17 audit-v4 (2026-05-08): subido 0.1 → 0.25. La causa real de la
    # no convergencia era interpolate_from_desired_state=false (resuelto en
    # F1.11). Con scaling=0.1, una trayectoria de 8s nominal dura 80s sim
    # ≈ 130s wall — supera el first_attempt_timeout (120s). 0.25 = 32s sim
    # ≈ 50s wall, dentro del budget para fases cortas.
    # F1.18 (2026-05-08): scaling per-call. Llamadores (plan_to_pose_server)
    # detectan TRANSPORT (target Z < 0.05 base_link) y pasan 0.5 scaling
    # para las trayectorias largas (~1m) que con 0.25 superan 120s wall.
    req.max_velocity_scaling_factor = float(velocity_scaling_factor)
    req.max_acceleration_scaling_factor = float(acceleration_scaling_factor)

    # Goal constraints: position + orientation sobre ee_frame.
    pc = PositionConstraint()
    pc.header.frame_id = str(base_frame)
    pc.link_name = str(ee_frame)
    pc.target_point_offset.x = 0.0
    pc.target_point_offset.y = 0.0
    pc.target_point_offset.z = 0.0
    bv = BoundingVolume()
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [float(position_tol_m)]
    bv.primitives = [sphere]
    bv.primitive_poses = [
        Pose(
            position=Point(
                x=float(target_xyz[0]),
                y=float(target_xyz[1]),
                z=float(target_xyz[2]),
            ),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    ]
    pc.constraint_region = bv
    pc.weight = 1.0

    oc = OrientationConstraint()
    oc.header.frame_id = str(base_frame)
    oc.link_name = str(ee_frame)
    oc.orientation = Quaternion(
        x=float(target_quat_xyzw[0]),
        y=float(target_quat_xyzw[1]),
        z=float(target_quat_xyzw[2]),
        w=float(target_quat_xyzw[3]),
    )
    oc.absolute_x_axis_tolerance = float(orientation_tol_rad)
    oc.absolute_y_axis_tolerance = float(orientation_tol_rad)
    oc.absolute_z_axis_tolerance = float(orientation_tol_rad)
    oc.weight = 1.0

    constraints = Constraints()
    constraints.position_constraints = [pc]
    constraints.orientation_constraints = [oc]
    req.goal_constraints = [constraints]

    # PlanningOptions: plan + execute (no plan_only).
    opts = PlanningOptions()
    opts.plan_only = False
    opts.replan = False
    opts.replan_attempts = 0
    opts.look_around = False
    goal.planning_options = opts

    return goal


# MoveItErrorCodes mapping. Sólo los más comunes; el resto se reportan
# como "moveit_err:val=N" para evitar tener que sincronizar el enum entero.
_MOVEIT_ERROR_NAMES = {
    1: "SUCCESS",
    99999: "FAILURE",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -19: "INVALID_OBJECT_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
}


def parse_move_group_result(result_wrapper: Any) -> Tuple[bool, str]:
    """Decodifica result de MoveGroup en (success, reason).

    El ``result_wrapper`` es lo que devuelve ``goal_handle.get_result_async()
    .result()`` — su atributo ``.result`` contiene el ``MoveGroup.Result``
    propiamente dicho con ``error_code.val``.
    """
    if result_wrapper is None:
        return False, "no_result"
    payload = getattr(result_wrapper, "result", result_wrapper)
    if payload is None:
        return False, "no_payload"
    err = getattr(payload, "error_code", None)
    if err is None:
        return False, "no_error_code"
    val = int(getattr(err, "val", 0))
    name = _MOVEIT_ERROR_NAMES.get(val, f"unknown:val={val}")
    if val == 1:
        return True, f"moveit:{name}"
    return False, f"moveit_err:{name}"
