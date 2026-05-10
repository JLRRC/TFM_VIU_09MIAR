"""Modo REAL_BRIDGE de plan_to_pose_server (audit Action 12).

Extraído de plan_to_pose_server.py para reducir el monolito (1507 LOC).
Toma el `node` (PlanToPoseServer) como primer arg y accede a sus atributos
de estado del bridge (`_bridge_pose_pub`, `_bridge_result_lock`, etc.).

Comportamiento idéntico al método original. Lógica pura ya existente
(encode_request_frame, parse_bridge_result, normalize_quat) sigue
viviendo en `plan_to_pose_logic.py`.
"""
from __future__ import annotations

import time
import uuid
from typing import TYPE_CHECKING

from geometry_msgs.msg import PoseStamped

from .plan_to_pose_logic import (
    PlanToPoseGoal,
    PlanToPoseResult,
    encode_request_frame,
    normalize_quat,
    parse_bridge_result,
)

if TYPE_CHECKING:  # pragma: no cover
    from .plan_to_pose_server import PlanToPoseServer


def execute_real_bridge(
    node: "PlanToPoseServer", goal: PlanToPoseGoal, start_mono: float
) -> PlanToPoseResult:
    """Publica el goal al bridge MoveIt y espera result correlado por UUID.

    Estado del bridge en `node` (instance attrs):
      - _bridge_pose_pub
      - _next_request_id
      - _bridge_base_frame
      - _bridge_result_lock
      - _bridge_pending_uuid / _bridge_pending_text / _bridge_pending_event
      - _bridge_result_timeout
    """
    if node._bridge_pose_pub is None:
        return PlanToPoseResult(
            success=False,
            reason="bridge_publisher_not_initialized",
            final_xyz=goal.target_xyz,
            final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
            duration_sec=time.monotonic() - start_mono,
            attempts=0,
        )

    request_uuid = uuid.uuid4().hex
    node._next_request_id += 1
    request_id = int(node._next_request_id)
    frame_id = encode_request_frame(
        node._bridge_base_frame,
        request_id,
        request_uuid,
        phase_label="PLAN_TO_POSE",
    )

    msg = PoseStamped()
    msg.header.frame_id = frame_id
    try:
        msg.header.stamp = node.get_clock().now().to_msg()
    except Exception:
        pass
    msg.pose.position.x = float(goal.target_xyz[0])
    msg.pose.position.y = float(goal.target_xyz[1])
    msg.pose.position.z = float(goal.target_xyz[2])
    msg.pose.orientation.x = float(goal.target_quat_xyzw[0])
    msg.pose.orientation.y = float(goal.target_quat_xyzw[1])
    msg.pose.orientation.z = float(goal.target_quat_xyzw[2])
    msg.pose.orientation.w = float(goal.target_quat_xyzw[3])

    # Armar pending antes de publicar para evitar race con result rápido.
    with node._bridge_result_lock:
        node._bridge_pending_uuid = request_uuid
        node._bridge_pending_text = None
        node._bridge_pending_event.clear()

    try:
        node._bridge_pose_pub.publish(msg)
    except Exception as exc:
        return PlanToPoseResult(
            success=False,
            reason=f"bridge_publish_exception:{type(exc).__name__}:{exc}",
            final_xyz=goal.target_xyz,
            final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
            duration_sec=time.monotonic() - start_mono,
            attempts=1,
        )

    node.get_logger().info(
        f"[PLAN_TO_POSE][BRIDGE] published rid={request_id} uid={request_uuid} "
        f"target=({goal.target_xyz[0]:.3f},{goal.target_xyz[1]:.3f},{goal.target_xyz[2]:.3f})"
    )

    timeout = float(max(1.0, node._bridge_result_timeout))
    ok_wait = node._bridge_pending_event.wait(timeout=timeout)
    with node._bridge_result_lock:
        text = node._bridge_pending_text
        node._bridge_pending_uuid = None  # release slot

    if not ok_wait or text is None:
        return PlanToPoseResult(
            success=False,
            reason=f"bridge_result_timeout:{timeout:.1f}s",
            final_xyz=goal.target_xyz,
            final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
            duration_sec=time.monotonic() - start_mono,
            attempts=1,
        )

    success, reason, _uid = parse_bridge_result(text)
    ok = bool(success) if success is not None else False
    return PlanToPoseResult(
        success=ok,
        reason=reason or ("ok" if ok else "bridge_unknown"),
        final_xyz=goal.target_xyz,
        final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
        duration_sec=time.monotonic() - start_mono,
        attempts=1,
    )
