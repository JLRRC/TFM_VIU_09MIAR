#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Pick demo sequence helper for the panel."""
from __future__ import annotations

import os
import json
import math
import time
import threading
import uuid
from datetime import datetime, timezone
from pathlib import Path

try:
    from std_msgs.msg import Empty
except Exception:  # pragma: no cover - ROS not available in unit contexts
    Empty = None

try:
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray
except Exception:  # pragma: no cover - ROS not available in unit contexts
    Point = None
    Marker = None
    MarkerArray = None

from ur5_tools.gripper_geometry import (
    RG2_PINCH_CENTER_FRAME,
    RG2_TCP_FRAME,
    TOOL0_FRAME,
    load_gripper_geometry,
)

from .panel_robot_presets import (
    JOINT_TABLE_POSE_RAD,
    JOINT_PICK_IMAGE_POSE_RAD,
    JOINT_GRASP_DOWN_POSE_RAD,
    JOINT_BASKET_POSE_RAD,
    JOINT_BASKET_DEMO_RELEASE_POSE_RAD,
    JOINT_PICK_DEMO_REFERENCE_PRE_CLOSE_POSE_RAD,
)
from .panel_config import (
    BASKET_DROP,
    WORLD_FRAME,
    BASE_FRAME,
    GRIPPER_ATTACH_PREFIX,
    GRIPPER_CLOSED_RAD,
    GRIPPER_OPEN_RAD,
    GRIPPER_TCP_Z_OFFSET,
    PICK_DEMO_TRANSPORT_Z_OFFSET,
    PICK_DEMO_DROP_Z_OFFSET,
    UR5_JOINT_NAMES,
    GRIPPER_JOINT_NAMES,
)
from .panel_robot_presets import PICK_DEMO_OBJECT_NAME
from .panel_pick_demo_params import (
    load_pick_demo_params,
    get_pick_demo_params as _get_pick_demo_params,
)
from .panel_objects import (
    mark_object_grasped,
    mark_object_attached,
    mark_object_released,
    get_object_state,
    is_on_table,
    update_object_state,
    ObjectOwner,
    ObjectLogicalState,
)
from .panel_readiness import tf_ready_status
from .directo_geometry import (
    angle_shortest_diff_rad,
    _pick_demo_tuple3,
    _pick_demo_fmt_scalar,
    _pick_demo_env_float,
    _pick_demo_env_int,
    _pick_demo_env_flag,
    _effective_direct_grasp_z,
    _direct_runtime_target_tol_m,
    _is_demo_basket_transport_stage,
    _compute_demo_basket_targets,
    _compute_demo_linear_stage_targets,
    _compute_demo_stage_count_for_distance,
    _compute_demo_transport_recovery_stage_targets,
    _compute_demo_transport_micro_recovery_target,
    _compute_demo_joint_prep_waypoints,
    _compute_demo_transport_prep_joint_tol,
)
from .panel_utils import (
    fmt_vec3,
    get_pose,
    transform_point_to_frame,
    world_to_base,
    world_xyz_to_pixel_float,
    table_xy_to_pixel_float,
)
from .ur5_kinematics import fk_ur5, ik_ur5
from .attach_gate_evaluator import AttachGateEvaluator, AttachGateConfig
from .directo_gate_evaluator import (
    _should_apply_global_step_timeout_extra,
    _coerce_ur5_joint_vector,
    _normalize_joint_goal_for_execution,
    _build_transport_seed_candidates,
    _evaluate_transport_stage_preexec_model_guard,
    _direct_pregrasp_gate_caps,
    _should_transport_prep_failure_jump_to_replan,
    _evaluate_transport_stage_postcheck,
    _transport_prep_failure_policy,
)
from .pick_demo.internal_helpers import (
    _live_joint_seed_or_none,
    _resolve_live_object_world,
    _resolve_live_object_base,
    _select_pick_demo_cycle_object_reference,
    _demo_object_in_basket,
    _validate_demo_transport_follow,
)
from .pick_demo.pure_helpers import (
    iso_now as _iso_now,
    json_safe as _json_safe,
    vector_minus as _vector_minus,
    vec_norm as _vec_norm,
    z_delta as _z_delta,
)
from .pick_demo.marker_helpers import (
    make_sphere_marker as _make_sphere_marker_pure,
    make_arrow_marker as _make_arrow_marker_pure,
)
from .pick_demo.debug_markers import (
    publish_direct_debug_markers as _publish_direct_debug_markers_pure,
)
from .pick_demo.metrics import (
    alignment_metrics_base as _alignment_metrics_base,
    compute_pose_consistency_metrics as _compute_pose_consistency_metrics_pure,
    grasp_down_runtime_metrics as _grasp_down_runtime_metrics_pure,
    joint_delta_metrics as _joint_delta_metrics_pure,
    joint_error_metrics as _joint_error_metrics_pure,
)
from .pick_demo.geometry import (
    apply_local_offset_to_fk as _apply_local_offset_to_fk,
    dynamic_pre_close_reference as _dynamic_pre_close_reference_pure,
    object_top_pose as _object_top_pose_pure,
    vec_dist3 as _dist,  # F3-step1: alias canónico, reemplaza la def local previa.
)

# F3-step1: alias module-level (reemplaza def local en worker).
_tuple3 = _pick_demo_tuple3


def _pose_position(target_frame: str, source_frame: str, *, timeout_sec: float = 0.20):
    pose, _pose_err = get_pose(target_frame, source_frame, timeout_sec=timeout_sec)
    if pose is None:
        return None
    return _tuple3(pose.get("position"))


def _object_top_pose(pose):
    return _object_top_pose_pure(
        _tuple3(pose),
        _pick_demo_env_float("PANEL_PICK_DEMO_OBJECT_HEIGHT_M", 0.05, minimum=0.001),
    )
from .pick_demo.phase_checks import (
    build_approach_coarse_phase_check as _build_approach_coarse_phase_check_pure,
)
from .pick_demo.audit_emit import (
    AuditEmitContext,
    audit_emit as _audit_emit_pure,
)
from .pick_demo.transport_replan import (
    TransportReplanContext,
    attempt_transport_replan as _attempt_transport_replan_pure,
)
from .pick_demo.align_grasp import (
    AlignGraspContext,
    AlignGraspState,
    align_demo_grasp_direct as _align_demo_grasp_direct_pure,
)
from .pick_demo.grasp_down import (
    GraspDownContext,
    run_grasp_down_conservative as _run_grasp_down_conservative_pure,
)
from .pick_demo.joint_step import (
    JointStepContext,
    run_joint_step as _run_joint_step_pure,
)
from .pick_demo.wait_gripper_target import (
    wait_for_gripper_target as _wait_for_gripper_target_pure,
)
from .pick_demo.wait_runtime_tcp_stable import (
    wait_runtime_tcp_stable as _wait_runtime_tcp_stable_pure,
)

_DIRECT_GRIPPER_GEOMETRY = load_gripper_geometry()
_DIRECT_TOOL0_TO_SOURCE_OFFSET = _DIRECT_GRIPPER_GEOMETRY.xyz_for_frame(
    RG2_PINCH_CENTER_FRAME
)

DIRECT_ROUTE_MODE = "direct_rg2_pinch_center"
DIRECT_SOURCE_FRAME = RG2_PINCH_CENTER_FRAME
DIRECT_LEGACY_TCP_FRAME = RG2_TCP_FRAME
DIRECT_EXECUTION_FRAME = TOOL0_FRAME
DIRECT_EXECUTION_IK_MODE = "formal_rg2_pinch_center_source_to_tool0_numeric"
DIRECT_TOOL0_TO_RG2_TCP_Z_M = float(_DIRECT_TOOL0_TO_SOURCE_OFFSET[2])
DIRECT_GRASP_AUDIT_PREFIX = "[PICK][DIRECT_GRASP_AUDIT]"


# _effective_direct_grasp_z, _direct_runtime_target_tol_m,
# pick_demo_target_semantics, _is_demo_basket_transport_stage/motion
# → moved to directo_geometry.py


# _should_apply_global_step_timeout_extra → moved to directo_gate_evaluator.py

# _pick_demo_tuple3, _pick_demo_fmt_scalar, _pick_demo_env_{float,int,flag}
# → moved to directo_geometry.py

# _coerce_ur5_joint_vector → moved to directo_gate_evaluator.py


# F3-step1.2: helpers triviales promovidos del closure ``run_pick_demo.worker``.
# Sin dependencias del closure (free=0). Cada ``def`` local equivalente queda
# eliminada del worker; Python resuelve la referencia por scope hacia este
# nivel module-level.

def _fmt_vec(vec) -> str:
    return fmt_vec3(vec)


def _fmt_scalar(value, *, digits: int = 3) -> str:
    return _pick_demo_fmt_scalar(value, digits=digits)


def _fmt_px(px) -> str:
    if px is None:
        return "none"
    try:
        return f"({float(px[0]):.1f},{float(px[1]):.1f})"
    except Exception:
        return "none"


def _execution_type_from_decision(decision: str | None) -> str:
    # F3-step40 (2026-05-08): delega a pick_demo.decision_helpers (puro).
    from .pick_demo.decision_helpers import execution_type_from_decision
    return execution_type_from_decision(decision)


def _grasp_down_permissive_ik_err_tol() -> float:
    requested = _get_pick_demo_params().grasp_down_permissive_ik_err_tol
    return min(0.025, max(0.010, requested))


def _joint_delta_metrics(reference_joints, final_joints) -> dict:
    return _joint_delta_metrics_pure(
        reference_joints, final_joints, UR5_JOINT_NAMES
    )


def _write_json_snapshot(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(_json_safe(payload), fh, indent=2, ensure_ascii=False, sort_keys=True)


# F3-step1.3: helpers UI promovidos del closure ``run_pick_demo.worker``.
# Toman ``panel`` como primer parámetro. Los callsites pasan a ``fn(panel)``
# (llamadas directas) o ``lambda: fn(panel)`` (cuando se usan como callables
# en ``signal_run_ui.emit`` o ``cmd_fn=...``).

def _direct_debug_stamp(panel):
    try:
        node = getattr(panel, "_moveit_node", None) or getattr(panel, "_node", None)
        if node is not None and hasattr(node, "get_clock"):
            return node.get_clock().now().to_msg()
    except Exception:
        pass
    return None


def _camera_frame_size(panel) -> tuple[int, int]:
    view = getattr(panel, "camera_view", None)
    if view is None:
        return 0, 0
    try:
        fw = int(getattr(view, "_img_width", 0) or 0)
        fh = int(getattr(view, "_img_height", 0) or 0)
    except Exception:
        fw, fh = 0, 0
    return fw, fh


def _current_arm_joint_snapshot(panel) -> list[float | None]:
    snapshot = dict(getattr(panel, "_last_joint_positions", {}) or {})
    values = []
    for joint_name in UR5_JOINT_NAMES:
        current = snapshot.get(joint_name)
        if current is None:
            values.append(None)
        else:
            values.append(float(current))
    return values


def _camera_audit_meta(panel) -> dict:
    topic = str(getattr(panel, "camera_topic", "") or "none").strip() or "none"
    image_ts = None
    frame_data = getattr(panel, "_last_camera_frame", None)
    if frame_data:
        try:
            image_ts = float(frame_data[3])
        except Exception:
            image_ts = None
    return {
        "topic": topic,
        "frame": "unknown",
        "image_timestamp": image_ts,
    }


def _joint_error_snapshot(panel, joints) -> str:
    names = list(getattr(panel, "UR5_JOINT_NAMES", []) or [])
    if not names:
        names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
    parts = []
    for idx, name in enumerate(names):
        if idx >= len(joints):
            break
        curr = panel._last_joint_positions.get(name)
        if curr is None:
            parts.append(f"{name}=n/a")
            continue
        diff = abs(float(curr) - float(joints[idx]))
        parts.append(f"{name}={diff:.3f}")
    return " ".join(parts)


def _joint_error_metrics(panel, joints) -> dict:
    names = list(getattr(panel, "UR5_JOINT_NAMES", []) or []) or [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    snapshot = dict(getattr(panel, "_last_joint_positions", {}) or {})
    return _joint_error_metrics_pure(joints, names, snapshot)


def _open_gripper_short(panel) -> None:
    panel._command_gripper(False, log_action="DROP", force=True)


def _lock_pick_demo_button(panel) -> None:
    panel.btn_pick_demo.setEnabled(False)
    panel.btn_pick_demo.setToolTip("Ya ejecutado: objeto demo confirmado en cesta")
    panel._ui_set_status("Pick demo completado", error=False)
    panel._emit_log("[PICK][DIRECT] AVISO: TRAMO FINAL COMPLETADO route=basket")
    panel._emit_log("[PICK][DIRECT] SECUENCIA COMPLETADA EXITOSAMENTE route=basket")
    panel._emit_log("[PICK][DEMO] boton deshabilitado (objeto confirmado en cesta)")


def _disable_button_anyway(panel) -> None:
    panel.btn_pick_demo.setEnabled(False)
    panel.btn_pick_demo.setToolTip(
        "Secuencia completada sin entrega valida en cesta"
    )
    panel._ui_set_status("Pick demo fallido: cesta no confirmada", error=True)


# F3-step1.4 + F3-step40 (2026-05-08): cadena de helpers IK seed deviation
# delegada a pick_demo.seed_metrics (puro, testeable offline).
from .pick_demo.seed_metrics import (
    seed_devs as _seed_devs,
    seed_max_dev as _seed_max_dev,
    seed_sum_dev as _seed_sum_dev,
)


# F3-step1.4: helper UI promovido del closure (4 callsites; 3 callables
# usan ``lambda: _close_only(panel)``, 1 invocación directa pasa panel).
def _close_only(panel) -> None:
    panel._command_gripper(True, log_action="PICK", force=True)


# F3-step2: _resolve_direct_execution_target promovido del closure
# _move_tcp_direct (118 LOC fuera de la fn gigante de 1284 LOC).
# Convierte un objetivo TCP en base_link al objetivo equivalente para
# tool0 (donde IK numérico de UR5 actua), aplicando:
#   * Negación X/Y por la rotación Rz(π) entre base_link y
#     base_link_inertia (frame raíz de ur_description).
#   * Compensación del offset rg2_pinch_center→tool0 vía R_tool0.
#   * Validación de "no offset env override" (kill-switches).
#   * Validación de canonicalidad del offset (delta < 2mm).
def _resolve_direct_execution_target(
    panel,
    tcp_target_base,
    tool_rot,
    *,
    tool_rot_source: str,
) -> dict:
    # DIRECT keeps rg2_pinch_center as the operational grasp semantics.
    # Numeric UR5 IK still solves in tool0, so the only allowed
    # conversion lives here as a fixed, traceable transform.
    env_xyz = str(
        os.environ.get("PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_XYZ", "") or ""
    ).strip()
    env_value = os.environ.get("PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_M", "")
    local_offset = None
    offset_source = None
    if env_xyz:
        panel._emit_log(
            "[PICK][DIRECT][OFFSET_ABORT] "
            "reason=runtime_env_offset_forbidden "
            "offset_source=env:PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_XYZ"
        )
        raise RuntimeError("direct_runtime_offset_env_forbidden_xyz")
    if local_offset is None and str(env_value).strip():
        panel._emit_log(
            "[PICK][DIRECT][OFFSET_ABORT] "
            "reason=runtime_env_offset_forbidden "
            "offset_source=env:PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_M"
        )
        raise RuntimeError("direct_runtime_offset_env_forbidden_m")
    if local_offset is None:
        pose_tool0_tcp, _pose_err = get_pose(
            DIRECT_EXECUTION_FRAME,
            DIRECT_SOURCE_FRAME,
            timeout_sec=0.25,
        )
        if pose_tool0_tcp:
            try:
                px, py, pz = pose_tool0_tcp["position"]
                local_offset = (float(px), float(py), float(pz))
                offset_source = f"tf:{DIRECT_EXECUTION_FRAME}<-{DIRECT_SOURCE_FRAME}"
            except Exception:
                local_offset = None
    if local_offset is None:
        local_offset = tuple(_DIRECT_TOOL0_TO_SOURCE_OFFSET)
        offset_source = "urdf:rg2_pinch_center_joint"
    canonical_offset = tuple(float(v) for v in _DIRECT_TOOL0_TO_SOURCE_OFFSET)
    canonical_offset_delta_m = math.sqrt(
        sum(
            (float(local_offset[idx]) - float(canonical_offset[idx])) ** 2
            for idx in range(3)
        )
    )
    if canonical_offset_delta_m > 0.002:
        panel._emit_log(
            "[PICK][DIRECT][OFFSET_ABORT] "
            f"reason=non_canonical_offset offset_source={offset_source} "
            f"offset_local={_fmt_vec(local_offset)} "
            f"canonical_offset={_fmt_vec(canonical_offset)} "
            f"offset_delta_m={canonical_offset_delta_m:.4f}/0.0020"
        )
        raise RuntimeError(
            f"direct_runtime_offset_non_canonical delta_m={canonical_offset_delta_m:.4f}"
        )
    tcp_offset_m = float(
        math.sqrt(
            (float(local_offset[0]) ** 2)
            + (float(local_offset[1]) ** 2)
            + (float(local_offset[2]) ** 2)
        )
    )
    # The UR5 URDF kinematic chain roots at base_link_inertia, which has a
    # fixed Rz(π) rotation relative to base_link (standard ur_description).
    # fk_ur5() returns positions in base_link_inertia frame.  To express a
    # runtime target given in base_link into this model frame, negate X and Y.
    # This negation is ALWAYS required and is NOT controlled by any env var.
    target_model = (
        -float(tcp_target_base[0]),
        -float(tcp_target_base[1]),
        float(tcp_target_base[2]),
    )
    offset_vector = (
        float(tool_rot[0, 0]) * float(local_offset[0])
        + float(tool_rot[0, 1]) * float(local_offset[1])
        + float(tool_rot[0, 2]) * float(local_offset[2]),
        float(tool_rot[1, 0]) * float(local_offset[0])
        + float(tool_rot[1, 1]) * float(local_offset[1])
        + float(tool_rot[1, 2]) * float(local_offset[2]),
        float(tool_rot[2, 0]) * float(local_offset[0])
        + float(tool_rot[2, 1]) * float(local_offset[1])
        + float(tool_rot[2, 2]) * float(local_offset[2]),
    )
    # operational grasp frame = tool0 + (R_tool0 * local_offset); solve tool0 target.
    execution_target_tool0 = (
        float(target_model[0]) - float(offset_vector[0]),
        float(target_model[1]) - float(offset_vector[1]),
        float(target_model[2]) - float(offset_vector[2]),
    )
    execution_target_tool0_base = (
        -float(execution_target_tool0[0]),
        -float(execution_target_tool0[1]),
        float(execution_target_tool0[2]),
    )
    return {
        "source_frame": DIRECT_SOURCE_FRAME,
        "source_pose": _tuple3(tcp_target_base),
        "target_model": _tuple3(target_model),
        "execution_frame": DIRECT_EXECUTION_FRAME,
        "execution_pose": _tuple3(execution_target_tool0),
        "execution_pose_base": _tuple3(execution_target_tool0_base),
        "offset_local": _tuple3(local_offset),
        "offset_vector": _tuple3(offset_vector),
        "offset_m": float(tcp_offset_m),
        "offset_source": str(offset_source or "unknown"),
        "offset_mode": "fixed_subtract",
        "tool_rot_source": str(tool_rot_source or "unknown"),
        "model_frame_note": "base_link_to_base_link_inertia_rz_pi",
        "ik_mode": DIRECT_EXECUTION_IK_MODE,
    }



# F5 LEGACY REMOVED (2026-05-08)
# El closure run_pick_demo (~8.080 LOC) y su helper _record_legacy_invocation
# fueron borrados en commit F5-legacy-removed-20260508. El path canónico
# es ahora el orchestrator (tfm_orchestrator/pick_orchestrator_lifecycle_node)
# vía action /pick_place. Los helpers públicos de este módulo (top-level
# arriba) siguen exportándose para consumers que los necesiten desde panel
# (ej. pick_demo_dispatcher, panel_calib_actions).
#
# Si necesitas recuperar el legacy: git checkout audit-pre-borrar-legacy-20260508
# Bug bloqueante en orchestrator: docs/BUG_CONTROLLER_FEEDBACK_HANG.md
