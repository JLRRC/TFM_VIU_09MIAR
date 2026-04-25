#!/usr/bin/env python3
"""Runtime audit helpers for the STEP by STEP panel."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from .panel_config import GRIPPER_CLOSED_RAD, GRIPPER_OPEN_RAD, POSE_INFO_MAX_AGE_SEC
from .panel_pick_demo import pick_demo_target_semantics
from .panel_utils import get_tf_helper, world_to_base

Vec3 = Tuple[float, float, float]

_STATUS_RANK = {
    "OK": 0,
    "AVISO": 1,
    "STALE": 2,
    "ERROR": 3,
}


@dataclass(frozen=True)
class RuntimeAuditBlock:
    title: str
    planned_lines: List[str]
    runtime_lines: List[str]
    status: str
    summary: str


@dataclass(frozen=True)
class RuntimeAuditSnapshot:
    blocks: Dict[str, RuntimeAuditBlock]
    metrics: Dict[str, object]


def runtime_status_style(status: str) -> str:
    normalized = str(status or "STALE").strip().upper()
    if normalized == "OK":
        return "background:#dcfce7; color:#166534; border-radius:10px; padding:3px 10px; font-weight:700;"
    if normalized == "AVISO":
        return "background:#fef3c7; color:#9a3412; border-radius:10px; padding:3px 10px; font-weight:700;"
    if normalized == "ERROR":
        return "background:#fee2e2; color:#991b1b; border-radius:10px; padding:3px 10px; font-weight:700;"
    return "background:#e2e8f0; color:#475569; border-radius:10px; padding:3px 10px; font-weight:700;"


def compute_step_history_metrics(panel: Any, row: Dict[str, object]) -> Dict[str, object]:
    target_base = _vec3(row.get("target"))
    actual_base = _vec3(row.get("actual"))
    exec_base = _vec3(row.get("exec_target_snapshot"))
    obj_world = _vec3(row.get("object_world_snapshot"))
    obj_base = _world_to_base(obj_world)
    phase_name = str(row.get("phase") or "").strip().upper()
    target_kind, target_note = pick_demo_target_semantics(phase_name)
    dist_tcp_obj = _distance(actual_base, obj_base)
    dist_target_obj = _distance(target_base, obj_base)
    err_tcp_exec = _distance(actual_base, exec_base)
    return {
        "dist_tcp_obj": dist_tcp_obj,
        "dist_target_obj": dist_target_obj,
        "err_tcp_exec": err_tcp_exec,
        "target_kind": target_kind,
        "target_note": target_note,
    }


def build_runtime_audit_snapshot(panel: Any) -> RuntimeAuditSnapshot:
    current_phase = str(getattr(panel, "_step_current_phase", "") or "").strip().upper()
    op_frame = str(panel._step_operational_frame_name() or "rg2_pinch_center").strip() or "rg2_pinch_center"
    target_kind, target_note = pick_demo_target_semantics(current_phase)

    panel_tcp_world = _base_pose_to_world(panel, panel._step_fetch_live_pose(op_frame))
    tf_tcp_world = _base_pose_to_world(panel, panel._step_fetch_live_pose("rg2_pinch_center"))
    tool0_world = _base_pose_to_world(panel, panel._step_fetch_live_pose("tool0"))
    rg2_base_world = _base_pose_to_world(panel, panel._step_fetch_live_pose("rg2_hand"))
    rg2_hand_world = _base_pose_to_world(panel, panel._step_fetch_live_pose("rg2_hand"))
    rg2_left_world = _base_pose_to_world(panel, panel._step_fetch_live_pose("rg2_leftfinger"))
    rg2_right_world = _base_pose_to_world(panel, panel._step_fetch_live_pose("rg2_rightfinger"))
    base_link_world = _base_pose_to_world(panel, (0.0, 0.0, 0.0))

    target_world = _base_pose_to_world(panel, getattr(panel, "_step_phase_position", None))
    object_panel_world = _base_pose_to_world(panel, getattr(panel, "_step_object_position", None))
    object_gazebo_world = _vec3(panel._step_fetch_object_world())
    exec_world = _base_pose_to_world(panel, _current_exec_target(panel, current_phase))

    tcp_panel_vs_tf = _delta(panel_tcp_world, tf_tcp_world)
    tcp_vs_target = _delta(tf_tcp_world, target_world)
    tcp_vs_object = _delta(tf_tcp_world, object_gazebo_world or object_panel_world)
    object_panel_vs_gazebo = _delta(object_panel_world, object_gazebo_world)
    target_vs_object = _delta(target_world, object_gazebo_world or object_panel_world)
    tool0_vs_tcp = _delta(tool0_world, tf_tcp_world)
    rg2_base_vs_tcp = _delta(rg2_base_world, tf_tcp_world)
    base_link_expected = (-0.85, 0.0, 0.85)
    base_link_vs_expected = _delta(base_link_world, base_link_expected)

    tf_helper = get_tf_helper()
    tf_stats = tf_helper.tf_listener_stats() if tf_helper is not None else (0, 0)

    joint_state = panel._read_gripper_feedback_state()
    joint_positions = dict(joint_state.get("joint_positions") or {})
    joint_age = _safe_float(joint_state.get("joint_state_age_sec"))
    controller_map = dict(getattr(panel, "_controller_state_map", {}) or {})
    controller_age = _age_from_wall(getattr(panel, "_controller_state_ts", 0.0))

    pose_count = 0
    pose_age = None
    pose_entities = 0
    pose_topic = ""
    ros_worker = getattr(panel, "ros_worker", None)
    if getattr(panel, "_ros_worker_started", False) and ros_worker is not None:
        try:
            pose_count, pose_age, pose_entities, pose_topic = ros_worker.pose_info_details()
        except Exception:
            pose_count, pose_age, pose_entities, pose_topic = 0, None, 0, ""

    dh_tf_status = "OK"
    dh_tf_summary = "TCP operacional y frame global coherentes."
    if op_frame != "rg2_pinch_center":
        dh_tf_status = _worse_status(dh_tf_status, "ERROR")
        dh_tf_summary = f"Frame operacional inesperado: {op_frame}."
    if tf_tcp_world is None:
        dh_tf_status = _worse_status(dh_tf_status, "STALE")
        dh_tf_summary = "No hay TF live para rg2_pinch_center."
    if _classify_pose_delta(tcp_panel_vs_tf) in {"AVISO", "ERROR"}:
        dh_tf_status = _worse_status(dh_tf_status, _classify_pose_delta(tcp_panel_vs_tf))
        dh_tf_summary = "El panel no coincide con el TCP real de TF."
    if _classify_base_delta(base_link_vs_expected) == "ERROR":
        dh_tf_status = _worse_status(dh_tf_status, "ERROR")
        dh_tf_summary = "world -> base_link no coincide con el anclaje esperado."

    joints_status = "OK"
    joints_summary = "Joint states y controladores listos."
    inferred_state = _interpret_gripper_state(joint_positions, joint_state.get("inferred_state"))
    if not joint_positions:
        joints_status = _worse_status(joints_status, "ERROR")
        joints_summary = "No hay joints del RG2 en joint_states."
    elif joint_age is not None and joint_age > 0.5:
        joints_status = _worse_status(joints_status, "STALE")
        joints_summary = f"joint_states stale ({joint_age:.2f}s)."
    elif inferred_state == "INTERMEDIA":
        joints_status = _worse_status(joints_status, "AVISO")
        joints_summary = "La apertura del RG2 es intermedia o ambigua."
    controller_states = {
        "joint_state_broadcaster": _controller_state(controller_map, "joint_state_broadcaster"),
        "joint_trajectory_controller": _controller_state(controller_map, "joint_trajectory_controller"),
        "gripper_controller": _controller_state(controller_map, "gripper_controller"),
    }
    for state in controller_states.values():
        if state and str(state).strip().lower() != "active":
            joints_status = _worse_status(joints_status, "ERROR")
            joints_summary = "Hay controladores requeridos sin estado active."
    if not any(controller_states.values()) and bool(getattr(panel, "_controllers_ok", False)) is False:
        joints_status = _worse_status(joints_status, "STALE")
        joints_summary = str(getattr(panel, "_controllers_reason", "controladores sin diagnostico") or "controladores sin diagnostico")

    gazebo_status = "OK"
    gazebo_summary = "Objeto y pose/info coherentes con el panel."
    if pose_count <= 0:
        gazebo_status = _worse_status(gazebo_status, "ERROR")
        gazebo_summary = "No llegan mensajes de pose/info."
    elif pose_age is None or not math.isfinite(float(pose_age)):
        gazebo_status = _worse_status(gazebo_status, "STALE")
        gazebo_summary = "Edad de pose/info no disponible."
    elif float(pose_age) > float(POSE_INFO_MAX_AGE_SEC):
        gazebo_status = _worse_status(gazebo_status, "STALE")
        gazebo_summary = f"pose/info stale ({float(pose_age):.2f}s)."
    object_delta_status = _classify_object_delta(object_panel_vs_gazebo)
    if object_delta_status in {"AVISO", "ERROR"}:
        gazebo_status = _worse_status(gazebo_status, object_delta_status)
        gazebo_summary = "La pose del objeto en panel no coincide con Gazebo/pose_info."
    target_xy_delta = _xy_mm(target_vs_object)
    if target_kind == "OBJETO_MAS_CLEARANCE":
        if target_xy_delta is not None and target_xy_delta > 20.0:
            gazebo_status = _worse_status(gazebo_status, "ERROR")
            gazebo_summary = "APPROACH_COARSE pierde XY respecto al objeto."
    elif target_kind == "CONTACTO_GRASP":
        target_dist_mm = _dist_mm(target_vs_object)
        if target_dist_mm is not None and target_dist_mm > 15.0:
            gazebo_status = _worse_status(gazebo_status, "AVISO")
            gazebo_summary = "GRASP_DOWN_JOINT aun no cae cerca del objeto/contacto."

    pose_info_state = "ok"
    if pose_count <= 0:
        pose_info_state = "sin_dato"
    elif pose_age is None or not math.isfinite(float(pose_age)):
        pose_info_state = "sin_edad"
    elif float(pose_age) > float(POSE_INFO_MAX_AGE_SEC):
        pose_info_state = "stale"

    dh_tf_planned = [
        f"[URDF] Frame operacional esperado: rg2_pinch_center",
        f"[PANEL] TCP panel world: {_fmt_vec(panel_tcp_world)}",
        f"[PANEL] Target fase world: {_fmt_vec(target_world)}",
        f"[PANEL] Objeto panel world: {_fmt_vec(object_panel_world)}",
        f"[SEM] Tipo target: {target_kind}",
    ]
    dh_tf_runtime = [
        f"[TF] TCP world -> rg2_pinch_center: {_fmt_vec(tf_tcp_world)}",
        f"[TF] tool0 world: {_fmt_vec(tool0_world)}",
        f"[TF] rg2_hand world: {_fmt_vec(rg2_base_world)}",
        f"[TF] base_link world: {_fmt_vec(base_link_world)}",
        f"[CHECK] error TCP panel vs TF: {_fmt_delta(tcp_panel_vs_tf)}",
        f"[CHECK] error TCP vs target: {_fmt_delta(tcp_vs_target)}{_pregrasp_note(current_phase, tcp_vs_target)}",
        f"[CHECK] error TCP vs objeto: {_fmt_delta(tcp_vs_object)}{_pregrasp_note(current_phase, tcp_vs_object)}",
        f"[TF] Frescura: tf_msgs={int(tf_stats[0])} tf_static={int(tf_stats[1])}",
    ]

    joints_planned = [
        f"[PHASE] Pinza esperada por fase: {panel._step_phase_gripper_state(panel._step_effective_flow(), current_phase)}",
        f"[PANEL] Pinza live panel: {panel._step_live_gripper_state()}",
        f"[CFG] Umbrales cfg: open={float(GRIPPER_OPEN_RAD):.4f} m | closed={float(GRIPPER_CLOSED_RAD):.4f} m",
    ]
    joints_runtime = [
        f"[JOINTS] rg2_finger_joint1: {_fmt_scalar(joint_positions.get('rg2_finger_joint1'), unit=' m')}",
        f"[JOINTS] rg2_finger_joint2: {_fmt_scalar(joint_positions.get('rg2_finger_joint2'), unit=' m')}",
        f"[JOINTS] Apertura interpretada: {inferred_state}",
        f"[JOINTS] Edad joint_states: {_fmt_age(joint_age)}",
        f"[CTRL] joint_state_broadcaster: {controller_states['joint_state_broadcaster'] or 'sin dato'}",
        f"[CTRL] joint_trajectory_controller: {controller_states['joint_trajectory_controller'] or 'sin dato'}",
        f"[CTRL] gripper_controller: {controller_states['gripper_controller'] or 'sin dato'}",
        f"[CTRL] Edad cache controladores: {_fmt_age(controller_age)} | fuente={getattr(panel, '_controller_state_source', '') or 'sin dato'}",
    ]

    gazebo_planned = [
        f"[PANEL] Objeto panel world: {_fmt_vec(object_panel_world)}",
        f"[PANEL] Target fase world: {_fmt_vec(target_world)}",
        f"[PANEL] Exec enviado: {_fmt_vec(exec_world)}",
        f"[SEM] {target_kind}: {target_note}",
    ]
    gazebo_runtime = [
        f"[GAZEBO] Objeto pose/info: {_fmt_vec(object_gazebo_world)}",
        f"[CHECK] error objeto panel vs Gazebo: {_fmt_delta(object_panel_vs_gazebo)}",
        f"[CHECK] error target vs objeto: {_fmt_delta(target_vs_object)}",
        f"[SDF/TF] rg2_hand: {_fmt_optional_semantic_frame(rg2_hand_world)}",
        f"[SDF/TF] rg2_leftfinger: {_fmt_optional_semantic_frame(rg2_left_world)}",
        f"[SDF/TF] rg2_rightfinger: {_fmt_optional_semantic_frame(rg2_right_world)}",
        f"[GAZEBO] pose/info: topic={pose_topic or 'sin dato'} age={_fmt_age(pose_age)} entities={int(pose_entities)} state={pose_info_state}",
    ]

    return RuntimeAuditSnapshot(
        blocks={
            "dh_tf": RuntimeAuditBlock(
                title="BLOQUE 1 - DH / TF",
                planned_lines=dh_tf_planned,
                runtime_lines=dh_tf_runtime,
                status=dh_tf_status,
                summary=dh_tf_summary,
            ),
            "joints_control": RuntimeAuditBlock(
                title="BLOQUE 2 - JOINTS / CONTROL",
                planned_lines=joints_planned,
                runtime_lines=joints_runtime,
                status=joints_status,
                summary=joints_summary,
            ),
            "sdf_gazebo": RuntimeAuditBlock(
                title="BLOQUE 3 - SDF / GAZEBO",
                planned_lines=gazebo_planned,
                runtime_lines=gazebo_runtime,
                status=gazebo_status,
                summary=gazebo_summary,
            ),
        },
        metrics={
            "tcp_panel_world": panel_tcp_world,
            "tcp_tf_world": tf_tcp_world,
            "tool0_world": tool0_world,
            "rg2_base_world": rg2_base_world,
            "object_panel_world": object_panel_world,
            "object_gazebo_world": object_gazebo_world,
            "target_world": target_world,
            "exec_world": exec_world,
            "target_kind": target_kind,
            "target_note": target_note,
            "tool0_vs_tcp": tool0_vs_tcp,
            "rg2_base_vs_tcp": rg2_base_vs_tcp,
            "base_link_world": base_link_world,
        },
    )


def _base_pose_to_world(panel: Any, base_pose: object) -> Optional[Vec3]:
    pose = _vec3(base_pose)
    if pose is None:
        return None
    try:
        world_pose = panel._step_display_position(pose)
    except Exception:
        world_pose = None
    return _vec3(world_pose)


def _current_exec_target(panel: Any, current_phase: str) -> Optional[Vec3]:
    rows = list(getattr(panel, "_step_history_rows", []) or [])
    for row in reversed(rows):
        if current_phase and str(row.get("phase") or "").strip().upper() != current_phase:
            continue
        return _vec3(row.get("exec_target_snapshot"))
    if rows:
        return _vec3(rows[-1].get("exec_target_snapshot"))
    return None


def _controller_state(state_map: Dict[str, str], target_name: str) -> str:
    target = str(target_name or "").strip().lstrip("/")
    for key, value in state_map.items():
        normalized = str(key or "").strip().lstrip("/")
        if normalized == target or normalized.endswith("/" + target):
            return str(value or "")
    return ""


def _vec3(value: object) -> Optional[Vec3]:
    if not isinstance(value, (list, tuple)) or len(value) < 3:
        return None
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except Exception:
        return None


def _world_to_base(world_pose: Optional[Vec3]) -> Optional[Vec3]:
    if world_pose is None:
        return None
    try:
        return world_to_base(float(world_pose[0]), float(world_pose[1]), float(world_pose[2]))
    except Exception:
        return None


def _delta(lhs: Optional[Vec3], rhs: Optional[Vec3]) -> Optional[Dict[str, float]]:
    if lhs is None or rhs is None:
        return None
    dx = float(lhs[0]) - float(rhs[0])
    dy = float(lhs[1]) - float(rhs[1])
    dz = float(lhs[2]) - float(rhs[2])
    xy = math.hypot(dx, dy)
    dist = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
    return {
        "dx": dx,
        "dy": dy,
        "dz": dz,
        "xy": xy,
        "dist": dist,
    }


def _distance(lhs: Optional[Vec3], rhs: Optional[Vec3]) -> Optional[float]:
    delta = _delta(lhs, rhs)
    if delta is None:
        return None
    return float(delta["dist"])


def _classify_pose_delta(delta: Optional[Dict[str, float]]) -> str:
    if delta is None:
        return "STALE"
    dist = float(delta["dist"])
    if dist <= 0.005:
        return "OK"
    if dist <= 0.015:
        return "AVISO"
    return "ERROR"


def _classify_object_delta(delta: Optional[Dict[str, float]]) -> str:
    if delta is None:
        return "STALE"
    dist = float(delta["dist"])
    if dist <= 0.005:
        return "OK"
    if dist <= 0.010:
        return "AVISO"
    return "ERROR"


def _classify_base_delta(delta: Optional[Dict[str, float]]) -> str:
    if delta is None:
        return "ERROR"
    dist = float(delta["dist"])
    if dist <= 0.010:
        return "OK"
    return "ERROR"


def _worse_status(lhs: str, rhs: str) -> str:
    return lhs if _STATUS_RANK.get(lhs, 2) >= _STATUS_RANK.get(rhs, 2) else rhs


def _fmt_vec(value: Optional[Vec3]) -> str:
    if value is None:
        return "sin dato"
    return f"({float(value[0]):.3f}, {float(value[1]):.3f}, {float(value[2]):.3f}) m"


def _fmt_scalar(value: object, *, unit: str = "") -> str:
    try:
        return f"{float(value):.3f}{unit}"
    except Exception:
        return "sin dato"


def _fmt_age(value: object) -> str:
    try:
        age = float(value)
    except Exception:
        return "sin dato"
    if not math.isfinite(age):
        return "sin dato"
    return f"{age:.2f}s"


def _fmt_delta(delta: Optional[Dict[str, float]]) -> str:
    if delta is None:
        return "sin dato"
    dx_mm = float(delta["dx"]) * 1000.0
    dy_mm = float(delta["dy"]) * 1000.0
    dz_mm = float(delta["dz"]) * 1000.0
    xy_mm = float(delta["xy"]) * 1000.0
    dist_mm = float(delta["dist"]) * 1000.0
    return (
        f"dx={dx_mm:.1f} dy={dy_mm:.1f} dz={dz_mm:.1f} mm | "
        f"XY={xy_mm:.1f} mm | 3D={dist_mm:.1f} mm ({float(delta['dist']):.3f} m)"
    )


def _fmt_optional_semantic_frame(value: Optional[Vec3]) -> str:
    if value is None:
        return "sin dato (solo SDF visual o frame no publicado en TF)"
    return _fmt_vec(value)


def _pregrasp_note(current_phase: str, delta: Optional[Dict[str, float]]) -> str:
    if delta is None:
        return ""
    dz_mm = abs(float(delta["dz"]) * 1000.0)
    if str(current_phase or "").strip().upper() == "GRASP_DOWN_JOINT" and 20.0 <= dz_mm <= 50.0:
        return " | margen de aproximacion pre-grasp"
    return ""


def _interpret_gripper_state(positions: Dict[str, float], inferred_state: object) -> str:
    state_name = str(inferred_state or "").strip().lower()
    if state_name == "open":
        return "ABIERTA"
    if state_name == "closed":
        return "CERRADA"
    if not positions:
        return "SIN_DATO"
    mags = [abs(float(value)) for value in positions.values()]
    avg = sum(mags) / max(1, len(mags))
    _open_m = abs(float(GRIPPER_OPEN_RAD))
    _closed_m = abs(float(GRIPPER_CLOSED_RAD))
    midpoint = (_open_m + _closed_m) / 2.0
    # Hysteresis proportional to range so prismatic [0,0.055] and revolute work correctly.
    _hyst = max(0.005, 0.15 * abs(_open_m - _closed_m))
    if avg > midpoint + _hyst:
        return "ABIERTA"
    if avg < midpoint - _hyst:
        return "CERRADA"
    return "INTERMEDIA"


def _safe_float(value: object) -> Optional[float]:
    try:
        parsed = float(value)
    except Exception:
        return None
    if not math.isfinite(parsed):
        return None
    return parsed


def _age_from_wall(wall_ts: object) -> Optional[float]:
    try:
        ts = float(wall_ts)
    except Exception:
        return None
    if ts <= 0.0:
        return None
    return max(0.0, time.time() - ts)


def _xy_mm(delta: Optional[Dict[str, float]]) -> Optional[float]:
    if delta is None:
        return None
    return float(delta["xy"]) * 1000.0


def _dist_mm(delta: Optional[Dict[str, float]]) -> Optional[float]:
    if delta is None:
        return None
    return float(delta["dist"]) * 1000.0