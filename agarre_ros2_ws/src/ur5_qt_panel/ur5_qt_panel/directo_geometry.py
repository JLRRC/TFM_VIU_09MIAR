#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/directo_geometry.py
"""Pure-geometry helpers for the DIRECTO pick-demo pipeline.

All functions in this module are:
  - Free of ROS, Qt, and panel-state dependencies.
  - Safe to import and unit-test without a running ROS node.
  - Extracted from panel_pick_demo.py to make them independently testable.
"""
from __future__ import annotations

import math
import os
from typing import Optional


# ---------------------------------------------------------------------------
# Math primitives
# ---------------------------------------------------------------------------

def angle_shortest_diff_rad(current: float, target: float) -> float:
    """Return the shortest signed angular distance between two angles."""
    delta = float(current) - float(target)
    return math.atan2(math.sin(delta), math.cos(delta))


# ---------------------------------------------------------------------------
# Data helpers
# ---------------------------------------------------------------------------

def _pick_demo_tuple3(data) -> Optional[tuple[float, float, float]]:
    if data is None:
        return None
    try:
        return (
            float(data[0]),
            float(data[1]),
            float(data[2]),
        )
    except Exception:
        return None


def _pick_demo_fmt_scalar(value, *, digits: int = 3) -> str:
    try:
        return f"{float(value):.{digits}f}"
    except Exception:
        return "none"


# ---------------------------------------------------------------------------
# Environment variable readers
# ---------------------------------------------------------------------------

def _pick_demo_env_float(
    name: str,
    default: float,
    *,
    minimum: float = 0.0,
    maximum: Optional[float] = None,
) -> float:
    try:
        value = float(os.environ.get(name, str(default)) or default)
    except Exception:
        value = float(default)
    value = max(float(minimum), float(value))
    if maximum is not None:
        value = min(float(maximum), float(value))
    return float(value)


def _pick_demo_env_int(name: str, default: int, *, minimum: int = 0) -> int:
    try:
        value = int(float(os.environ.get(name, str(default)) or default))
    except Exception:
        value = int(default)
    return max(int(minimum), int(value))


def _pick_demo_env_flag(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return bool(default)
    return str(raw).strip().lower() not in {"", "0", "false", "no", "off"}


# ---------------------------------------------------------------------------
# DIRECTO frame and target semantics
# ---------------------------------------------------------------------------

_RG2_PINCH_CENTER_FRAME = "rg2_pinch_center"


def _effective_direct_grasp_z(source_frame: str, requested_offset_m: float) -> float:
    """Return the runtime grasp Z offset for the active operational grasp frame.

    `rg2_pinch_center` is already the functional contact frame in this project, so
    carrying over the old `rg2_tcp` vertical offset would shift DIRECTO upward and
    break physical contact.
    """
    if str(source_frame or "").strip() == _RG2_PINCH_CENTER_FRAME:
        return 0.0
    return float(requested_offset_m)


def _direct_runtime_target_tol_m(label: str) -> float:
    label_name = str(label or "").strip().upper()
    if label_name in {"APPROACH_COARSE", "APPROACH_COARSE_XY_CORR", "APPROACH_COARSE_Z_CORR"}:
        return max(
            0.006,
            float(
                os.environ.get(
                    "PANEL_PICK_DEMO_APPROACH_COARSE_TCP_TOL_M",
                    "0.015",
                )
                or 0.015
            ),
        )
    if label_name == "APPROACH_COARSE_REFINE":
        return max(
            0.006,
            float(
                os.environ.get(
                    "PANEL_PICK_DEMO_APPROACH_COARSE_REFINE_TCP_TOL_M",
                    "0.006",
                )
                or 0.006
            ),
        )
    if label_name == "GRASP_DOWN_JOINT":
        return max(
            0.005,
            float(os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_TCP_TOL_M", "0.020") or 0.020),
        )
    if label_name == "GRASP_ALIGN_IK":
        return max(
            0.005,
            float(os.environ.get("PANEL_PICK_DEMO_GRASP_ALIGN_TCP_TOL_M", "0.015") or 0.015),
        )
    # CESTA_STAGE_* (transport hacia cesta) y CESTA_RELEASE: la cesta es
    # grande comparada con la pinza; 40mm de tolerancia (default heredado)
    # bloqueaba los TRANSPORT_POSTCHECK con error 49mm (ver Layer 9
    # 2026-04-27). Default elevado a 60mm; override via env si hace falta.
    if label_name.startswith("CESTA_STAGE_") or label_name == "CESTA_RELEASE":
        return max(
            0.02,
            float(
                os.environ.get(
                    "PANEL_PICK_DEMO_BASKET_TRANSPORT_TCP_TOL_M",
                    "0.060",
                )
                or 0.060
            ),
        )
    return max(
        0.01,
        float(os.environ.get("PANEL_PICK_DEMO_DIRECT_IK_TCP_TOL_M", "0.040") or 0.040),
    )


def pick_demo_target_semantics(phase_name: str) -> tuple[str, str]:
    label_name = str(phase_name or "").strip().upper()
    if label_name in {
        "APPROACH_COARSE",
        "APPROACH_COARSE_REFINE",
        "APPROACH_COARSE_XY_CORR",
        "APPROACH_COARSE_Z_CORR",
    }:
        return (
            "OBJETO_MAS_CLEARANCE",
            "APPROACH_COARSE fija XY sobre el objeto y permite clearance en Z antes del contacto.",
        )
    if label_name == "GRASP_DOWN_JOINT" or label_name.startswith("GRASP_DOWN_JOINT_"):
        return (
            "CONTACTO_GRASP",
            "GRASP_DOWN_JOINT baja hacia contacto preservando el XY validado en APPROACH_COARSE.",
        )
    if label_name == "GRASP_ALIGN_IK":
        return (
            "OBJETO_EXACTO",
            "GRASP_ALIGN_IK corrige la alineacion fina sobre la referencia de grasp del objeto.",
        )
    if label_name in {"PRE_CLOSE", "CLOSE", "ATTACH_GATE"}:
        return (
            "EXEC_REAL",
            "La fase depende de la ejecucion real del cierre y del attach, no de un target geometrico nuevo.",
        )
    if label_name in {"HOME_INITIAL", "MESA_READY", "INICIO"}:
        return (
            "CACHE",
            "Fila preparatoria o de control; no representa un target de contacto sobre el objeto.",
        )
    return (
        "CACHE",
        "Semantica de target no clasificada para esta fase.",
    )


def _is_demo_basket_transport_stage(label: str) -> bool:
    return str(label or "").strip().upper().startswith("CESTA_STAGE_")


def _is_demo_basket_transport_motion(label: str) -> bool:
    label_name = str(label or "").strip().upper()
    return _is_demo_basket_transport_stage(label_name) or label_name == "CESTA_RELEASE"


# ---------------------------------------------------------------------------
# Transport geometry
# ---------------------------------------------------------------------------

def _compute_demo_basket_targets(
    basket_base,
    *,
    transport_z_offset: float,
    release_z_offset: float,
) -> dict:
    basket_base_3 = _pick_demo_tuple3(basket_base)
    if basket_base_3 is None:
        raise ValueError("basket_base_unavailable")
    transport_target = (
        float(basket_base_3[0]),
        float(basket_base_3[1]),
        float(basket_base_3[2]) + float(transport_z_offset),
    )
    release_target = (
        float(basket_base_3[0]),
        float(basket_base_3[1]),
        float(basket_base_3[2]) + float(release_z_offset),
    )
    return {
        "basket_base": basket_base_3,
        "transport_target_base": transport_target,
        "release_target_base": release_target,
    }


def _compute_demo_linear_stage_targets(
    start_base,
    end_base,
    *,
    stages: int,
) -> list[tuple[float, float, float]]:
    start_base_3 = _pick_demo_tuple3(start_base)
    end_base_3 = _pick_demo_tuple3(end_base)
    if start_base_3 is None or end_base_3 is None:
        raise ValueError("stage_target_unavailable")
    total_stages = max(1, int(stages))
    targets = []
    for stage_idx in range(1, total_stages + 1):
        frac = float(stage_idx) / float(total_stages)
        targets.append(
            (
                float(start_base_3[0]) + (float(end_base_3[0]) - float(start_base_3[0])) * frac,
                float(start_base_3[1]) + (float(end_base_3[1]) - float(start_base_3[1])) * frac,
                float(start_base_3[2]) + (float(end_base_3[2]) - float(start_base_3[2])) * frac,
            )
        )
    return targets


def _compute_demo_stage_count_for_distance(
    start_base,
    end_base,
    *,
    min_stages: int,
    max_stage_dist_m: float,
    max_stages: int,
) -> int:
    start_base_3 = _pick_demo_tuple3(start_base)
    end_base_3 = _pick_demo_tuple3(end_base)
    if start_base_3 is None or end_base_3 is None:
        raise ValueError("stage_count_target_unavailable")
    total_dist_m = math.sqrt(
        (float(end_base_3[0]) - float(start_base_3[0])) ** 2
        + (float(end_base_3[1]) - float(start_base_3[1])) ** 2
        + (float(end_base_3[2]) - float(start_base_3[2])) ** 2
    )
    safe_max_stage_dist_m = max(0.01, float(max_stage_dist_m))
    requested_min = max(1, int(min_stages))
    requested_max = max(requested_min, int(max_stages))
    adaptive_count = max(1, int(math.ceil(total_dist_m / safe_max_stage_dist_m)))
    return min(requested_max, max(requested_min, adaptive_count))


def _compute_demo_transport_recovery_stage_targets(
    current_tcp_base,
    target_tcp_base,
    *,
    min_remaining_dist_m: float,
    min_stages: int,
    max_stage_dist_m: float,
    max_stages: int,
) -> list[tuple[float, float, float]]:
    current_tcp_base_3 = _pick_demo_tuple3(current_tcp_base)
    target_tcp_base_3 = _pick_demo_tuple3(target_tcp_base)
    if current_tcp_base_3 is None or target_tcp_base_3 is None:
        raise ValueError("transport_recovery_target_unavailable")
    remaining_dist_m = math.sqrt(
        (float(target_tcp_base_3[0]) - float(current_tcp_base_3[0])) ** 2
        + (float(target_tcp_base_3[1]) - float(current_tcp_base_3[1])) ** 2
        + (float(target_tcp_base_3[2]) - float(current_tcp_base_3[2])) ** 2
    )
    if remaining_dist_m < max(0.0, float(min_remaining_dist_m)):
        return []
    recovery_stage_count = _compute_demo_stage_count_for_distance(
        current_tcp_base_3,
        target_tcp_base_3,
        min_stages=max(1, int(min_stages)),
        max_stage_dist_m=max_stage_dist_m,
        max_stages=max(max(1, int(min_stages)), int(max_stages)),
    )
    if recovery_stage_count <= 1:
        return []
    return _compute_demo_linear_stage_targets(
        current_tcp_base_3,
        target_tcp_base_3,
        stages=recovery_stage_count,
    )


def _compute_demo_transport_micro_recovery_target(
    current_tcp_base,
    target_tcp_base,
    *,
    step_m: float,
    minimum_remaining_dist_m: float,
) -> Optional[tuple[float, float, float]]:
    current_tcp_base_3 = _pick_demo_tuple3(current_tcp_base)
    target_tcp_base_3 = _pick_demo_tuple3(target_tcp_base)
    if current_tcp_base_3 is None or target_tcp_base_3 is None:
        raise ValueError("transport_micro_recovery_target_unavailable")
    remaining_dist_m = math.sqrt(
        (float(target_tcp_base_3[0]) - float(current_tcp_base_3[0])) ** 2
        + (float(target_tcp_base_3[1]) - float(current_tcp_base_3[1])) ** 2
        + (float(target_tcp_base_3[2]) - float(current_tcp_base_3[2])) ** 2
    )
    safe_step_m = max(0.001, float(step_m))
    safe_min_remaining_dist_m = max(0.0, float(minimum_remaining_dist_m))
    if remaining_dist_m <= safe_step_m + safe_min_remaining_dist_m:
        return None
    scale = safe_step_m / remaining_dist_m
    return (
        float(current_tcp_base_3[0])
        + (float(target_tcp_base_3[0]) - float(current_tcp_base_3[0])) * scale,
        float(current_tcp_base_3[1])
        + (float(target_tcp_base_3[1]) - float(current_tcp_base_3[1])) * scale,
        float(current_tcp_base_3[2])
        + (float(target_tcp_base_3[2]) - float(current_tcp_base_3[2])) * scale,
    )


# ---------------------------------------------------------------------------
# Joint-space geometry
# ---------------------------------------------------------------------------

def _compute_demo_joint_prep_waypoint(
    seed_joints,
    target_joints,
    *,
    blend: float,
) -> list[float]:
    seed_list = [float(v) for v in seed_joints]
    target_list = [float(v) for v in target_joints]
    if len(seed_list) != len(target_list):
        raise ValueError("joint_prep_length_mismatch")
    safe_blend = min(0.95, max(0.05, float(blend)))
    return [
        float(seed_q) - safe_blend * float(angle_shortest_diff_rad(float(seed_q), float(target_q)))
        for seed_q, target_q in zip(seed_list, target_list)
    ]


def _compute_demo_joint_prep_waypoints(
    seed_joints,
    target_joints,
    *,
    blend: float,
    max_joint_delta_rad: float,
    max_sum_delta_rad: float,
    max_steps: int,
    max_shoulder_delta_rad: Optional[float] = None,
) -> list[list[float]]:
    seed_list = [float(v) for v in seed_joints]
    target_list = [float(v) for v in target_joints]
    if len(seed_list) != len(target_list):
        raise ValueError("joint_prep_length_mismatch")
    if not seed_list:
        return []
    safe_blend = min(0.95, max(0.05, float(blend)))
    safe_max_joint_delta_rad = max(0.05, float(max_joint_delta_rad))
    safe_max_sum_delta_rad = max(safe_max_joint_delta_rad, float(max_sum_delta_rad))
    safe_max_steps = max(1, int(max_steps))
    safe_max_shoulder_delta_rad = None
    if max_shoulder_delta_rad is not None:
        safe_max_shoulder_delta_rad = max(0.03, float(max_shoulder_delta_rad))
    prep_deltas = [
        abs(float(angle_shortest_diff_rad(float(seed_q), float(target_q))))
        for seed_q, target_q in zip(seed_list, target_list)
    ]
    max_delta = max(prep_deltas) if prep_deltas else 0.0
    sum_delta = sum(prep_deltas)
    shoulder_delta = 0.0
    if prep_deltas:
        shoulder_indices = (0, 1)
        shoulder_deltas = [
            float(prep_deltas[idx])
            for idx in shoulder_indices
            if 0 <= idx < len(prep_deltas)
        ]
        shoulder_delta = max(shoulder_deltas) if shoulder_deltas else 0.0

    def _segment_count_for_limit(value: float, limit: float) -> int:
        if value <= 0.0:
            return 1
        ratio = float(value) / max(1e-9, float(limit))
        return max(1, int(math.ceil(max(0.0, ratio - 1e-9))))

    segment_count = max(
        1,
        _segment_count_for_limit(max_delta, safe_max_joint_delta_rad),
        _segment_count_for_limit(sum_delta, safe_max_sum_delta_rad),
        int(math.ceil(1.0 / safe_blend)),
    )
    if safe_max_shoulder_delta_rad is not None:
        segment_count = max(
            segment_count,
            _segment_count_for_limit(shoulder_delta, safe_max_shoulder_delta_rad),
        )
    segment_count = min(safe_max_steps, segment_count)
    if segment_count <= 1:
        return []
    return [
        _compute_demo_joint_prep_waypoint(
            seed_list,
            target_list,
            blend=float(step_idx) / float(segment_count),
        )
        for step_idx in range(1, segment_count)
    ]


def _compute_demo_transport_prep_joint_tol(
    start_joints,
    target_joints,
    *,
    configured_tol_rad: float,
    minimum_tol_rad: float = 0.02,
    max_fraction: float = 0.45,
) -> float:
    start_list = [float(v) for v in start_joints]
    target_list = [float(v) for v in target_joints]
    if len(start_list) != len(target_list):
        raise ValueError("joint_prep_tol_length_mismatch")
    if not start_list:
        return max(float(minimum_tol_rad), float(configured_tol_rad))
    safe_configured_tol = max(float(minimum_tol_rad), float(configured_tol_rad))
    safe_minimum_tol = max(0.01, float(minimum_tol_rad))
    safe_fraction = min(0.90, max(0.10, float(max_fraction)))
    max_delta = max(
        abs(float(angle_shortest_diff_rad(float(start_q), float(target_q))))
        for start_q, target_q in zip(start_list, target_list)
    )
    dynamic_tol = max(safe_minimum_tol, float(max_delta) * safe_fraction)
    return min(safe_configured_tol, dynamic_tol)


def _joint_step_wait_timeout(
    timeout_sec: Optional[float],
    *,
    effective_move_sec: float,
    step_timeout_extra_sec: float,
    apply_step_timeout_extra: bool = True,
) -> float:
    wait_timeout = (
        float(effective_move_sec) + 2.0
        if timeout_sec is None
        else float(timeout_sec)
    )
    if apply_step_timeout_extra:
        wait_timeout += max(0.0, float(step_timeout_extra_sec))
    return float(wait_timeout)
