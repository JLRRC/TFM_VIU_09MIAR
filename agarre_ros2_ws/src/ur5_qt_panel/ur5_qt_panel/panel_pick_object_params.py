#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_object_params.py
# Contenido: F2 — parámetros runtime del flujo pick_object (botón "Agarre objeto").
# Uso breve:
#   from .panel_pick_object_params import get_pick_object_params
#   params = get_pick_object_params()
#   tol = params.approach_tol_m
"""Parámetros runtime del flujo pick_object (F2).

Hermano de ``panel_pick_demo_params``. Cubre los 102 tunables estáticos
del botón "Agarre objeto" (panel_pick_object.py). Misma prioridad de
resolución y mismas reglas:

    env var (PANEL_PICK_OBJECT_*)  >  YAML override  >  default del dataclass

Para tunear sin perder reproducibilidad:
1. Editar ``agarre_ros2_ws/src/ur5_qt_panel/config/pick_object_runtime.yaml``
2. (Alternativa rápida) ``export PANEL_PICK_OBJECT_<NAME>=value`` y relanzar

F2-step1 (2026-05-02) — añadidos 11 campos para reducir 28→14 reads en
panel_pick_object.py: grasp_step_tol_m unificado a 0.040 (la divergencia
0.04/0.040/0.045 se preserva via overrides Optional), tolerancias de
GRASP_DOWN/GRASP_TF_STABLE/GRASP_Z_REACH segregadas, attach_xy/z_tol_m
desambiguados (prefijo PANEL_ATTACH_*), pre_grasp_recenter_tol_m_override
y pre_grasp_abort_recovery_tol_m_override como Optional con fallback al
campo base pre_grasp_tol_m.

NO migrados (fuera de scope F2-step1):
- 6 vars con default dinámico que depende de ``moveit_exclusive``
  (TRANSPORT_JOINT_FALLBACK, APPROACH_JOINT_FALLBACK, etc.).
- ``PANEL_STRICT_PHYSICS_MODE``: cross-cutting global.
- ``PANEL_MOVEIT_BRIDGE_*``, ``PANEL_TFM_CANONICAL_*``: dataclasses propios.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, Dict, Optional

try:
    import yaml  # type: ignore
except ImportError:
    yaml = None  # type: ignore


def _resolve_default_yaml_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = Path(get_package_share_directory("ur5_qt_panel"))
        candidate = share_dir / "config" / "pick_object_runtime.yaml"
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    return (
        Path(__file__).resolve().parent.parent
        / "config"
        / "pick_object_runtime.yaml"
    )


_DEFAULT_YAML_PATH = _resolve_default_yaml_path()


@dataclass(frozen=True)
class PickObjectParams:
    """102 parámetros estáticos del flujo pick_object."""
    adapt_grasp_xy: bool = True  # PANEL_PICK_OBJECT_ADAPT_GRASP_XY
    adapt_grasp_xy_max_delta_m: float = 0.015  # PANEL_PICK_OBJECT_ADAPT_GRASP_XY_MAX_DELTA_M
    allow_degraded_open_guard: bool = True  # PANEL_PICK_OBJECT_ALLOW_DEGRADED_OPEN_GUARD
    allow_joint_preflight_in_moveit: bool = True  # PANEL_PICK_OBJECT_ALLOW_JOINT_PREFLIGHT_IN_MOVEIT
    allow_joint_recovery_in_moveit: bool = True  # PANEL_PICK_OBJECT_ALLOW_JOINT_RECOVERY_IN_MOVEIT
    allow_tf_helper_fallback: bool = False  # PANEL_PICK_OBJECT_ALLOW_TF_HELPER_FALLBACK
    approach_clearance_m: float = 0.2  # PANEL_PICK_OBJECT_APPROACH_CLEARANCE_M
    approach_skip_retry_on_fallback: bool = True  # PANEL_PICK_OBJECT_APPROACH_SKIP_RETRY_ON_FALLBACK
    approach_tol_m: float = 0.1  # PANEL_PICK_OBJECT_APPROACH_TOL_M
    # F2-step1: PANEL_ATTACH_* (prefijo distinto, son cross-cutting de attach
    # gate del flujo pick_object — viven aquí porque sólo los lee este flujo).
    attach_xy_tol_m: float = 0.02  # PANEL_ATTACH_XY_TOL_M
    attach_z_clearance_m: float = 0.0  # PANEL_PICK_OBJECT_ATTACH_Z_CLEARANCE_M
    attach_z_ref_mode: str = 'top'  # PANEL_PICK_OBJECT_ATTACH_Z_REF_MODE
    attach_z_tol_m: float = 0.04  # PANEL_ATTACH_Z_TOL_M
    carry_gate_enable: bool = True  # PANEL_PICK_OBJECT_CARRY_GATE_ENABLE
    carry_gate_max_dist_m: float = 0.18  # PANEL_PICK_OBJECT_CARRY_GATE_MAX_DIST_M
    carry_gate_min_consecutive: int = 2  # PANEL_PICK_OBJECT_CARRY_GATE_MIN_CONSECUTIVE
    carry_gate_require_state: bool = True  # PANEL_PICK_OBJECT_CARRY_GATE_REQUIRE_STATE
    carry_gate_sample_dt_sec: float = 0.08  # PANEL_PICK_OBJECT_CARRY_GATE_SAMPLE_DT_SEC
    carry_gate_timeout_sec: float = 1.4  # PANEL_PICK_OBJECT_CARRY_GATE_TIMEOUT_SEC
    carry_joint_time_sec: float = 8.0  # PANEL_PICK_OBJECT_CARRY_JOINT_TIME_SEC
    # F2-step1: contact_down_z_m + extra_down_z_m son aliases (legacy primero).
    # None = usar default = max(0.020, PICK_DEMO_GRASP_Z_OFFSET).
    contact_down_z_m: Optional[float] = None  # PANEL_PICK_OBJECT_CONTACT_DOWN_Z_M
    deterministic_carry_gate_max_m: float = 0.8  # PANEL_PICK_OBJECT_DETERMINISTIC_CARRY_GATE_MAX_M
    deterministic_carry_gate_min_consecutive: int = 1  # PANEL_PICK_OBJECT_DETERMINISTIC_CARRY_GATE_MIN_CONSECUTIVE
    deterministic_joint_after_approach: bool = False  # PANEL_PICK_OBJECT_DETERMINISTIC_JOINT_AFTER_APPROACH
    # F2-step1: alias legacy de contact_down_z_m. Si ambos están vacíos cae al default.
    extra_down_z_m: Optional[float] = None  # PANEL_PICK_OBJECT_EXTRA_DOWN_Z
    force_home_start: bool = True  # PANEL_PICK_OBJECT_FORCE_HOME_START
    # F2-step1: PANEL_PICK_OBJECT_GRASP_CARTESIAN (default off; el flujo lo
    # mutaba en runtime para fallbacks → la mutación se queda como side-effect).
    grasp_cartesian: bool = False  # PANEL_PICK_OBJECT_GRASP_CARTESIAN
    grasp_joint_fallback: bool = False  # PANEL_PICK_OBJECT_GRASP_JOINT_FALLBACK
    grasp_micro_steps_max: int = 0  # PANEL_PICK_OBJECT_GRASP_MICRO_STEPS_MAX
    grasp_micro_step_m: float = 0.012  # PANEL_PICK_OBJECT_GRASP_MICRO_STEP_M
    grasp_moveit_retry: bool = True  # PANEL_PICK_OBJECT_GRASP_MOVEIT_RETRY
    grasp_moveit_retry_count: int = 2  # PANEL_PICK_OBJECT_GRASP_MOVEIT_RETRY_COUNT
    # F2-step1: tolerancia canónica de pasos GRASP_DOWN. Defaults históricos
    # divergían: 0.04 (line 3017), 0.045 (line 3912), 0.040 (lines 3976/3987).
    # Cada campo guarda su propio default histórico para preservar comportamiento.
    # Nota: la cadena legacy "TF_STABLE_TOL hereda de STEP_TOL si éste se setea
    # via env" se simplifica — ahora cada uno se override por su propio env var.
    grasp_step_tol_m: float = 0.040  # PANEL_PICK_OBJECT_GRASP_STEP_TOL_M
    grasp_tf_stable_min_ok: int = 4  # PANEL_PICK_OBJECT_GRASP_TF_STABLE_MIN_OK
    grasp_tf_stable_samples: int = 5  # PANEL_PICK_OBJECT_GRASP_TF_STABLE_SAMPLES
    grasp_tf_stable_tol_m: float = 0.045  # PANEL_PICK_OBJECT_GRASP_TF_STABLE_TOL_M
    grasp_z_reach_tol_m: float = 0.040  # PANEL_PICK_OBJECT_GRASP_Z_REACH_TOL_M
    grasp_z_reach_tol_fallback_m: float = 0.040  # PANEL_PICK_OBJECT_GRASP_Z_REACH_TOL_FALLBACK_M
    gripper_open_settle_sec: float = 0.7  # PANEL_PICK_OBJECT_GRIPPER_OPEN_SETTLE_SEC
    home_start_dur_sec: float = 8.0  # PANEL_PICK_OBJECT_HOME_START_DUR_SEC
    home_start_if_far: bool = True  # PANEL_PICK_OBJECT_HOME_START_IF_FAR
    home_start_if_far_max_err_rad: float = 2.0  # PANEL_PICK_OBJECT_HOME_START_IF_FAR_MAX_ERR_RAD
    home_start_ready_wait_sec: float = 45.0  # PANEL_PICK_OBJECT_HOME_START_READY_WAIT_SEC
    home_start_require_reached: bool = False  # PANEL_PICK_OBJECT_HOME_START_REQUIRE_REACHED
    home_tol_rad: float = 0.08  # PANEL_PICK_OBJECT_HOME_TOL_RAD
    insert_m: float = 0.015  # PANEL_PICK_OBJECT_INSERT_M
    joint_settle_extra_sec: float = 2.5  # PANEL_PICK_OBJECT_JOINT_SETTLE_EXTRA_SEC
    lift_clearance_m: float = 0.2  # PANEL_PICK_OBJECT_LIFT_CLEARANCE_M
    max_obj_move_before_close_m: float = 0.025  # PANEL_PICK_OBJECT_MAX_OBJ_MOVE_BEFORE_CLOSE_M
    max_tcp_above_obj_z: float = 0.045  # PANEL_PICK_OBJECT_MAX_TCP_ABOVE_OBJ_Z
    min_approach_clearance_m: float = 0.3  # PANEL_PICK_OBJECT_MIN_APPROACH_CLEARANCE_M
    min_opening_before_descent_m: float = 0.03  # PANEL_PICK_OBJECT_MIN_OPENING_BEFORE_DESCENT_M
    min_opening_m: float = 0.02  # PANEL_PICK_OBJECT_MIN_OPENING_M
    min_pre_clearance_m: float = 0.15  # PANEL_PICK_OBJECT_MIN_PRE_CLEARANCE_M
    moveit_active_request_grace_sec: float = 90.0  # PANEL_PICK_OBJECT_MOVEIT_ACTIVE_REQUEST_GRACE_SEC
    moveit_active_request_hb_sec: float = 2.5  # PANEL_PICK_OBJECT_MOVEIT_ACTIVE_REQUEST_HB_SEC
    moveit_bridge_recover_timeout_sec: float = 10.0  # PANEL_PICK_OBJECT_MOVEIT_BRIDGE_RECOVER_TIMEOUT_SEC
    moveit_exclusive: bool = True  # PANEL_PICK_OBJECT_MOVEIT_EXCLUSIVE
    object_xy_gate_tol_m: float = 0.03  # PANEL_PICK_OBJECT_OBJECT_XY_GATE_TOL_M
    open_cmd_ack_sec: float = 3.0  # PANEL_PICK_OBJECT_OPEN_CMD_ACK_SEC
    open_wait_sec: float = 2.2  # PANEL_PICK_OBJECT_OPEN_WAIT_SEC
    orientation_xyzw: str = '0.70710678,0.0,0.70710678,0.0'  # PANEL_PICK_OBJECT_ORIENTATION_XYZW
    path_tol_m: float = 0.03  # PANEL_PICK_OBJECT_PATH_TOL_M
    pick_image_preflight_require_reached: bool = True  # PANEL_PICK_OBJECT_PICK_IMAGE_PREFLIGHT_REQUIRE_REACHED
    pick_image_preflight_tol_rad: float = 0.05  # PANEL_PICK_OBJECT_PICK_IMAGE_PREFLIGHT_TOL_RAD
    post_lift_max_dist_m: float = 0.18  # PANEL_PICK_OBJECT_POST_LIFT_MAX_DIST_M
    post_lift_min_consecutive: int = 2  # PANEL_PICK_OBJECT_POST_LIFT_MIN_CONSECUTIVE
    preflight_mode: str = 'mesa'  # PANEL_PICK_OBJECT_PREFLIGHT_MODE
    # F2-step1: None ⇒ usa pre_grasp_tol_m. Override para path-tol abort recovery.
    pre_grasp_abort_recovery_tol_m_override: Optional[float] = None  # PANEL_PICK_OBJECT_PRE_GRASP_ABORT_RECOVERY_TOL_M
    pre_grasp_align_xy_tol_m: float = 0.045  # PANEL_PICK_OBJECT_PRE_GRASP_ALIGN_XY_TOL_M
    pre_grasp_align_z_tol_m: float = 0.05  # PANEL_PICK_OBJECT_PRE_GRASP_ALIGN_Z_TOL_M
    pre_grasp_recenter_enable: bool = False  # PANEL_PICK_OBJECT_PRE_GRASP_RECENTER_ENABLE
    # F2-step1: None ⇒ usa pre_grasp_tol_m. Override para fase PRE_GRASP_RECENTER.
    pre_grasp_recenter_tol_m_override: Optional[float] = None  # PANEL_PICK_OBJECT_PRE_GRASP_RECENTER_TOL_M
    pre_grasp_tol_m: float = 0.1  # PANEL_PICK_OBJECT_PRE_GRASP_TOL_M
    pre_margin_m: float = 0.04  # PANEL_PICK_OBJECT_PRE_MARGIN_M
    selection_max_age_sec: float = 600.0  # PANEL_PICK_OBJECT_SELECTION_MAX_AGE_SEC
    staged_lift_always: bool = False  # PANEL_PICK_OBJECT_STAGED_LIFT_ALWAYS
    step_tol_m: float = 0.02  # PANEL_PICK_OBJECT_STEP_TOL_M
    strict_contact_gate_enable: bool = True  # PANEL_PICK_OBJECT_STRICT_CONTACT_GATE_ENABLE
    strict_contact_settle_sec: float = 0.7  # PANEL_PICK_OBJECT_STRICT_CONTACT_SETTLE_SEC
    strict_lift_settle_sec: float = 0.2  # PANEL_PICK_OBJECT_STRICT_LIFT_SETTLE_SEC
    strict_lift_stage_max_dist_m: float = 0.245  # PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_MAX_DIST_M
    strict_lift_stage_min_consecutive: int = 1  # PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_MIN_CONSECUTIVE
    strict_lift_stage_step_m: float = 0.05  # PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_STEP_M
    strict_lift_stage_timeout_sec: float = 0.9  # PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_TIMEOUT_SEC
    strict_max_blocked_opening_m: float = 0.85  # PANEL_PICK_OBJECT_STRICT_MAX_BLOCKED_OPENING_M
    strict_min_blocked_opening_m: float = 0.05  # PANEL_PICK_OBJECT_STRICT_MIN_BLOCKED_OPENING_M
    strict_min_effort_abs: float = 0.0  # PANEL_PICK_OBJECT_STRICT_MIN_EFFORT_ABS
    strict_probe_lift_m: float = 0.05  # PANEL_PICK_OBJECT_STRICT_PROBE_LIFT_M
    strict_probe_max_dist_m: float = 0.16  # PANEL_PICK_OBJECT_STRICT_PROBE_MAX_DIST_M
    strict_probe_regrasp_step_m: float = 0.006  # PANEL_PICK_OBJECT_STRICT_PROBE_REGRASP_STEP_M
    strict_probe_retries: int = 2  # PANEL_PICK_OBJECT_STRICT_PROBE_RETRIES
    strict_probe_timeout_sec: float = 0.9  # PANEL_PICK_OBJECT_STRICT_PROBE_TIMEOUT_SEC
    strict_regrasp_floor_margin_m: float = 0.008  # PANEL_PICK_OBJECT_STRICT_REGRASP_FLOOR_MARGIN_M
    strict_regrasp_max: int = 1  # PANEL_PICK_OBJECT_STRICT_REGRASP_MAX
    strict_regrasp_step_m: float = 0.004  # PANEL_PICK_OBJECT_STRICT_REGRASP_STEP_M
    table_margin_m: float = 0.015  # PANEL_PICK_OBJECT_TABLE_MARGIN_M
    tcp_obj_dist_max_m: float = 0.055  # PANEL_PICK_OBJECT_TCP_OBJ_DIST_MAX_M
    transport_clearance_m: float = 0.2  # PANEL_PICK_OBJECT_TRANSPORT_CLEARANCE_M
    transport_moveit_wait_sec: float = 460.0  # PANEL_PICK_OBJECT_TRANSPORT_MOVEIT_WAIT_SEC
    transport_skip_constraints: bool = True  # PANEL_PICK_OBJECT_TRANSPORT_SKIP_CONSTRAINTS
    transport_split_moveit: bool = True  # PANEL_PICK_OBJECT_TRANSPORT_SPLIT_MOVEIT
    transport_split_stages: int = 3  # PANEL_PICK_OBJECT_TRANSPORT_SPLIT_STAGES
    world_ready_poll_sec: float = 0.25  # PANEL_PICK_OBJECT_WORLD_READY_POLL_SEC
    world_ready_scope: str = 'target'  # PANEL_PICK_OBJECT_WORLD_READY_SCOPE
    world_ready_wait_sec: float = 20.0  # PANEL_PICK_OBJECT_WORLD_READY_WAIT_SEC


ENV_VAR_BY_FIELD: Dict[str, str] = {
    "adapt_grasp_xy": "PANEL_PICK_OBJECT_ADAPT_GRASP_XY",
    "adapt_grasp_xy_max_delta_m": "PANEL_PICK_OBJECT_ADAPT_GRASP_XY_MAX_DELTA_M",
    "allow_degraded_open_guard": "PANEL_PICK_OBJECT_ALLOW_DEGRADED_OPEN_GUARD",
    "allow_joint_preflight_in_moveit": "PANEL_PICK_OBJECT_ALLOW_JOINT_PREFLIGHT_IN_MOVEIT",
    "allow_joint_recovery_in_moveit": "PANEL_PICK_OBJECT_ALLOW_JOINT_RECOVERY_IN_MOVEIT",
    "allow_tf_helper_fallback": "PANEL_PICK_OBJECT_ALLOW_TF_HELPER_FALLBACK",
    "approach_clearance_m": "PANEL_PICK_OBJECT_APPROACH_CLEARANCE_M",
    "approach_skip_retry_on_fallback": "PANEL_PICK_OBJECT_APPROACH_SKIP_RETRY_ON_FALLBACK",
    "approach_tol_m": "PANEL_PICK_OBJECT_APPROACH_TOL_M",
    "attach_xy_tol_m": "PANEL_ATTACH_XY_TOL_M",  # F2-step1
    "attach_z_clearance_m": "PANEL_PICK_OBJECT_ATTACH_Z_CLEARANCE_M",
    "attach_z_ref_mode": "PANEL_PICK_OBJECT_ATTACH_Z_REF_MODE",
    "attach_z_tol_m": "PANEL_ATTACH_Z_TOL_M",  # F2-step1
    "carry_gate_enable": "PANEL_PICK_OBJECT_CARRY_GATE_ENABLE",
    "carry_gate_max_dist_m": "PANEL_PICK_OBJECT_CARRY_GATE_MAX_DIST_M",
    "carry_gate_min_consecutive": "PANEL_PICK_OBJECT_CARRY_GATE_MIN_CONSECUTIVE",
    "carry_gate_require_state": "PANEL_PICK_OBJECT_CARRY_GATE_REQUIRE_STATE",
    "carry_gate_sample_dt_sec": "PANEL_PICK_OBJECT_CARRY_GATE_SAMPLE_DT_SEC",
    "carry_gate_timeout_sec": "PANEL_PICK_OBJECT_CARRY_GATE_TIMEOUT_SEC",
    "carry_joint_time_sec": "PANEL_PICK_OBJECT_CARRY_JOINT_TIME_SEC",
    "contact_down_z_m": "PANEL_PICK_OBJECT_CONTACT_DOWN_Z_M",  # F2-step1
    "deterministic_carry_gate_max_m": "PANEL_PICK_OBJECT_DETERMINISTIC_CARRY_GATE_MAX_M",
    "deterministic_carry_gate_min_consecutive": "PANEL_PICK_OBJECT_DETERMINISTIC_CARRY_GATE_MIN_CONSECUTIVE",
    "deterministic_joint_after_approach": "PANEL_PICK_OBJECT_DETERMINISTIC_JOINT_AFTER_APPROACH",
    "extra_down_z_m": "PANEL_PICK_OBJECT_EXTRA_DOWN_Z",  # F2-step1
    "force_home_start": "PANEL_PICK_OBJECT_FORCE_HOME_START",
    "grasp_cartesian": "PANEL_PICK_OBJECT_GRASP_CARTESIAN",  # F2-step1
    "grasp_joint_fallback": "PANEL_PICK_OBJECT_GRASP_JOINT_FALLBACK",
    "grasp_micro_steps_max": "PANEL_PICK_OBJECT_GRASP_MICRO_STEPS_MAX",
    "grasp_micro_step_m": "PANEL_PICK_OBJECT_GRASP_MICRO_STEP_M",
    "grasp_moveit_retry": "PANEL_PICK_OBJECT_GRASP_MOVEIT_RETRY",
    "grasp_moveit_retry_count": "PANEL_PICK_OBJECT_GRASP_MOVEIT_RETRY_COUNT",
    "grasp_step_tol_m": "PANEL_PICK_OBJECT_GRASP_STEP_TOL_M",  # F2-step1
    "grasp_tf_stable_min_ok": "PANEL_PICK_OBJECT_GRASP_TF_STABLE_MIN_OK",
    "grasp_tf_stable_samples": "PANEL_PICK_OBJECT_GRASP_TF_STABLE_SAMPLES",
    "grasp_tf_stable_tol_m": "PANEL_PICK_OBJECT_GRASP_TF_STABLE_TOL_M",  # F2-step1
    "grasp_z_reach_tol_fallback_m": "PANEL_PICK_OBJECT_GRASP_Z_REACH_TOL_FALLBACK_M",  # F2-step1
    "grasp_z_reach_tol_m": "PANEL_PICK_OBJECT_GRASP_Z_REACH_TOL_M",  # F2-step1
    "gripper_open_settle_sec": "PANEL_PICK_OBJECT_GRIPPER_OPEN_SETTLE_SEC",
    "home_start_dur_sec": "PANEL_PICK_OBJECT_HOME_START_DUR_SEC",
    "home_start_if_far": "PANEL_PICK_OBJECT_HOME_START_IF_FAR",
    "home_start_if_far_max_err_rad": "PANEL_PICK_OBJECT_HOME_START_IF_FAR_MAX_ERR_RAD",
    "home_start_ready_wait_sec": "PANEL_PICK_OBJECT_HOME_START_READY_WAIT_SEC",
    "home_start_require_reached": "PANEL_PICK_OBJECT_HOME_START_REQUIRE_REACHED",
    "home_tol_rad": "PANEL_PICK_OBJECT_HOME_TOL_RAD",
    "insert_m": "PANEL_PICK_OBJECT_INSERT_M",
    "joint_settle_extra_sec": "PANEL_PICK_OBJECT_JOINT_SETTLE_EXTRA_SEC",
    "lift_clearance_m": "PANEL_PICK_OBJECT_LIFT_CLEARANCE_M",
    "max_obj_move_before_close_m": "PANEL_PICK_OBJECT_MAX_OBJ_MOVE_BEFORE_CLOSE_M",
    "max_tcp_above_obj_z": "PANEL_PICK_OBJECT_MAX_TCP_ABOVE_OBJ_Z",
    "min_approach_clearance_m": "PANEL_PICK_OBJECT_MIN_APPROACH_CLEARANCE_M",
    "min_opening_before_descent_m": "PANEL_PICK_OBJECT_MIN_OPENING_BEFORE_DESCENT_M",
    "min_opening_m": "PANEL_PICK_OBJECT_MIN_OPENING_M",
    "min_pre_clearance_m": "PANEL_PICK_OBJECT_MIN_PRE_CLEARANCE_M",
    "moveit_active_request_grace_sec": "PANEL_PICK_OBJECT_MOVEIT_ACTIVE_REQUEST_GRACE_SEC",
    "moveit_active_request_hb_sec": "PANEL_PICK_OBJECT_MOVEIT_ACTIVE_REQUEST_HB_SEC",
    "moveit_bridge_recover_timeout_sec": "PANEL_PICK_OBJECT_MOVEIT_BRIDGE_RECOVER_TIMEOUT_SEC",
    "moveit_exclusive": "PANEL_PICK_OBJECT_MOVEIT_EXCLUSIVE",
    "object_xy_gate_tol_m": "PANEL_PICK_OBJECT_OBJECT_XY_GATE_TOL_M",
    "open_cmd_ack_sec": "PANEL_PICK_OBJECT_OPEN_CMD_ACK_SEC",
    "open_wait_sec": "PANEL_PICK_OBJECT_OPEN_WAIT_SEC",
    "orientation_xyzw": "PANEL_PICK_OBJECT_ORIENTATION_XYZW",
    "path_tol_m": "PANEL_PICK_OBJECT_PATH_TOL_M",
    "pick_image_preflight_require_reached": "PANEL_PICK_OBJECT_PICK_IMAGE_PREFLIGHT_REQUIRE_REACHED",
    "pick_image_preflight_tol_rad": "PANEL_PICK_OBJECT_PICK_IMAGE_PREFLIGHT_TOL_RAD",
    "post_lift_max_dist_m": "PANEL_PICK_OBJECT_POST_LIFT_MAX_DIST_M",
    "post_lift_min_consecutive": "PANEL_PICK_OBJECT_POST_LIFT_MIN_CONSECUTIVE",
    "preflight_mode": "PANEL_PICK_OBJECT_PREFLIGHT_MODE",
    "pre_grasp_abort_recovery_tol_m_override": "PANEL_PICK_OBJECT_PRE_GRASP_ABORT_RECOVERY_TOL_M",  # F2-step1
    "pre_grasp_align_xy_tol_m": "PANEL_PICK_OBJECT_PRE_GRASP_ALIGN_XY_TOL_M",
    "pre_grasp_align_z_tol_m": "PANEL_PICK_OBJECT_PRE_GRASP_ALIGN_Z_TOL_M",
    "pre_grasp_recenter_enable": "PANEL_PICK_OBJECT_PRE_GRASP_RECENTER_ENABLE",
    "pre_grasp_recenter_tol_m_override": "PANEL_PICK_OBJECT_PRE_GRASP_RECENTER_TOL_M",  # F2-step1
    "pre_grasp_tol_m": "PANEL_PICK_OBJECT_PRE_GRASP_TOL_M",
    "pre_margin_m": "PANEL_PICK_OBJECT_PRE_MARGIN_M",
    "selection_max_age_sec": "PANEL_PICK_OBJECT_SELECTION_MAX_AGE_SEC",
    "staged_lift_always": "PANEL_PICK_OBJECT_STAGED_LIFT_ALWAYS",
    "step_tol_m": "PANEL_PICK_OBJECT_STEP_TOL_M",
    "strict_contact_gate_enable": "PANEL_PICK_OBJECT_STRICT_CONTACT_GATE_ENABLE",
    "strict_contact_settle_sec": "PANEL_PICK_OBJECT_STRICT_CONTACT_SETTLE_SEC",
    "strict_lift_settle_sec": "PANEL_PICK_OBJECT_STRICT_LIFT_SETTLE_SEC",
    "strict_lift_stage_max_dist_m": "PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_MAX_DIST_M",
    "strict_lift_stage_min_consecutive": "PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_MIN_CONSECUTIVE",
    "strict_lift_stage_step_m": "PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_STEP_M",
    "strict_lift_stage_timeout_sec": "PANEL_PICK_OBJECT_STRICT_LIFT_STAGE_TIMEOUT_SEC",
    "strict_max_blocked_opening_m": "PANEL_PICK_OBJECT_STRICT_MAX_BLOCKED_OPENING_M",
    "strict_min_blocked_opening_m": "PANEL_PICK_OBJECT_STRICT_MIN_BLOCKED_OPENING_M",
    "strict_min_effort_abs": "PANEL_PICK_OBJECT_STRICT_MIN_EFFORT_ABS",
    "strict_probe_lift_m": "PANEL_PICK_OBJECT_STRICT_PROBE_LIFT_M",
    "strict_probe_max_dist_m": "PANEL_PICK_OBJECT_STRICT_PROBE_MAX_DIST_M",
    "strict_probe_regrasp_step_m": "PANEL_PICK_OBJECT_STRICT_PROBE_REGRASP_STEP_M",
    "strict_probe_retries": "PANEL_PICK_OBJECT_STRICT_PROBE_RETRIES",
    "strict_probe_timeout_sec": "PANEL_PICK_OBJECT_STRICT_PROBE_TIMEOUT_SEC",
    "strict_regrasp_floor_margin_m": "PANEL_PICK_OBJECT_STRICT_REGRASP_FLOOR_MARGIN_M",
    "strict_regrasp_max": "PANEL_PICK_OBJECT_STRICT_REGRASP_MAX",
    "strict_regrasp_step_m": "PANEL_PICK_OBJECT_STRICT_REGRASP_STEP_M",
    "table_margin_m": "PANEL_PICK_OBJECT_TABLE_MARGIN_M",
    "tcp_obj_dist_max_m": "PANEL_PICK_OBJECT_TCP_OBJ_DIST_MAX_M",
    "transport_clearance_m": "PANEL_PICK_OBJECT_TRANSPORT_CLEARANCE_M",
    "transport_moveit_wait_sec": "PANEL_PICK_OBJECT_TRANSPORT_MOVEIT_WAIT_SEC",
    "transport_skip_constraints": "PANEL_PICK_OBJECT_TRANSPORT_SKIP_CONSTRAINTS",
    "transport_split_moveit": "PANEL_PICK_OBJECT_TRANSPORT_SPLIT_MOVEIT",
    "transport_split_stages": "PANEL_PICK_OBJECT_TRANSPORT_SPLIT_STAGES",
    "world_ready_poll_sec": "PANEL_PICK_OBJECT_WORLD_READY_POLL_SEC",
    "world_ready_scope": "PANEL_PICK_OBJECT_WORLD_READY_SCOPE",
    "world_ready_wait_sec": "PANEL_PICK_OBJECT_WORLD_READY_WAIT_SEC",
}


def _coerce(field_name: str, raw: Any, default: Any) -> Any:
    if raw is None:
        return default
    if isinstance(default, bool):
        s = str(raw).strip().lower()
        return s in ("1", "true", "yes", "on")
    if isinstance(default, int) and not isinstance(default, bool):
        try:
            return int(raw)
        except (TypeError, ValueError):
            return default
    if isinstance(default, float):
        try:
            return float(raw)
        except (TypeError, ValueError):
            return default
    if default is None:
        s = str(raw).strip()
        if not s:
            return None
        try:
            return float(s)
        except (TypeError, ValueError):
            return None
    return str(raw)


def _read_yaml(path: Path) -> Dict[str, Any]:
    if yaml is None or not path.is_file():
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return {}
    if not isinstance(data, dict):
        return {}
    return data


def load_pick_object_params(yaml_path: Optional[Path] = None) -> PickObjectParams:
    """Cargar los parámetros con prioridad env > YAML > default."""
    target = yaml_path if yaml_path is not None else _DEFAULT_YAML_PATH
    yaml_data = _read_yaml(target)
    base = PickObjectParams()
    overrides: Dict[str, Any] = {}
    for f in fields(base):
        default = getattr(base, f.name)
        env_name = ENV_VAR_BY_FIELD.get(f.name, "")
        env_raw = os.environ.get(env_name) if env_name else None
        yaml_raw = yaml_data.get(f.name)
        if env_raw is not None and env_raw != "":
            overrides[f.name] = _coerce(f.name, env_raw, default)
        elif yaml_raw is not None:
            overrides[f.name] = _coerce(f.name, yaml_raw, default)
    if not overrides:
        return base
    return PickObjectParams(**{**{f.name: getattr(base, f.name) for f in fields(base)}, **overrides})


_PICK_OBJECT_PARAMS_CACHE: Optional[PickObjectParams] = None


def get_pick_object_params() -> PickObjectParams:
    """Lazy singleton (env > YAML > default).
    Para invalidar (tests): reset_pick_object_params_cache().
    """
    global _PICK_OBJECT_PARAMS_CACHE
    if _PICK_OBJECT_PARAMS_CACHE is None:
        _PICK_OBJECT_PARAMS_CACHE = load_pick_object_params()
    return _PICK_OBJECT_PARAMS_CACHE


def reset_pick_object_params_cache() -> None:
    """Invalida el singleton."""
    global _PICK_OBJECT_PARAMS_CACHE
    _PICK_OBJECT_PARAMS_CACHE = None
