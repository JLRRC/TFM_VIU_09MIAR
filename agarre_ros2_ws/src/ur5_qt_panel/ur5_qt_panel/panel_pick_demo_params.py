#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo_params.py
# Contenido: F2 parcial — infraestructura de parámetros runtime para pick_demo.
# Uso breve:
#   from .panel_pick_demo_params import load_pick_demo_params
#   params = load_pick_demo_params()
#   z_offset = params.grasp_tcp_z_offset_m
"""Parámetros runtime del flujo pick_demo (F2 parcial).

Hoy ``panel_pick_demo.py`` lee 151 ``os.environ`` distintas. Este módulo
introduce una capa única (dataclass) con los **11 parámetros más críticos**
para que el tuneo se haga vía YAML en lugar de exportar variables. Es
trabajo incremental: el código antiguo que sigue leyendo ``os.environ``
NO se toca; este módulo simplemente provee otra puerta de entrada con
estas reglas de prioridad:

    env var > YAML override > default del dataclass

Para tunear sin perder reproducibilidad:
1. Editar ``agarre_ros2_ws/src/ur5_qt_panel/config/pick_demo_runtime.yaml``
2. (Alternativa rápida) ``export PANEL_PICK_DEMO_<NAME>=value`` y relanzar

La tabla de equivalencias está en el YAML (cada clave del YAML lleva el
nombre de la env como comentario).

Migración futura: cuando se quiera quitar un ``os.environ.get`` del
flujo grande, cambiarlo por ``params.<campo>`` aquí y validar que el
default del dataclass coincide con el default histórico de la env.
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


# Ruta absoluta del YAML por defecto. Si no existe, se usan los defaults
# del dataclass (idénticos a los que tenía cada ``os.environ.get`` en
# panel_pick_demo / pick_demo/internal_helpers / directo_geometry).
def _resolve_default_yaml_path() -> Path:
    """Resolver la ruta del YAML tanto en install como en source-tree."""
    # 1) Install (colcon): share/ur5_qt_panel/config/pick_demo_runtime.yaml
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = Path(get_package_share_directory("ur5_qt_panel"))
        candidate = share_dir / "config" / "pick_demo_runtime.yaml"
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    # 2) Source-tree (dev): src/ur5_qt_panel/config/pick_demo_runtime.yaml
    src_candidate = (
        Path(__file__).resolve().parent.parent
        / "config"
        / "pick_demo_runtime.yaml"
    )
    return src_candidate


_DEFAULT_YAML_PATH = _resolve_default_yaml_path()


@dataclass(frozen=True)
class PickDemoParams:
    """11 parámetros críticos del flujo pick_demo.

    Defaults extraídos de las llamadas ``os.environ.get(NAME, default)`` del
    código histórico. Cualquier cambio aquí debe mantener paridad con esos
    defaults para no alterar comportamiento sin querer.
    """

    # -- IK seed override --------------------------------------------------
    # Empty string ≡ "auto" (el flujo elige seed dinámicamente).
    ik_seed_joints: str = ""  # PANEL_PICK_DEMO_IK_SEED_JOINTS

    # -- Tolerancias de gates ----------------------------------------------
    grasp_down_util_z_err_tol_m: float = 0.025  # PANEL_PICK_DEMO_GRASP_DOWN_UTIL_Z_ERR_TOL_M
    close_z_err_tol_m: float = 0.012            # PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M
    close_xy_tol_m: float = 0.012               # PANEL_PICK_DEMO_CLOSE_XY_TOL_M
    approach_coarse_keep_xy_tol_m: float = 0.020  # PANEL_PICK_DEMO_APPROACH_COARSE_KEEP_XY_TOL_M

    # -- Geometría TCP -----------------------------------------------------
    grasp_tcp_z_offset_m: float = 0.0  # PANEL_PICK_DEMO_GRASP_TCP_Z_OFFSET_M

    # -- Pinza -------------------------------------------------------------
    gripper_closed_opening_thr_m: float = 0.020  # PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M

    # -- Tiempos -----------------------------------------------------------
    # None ≡ usar el spinbox del panel (joint_time).
    move_sec: Optional[float] = None  # PANEL_PICK_DEMO_MOVE_SEC

    # -- Pesos / tolerancias IK GRASP_DOWN --------------------------------
    grasp_down_ik_err_tol: float = 0.080      # PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL
    grasp_down_ik_seed_weight: float = 0.65   # PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT
    grasp_down_rot_weight: float = 0.10       # PANEL_PICK_DEMO_GRASP_DOWN_ROT_WEIGHT

    # -- Tolerancias TCP por fase del pick (DIRECTO) ----------------------
    # Consumidas en directo_geometry._direct_runtime_target_tol_m. Cada fase
    # del flujo tiene su propia tolerancia objetivo del TCP en metros.
    approach_coarse_tcp_tol_m: float = 0.015        # PANEL_PICK_DEMO_APPROACH_COARSE_TCP_TOL_M
    approach_coarse_refine_tcp_tol_m: float = 0.006 # PANEL_PICK_DEMO_APPROACH_COARSE_REFINE_TCP_TOL_M
    grasp_down_tcp_tol_m: float = 0.020             # PANEL_PICK_DEMO_GRASP_DOWN_TCP_TOL_M
    grasp_align_tcp_tol_m: float = 0.015            # PANEL_PICK_DEMO_GRASP_ALIGN_TCP_TOL_M
    basket_transport_tcp_tol_m: float = 0.060       # PANEL_PICK_DEMO_BASKET_TRANSPORT_TCP_TOL_M
    direct_ik_tcp_tol_m: float = 0.040              # PANEL_PICK_DEMO_DIRECT_IK_TCP_TOL_M

    # -- GRASP_DOWN: branch guard (gates de seguridad de rama IK) ---------
    grasp_down_branch_guard_xy_tol_m: float = 0.010    # PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_GUARD_XY_TOL_M
    grasp_down_branch_guard_z_min_m: float = 0.015     # PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_GUARD_Z_MIN_M
    grasp_down_branch_guard_max_dev_rad: float = 0.35  # PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_GUARD_MAX_DEV_RAD
    grasp_down_branch_guard_sum_dev_rad: float = 0.75  # PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_GUARD_SUM_DEV_RAD

    # -- GRASP_DOWN: deltas máximos del branch IK (límites por junta) -----
    grasp_down_branch_max_delta_rad: float = 0.95              # PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_MAX_DELTA_RAD
    grasp_down_branch_sum_delta_rad: float = 1.80              # PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_SUM_DELTA_RAD
    grasp_down_branch_shoulder_lift_delta_rad: float = 0.80    # PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_SHOULDER_LIFT_DELTA_RAD
    grasp_down_branch_elbow_delta_rad: float = 0.85            # PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_ELBOW_DELTA_RAD
    grasp_down_branch_wrist1_delta_rad: float = 0.85           # PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_WRIST1_DELTA_RAD

    # -- GRASP_DOWN: deltas máximos del phase IK (rama principal) ---------
    grasp_down_phase_max_delta_rad: float = 2.35               # PANEL_PICK_DEMO_GRASP_DOWN_PHASE_MAX_DELTA_RAD
    grasp_down_phase_sum_delta_rad: float = 6.20               # PANEL_PICK_DEMO_GRASP_DOWN_PHASE_SUM_DELTA_RAD
    grasp_down_phase_critical_sum_delta_rad: float = 2.85      # PANEL_PICK_DEMO_GRASP_DOWN_PHASE_CRITICAL_SUM_DELTA_RAD
    grasp_down_phase_shoulder_lift_delta_rad: float = 1.20     # PANEL_PICK_DEMO_GRASP_DOWN_PHASE_SHOULDER_LIFT_DELTA_RAD
    grasp_down_phase_elbow_delta_rad: float = 1.15             # PANEL_PICK_DEMO_GRASP_DOWN_PHASE_ELBOW_DELTA_RAD
    grasp_down_phase_wrist1_delta_rad: float = 1.10            # PANEL_PICK_DEMO_GRASP_DOWN_PHASE_WRIST1_DELTA_RAD

    # -- GRASP_DOWN: tolerancias estrictas + paso del segmento ------------
    grasp_down_strict_xy_tol_m: float = 0.012     # PANEL_PICK_DEMO_GRASP_DOWN_STRICT_XY_TOL_M
    grasp_down_strict_z_tol_m: float = 0.025      # PANEL_PICK_DEMO_GRASP_DOWN_STRICT_Z_TOL_M
    grasp_down_strict_dist_tol_m: float = 0.025   # PANEL_PICK_DEMO_GRASP_DOWN_STRICT_DIST_TOL_M
    grasp_down_segment_xy_step_m: float = 0.020   # PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_XY_STEP_M
    grasp_down_segment_z_step_m: float = 0.005    # PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_Z_STEP_M

    # -- F2 final batch: 94 tunables estáticos del flujo pick_demo --------
    # Orden alfabético; defaults coinciden con los fallbacks históricos
    # de cada env var. Familias: AC_PHASE_CHECK, ALIGN, APPROACH_COARSE,
    # ATTACH, CARRY, CLOSE, DEBUG, DIRECT_IK, FALLBACK_PRESET,
    # GRASP_ALIGN, GRASP_DOWN restantes, GRIPPER, HANDOFF,
    # MANUAL_LIKE_ATTACH, MANUAL_REF_STALE, POST_*, PRE_CLOSE, RELEASE,
    # ROUTE / state flags.
    ac_phase_check_settle_sec: float = 3.0  # PANEL_PICK_DEMO_AC_PHASE_CHECK_SETTLE_SEC
    ac_phase_check_stable_samples: int = 3  # PANEL_PICK_DEMO_AC_PHASE_CHECK_STABLE_SAMPLES
    ac_phase_check_threshold_m: float = 0.004  # PANEL_PICK_DEMO_AC_PHASE_CHECK_THRESHOLD_M
    align_exit_xy_tol_m: float = 0.006  # PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M
    align_exit_z_tol_m: float = 0.01  # PANEL_PICK_DEMO_ALIGN_EXIT_Z_TOL_M
    align_ik_err_tol: float = 0.08  # PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL
    align_ik_seed_weight: float = 0.5  # PANEL_PICK_DEMO_ALIGN_IK_SEED_WEIGHT
    align_no_effect_tol_m: float = 0.002  # PANEL_PICK_DEMO_ALIGN_NO_EFFECT_TOL_M
    align_rot_weight: float = 0.1  # PANEL_PICK_DEMO_ALIGN_ROT_WEIGHT
    align_xy_lock_factor: float = 2.0  # PANEL_PICK_DEMO_ALIGN_XY_LOCK_FACTOR
    align_z_bias_cap_m: float = 0.03  # PANEL_PICK_DEMO_ALIGN_Z_BIAS_CAP_M
    align_z_bias_gain: float = 0.7  # PANEL_PICK_DEMO_ALIGN_Z_BIAS_GAIN
    align_z_improve_min_m: float = 0.006  # PANEL_PICK_DEMO_ALIGN_Z_IMPROVE_MIN_M
    align_z_residual_tol_m: float = 0.015  # PANEL_PICK_DEMO_ALIGN_Z_RESIDUAL_TOL_M
    approach_coarse_gate_poll_sec: float = 0.1  # PANEL_PICK_DEMO_APPROACH_COARSE_GATE_POLL_SEC
    approach_coarse_gate_settle_sec: float = 0.8  # PANEL_PICK_DEMO_APPROACH_COARSE_GATE_SETTLE_SEC
    approach_coarse_gate_stable_samples: int = 2  # PANEL_PICK_DEMO_APPROACH_COARSE_GATE_STABLE_SAMPLES
    approach_coarse_gate_z_tol_m: float = 0.008  # PANEL_PICK_DEMO_APPROACH_COARSE_GATE_Z_TOL_M
    approach_coarse_handoff_dist_tol_m: float = 0.015  # PANEL_PICK_DEMO_APPROACH_COARSE_HANDOFF_DIST_TOL_M
    approach_coarse_handoff_dz_tol_m: float = 0.015  # PANEL_PICK_DEMO_APPROACH_COARSE_HANDOFF_DZ_TOL_M
    approach_coarse_max_skip_m: float = 0.06  # PANEL_PICK_DEMO_APPROACH_COARSE_MAX_SKIP_M
    approach_coarse_relaxed_handoff_xy_tol_m: float = 0.01  # PANEL_PICK_DEMO_APPROACH_COARSE_RELAXED_HANDOFF_XY_TOL_M
    approach_coarse_relaxed_skip_pose_ok: bool = False  # PANEL_PICK_DEMO_APPROACH_COARSE_RELAXED_SKIP_POSE_OK
    approach_coarse_skip_xy_tol_m: float = 0.03  # PANEL_PICK_DEMO_APPROACH_COARSE_SKIP_XY_TOL_M
    approach_coarse_skip_z_tol_m: float = 0.04  # PANEL_PICK_DEMO_APPROACH_COARSE_SKIP_Z_TOL_M
    approach_coarse_util_dist_tol_m: float = 0.26  # PANEL_PICK_DEMO_APPROACH_COARSE_UTIL_DIST_TOL_M
    approach_coarse_util_xy_tol_m: float = 0.18  # PANEL_PICK_DEMO_APPROACH_COARSE_UTIL_XY_TOL_M
    approach_coarse_util_z_err_tol_m: float = 0.18  # PANEL_PICK_DEMO_APPROACH_COARSE_UTIL_Z_ERR_TOL_M
    approach_coarse_xy_corr_max_m: float = 0.04  # PANEL_PICK_DEMO_APPROACH_COARSE_XY_CORR_MAX_M
    approach_coarse_xy_corr_tol_m: float = 0.015  # PANEL_PICK_DEMO_APPROACH_COARSE_XY_CORR_TOL_M
    approach_coarse_z_corr_tol_m: float = 0.02  # PANEL_PICK_DEMO_APPROACH_COARSE_Z_CORR_TOL_M
    attach_follow_max_tcp_dist_m: float = 0.04  # PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M
    attach_max_rel_drift_m: float = 0.012  # PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M
    attach_max_tf_visual_gap_m: float = 0.02  # PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M
    attach_min_stable_samples: int = 5  # PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES
    attach_settle_sec: float = 1.8  # PANEL_PICK_DEMO_ATTACH_SETTLE_SEC
    attach_stable_window_sec: float = 0.35  # PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC
    attach_xy_tol_m: float = 0.012  # PANEL_PICK_DEMO_ATTACH_XY_TOL_M
    attach_z_tol_m: float = 0.015  # PANEL_PICK_DEMO_ATTACH_Z_TOL_M
    carry_home_max_tcp_dist_m: float = 0.2  # PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M
    carry_settle_sec: float = 3.0  # PANEL_PICK_DEMO_CARRY_SETTLE_SEC
    close_confirm_timeout_sec: float = 1.8  # PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC
    close_fallback_opening_sum: float = 0.4  # PANEL_PICK_DEMO_CLOSE_FALLBACK_OPENING_SUM
    close_min_delta_sum: float = 0.08  # PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM
    debug_pause_grasp_align_ik: bool = False  # PANEL_PICK_DEMO_DEBUG_PAUSE_GRASP_ALIGN_IK
    direct_ik_joint_tol_rad: float = 0.03  # PANEL_PICK_DEMO_DIRECT_IK_JOINT_TOL_RAD
    direct_ik_runtime_attempts: int = 5  # PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_ATTEMPTS
    direct_ik_runtime_settle_delta_m: float = 0.003  # PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_DELTA_M
    direct_ik_runtime_settle_poll_sec: float = 0.1  # PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_POLL_SEC
    direct_ik_runtime_settle_samples: int = 3  # PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SAMPLES
    direct_ik_runtime_settle_sec: float = 2.5  # PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SEC
    direct_ik_seed_weight: float = 0.035  # PANEL_PICK_DEMO_DIRECT_IK_SEED_WEIGHT
    direct_ik_tcp_timeout_sec: float = 4.0  # PANEL_PICK_DEMO_DIRECT_IK_TCP_TIMEOUT_SEC
    extra_grasp_down_m: float = 0.0  # PANEL_PICK_DEMO_EXTRA_GRASP_DOWN_M
    fallback_preset_max_dist_m: float = 0.1  # PANEL_PICK_DEMO_FALLBACK_PRESET_MAX_DIST_M
    fallback_preset_max_xy_m: float = 0.05  # PANEL_PICK_DEMO_FALLBACK_PRESET_MAX_XY_M
    grasp_align_joint_tol_rad: float = 0.01  # PANEL_PICK_DEMO_GRASP_ALIGN_JOINT_TOL_RAD
    grasp_align_max_attempts: int = 3  # PANEL_PICK_DEMO_GRASP_ALIGN_MAX_ATTEMPTS
    grasp_down_disable_permissive_fallback: bool = False  # PANEL_PICK_DEMO_GRASP_DOWN_DISABLE_PERMISSIVE_FALLBACK
    grasp_down_extra_z_m: float = 0.0  # PANEL_PICK_DEMO_GRASP_DOWN_EXTRA_Z_M
    grasp_down_gate_poll_sec: float = 0.1  # PANEL_PICK_DEMO_GRASP_DOWN_GATE_POLL_SEC
    grasp_down_gate_settle_sec: float = 0.8  # PANEL_PICK_DEMO_GRASP_DOWN_GATE_SETTLE_SEC
    grasp_down_gate_stable_samples: int = 2  # PANEL_PICK_DEMO_GRASP_DOWN_GATE_STABLE_SAMPLES
    grasp_down_max_attempts: int = 4  # PANEL_PICK_DEMO_GRASP_DOWN_MAX_ATTEMPTS
    grasp_down_permissive_ik_err_tol: float = 0.015  # PANEL_PICK_DEMO_GRASP_DOWN_PERMISSIVE_IK_ERR_TOL
    grasp_down_permissive_rot_weight: float = 0.35  # PANEL_PICK_DEMO_GRASP_DOWN_PERMISSIVE_ROT_WEIGHT
    grasp_down_permissive_seed_weight: float = 0.65  # PANEL_PICK_DEMO_GRASP_DOWN_PERMISSIVE_SEED_WEIGHT
    grasp_down_use_moveit_cartesian: bool = True  # PANEL_PICK_DEMO_GRASP_DOWN_USE_MOVEIT_CARTESIAN
    grasp_down_util_dist_tol_m: float = 0.22  # PANEL_PICK_DEMO_GRASP_DOWN_UTIL_DIST_TOL_M
    gripper_confirm_max_state_age_sec: float = 0.35  # PANEL_PICK_DEMO_GRIPPER_CONFIRM_MAX_STATE_AGE_SEC
    gripper_confirm_stable_samples: int = 2  # PANEL_PICK_DEMO_GRIPPER_CONFIRM_STABLE_SAMPLES
    gripper_target_tol_m: float = 0.035  # PANEL_PICK_DEMO_GRIPPER_TARGET_TOL_M
    handoff_target_jump_tol_m: float = 0.005  # PANEL_PICK_DEMO_HANDOFF_TARGET_JUMP_TOL_M
    manual_like_attach_max_tcp_dist_m: float = 0.14  # PANEL_PICK_DEMO_MANUAL_LIKE_ATTACH_MAX_TCP_DIST_M
    manual_like_attach_wait_sec: float = 0.9  # PANEL_PICK_DEMO_MANUAL_LIKE_ATTACH_WAIT_SEC
    manual_like_attach_xy_tol_m: float = 0.06  # PANEL_PICK_DEMO_MANUAL_LIKE_ATTACH_XY_TOL_M
    manual_like_attach_z_tol_m: float = 0.06  # PANEL_PICK_DEMO_MANUAL_LIKE_ATTACH_Z_TOL_M
    manual_ref_stale_xy_tol_m: float = 0.08  # PANEL_PICK_DEMO_MANUAL_REF_STALE_XY_TOL_M
    manual_ref_stale_z_below_tol_m: float = 0.005  # PANEL_PICK_DEMO_MANUAL_REF_STALE_Z_BELOW_TOL_M
    post_align_settle_sec: float = 0.2  # PANEL_PICK_DEMO_POST_ALIGN_SETTLE_SEC
    post_attach_hold_sec: float = 0.9  # PANEL_PICK_DEMO_POST_ATTACH_HOLD_SEC
    post_close_hold_sec: float = 0.45  # PANEL_PICK_DEMO_POST_CLOSE_HOLD_SEC
    post_close_mode: str = 'basket'  # PANEL_PICK_DEMO_POST_CLOSE_MODE
    pre_close_consecutive: int = 3  # PANEL_PICK_DEMO_PRE_CLOSE_CONSECUTIVE
    pre_close_realign_retries: int = 2  # PANEL_PICK_DEMO_PRE_CLOSE_REALIGN_RETRIES
    pre_close_wait_sec: float = 1.2  # PANEL_PICK_DEMO_PRE_CLOSE_WAIT_SEC
    release_open_confirm_timeout_sec: float = 1.8  # PANEL_PICK_DEMO_RELEASE_OPEN_CONFIRM_TIMEOUT_SEC
    release_wait_sec: float = 1.6  # PANEL_PICK_DEMO_RELEASE_WAIT_SEC
    route_mode: str = 'direct_ik_hybrid'  # PANEL_PICK_DEMO_ROUTE_MODE
    selected_base_stale_tol_m: float = 0.08  # PANEL_PICK_DEMO_SELECTED_BASE_STALE_TOL_M
    short_lift_m: float = 0.12  # PANEL_PICK_DEMO_SHORT_LIFT_M
    short_release_only: bool = False  # PANEL_PICK_DEMO_SHORT_RELEASE_ONLY
    skip_align_if_reachable: bool = True  # PANEL_PICK_DEMO_SKIP_ALIGN_IF_REACHABLE
    step_timeout_extra_sec: float = 0.0  # PANEL_PICK_DEMO_STEP_TIMEOUT_EXTRA_SEC
    object_source_divergence_tol_m: float = 0.150  # PANEL_PICK_DEMO_OBJECT_SOURCE_DIVERGENCE_TOL_M


# Mapeo campo dataclass → nombre de env var.
# Útil para implementar la prioridad env > YAML > default y para auditar
# qué parámetros existen.
ENV_VAR_BY_FIELD: Dict[str, str] = {
    "ik_seed_joints":                  "PANEL_PICK_DEMO_IK_SEED_JOINTS",
    "grasp_down_util_z_err_tol_m":     "PANEL_PICK_DEMO_GRASP_DOWN_UTIL_Z_ERR_TOL_M",
    "close_z_err_tol_m":               "PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M",
    "close_xy_tol_m":                  "PANEL_PICK_DEMO_CLOSE_XY_TOL_M",
    "approach_coarse_keep_xy_tol_m":   "PANEL_PICK_DEMO_APPROACH_COARSE_KEEP_XY_TOL_M",
    "grasp_tcp_z_offset_m":            "PANEL_PICK_DEMO_GRASP_TCP_Z_OFFSET_M",
    "gripper_closed_opening_thr_m":    "PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M",
    "move_sec":                        "PANEL_PICK_DEMO_MOVE_SEC",
    "grasp_down_ik_err_tol":           "PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL",
    "grasp_down_ik_seed_weight":       "PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT",
    "grasp_down_rot_weight":           "PANEL_PICK_DEMO_GRASP_DOWN_ROT_WEIGHT",
    "approach_coarse_tcp_tol_m":        "PANEL_PICK_DEMO_APPROACH_COARSE_TCP_TOL_M",
    "approach_coarse_refine_tcp_tol_m": "PANEL_PICK_DEMO_APPROACH_COARSE_REFINE_TCP_TOL_M",
    "grasp_down_tcp_tol_m":             "PANEL_PICK_DEMO_GRASP_DOWN_TCP_TOL_M",
    "grasp_align_tcp_tol_m":            "PANEL_PICK_DEMO_GRASP_ALIGN_TCP_TOL_M",
    "basket_transport_tcp_tol_m":       "PANEL_PICK_DEMO_BASKET_TRANSPORT_TCP_TOL_M",
    "direct_ik_tcp_tol_m":              "PANEL_PICK_DEMO_DIRECT_IK_TCP_TOL_M",
    "grasp_down_branch_guard_xy_tol_m":           "PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_GUARD_XY_TOL_M",
    "grasp_down_branch_guard_z_min_m":            "PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_GUARD_Z_MIN_M",
    "grasp_down_branch_guard_max_dev_rad":        "PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_GUARD_MAX_DEV_RAD",
    "grasp_down_branch_guard_sum_dev_rad":        "PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_GUARD_SUM_DEV_RAD",
    "grasp_down_branch_max_delta_rad":            "PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_MAX_DELTA_RAD",
    "grasp_down_branch_sum_delta_rad":            "PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_SUM_DELTA_RAD",
    "grasp_down_branch_shoulder_lift_delta_rad":  "PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_SHOULDER_LIFT_DELTA_RAD",
    "grasp_down_branch_elbow_delta_rad":          "PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_ELBOW_DELTA_RAD",
    "grasp_down_branch_wrist1_delta_rad":         "PANEL_PICK_DEMO_GRASP_DOWN_BRANCH_WRIST1_DELTA_RAD",
    "grasp_down_phase_max_delta_rad":             "PANEL_PICK_DEMO_GRASP_DOWN_PHASE_MAX_DELTA_RAD",
    "grasp_down_phase_sum_delta_rad":             "PANEL_PICK_DEMO_GRASP_DOWN_PHASE_SUM_DELTA_RAD",
    "grasp_down_phase_critical_sum_delta_rad":    "PANEL_PICK_DEMO_GRASP_DOWN_PHASE_CRITICAL_SUM_DELTA_RAD",
    "grasp_down_phase_shoulder_lift_delta_rad":   "PANEL_PICK_DEMO_GRASP_DOWN_PHASE_SHOULDER_LIFT_DELTA_RAD",
    "grasp_down_phase_elbow_delta_rad":           "PANEL_PICK_DEMO_GRASP_DOWN_PHASE_ELBOW_DELTA_RAD",
    "grasp_down_phase_wrist1_delta_rad":          "PANEL_PICK_DEMO_GRASP_DOWN_PHASE_WRIST1_DELTA_RAD",
    "grasp_down_strict_xy_tol_m":                 "PANEL_PICK_DEMO_GRASP_DOWN_STRICT_XY_TOL_M",
    "grasp_down_strict_z_tol_m":                  "PANEL_PICK_DEMO_GRASP_DOWN_STRICT_Z_TOL_M",
    "grasp_down_strict_dist_tol_m":               "PANEL_PICK_DEMO_GRASP_DOWN_STRICT_DIST_TOL_M",
    "grasp_down_segment_xy_step_m":               "PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_XY_STEP_M",
    "grasp_down_segment_z_step_m":                "PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_Z_STEP_M",
    "ac_phase_check_settle_sec": "PANEL_PICK_DEMO_AC_PHASE_CHECK_SETTLE_SEC",
    "ac_phase_check_stable_samples": "PANEL_PICK_DEMO_AC_PHASE_CHECK_STABLE_SAMPLES",
    "ac_phase_check_threshold_m": "PANEL_PICK_DEMO_AC_PHASE_CHECK_THRESHOLD_M",
    "align_exit_xy_tol_m": "PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M",
    "align_exit_z_tol_m": "PANEL_PICK_DEMO_ALIGN_EXIT_Z_TOL_M",
    "align_ik_err_tol": "PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL",
    "align_ik_seed_weight": "PANEL_PICK_DEMO_ALIGN_IK_SEED_WEIGHT",
    "align_no_effect_tol_m": "PANEL_PICK_DEMO_ALIGN_NO_EFFECT_TOL_M",
    "align_rot_weight": "PANEL_PICK_DEMO_ALIGN_ROT_WEIGHT",
    "align_xy_lock_factor": "PANEL_PICK_DEMO_ALIGN_XY_LOCK_FACTOR",
    "align_z_bias_cap_m": "PANEL_PICK_DEMO_ALIGN_Z_BIAS_CAP_M",
    "align_z_bias_gain": "PANEL_PICK_DEMO_ALIGN_Z_BIAS_GAIN",
    "align_z_improve_min_m": "PANEL_PICK_DEMO_ALIGN_Z_IMPROVE_MIN_M",
    "align_z_residual_tol_m": "PANEL_PICK_DEMO_ALIGN_Z_RESIDUAL_TOL_M",
    "approach_coarse_gate_poll_sec": "PANEL_PICK_DEMO_APPROACH_COARSE_GATE_POLL_SEC",
    "approach_coarse_gate_settle_sec": "PANEL_PICK_DEMO_APPROACH_COARSE_GATE_SETTLE_SEC",
    "approach_coarse_gate_stable_samples": "PANEL_PICK_DEMO_APPROACH_COARSE_GATE_STABLE_SAMPLES",
    "approach_coarse_gate_z_tol_m": "PANEL_PICK_DEMO_APPROACH_COARSE_GATE_Z_TOL_M",
    "approach_coarse_handoff_dist_tol_m": "PANEL_PICK_DEMO_APPROACH_COARSE_HANDOFF_DIST_TOL_M",
    "approach_coarse_handoff_dz_tol_m": "PANEL_PICK_DEMO_APPROACH_COARSE_HANDOFF_DZ_TOL_M",
    "approach_coarse_max_skip_m": "PANEL_PICK_DEMO_APPROACH_COARSE_MAX_SKIP_M",
    "approach_coarse_relaxed_handoff_xy_tol_m": "PANEL_PICK_DEMO_APPROACH_COARSE_RELAXED_HANDOFF_XY_TOL_M",
    "approach_coarse_relaxed_skip_pose_ok": "PANEL_PICK_DEMO_APPROACH_COARSE_RELAXED_SKIP_POSE_OK",
    "approach_coarse_skip_xy_tol_m": "PANEL_PICK_DEMO_APPROACH_COARSE_SKIP_XY_TOL_M",
    "approach_coarse_skip_z_tol_m": "PANEL_PICK_DEMO_APPROACH_COARSE_SKIP_Z_TOL_M",
    "approach_coarse_util_dist_tol_m": "PANEL_PICK_DEMO_APPROACH_COARSE_UTIL_DIST_TOL_M",
    "approach_coarse_util_xy_tol_m": "PANEL_PICK_DEMO_APPROACH_COARSE_UTIL_XY_TOL_M",
    "approach_coarse_util_z_err_tol_m": "PANEL_PICK_DEMO_APPROACH_COARSE_UTIL_Z_ERR_TOL_M",
    "approach_coarse_xy_corr_max_m": "PANEL_PICK_DEMO_APPROACH_COARSE_XY_CORR_MAX_M",
    "approach_coarse_xy_corr_tol_m": "PANEL_PICK_DEMO_APPROACH_COARSE_XY_CORR_TOL_M",
    "approach_coarse_z_corr_tol_m": "PANEL_PICK_DEMO_APPROACH_COARSE_Z_CORR_TOL_M",
    "attach_follow_max_tcp_dist_m": "PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M",
    "attach_max_rel_drift_m": "PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M",
    "attach_max_tf_visual_gap_m": "PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M",
    "attach_min_stable_samples": "PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES",
    "attach_settle_sec": "PANEL_PICK_DEMO_ATTACH_SETTLE_SEC",
    "attach_stable_window_sec": "PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC",
    "attach_xy_tol_m": "PANEL_PICK_DEMO_ATTACH_XY_TOL_M",
    "attach_z_tol_m": "PANEL_PICK_DEMO_ATTACH_Z_TOL_M",
    "carry_home_max_tcp_dist_m": "PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M",
    "carry_settle_sec": "PANEL_PICK_DEMO_CARRY_SETTLE_SEC",
    "close_confirm_timeout_sec": "PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC",
    "close_fallback_opening_sum": "PANEL_PICK_DEMO_CLOSE_FALLBACK_OPENING_SUM",
    "close_min_delta_sum": "PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM",
    "debug_pause_grasp_align_ik": "PANEL_PICK_DEMO_DEBUG_PAUSE_GRASP_ALIGN_IK",
    "direct_ik_joint_tol_rad": "PANEL_PICK_DEMO_DIRECT_IK_JOINT_TOL_RAD",
    "direct_ik_runtime_attempts": "PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_ATTEMPTS",
    "direct_ik_runtime_settle_delta_m": "PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_DELTA_M",
    "direct_ik_runtime_settle_poll_sec": "PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_POLL_SEC",
    "direct_ik_runtime_settle_samples": "PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SAMPLES",
    "direct_ik_runtime_settle_sec": "PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SEC",
    "direct_ik_seed_weight": "PANEL_PICK_DEMO_DIRECT_IK_SEED_WEIGHT",
    "direct_ik_tcp_timeout_sec": "PANEL_PICK_DEMO_DIRECT_IK_TCP_TIMEOUT_SEC",
    "extra_grasp_down_m": "PANEL_PICK_DEMO_EXTRA_GRASP_DOWN_M",
    "fallback_preset_max_dist_m": "PANEL_PICK_DEMO_FALLBACK_PRESET_MAX_DIST_M",
    "fallback_preset_max_xy_m": "PANEL_PICK_DEMO_FALLBACK_PRESET_MAX_XY_M",
    "grasp_align_joint_tol_rad": "PANEL_PICK_DEMO_GRASP_ALIGN_JOINT_TOL_RAD",
    "grasp_align_max_attempts": "PANEL_PICK_DEMO_GRASP_ALIGN_MAX_ATTEMPTS",
    "grasp_down_disable_permissive_fallback": "PANEL_PICK_DEMO_GRASP_DOWN_DISABLE_PERMISSIVE_FALLBACK",
    "grasp_down_extra_z_m": "PANEL_PICK_DEMO_GRASP_DOWN_EXTRA_Z_M",
    "grasp_down_gate_poll_sec": "PANEL_PICK_DEMO_GRASP_DOWN_GATE_POLL_SEC",
    "grasp_down_gate_settle_sec": "PANEL_PICK_DEMO_GRASP_DOWN_GATE_SETTLE_SEC",
    "grasp_down_gate_stable_samples": "PANEL_PICK_DEMO_GRASP_DOWN_GATE_STABLE_SAMPLES",
    "grasp_down_max_attempts": "PANEL_PICK_DEMO_GRASP_DOWN_MAX_ATTEMPTS",
    "grasp_down_permissive_ik_err_tol": "PANEL_PICK_DEMO_GRASP_DOWN_PERMISSIVE_IK_ERR_TOL",
    "grasp_down_permissive_rot_weight": "PANEL_PICK_DEMO_GRASP_DOWN_PERMISSIVE_ROT_WEIGHT",
    "grasp_down_permissive_seed_weight": "PANEL_PICK_DEMO_GRASP_DOWN_PERMISSIVE_SEED_WEIGHT",
    "grasp_down_use_moveit_cartesian": "PANEL_PICK_DEMO_GRASP_DOWN_USE_MOVEIT_CARTESIAN",
    "grasp_down_util_dist_tol_m": "PANEL_PICK_DEMO_GRASP_DOWN_UTIL_DIST_TOL_M",
    "gripper_confirm_max_state_age_sec": "PANEL_PICK_DEMO_GRIPPER_CONFIRM_MAX_STATE_AGE_SEC",
    "gripper_confirm_stable_samples": "PANEL_PICK_DEMO_GRIPPER_CONFIRM_STABLE_SAMPLES",
    "gripper_target_tol_m": "PANEL_PICK_DEMO_GRIPPER_TARGET_TOL_M",
    "handoff_target_jump_tol_m": "PANEL_PICK_DEMO_HANDOFF_TARGET_JUMP_TOL_M",
    "manual_like_attach_max_tcp_dist_m": "PANEL_PICK_DEMO_MANUAL_LIKE_ATTACH_MAX_TCP_DIST_M",
    "manual_like_attach_wait_sec": "PANEL_PICK_DEMO_MANUAL_LIKE_ATTACH_WAIT_SEC",
    "manual_like_attach_xy_tol_m": "PANEL_PICK_DEMO_MANUAL_LIKE_ATTACH_XY_TOL_M",
    "manual_like_attach_z_tol_m": "PANEL_PICK_DEMO_MANUAL_LIKE_ATTACH_Z_TOL_M",
    "manual_ref_stale_xy_tol_m": "PANEL_PICK_DEMO_MANUAL_REF_STALE_XY_TOL_M",
    "manual_ref_stale_z_below_tol_m": "PANEL_PICK_DEMO_MANUAL_REF_STALE_Z_BELOW_TOL_M",
    "post_align_settle_sec": "PANEL_PICK_DEMO_POST_ALIGN_SETTLE_SEC",
    "post_attach_hold_sec": "PANEL_PICK_DEMO_POST_ATTACH_HOLD_SEC",
    "post_close_hold_sec": "PANEL_PICK_DEMO_POST_CLOSE_HOLD_SEC",
    "post_close_mode": "PANEL_PICK_DEMO_POST_CLOSE_MODE",
    "pre_close_consecutive": "PANEL_PICK_DEMO_PRE_CLOSE_CONSECUTIVE",
    "pre_close_realign_retries": "PANEL_PICK_DEMO_PRE_CLOSE_REALIGN_RETRIES",
    "pre_close_wait_sec": "PANEL_PICK_DEMO_PRE_CLOSE_WAIT_SEC",
    "release_open_confirm_timeout_sec": "PANEL_PICK_DEMO_RELEASE_OPEN_CONFIRM_TIMEOUT_SEC",
    "release_wait_sec": "PANEL_PICK_DEMO_RELEASE_WAIT_SEC",
    "route_mode": "PANEL_PICK_DEMO_ROUTE_MODE",
    "selected_base_stale_tol_m": "PANEL_PICK_DEMO_SELECTED_BASE_STALE_TOL_M",
    "short_lift_m": "PANEL_PICK_DEMO_SHORT_LIFT_M",
    "short_release_only": "PANEL_PICK_DEMO_SHORT_RELEASE_ONLY",
    "skip_align_if_reachable": "PANEL_PICK_DEMO_SKIP_ALIGN_IF_REACHABLE",
    "step_timeout_extra_sec": "PANEL_PICK_DEMO_STEP_TIMEOUT_EXTRA_SEC",
    "object_source_divergence_tol_m": "PANEL_PICK_DEMO_OBJECT_SOURCE_DIVERGENCE_TOL_M",
}


def _coerce(field_name: str, raw: Any, default: Any) -> Any:
    """Convertir ``raw`` al tipo del default; devolver ``default`` si falla."""
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
        # Optional[float] como move_sec: aceptar float o None
        s = str(raw).strip()
        if not s:
            return None
        try:
            return float(s)
        except (TypeError, ValueError):
            return None
    # str u otros
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


def load_pick_demo_params(yaml_path: Optional[Path] = None) -> PickDemoParams:
    """Cargar los parámetros con prioridad env > YAML > default.

    El env var (``ENV_VAR_BY_FIELD[campo]``), si existe en ``os.environ``,
    gana. Si no, se busca la clave en el YAML (nombre del campo, p.ej.
    ``grasp_tcp_z_offset_m``). Si tampoco está, se usa el default del
    dataclass. Cualquier valor que no se pueda coercionar al tipo
    correspondiente cae al default sin levantar excepción.
    """
    target = yaml_path if yaml_path is not None else _DEFAULT_YAML_PATH
    yaml_data = _read_yaml(target)

    base = PickDemoParams()
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
        # else: keep default

    if not overrides:
        return base
    return PickDemoParams(**{**{f.name: getattr(base, f.name) for f in fields(base)}, **overrides})


# ---------------------------------------------------------------------------
# Lazy singleton — usado por panel_pick_demo y directo_geometry para evitar
# re-leer YAML/env en cada gate del pick. Vive aquí para que ambos módulos
# importen el mismo helper sin riesgo de dependencia circular.
# ---------------------------------------------------------------------------
_PICK_DEMO_PARAMS_CACHE: Optional[PickDemoParams] = None


def get_pick_demo_params() -> PickDemoParams:
    """Lazy singleton de PickDemoParams (env > YAML > default).

    F2: las lecturas migradas usan este helper en lugar de os.environ.get.
    Para invalidar (tests), usa reset_pick_demo_params_cache().
    """
    global _PICK_DEMO_PARAMS_CACHE
    if _PICK_DEMO_PARAMS_CACHE is None:
        _PICK_DEMO_PARAMS_CACHE = load_pick_demo_params()
    return _PICK_DEMO_PARAMS_CACHE


def reset_pick_demo_params_cache() -> None:
    """Invalida el singleton. Útil en tests que mutan env vars."""
    global _PICK_DEMO_PARAMS_CACHE
    _PICK_DEMO_PARAMS_CACHE = None
