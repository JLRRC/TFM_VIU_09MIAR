#!/usr/bin/env python3
"""Pure gate-evaluation helpers for the DIRECTO pick-demo pipeline.

All functions here are pure or near-pure (no panel/Qt/ROS state).
Dependencies: math, typing, directo_geometry, panel_config, ur5_kinematics.
"""
from __future__ import annotations

import math

from .directo_geometry import (
    _is_demo_basket_transport_motion,
    _is_demo_basket_transport_stage,
    _pick_demo_fmt_scalar,
    _pick_demo_tuple3,
)
from .panel_config import UR5_JOINT_NAMES
from .ur5_kinematics import fk_ur5


def _should_apply_global_step_timeout_extra(
    label: str,
    *,
    requested: bool,
) -> bool:
    if not bool(requested):
        return False
    if _is_demo_basket_transport_motion(label):
        return False
    return True


def _coerce_ur5_joint_vector(values) -> list[float] | None:
    try:
        joints = [float(value) for value in values]
    except Exception:
        return None
    if len(joints) != len(UR5_JOINT_NAMES):
        return None
    if not all(math.isfinite(value) for value in joints):
        return None
    return joints


def _normalize_joint_goal_near_seed(joint_goal, seed) -> list[float]:
    joint_goal_list = [float(value) for value in joint_goal]
    seed_list = [float(value) for value in seed]
    two_pi = 2.0 * math.pi
    return [
        float(q) + two_pi * round((float(s) - float(q)) / two_pi)
        for q, s in zip(joint_goal_list, seed_list)
    ]


def _resolve_joint_goal_normalization_seed(
    *,
    label: str,
    fallback_seed,
    retry_seed=None,
) -> list[float]:
    fallback_seed_list = _coerce_ur5_joint_vector(fallback_seed)
    retry_seed_list = _coerce_ur5_joint_vector(retry_seed)
    if _is_demo_basket_transport_stage(label) and retry_seed_list is not None:
        return retry_seed_list
    if fallback_seed_list is not None:
        return fallback_seed_list
    if retry_seed_list is not None:
        return retry_seed_list
    raise ValueError("joint_goal_normalization_seed_unavailable")


def _normalize_joint_goal_for_execution(
    *,
    label: str,
    joint_goal,
    fallback_seed,
    retry_seed=None,
) -> list[float]:
    normalization_seed = _resolve_joint_goal_normalization_seed(
        label=label,
        fallback_seed=fallback_seed,
        retry_seed=retry_seed,
    )
    return _normalize_joint_goal_near_seed(joint_goal, normalization_seed)


def _build_transport_seed_candidates(
    *,
    base_seed,
    live_seed=None,
    last_transport_joint_goal=None,
    prep_reference_seed=None,
) -> list[tuple[list[float], str]]:
    candidates: list[tuple[list[float], str]] = []
    seen: set[tuple[float, ...]] = set()

    def _append(seed_values, source: str) -> None:
        joints = _coerce_ur5_joint_vector(seed_values)
        if joints is None:
            return
        key = tuple(round(float(value), 6) for value in joints)
        if key in seen:
            return
        seen.add(key)
        candidates.append((joints, source))

    _append(base_seed, "base_seed")
    _append(live_seed, "live_joints")
    _append(last_transport_joint_goal, "last_transport_ok")
    _append(prep_reference_seed, "prep_reference")

    corridor_seed = (
        _coerce_ur5_joint_vector(last_transport_joint_goal)
        or _coerce_ur5_joint_vector(prep_reference_seed)
        or _coerce_ur5_joint_vector(live_seed)
        or _coerce_ur5_joint_vector(base_seed)
    )
    if corridor_seed is None:
        return candidates

    wrist_variant = list(corridor_seed)
    wrist_variant[4] = -(math.pi / 2.0)
    if abs(float(wrist_variant[5])) > math.radians(5.0):
        wrist_variant[5] = 0.0
    _append(wrist_variant, "corridor_wrist2")

    shoulder_elbow_variant = list(corridor_seed)
    shoulder_elbow_variant[0] = float(corridor_seed[0]) - math.radians(18.0)
    shoulder_elbow_variant[1] = float(corridor_seed[1]) - math.radians(8.0)
    shoulder_elbow_variant[2] = float(corridor_seed[2]) + math.radians(16.0)
    shoulder_elbow_variant[4] = -(math.pi / 2.0)
    shoulder_elbow_variant[5] = 0.0
    _append(shoulder_elbow_variant, "corridor_shoulder_elbow")
    return candidates


def _evaluate_transport_stage_preexec_model_guard(
    *,
    label: str,
    target_ik,
    joint_goal,
    tol_m: float,
    fk_fn=fk_ur5,
) -> dict:
    safe_tol_m = max(0.001, float(tol_m))
    if not _is_demo_basket_transport_stage(label):
        return {
            "ok": True,
            "reason": "skipped_non_transport_stage",
            "model_target_tol_m": safe_tol_m,
            "model_target_err_m": None,
            "fk_target": None,
        }
    target_ik_3 = _pick_demo_tuple3(target_ik)
    joint_goal_list = _coerce_ur5_joint_vector(joint_goal)
    if target_ik_3 is None or joint_goal_list is None:
        return {
            "ok": False,
            "reason": "invalid_input",
            "model_target_tol_m": safe_tol_m,
            "model_target_err_m": None,
            "fk_target": None,
        }
    try:
        fk_target, _fk_rot = fk_fn(joint_goal_list)
    except Exception as exc:
        return {
            "ok": False,
            "reason": f"fk_error:{exc}",
            "model_target_tol_m": safe_tol_m,
            "model_target_err_m": None,
            "fk_target": None,
        }
    fk_target_3 = _pick_demo_tuple3(fk_target)
    if fk_target_3 is None:
        return {
            "ok": False,
            "reason": "fk_invalid",
            "model_target_tol_m": safe_tol_m,
            "model_target_err_m": None,
            "fk_target": None,
        }
    model_target_err_m = math.sqrt(
        (float(fk_target_3[0]) - float(target_ik_3[0])) ** 2
        + (float(fk_target_3[1]) - float(target_ik_3[1])) ** 2
        + (float(fk_target_3[2]) - float(target_ik_3[2])) ** 2
    )
    reason = "ok"
    if model_target_err_m > safe_tol_m:
        reason = f"model_target_err={_pick_demo_fmt_scalar(model_target_err_m)}/{safe_tol_m:.3f}"
    return {
        "ok": bool(model_target_err_m <= safe_tol_m),
        "reason": reason,
        "model_target_tol_m": safe_tol_m,
        "model_target_err_m": float(model_target_err_m),
        "fk_target": fk_target_3,
    }


def _direct_pregrasp_gate_caps(phase: str | None) -> dict[str, float] | None:
    phase_name = str(phase or "").strip().upper()
    if phase_name not in {"APPROACH_COARSE", "GRASP_DOWN_JOINT"}:
        return None
    # Hard diagnostic caps for the handoff that leads into GRASP_DOWN.
    # Runtime profiles may relax generic defaults, but this interval must keep
    # tight source freshness and jump checks or the phase boundary becomes
    # meaningless.
    import os as _os
    def _envf(name: str, default: float) -> float:
        try:
            v = _os.environ.get(name)
            return float(v) if v is not None and str(v).strip() != "" else float(default)
        except Exception:
            return float(default)
    return {
        "source_tol_m": _envf("PANEL_PICK_DEMO_PREGRASP_SOURCE_TOL_M", 0.006),
        "source_age_tol_sec": _envf("PANEL_PICK_DEMO_PREGRASP_SOURCE_AGE_TOL_SEC", 0.400),
        "source_sync_tol_sec": _envf("PANEL_PICK_DEMO_PREGRASP_SOURCE_SYNC_TOL_SEC", 0.400),
        "phase_jump_tol_m": _envf("PANEL_PICK_DEMO_PREGRASP_PHASE_JUMP_TOL_M", 0.010),
        "coarse_xy_tol_m": _envf("PANEL_PICK_DEMO_PREGRASP_COARSE_XY_TOL_M", 0.006),
        "keep_xy_tol_m": _envf("PANEL_PICK_DEMO_PREGRASP_KEEP_XY_TOL_M", 0.005),
        "object_divergence_tol_m": _envf("PANEL_PICK_DEMO_PREGRASP_OBJ_DIV_TOL_M", 0.020),
    }


def _should_transport_prep_failure_jump_to_replan(
    *,
    failed_segment_index: int,
    total_segments: int,
    max_joint_residual_rad: float | None,
    shoulder_joint_residual_rad: float | None,
    min_failed_segment_fraction: float,
    max_joint_residual_threshold_rad: float,
    shoulder_joint_residual_threshold_rad: float,
) -> bool:
    if max_joint_residual_rad is None or shoulder_joint_residual_rad is None:
        return False
    safe_total_segments = max(1, int(total_segments))
    safe_failed_segment_index = min(
        safe_total_segments,
        max(1, int(failed_segment_index)),
    )
    safe_failed_fraction = float(safe_failed_segment_index) / float(safe_total_segments)
    safe_min_failed_fraction = min(
        1.0,
        max(0.0, float(min_failed_segment_fraction)),
    )
    return bool(
        safe_failed_fraction >= safe_min_failed_fraction
        and (
            float(max_joint_residual_rad) >= float(max_joint_residual_threshold_rad)
            or float(shoulder_joint_residual_rad) >= float(shoulder_joint_residual_threshold_rad)
        )
    )


def _evaluate_transport_stage_postcheck(
    *,
    label: str,
    runtime_target_ok: bool | None,
    runtime_target_dist_m: float | None,
    runtime_target_tol_m: float,
    model_target_err_m: float | None,
    model_target_tol_m: float,
    joint_accept_source: str | None = None,
) -> dict:
    safe_runtime_target_tol_m = max(0.001, float(runtime_target_tol_m))
    safe_model_target_tol_m = max(0.001, float(model_target_tol_m))
    reasons: list[str] = []
    bypass_model_target_err = bool(
        "_RECOVER_" in str(label or "").strip().upper()
        and str(joint_accept_source or "").strip().lower() == "runtime_target"
        and runtime_target_ok is True
    )
    if runtime_target_ok is False:
        if runtime_target_dist_m is None:
            reasons.append("runtime_target=unconfirmed")
        elif float(runtime_target_dist_m) > safe_runtime_target_tol_m:
            reasons.append(
                "runtime_target_dist="
                f"{_pick_demo_fmt_scalar(runtime_target_dist_m)}/{safe_runtime_target_tol_m:.3f}"
            )
    if (
        not bypass_model_target_err
        and model_target_err_m is not None
        and float(model_target_err_m) > safe_model_target_tol_m
    ):
        reasons.append(
            "model_target_err="
            f"{_pick_demo_fmt_scalar(model_target_err_m)}/{safe_model_target_tol_m:.3f}"
        )
    return {
        "ok": not reasons,
        "reason": (
            "runtime_target_ok_recovery_model_guard_bypassed"
            if not reasons and bypass_model_target_err
            else ("ok" if not reasons else " ".join(reasons))
        ),
        "runtime_target_tol_m": safe_runtime_target_tol_m,
        "model_target_tol_m": safe_model_target_tol_m,
        "model_target_bypassed": bool(bypass_model_target_err),
    }


def _transport_prep_failure_policy(*, strict_mode: bool) -> str:
    return "raise" if bool(strict_mode) else "continue_final_stage"
