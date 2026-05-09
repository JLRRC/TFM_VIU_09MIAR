#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/grasp_down.py
# Contenido: F3-step3d — extracción de _run_grasp_down_conservative (526 LOC).
"""Grasp down conservative phase extraído del closure run_pick_demo.worker.

``run_grasp_down_conservative`` ejecuta la fase de DESCENSO PROGRESIVO
del Pick Demo: divide el segmento desde el TCP actual hasta el target
de grasp en sub-waypoints intermedios, y los recorre uno a uno con IK
estricto + checks de calidad articular + fallback a preset si la rama
del IK diverge de la del seed.

Antes de F3-step3d vivía como ``def`` anidada de 526 LOC dentro del
closure ``run_pick_demo.worker``, capturando 13 deps reales (panel +
11 callables del closure + move_sec). Aquí se extrae con el patrón
estándar dataclass + función pura + wrapper delgado.

NOTA: la fn original devuelve ``tuple[dict | None, str, dict]`` con
(metrics, decision_label, debug_info). El wrapper preserva esa firma
1:1.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from typing import Any, Callable, Optional, Tuple

from ur5_tools.gripper_geometry import RG2_PINCH_CENTER_FRAME, RG2_TCP_FRAME

from ..panel_pick_demo_params import get_pick_demo_params as _get_pick_demo_params
from ..panel_robot_presets import JOINT_GRASP_DOWN_POSE_RAD
from .pure_helpers import json_safe as _json_safe

# Frames canónicos (replicados de panel_pick_demo.py para evitar circular import)
DIRECT_SOURCE_FRAME = RG2_PINCH_CENTER_FRAME
DIRECT_LEGACY_TCP_FRAME = RG2_TCP_FRAME


def _grasp_down_permissive_ik_err_tol() -> float:
    """F3-step1.2: replicado aquí para evitar circular import con panel_pick_demo.

    El callsite original era nested closure en run_pick_demo. Tras extracción a
    grasp_down.py, los 3 callsites internos siguen usando el nombre original
    pero ahora la función vive en este módulo.
    """
    requested = float(_get_pick_demo_params().grasp_down_permissive_ik_err_tol)
    return min(0.025, max(0.010, requested))


@dataclass
class GraspDownContext:
    """Captura de dependencias closure de run_grasp_down_conservative."""

    panel: Any
    live_object_base: Callable[[], Any]
    live_tcp_base: Callable[[], Any]
    move_tcp_direct: Callable[..., Any]
    run_joint_step: Callable[..., Any]
    append_trace: Callable[[str], None]
    emit_pose_consistency: Callable[..., Any]
    pose_consistency_metrics: Callable[..., Any]
    grasp_down_joint_quality: Callable[..., Any]
    grasp_down_runtime_metrics: Callable[..., Any]
    grasp_down_waypoints: Callable[..., Any]
    joint_preset_fallback_ok: Callable[..., Any]
    current_joint_seed: Callable[[], Any]
    move_sec: float
    # Helpers module-level inyectados:
    get_pick_demo_params: Callable[[], Any]
    fmt_vec: Callable[[Any], str]
    fmt_scalar: Callable[..., str]
    tuple3: Callable[[Any], Any]
    iso_now: Callable[[], str]


def run_grasp_down_conservative(
    ctx: GraspDownContext,
    *,
    target_base,
    obj_base,
    timeout_sec: float,
    audit_target_source: str,
    phase_seed_joints=None,
) -> Tuple[Optional[dict], str, dict]:
    """Ejecuta GRASP_DOWN segmentado (waypoints intermedios + IK estricto).

    Devuelve ``(metrics, decision_label, debug_info)``. ``metrics`` es
    None si no se completó el descenso; ``decision_label`` describe el
    path tomado (full / preset_fallback / aborted); ``debug_info`` es
    un dict con telemetría de cada waypoint.
    """
    strict_xy_tol = max(0.006, ctx.get_pick_demo_params().grasp_down_strict_xy_tol_m)
    strict_z_tol = max(0.008, ctx.get_pick_demo_params().grasp_down_strict_z_tol_m)
    target_dist_tol = max(strict_xy_tol, ctx.get_pick_demo_params().grasp_down_strict_dist_tol_m)
    max_attempts = max(
        1,
        ctx.get_pick_demo_params().grasp_down_max_attempts,
    )
    rot_weight = max(0.0, ctx.get_pick_demo_params().grasp_down_rot_weight)
    ik_err_tol = max(0.035, ctx.get_pick_demo_params().grasp_down_ik_err_tol)
    joint_weight = max(0.0, ctx.get_pick_demo_params().grasp_down_ik_seed_weight)

    def _grasp_down_permissive_rot_weight() -> float:
        return max(
            rot_weight,
            ctx.get_pick_demo_params().grasp_down_permissive_rot_weight,
        )

    # F3-step1.2: _grasp_down_permissive_ik_err_tol promovido a module-level.

    def _grasp_down_permissive_joint_weight() -> float:
        return max(
            joint_weight,
            ctx.get_pick_demo_params().grasp_down_permissive_seed_weight,
        )

    grasp_down_disable_permissive_fallback = ctx.get_pick_demo_params().grasp_down_disable_permissive_fallback

    last_debug = None
    last_metrics = ctx.grasp_down_runtime_metrics(target_base=target_base, obj_base=obj_base)
    last_route = "cartesian_like_descent"
    _gd_seed_injected = False
    ctx.panel._emit_log(
        "[DIAG][GD_CONSERVATIVE_ENTRY] "
        f"tcp_base={ctx.fmt_vec(ctx.tuple3(ctx.live_tcp_base()))} "
        f"target_base={ctx.fmt_vec(ctx.tuple3(target_base))}"
    )
    # ── Permissive-first for large Z gap (Z-only) ────────────────────────
    # When the TCP is ≥20mm above the grasp target (standard after
    # APPROACH_COARSE at +35mm clearance), the multi-segment waypoint
    # approach sends tiny joint commands so fast the controller cannot
    # track them → arm barely moves → visual "hover/separation."
    # XY se congela al TCP actual para evitar drift lateral: permissive-first
    # solo desciende en Z. La corrección XY residual queda para el loop
    # segmentado con joint_weight=0.45, que la ejecuta en pasos controlados.
    _pf_tcp = ctx.tuple3(ctx.live_tcp_base())
    _pf_tgt = ctx.tuple3(target_base)
    if _pf_tcp is not None and _pf_tgt is not None:
        _pf_z_gap = abs(float(_pf_tcp[2]) - float(_pf_tgt[2]))
        _pf_xy_gap = math.hypot(
            float(_pf_tcp[0]) - float(_pf_tgt[0]),
            float(_pf_tcp[1]) - float(_pf_tgt[1]),
        )
        if _pf_z_gap >= 0.020 and grasp_down_disable_permissive_fallback:
            _pf_msg = (
                "[PICK][DIRECT][GRASP_DOWN_FALLBACK] "
                "reason=large_z_gap_pre_motion "
                f"z_gap={_pf_z_gap:.3f} xy_gap={_pf_xy_gap:.3f} "
                "strategy=permissive_z_only_descent_disabled"
            )
            ctx.panel._emit_log(_pf_msg)
            ctx.append_trace(_pf_msg)
        elif _pf_z_gap >= 0.020:
            _pf_z_only_target = (
                float(_pf_tcp[0]),
                float(_pf_tcp[1]),
                float(_pf_tgt[2]),
            )
            _pf_msg = (
                "[PICK][DIRECT][GRASP_DOWN_FALLBACK] "
                f"reason=large_z_gap_pre_motion "
                f"z_gap={_pf_z_gap:.3f} xy_gap={_pf_xy_gap:.3f} "
                f"strategy=permissive_z_only_descent "
                f"pf_target={ctx.fmt_vec(_pf_z_only_target)} "
                f"full_target={ctx.fmt_vec(_pf_tgt)}"
            )
            ctx.panel._emit_log(_pf_msg)
            ctx.append_trace(_pf_msg)
            try:
                last_debug = ctx.move_tcp_direct(
                    label="GRASP_DOWN_JOINT",
                    target_tcp_runtime=_pf_z_only_target,
                    timeout_sec=max(float(timeout_sec), ctx.move_sec + 3.0),
                    audit_target_source=f"{audit_target_source}:permissive_pre",
                    target_pose_original=_pf_z_only_target,
                    target_frame_original="base_link",
                    rot_weight=_grasp_down_permissive_rot_weight(),
                    ik_err_tol=_grasp_down_permissive_ik_err_tol(),
                    joint_weight=_grasp_down_permissive_joint_weight(),
                    force_send=True,
                )
                last_route = "permissive_direct_descent"
            except Exception as _pf_exc:
                ctx.panel._emit_log(
                    f"[PICK][DIRECT][GRASP_DOWN] permissive_pre_failed:{_pf_exc}"
                )
                last_debug = None
                last_route = "cartesian_like_descent"
    # If permissive_pre succeeded and z_gap is within the outer PHASE_CHECK
    # tolerance (UTIL_Z_ERR_TOL, not the tighter strict_z_tol), skip the
    # waypoint loop entirely.  Use the TCP position already settled inside
    # ctx.move_tcp_direct to avoid a stale ctx.live_tcp_base() read.
    # NOTE: strict_z_tol is 8mm (conservative descent gate) but the DH/SDF
    # divergence always leaves ~13mm Z residual → strict_z_tol would never
    # pass.  Use UTIL_Z_ERR_TOL (25mm) which is what the outer PHASE_CHECK
    # uses, so "early return here" ↔ "PHASE_CHECK will pass out there".
    if last_route == "permissive_direct_descent" and last_debug is not None:
        _pf_util_z_tol = max(
            0.008,
            ctx.get_pick_demo_params().grasp_down_util_z_err_tol_m,
        )
        _pf_actual = ctx.tuple3(
            (last_debug or {}).get("runtime_target_stable_pos")
            or (last_debug or {}).get("runtime_target_pos")
            or ctx.live_tcp_base()
        )
        if _pf_actual is not None:
            _pf_remaining = abs(float(_pf_actual[2]) - float(_pf_tgt[2]))
            _pf_xy_after = math.hypot(
                float(_pf_actual[0]) - float(_pf_tgt[0]),
                float(_pf_actual[1]) - float(_pf_tgt[1]),
            )
            _pf_msg2 = (
                "[DIAG][PERMISSIVE_RESULT] "
                f"actual={ctx.fmt_vec(_pf_actual)} "
                f"target={ctx.fmt_vec(_pf_tgt)} "
                f"xy_err_after={_pf_xy_after:.4f} "
                f"z_remaining={_pf_remaining:.4f}/{_pf_util_z_tol:.3f} "
                f"early_return={str(_pf_remaining <= _pf_util_z_tol).lower()}"
            )
            ctx.panel._emit_log(_pf_msg2)
            ctx.append_trace(_pf_msg2)
            if _pf_remaining <= _pf_util_z_tol:
                last_metrics = ctx.grasp_down_runtime_metrics(
                    target_base=target_base,
                    tcp_base=_pf_actual,
                    obj_base=ctx.tuple3(ctx.live_object_base()) or ctx.tuple3(obj_base),
                )
                return last_debug, "permissive_direct_descent:pre_motion", last_metrics
    # ─────────────────────────────────────────────────────────────────────
    for attempt in range(1, max_attempts + 1):
        actual_before = ctx.tuple3(ctx.live_tcp_base())
        object_now = ctx.tuple3(ctx.live_object_base()) or ctx.tuple3(obj_base)
        ctx.panel._emit_log(
            "[DIAG][GD_WAYPOINT_LOOP] "
            f"attempt={attempt} "
            f"actual={ctx.fmt_vec(actual_before)} "
            f"target={ctx.fmt_vec(ctx.tuple3(target_base))}"
        )
        step_divider = float(2 ** (attempt - 1))
        waypoint_xy_step = max(
            0.008,
            ctx.get_pick_demo_params().grasp_down_segment_xy_step_m / step_divider,
        )
        waypoint_z_step = max(
            0.005,
            ctx.get_pick_demo_params().grasp_down_segment_z_step_m / step_divider,
        )
        waypoints = ctx.grasp_down_waypoints(
            actual_before,
            target_base,
            max_xy_step_m=waypoint_xy_step,
            max_z_step_m=waypoint_z_step,
        )
        if not waypoints:
            raise RuntimeError("grasp_down_waypoints_unavailable")
        mode = "cartesian" if len(waypoints) > 1 else "hybrid"
        route = (
            f"cartesian_like_segments:{len(waypoints)}"
            if len(waypoints) > 1
            else "direct_ik"
        )
        try:
            for segment_idx, waypoint in enumerate(waypoints, start=1):
                last_debug = ctx.move_tcp_direct(
                    label="GRASP_DOWN_JOINT",
                    target_tcp_runtime=waypoint,
                    timeout_sec=max(float(timeout_sec), ctx.move_sec + 2.0 + float(attempt)),
                    audit_target_source=f"{audit_target_source}:segment_{segment_idx}_of_{len(waypoints)}",
                    target_pose_original=waypoint,
                    target_frame_original="base_link",
                    rot_weight=rot_weight,
                    ik_err_tol=ik_err_tol,
                    joint_weight=joint_weight,
                    force_send=True,
                )
            last_route = route
        except Exception as exc:
            previous_xy_err = last_metrics.get("xy_err_target")
            if grasp_down_disable_permissive_fallback:
                if attempt < max_attempts:
                    _fb_msg = (
                        "[PICK][DIRECT][GRASP_DOWN_FALLBACK] "
                        f"reason=direct_ik_exception:{exc} "
                        f"previous_xy_err={ctx.fmt_scalar(previous_xy_err)} "
                        f"strategy=retry_segmented_refine_without_permissive segments={len(waypoints)}"
                    )
                    ctx.panel._emit_log(_fb_msg)
                    ctx.append_trace(_fb_msg)
                    continue
                if ctx.joint_preset_fallback_ok(
                    "GRASP_DOWN_JOINT",
                    JOINT_GRASP_DOWN_POSE_RAD,
                    target_base=target_base,
                    obj_base=obj_base,
                ):
                    last_route = "joint_preset_last_resort"
                    _fb_msg = (
                        "[PICK][DIRECT][GRASP_DOWN_FALLBACK] "
                        f"reason=direct_ik_exception:{exc} "
                        f"previous_xy_err={ctx.fmt_scalar(previous_xy_err)} "
                        "strategy=joint_preset_last_resort"
                    )
                    ctx.panel._emit_log(_fb_msg)
                    ctx.append_trace(_fb_msg)
                    ctx.run_joint_step(
                        "GRASP_DOWN_JOINT_FALLBACK",
                        JOINT_GRASP_DOWN_POSE_RAD,
                        timeout_sec=ctx.move_sec + 6.0,
                        tol_rad=0.08,
                    )
                    last_debug = {
                        "ik_solution": [float(v) for v in JOINT_GRASP_DOWN_POSE_RAD],
                        "runtime_target_ok": False,
                        "runtime_target_dist": None,
                    }
                else:
                    raise
                continue
            permissive_rot_weight = _grasp_down_permissive_rot_weight()
            permissive_ik_err_tol = _grasp_down_permissive_ik_err_tol()
            permissive_joint_weight = _grasp_down_permissive_joint_weight()
            try:
                _fb_msg = (
                    "[PICK][DIRECT][GRASP_DOWN_FALLBACK] "
                    f"reason=direct_ik_exception:{exc} "
                    f"previous_xy_err={ctx.fmt_scalar(previous_xy_err)} "
                    "strategy=permissive_direct_descent"
                )
                ctx.panel._emit_log(_fb_msg)
                ctx.append_trace(_fb_msg)
                last_debug = ctx.move_tcp_direct(
                    label="GRASP_DOWN_JOINT",
                    target_tcp_runtime=target_base,
                    timeout_sec=max(float(timeout_sec), ctx.move_sec + 3.0 + float(attempt)),
                    audit_target_source=f"{audit_target_source}:permissive_final",
                    target_pose_original=target_base,
                    target_frame_original="base_link",
                    rot_weight=permissive_rot_weight,
                    ik_err_tol=permissive_ik_err_tol,
                    joint_weight=permissive_joint_weight,
                    force_send=True,
                )
                last_route = "permissive_direct_descent"
            except Exception as permissive_exc:
                if attempt < max_attempts:
                    _fb_msg = (
                        "[PICK][DIRECT][GRASP_DOWN_FALLBACK] "
                        f"reason=permissive_direct_exception:{permissive_exc} "
                        f"previous_xy_err={ctx.fmt_scalar(previous_xy_err)} "
                        f"strategy=retry_segmented_refine segments={len(waypoints)}"
                    )
                    ctx.panel._emit_log(_fb_msg)
                    ctx.append_trace(_fb_msg)
                    continue
                exc = permissive_exc
                if ctx.joint_preset_fallback_ok(
                    "GRASP_DOWN_JOINT",
                    JOINT_GRASP_DOWN_POSE_RAD,
                    target_base=target_base,
                    obj_base=obj_base,
                ):
                    last_route = "joint_preset_last_resort"
                    _fb_msg = (
                        "[PICK][DIRECT][GRASP_DOWN_FALLBACK] "
                        f"reason=direct_ik_exception:{exc} "
                        f"previous_xy_err={ctx.fmt_scalar(previous_xy_err)} "
                        "strategy=joint_preset_last_resort"
                    )
                    ctx.panel._emit_log(_fb_msg)
                    ctx.append_trace(_fb_msg)
                    ctx.run_joint_step(
                        "GRASP_DOWN_JOINT_FALLBACK",
                        JOINT_GRASP_DOWN_POSE_RAD,
                        timeout_sec=ctx.move_sec + 6.0,
                        tol_rad=0.08,
                    )
                    last_debug = {
                        "ik_solution": [float(v) for v in JOINT_GRASP_DOWN_POSE_RAD],
                        "runtime_target_ok": False,
                        "runtime_target_dist": None,
                    }
                else:
                    raise
        actual_after = ctx.tuple3(ctx.live_tcp_base())
        object_after = ctx.tuple3(ctx.live_object_base()) or object_now
        q_after = [float(v) for v in (ctx.current_joint_seed() or [])]
        # ── DH false-satisfied detection ─────────────────────────────────
        # When the IK reports joint_delta≈0 but physical TCP is still far
        # from target in Z, the DH model is "already satisfied" due to the
        # ~13mm DH/SDF Z divergence.  Arm stays put.  Force permissive.
        if last_route != "permissive_direct_descent":
            _dhs_seed = [float(v) for v in (phase_seed_joints or [])]
            _dhs_sum_delta = (
                sum(abs(a - b) for a, b in zip(q_after, _dhs_seed))
                if len(_dhs_seed) == len(q_after) and _dhs_seed
                else 1.0
            )
            _dhs_z_gap = (
                abs(float(actual_after[2]) - float(target_base[2]))
                if actual_after is not None
                else 0.0
            )
            if _dhs_z_gap > 0.010 and grasp_down_disable_permissive_fallback:
                _dhs_msg = (
                    "[PICK][DIRECT][GRASP_DOWN_FALLBACK] "
                    f"reason=dh_false_satisfied "
                    f"joint_delta_sum={_dhs_sum_delta:.4f} "
                    f"z_gap={_dhs_z_gap:.3f} "
                    "strategy=permissive_direct_descent_disabled"
                )
                ctx.panel._emit_log(_dhs_msg)
                ctx.append_trace(_dhs_msg)
            elif _dhs_z_gap > 0.010:
                _dhs_msg = (
                    "[PICK][DIRECT][GRASP_DOWN_FALLBACK] "
                    f"reason=dh_false_satisfied "
                    f"joint_delta_sum={_dhs_sum_delta:.4f} "
                    f"z_gap={_dhs_z_gap:.3f} "
                    "strategy=permissive_direct_descent"
                )
                ctx.panel._emit_log(_dhs_msg)
                ctx.append_trace(_dhs_msg)
                try:
                    last_debug = ctx.move_tcp_direct(
                        label="GRASP_DOWN_JOINT",
                        target_tcp_runtime=target_base,
                        timeout_sec=max(
                            float(timeout_sec), ctx.move_sec + 3.0 + float(attempt)
                        ),
                        audit_target_source=f"{audit_target_source}:permissive_dhs",
                        target_pose_original=target_base,
                        target_frame_original="base_link",
                        rot_weight=_grasp_down_permissive_rot_weight(),
                        ik_err_tol=_grasp_down_permissive_ik_err_tol(),
                        joint_weight=_grasp_down_permissive_joint_weight(),
                        force_send=True,
                    )
                    last_route = "permissive_direct_descent"
                    actual_after = ctx.tuple3(ctx.live_tcp_base())
                    object_after = ctx.tuple3(ctx.live_object_base()) or object_now
                    q_after = [float(v) for v in (ctx.current_joint_seed() or [])]
                except Exception as _dhs_exc:
                    ctx.panel._emit_log(
                        "[PICK][DIRECT][GRASP_DOWN] "
                        f"permissive_dhs_failed:{_dhs_exc}"
                    )
        # ─────────────────────────────────────────────────────────────────
        if isinstance(last_debug, dict):
            if q_after:
                last_debug["q_after"] = q_after
            if not last_debug.get("seed") and phase_seed_joints:
                last_debug["seed"] = [float(v) for v in phase_seed_joints]
        last_metrics = ctx.grasp_down_runtime_metrics(
            target_base=target_base,
            tcp_base=actual_after,
            obj_base=object_after,
        )
        pose_consistency = ctx.pose_consistency_metrics(
            phase="GRASP_DOWN_JOINT",
            tcp_base=actual_after,
            target_base=target_base,
        )
        last_metrics["pose_consistency"] = _json_safe(pose_consistency)
        runtime_ok = bool((last_debug or {}).get("runtime_target_ok"))
        xy_err_target = last_metrics.get("xy_err_target")
        z_err_target = last_metrics.get("z_err_target")
        target_dist = last_metrics.get("target_dist")
        ok = bool(
            xy_err_target is not None
            and z_err_target is not None
            and target_dist is not None
            and float(xy_err_target) <= strict_xy_tol
            and abs(float(z_err_target)) <= strict_z_tol
            and float(target_dist) <= target_dist_tol
        )
        quality = ctx.grasp_down_joint_quality(
            phase_seed_joints=phase_seed_joints,
            command_seed_joints=(last_debug or {}).get("seed"),
            final_joints=(last_debug or {}).get("q_after") or q_after,
            runtime_ok=runtime_ok,
            geometry_ok=bool(ok and pose_consistency.get("sources_ok")),
        )
        last_metrics["joint_quality"] = quality
        exec_msg = (
            "[PICK][DIRECT][GRASP_DOWN_EXEC] "
            f"mode={mode} target_xyz={ctx.fmt_vec(target_base)} "
            f"actual_before={ctx.fmt_vec(actual_before)} "
            f"actual_after={ctx.fmt_vec(actual_after)} "
            f"object_xyz={ctx.fmt_vec(object_after)}"
        )
        ctx.panel._emit_log(exec_msg)
        ctx.append_trace(exec_msg)
        result_msg = (
            "[PICK][DIRECT][GRASP_DOWN_RESULT] "
            f"xy_err={ctx.fmt_scalar(xy_err_target)}/{strict_xy_tol:.3f} "
            f"z_err={ctx.fmt_scalar(z_err_target)}/{strict_z_tol:.3f} "
            f"joint_err={ctx.fmt_scalar(quality.get('max_joint_delta'))}/{ctx.fmt_scalar(quality.get('sum_joint_delta'))} "
            f"max_joint_delta={ctx.fmt_scalar(quality.get('max_joint_delta'))} "
            f"sum_joint_delta={ctx.fmt_scalar(quality.get('sum_joint_delta'))} "
            f"joint_goal={json.dumps(_json_safe((last_debug or {}).get('ik_solution')), ensure_ascii=True)} "
            f"route={last_route} runtime_ok={str(runtime_ok).lower()} "
            f"result={'OK' if (ok and runtime_ok and bool(quality.get('branch_ok'))) else 'NO'}"
        )
        ctx.panel._emit_log(result_msg)
        ctx.append_trace(result_msg)
        branch_msg = (
            "[PICK][DIRECT][GRASP_DOWN_BRANCH] "
            f"seed_joints={json.dumps(_json_safe(quality.get('phase_seed_joints')), ensure_ascii=True)} "
            f"command_seed_joints={json.dumps(_json_safe(quality.get('command_seed_joints')), ensure_ascii=True)} "
            f"final_joints={json.dumps(_json_safe(quality.get('final_joints')), ensure_ascii=True)} "
            f"branch_change={str(bool(quality.get('branch_change'))).lower()} "
            f"max_joint_delta={ctx.fmt_scalar(quality.get('max_joint_delta'))} "
            f"sum_joint_delta={ctx.fmt_scalar(quality.get('sum_joint_delta'))} "
            f"critical_joints_delta={json.dumps(_json_safe(quality.get('critical_joints_delta')), ensure_ascii=True)}"
        )
        ctx.panel._emit_log(branch_msg)
        ctx.append_trace(branch_msg)
        quality_ok = bool(runtime_ok and ok and quality.get("branch_ok"))
        pose_sources_ok = bool(pose_consistency.get("sources_ok"))
        quality_msg = (
            "[PICK][DIRECT][GRASP_DOWN_JOINT_QUALITY] "
            f"runtime_ok={str(runtime_ok).lower()} "
            f"geometry_ok={str(ok).lower()} "
            f"pose_ok={str(pose_sources_ok).lower()} "
            f"branch_ok={str(bool(quality.get('branch_ok'))).lower()} "
            f"result={'OK' if (quality_ok and pose_sources_ok) else 'NO'} "
            f"reason={quality.get('reason')}"
        )
        ctx.panel._emit_log(quality_msg)
        ctx.append_trace(quality_msg)
        ctx.emit_pose_consistency(
            phase="GRASP_DOWN_JOINT",
            stage=f"attempt_{attempt}",
            metrics=pose_consistency,
        )
        visual_msg = (
            "[PICK][DIRECT][GRASP_DOWN_VISUAL] "
            f"tcp_frame={DIRECT_SOURCE_FRAME} actual_xyz={ctx.fmt_vec(actual_after)} "
            f"object_xyz={ctx.fmt_vec(object_after)} "
            f"note=step_by_step_operational_frame visual_frame={DIRECT_LEGACY_TCP_FRAME}"
        )
        ctx.panel._emit_log(visual_msg)
        ctx.append_trace(visual_msg)
        # Accept when both quality checks pass.
        # Also accept on geometry_only: if TCP is within strict geometric
        # tolerance (z_err < 25mm, xy_err < 12mm) AND no branch change,
        # even when pose_sources_ok=False or runtime_ok=False.  This handles
        # the DH/SDF FK divergence at the grasp height (~13mm) that makes
        # ctx.panel._last_tcp_base (DH FK) diverge from the live TF TCP by more
        # than source_tol=6mm, causing a spurious sources_ok=False.
        geometry_only_ok = bool(ok and quality.get("branch_ok"))
        if (quality_ok and pose_sources_ok) or geometry_only_ok:
            if quality_ok and pose_sources_ok:
                accept_note = "geometry_and_joint_quality_ok"
                decision = f"{last_route}:runtime_converged" if runtime_ok else f"{last_route}:metrics_converged"
            else:
                accept_note = "geometry_ok_fk_model_diverges_accepted"
                decision = f"{last_route}:geometry_converged"
            accept_msg = (
                "[PICK][DIRECT][GRASP_DOWN_ACCEPT] "
                f"route={last_route} reason={accept_note} "
                f"runtime_ok={str(runtime_ok).lower()} "
                f"pose_sources_ok={str(pose_sources_ok).lower()}"
            )
            ctx.panel._emit_log(accept_msg)
            ctx.append_trace(accept_msg)
            if _gd_seed_injected:
                os.environ.pop("PANEL_PICK_DEMO_IK_SEED_JOINTS", None)
            return last_debug, decision, last_metrics
        reject_reason = (
            "pose_source_mismatch"
            if not pose_sources_ok
            else quality.get("reason")
        )
        reject_msg = (
            "[PICK][DIRECT][GRASP_DOWN_REJECT] "
            f"route={last_route} reason={reject_reason} "
            f"max_joint_delta={ctx.fmt_scalar(quality.get('max_joint_delta'))} "
            f"sum_joint_delta={ctx.fmt_scalar(quality.get('sum_joint_delta'))} "
            f"critical_joints_delta={json.dumps(_json_safe(quality.get('critical_joints_delta')), ensure_ascii=True)}"
        )
        ctx.panel._emit_log(reject_msg)
        ctx.append_trace(reject_msg)
        if last_route == "permissive_direct_descent" and reject_reason == "visual_or_joint_pose_bad":
            _fb_msg = (
                "[PICK][DIRECT][GRASP_DOWN_FALLBACK] "
                "route=permissive_direct_descent rejected=true "
                "reason=visual_or_joint_pose_bad"
            )
            ctx.panel._emit_log(_fb_msg)
            ctx.append_trace(_fb_msg)
        if attempt < max_attempts:
            _fb_msg = (
                "[PICK][DIRECT][GRASP_DOWN_FALLBACK] "
                f"reason={reject_reason} "
                f"previous_xy_err={ctx.fmt_scalar(xy_err_target)} "
                f"strategy=retry_segmented_refine segments={len(waypoints)}"
            )
            ctx.panel._emit_log(_fb_msg)
            ctx.append_trace(_fb_msg)
    if _gd_seed_injected:
        os.environ.pop("PANEL_PICK_DEMO_IK_SEED_JOINTS", None)
    raise RuntimeError(
        "grasp_down_runtime_not_converged "
        f"xy_err={ctx.fmt_scalar(last_metrics.get('xy_err_target'))} "
        f"z_err={ctx.fmt_scalar(last_metrics.get('z_err_target'))} "
        f"route={last_route}"
    )
