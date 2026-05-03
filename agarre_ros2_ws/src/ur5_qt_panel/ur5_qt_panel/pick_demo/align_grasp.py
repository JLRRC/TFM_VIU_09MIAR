#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/align_grasp.py
# Contenido: F3-step3c — extracción de _align_demo_grasp_direct (533 LOC).
"""Align demo grasp direct extraído del closure run_pick_demo.worker.

``align_demo_grasp_direct`` ejecuta la fase de **alineación pre-grasp**
(GRASP_ALIGN_IK) del Pick Demo: itera ``direct_ik_runtime_attempts``
veces resolviendo IK desde el TCP actual hasta el objetivo
``obj_xy + _DIRECTO_GRASP_Z + grasp_contact_z_offset_m`` con biases
de Z para corregir errores residuales entre intentos.

Antes de F3-step3c vivía como ``def`` anidada de 533 LOC dentro del
closure ``run_pick_demo.worker``, capturando 11 free vars (``panel`` +
6 helpers callables del closure + ``move_sec`` + 2 constantes/values
del frame + un dict mutable ``_direct_debug_state``) y mutando una
var ``z_alineada_alert_emitted`` vía ``nonlocal``.

Aquí se extrae con:

* ``AlignGraspContext`` dataclass: captura las 11 deps closure como
  callables/refs.
* ``AlignGraspState`` dataclass MUTABLE: contiene
  ``z_alineada_alert_emitted: bool`` (la única var nonlocal). El
  wrapper en el closure lee/escribe este campo antes/después de la
  llamada para preservar la semántica original.
* ``align_demo_grasp_direct(ctx, state) -> dict | None``: cuerpo
  legacy 1:1 con sustituciones name→ctx.X / state.X. Returns
  preservados.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Optional

from .geometry import vec_dist3 as _dist
# Resto de helpers module-level (fmt_vec, fmt_scalar, tuple3, params,
# frames) se inyectan vía ctx.<fn> para evitar acoplamiento de imports.


@dataclass
class AlignGraspContext:
    """Captura de dependencias closure del align_demo_grasp_direct."""

    panel: Any
    live_object_base: Callable[[], Any]
    live_tcp_base: Callable[[], Any]
    move_tcp_direct: Callable[..., Any]
    append_trace: Callable[[str], None]
    resolved_align_object_base: Callable[[], Any]
    pre_close_alignment_metrics: Callable[[], dict]
    direct_debug_state: dict  # mutable
    move_sec: float
    directo_grasp_z: float
    grasp_contact_z_offset_m: float
    # Helpers module-level inyectados (todos disponibles vía import en
    # panel_pick_demo). Inyectarlos por ctx evita duplicar imports entre
    # módulos pick_demo/.
    get_pick_demo_params: Callable[[], Any]
    fmt_vec: Callable[[Any], str]
    fmt_scalar: Callable[..., str]
    tuple3: Callable[[Any], Any]
    direct_source_frame: str
    direct_execution_frame: str


@dataclass
class AlignGraspState:
    """Estado mutable que la fn pura puede modificar (originalmente nonlocal)."""

    z_alineada_alert_emitted: bool = False


def align_demo_grasp_direct(
    ctx: AlignGraspContext,
    state: AlignGraspState,
) -> Optional[dict]:
    """Ejecuta la fase GRASP_ALIGN_IK con retries + Z-bias.

    Devuelve ``dict`` con métricas del último intento exitoso o None
    si la sesión no produjo resultado. Lanza ``RuntimeError`` si la
    pose del objeto no está disponible al inicio.

    Sustituciones canónicas frente al legacy:
      * ``panel.X`` → ``ctx.panel.X``.
      * ``_live_object_base()`` → ``ctx.live_object_base()``.
      * helpers / vars del closure → ``ctx.<campo>``.
      * ``z_alineada_alert_emitted`` → ``state.z_alineada_alert_emitted``.
      * ``_get_pick_demo_params()`` → ``ctx.get_pick_demo_params()``.
      * ``_fmt_vec`` / ``_fmt_scalar`` / ``_tuple3`` → ``ctx.<fn>``.
      * ``DIRECT_SOURCE_FRAME`` / ``DIRECT_EXECUTION_FRAME`` → ctx.
    """
    max_attempts = max(
        1,
        int(
            ctx.get_pick_demo_params().direct_ik_runtime_attempts
        ),
    )
    align_z_residual_tol = max(
        0.003,
        ctx.get_pick_demo_params().align_z_residual_tol_m,
    )
    align_z_improve_min = max(
        0.001,
        ctx.get_pick_demo_params().align_z_improve_min_m,
    )
    align_z_bias_gain = max(
        0.0,
        ctx.get_pick_demo_params().align_z_bias_gain,
    )
    align_z_bias_cap_m = max(
        0.0,
        ctx.get_pick_demo_params().align_z_bias_cap_m,
    )
    last_metrics = None
    last_debug = None
    for attempt in range(1, max_attempts + 1):
        obj_base_live = ctx.live_object_base()
        obj_base, align_target_source, _align_target_extra = ctx.resolved_align_object_base()
        if obj_base is None:
            raise RuntimeError("demo_object_pose_unavailable_before_align")
        target_z_expected = float(obj_base[2]) + ctx.directo_grasp_z + ctx.grasp_contact_z_offset_m
        if abs(ctx.grasp_contact_z_offset_m) > 1e-9:
            ctx.panel._emit_log(
                "[GRASP_Z_FIX] phase=GRASP_ALIGN_IK "
                f"obj_z_base={float(obj_base[2]):.4f} "
                f"offset_applied={ctx.grasp_contact_z_offset_m:.4f} "
                f"target_z_corrected={target_z_expected:.4f}"
            )
        target_tcp_runtime_raw = (
            float(obj_base[0]),
            float(obj_base[1]),
            float(target_z_expected),
        )
        target_tcp_runtime = target_tcp_runtime_raw
        tcp_before = ctx.live_tcp_base()
        ctx.panel._emit_log(
            f"[GRASP_Z_TRACE] attempt={attempt} "
            f"obj_base_z={float(obj_base[2]):.4f} "
            f"ctx.grasp_contact_z_offset_m={ctx.grasp_contact_z_offset_m:.4f} "
            f"ctx.directo_grasp_z={ctx.directo_grasp_z:.4f} "
            f"target_z_expected={target_z_expected:.4f} "
            f"target_tcp_runtime_raw_z={float(target_tcp_runtime_raw[2]):.4f} "
            f"tcp_before_z={f'{float(tcp_before[2]):.4f}' if tcp_before else 'N/A'}"
        )
        pre_metrics = ctx.pre_close_alignment_metrics()
        xy_lock_factor = max(
            1.0,
            ctx.get_pick_demo_params().align_xy_lock_factor,
        )
        align_exit_xy_tol = max(
            0.004,
            ctx.get_pick_demo_params().align_exit_xy_tol_m,
        )
        align_exit_z_tol = max(
            0.006,
            ctx.get_pick_demo_params().align_exit_z_tol_m,
        )
        xy_tol_pre = float(pre_metrics.get("xy_tol") or 0.035)
        z_tol_pre = float(pre_metrics.get("z_tol") or 0.030)
        xy_dist_pre = (
            float(pre_metrics.get("xy_dist"))
            if pre_metrics.get("xy_dist") is not None
            else None
        )
        z_error_pre = (
            float(pre_metrics.get("z_error"))
            if pre_metrics.get("z_error") is not None
            else None
        )
        already_aligned = bool(
            xy_dist_pre is not None
            and z_error_pre is not None
            and xy_dist_pre <= align_exit_xy_tol
            and z_error_pre <= align_exit_z_tol
        )
        if already_aligned:
            ctx.panel._emit_log(
                "[PICK][DIRECT][ALIGN_TRACE] "
                f"attempt={attempt}/{max_attempts} "
                "align_target_frame=base_link "
                f"align_target_pose={ctx.fmt_vec(target_tcp_runtime_raw)} "
                f"align_current_tcp={ctx.fmt_vec(tcp_before)} "
                f"align_object_pose={ctx.fmt_vec(obj_base)} "
                f"align_object_pose_live={ctx.fmt_vec(obj_base_live)} "
                "align_offset_vector=skip_move_already_aligned "
                "align_reached_condition=true "
                "align_timeout_reason=none "
                f"align_target_source={align_target_source}"
            )
            return {
                "label": "GRASP_ALIGN_IK",
                "runtime_target_ok": True,
                "runtime_target_dist": float(pre_metrics.get("tcp_obj_dist") or 0.0),
                "target_tcp_runtime": ctx.tuple3(target_tcp_runtime_raw),
                "target_source_frame": ctx.direct_source_frame,
                "target_execution_frame": ctx.direct_execution_frame,
                "offset_vector": None,
            }
        z_residual_pre = None
        if tcp_before is not None:
            z_residual_pre = float(tcp_before[2]) - float(target_z_expected)
        decision = "full_xy_z"
        keep_xy = bool(
            tcp_before is not None
            and xy_dist_pre is not None
            and xy_dist_pre <= (xy_tol_pre * xy_lock_factor)
        )
        ctx.direct_debug_state["keep_xy"] = bool(keep_xy)
        if keep_xy:
            # Si el TCP ya esta encima en XY, evitar correccion lateral
            # que pueda degradar la alineacion visual; solo corregir Z.
            target_tcp_runtime = (
                float(tcp_before[0]),
                float(tcp_before[1]),
                float(target_tcp_runtime_raw[2]),
            )
            decision = "z_only_keep_xy"
        z_bias_cmd = 0.0
        if (
            attempt > 1
            and z_residual_pre is not None
            and abs(float(z_residual_pre)) > align_z_residual_tol
        ):
            z_bias_cmd = max(
                -align_z_bias_cap_m,
                min(align_z_bias_cap_m, float(z_residual_pre) * align_z_bias_gain),
            )
            target_z_bias = float(target_z_expected) - float(z_bias_cmd)
            if keep_xy and tcp_before is not None:
                target_tcp_runtime = (
                    float(tcp_before[0]),
                    float(tcp_before[1]),
                    float(target_z_bias),
                )
                decision = "z_only_keep_xy_bias"
            else:
                target_tcp_runtime = (
                    float(target_tcp_runtime_raw[0]),
                    float(target_tcp_runtime_raw[1]),
                    float(target_z_bias),
                )
                decision = "full_xy_z_bias"
        delta_raw = None
        delta_used = None
        # La conversión Rz(π) base_link→base_link_inertia ocurre dentro de
        # _resolve_direct_execution_target (llamada por ctx.move_tcp_direct).
        # target_tcp_runtime aquí está en base_link; la negación X/Y se aplica
        # más adelante, dentro del solver IK, no en este bloque.
        if tcp_before is not None:
            delta_raw = (
                float(target_tcp_runtime_raw[0]) - float(tcp_before[0]),
                float(target_tcp_runtime_raw[1]) - float(tcp_before[1]),
                float(target_tcp_runtime_raw[2]) - float(tcp_before[2]),
            )
            delta_used = (
                float(target_tcp_runtime[0]) - float(tcp_before[0]),
                float(target_tcp_runtime[1]) - float(tcp_before[1]),
                float(target_tcp_runtime[2]) - float(tcp_before[2]),
            )
        live_anchor_dx = (
            float(obj_base_live[0]) - float(obj_base[0])
            if obj_base_live is not None and obj_base is not None
            else None
        )
        live_anchor_dy = (
            float(obj_base_live[1]) - float(obj_base[1])
            if obj_base_live is not None and obj_base is not None
            else None
        )
        live_anchor_dz = (
            float(obj_base_live[2]) - float(obj_base[2])
            if obj_base_live is not None and obj_base is not None
            else None
        )
        live_anchor_norm = (
            math.sqrt(live_anchor_dx**2 + live_anchor_dy**2 + live_anchor_dz**2)
            if live_anchor_dx is not None
            else None
        )
        ctx.panel._emit_log(
            "[PICK][DIRECT][IK_GEOM] "
            f"attempt={attempt}/{max_attempts} "
            f"tcp_before={ctx.fmt_vec(tcp_before)} "
            f"obj_base={ctx.fmt_vec(obj_base)} "
            f"obj_base_live={ctx.fmt_vec(obj_base_live)} "
            f"target_tcp_raw={ctx.fmt_vec(target_tcp_runtime_raw)} "
            f"target_tcp_used={ctx.fmt_vec(target_tcp_runtime)} "
            f"delta_raw={ctx.fmt_vec(delta_raw)} "
            f"delta_used={ctx.fmt_vec(delta_used)} "
            f"xy_dist_pre={ctx.fmt_scalar(xy_dist_pre)} "
            f"z_error_pre={ctx.fmt_scalar(z_error_pre)} "
            f"z_residual_pre={ctx.fmt_scalar(z_residual_pre)} "
            f"target_z_expected={target_z_expected:.3f} "
            f"z_bias_cmd={z_bias_cmd:.3f} "
            f"align_z_residual_tol={align_z_residual_tol:.3f} "
            f"xy_tol_pre={xy_tol_pre:.3f} "
            f"z_tol_pre={z_tol_pre:.3f} "
            f"xy_lock_factor={xy_lock_factor:.2f} "
            f"align_target_source={align_target_source} "
            f"live_anchor_delta=({ctx.fmt_scalar(live_anchor_dx)},{ctx.fmt_scalar(live_anchor_dy)},{ctx.fmt_scalar(live_anchor_dz)}) "
            f"live_anchor_norm={ctx.fmt_scalar(live_anchor_norm)} "
            f"decision={decision}"
        )
        ctx.panel._emit_log(
            "[PICK][DIRECT][ALIGN_TRACE] "
            f"attempt={attempt}/{max_attempts} "
            "align_target_frame=base_link "
            f"align_target_pose={ctx.fmt_vec(target_tcp_runtime)} "
            f"align_current_tcp={ctx.fmt_vec(tcp_before)} "
            f"align_object_pose={ctx.fmt_vec(obj_base)} "
            f"align_object_pose_live={ctx.fmt_vec(obj_base_live)} "
            "align_offset_vector=deferred_to_direct_ik "
            f"align_error_xyz={ctx.fmt_vec(delta_used)} "
            f"align_error_norm={ctx.fmt_scalar(_dist(target_tcp_runtime, tcp_before) if tcp_before is not None else None)} "
            "align_reached_condition=pre_move_false "
            "align_timeout_reason=none "
            f"align_target_source={align_target_source}"
        )
        # ----------------------------------------------------------
        # [COMPARE] Directo live vs Directo2 preset — en cada intento de align
        # ----------------------------------------------------------
        try:
            _cmp_obj = obj_base  # base_link
            _cmp_tcp = tcp_before  # rg2_pinch_center en base_link (live)
            _cmp_target = target_tcp_runtime  # target computado en base_link
            _cmp_xy = None
            _cmp_dz = None
            _cmp_3d = None
            if _cmp_tcp is not None and _cmp_obj is not None:
                _dxx = float(_cmp_tcp[0]) - float(_cmp_obj[0])
                _dyy = float(_cmp_tcp[1]) - float(_cmp_obj[1])
                _dzz = float(_cmp_tcp[2]) - float(_cmp_obj[2])
                _cmp_xy = math.sqrt(_dxx ** 2 + _dyy ** 2)
                _cmp_dz = _dzz
                _cmp_3d = math.sqrt(_dxx ** 2 + _dyy ** 2 + _dzz ** 2)
            ctx.panel._emit_log(
                "[COMPARE][OBJECT] "
                f"attempt={attempt}/{max_attempts} "
                f"obj_base_link={ctx.fmt_vec(_cmp_obj)} "
                f"obj_z_tgt={target_z_expected:.3f} "
                f"gripper_tcp_z_offset={float(GRIPPER_TCP_Z_OFFSET):.3f}"
            )
            ctx.panel._emit_log(
                "[COMPARE][PINCH] "
                f"attempt={attempt}/{max_attempts} "
                f"ee_frame={ctx.direct_source_frame} "
                f"tcp_base_link={ctx.fmt_vec(_cmp_tcp)} "
                f"xy_dist_to_obj={_cmp_xy:.4f if _cmp_xy is not None else 'N/A'} "
                f"dz_to_obj={_cmp_dz:.4f if _cmp_dz is not None else 'N/A'} "
                f"dist3d_to_obj={_cmp_3d:.4f if _cmp_3d is not None else 'N/A'}"
            )
            ctx.panel._emit_log(
                "[COMPARE][TARGET] "
                f"attempt={attempt}/{max_attempts} "
                f"target_base_link={ctx.fmt_vec(_cmp_target)} "
                f"source_frame={ctx.direct_source_frame} "
                f"ik_frame={ctx.direct_execution_frame} "
                f"decision={decision}"
            )
            _div_xy = _cmp_xy
            _div_dz = (
                abs(_cmp_dz - (ctx.directo_grasp_z + ctx.grasp_contact_z_offset_m))
                if _cmp_dz is not None else None
            )
            ctx.panel._emit_log(
                "[COMPARE][DIVERGENCE] "
                f"attempt={attempt}/{max_attempts} "
                f"xy_dist_tcp_obj={_div_xy:.4f if _div_xy is not None else 'N/A'} "
                f"dz_error_vs_offset={_div_dz:.4f if _div_dz is not None else 'N/A'} "
                f"gripper_tcp_z_offset={float(GRIPPER_TCP_Z_OFFSET):.3f} "
                "note=xy_should_be_lt_0.020_dz_error_should_be_lt_0.015"
            )
            _rc = "OK_WITHIN_TOLERANCE" if (
                _div_xy is not None and _div_xy < 0.020
            ) else "XY_MISALIGNED_PRESET_OR_IK_DRIFT"
            ctx.panel._emit_log(
                f"[COMPARE][ROOT_CAUSE] "
                f"attempt={attempt}/{max_attempts} "
                f"verdict={_rc} "
                f"xy_tol=0.020 "
                f"route={DIRECT_ROUTE_MODE} "
                f"source_frame={ctx.direct_source_frame}"
            )
        except Exception as _exc_cmp:
            ctx.panel._emit_log(f"[COMPARE][DIVERGENCE] error={_exc_cmp}")
        # ----------------------------------------------------------
        ctx.panel._emit_log(
            f"[GRASP_Z_TRACE] pre_move attempt={attempt} decision={decision} "
            f"target_tcp_runtime_z={float(target_tcp_runtime[2]):.4f} "
            f"target_tcp_runtime_raw_z={float(target_tcp_runtime_raw[2]):.4f} "
            f"z_bias_cmd={z_bias_cmd:.4f} "
            f"keep_xy={str(keep_xy).lower()}"
        )
        try:
            last_debug = ctx.move_tcp_direct(
                label="GRASP_ALIGN_IK",
                target_tcp_runtime=target_tcp_runtime,
                timeout_sec=ctx.move_sec + 8.0,
                audit_target_source=align_target_source,
                target_pose_original=obj_base,
                target_frame_original="base_link",
                rot_weight=max(
                    0.0,
                    ctx.get_pick_demo_params().align_rot_weight,
                ),
                ik_err_tol=max(
                    0.035,
                    ctx.get_pick_demo_params().align_ik_err_tol,
                ),
                joint_weight=max(
                    0.0,
                    ctx.get_pick_demo_params().align_ik_seed_weight,
                ),
                # GRASP_ALIGN_IK is the last geometric refinement before
                # closing.  If we skip publishing because the joints are
                # merely "close enough" in joint space, the visible pinch
                # center can stay 10-15 mm above the object and the grasp
                # remains visually wrong despite a valid IK target.
                force_send=True,
            )
        except Exception as exc:
            tcp_fail = ctx.live_tcp_base()
            fail_err_xyz = None
            fail_err_norm = None
            if tcp_fail is not None:
                fail_err_xyz = (
                    float(target_tcp_runtime[0]) - float(tcp_fail[0]),
                    float(target_tcp_runtime[1]) - float(tcp_fail[1]),
                    float(target_tcp_runtime[2]) - float(tcp_fail[2]),
                )
                fail_err_norm = _dist(target_tcp_runtime, tcp_fail)
            ctx.panel._emit_log(
                "[PICK][DIRECT][ALIGN_TRACE] "
                f"attempt={attempt}/{max_attempts} "
                "align_target_frame=base_link "
                f"align_target_pose={ctx.fmt_vec(target_tcp_runtime)} "
                f"align_current_tcp={ctx.fmt_vec(tcp_fail)} "
                f"align_object_pose={ctx.fmt_vec(obj_base)} "
                f"align_object_pose_live={ctx.fmt_vec(obj_base_live)} "
                "align_offset_vector=direct_ik_logged_separately "
                f"align_error_xyz={ctx.fmt_vec(fail_err_xyz)} "
                f"align_error_norm={ctx.fmt_scalar(fail_err_norm)} "
                "align_reached_condition=false "
                f"align_timeout_reason={exc} "
                f"align_target_source={align_target_source}"
            )
            # If PRE_CLOSE is already satisfied despite the joint timeout,
            # keep the sequence moving instead of aborting a valid grasp.
            _pre_fail_metrics = ctx.pre_close_alignment_metrics()
            _pre_fail_xy = _pre_fail_metrics.get("xy_dist")
            _pre_fail_z = _pre_fail_metrics.get("z_error")
            _pre_fail_exit_ok = bool(
                _pre_fail_xy is not None
                and _pre_fail_z is not None
                and float(_pre_fail_xy) <= align_exit_xy_tol
                and float(_pre_fail_z) <= align_exit_z_tol
            )
            if _pre_fail_exit_ok:
                ctx.panel._emit_log(
                    "[PICK][DIRECT][ALIGN_TRACE] "
                    f"attempt={attempt}/{max_attempts} "
                    "align_graceful_fallback=strict_alignment_ok_despite_joint_timeout "
                    f"align_target_source={align_target_source} "
                    f"xy_dist={ctx.fmt_scalar(_pre_fail_metrics.get('xy_dist'))} "
                    f"z_error={ctx.fmt_scalar(_pre_fail_metrics.get('z_error'))}"
                )
                last_debug = {}
                last_metrics = _pre_fail_metrics
                break
            raise
        last_metrics = ctx.pre_close_alignment_metrics()
        runtime_ok = bool((last_debug or {}).get("runtime_target_ok"))
        tcp_after = ctx.live_tcp_base()
        obj_after = ctx.live_object_base()
        z_error_after = None
        target_tcp_used_z = float(target_tcp_runtime[2])
        tcp_final_z = None
        z_expected_after = None
        if obj_after is not None:
            z_expected_after = float(obj_after[2]) + ctx.directo_grasp_z + ctx.grasp_contact_z_offset_m
            if abs(ctx.grasp_contact_z_offset_m) > 1e-9:
                ctx.panel._emit_log(
                    "[GRASP_Z_FIX] phase=GRASP_ALIGN_IK_convergence "
                    f"obj_z={float(obj_after[2]):.4f} "
                    f"ctx.directo_grasp_z={ctx.directo_grasp_z:.4f} "
                    f"ctx.grasp_contact_z_offset_m={ctx.grasp_contact_z_offset_m:.4f} "
                    f"z_expected_after={z_expected_after:.4f}"
                )
        if tcp_after is not None:
            tcp_final_z = float(tcp_after[2])
        if z_expected_after is not None and tcp_final_z is not None:
            z_error_after = abs(float(tcp_final_z) - float(z_expected_after))
        xy_dist_after = (
            float(last_metrics.get("xy_dist"))
            if last_metrics.get("xy_dist") is not None
            else None
        )
        z_error_after_metrics = (
            float(last_metrics.get("z_error"))
            if last_metrics.get("z_error") is not None
            else None
        )
        z_error_after_cmp = (
            float(z_error_after)
            if z_error_after is not None
            else z_error_after_metrics
        )
        xy_after_ok = bool(
            xy_dist_after is not None
            and xy_dist_after <= align_exit_xy_tol
        )
        z_after_ok = bool(
            z_error_after_cmp is not None
            and z_error_after_cmp <= align_exit_z_tol
        )
        pose_sources_ok = bool(last_metrics.get("pose_source_ok"))
        tcp_motion_delta = _dist(tcp_before, tcp_after) if tcp_before is not None and tcp_after is not None else None
        no_effect_attempt = bool(
            tcp_motion_delta is not None
            and float(tcp_motion_delta) <= max(
                0.0015,
                ctx.get_pick_demo_params().align_no_effect_tol_m,
            )
        )
        convergence_ok = bool(runtime_ok and xy_after_ok and z_after_ok and pose_sources_ok)
        z_improved = bool(
            z_error_pre is not None
            and z_error_after_cmp is not None
            and (float(z_error_pre) - float(z_error_after_cmp)) >= align_z_improve_min
        )
        z_trace_msg = (
            "[PICK][DIRECT][IK_Z_TRACE] "
            f"attempt={attempt}/{max_attempts} "
            f"obj_base={ctx.fmt_vec(obj_base)} "
            f"tcp_before={ctx.fmt_vec(tcp_before)} "
            f"target_tcp_raw={ctx.fmt_vec(target_tcp_runtime_raw)} "
            f"target_tcp_used={ctx.fmt_vec(target_tcp_runtime)} "
            f"target_z_expected={ctx.fmt_scalar(target_z_expected)} "
            f"target_tcp_raw_z={ctx.fmt_scalar(target_tcp_runtime_raw[2])} "
            f"target_tcp_used_z={ctx.fmt_scalar(target_tcp_used_z)} "
            f"tcp_final_z={ctx.fmt_scalar(tcp_final_z)} "
            f"obj_expected_z={ctx.fmt_scalar(z_expected_after)} "
            f"z_error_before={ctx.fmt_scalar(z_error_pre)} "
            f"z_error_after={ctx.fmt_scalar(z_error_after_cmp)} "
            f"xy_dist_before={ctx.fmt_scalar(xy_dist_pre)} "
            f"xy_dist_after={ctx.fmt_scalar(xy_dist_after)} "
            f"pose_ok={str(pose_sources_ok).lower()} "
            f"tcp_motion_delta={ctx.fmt_scalar(tcp_motion_delta)} "
            f"retries_used={attempt - 1} "
            f"convergence_criterion=runtime_ok&&xy<={align_exit_xy_tol:.3f}&&z<={align_exit_z_tol:.3f} "
            f"convergence_ok={str(convergence_ok).lower()} "
            f"decision={decision}"
        )
        ctx.panel._emit_log(z_trace_msg)
        ctx.append_trace(z_trace_msg)
        align_err_xyz = None
        align_err_norm = None
        if tcp_after is not None:
            align_err_xyz = (
                float(target_tcp_runtime[0]) - float(tcp_after[0]),
                float(target_tcp_runtime[1]) - float(tcp_after[1]),
                float(target_tcp_runtime[2]) - float(tcp_after[2]),
            )
            align_err_norm = _dist(target_tcp_runtime, tcp_after)
        ctx.panel._emit_log(
            "[PICK][DIRECT][ALIGN_TRACE] "
            f"attempt={attempt}/{max_attempts} "
            "align_target_frame=base_link "
            f"align_target_pose={ctx.fmt_vec(target_tcp_runtime)} "
            f"align_current_tcp={ctx.fmt_vec(tcp_after)} "
            f"align_object_pose={ctx.fmt_vec(obj_base)} "
            f"align_object_pose_live={ctx.fmt_vec(obj_after)} "
            f"align_offset_vector={ctx.fmt_vec((last_debug or {}).get('offset_vector'))} "
            f"align_error_xyz={ctx.fmt_vec(align_err_xyz)} "
            f"align_error_norm={ctx.fmt_scalar(align_err_norm)} "
            f"align_reached_condition={str(convergence_ok).lower()} "
            "align_timeout_reason=none "
            f"align_target_source={align_target_source}"
        )
        ctx.panel._emit_log(
            "[PICK][DIRECT][IK_RUNTIME] "
            f"attempt={attempt}/{max_attempts} "
            f"runtime_ok={runtime_ok} "
            f"xy_dist={ctx.fmt_scalar(last_metrics.get('xy_dist'))} "
            f"z_error={ctx.fmt_scalar(last_metrics.get('z_error'))} "
            f"pose_ok={str(pose_sources_ok).lower()} "
            f"xy_error_tight={ctx.fmt_scalar(xy_dist_after)}/{align_exit_xy_tol:.3f} "
            f"z_error_tight={ctx.fmt_scalar(z_error_after_cmp)}/{align_exit_z_tol:.3f} "
            f"tcp_obj_dist={ctx.fmt_scalar(last_metrics.get('tcp_obj_dist'))} "
            f"ok_pre_close={bool(last_metrics.get('ok'))} "
            f"ok_align_z={str(convergence_ok).lower()}"
        )
        if no_effect_attempt and not convergence_ok:
            ctx.panel._emit_log(
                "[PICK][DIRECT][ALIGN_TRACE] "
                f"attempt={attempt}/{max_attempts} "
                "align_no_effect=true "
                f"tcp_motion_delta={ctx.fmt_scalar(tcp_motion_delta)} "
                f"xy_dist={ctx.fmt_scalar(xy_dist_after)} "
                f"z_error={ctx.fmt_scalar(z_error_after_cmp)}"
            )
        if convergence_ok and not state.z_alineada_alert_emitted and z_improved:
            state.z_alineada_alert_emitted = True
            z_msg = (
                "[PICK][DIRECT] AVISO: Z ALINEADA "
                f"attempt={attempt}/{max_attempts} "
                f"z_error_before={ctx.fmt_scalar(z_error_pre)} "
                f"z_error_after={ctx.fmt_scalar(z_error_after_cmp)} "
                f"xy_dist_after={ctx.fmt_scalar(xy_dist_after)} "
                f"target_z_expected={ctx.fmt_scalar(target_z_expected)} "
                f"tcp_final_z={ctx.fmt_scalar(tcp_final_z)}"
            )
            ctx.panel._emit_log(z_msg)
            ctx.append_trace(z_msg)
        if convergence_ok:
            return last_debug
    raise RuntimeError(
        "grasp_align_ik_runtime_not_aligned "
        f"attempts={max_attempts} "
        f"xy_dist={ctx.fmt_scalar((last_metrics or {}).get('xy_dist'))} "
        f"z_error={ctx.fmt_scalar((last_metrics or {}).get('z_error'))}"
    )
