#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/transport_replan.py
# Contenido: F3-step3b — extracción de _attempt_transport_replan (184 LOC).
"""Transport replan helper extraído del closure _move_tcp_direct.

Cuando la fase TRANSPORT del Pick Demo falla en una etapa intermedia,
``attempt_transport_replan`` reanuda el viaje desde la pose actual del
TCP hacia el target original, pero **dividido en sub-stages** más cortas
para evitar repetir el problema (típicamente jumps o singularidades).

Antes de F3-step3b vivía como ``def`` anidada dentro de ``_move_tcp_direct``
(que a su vez vive en el closure ``worker``), capturando 17 free vars
(panel + helpers + casi todos los kwargs de _move_tcp_direct). Aquí se
extrae a module-level con ``TransportReplanContext`` como único arg de
contexto + ``exec_exc`` específico de la llamada.

El wrapper local en _move_tcp_direct queda en ~10 LOC: construye el ctx
y delega. Los 4 callsites legacy NO cambian.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional


@dataclass
class TransportReplanContext:
    """Contexto que captura las dependencias closure de transport_replan.

    Atributos:
        panel: instancia ControlPanelV2 (para _emit_log).
        live_tcp_base_fn: callable() -> tuple3|None. Pose live TCP en base.
        move_tcp_direct_fn: callable. La función gigante _move_tcp_direct
            misma; necesaria porque el replan re-invoca el flujo entero
            sobre cada sub-stage. Inyectada para evitar referencia
            recursiva al construir el ctx.
        env_int_fn / env_float_fn / env_flag_fn: helpers de
            ``directo_geometry`` o equivalente para resolver env vars
            con conversión.
        compute_recovery_stages_fn: callable que produce la lista de
            sub-targets entre la pose live y el target original.
        compute_micro_recovery_fn: callable para el micro-recovery
            adicional en CESTA_STAGE_1_RECOVER_2.
        dist_fn: callable(a, b) -> float|None. Distancia 3D.
        tuple3_fn / fmt_vec_fn / fmt_scalar_fn: formatters / converter.
        label, audit_target_source, target_tcp_runtime_3,
            target_pose_original, target_frame_original, rot_weight,
            ik_err_tol, joint_weight, move_sec, timeout_sec,
            transport_replan_remaining, is_transport_stage:
            valores capturados del frame de _move_tcp_direct.
    """

    panel: Any
    live_tcp_base_fn: Callable[[], Any]
    move_tcp_direct_fn: Callable[..., Any]
    env_int_fn: Callable[..., int]
    env_float_fn: Callable[..., float]
    env_flag_fn: Callable[..., bool]
    compute_recovery_stages_fn: Callable[..., Any]
    compute_micro_recovery_fn: Callable[..., Any]
    dist_fn: Callable[[Any, Any], Any]
    tuple3_fn: Callable[[Any], Any]
    fmt_vec_fn: Callable[[Any], str]
    fmt_scalar_fn: Callable[..., str]
    label: str
    audit_target_source: Optional[str]
    target_tcp_runtime_3: Any
    target_pose_original: Any
    target_frame_original: Optional[str]
    rot_weight: float
    ik_err_tol: Any
    joint_weight: float
    move_sec: float
    timeout_sec: float
    transport_replan_remaining: int
    is_transport_stage: bool


def attempt_transport_replan(
    ctx: TransportReplanContext,
    exec_exc: BaseException,
) -> Any:
    """Replan del segmento TRANSPORT tras un fallo, dividido en sub-stages.

    Devuelve el resultado del último ``move_tcp_direct_fn`` exitoso o
    None si no procede el replan (no hay attempts disponibles, no hay
    sub-stages computables, o no es una stage TRANSPORT).
    """
    if not (ctx.is_transport_stage and ctx.transport_replan_remaining > 0):
        return None
    panel = ctx.panel
    label = ctx.label
    recovery_live_tcp = ctx.tuple3_fn(ctx.live_tcp_base_fn())
    recovery_min_stages = max(
        2,
        int(
            ctx.env_int_fn(
                "PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MIN_STAGES",
                2,
                minimum=2,
            )
        ),
    )
    recovery_max_stages = max(
        recovery_min_stages,
        int(
            ctx.env_int_fn(
                "PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MAX_STAGES",
                4,
                minimum=recovery_min_stages,
            )
        ),
    )
    try:
        recovery_targets = ctx.compute_recovery_stages_fn(
            recovery_live_tcp,
            ctx.target_tcp_runtime_3,
            min_remaining_dist_m=ctx.env_float_fn(
                "PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MIN_REMAINING_DIST_M",
                0.060,
                minimum=0.0,
            ),
            min_stages=recovery_min_stages,
            max_stage_dist_m=ctx.env_float_fn(
                "PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MAX_STAGE_DIST_M",
                0.050,
                minimum=0.01,
            ),
            max_stages=recovery_max_stages,
        )
    except Exception as recovery_exc:
        panel._emit_log(
            "[PICK][DIRECT][TRANSPORT_REPLAN] "
            f"label={label} status=skipped "
            f"reason=compute_failed:{recovery_exc}"
        )
        return None
    if not recovery_targets:
        return None
    remaining_dist_m = ctx.dist_fn(recovery_live_tcp, ctx.target_tcp_runtime_3)
    per_stage_timeout_sec = max(
        8.0,
        max(
            float(ctx.move_sec) + 2.0,
            float(ctx.timeout_sec) / float(len(recovery_targets)),
        ),
    )
    panel._emit_log(
        "[PICK][DIRECT][TRANSPORT_REPLAN] "
        f"label={label} status=start "
        f"reason={str(exec_exc)} "
        f"remaining_attempts={int(ctx.transport_replan_remaining)} "
        f"segments={len(recovery_targets)} "
        f"remaining_dist_m={ctx.fmt_scalar_fn(remaining_dist_m)} "
        f"live_tcp={ctx.fmt_vec_fn(recovery_live_tcp)} "
        f"target_tcp={ctx.fmt_vec_fn(ctx.target_tcp_runtime_3)} "
        f"segment_timeout_sec={per_stage_timeout_sec:.3f}"
    )
    recovery_result = {}
    for recovery_idx, recovery_target in enumerate(recovery_targets, start=1):
        recovery_label = f"{label}_RECOVER_{recovery_idx}"
        recovery_source = (
            f"{str(ctx.audit_target_source or 'runtime_target')}_replan_{recovery_idx}"
        )
        if (
            recovery_label == "CESTA_STAGE_1_RECOVER_2"
            and ctx.env_flag_fn(
                "PANEL_PICK_DEMO_TRANSPORT_RECOVER2_MICRO_ENABLED",
                True,
            )
        ):
            _attempt_recover2_micro(ctx, recovery_label, recovery_target, per_stage_timeout_sec, recovery_source)
        recovery_result = ctx.move_tcp_direct_fn(
            label=recovery_label,
            target_tcp_runtime=recovery_target,
            timeout_sec=per_stage_timeout_sec,
            audit_target_source=recovery_source,
            target_pose_original=ctx.target_pose_original,
            target_frame_original=ctx.target_frame_original,
            rot_weight=ctx.rot_weight,
            ik_err_tol=ctx.ik_err_tol,
            joint_weight=ctx.joint_weight,
            force_send=True,
            transport_replan_remaining=ctx.transport_replan_remaining - 1,
        )
    panel._emit_log(
        "[PICK][DIRECT][TRANSPORT_REPLAN] "
        f"label={label} status=ok "
        f"segments={len(recovery_targets)} "
        f"remaining_attempts={int(ctx.transport_replan_remaining) - 1}"
    )
    return recovery_result


def _attempt_recover2_micro(
    ctx: TransportReplanContext,
    recovery_label: str,
    recovery_target: Any,
    per_stage_timeout_sec: float,
    recovery_source: str,
) -> None:
    """Sub-helper: en CESTA_STAGE_1_RECOVER_2, intenta un micro-step
    previo al recovery completo para evitar quedarse "atascado" lejos
    del target intermedio. Best-effort, no levanta — sólo loggea.
    """
    panel = ctx.panel
    recovery_live_tcp_before_micro = ctx.tuple3_fn(ctx.live_tcp_base_fn())
    micro_step_m = ctx.env_float_fn(
        "PANEL_PICK_DEMO_TRANSPORT_RECOVER2_MICRO_STEP_M",
        0.015,
        minimum=0.010,
    )
    micro_min_remaining_m = ctx.env_float_fn(
        "PANEL_PICK_DEMO_TRANSPORT_RECOVER2_MICRO_MIN_REMAINING_M",
        0.040,
        minimum=0.0,
    )
    micro_target = ctx.compute_micro_recovery_fn(
        recovery_live_tcp_before_micro,
        recovery_target,
        step_m=micro_step_m,
        minimum_remaining_dist_m=micro_min_remaining_m,
    )
    if micro_target is None:
        return
    remaining_before_micro = ctx.dist_fn(
        recovery_live_tcp_before_micro,
        recovery_target,
    )
    panel._emit_log(
        "[PICK][DIRECT][RECOVER2_MICRO] "
        f"label={recovery_label} status=start "
        f"live_tcp_before={ctx.fmt_vec_fn(recovery_live_tcp_before_micro)} "
        f"target_before={ctx.fmt_vec_fn(recovery_target)} "
        f"micro_target={ctx.fmt_vec_fn(micro_target)} "
        f"remaining_dist_before={ctx.fmt_scalar_fn(remaining_before_micro)} "
        f"step_m={micro_step_m:.3f}"
    )
    try:
        ctx.move_tcp_direct_fn(
            label=f"{recovery_label}_MICRO",
            target_tcp_runtime=micro_target,
            timeout_sec=per_stage_timeout_sec,
            audit_target_source=f"{recovery_source}_micro",
            target_pose_original=ctx.target_pose_original,
            target_frame_original=ctx.target_frame_original,
            rot_weight=ctx.rot_weight,
            ik_err_tol=ctx.ik_err_tol,
            joint_weight=ctx.joint_weight,
            force_send=True,
            transport_replan_remaining=0,
        )
    except Exception as micro_exc:
        recovery_live_tcp_after_micro = ctx.tuple3_fn(ctx.live_tcp_base_fn())
        remaining_after_micro = ctx.dist_fn(
            recovery_live_tcp_after_micro,
            recovery_target,
        )
        panel._emit_log(
            "[PICK][DIRECT][RECOVER2_MICRO] "
            f"label={recovery_label} status=failed "
            f"reason={micro_exc} "
            f"live_tcp_after={ctx.fmt_vec_fn(recovery_live_tcp_after_micro)} "
            f"target_after={ctx.fmt_vec_fn(recovery_target)} "
            f"remaining_dist_after={ctx.fmt_scalar_fn(remaining_after_micro)}"
        )
    else:
        recovery_live_tcp_after_micro = ctx.tuple3_fn(ctx.live_tcp_base_fn())
        remaining_after_micro = ctx.dist_fn(
            recovery_live_tcp_after_micro,
            recovery_target,
        )
        progress_after_micro = None
        if (
            remaining_before_micro is not None
            and remaining_after_micro is not None
        ):
            progress_after_micro = (
                float(remaining_before_micro)
                - float(remaining_after_micro)
            )
        panel._emit_log(
            "[PICK][DIRECT][RECOVER2_MICRO] "
            f"label={recovery_label} status=ok "
            f"live_tcp_after={ctx.fmt_vec_fn(recovery_live_tcp_after_micro)} "
            f"target_after={ctx.fmt_vec_fn(recovery_target)} "
            f"remaining_dist_after={ctx.fmt_scalar_fn(remaining_after_micro)} "
            f"progress_m={ctx.fmt_scalar_fn(progress_after_micro)}"
        )
