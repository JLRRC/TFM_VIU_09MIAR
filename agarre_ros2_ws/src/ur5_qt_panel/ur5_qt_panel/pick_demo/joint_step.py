#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/joint_step.py
# Contenido: F3-step3e — extracción de _run_joint_step (307 LOC).
"""Joint-space step executor extraído del closure run_pick_demo.worker.

``run_joint_step`` ejecuta un movimiento joint-space del UR5 a un
target articular dado, con check de tolerancia local + chequeo
opcional de runtime target en base_link.

Antes de F3-step3e vivía como ``def`` anidada de 307 LOC dentro del
closure ``run_pick_demo.worker``, capturando 6 deps reales
(``panel`` + ``move_sec`` + 4 helpers/funcs del closure). Tenía 4
nested defs internas (`_local_joint_target_ok`, `_runtime_target_ok`,
`_strict_refine_runtime_status`, `_emit_strict_refine_runtime_log`)
que aquí permanecen INTERNAS de la función pura — closures legítimos
sobre el bloque actual.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Optional

from ur5_tools.gripper_geometry import RG2_PINCH_CENTER_FRAME, RG2_TCP_FRAME

# Frames canónicos (replicado de panel_pick_demo.py para evitar circular import)
DIRECT_SOURCE_FRAME = RG2_PINCH_CENTER_FRAME
DIRECT_LEGACY_TCP_FRAME = RG2_TCP_FRAME


def _joint_error_snapshot(panel, joints) -> str:
    """Local copy: F3 extracted from panel_pick_demo to avoid circular import."""
    names = list(getattr(panel, "UR5_JOINT_NAMES", []) or []) or [
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
        curr = getattr(panel, "_last_joint_positions", {}).get(name)
        if curr is None:
            parts.append(f"{name}=n/a")
            continue
        diff = abs(float(curr) - float(joints[idx]))
        parts.append(f"{name}={diff:.3f}")
    return " ".join(parts)


from ..directo_geometry import (
    _is_demo_basket_transport_motion,
    _is_demo_basket_transport_stage,
    _joint_step_wait_timeout,
    _pick_demo_env_float,
)
from .internal_helpers import _wait_for_demo_runtime_target_progress


@dataclass
class JointStepContext:
    """Captura de dependencias closure de run_joint_step."""

    panel: Any
    live_tcp_base: Callable[[], Any]
    dist: Callable[[Any, Any], Any]
    append_trace: Callable[[str], None]
    move_sec: float
    tuple3: Callable[[Any], Any]
    fmt_vec: Callable[[Any], str]
    fmt_scalar: Callable[..., str]
    get_pick_demo_params: Callable[[], Any]
    direct_runtime_target_tol_m: Callable[[Any], float]
    iso_now: Callable[[], str]
    ur5_joint_names: Any
    angle_shortest_diff_rad: Callable[[float, float], float]
    get_global_step_timeout_extra: Callable[[], float]
    should_apply_global_step_timeout_extra: Callable[[Any], bool]


def _run_joint_step_basket_transport_grace(
    ctx,
    *,
    label: str,
    runtime_target_base,
    runtime_target_tol_m,
) -> bool:
    """F3-step28b: ventana extra de gracia para fases basket_transport (~36 LOC).

    Si _wait_for_demo_runtime_target_progress devuelve OK marca runtime_target
    como accept_source y devuelve True. False si timeout/dist > tol.
    """
    extra_runtime_wait_sec = _pick_demo_env_float(
        "PANEL_PICK_DEMO_TRANSPORT_RUNTIME_GRACE_SEC",
        35.0,
        minimum=0.0,
    )
    if extra_runtime_wait_sec <= 0.0:
        return False
    runtime_target_tol = float(
        runtime_target_tol_m
        if runtime_target_tol_m is not None
        else ctx.direct_runtime_target_tol_m(label)
    )
    wait_fn = getattr(ctx.panel, "_wait_for_tcp_base_target", None)
    runtime_grace = _wait_for_demo_runtime_target_progress(
        ctx.panel,
        label=label,
        target_xyz=runtime_target_base,
        timeout_sec=extra_runtime_wait_sec,
        tol_xyz_m=runtime_target_tol,
        live_tcp_base_fn=ctx.live_tcp_base,
        fallback_wait_fn=wait_fn,
        ee_frame=DIRECT_SOURCE_FRAME,
    )
    runtime_grace_ok = bool(runtime_grace.get("ok"))
    runtime_grace_pos = ctx.tuple3(runtime_grace.get("pos"))
    runtime_grace_dist = runtime_grace.get("dist_m")
    ctx.panel._emit_log(
        "[PICK][DIRECT][ROUTE] "
        f"phase={label} runtime_transport_grace_ok={str(bool(runtime_grace_ok)).lower()} "
        f"timeout_sec={extra_runtime_wait_sec:.1f} "
        f"target_tol={runtime_target_tol:.3f} "
        f"runtime_target_dist={ctx.fmt_scalar(runtime_grace_dist)} "
        f"runtime_target_pos={ctx.fmt_vec(ctx.tuple3(runtime_grace_pos))} "
        f"reason={runtime_grace.get('reason', 'unknown')} "
        f"best_dist={ctx.fmt_scalar(runtime_grace.get('best_dist_m'))} "
        f"elapsed_sec={ctx.fmt_scalar(runtime_grace.get('elapsed_sec'))}"
    )
    if runtime_grace_ok:
        ctx.panel._pick_demo_last_joint_target_accept_source = "runtime_target"
    return runtime_grace_ok


def _run_joint_step_retry_for_recovery_labels(
    ctx,
    *,
    label: str,
    label_name: str,
    joints,
    effective_move_sec: float,
    wait_timeout: float,
    tol_rad: float,
    runtime_target_base,
    is_basket_transport_motion: bool,
    is_basket_transport_stage: bool,
    local_joint_target_ok_fn,
    runtime_target_ok_fn,
) -> bool:
    """F3-step28a: retry/recovery del joint step para labels nominales (~38 LOC).

    Para labels HOME/MESA/PICK_IMAGE/etc. o basket_transport_stage, reintenta
    publish_joint_trajectory + wait con tol relajada. Si OK marca accept_source.
    Devuelve True si el retry tuvo éxito (caller debe return), False si debe
    raise el RuntimeError de timeout.
    """
    ctx.panel._emit_log(
        f"[PICK][RECOVERY] {label} no alcanzado; reintentando una vez diffs={_joint_error_snapshot(ctx.panel, joints)}"
    )
    ok_retry, info_retry = ctx.panel._publish_joint_trajectory(joints, effective_move_sec)
    if not ok_retry:
        raise RuntimeError(f"{label} retry fallo: {info_retry}")
    retry_timeout = max(wait_timeout, effective_move_sec + 4.0)
    retry_tol = max(tol_rad, 0.10 if is_basket_transport_motion else 0.06)
    if ctx.panel._wait_for_joint_target(joints, retry_timeout, tol_rad=retry_tol):
        ctx.panel._pick_demo_last_joint_target_accept_source = "joint_wait_retry"
        ctx.panel._emit_log(f"[PICK][RECOVERY] {label} alcanzado tras reintento")
        return True
    local_ok_after_retry, local_diffs_after_retry = local_joint_target_ok_fn(retry_tol)
    runtime_ok_after_retry, runtime_info_after_retry = runtime_target_ok_fn()
    has_runtime_target = ctx.tuple3(runtime_target_base) is not None
    accept_via_runtime_target = bool(
        is_basket_transport_motion
        and has_runtime_target
        and runtime_ok_after_retry
    )
    if local_ok_after_retry or accept_via_runtime_target:
        ctx.panel._pick_demo_last_joint_target_accept_source = (
            "runtime_target"
            if (accept_via_runtime_target and not local_ok_after_retry)
            else "local_joint_state"
        )
        ctx.panel._emit_log(
            "[PICK][DIRECT][ROUTE] "
            f"phase={label} joint_target_accept_after_retry_timeout=true "
            f"source={'runtime_target' if (accept_via_runtime_target and not local_ok_after_retry) else 'local_joint_state'} "
            f"diffs={local_diffs_after_retry} {runtime_info_after_retry}"
        )
        return True
    return False


def run_joint_step(
    ctx: JointStepContext,
    label,
    joints,
    timeout_sec=None,
    tol_rad=0.02,
    runtime_target_base=None,
    runtime_target_tol_m=None,
    force_send=False,
    move_sec_override=None,
    apply_step_timeout_extra=True,
):
    """Ejecuta un step joint-space al UR5 con validación de tolerancias."""
    label_name = str(label or "").strip().upper()
    _strict_refine_target_base = ctx.tuple3(runtime_target_base)
    _strict_refine_runtime_label = (
        label_name == "APPROACH_COARSE_REFINE"
        and _strict_refine_target_base is not None
    )
    _strict_refine_runtime_tol = (
        float(
            runtime_target_tol_m
            if runtime_target_tol_m is not None
            else ctx.direct_runtime_target_tol_m(label)
        )
        if _strict_refine_runtime_label
        else None
    )
    ctx.panel._pick_demo_last_joint_target_accept_source = None

    def _local_joint_target_ok(local_tol_rad: float):
        snapshot = dict(getattr(ctx.panel, "_last_joint_positions", {}) or {})
        if not snapshot:
            return False, "no_local_joint_state"
        parts = []
        for idx, name in enumerate(ctx.ur5_joint_names):
            if idx >= len(joints):
                break
            curr = snapshot.get(name)
            if curr is None:
                parts.append(f"{name}=n/a")
                return False, " ".join(parts)
            diff = abs(ctx.angle_shortest_diff_rad(curr, joints[idx]))
            parts.append(f"{name}={diff:.3f}")
            if diff > float(local_tol_rad):
                return False, " ".join(parts)
        return True, " ".join(parts)

    def _runtime_target_ok() -> tuple[bool, str]:
        target_base_3 = ctx.tuple3(runtime_target_base)
        if target_base_3 is None:
            return True, "runtime_target=none"
        tcp_base_3 = ctx.tuple3(ctx.live_tcp_base())
        if tcp_base_3 is None:
            return False, "runtime_target=tcp_unavailable"
        tol_m = float(
            runtime_target_tol_m
            if runtime_target_tol_m is not None
            else ctx.direct_runtime_target_tol_m(label)
        )
        dist_m = ctx.dist(tcp_base_3, target_base_3)
        if dist_m is None:
            return False, "runtime_target=dist_unavailable"
        return bool(float(dist_m) <= tol_m), f"runtime_target_dist={float(dist_m):.3f}/{tol_m:.3f}"

    def _strict_refine_runtime_status():
        if not _strict_refine_runtime_label:
            return None
        tcp_base_3 = ctx.tuple3(ctx.live_tcp_base())
        if tcp_base_3 is None:
            return {
                "target_ok": False,
                "accept_result": False,
                "dist_m": None,
                "z_error_m": None,
                "tcp_base": None,
            }
        dist_m = ctx.dist(tcp_base_3, _strict_refine_target_base)
        z_error_m = abs(
            float(tcp_base_3[2]) - float(_strict_refine_target_base[2])
        )
        target_ok = bool(
            dist_m is not None
            and math.isfinite(float(dist_m))
            and float(dist_m) <= float(_strict_refine_runtime_tol)
        )
        accept_result = bool(
            target_ok
            and math.isfinite(float(z_error_m))
            and float(z_error_m) <= float(_strict_refine_runtime_tol)
        )
        return {
            "target_ok": target_ok,
            "accept_result": accept_result,
            "dist_m": float(dist_m) if dist_m is not None else None,
            "z_error_m": float(z_error_m),
            "tcp_base": ctx.tuple3(tcp_base_3),
        }

    def _emit_strict_refine_runtime_log(stage: str, joint_target_ok: bool):
        state = _strict_refine_runtime_status()
        if state is None:
            return None
        ctx.panel._emit_log(
            "[PICK][DIRECT][COARSE_REFINE_EXEC] "
            f"stage={stage} "
            "label=APPROACH_COARSE_REFINE "
            f"refine_joint_target_ok={str(bool(joint_target_ok)).lower()} "
            f"refine_runtime_target_ok={str(bool(state.get('target_ok'))).lower()} "
            f"refine_runtime_target_dist={ctx.fmt_scalar(state.get('dist_m'))} "
            f"refine_runtime_target_pos={ctx.fmt_vec(state.get('tcp_base'))} "
            f"refine_runtime_z_error={ctx.fmt_scalar(state.get('z_error_m'))} "
            f"refine_execution_accept_result={str(bool(state.get('accept_result'))).lower()}"
        )
        return state

    ctx.panel._emit_log(f"[PICK] Paso joint: {label}" + (" [FORCE_SEND]" if force_send else ""))
    local_ok_before, local_diffs_before = _local_joint_target_ok(tol_rad)
    runtime_ok_before, runtime_info_before = _runtime_target_ok()
    if local_ok_before and not force_send:
        _strict_refine_before = _emit_strict_refine_runtime_log(
            "before_publish",
            local_ok_before,
        )
        if runtime_ok_before and (
            not _strict_refine_runtime_label
            or bool((_strict_refine_before or {}).get("accept_result"))
        ):
            ctx.panel._pick_demo_last_joint_target_accept_source = "local_joint_state"
            ctx.panel._emit_log(
                "[PICK][DIRECT][ROUTE] "
                f"phase={label} joint_target_already_satisfied=true "
                f"source=local_joint_state diffs={local_diffs_before} {runtime_info_before}"
            )
            return
        ctx.panel._emit_log(
            "[PICK][DIRECT][ROUTE] "
            f"phase={label} joint_target_already_satisfied=false "
            "reason=runtime_target_not_reached "
            f"source=local_joint_state diffs={local_diffs_before} {runtime_info_before}"
        )
    elif local_ok_before and force_send:
        ctx.panel._emit_log(
            "[PICK][DIRECT][ROUTE] "
            f"phase={label} force_send=true skipping_early_exit "
            f"diffs={local_diffs_before} {runtime_info_before}"
        )
    effective_move_sec = float(
        ctx.move_sec
        if move_sec_override is None
        else max(0.5, float(move_sec_override))
    )
    ok, info = ctx.panel._publish_joint_trajectory(
        joints,
        effective_move_sec,
        prefer_action=_strict_refine_runtime_label,
    )
    if _strict_refine_runtime_label:
        ctx.panel._emit_log(
            "[PICK][DIRECT][ROUTE] "
            f"phase=APPROACH_COARSE_REFINE trajectory_dispatch={info} "
            f"prefer_action={str(bool(_strict_refine_runtime_label)).lower()}"
        )
    if not ok:
        raise RuntimeError(f"{label} fallo: {info}")
    try:
        _step_extra = ctx.get_pick_demo_params().step_timeout_extra_sec
    except Exception:
        _step_extra = 0.0
    is_basket_transport_stage = _is_demo_basket_transport_stage(label_name)
    is_basket_transport_motion = _is_demo_basket_transport_motion(label_name)
    apply_global_step_timeout_extra = ctx.should_apply_global_step_timeout_extra(
        label_name,
        requested=apply_step_timeout_extra,
    )
    if (
        apply_step_timeout_extra
        and not apply_global_step_timeout_extra
        and _step_extra > 0.0
    ):
        ctx.panel._emit_log(
            "[PICK][DIRECT][ROUTE] "
            f"phase={label} step_timeout_extra_skipped={_step_extra:.1f} "
            "reason=basket_transport_motion"
        )
    wait_timeout = _joint_step_wait_timeout(
        timeout_sec,
        effective_move_sec=effective_move_sec,
        step_timeout_extra_sec=_step_extra,
        apply_step_timeout_extra=apply_global_step_timeout_extra,
    )
    joint_wait_ok = ctx.panel._wait_for_joint_target(joints, wait_timeout, tol_rad=tol_rad)
    if joint_wait_ok and not _strict_refine_runtime_label:
        ctx.panel._pick_demo_last_joint_target_accept_source = "joint_wait"
        return
    if joint_wait_ok and _strict_refine_runtime_label:
        ctx.panel._pick_demo_last_joint_target_accept_source = "joint_wait"
        _emit_strict_refine_runtime_log("after_joint_wait", True)
    local_ok_after_wait, local_diffs_after_wait = _local_joint_target_ok(max(tol_rad, 0.02))
    runtime_ok_after_wait, runtime_info_after_wait = _runtime_target_ok()
    _strict_refine_after_wait = _emit_strict_refine_runtime_log(
        "after_wait_timeout",
        local_ok_after_wait,
    )
    if _strict_refine_runtime_label:
        if bool((_strict_refine_after_wait or {}).get("accept_result")):
            ctx.panel._pick_demo_last_joint_target_accept_source = "runtime_target"
            ctx.panel._emit_log(
                "[PICK][DIRECT][ROUTE] "
                "phase=APPROACH_COARSE_REFINE "
                "refine_execution_accept_after_wait_timeout=true "
                f"diffs={local_diffs_after_wait} {runtime_info_after_wait}"
            )
            return
        ctx.panel._emit_log(
            "[PICK][DIRECT][ROUTE] "
            "phase=APPROACH_COARSE_REFINE "
            "refine_execution_accept_after_wait_timeout=false "
            "action=defer_to_move_tcp_direct_runtime_settle"
        )
        return
    if local_ok_after_wait and runtime_ok_after_wait:
        ctx.panel._pick_demo_last_joint_target_accept_source = "local_joint_state"
        ctx.panel._emit_log(
            "[PICK][DIRECT][ROUTE] "
            f"phase={label} joint_target_accept_after_wait_timeout=true "
            f"source=local_joint_state diffs={local_diffs_after_wait} {runtime_info_after_wait}"
        )
        return
    if is_basket_transport_motion and runtime_target_base is not None and not runtime_ok_after_wait:
        if _run_joint_step_basket_transport_grace(
            ctx,
            label=label,
            runtime_target_base=runtime_target_base,
            runtime_target_tol_m=runtime_target_tol_m,
        ):
            return
    if (
        label in {"HOME", "MESA", "PICK_IMAGE", "PICK_PRE_CLOSE_REF", "HOME_WITH_OBJECT", "CESTA", "CESTA_RELEASE", "HOME_FINAL"}
        or is_basket_transport_stage
    ):
        if _run_joint_step_retry_for_recovery_labels(
            ctx,
            label=label,
            label_name=label_name,
            joints=joints,
            effective_move_sec=effective_move_sec,
            wait_timeout=wait_timeout,
            tol_rad=tol_rad,
            runtime_target_base=runtime_target_base,
            is_basket_transport_motion=is_basket_transport_motion,
            is_basket_transport_stage=is_basket_transport_stage,
            local_joint_target_ok_fn=_local_joint_target_ok,
            runtime_target_ok_fn=_runtime_target_ok,
        ):
            return
    raise RuntimeError(
        f"{label} no alcanzado (timeout) diffs={_joint_error_snapshot(ctx.panel, joints)}"
    )
