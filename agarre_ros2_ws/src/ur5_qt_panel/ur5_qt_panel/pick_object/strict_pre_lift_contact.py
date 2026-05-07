#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_object/strict_pre_lift_contact.py
# Contenido: F3-step5bis-b — extracción de _ensure_strict_pre_lift_contact (137 LOC).
"""Strict pre-lift contact gate extraído del closure run_pick_object.worker.

``ensure_strict_pre_lift_contact`` valida que el gripper tiene contacto
físico real (apertura + esfuerzo dentro de bandas) antes de elevar.
Si no, intenta hasta N regrabs reduciendo Z incrementalmente.

Antes de F3-step5bis-b vivía como ``def`` anidada de 137 LOC dentro
del closure ``run_pick_object.worker``, capturando 7 deps reales
(panel + 4 callables del closure + strict_physics_mode + table_top_base)
+ 1 nonlocal (``last_metrics``). 2 callsites en el closure.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional


@dataclass
class StrictPreLiftContactContext:
    """Captura de dependencias closure de ensure_strict_pre_lift_contact."""

    panel: Any
    close_gripper_sync: Callable[..., Any]
    ensure_gripper_open_for_moveit: Callable[..., Any]
    read_gripper_contact_metrics: Callable[[], Any]
    run_moveit_step: Callable[..., Any]
    strict_physics_mode: bool
    table_top_base: Any
    get_pick_object_params: Callable[[], Any]


@dataclass
class StrictPreLiftContactState:
    """Estado mutable que la fn pura modifica (originalmente nonlocal)."""

    last_metrics: Optional[dict] = None


def ensure_strict_pre_lift_contact(
    ctx: StrictPreLiftContactContext,
    state: StrictPreLiftContactState,
    grasp_pose_live: dict,
    grasp_delay_live: float,
) -> None:
    """Valida contacto físico antes de LIFT, regrasps si falla."""
    if not ctx.strict_physics_mode:
        return
    enabled = ctx.get_pick_object_params().strict_contact_gate_enable
    if not enabled:
        return
    try:
        settle_sec = ctx.get_pick_object_params().strict_contact_settle_sec
    except Exception:
        settle_sec = 0.7
    try:
        min_blocked_opening_m = ctx.get_pick_object_params().strict_min_blocked_opening_m
    except Exception:
        min_blocked_opening_m = 0.050
    try:
        max_blocked_opening_m = ctx.get_pick_object_params().strict_max_blocked_opening_m
    except Exception:
        max_blocked_opening_m = 0.850
    try:
        min_effort_abs = ctx.get_pick_object_params().strict_min_effort_abs
    except Exception:
        min_effort_abs = 0.0
    try:
        max_regrasp = ctx.get_pick_object_params().strict_regrasp_max
    except Exception:
        max_regrasp = 1
    try:
        regrasp_step_m = ctx.get_pick_object_params().strict_regrasp_step_m
    except Exception:
        regrasp_step_m = 0.004
    try:
        regrasp_floor_margin_m = ctx.get_pick_object_params().strict_regrasp_floor_margin_m
    except Exception:
        regrasp_floor_margin_m = 0.008

    settle_sec = max(0.2, settle_sec)
    max_regrasp = max(0, min(max_regrasp, 3))
    regrasp_step_m = max(0.001, regrasp_step_m)
    min_z = float(ctx.table_top_base) + max(0.004, regrasp_floor_margin_m)
    state.last_metrics = None

    def _wait_for_contact_signal(attempt_idx: int) -> bool:
        # F3-step5bis-b: nonlocal moved to state
        deadline = time.time() + settle_sec
        best_opening = None
        best_effort = None
        saw_joint_state = False
        while time.time() < deadline:
            metrics = ctx.read_gripper_contact_metrics()
            state.last_metrics = metrics
            saw_joint_state = saw_joint_state or bool(metrics.get("has_joint_state"))
            opening_m = metrics.get("opening_m")
            effort_abs = metrics.get("effort_abs_max")
            if opening_m is not None:
                best_opening = float(opening_m) if best_opening is None else max(best_opening, float(opening_m))
            if effort_abs is not None:
                best_effort = float(effort_abs) if best_effort is None else max(best_effort, float(effort_abs))
            opening_ok = (
                opening_m is not None
                and float(opening_m) >= min_blocked_opening_m
                and float(opening_m) <= max_blocked_opening_m
            )
            effort_ok = (
                min_effort_abs > 0.0
                and effort_abs is not None
                and float(effort_abs) >= min_effort_abs
            )
            if opening_ok or effort_ok:
                ctx.panel._emit_log(
                    f"[PICK_OBJ][STRICT_CONTACT] ok attempt={attempt_idx} "
                    f"opening_m={0.0 if opening_m is None else float(opening_m):.4f} "
                    f"min_opening_m={min_blocked_opening_m:.4f} "
                    f"max_opening_m={max_blocked_opening_m:.4f} "
                    f"effort_abs={0.0 if effort_abs is None else float(effort_abs):.4f} "
                    f"min_effort_abs={min_effort_abs:.4f}"
                )
                return True
            time.sleep(0.05)
        if not saw_joint_state:
            ctx.panel._emit_log(
                "[PICK_OBJ][STRICT_CONTACT] joint_state unavailable; skipping pre-lift contact gate"
            )
            return True
        ctx.panel._emit_log(
            f"[PICK_OBJ][STRICT_CONTACT] no_contact attempt={attempt_idx} "
            f"opening_m={0.0 if best_opening is None else float(best_opening):.4f} "
            f"min_opening_m={min_blocked_opening_m:.4f} "
            f"max_opening_m={max_blocked_opening_m:.4f} "
            f"effort_abs={0.0 if best_effort is None else float(best_effort):.4f} "
            f"min_effort_abs={min_effort_abs:.4f}"
        )
        return False

    for attempt_idx in range(0, max_regrasp + 1):
        if _wait_for_contact_signal(attempt_idx):
            return
        if attempt_idx >= max_regrasp:
            break
        cur_pos = grasp_pose_live.get("position", (bx, by, bz))
        cur_x = float(cur_pos[0])
        cur_y = float(cur_pos[1])
        cur_z = float(cur_pos[2])
        next_z = max(min_z, cur_z - regrasp_step_m)
        if next_z >= (cur_z - 1e-4):
            ctx.panel._emit_log(
                f"[PICK_OBJ][STRICT_CONTACT] regrasp blocked cur_z={cur_z:.3f} min_z={min_z:.3f}"
            )
            break
        ctx.panel._emit_log(
            f"[PICK_OBJ][STRICT_CONTACT] retry attempt={attempt_idx + 1}/{max_regrasp} "
            f"reopen=true regrasp_z={next_z:.3f} prev_z={cur_z:.3f}"
        )
        ctx.ensure_gripper_open_for_moveit(reason=f"STRICT_REGRASP_{attempt_idx + 1}")
        regrasp_pose = dict(grasp_pose_live)
        regrasp_pose["position"] = (cur_x, cur_y, next_z)
        ctx.run_moveit_step(
            f"STRICT_GRASP_NUDGE_{attempt_idx + 1}",
            regrasp_pose,
            min(0.25, float(grasp_delay_live)),
        )
        grasp_pose_live["position"] = (cur_x, cur_y, next_z)
        ctx.close_gripper_sync(reason=f"STRICT_REGRASP_{attempt_idx + 1}")
        time.sleep(0.35)

    opening_txt = "0.0000"
    effort_txt = "0.0000"
    if state.last_metrics is not None:
        if state.last_metrics.get("opening_m") is not None:
            opening_txt = f"{float(state.last_metrics['opening_m']):.4f}"
        if state.last_metrics.get("effort_abs_max") is not None:
            effort_txt = f"{float(state.last_metrics['effort_abs_max']):.4f}"
    raise RuntimeError(
        "strict_pre_lift_contact_failed "
        f"opening_m={opening_txt} min_opening_m={min_blocked_opening_m:.4f} "
        f"max_opening_m={max_blocked_opening_m:.4f} "
        f"effort_abs={effort_txt} min_effort_abs={min_effort_abs:.4f}"
    )
