#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_object/ensure_gripper_open.py
# Contenido: F3-step4c — extracción de _ensure_gripper_open_for_moveit (156 LOC).
"""Pre-MoveIt gripper open guard extraído del closure run_pick_object.worker.

``ensure_gripper_open_for_moveit`` garantiza que el gripper está
abierto al menos ``min_opening_m`` antes de invocar MoveIt en fases
APPROACH/HOME donde un gripper cerrado bloquearía la planificación.

Antes de F3-step4c vivía como ``def`` anidada de 156 LOC dentro del
closure ``run_pick_object.worker``, capturando 2 deps reales
(panel + Float64MultiArray msg). 8 callsites en el closure.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional


@dataclass
class EnsureGripperOpenContext:
    """Captura de dependencias closure de ensure_gripper_open_for_moveit."""

    panel: Any
    get_pick_object_params: Callable[[], Any]
    norm_frame: Callable[[Any], Any]
    log_moveit_panel_trace: Callable[..., Any]


def ensure_gripper_open_for_moveit(
    ctx: EnsureGripperOpenContext,
    *,
    reason: str = "MOVEIT",
) -> None:
    """Garantiza que el gripper está abierto antes del próximo MoveIt step."""
    try:
        min_opening_m = ctx.get_pick_object_params().min_opening_m
    except Exception:
        min_opening_m = 0.020
    reason_upper = str(reason or "").strip().upper()
    if reason_upper in ("PRE_GRASP", "GRASP_DOWN"):
        try:
            min_opening_before_descent_m = ctx.get_pick_object_params().min_opening_before_descent_m
        except Exception:
            min_opening_before_descent_m = 0.030
        min_opening_m = max(min_opening_m, min_opening_before_descent_m)
    min_opening_m = max(0.0, min_opening_m)
    try:
        open_wait_sec = ctx.get_pick_object_params().open_wait_sec
    except Exception:
        open_wait_sec = 2.2
    open_wait_sec = max(0.1, open_wait_sec)
    try:
        open_cmd_ack_sec = ctx.get_pick_object_params().open_cmd_ack_sec
    except Exception:
        open_cmd_ack_sec = 3.0
    open_cmd_ack_sec = max(0.3, open_cmd_ack_sec)

    joint_names = [str(j).strip() for j in (GRIPPER_JOINT_NAMES or []) if str(j).strip()]
    if not joint_names:
        joint_names = ["rg2_finger_joint1", "rg2_finger_joint2"]

    def _direct_open_publish() -> bool:
        if Float64MultiArray is None:
            ctx.panel._emit_log(
                f"[PICK_OBJ][GRIPPER] direct_open_publish_unavailable before={reason}"
            )
            return False
        try:
            ctx.panel._ensure_moveit_node()
            pub = ctx.panel._get_gripper_publisher(GRIPPER_CMD_TOPIC)
            if pub is None:
                ctx.panel._emit_log(
                    f"[PICK_OBJ][GRIPPER] direct_open_publish_no_publisher before={reason}"
                )
                return False
            target = float(GRIPPER_OPEN_RAD)
            msg = Float64MultiArray()
            msg.data = [target, float(target) * float(GRIPPER_JOINT2_SIGN)]
            ctx.panel._gripper_closed = False
            ctx.panel._gripper_is_closed = False
            pub.publish(msg)
            ctx.panel._emit_log(
                "[PICK_OBJ][GRIPPER] direct_open_publish "
                f"before={reason} target={target:.3f} "
                f"topic={GRIPPER_CMD_TOPIC} ack_timeout={float(open_cmd_ack_sec):.2f}s"
            )
            return True
        except Exception as exc:
            ctx.panel._emit_log(
                "[PICK_OBJ][GRIPPER] direct_open_publish_fail "
                f"before={reason} err={type(exc).__name__}:{exc}"
            )
            return False

    def _current_opening_m() -> tuple[Optional[float], bool]:
        if ctx.panel.ros_worker is None:
            return None, False
        payload, payload_wall = ctx.panel.ros_worker.get_last_joint_state()
        if not payload:
            return None, False
        if payload_wall <= 0.0 or (time.time() - float(payload_wall)) > 1.0:
            return None, False
        try:
            names = list(payload.get("name", []))
            pos = list(payload.get("position", []))
        except Exception:
            return None, False
        if not names or not pos:
            return None, False
        pos_map = {str(n): float(p) for n, p in zip(names, pos)}
        vals = [abs(float(pos_map[j])) for j in joint_names if j in pos_map]
        if len(vals) != len(joint_names):
            return None, True
        return float(sum(vals)), True

    opening_ok = False
    saw_joint_state = False
    last_opening_m: Optional[float] = None
    for attempt in (1, 2):
        state = {"done": False, "ok": False}

        def _open_cmd() -> None:
            state["ok"] = bool(
                ctx.panel._command_gripper(False, log_action="PICK", force=True)
            )
            state["done"] = True

        ctx.panel.signal_run_ui.emit(_open_cmd)
        deadline = time.time() + open_cmd_ack_sec
        while time.time() < deadline:
            if state["done"]:
                break
            time.sleep(0.03)
        if not state["done"]:
            ctx.panel._emit_log(
                "[PICK_OBJ][GRIPPER] ui_open_dispatch_timeout "
                f"before={reason} ack_timeout={float(open_cmd_ack_sec):.2f}s "
                "fallback=direct_publish"
            )
            state["ok"] = _direct_open_publish()
            state["done"] = True
        if not state["ok"]:
            raise RuntimeError("gripper_open_command_failed_before_moveit")
        if bool(getattr(ctx.panel, "_gripper_closed", False)):
            raise RuntimeError("gripper_state_closed_before_moveit")

        wait_deadline = time.time() + open_wait_sec
        while time.time() < wait_deadline:
            opening_m, has_js = _current_opening_m()
            saw_joint_state = saw_joint_state or has_js
            if opening_m is not None:
                last_opening_m = float(opening_m)
                if last_opening_m >= min_opening_m:
                    opening_ok = True
                    break
            time.sleep(0.05)
        if opening_ok:
            break
        if attempt == 1:
            ctx.panel._emit_log(
                f"[PICK_OBJ][GRIPPER] apertura insuficiente antes de {reason}; "
                "reintentando abrir"
            )

    if saw_joint_state and not opening_ok:
        allow_degraded_guard = ctx.get_pick_object_params().allow_degraded_open_guard
        if allow_degraded_guard and not bool(getattr(ctx.panel, "_gripper_closed", False)):
            ctx.panel._emit_log(
                "[PICK_OBJ][GRIPPER] apertura no confirmada por joint_state; "
                f"continuando en modo degradado before={reason} "
                f"opening_m={0.0 if last_opening_m is None else float(last_opening_m):.4f} "
                f"min_opening_m={float(min_opening_m):.4f}"
            )
            return
        raise RuntimeError(
            f"gripper_opening_below_threshold_before_{str(reason).lower()} "
            f"opening_m={0.0 if last_opening_m is None else float(last_opening_m):.4f} "
            f"min_opening_m={float(min_opening_m):.4f}"
        )
    if saw_joint_state and opening_ok:
        ctx.panel._emit_log(
            f"[PICK_OBJ][GRIPPER] apertura_ok before={reason} "
            f"opening_m={float(last_opening_m or 0.0):.4f} "
            f"min_opening_m={float(min_opening_m):.4f}"
        )
    ctx.panel._emit_log(
        f"[PICK_OBJ][GRIPPER] open_guard before={reason} "
        f"min_opening_m={float(min_opening_m):.4f} open_wait_sec={float(open_wait_sec):.2f}"
    )
