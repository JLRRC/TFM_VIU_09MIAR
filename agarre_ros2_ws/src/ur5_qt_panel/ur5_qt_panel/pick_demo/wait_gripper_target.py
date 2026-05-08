#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/wait_gripper_target.py
# Contenido: F3-step36 — wait_for_gripper_target extraído del closure run_pick_demo.
"""Wait loop para confirmar gripper abierto/cerrado.

Función pura extraída del closure ``run_pick_demo._wait_for_gripper_target``
(145 LOC). El wrapper en panel_pick_demo se reduce a ~10 LOC delegando
aquí. Todas las dependencias (panel, callables del closure, params)
se inyectan como args.

Patrón aplicado: callables-as-args (idéntico a F3-step5bis-c usado en
panel_pick_object para _ensure_moveit_bridge_path).
"""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, Optional, Tuple


def wait_for_gripper_target(
    panel: Any,
    *,
    closed: bool,
    timeout_sec: float = 1.8,
    opening_ref_sum: Optional[float] = None,
    cmd_fn: Optional[Callable[[], None]] = None,
    cmd_retry_sec: float = 0.5,
    # Callables-as-args (closure-captured en run_pick_demo):
    read_gripper_state: Callable[..., Dict[str, Any]],
    append_trace: Callable[[str], None],
    monitor_alcance: Callable[..., None],
    # Params snapshot (caller invoca _get_pick_demo_params() y pasa los 4 valores):
    gripper_confirm_stable_samples: int,
    gripper_confirm_max_state_age_sec: float,
    close_min_delta_sum: float,
    close_fallback_opening_sum: float,
) -> Tuple[bool, Dict[str, Any]]:
    """Wait hasta que el gripper alcance el target (closed o open).

    Devuelve ``(ok, state)``. ``ok`` True cuando el confirm es estable
    durante ``gripper_confirm_stable_samples`` muestras consecutivas.
    """
    required_hits = max(1, int(gripper_confirm_stable_samples))
    max_state_age_sec = max(0.05, float(gripper_confirm_max_state_age_sec))
    close_min_delta_sum = max(0.005, float(close_min_delta_sum))
    close_fallback_opening_sum = max(0.02, float(close_fallback_opening_sum))
    start = time.monotonic()
    stable_hits = 0
    last_state = read_gripper_state(expected_closed=closed)
    best_close_delta = float("-inf")
    last_debug_log_ts = 0.0
    append_trace(
        "[PICK][DIRECT][GRIPPER] "
        f"wait_start target={'closed' if closed else 'open'} "
        f"target_mag={last_state.get('target_mag')} timeout={timeout_sec:.2f}s "
        f"stable_hits={required_hits} opening_ref_sum={opening_ref_sum}"
    )
    _last_cmd_retry_ts = time.monotonic()
    _cmd_retry_count = 0
    while (time.monotonic() - start) <= timeout_sec:
        if cmd_fn is not None:
            _now_retry = time.monotonic()
            if (_now_retry - _last_cmd_retry_ts) >= cmd_retry_sec:
                _cmd_retry_count += 1
                panel.signal_run_ui.emit(cmd_fn)
                try:
                    cmd_fn()
                except Exception as _exc_cmdfn:
                    panel._emit_log(
                        "[PICK][DIRECT][GRIPPER][ERR] direct cmd_fn failed: "
                        f"{type(_exc_cmdfn).__name__}: {_exc_cmdfn}"
                    )
                _last_cmd_retry_ts = _now_retry
                append_trace(
                    "[PICK][DIRECT][GRIPPER] "
                    f"cmd_retry #{_cmd_retry_count} "
                    f"target={'closed' if closed else 'open'} "
                    f"elapsed={_now_retry - start:.2f}s"
                )
        monitor_alcance(trigger=f"GRIPPER_WAIT_{'CLOSE' if closed else 'OPEN'}")
        state = read_gripper_state(expected_closed=closed)
        last_state = state
        measured_ok = bool(state.get("measured_target_ok"))
        joint_age = state.get("joint_state_age_sec")
        age_ok = (
            joint_age is None
            or float(joint_age) <= max_state_age_sec
        )
        opening_sum = state.get("opening_sum")
        close_delta = None
        if (
            opening_ref_sum is not None
            and opening_sum is not None
        ):
            close_delta = float(opening_ref_sum) - float(opening_sum)
            best_close_delta = max(best_close_delta, float(close_delta))
        close_heuristic_ok = False
        if closed:
            if (
                age_ok
                and bool(state.get("closed_flag"))
                and opening_sum is not None
            ):
                delta_ok = (
                    close_delta is not None
                    and float(close_delta) >= float(close_min_delta_sum)
                )
                fallback_ok = (
                    opening_ref_sum is None
                    and float(opening_sum) <= float(close_fallback_opening_sum)
                )
                close_heuristic_ok = bool(delta_ok or fallback_ok)
        confirm_mode = "none"
        confirm_ok = False
        if measured_ok and age_ok:
            confirm_ok = True
            confirm_mode = "measured_target_ok"
        elif close_heuristic_ok:
            confirm_ok = True
            confirm_mode = "closing_delta_ok"
        now_ts = time.monotonic()
        if now_ts >= (last_debug_log_ts + 0.20):
            append_trace(
                "[PICK][DIRECT][GRIPPER] "
                f"wait_sample target={'closed' if closed else 'open'} "
                f"closed_flag={bool(state.get('closed_flag'))} "
                f"measured_ok={measured_ok} age_ok={age_ok} "
                f"opening_sum={state.get('opening_sum')} "
                f"max_abs_err={state.get('max_abs_err')} "
                f"close_delta={close_delta} min_delta={close_min_delta_sum} mode={confirm_mode}"
            )
            last_debug_log_ts = now_ts
        if confirm_ok:
            stable_hits += 1
            if stable_hits >= required_hits:
                done_state = dict(state)
                done_state["confirm_mode"] = confirm_mode
                done_state["close_delta_from_ref"] = close_delta
                done_state["close_delta_best"] = (
                    None
                    if best_close_delta == float("-inf")
                    else float(best_close_delta)
                )
                append_trace(
                    "[PICK][DIRECT][GRIPPER] "
                    f"wait_ok target={'closed' if closed else 'open'} "
                    f"opening_sum={state.get('opening_sum')} "
                    f"max_abs_err={state.get('max_abs_err')} "
                    f"age={state.get('joint_state_age_sec')} mode={confirm_mode} "
                    f"close_delta={close_delta}"
                )
                return True, done_state
        else:
            stable_hits = 0
        time.sleep(0.05)
    timeout_state = dict(last_state or {})
    timeout_state["confirm_mode"] = "timeout"
    timeout_state["close_delta_best"] = (
        None if best_close_delta == float("-inf") else float(best_close_delta)
    )
    append_trace(
        "[PICK][DIRECT][GRIPPER] "
        f"wait_timeout target={'closed' if closed else 'open'} "
        f"opening_sum={last_state.get('opening_sum')} "
        f"max_abs_err={last_state.get('max_abs_err')} "
        f"age={last_state.get('joint_state_age_sec')} "
        f"closed_flag={bool(last_state.get('closed_flag'))} "
        f"measured_ok={bool(last_state.get('measured_target_ok'))} "
        f"opening_ref_sum={opening_ref_sum} "
        f"close_delta_best={timeout_state.get('close_delta_best')}"
    )
    return False, timeout_state
