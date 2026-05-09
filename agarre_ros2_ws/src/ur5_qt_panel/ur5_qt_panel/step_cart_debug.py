#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/step_cart_debug.py
# Contenido: F3 — callbacks de la ventana de debug cartesiano paso-a-paso.
"""Callbacks de la ventana de debug cartesiano del modo STEP_BY_STEP.

Extraídas de ``panel_step_callbacks.py`` (god-file de 2.2 kLOC) para
reducir su tamaño y agrupar la funcionalidad cohesiva del nudge
cartesiano paso-a-paso. Cada función toma ``panel`` (instancia de
``ControlPanelV2``) como primer parámetro y usa atributos/métodos
del panel para state y emit signals.

Las funciones se re-exportan desde ``panel_step_callbacks`` para no
romper consumidores existentes (panel_v2 y otros que importan estos
nombres directamente).
"""

from __future__ import annotations

from .panel_step_ui import build_step_cart_debug_window


def _step_cart_debug_move_delta(panel, dx_m: float, dy_m: float, dz_m: float) -> None:
    if panel._step_mode != "STEP_BY_STEP":
        panel._step_cart_debug_set_status("Solo disponible en STEP_BY_STEP", error=True)
        return
    if panel._step_cart_debug_move_inflight:
        panel._step_cart_debug_set_status("Movimiento en curso...", error=False)
        return
    before = panel._step_fetch_live_pose("rg2_pinch_center")
    if before is None:
        panel._step_cart_debug_set_status("TF rg2_pinch_center no disponible", error=True)
        panel._step_cart_debug_log_event(
            "move_rejected",
            reason="pinch_pose_unavailable",
            delta_m=[float(dx_m), float(dy_m), float(dz_m)],
        )
        return
    target = (
        float(before[0]) + float(dx_m),
        float(before[1]) + float(dy_m),
        float(before[2]) + float(dz_m),
    )
    panel._step_cart_debug_set_status("Ejecutando nudge cartesiano...", error=False)
    panel._step_cart_debug_log_event(
        "move_request",
        delta_m=[float(dx_m), float(dy_m), float(dz_m)],
        before=before,
        target=target,
    )

    def worker() -> None:
        panel._step_cart_debug_move_inflight = True
        ok = False
        info = ""
        after = None
        axis_err = None
        try:
            axis_tol_m = max(0.0010, panel._step_cart_debug_step_m() * 0.30)
            max_attempts = 3
            for attempt in range(1, max_attempts + 1):
                ok, info, after, axis_err = panel._step_cartesian_move_runtime_target(
                    target,
                    timeout_sec=4.0,
                    axis_tol_m=axis_tol_m,
                )
                panel._step_cart_debug_log_event(
                    "move_attempt",
                    attempt=int(attempt),
                    max_attempts=int(max_attempts),
                    target=target,
                    after=after,
                    axis_error_to_target=axis_err,
                    success=bool(ok),
                    result=str(info),
                    axis_tol_m=float(axis_tol_m),
                )
                if ok:
                    break
        except Exception as exc:
            ok = False
            info = f"exception:{exc}"
        finally:
            panel._step_cart_debug_move_inflight = False

        delta_after = None
        if before is not None and after is not None:
            delta_after = (
                float(after[0]) - float(before[0]),
                float(after[1]) - float(before[1]),
                float(after[2]) - float(before[2]),
            )

        def _ui_done() -> None:
            if ok:
                panel._step_cart_debug_set_status("Nudge ejecutado", error=False)
            else:
                panel._step_cart_debug_set_status(f"Error: {info}", error=True)
            panel._step_cart_debug_refresh()

        panel.signal_run_ui.emit(_ui_done)
        panel._step_cart_debug_log_event(
            "move_result",
            success=bool(ok),
            result=str(info),
            delta_m=[float(dx_m), float(dy_m), float(dz_m)],
            before=before,
            target=target,
            after=after,
            delta_after=delta_after,
            axis_error_to_target=axis_err,
        )

    panel._run_async(worker, name="step_cartesian_debug_move")


def _step_cart_debug_handle_axis(panel, axis: str, sign: int) -> None:
    step = panel._step_cart_debug_step_m()
    axis_n = str(axis or "").strip().upper()
    s = 1.0 if int(sign) >= 0 else -1.0
    dx = s * step if axis_n == "X" else 0.0
    dy = s * step if axis_n == "Y" else 0.0
    dz = s * step if axis_n == "Z" else 0.0
    panel._step_cart_debug_move_delta(dx, dy, dz)


def _step_cart_debug_run_validation_xyz(panel) -> None:
    if panel._step_mode != "STEP_BY_STEP":
        panel._step_cart_debug_set_status("Validación disponible solo en STEP_BY_STEP", error=True)
        return
    if panel._step_cart_debug_move_inflight:
        panel._step_cart_debug_set_status("Movimiento en curso...", error=False)
        return

    def worker() -> None:
        panel._step_cart_debug_move_inflight = True
        try:
            step = 0.005
            axis_tol_m = max(0.0010, step * 0.30)
            tests = [
                ("X+", (step, 0.0, 0.0)),
                ("Y+", (0.0, step, 0.0)),
                ("Z+", (0.0, 0.0, step)),
            ]
            panel._step_cart_debug_log_event(
                "validation_xyz_start",
                step_m=float(step),
                axis_tol_m=float(axis_tol_m),
            )

            for name, delta in tests:
                before = panel._step_fetch_live_pose("rg2_pinch_center")
                if before is None:
                    panel._step_cart_debug_log_event(
                        "validation_xyz_case",
                        case=name,
                        status="failed",
                        reason="before_pose_unavailable",
                    )
                    continue
                target = (
                    float(before[0]) + float(delta[0]),
                    float(before[1]) + float(delta[1]),
                    float(before[2]) + float(delta[2]),
                )
                ok, info, after, err_xyz = panel._step_cartesian_move_runtime_target(
                    target,
                    timeout_sec=4.0,
                    axis_tol_m=axis_tol_m,
                )
                delta_after = None
                if after is not None:
                    delta_after = (
                        float(after[0]) - float(before[0]),
                        float(after[1]) - float(before[1]),
                        float(after[2]) - float(before[2]),
                    )
                panel._step_cart_debug_log_event(
                    "validation_xyz_case",
                    case=name,
                    status="ok" if ok else "failed",
                    before=before,
                    target=target,
                    after=after,
                    delta_after=delta_after,
                    axis_error_to_target=err_xyz,
                    result=str(info),
                )

            panel._step_cart_debug_log_event("validation_xyz_end")

            def _ui_ok() -> None:
                panel._step_cart_debug_set_status("Validación XYZ registrada en historico", error=False)
                panel._step_cart_debug_refresh()

            panel.signal_run_ui.emit(_ui_ok)
        except Exception as exc:
            panel._step_cart_debug_log_event("validation_xyz_error", error=str(exc))
            _exc_msg = str(exc)

            def _ui_fail() -> None:
                panel._step_cart_debug_set_status(f"Error validación: {_exc_msg}", error=True)
                panel._step_cart_debug_refresh()

            panel.signal_run_ui.emit(_ui_fail)
        finally:
            panel._step_cart_debug_move_inflight = False

    panel._run_async(worker, name="step_cartesian_debug_validation")


def _ensure_step_cart_debug_window(panel) -> None:
    if panel._step_cart_debug_window is not None:
        return
    build_step_cart_debug_window(panel)


def _show_step_cart_debug_window(panel) -> None:
    panel._ensure_step_cart_debug_window()
    if panel._step_cart_debug_window is None:
        return
    if panel._step_window is not None:
        geo = panel._step_window.geometry()
        panel._step_cart_debug_window.move(geo.x() + geo.width() + 12, geo.y())
    panel._step_cart_debug_window.show()
    panel._step_cart_debug_refresh()
