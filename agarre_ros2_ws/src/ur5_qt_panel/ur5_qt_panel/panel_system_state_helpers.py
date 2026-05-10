#!/usr/bin/env python3
"""F6 (auditoría 2026-05-10): system state + camera diagnostics helpers.

Funciones que coordinan el estado externo del sistema con el estado
local del panel (system_state_manager → SystemState/MoveItState) y
emiten diagnósticos cuando hay fallos de cámara.

Extraídas de ``panel_helpers.py`` líneas ~933–1024. ``panel_helpers``
re-exporta cada símbolo para preservar la API que usan los mixins.
"""
from __future__ import annotations

import time
from typing import Optional, Tuple

from .panel_config import CAMERA_TOPIC_PREFIX
from .panel_external_state import (
    apply_external_system_state,
    external_state_active,
    resolve_external_state,
)
from .panel_state import MoveItState, SystemState


def _log_ros_message(panel, msg: str):
    """Mostrar siempre los mensajes provenientes del RosWorker, pero solo
    cuando el bridge esté activo."""
    # Los errores de inicialización ROS son siempre visibles (no esperar al bridge).
    if "[ROS] ERROR rclpy" in msg:
        panel._emit_log(msg)
        return
    if not panel._bridge_running:
        return
    panel._emit_log(msg)


def _on_system_state_update(panel, state: str, reason: str) -> None:
    state = (state or "").strip().upper()
    if not state:
        return
    prev = panel._external_state
    panel._external_state = state
    panel._external_state_reason = reason or ""
    panel._external_state_last = time.time()
    # Trigger a state resolution pass so _system_state reflects the
    # external state immediately (without waiting for the next
    # _refresh_controls call, which may only fire on demand).
    if prev != state or panel._system_state in (SystemState.BOOT, SystemState.WAITING_GAZEBO):
        try:
            panel.signal_refresh_controls.emit()
        except RuntimeError:
            pass
    # F-audit (2026-05-10): hot-fix del cleanup incompleto del botón
    # "Test Robot" eliminado. Antes, btn_pick_object y btn_pick_demo
    # quedaban DISABLED para siempre porque _set_robot_test_done(True)
    # solo se invocaba desde el botón eliminado. Ahora, cuando el stack
    # llega a READY (system_state externo), disparamos la cascada de
    # habilitación una sola vez por sesión.
    if (
        state == "READY"
        and not getattr(panel, "_pick_buttons_auto_enabled", False)
    ):
        try:
            panel._set_robot_test_done(True)
            panel._pick_buttons_auto_enabled = True
            panel._emit_log(
                "[STATE] AUTO_TEST_DONE — pick buttons habilitados al llegar READY "
                "(hot-fix audit 2026-05-10 por cleanup botón Test Robot)"
            )
        except Exception as exc:
            panel._log(
                f"[STATE] AUTO_TEST_DONE fallo en _set_robot_test_done: "
                f"{type(exc).__name__}:{exc}"
            )


def _external_state_active(panel) -> bool:
    return external_state_active(panel)


def _resolve_external_state(panel) -> Tuple[Optional[SystemState], str]:
    return resolve_external_state(panel)


def _apply_external_system_state(panel) -> None:
    apply_external_system_state(panel)


def _log_camera_diagnostics(panel, reason: str):
    """Emitir detalles adicionales para debugging cuando hay fallos de cámara."""
    from .panel_camera import _runtime_time  # lazy: evita ciclo con panel_camera
    if not panel._camera_required:
        return
    if not panel._debug_logs_enabled:
        return
    node_ready = panel.ros_worker.node_ready()
    ctrl_ok = panel._ros2_control_available()
    clock_ok, clock_age = panel._clock_status()
    bridge_ok = panel._bridge_running
    last_age = "n/a"
    if panel._last_camera_frame_ts:
        last_age = f"{_runtime_time() - panel._last_camera_frame_ts:.1f}s"
    topics = []
    if node_ready:
        try:
            topics = panel.ros_worker.list_topic_names()
        except Exception as exc:
            panel._log(f"[CAMERA-DIAG] fallo listando topics: {exc}")
    camera_topics = [t for t in topics if t.startswith(CAMERA_TOPIC_PREFIX)]
    diag = (
        f"[CAMERA-DIAG] {reason} node_ready={node_ready} ros2_ctrl={ctrl_ok} "
        f"clock={clock_ok}:{clock_age} bridge={bridge_ok} "
        f"last_frame_age={last_age} camera_topics={len(camera_topics)}/{len(topics)}"
    )
    panel._log(diag)
    if camera_topics:
        preview = ", ".join(camera_topics[:4])
        suffix = "..." if len(camera_topics) > 4 else ""
        panel._log(f"[CAMERA-DIAG] camera topics sample: {preview}{suffix}")


def _sync_moveit_from_system_state(panel) -> None:
    if not panel._moveit_required:
        moveit_detected = bool(
            panel._moveit_running or panel._moveit_bridge_detected() or panel._moveit_status_ready()
        )
        if moveit_detected:
            panel._moveit_required = True
            if panel._moveit_state != MoveItState.READY:
                panel._moveit_state = MoveItState.READY
                panel._moveit_state_reason = "move_group detectado"
            panel._emit_log("[MOVEIT] move_group detectado; saliendo de modo manual automáticamente")
            return
        panel._moveit_state = MoveItState.OFF
        panel._moveit_state_reason = "manual"
        return
    if not panel._external_state_active():
        return
    if panel._system_state == SystemState.READY_MOVEIT:
        if panel._moveit_state != MoveItState.READY:
            panel._moveit_state = MoveItState.READY
            panel._moveit_state_reason = "move_group listo (externo)"
        return
    if panel._system_state == SystemState.READY_VISION:
        panel._moveit_state = MoveItState.WAITING_MOVEIT_READY
        panel._moveit_state_reason = panel._system_state_reason or "move_group no listo"


__all__ = [
    "_log_ros_message",
    "_on_system_state_update",
    "_external_state_active",
    "_resolve_external_state",
    "_apply_external_system_state",
    "_log_camera_diagnostics",
    "_sync_moveit_from_system_state",
]
