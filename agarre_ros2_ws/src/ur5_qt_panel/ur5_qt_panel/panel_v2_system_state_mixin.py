#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_system_state_mixin.py
# Contenido: F14-step7 (2026-05-01) — mixin signals/status/system_state.
"""Mixin signals + status + system_state (F14-step7).

Séptimo paso del refactor F14: extrae el grupo más grande de
wrappers thin de la clase ``ControlPanelV2`` — los relacionados con
construcción de la UI, debouncing de clicks, gestión de status/LED,
respuesta a las señales Qt internas (``signal_tf_ready``,
``signal_calib_ready``, etc.), y la máquina de system_state que
agrega el estado global del stack.

Métodos agrupados (29 wrappers):

UI construction & event:
* ``_build_ui``, ``_debounced_btn_action``, ``showEvent``.

Status / LED / signal handlers:
* ``_set_status``, ``_set_status_async``, ``_set_led_async``.
* ``_on_tf_ready_signal``, ``_on_calib_ready_signal``,
  ``_run_startup_tf_sanity_check_once``,
  ``_on_controllers_ready_signal``, ``_on_error_signal``,
  ``_on_moveit_state_signal``, ``_on_trace_ready``,
  ``_on_calibration_check``.

System state machine:
* ``_set_system_state``, ``_effective_system_state``.
* ``_trigger_fatal``, ``_resolve_system_state``,
  ``_build_state_snapshot``, ``_evaluate_system_state``,
  ``_update_system_state``.
* ``_check_critical_timeouts``, ``_resolve_critical_fault``.
* ``_state_ready_basic``, ``_state_ready_vision``,
  ``_state_ready_moveit``, ``_state_ready_level``,
  ``_manual_control_ready``.
* ``_calibration_topic_allowed``, ``_overhead_camera_active``.

Todos delegan al módulo ``panel_state_methods`` (alias ``_stm``).
"""

from __future__ import annotations

from . import panel_state_methods as _stm


class PanelV2SystemStateMixin:
    """Wrappers thin de UI build, signals, status y system_state."""

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self, *args, **kwargs):
        return _stm._build_ui(self, *args, **kwargs)

    def _debounced_btn_action(self, *args, **kwargs):
        return _stm._debounced_btn_action(self, *args, **kwargs)

    def showEvent(self, *args, **kwargs):
        return _stm.showEvent(self, *args, **kwargs)

    # ------------------------------------------------------------------
    # Status / LED / signal handlers
    # ------------------------------------------------------------------

    def _set_status(self, *args, **kwargs):
        return _stm._set_status(self, *args, **kwargs)

    def _set_status_async(self, *args, **kwargs):
        return _stm._set_status_async(self, *args, **kwargs)

    def _set_led_async(self, *args, **kwargs):
        return _stm._set_led_async(self, *args, **kwargs)

    def _on_tf_ready_signal(self, *args, **kwargs):
        return _stm._on_tf_ready_signal(self, *args, **kwargs)

    def _on_calib_ready_signal(self, *args, **kwargs):
        return _stm._on_calib_ready_signal(self, *args, **kwargs)

    def _run_startup_tf_sanity_check_once(self, *args, **kwargs):
        return _stm._run_startup_tf_sanity_check_once(self, *args, **kwargs)

    def _on_controllers_ready_signal(self, *args, **kwargs):
        return _stm._on_controllers_ready_signal(self, *args, **kwargs)

    def _on_error_signal(self, *args, **kwargs):
        return _stm._on_error_signal(self, *args, **kwargs)

    def _on_moveit_state_signal(self, *args, **kwargs):
        return _stm._on_moveit_state_signal(self, *args, **kwargs)

    def _on_trace_ready(self, *args, **kwargs):
        return _stm._on_trace_ready(self, *args, **kwargs)

    def _on_calibration_check(self, *args, **kwargs):
        return _stm._on_calibration_check(self, *args, **kwargs)

    # ------------------------------------------------------------------
    # System state machine
    # ------------------------------------------------------------------

    def _set_system_state(self, *args, **kwargs):
        return _stm._set_system_state(self, *args, **kwargs)

    def _effective_system_state(self, *args, **kwargs):
        return _stm._effective_system_state(self, *args, **kwargs)

    def _trigger_fatal(self, *args, **kwargs):
        return _stm._trigger_fatal(self, *args, **kwargs)

    def _resolve_system_state(self, *args, **kwargs):
        return _stm._resolve_system_state(self, *args, **kwargs)

    def _build_state_snapshot(self, *args, **kwargs):
        return _stm._build_state_snapshot(self, *args, **kwargs)

    def _evaluate_system_state(self, *args, **kwargs):
        return _stm._evaluate_system_state(self, *args, **kwargs)

    def _update_system_state(self, *args, **kwargs):
        return _stm._update_system_state(self, *args, **kwargs)

    def _check_critical_timeouts(self, *args, **kwargs):
        return _stm._check_critical_timeouts(self, *args, **kwargs)

    def _resolve_critical_fault(self, *args, **kwargs):
        return _stm._resolve_critical_fault(self, *args, **kwargs)

    def _state_ready_basic(self, *args, **kwargs):
        return _stm._state_ready_basic(self, *args, **kwargs)

    def _state_ready_vision(self, *args, **kwargs):
        return _stm._state_ready_vision(self, *args, **kwargs)

    def _state_ready_moveit(self, *args, **kwargs):
        return _stm._state_ready_moveit(self, *args, **kwargs)

    def _state_ready_level(self, *args, **kwargs):
        return _stm._state_ready_level(self, *args, **kwargs)

    def _manual_control_ready(self, *args, **kwargs):
        return _stm._manual_control_ready(self, *args, **kwargs)

    def _calibration_topic_allowed(self, *args, **kwargs):
        return _stm._calibration_topic_allowed(self, *args, **kwargs)

    def _overhead_camera_active(self, *args, **kwargs):
        return _stm._overhead_camera_active(self, *args, **kwargs)
