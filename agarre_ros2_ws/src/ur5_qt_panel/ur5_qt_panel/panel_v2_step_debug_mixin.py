#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_step_debug_mixin.py
# Contenido: F14-step8 (2026-05-01) — mixin step UI + cart debug + direct flow.
"""Mixin step UI + cart debug + direct flow (F14-step8).

Octavo paso del refactor F14: extrae el grupo más grande de wrappers
que quedaba en ``ControlPanelV2`` — los relacionados con la
visualización paso a paso del pipeline (``step_*``), el debug
cartesiano (``step_cart_debug_*``) y el flow del direct pick
(``_direct_*_waiting_for_approach_confirmation``).

Métodos extraídos (72 wrappers):

UI step + debug motion (10):
* ``_set_debug_motion_button_waiting``, ``_on_step_mode_combo_changed``,
  ``_set_step_mode``, ``_ensure_step_window``,
  ``_on_step_continue_clicked``, ``_on_step_phase_start_clicked``,
  ``_on_step_window_finished``, ``_on_debug_motion_button``,
  ``_debug_motion_wait_for_continue``,
  ``_set_debug_motion_button_waiting``.

Direct flow (4):
* ``_direct_clear_waiting_for_approach_confirmation``,
  ``_direct_enter_waiting_for_approach_confirmation``,
  ``_direct_release_waiting_for_approach_confirmation``,
  ``_direct_waiting_for_approach_confirmation``.

Step cart debug (10):
* ``_step_cart_debug_trace_path``, ``_step_cart_debug_log_event``,
  ``_step_cart_debug_sample``, ``_step_cart_debug_set_status``,
  ``_step_cart_debug_refresh``, ``_step_cart_debug_step_m``,
  ``_ensure_step_cart_debug_window``, ``_show_step_cart_debug_window``,
  ``_step_cart_debug_handle_axis``, ``_step_cart_debug_move_delta``,
  ``_step_cart_debug_run_validation_xyz``,
  ``_step_cartesian_move_runtime_target``.

Step pipeline + history (40+):
* ``_step_assess_target_reached``, ``_step_capture_start_pose``,
  ``_step_display_position``, ``_step_effective_flow``,
  ``_step_fetch_live_pose``, ``_step_fetch_object_world``,
  ``_step_find_history_row``, ``_step_format_inline_*``,
  ``_step_format_xyz``, ``_step_live_*``,
  ``_step_operational_frame_name``, ``_step_phase_*``,
  ``_step_pipeline_*``, ``_step_pre_insert_inicio_row``,
  ``_step_predict_next_phase``, ``_step_prepare_pipeline_view``,
  ``_step_present_flow_name``, ``_step_record_*``,
  ``_step_refresh_pipeline_table``, ``_step_reset_sequence_view``,
  ``_step_runtime_refresh``, ``_step_selected_object_name``,
  ``_step_set_exec_target``, ``_step_status_item``,
  ``_step_update_phase_result``, ``_step_update_row_object_metrics``,
  ``_step_upsert_history_row_ordered``, ``_step_wait_for_phase``,
  ``_step_window_*``.

Read gripper feedback (1):
* ``_read_gripper_feedback_state``.

Step joint via panel_motion_control (1):
* ``_step_joint`` (delega a ``_mc`` en lugar de ``_ph``).

Todos los demás wrappers delegan a ``panel_helpers`` (alias
``_ph``).
"""

from __future__ import annotations

from . import panel_helpers as _ph
from . import panel_motion_control as _mc


class PanelV2StepDebugMixin:
    """Wrappers thin de step UI / cart debug / direct flow."""

    # ------------------------------------------------------------------
    # UI step + debug motion
    # ------------------------------------------------------------------

    def _set_debug_motion_button_waiting(self, *args, **kwargs):
        return _ph._set_debug_motion_button_waiting(self, *args, **kwargs)

    def _on_step_mode_combo_changed(self, *args, **kwargs):
        return _ph._on_step_mode_combo_changed(self, *args, **kwargs)

    def _set_step_mode(self, *args, **kwargs):
        return _ph._set_step_mode(self, *args, **kwargs)

    def _ensure_step_window(self, *args, **kwargs):
        return _ph._ensure_step_window(self, *args, **kwargs)

    def _on_step_continue_clicked(self, *args, **kwargs):
        return _ph._on_step_continue_clicked(self, *args, **kwargs)

    def _on_step_phase_start_clicked(self, *args, **kwargs):
        return _ph._on_step_phase_start_clicked(self, *args, **kwargs)

    def _on_step_window_finished(self, *args, **kwargs):
        return _ph._on_step_window_finished(self, *args, **kwargs)

    def _on_debug_motion_button(self, *args, **kwargs):
        # NOTE: el legacy descartaba *args, **kwargs. Preservamos.
        return _ph._on_debug_motion_button(self)

    def _debug_motion_wait_for_continue(self, *args, **kwargs):
        return _ph._debug_motion_wait_for_continue(self, *args, **kwargs)

    # ------------------------------------------------------------------
    # Direct flow waiting for approach confirmation
    # ------------------------------------------------------------------

    def _direct_clear_waiting_for_approach_confirmation(self, *args, **kwargs):
        return _ph._direct_clear_waiting_for_approach_confirmation(
            self, *args, **kwargs
        )

    def _direct_enter_waiting_for_approach_confirmation(self, *args, **kwargs):
        return _ph._direct_enter_waiting_for_approach_confirmation(
            self, *args, **kwargs
        )

    def _direct_release_waiting_for_approach_confirmation(self, *args, **kwargs):
        return _ph._direct_release_waiting_for_approach_confirmation(
            self, *args, **kwargs
        )

    def _direct_waiting_for_approach_confirmation(self, *args, **kwargs):
        return _ph._direct_waiting_for_approach_confirmation(
            self, *args, **kwargs
        )

    # ------------------------------------------------------------------
    # Step cart debug window
    # ------------------------------------------------------------------

    def _step_cart_debug_trace_path(self, *args, **kwargs):
        return _ph._step_cart_debug_trace_path(self, *args, **kwargs)

    def _step_cart_debug_log_event(self, *args, **kwargs):
        return _ph._step_cart_debug_log_event(self, *args, **kwargs)

    def _step_cart_debug_sample(self, *args, **kwargs):
        return _ph._step_cart_debug_sample(self, *args, **kwargs)

    def _step_cart_debug_set_status(self, *args, **kwargs):
        return _ph._step_cart_debug_set_status(self, *args, **kwargs)

    def _step_cart_debug_refresh(self, *args, **kwargs):
        return _ph._step_cart_debug_refresh(self, *args, **kwargs)

    def _step_cart_debug_step_m(self, *args, **kwargs):
        return _ph._step_cart_debug_step_m(self, *args, **kwargs)

    def _ensure_step_cart_debug_window(self, *args, **kwargs):
        return _ph._ensure_step_cart_debug_window(self, *args, **kwargs)

    def _show_step_cart_debug_window(self, *args, **kwargs):
        return _ph._show_step_cart_debug_window(self, *args, **kwargs)

    def _step_cart_debug_handle_axis(self, *args, **kwargs):
        return _ph._step_cart_debug_handle_axis(self, *args, **kwargs)

    def _step_cart_debug_move_delta(self, *args, **kwargs):
        return _ph._step_cart_debug_move_delta(self, *args, **kwargs)

    def _step_cart_debug_run_validation_xyz(self, *args, **kwargs):
        return _ph._step_cart_debug_run_validation_xyz(self, *args, **kwargs)

    def _step_cartesian_move_runtime_target(self, *args, **kwargs):
        return _ph._step_cartesian_move_runtime_target(self, *args, **kwargs)

    # ------------------------------------------------------------------
    # Step pipeline + history + phase
    # ------------------------------------------------------------------

    def _step_assess_target_reached(self, *args, **kwargs):
        return _ph._step_assess_target_reached(self, *args, **kwargs)

    def _step_capture_start_pose(self, *args, **kwargs):
        return _ph._step_capture_start_pose(self, *args, **kwargs)

    def _step_display_position(self, *args, **kwargs):
        return _ph._step_display_position(self, *args, **kwargs)

    def _step_effective_flow(self, *args, **kwargs):
        return _ph._step_effective_flow(self, *args, **kwargs)

    def _step_fetch_live_pose(self, *args, **kwargs):
        return _ph._step_fetch_live_pose(self, *args, **kwargs)

    def _step_fetch_object_world(self, *args, **kwargs):
        return _ph._step_fetch_object_world(self, *args, **kwargs)

    def _step_find_history_row(self, *args, **kwargs):
        return _ph._step_find_history_row(self, *args, **kwargs)

    def _step_format_inline_rpy(self, *args, **kwargs):
        return _ph._step_format_inline_rpy(self, *args, **kwargs)

    def _step_format_inline_xyz(self, *args, **kwargs):
        return _ph._step_format_inline_xyz(self, *args, **kwargs)

    def _step_format_xyz(self, *args, **kwargs):
        return _ph._step_format_xyz(self, *args, **kwargs)

    def _step_live_gripper_state(self, *args, **kwargs):
        return _ph._step_live_gripper_state(self, *args, **kwargs)

    def _step_live_pose_text(self, *args, **kwargs):
        return _ph._step_live_pose_text(self, *args, **kwargs)

    def _step_operational_frame_name(self, *args, **kwargs):
        return _ph._step_operational_frame_name(self, *args, **kwargs)

    def _step_phase_action_text(self, *args, **kwargs):
        return _ph._step_phase_action_text(self, *args, **kwargs)

    def _step_phase_completed(self, *args, **kwargs):
        return _ph._step_phase_completed(self, *args, **kwargs)

    def _step_phase_gate_already_owned(self, *args, **kwargs):
        return _ph._step_phase_gate_already_owned(self, *args, **kwargs)

    def _step_phase_gripper_state(self, *args, **kwargs):
        return _ph._step_phase_gripper_state(self, *args, **kwargs)

    def _step_phase_intent(self, *args, **kwargs):
        return _ph._step_phase_intent(self, *args, **kwargs)

    def _step_phase_sequence(self, *args, **kwargs):
        return _ph._step_phase_sequence(self, *args, **kwargs)

    def _step_pipeline_phase_state(self, *args, **kwargs):
        return _ph._step_pipeline_phase_state(self, *args, **kwargs)

    def _step_pipeline_rebuild(self, *args, **kwargs):
        return _ph._step_pipeline_rebuild(self, *args, **kwargs)

    def _step_pre_insert_inicio_row(self, *args, **kwargs):
        return _ph._step_pre_insert_inicio_row(self, *args, **kwargs)

    def _step_predict_next_phase(self, *args, **kwargs):
        return _ph._step_predict_next_phase(self, *args, **kwargs)

    def _step_prepare_pipeline_view(self, *args, **kwargs):
        return _ph._step_prepare_pipeline_view(self, *args, **kwargs)

    def _step_present_flow_name(self, *args, **kwargs):
        return _ph._step_present_flow_name(self, *args, **kwargs)

    def _step_record_current_phase_actual(self, *args, **kwargs):
        return _ph._step_record_current_phase_actual(self, *args, **kwargs)

    def _step_record_direct_event_snapshot(self, *args, **kwargs):
        return _ph._step_record_direct_event_snapshot(self, *args, **kwargs)

    def _step_record_direct_home_initial(self, *args, **kwargs):
        return _ph._step_record_direct_home_initial(self, *args, **kwargs)

    def _step_record_direct_initial_snapshot(self, *args, **kwargs):
        return _ph._step_record_direct_initial_snapshot(self, *args, **kwargs)

    def _step_record_direct_mesa_ready(self, *args, **kwargs):
        return _ph._step_record_direct_mesa_ready(self, *args, **kwargs)

    def _step_record_history(self, *args, **kwargs):
        return _ph._step_record_history(self, *args, **kwargs)

    def _step_refresh_pipeline_table(self, *args, **kwargs):
        return _ph._step_refresh_pipeline_table(self, *args, **kwargs)

    def _step_reset_sequence_view(self, *args, **kwargs):
        return _ph._step_reset_sequence_view(self, *args, **kwargs)

    def _step_runtime_refresh(self, *args, **kwargs):
        return _ph._step_runtime_refresh(self, *args, **kwargs)

    def _step_selected_object_name(self, *args, **kwargs):
        return _ph._step_selected_object_name(self, *args, **kwargs)

    def _step_set_exec_target(self, *args, **kwargs):
        return _ph._step_set_exec_target(self, *args, **kwargs)

    def _step_status_item(self, *args, **kwargs):
        return _ph._step_status_item(self, *args, **kwargs)

    def _step_update_phase_result(self, *args, **kwargs):
        return _ph._step_update_phase_result(self, *args, **kwargs)

    def _step_update_row_object_metrics(self, *args, **kwargs):
        return _ph._step_update_row_object_metrics(self, *args, **kwargs)

    def _step_upsert_history_row_ordered(self, *args, **kwargs):
        return _ph._step_upsert_history_row_ordered(self, *args, **kwargs)

    def _step_wait_for_phase(self, *args, **kwargs):
        return _ph._step_wait_for_phase(self, *args, **kwargs)

    def _step_window_hide(self, *args, **kwargs):
        return _ph._step_window_hide(self, *args, **kwargs)

    def _step_window_maybe_refresh(self, *args, **kwargs):
        return _ph._step_window_maybe_refresh(self, *args, **kwargs)

    def _step_window_refresh(self, *args, **kwargs):
        return _ph._step_window_refresh(self, *args, **kwargs)

    def _step_window_set_waiting(self, *args, **kwargs):
        return _ph._step_window_set_waiting(self, *args, **kwargs)

    # ------------------------------------------------------------------
    # Read gripper feedback
    # ------------------------------------------------------------------

    def _read_gripper_feedback_state(self, *args, **kwargs):
        return _ph._read_gripper_feedback_state(self, *args, **kwargs)

    # ------------------------------------------------------------------
    # Step joint (panel_motion_control)
    # ------------------------------------------------------------------

    def _step_joint(self, *args, **kwargs):
        return _mc._step_joint(self, *args, **kwargs)
