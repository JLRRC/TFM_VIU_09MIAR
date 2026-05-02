#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_runtime_diagnostics_mixin.py
# Contenido: F14-step9 (2026-05-01) — mixin runtime + diagnostics + camera + controllers + gazebo.
"""Mixin runtime/diagnostics/camera/controllers/gazebo (F14-step9).

Noveno paso del refactor F14: mixin grande que agrupa 97 wrappers
thin del panel relacionados con:

* Logs / async / external state (panel_helpers _ph, ~24).
* Gazebo state + controllers + camera + bridge presets + start/stop
  (panel_gz_startup _gs, ~73).

Tres wrappers preservan el comportamiento legacy de NO propagar
*args/**kwargs: ``_recover_runtime``, ``_start_gazebo``,
``_toggle_debug_poses``.

Extracción genérica con generación automática para evitar errores
manuales en 97 firmas idénticas.
"""

from __future__ import annotations

from . import panel_helpers as _ph
from . import panel_gz_startup as _gs


class PanelV2RuntimeDiagnosticsMixin:
    """97 wrappers thin de runtime / diagnostics / camera / gazebo."""

    def _log_error(self, *args, **kwargs):
        return _ph._log_error(self, *args, **kwargs)

    def _log_warning(self, *args, **kwargs):
        return _ph._log_warning(self, *args, **kwargs)

    def _on_async_error(self, *args, **kwargs):
        return _ph._on_async_error(self, *args, **kwargs)

    def _run_ui_callable(self, *args, **kwargs):
        return _ph._run_ui_callable(self, *args, **kwargs)

    def _run_ui_delayed(self, *args, **kwargs):
        return _ph._run_ui_delayed(self, *args, **kwargs)

    # F14-step8: 72 wrappers step/direct/cart_debug/debug_motion heredados
    # de PanelV2StepDebugMixin. Ver panel_v2_step_debug_mixin.py.

    def _run_async(self, *args, **kwargs):
        return _ph._run_async(self, *args, **kwargs)

    def _log_ros_message(self, *args, **kwargs):
        return _ph._log_ros_message(self, *args, **kwargs)

    def _on_system_state_update(self, *args, **kwargs):
        return _ph._on_system_state_update(self, *args, **kwargs)

    def _external_state_active(self, *args, **kwargs):
        return _ph._external_state_active(self, *args, **kwargs)

    def _resolve_external_state(self, *args, **kwargs):
        return _ph._resolve_external_state(self, *args, **kwargs)

    def _apply_external_system_state(self, *args, **kwargs):
        return _ph._apply_external_system_state(self, *args, **kwargs)

    def _log_camera_diagnostics(self, *args, **kwargs):
        return _ph._log_camera_diagnostics(self, *args, **kwargs)

    def _sync_moveit_from_system_state(self, *args, **kwargs):
        return _ph._sync_moveit_from_system_state(self, *args, **kwargs)

    def _clock_status(self, *args, **kwargs):
        return _ph._clock_status(self, *args, **kwargs)

    def _joint_states_status(self, *args, **kwargs):
        return _ph._joint_states_status(self, *args, **kwargs)

    def _bridge_transport_detected(self, *args, **kwargs):
        return _ph._bridge_transport_detected(self, *args, **kwargs)

    def _bridge_ready_status(self, *args, **kwargs):
        return _ph._bridge_ready_status(self, *args, **kwargs)

    def _tf_chain_ready_status(self, *args, **kwargs):
        return _ph._tf_chain_ready_status(self, *args, **kwargs)

    def _camera_depth_expectation(self, *args, **kwargs):
        return _ph._camera_depth_expectation(self, *args, **kwargs)

    def _camera_runtime_flags(self, *args, **kwargs):
        return _ph._camera_runtime_flags(self, *args, **kwargs)

    def _sync_external_release_state(self, *args, **kwargs):
        return _ph._sync_external_release_state(self, *args, **kwargs)

    def _pose_info_topic(self, *args, **kwargs):
        return _ph._pose_info_topic(self, *args, **kwargs)

    def _pose_info_active(self, *args, **kwargs):
        return _ph._pose_info_active(self, *args, **kwargs)

    def _pose_info_ready(self, *args, **kwargs):
        return _ph._pose_info_ready(self, *args, **kwargs)

    def _gazebo_process_signal(self, *args, **kwargs):
        return _ph._gazebo_process_signal(self, *args, **kwargs)



    def _gazebo_bridge_signal(self, *args, **kwargs):
        return _gs._gazebo_bridge_signal(self, *args, **kwargs)

    def _gazebo_state(self, *args, **kwargs):
        return _gs._gazebo_state(self, *args, **kwargs)

    def _ros2_control_available(self, *args, **kwargs):
        return _gs._ros2_control_available(self, *args, **kwargs)

    def _controller_manager_path(self, *args, **kwargs):
        return _gs._controller_manager_path(self, *args, **kwargs)

    def _select_traj_topic(self, *args, **kwargs):
        return _gs._select_traj_topic(self, *args, **kwargs)

    def _ensure_pose_subscription(self, *args, **kwargs):
        return _gs._ensure_pose_subscription(self, *args, **kwargs)

    def _discover_pose_info_topic(self, *args, **kwargs):
        return _gs._discover_pose_info_topic(self, *args, **kwargs)

    def _start_pose_info_watch(self, *args, **kwargs):
        return _gs._start_pose_info_watch(self, *args, **kwargs)

    def _update_pose_info_status(self, *args, **kwargs):
        return _gs._update_pose_info_status(self, *args, **kwargs)

    def _log_button(self, *args, **kwargs):
        return _gs._log_button(self, *args, **kwargs)

    def _cleanup_stray_processes(self, *args, **kwargs):
        return _gs._cleanup_stray_processes(self, *args, **kwargs)

    def _clean_cache_dirs(self, *args, **kwargs):
        return _gs._clean_cache_dirs(self, *args, **kwargs)

    def _close_terminal(self, *args, **kwargs):
        return _gs._close_terminal(self, *args, **kwargs)

    def _refresh_camera_topics(self, *args, **kwargs):
        return _gs._refresh_camera_topics(self, *args, **kwargs)

    def _controllers_ready(self, *args, **kwargs):
        return _gs._controllers_ready(self, *args, **kwargs)

    def _is_transient_controller_reason(self, *args, **kwargs):
        return _gs._is_transient_controller_reason(self, *args, **kwargs)

    def _controllers_last_ok_age(self, *args, **kwargs):
        return _gs._controllers_last_ok_age(self, *args, **kwargs)

    def _can_use_controller_last_ok(self, *args, **kwargs):
        return _gs._can_use_controller_last_ok(self, *args, **kwargs)

    def _controller_state_kind(self, *args, **kwargs):
        return _gs._controller_state_kind(self, *args, **kwargs)

    def _list_controllers(self, *args, **kwargs):
        return _gs._list_controllers(self, *args, **kwargs)

    def _wait_for_controllers_ready(self, *args, **kwargs):
        return _gs._wait_for_controllers_ready(self, *args, **kwargs)

    def _schedule_camera_health_check(self, *args, **kwargs):
        return _gs._schedule_camera_health_check(self, *args, **kwargs)

    def _check_camera_topic_health(self, *args, **kwargs):
        return _gs._check_camera_topic_health(self, *args, **kwargs)

    def _update_camera_topics(self, *args, **kwargs):
        return _gs._update_camera_topics(self, *args, **kwargs)

    def _connect_camera(self, *args, **kwargs):
        return _gs._connect_camera(self, *args, **kwargs)

    def _switch_camera_topic(self, *args, **kwargs):
        return _gs._switch_camera_topic(self, *args, **kwargs)

    def _set_far_front_camera_view(self, *args, **kwargs):
        return _gs._set_far_front_camera_view(self, *args, **kwargs)

    def _set_top_camera_view(self, *args, **kwargs):
        return _gs._set_top_camera_view(self, *args, **kwargs)

    def _set_wrist_camera_view(self, *args, **kwargs):
        return _gs._set_wrist_camera_view(self, *args, **kwargs)

    def _subscribe_camera(self, *args, **kwargs):
        return _gs._subscribe_camera(self, *args, **kwargs)

    def _start_camera_health_check(self, *args, **kwargs):
        return _gs._start_camera_health_check(self, *args, **kwargs)

    def _unsubscribe_camera(self, *args, **kwargs):
        return _gs._unsubscribe_camera(self, *args, **kwargs)

    def _clear_camera_frame(self, *args, **kwargs):
        return _gs._clear_camera_frame(self, *args, **kwargs)

    def _resolve_camera_msg_type(self, *args, **kwargs):
        return _gs._resolve_camera_msg_type(self, *args, **kwargs)

    def _auto_connect_camera(self, *args, **kwargs):
        return _gs._auto_connect_camera(self, *args, **kwargs)

    def _ensure_ros_worker_started(self, *args, **kwargs):
        return _gs._ensure_ros_worker_started(self, *args, **kwargs)

    def _ensure_grasp_rect_subscription(self, *args, **kwargs):
        return _gs._ensure_grasp_rect_subscription(self, *args, **kwargs)

    def _auto_subscribe_joints(self, *args, **kwargs):
        return _gs._auto_subscribe_joints(self, *args, **kwargs)

    def _discover_joint_states_topic(self, *args, **kwargs):
        return _gs._discover_joint_states_topic(self, *args, **kwargs)

    def _on_bridge_ready(self, *args, **kwargs):
        return _gs._on_bridge_ready(self, *args, **kwargs)

    def _on_image(self, *args, **kwargs):
        return _gs._on_image(self, *args, **kwargs)

    def _on_grasp_rect(self, *args, **kwargs):
        return _gs._on_grasp_rect(self, *args, **kwargs)

    def _reset_camera_retry_backoff(self, *args, **kwargs):
        return _gs._reset_camera_retry_backoff(self, *args, **kwargs)

    def _refresh_camera_display(self, *args, **kwargs):
        return _gs._refresh_camera_display(self, *args, **kwargs)

    def _check_camera_stream(self, *args, **kwargs):
        return _gs._check_camera_stream(self, *args, **kwargs)

    def _on_joint_state(self, *args, **kwargs):
        return _gs._on_joint_state(self, *args, **kwargs)

    def _fill_worlds(self, *args, **kwargs):
        return _gs._fill_worlds(self, *args, **kwargs)

    def _fill_bridge_presets(self, *args, **kwargs):
        return _gs._fill_bridge_presets(self, *args, **kwargs)

    def _apply_bridge_preset(self, *args, **kwargs):
        return _gs._apply_bridge_preset(self, *args, **kwargs)

    def _choose_world(self, *args, **kwargs):
        return _gs._choose_world(self, *args, **kwargs)

    def _choose_yaml(self, *args, **kwargs):
        return _gs._choose_yaml(self, *args, **kwargs)

    def _run_script(self, *args, **kwargs):
        return _gs._run_script(self, *args, **kwargs)

    def _toggle_debug(self, *args, **kwargs):
        return _gs._toggle_debug(self, *args, **kwargs)

    def _start_all(self, *args, **kwargs):
        return _gs._start_all(self, *args, **kwargs)

    def _log_moveit_autostart_blocked(self, *args, **kwargs):
        return _gs._log_moveit_autostart_blocked(self, *args, **kwargs)

    def _on_start_fatal(self, *args, **kwargs):
        return _gs._on_start_fatal(self, *args, **kwargs)

    def _stop_all(self, *args, **kwargs):
        return _gs._stop_all(self, *args, **kwargs)

    def _recover_runtime(self, *args, **kwargs):
        return _gs._recover_runtime(self)

    def _system_running(self, *args, **kwargs):
        return _gs._system_running(self, *args, **kwargs)

    def _schedule_start_enable_check(self, *args, **kwargs):
        return _gs._schedule_start_enable_check(self, *args, **kwargs)

    def _start_gazebo(self, *args, **kwargs):
        return _gs._start_gazebo(self)

    def _parse_first_json_object(self, *args, **kwargs):
        return _gs._parse_first_json_object(self, *args, **kwargs)

    def _detect_world_name(self, *args, **kwargs):
        return _gs._detect_world_name(self, *args, **kwargs)

    def _read_world_stats(self, *args, **kwargs):
        return _gs._read_world_stats(self, *args, **kwargs)

    def _try_unpause_world(self, *args, **kwargs):
        return _gs._try_unpause_world(self, *args, **kwargs)

    def _probe_pose_motion(self, *args, **kwargs):
        return _gs._probe_pose_motion(self, *args, **kwargs)

    def check_physics_runtime(self, *args, **kwargs):
        return _gs.check_physics_runtime(self, *args, **kwargs)

    def _schedule_physics_runtime_check(self, *args, **kwargs):
        return _gs._schedule_physics_runtime_check(self, *args, **kwargs)

    def _throw_objects(self, *args, **kwargs):
        return _gs._throw_objects(self, *args, **kwargs)

    def _toggle_debug_poses(self, *args, **kwargs):
        return _gs._toggle_debug_poses(self)

    def _start_debug_poses(self, *args, **kwargs):
        return _gs._start_debug_poses(self, *args, **kwargs)

    def _stop_debug_poses(self, *args, **kwargs):
        return _gs._stop_debug_poses(self, *args, **kwargs)



