#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_subprocess_motion_mixin.py
# Contenido: F14-step10 (2026-05-01) — mixin start/stop/refresh + motion control.
"""Mixin subprocess + motion control (F14-step10).

Décimo paso del refactor F14: 64 wrappers thin para gestión de
procesos del stack (gazebo, bridges, MoveIt, rosbag, system stats),
estado del MoveIt y motion control (joint sliders, baseline,
home, gripper force).

Cobertura por delegate target:

* panel_status_mgmt (_sm, 47 wrappers): start/stop/refresh
  workflow del stack, MoveIt status checks, listing de topics y
  actions, system stats, state machine.
* panel_motion_control (_mc, 17 wrappers): joint convergence,
  baseline motion, home, gripper force, send joints.

Tres firmas legacy preservadas (NO propagan *args/**kwargs):
``_stop_gazebo``, ``_go_home``, ``_send_joints``.
"""

from __future__ import annotations

from . import panel_status_mgmt as _sm
from . import panel_motion_control as _mc


class PanelV2SubprocessMotionMixin:
    """64 wrappers thin de subprocess + motion control."""

    def _nudge_drop_objects(self, *args, **kwargs):
        return _sm._nudge_drop_objects(self, *args, **kwargs)

    def _start_release_service(self, *args, **kwargs):
        return _sm._start_release_service(self, *args, **kwargs)

    def _start_world_tf_publisher(self, *args, **kwargs):
        return _sm._start_world_tf_publisher(self, *args, **kwargs)

    def _stop_world_tf_publisher(self, *args, **kwargs):
        return _sm._stop_world_tf_publisher(self, *args, **kwargs)

    def _stop_gazebo(self, *args, **kwargs):
        return _sm._stop_gazebo(self)

    def _start_robot_state_publisher(self, *args, **kwargs):
        return _sm._start_robot_state_publisher(self, *args, **kwargs)

    def _start_bridge(self, *args, **kwargs):
        return _sm._start_bridge(self, *args, **kwargs)

    def _spawn_controllers_async(self, *args, **kwargs):
        return _sm._spawn_controllers_async(self, *args, **kwargs)

    def _stop_bridge(self, *args, **kwargs):
        return _sm._stop_bridge(self, *args, **kwargs)

    def _start_moveit(self, *args, **kwargs):
        return _sm._start_moveit(self, *args, **kwargs)

    def _stop_moveit(self, *args, **kwargs):
        return _sm._stop_moveit(self, *args, **kwargs)

    def _wait_for_moveit_ready(self, *args, **kwargs):
        return _sm._wait_for_moveit_ready(self, *args, **kwargs)

    def _start_moveit_bridge(self, *args, **kwargs):
        return _sm._start_moveit_bridge(self, *args, **kwargs)

    def _stop_moveit_bridge(self, *args, **kwargs):
        return _sm._stop_moveit_bridge(self, *args, **kwargs)

    def _clear_moveit_bridge_launching(self, *args, **kwargs):
        return _sm._clear_moveit_bridge_launching(self, *args, **kwargs)

    def _kill_proc(self, *args, **kwargs):
        return _sm._kill_proc(self, *args, **kwargs)

    def _proc_alive(self, *args, **kwargs):
        return _sm._proc_alive(self, *args, **kwargs)

    def _save_home_from_sliders(self, *args, **kwargs):
        return _sm._save_home_from_sliders(self, *args, **kwargs)

    def _rosbag_running(self, *args, **kwargs):
        return _sm._rosbag_running(self, *args, **kwargs)

    def _start_bag(self, *args, **kwargs):
        return _sm._start_bag(self, *args, **kwargs)

    def _stop_bag(self, *args, **kwargs):
        return _sm._stop_bag(self, *args, **kwargs)

    def _refresh_status_sync(self, *args, **kwargs):
        return _sm._refresh_status_sync(self, *args, **kwargs)

    def _refresh_status_async(self, *args, **kwargs):
        return _sm._refresh_status_async(self, *args, **kwargs)

    def _apply_status(self, *args, **kwargs):
        return _sm._apply_status(self, *args, **kwargs)

    def _moveit_topics_ready(self, *args, **kwargs):
        return _sm._moveit_topics_ready(self, *args, **kwargs)

    def _moveit_status_ready(self, *args, **kwargs):
        return _sm._moveit_status_ready(self, *args, **kwargs)

    def _moveit_action_ready(self, *args, **kwargs):
        return _sm._moveit_action_ready(self, *args, **kwargs)

    def _list_topic_names(self, *args, **kwargs):
        return _sm._list_topic_names(self, *args, **kwargs)

    def _list_action_names(self, *args, **kwargs):
        return _sm._list_action_names(self, *args, **kwargs)

    def _topic_has_any_publishers(self, *args, **kwargs):
        return _sm._topic_has_any_publishers(self, *args, **kwargs)

    def _world_frame_last_first(self, *args, **kwargs):
        return _sm._world_frame_last_first(self, *args, **kwargs)

    def _world_frame_config_first(self, *args, **kwargs):
        return _sm._world_frame_config_first(self, *args, **kwargs)

    def _follow_joint_traj_ready(self, *args, **kwargs):
        return _sm._follow_joint_traj_ready(self, *args, **kwargs)

    def _moveit_bridge_detected(self, *args, **kwargs):
        return _sm._moveit_bridge_detected(self, *args, **kwargs)

    def _move_group_startup_ready(self, *args, **kwargs):
        return _sm._move_group_startup_ready(self, *args, **kwargs)

    def _move_group_ready(self, *args, **kwargs):
        return _sm._move_group_ready(self, *args, **kwargs)

    def _moveit_ready(self, *args, **kwargs):
        return _sm._moveit_ready(self, *args, **kwargs)

    def _update_moveit_status_label(self, *args, **kwargs):
        return _sm._update_moveit_status_label(self, *args, **kwargs)

    def _update_system_stats(self, *args, **kwargs):
        return _sm._update_system_stats(self, *args, **kwargs)

    def _set_stat_label(self, *args, **kwargs):
        return _sm._set_stat_label(self, *args, **kwargs)

    def _known_process_pids(self, *args, **kwargs):
        return _sm._known_process_pids(self, *args, **kwargs)

    def _list_stale_processes(self, *args, **kwargs):
        return _sm._list_stale_processes(self, *args, **kwargs)

    def _detect_stale_processes(self, *args, **kwargs):
        return _sm._detect_stale_processes(self, *args, **kwargs)

    def _refresh_controls(self, *args, **kwargs):
        return _sm._refresh_controls(self, *args, **kwargs)

    def _maybe_auto_run_pick_demo(self, *args, **kwargs):
        return _sm._maybe_auto_run_pick_demo(self, *args, **kwargs)

    def _wait_for_state_change(self, *args, **kwargs):
        return _sm._wait_for_state_change(self, *args, **kwargs)

    def _schedule_controller_check(self, *args, **kwargs):
        return _sm._schedule_controller_check(self, *args, **kwargs)


    def _update_ui_state(self, *args, **kwargs):
        return _mc._update_ui_state(self, *args, **kwargs)

    def _effective_mode(self, *args, **kwargs):
        return _mc._effective_mode(self, *args, **kwargs)

    def _apply_home_joint2_offset(self, *args, **kwargs):
        return _mc._apply_home_joint2_offset(self, *args, **kwargs)

    def _schedule_home_offset_retry(self, *args, **kwargs):
        return _mc._schedule_home_offset_retry(self, *args, **kwargs)

    def _get_home_joint_pose(self, *args, **kwargs):
        return _mc._get_home_joint_pose(self, *args, **kwargs)

    def _get_gripper_force(self, *args, **kwargs):
        return _mc._get_gripper_force(self, *args, **kwargs)

    def _wait_for_joint_convergence(self, *args, **kwargs):
        return _mc._wait_for_joint_convergence(self, *args, **kwargs)

    def _run_baseline_motion(self, *args, **kwargs):
        return _mc._run_baseline_motion(self, *args, **kwargs)

    def _go_home(self, *args, **kwargs):
        return _mc._go_home(self)

    def _set_test_failed(self, *args, **kwargs):
        return _mc._set_test_failed(self, *args, **kwargs)

    def _set_robot_test_done(self, *args, **kwargs):
        return _mc._set_robot_test_done(self, *args, **kwargs)

    def _set_panel_flow_state(self, *args, **kwargs):
        return _mc._set_panel_flow_state(self, *args, **kwargs)

    def _update_panel_flow_state(self, *args, **kwargs):
        return _mc._update_panel_flow_state(self, *args, **kwargs)

    def _compute_reach_overlay_points(self, *args, **kwargs):
        return _mc._compute_reach_overlay_points(self, *args, **kwargs)

    # F14-step8: _step_joint heredado de PanelV2StepDebugMixin.

    def _maybe_send_auto(self, *args, **kwargs):
        return _mc._maybe_send_auto(self, *args, **kwargs)

    def _send_joints(self, *args, **kwargs):
        return _mc._send_joints(self)

    def _send_joints_retry(self, *args, **kwargs):
        return _mc._send_joints_retry(self, *args, **kwargs)


