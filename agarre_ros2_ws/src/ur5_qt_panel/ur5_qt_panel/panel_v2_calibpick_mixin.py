#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_calibpick_mixin.py
# Contenido: F14-step (2026-05-02) — mixin CalibPick extraído de ControlPanelV2.
"""Mixin CalibPick extraído de ControlPanelV2 (F14).

F14-step15 — wrappers thin de calibración, pick demo, slider, joint queries, table/safety, object report, go_home/table/basket. Aliases: _ca, _om.
"""

from __future__ import annotations




class PanelV2CalibPickMixin:
    """Wrappers thin de calibpick."""

    def _publish_calib_grid_marker(self, *args, **kwargs):
        return _ca._publish_calib_grid_marker(self, *args, **kwargs)

    def _start_calibration(self, *args, **kwargs):
        return _ca._start_calibration(self)

    def _set_calibrate_button_text(self, *args, **kwargs):
        return _ca._set_calibrate_button_text(self, *args, **kwargs)

    def _capture_calibration_frame(self, *args, **kwargs):
        return _ca._capture_calibration_frame(self, *args, **kwargs)

    def _auto_calibrate_from_camera(self, *args, **kwargs):
        return _ca._auto_calibrate_from_camera(self, *args, **kwargs)

    def _pick_confirm_dialog(self, *args, **kwargs):
        return _ca._pick_confirm_dialog(*args, **kwargs)

    def _run_pick_demo(self, *args, **kwargs):
        return _ca._run_pick_demo(self)

    def _run_pick_object(self, *args, **kwargs):
        return _ca._run_pick_object(self)

    def _get_object_world_position(self, *args, **kwargs):
        return _ca._get_object_world_position(self, *args, **kwargs)

    def _on_slider_change(self, *args, **kwargs):
        return _ca._on_slider_change(self, *args, **kwargs)

    def _slider_to_deg(self, *args, **kwargs):
        return _ca._slider_to_deg(self, *args, **kwargs)

    def _current_joint_positions_rad(self, *args, **kwargs):
        return _ca._current_joint_positions_rad(self, *args, **kwargs)

    def _resolve_table_top_z(self, *args, **kwargs):
        return _om._resolve_table_top_z(self, *args, **kwargs)

    def _resolve_safety_surface_z(self, *args, **kwargs):
        return _om._resolve_safety_surface_z(self, *args, **kwargs)

    def _check_robot_above_table(self, *args, **kwargs):
        return _om._check_robot_above_table(self, *args, **kwargs)

    def _load_sdf_geometry_cache(self, *args, **kwargs):
        return _om._load_sdf_geometry_cache(self, *args, **kwargs)

    def _load_joint_limits(self, *args, **kwargs):
        return _om._load_joint_limits(self, *args, **kwargs)

    def _post_calibration_pipeline(self, *args, **kwargs):
        return _om._post_calibration_pipeline(self, *args, **kwargs)

    def _build_object_report(self, *args, **kwargs):
        return _om._build_object_report(self, *args, **kwargs)

    def _is_pickable(self, *args, **kwargs):
        return _om._is_pickable(self, *args, **kwargs)

    def _log_object_report(self, *args, **kwargs):
        return _om._log_object_report(self, *args, **kwargs)

    def _go_table(self, *args, **kwargs):
        return _om._go_table(self)

    def _go_basket(self, *args, **kwargs):
        return _om._go_basket(self)

    def _toggle_gripper_button(self, *args, **kwargs):
        return _om._toggle_gripper_button(self, *args, **kwargs)

    def _on_camera_click(self, *args, **kwargs):
        return _om._on_camera_click(self, *args, **kwargs)

    def _load_table_calibration(self, *args, **kwargs):
        return _om._load_table_calibration(self, *args, **kwargs)

    def _refresh_objects_from_gz_async(self, *args, **kwargs):
        return _om._refresh_objects_from_gz_async(self, *args, **kwargs)

    def _resolve_pose_object_name(self, *args, **kwargs):
        return _om._resolve_pose_object_name(self, *args, **kwargs)

    def _extract_pose_updates(self, *args, **kwargs):
        return _om._extract_pose_updates(self, *args, **kwargs)

    def _refresh_objects_from_gz(self, *args, **kwargs):
        return _om._refresh_objects_from_gz(self, *args, **kwargs)

    def _settle_targets(self, *args, **kwargs):
        return _om._settle_targets(self, *args, **kwargs)

    def _read_world_pose_info(self, *args, **kwargs):
        return _om._read_world_pose_info(self, *args, **kwargs)



