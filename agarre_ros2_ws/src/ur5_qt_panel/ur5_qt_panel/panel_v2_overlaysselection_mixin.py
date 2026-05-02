#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_overlaysselection_mixin.py
# Contenido: F14-step (2026-05-02) — mixin OverlaysSelection extraído de ControlPanelV2.
"""Mixin OverlaysSelection extraído de ControlPanelV2 (F14).

F14-step16 — wrappers thin de overlays (calib/grasp/reach/tcp/test_corner) + selección de objeto + history/fps. Aliases: _cs, _do.
"""

from __future__ import annotations

from . import panel_calib_selection as _cs
from . import panel_draw_overlays as _do


class PanelV2OverlaysSelectionMixin:
    """Wrappers thin de overlaysselection."""

    def _compute_homography(self, *args, **kwargs):
        return _do._compute_homography(*args, **kwargs)

    def _draw_calib_overlay(self, *args, **kwargs):
        return _do._draw_calib_overlay(self, *args, **kwargs)

    def _draw_selection_overlay(self, *args, **kwargs):
        return _do._draw_selection_overlay(self, *args, **kwargs)

    def _draw_grasp_overlay(self, *args, **kwargs):
        return _do._draw_grasp_overlay(self, *args, **kwargs)

    def _tfm_overlay_focus_enabled(self, *args, **kwargs):
        return _do._tfm_overlay_focus_enabled(self, *args, **kwargs)

    def _should_draw_reach_overlay(self, *args, **kwargs):
        return _do._should_draw_reach_overlay(self, *args, **kwargs)

    def _should_draw_selection_overlay(self, *args, **kwargs):
        return _do._should_draw_selection_overlay(self, *args, **kwargs)

    def _should_draw_test_corner_overlay(self, *args, **kwargs):
        return _do._should_draw_test_corner_overlay(self, *args, **kwargs)

    def _should_draw_tcp_pose_overlay(self, *args, **kwargs):
        return _do._should_draw_tcp_pose_overlay(self, *args, **kwargs)

    def _base_to_world_coords(self, *args, **kwargs):
        return _do._base_to_world_coords(self, *args, **kwargs)

    def _compute_test_corner_base_points(self, *args, **kwargs):
        return _do._compute_test_corner_base_points(self, *args, **kwargs)

    def _compute_test_corner_world_points(self, *args, **kwargs):
        return _do._compute_test_corner_world_points(self, *args, **kwargs)

    def _draw_test_corner_overlay(self, *args, **kwargs):
        return _do._draw_test_corner_overlay(self, *args, **kwargs)

    def _draw_tcp_pose_overlay(self, *args, **kwargs):
        return _do._draw_tcp_pose_overlay(self, *args, **kwargs)

    def _save_overhead_frame_with_overlays(self, *args, **kwargs):
        return _do._save_overhead_frame_with_overlays(self, *args, **kwargs)

    def _save_grasp_overlay(self, *args, **kwargs):
        return _do._save_grasp_overlay(self, *args, **kwargs)

    def _refresh_grasp_overlay_now(self, *args, **kwargs):
        return _do._refresh_grasp_overlay_now(self, *args, **kwargs)

    def _draw_reach_overlay(self, *args, **kwargs):
        return _do._draw_reach_overlay(self, *args, **kwargs)


    def _handle_calibration_click(self, *args, **kwargs):
        return _cs._handle_calibration_click(self, *args, **kwargs)

    def _finish_calibration(self, *args, **kwargs):
        return _cs._finish_calibration(self, *args, **kwargs)

    def _handle_object_selection_click(self, *args, **kwargs):
        return _cs._handle_object_selection_click(self, *args, **kwargs)

    def _select_object(self, *args, **kwargs):
        return _cs._select_object(self, *args, **kwargs)

    def _log_selection_tf(self, *args, **kwargs):
        return _cs._log_selection_tf(self, *args, **kwargs)

    def _start_tf_diagnose_async(self, *args, **kwargs):
        return _cs._start_tf_diagnose_async(self, *args, **kwargs)

    def _start_pick_tf_resolve(self, *args, **kwargs):
        return _cs._start_pick_tf_resolve(self, *args, **kwargs)

    def _selection_candidate(self, *args, **kwargs):
        return _cs._selection_candidate(self, *args, **kwargs)

    def _selection_to_base(self, *args, **kwargs):
        return _cs._selection_to_base(self, *args, **kwargs)

    def _on_object_clicked(self, *args, **kwargs):
        return _cs._on_object_clicked(self, *args, **kwargs)

    def _update_objects(self, *args, **kwargs):
        return _cs._update_objects(self, *args, **kwargs)

    def _push_history(self, *args, **kwargs):
        return _cs._push_history(self, *args, **kwargs)

    def _mean_history(self, *args, **kwargs):
        return _cs._mean_history(self, *args, **kwargs)

    def _update_fps_stats(self, *args, **kwargs):
        return _cs._update_fps_stats(self, *args, **kwargs)

