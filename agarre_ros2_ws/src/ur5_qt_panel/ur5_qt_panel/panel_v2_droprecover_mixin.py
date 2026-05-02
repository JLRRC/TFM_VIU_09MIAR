#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_droprecover_mixin.py
# Contenido: F14-step (2026-05-02) — mixin DropRecover extraído de ControlPanelV2.
"""Mixin DropRecover extraído de ControlPanelV2 (F14).

F14-step14 — 19 wrappers thin de drop/release/recover/hold + helpers de pose. Todos delegan a panel_status_mgmt (_sm). 1 firma legacy (_release_objects sin propagar args).
"""

from __future__ import annotations

from . import panel_status_mgmt as _sm


class PanelV2DropRecoverMixin:
    """Wrappers thin de droprecover."""

    def _log_trace(self, *args, **kwargs):
        return _sm._log_trace(self, *args, **kwargs)

    def _apply_debug_button_style(self, *args, **kwargs):
        return _sm._apply_debug_button_style(self, *args, **kwargs)

    def _print_pose_snapshot(self, *args, **kwargs):
        return _sm._print_pose_snapshot(self, *args, **kwargs)

    def _drop_detach_supported(self, *args, **kwargs):
        return _sm._drop_detach_supported(self, *args, **kwargs)

    def _release_objects(self, *args, **kwargs):
        return _sm._release_objects(self)

    def _schedule_release_retry(self, *args, **kwargs):
        return _sm._schedule_release_retry(self, *args, **kwargs)

    def _attach_drop_objects(self, *args, **kwargs):
        return _sm._attach_drop_objects(self, *args, **kwargs)

    def _maybe_nudge_drop_objects(self, *args, **kwargs):
        return _sm._maybe_nudge_drop_objects(self, *args, **kwargs)

    def _resolve_set_pose_service(self, *args, **kwargs):
        return _sm._resolve_set_pose_service(self, *args, **kwargs)

    def _resolve_gz_cli(self, *args, **kwargs):
        return _sm._resolve_gz_cli(self, *args, **kwargs)

    def _resolve_world_sdf_path(self, *args, **kwargs):
        return _sm._resolve_world_sdf_path(self, *args, **kwargs)

    def _load_pick_demo_recover_sdf(self, *args, **kwargs):
        return _sm._load_pick_demo_recover_sdf(self, *args, **kwargs)

    def _run_gz_service_cli(self, *args, **kwargs):
        return _sm._run_gz_service_cli(self, *args, **kwargs)

    def _recover_pick_demo_to_table_gz(self, *args, **kwargs):
        return _sm._recover_pick_demo_to_table_gz(self, *args, **kwargs)

    def _maybe_hold_drop_objects(self, *args, **kwargs):
        return _sm._maybe_hold_drop_objects(self, *args, **kwargs)

    def _maybe_recover_pick_demo(self, *args, **kwargs):
        return _sm._maybe_recover_pick_demo(self, *args, **kwargs)

    def _recover_pick_demo_to_table(self, *args, **kwargs):
        return _sm._recover_pick_demo_to_table(self, *args, **kwargs)

    def _hold_drop_objects(self, *args, **kwargs):
        return _sm._hold_drop_objects(self, *args, **kwargs)

    def _hold_drop_objects_gz(self, *args, **kwargs):
        return _sm._hold_drop_objects_gz(self, *args, **kwargs)

    def _drop_hold_tick(self, *args, **kwargs):
        return _sm._drop_hold_tick(self, *args, **kwargs)

    # F14-step10: 64 wrappers subprocess + motion control heredados
    # de PanelV2SubprocessMotionMixin (panel_status_mgmt _sm + panel_motion_control _mc).

    # F14-step12: 27 wrappers TFM grasp + remote callbacks heredados
    # de PanelV2TfmRemoteMixin.

