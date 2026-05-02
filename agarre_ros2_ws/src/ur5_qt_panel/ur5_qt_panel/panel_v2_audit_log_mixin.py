#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_audit_log_mixin.py
# Contenido: F14-step11 (2026-05-01) — mixin audit/log/ready/control_status.
"""Mixin audit + log + ready checks + control status (F14-step11).

Decimoprimer paso del refactor F14: 29 wrappers thin que cubren:

* Control status checks: _moveit_control_status, _manual_control_status,
  _external_publishers_for_topic, _bridge_publishers_only, etc.
* Audit / log: _emit_log, _metric_mark, _audit_root, _audit_append,
  _audit_write_json, _sha256_file, _should_emit_log, _log,
  _emit_log_throttled.
* UI button/state: _set_motion_lock, _set_btn_state,
  _set_launching_style, _clear_launching_if_timeout,
  _controller_drop_grace_active.
* Ready-basic checks: _require_ready_basic, _basic_ready_status,
  _pick_demo_remote_ready_status, _auto_release_drop_objects_when_ready,
  _require_ready_vision, _require_manual_ready, _block_if_managed.

Todos delegan a panel_helpers (_ph).
"""

from __future__ import annotations

from . import panel_helpers as _ph


class PanelV2AuditLogMixin:
    """29 wrappers thin de audit/log/ready/control_status."""

    def _moveit_control_status(self, *args, **kwargs):
        return _ph._moveit_control_status(self, *args, **kwargs)

    def _manual_control_status(self, *args, **kwargs):
        return _ph._manual_control_status(self, *args, **kwargs)

    def _external_publishers_for_topic(self, *args, **kwargs):
        return _ph._external_publishers_for_topic(self, *args, **kwargs)

    def _bridge_publishers_only(self, *args, **kwargs):
        return _ph._bridge_publishers_only(self, *args, **kwargs)

    def _set_robot_test_blocked(self, *args, **kwargs):
        return _ph._set_robot_test_blocked(self, *args, **kwargs)

    def _await_external_publishers_clear(self, *args, **kwargs):
        return _ph._await_external_publishers_clear(self, *args, **kwargs)

    def _schedule_robot_test_cleanup_check(self, *args, **kwargs):
        return _ph._schedule_robot_test_cleanup_check(self, *args, **kwargs)

    def _update_camera_topics_async(self, *args, **kwargs):
        return _ph._update_camera_topics_async(self, *args, **kwargs)

    def _emit_log(self, *args, **kwargs):
        return _ph._emit_log(self, *args, **kwargs)

    def _metric_mark(self, *args, **kwargs):
        return _ph._metric_mark(self, *args, **kwargs)

    def _audit_root(self, *args, **kwargs):
        return _ph._audit_root(self, *args, **kwargs)

    def _audit_append(self, *args, **kwargs):
        return _ph._audit_append(self, *args, **kwargs)

    def _audit_write_json(self, *args, **kwargs):
        return _ph._audit_write_json(self, *args, **kwargs)

    def _sha256_file(self, *args, **kwargs):
        return _ph._sha256_file(self, *args, **kwargs)

    def _should_emit_log(self, *args, **kwargs):
        return _ph._should_emit_log(self, *args, **kwargs)

    def _set_motion_lock(self, *args, **kwargs):
        return _ph._set_motion_lock(self, *args, **kwargs)

    def _set_btn_state(self, *args, **kwargs):
        return _ph._set_btn_state(self, *args, **kwargs)

    def _set_launching_style(self, *args, **kwargs):
        return _ph._set_launching_style(self, *args, **kwargs)

    def _clear_launching_if_timeout(self, *args, **kwargs):
        return _ph._clear_launching_if_timeout(self, *args, **kwargs)

    def _controller_drop_grace_active(self, *args, **kwargs):
        return _ph._controller_drop_grace_active(self, *args, **kwargs)

    def _require_ready_basic(self, *args, **kwargs):
        return _ph._require_ready_basic(self, *args, **kwargs)

    def _basic_ready_status(self, *args, **kwargs):
        return _ph._basic_ready_status(self, *args, **kwargs)

    def _pick_demo_remote_ready_status(self, *args, **kwargs):
        return _ph._pick_demo_remote_ready_status(self, *args, **kwargs)

    def _auto_release_drop_objects_when_ready(self, *args, **kwargs):
        return _ph._auto_release_drop_objects_when_ready(self, *args, **kwargs)

    def _require_ready_vision(self, *args, **kwargs):
        return _ph._require_ready_vision(self, *args, **kwargs)

    def _require_manual_ready(self, *args, **kwargs):
        return _ph._require_manual_ready(self, *args, **kwargs)

    def _log(self, *args, **kwargs):
        return _ph._log(self, *args, **kwargs)

    def _emit_log_throttled(self, *args, **kwargs):
        return _ph._emit_log_throttled(self, *args, **kwargs)

    def _block_if_managed(self, *args, **kwargs):
        return _ph._block_if_managed(self, *args, **kwargs)

    # F14-step9: 97 wrappers logs/async/external_state/status/pose_info/
    # gazebo/controllers/camera/bridge_presets/start/stop heredados de
    # PanelV2RuntimeDiagnosticsMixin. Ver panel_v2_runtime_diagnostics_mixin.py.

