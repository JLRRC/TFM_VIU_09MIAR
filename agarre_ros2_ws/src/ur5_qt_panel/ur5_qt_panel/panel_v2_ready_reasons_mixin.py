#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_ready_reasons_mixin.py
# Contenido: F14-step13 (2026-05-02) — mixin ready-reasons + status checks.
"""Mixin ready-reasons + status checks (F14-step13).

15 wrappers thin que cubren las funciones helper del panel para
diagnosticar la razón por la que un componente no está ready, y
para reportar status estructurado de actuadores/visión/calibración.

Todos delegan a panel_helpers (_ph). 5 son helpers estáticos sin
self: _ros_node_not_ready_reason, _controller_manager_not_ready_reason,
_list_controllers_not_ready_reason, _tfm_infer_waitable_reason,
_pose_info_not_ready_reason.
"""

from __future__ import annotations

from . import panel_helpers as _ph


class PanelV2ReadyReasonsMixin:
    """15 wrappers thin de ready-reasons + status checks."""

    def _moveit_not_ready_reason(self, *args, **kwargs):
        return _ph._moveit_not_ready_reason(self, *args, **kwargs)

    def _set_moveit_wait_status(self, *args, **kwargs):
        return _ph._set_moveit_wait_status(self, *args, **kwargs)

    def _controllers_not_ready_reason(self, *args, **kwargs):
        return _ph._controllers_not_ready_reason(self, *args, **kwargs)

    def _ros_node_not_ready_reason(self, *args, **kwargs):
        return _ph._ros_node_not_ready_reason(*args, **kwargs)

    def _controller_manager_not_ready_reason(self, *args, **kwargs):
        return _ph._controller_manager_not_ready_reason(*args, **kwargs)

    def _list_controllers_not_ready_reason(self, *args, **kwargs):
        return _ph._list_controllers_not_ready_reason(*args, **kwargs)

    def _camera_not_ready_reason(self, *args, **kwargs):
        return _ph._camera_not_ready_reason(self, *args, **kwargs)

    def _tfm_experiment_ready_status(self, *args, **kwargs):
        return _ph._tfm_experiment_ready_status(self, *args, **kwargs)

    def _tfm_infer_ready_status(self, *args, **kwargs):
        return _ph._tfm_infer_ready_status(self, *args, **kwargs)

    def _tfm_infer_waitable_reason(self, *args, **kwargs):
        return _ph._tfm_infer_waitable_reason(*args, **kwargs)

    def _current_grasp_status(self, *args, **kwargs):
        return _ph._current_grasp_status(self, *args, **kwargs)

    def _restore_execute_selection_context(self, *args, **kwargs):
        return _ph._restore_execute_selection_context(self, *args, **kwargs)

    def _calibration_action_status(self, *args, **kwargs):
        return _ph._calibration_action_status(self, *args, **kwargs)

    def _pose_info_not_ready_reason(self, *args, **kwargs):
        return _ph._pose_info_not_ready_reason(*args, **kwargs)

    def _tf_not_ready_reason(self, *args, **kwargs):
        return _ph._tf_not_ready_reason(self, *args, **kwargs)

    # F14-step11: 29 wrappers audit/log/ready/control_status heredados
    # de PanelV2AuditLogMixin.

