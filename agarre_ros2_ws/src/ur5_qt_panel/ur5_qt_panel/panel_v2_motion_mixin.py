#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_motion_mixin.py
# Contenido: F14-step5 (2026-05-01) — mixin de motion + waits extraído de ControlPanelV2.
"""Mixin de motion + waits (F14-step5).

Quinto paso del refactor F14: extrae los wrappers thin de la lógica
de cliente de action FollowJointTrajectory + helpers de espera de
joint/TCP para sincronizar el flujo del pick demo. Todos delegan al
módulo ``panel_state_methods`` (alias ``_stm``).

Métodos agrupados:

* ``_traj_action_target`` — nombre del action target del controller.
* ``_resolve_traj_action_name`` — resuelve el nombre canónico.
* ``_get_action_client`` — cache de ActionClient para FJT.
* ``_wait_action_server`` — espera el server (helper estático).
* ``_format_action_error`` — formato uniforme de errores de action.
* ``_joint_motion_since`` — detección de movimiento desde un instante.
* ``_wait_for_joint_target`` — espera blocking hasta alcanzar joints.
* ``_wait_for_tcp_base_z`` — espera blocking hasta alcanzar Z base.
* ``_wait_for_tcp_base_target`` — espera blocking hasta TCP target.

``ControlPanelV2`` lo incluye como mixin junto a los anteriores.
"""

from __future__ import annotations

from . import panel_state_methods as _stm


class PanelV2MotionMixin:
    """Wrappers thin de lógica de motion / action client / waits."""

    def _traj_action_target(self, *args, **kwargs):
        return _stm._traj_action_target(self, *args, **kwargs)

    def _resolve_traj_action_name(self, *args, **kwargs):
        return _stm._resolve_traj_action_name(self, *args, **kwargs)

    def _get_action_client(self, *args, **kwargs):
        return _stm._get_action_client(self, *args, **kwargs)

    def _wait_action_server(self, *args, **kwargs):
        # Helper estático — NO pasa self.
        return _stm._wait_action_server(*args, **kwargs)

    def _format_action_error(self, *args, **kwargs):
        return _stm._format_action_error(self, *args, **kwargs)

    def _joint_motion_since(self, *args, **kwargs):
        return _stm._joint_motion_since(self, *args, **kwargs)

    def _wait_for_joint_target(self, *args, **kwargs):
        return _stm._wait_for_joint_target(self, *args, **kwargs)

    def _wait_for_tcp_base_z(self, *args, **kwargs):
        return _stm._wait_for_tcp_base_z(self, *args, **kwargs)

    def _wait_for_tcp_base_target(self, *args, **kwargs):
        return _stm._wait_for_tcp_base_target(self, *args, **kwargs)
