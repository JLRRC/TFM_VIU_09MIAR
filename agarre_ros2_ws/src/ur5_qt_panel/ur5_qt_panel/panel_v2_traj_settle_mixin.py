#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_traj_settle_mixin.py
# Contenido: F14-step6 (2026-05-01) — mixin trajectoria + objects settle.
"""Mixin trayectoria FollowJointTrajectory + objects settle (F14-step6).

Sexto paso del refactor F14: extrae los wrappers thin de:

* Envío y fallback de trajectories al action FJT.
* Espera a que los objetos del mundo Gazebo terminen de "asentarse"
  (reposo físico tras spawn / drop) antes de continuar el flujo del
  pick demo.

Métodos agrupados (13 wrappers):

Trayectoria:
* ``_send_joint_trajectory_action`` — envía un FJT goal con timeout.
* ``_schedule_traj_action_fallback`` — fallback async si el server
  no responde.
* ``_clamp_joint_positions`` — limita a joint_limits.yaml.
* ``_log_traj_action_fallback`` — emisión estructurada del fallback.

Objects settle:
* ``_start_objects_settle_watch`` — arranca observador de pose/info.
* ``_invalidate_settle`` — invalida el estado settle vigente.
* ``_run_fall_test_async`` — fall test físico async.
* ``_objects_settle_worker`` — hilo de detección de reposo.
* ``_handle_objects_settled`` — callback al detectar reposo.
* ``_log_calib_blocked`` — log de bloqueo durante calibración.
* ``_log_settle_snapshot`` — snapshot de poses tras settle.
* ``_request_settle_snapshot`` — pedir un snapshot manual.
* ``wait_for_objects_to_settle`` — wait blocking pública.

Todos delegan al módulo ``panel_state_methods`` (alias ``_stm``).
"""

from __future__ import annotations

from . import panel_state_methods as _stm


class PanelV2TrajSettleMixin:
    """Wrappers thin de trayectoria FJT + objects settle."""

    # ------------------------------------------------------------------
    # Trajectory action client
    # ------------------------------------------------------------------

    def _send_joint_trajectory_action(self, *args, **kwargs):
        return _stm._send_joint_trajectory_action(self, *args, **kwargs)

    def _schedule_traj_action_fallback(self, *args, **kwargs):
        return _stm._schedule_traj_action_fallback(self, *args, **kwargs)

    def _clamp_joint_positions(self, *args, **kwargs):
        return _stm._clamp_joint_positions(self, *args, **kwargs)

    def _log_traj_action_fallback(self, *args, **kwargs):
        return _stm._log_traj_action_fallback(self, *args, **kwargs)

    # ------------------------------------------------------------------
    # Objects settle watcher
    # ------------------------------------------------------------------

    def _start_objects_settle_watch(self, *args, **kwargs):
        return _stm._start_objects_settle_watch(self, *args, **kwargs)

    def _invalidate_settle(self, *args, **kwargs):
        return _stm._invalidate_settle(self, *args, **kwargs)

    def _run_fall_test_async(self, *args, **kwargs):
        return _stm._run_fall_test_async(self, *args, **kwargs)

    def _objects_settle_worker(self, *args, **kwargs):
        return _stm._objects_settle_worker(self, *args, **kwargs)

    def _handle_objects_settled(self, *args, **kwargs):
        return _stm._handle_objects_settled(self, *args, **kwargs)

    def _log_calib_blocked(self, *args, **kwargs):
        return _stm._log_calib_blocked(self, *args, **kwargs)

    def _log_settle_snapshot(self, *args, **kwargs):
        return _stm._log_settle_snapshot(self, *args, **kwargs)

    def _request_settle_snapshot(self, *args, **kwargs):
        return _stm._request_settle_snapshot(self, *args, **kwargs)

    def wait_for_objects_to_settle(self, *args, **kwargs):
        return _stm.wait_for_objects_to_settle(self, *args, **kwargs)
