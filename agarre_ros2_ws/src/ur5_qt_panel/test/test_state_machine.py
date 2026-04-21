#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_state_machine.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Tests unitarios de la logica de evaluacion de estado del panel."""
from __future__ import annotations

import time

from ur5_qt_panel.panel_external_state import external_state_active, resolve_external_state
from ur5_qt_panel.panel_state import (
    MoveItState,
    PanelStateEvaluator,
    PanelStateSnapshot,
    SystemState,
)
from ur5_qt_panel.panel_state_machine import PanelStateMachine


class _PanelStub:
    def __init__(self, active: bool, target, reason: str):
        self._active = active
        self._target = target
        self._reason = reason

    def _external_state_active(self) -> bool:
        return self._active

    def _resolve_external_state(self):
        return self._target, self._reason


class _RosWorkerStub:
    def __init__(self, state: str = "", reason: str = "", wall: float = 0.0):
        self._state = state
        self._reason = reason
        self._wall = wall

    def system_state_snapshot(self):
        return self._state, self._reason, self._wall


class _ExternalStatePanelStub:
    def __init__(
        self,
        *,
        ui_state: str = "",
        ui_reason: str = "",
        ui_age_sec: float = 0.0,
        worker_state: str = "",
        worker_reason: str = "",
        worker_age_sec: float = 0.0,
    ):
        self._external_state = ui_state
        self._external_state_reason = ui_reason
        self._external_state_last = time.time() - float(ui_age_sec)
        worker_wall = 0.0
        if worker_state:
            worker_wall = time.monotonic() - float(worker_age_sec)
        self.ros_worker = _RosWorkerStub(worker_state, worker_reason, worker_wall)
        self._moveit_required = False
        self._moveit_state = MoveItState.READY


def _snapshot(**overrides) -> PanelStateSnapshot:
    base = PanelStateSnapshot(
        gazebo_state="GAZEBO_READY",
        controllers_ok=True,
        controllers_reason="",
        tf_ready=True,
        bridge_running=True,
        camera_required=False,
        camera_stream_ok=True,
        pose_info_ok=True,
        calibration_ready=True,
        objects_settled=True,
        moveit_required=False,
        moveit_state=MoveItState.READY,
        moveit_state_reason="",
    )
    return base.__class__(**{**base.__dict__, **overrides})


def test_state_machine_waits_for_external() -> None:
    panel = _PanelStub(active=False, target=None, reason="")
    decision = PanelStateMachine().decide_external(panel)
    assert decision.state == SystemState.WAITING_GAZEBO
    assert "system_state" in decision.reason


def test_state_machine_fatal_passthrough() -> None:
    panel = _PanelStub(active=True, target=SystemState.ERROR_FATAL, reason="fatal test")
    decision = PanelStateMachine().decide_external(panel)
    assert decision.state == SystemState.ERROR_FATAL
    assert decision.fatal_reason == "fatal test"


def test_external_state_active_uses_ros_worker_when_ui_stale() -> None:
    panel = _ExternalStatePanelStub(
        ui_state="READY",
        ui_reason="ui stale",
        ui_age_sec=8.0,
        worker_state="READY",
        worker_reason="Sistema listo",
        worker_age_sec=0.2,
    )
    assert external_state_active(panel) is True


def test_resolve_external_state_prefers_fresh_ros_worker_state() -> None:
    panel = _ExternalStatePanelStub(
        ui_state="WAITING_GAZEBO",
        ui_reason="ui stale",
        ui_age_sec=9.0,
        worker_state="READY",
        worker_reason="Sistema listo",
        worker_age_sec=0.1,
    )
    target, reason = resolve_external_state(panel)
    assert target == SystemState.READY_VISION
    assert reason == "Sistema listo"


def test_evaluator_waits_for_gazebo() -> None:
    snap = _snapshot(gazebo_state="GAZEBO_STARTING")
    state, reason = PanelStateEvaluator.resolve(snap)
    assert state == SystemState.WAITING_GAZEBO
    assert "Gazebo" in reason


def test_evaluator_waits_for_controllers() -> None:
    snap = _snapshot(controllers_ok=False, controllers_reason="no controllers")
    state, reason = PanelStateEvaluator.resolve(snap)
    assert state == SystemState.WAITING_CONTROLLERS
    assert "no controllers" in reason


def test_evaluator_tf_gate() -> None:
    snap = _snapshot(tf_ready=False)
    state, reason = PanelStateEvaluator.resolve(snap, tf_reason="TF missing")
    assert state == SystemState.READY_BASIC
    assert reason == "TF missing"


def test_evaluator_camera_gate() -> None:
    snap = _snapshot(camera_required=True, camera_stream_ok=False)
    state, reason = PanelStateEvaluator.resolve(snap)
    assert state == SystemState.READY_BASIC
    assert "Cámara" in reason


def test_evaluator_pose_gate() -> None:
    snap = _snapshot(pose_info_ok=False)
    state, reason = PanelStateEvaluator.resolve(snap)
    assert state == SystemState.READY_BASIC
    assert "pose/info" in reason


def test_evaluator_moveit_gate() -> None:
    snap = _snapshot(moveit_required=True, moveit_state=MoveItState.STARTING, moveit_state_reason="wait")
    state, reason = PanelStateEvaluator.resolve(snap)
    assert state == SystemState.READY_VISION
    assert reason == "wait"


def test_evaluator_ready_moveit() -> None:
    snap = _snapshot(moveit_required=True, moveit_state=MoveItState.READY)
    state, reason = PanelStateEvaluator.resolve(snap)
    assert state == SystemState.READY_MOVEIT
    assert "MoveIt" in reason
