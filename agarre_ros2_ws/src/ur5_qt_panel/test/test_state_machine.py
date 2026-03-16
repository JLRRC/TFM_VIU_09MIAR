#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_state_machine.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Unit tests for panel state evaluation logic."""
from __future__ import annotations

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
