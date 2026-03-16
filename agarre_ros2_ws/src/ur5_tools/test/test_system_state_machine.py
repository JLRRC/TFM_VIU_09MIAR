#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_system_state_machine.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
"""Unit tests for SystemStateMachine decision logic."""

import pytest

rclpy = pytest.importorskip("rclpy")

from ur5_tools.system_state_manager import (  # noqa: E402
    DependencySnapshot,
    SystemInputs,
    SystemState,
    SystemStateMachine,
)


def _snapshot(
    *,
    clock_ok: bool = True,
    pose_ok: bool = True,
    camera_ok: bool = True,
    tf_ok: bool = True,
    controllers_ready: bool = True,
    missing_controllers=None,
    moveit_ready: bool = True,
):
    if missing_controllers is None:
        missing_controllers = (
            [] if controllers_ready else ["joint_trajectory_controller"]
        )
    return DependencySnapshot(
        clock_ok=clock_ok,
        clock_age=0.0 if clock_ok else float("inf"),
        pose_ok=pose_ok,
        pose_age=0.0 if pose_ok else float("inf"),
        pose_model_seen=pose_ok,
        pose_entities_seen=pose_ok,
        camera_ok=camera_ok,
        camera_age=0.0 if camera_ok else float("inf"),
        tf_ok=tf_ok,
        tf_reason="ok" if tf_ok else "missing",
        controllers_ready=controllers_ready,
        missing_controllers=missing_controllers,
        moveit_ready=moveit_ready,
    )


def test_decide_waiting_gazebo_when_clock_missing():
    fsm = SystemStateMachine(required_controllers=[], moveit_required=False)
    snap = _snapshot(clock_ok=False)
    decision = fsm.decide(
        SystemInputs(snapshot=snap, startup_expired=False, ever_ready=False)
    )
    assert decision.state == SystemState.WAITING_GAZEBO


def test_decide_waiting_bridge_when_pose_missing():
    fsm = SystemStateMachine(required_controllers=[], moveit_required=False)
    snap = _snapshot(pose_ok=False)
    decision = fsm.decide(
        SystemInputs(snapshot=snap, startup_expired=False, ever_ready=False)
    )
    assert decision.state == SystemState.WAITING_BRIDGE


def test_decide_waiting_controllers_when_missing():
    fsm = SystemStateMachine(
        required_controllers=["joint_trajectory_controller"], moveit_required=False
    )
    snap = _snapshot(
        controllers_ready=False,
        missing_controllers=["joint_trajectory_controller"],
    )
    decision = fsm.decide(
        SystemInputs(snapshot=snap, startup_expired=False, ever_ready=False)
    )
    assert decision.state == SystemState.WAITING_CONTROLLERS


def test_decide_waiting_tf_when_tf_missing():
    fsm = SystemStateMachine(required_controllers=[], moveit_required=False)
    snap = _snapshot(tf_ok=False)
    decision = fsm.decide(
        SystemInputs(snapshot=snap, startup_expired=False, ever_ready=False)
    )
    assert decision.state == SystemState.WAITING_TF


def test_decide_waiting_camera_when_camera_missing():
    fsm = SystemStateMachine(required_controllers=[], moveit_required=False)
    snap = _snapshot(camera_ok=False)
    decision = fsm.decide(
        SystemInputs(snapshot=snap, startup_expired=False, ever_ready=False)
    )
    assert decision.state == SystemState.WAITING_CAMERA


def test_decide_waiting_moveit_when_required():
    fsm = SystemStateMachine(required_controllers=[], moveit_required=True)
    snap = _snapshot(moveit_ready=False)
    decision = fsm.decide(
        SystemInputs(snapshot=snap, startup_expired=False, ever_ready=False)
    )
    assert decision.state == SystemState.WAITING_MOVEIT


def test_decide_ready_when_all_ok():
    fsm = SystemStateMachine(required_controllers=[], moveit_required=False)
    snap = _snapshot()
    decision = fsm.decide(
        SystemInputs(snapshot=snap, startup_expired=False, ever_ready=False)
    )
    assert decision.state == SystemState.READY


def test_decide_fatal_reason_when_startup_expired_and_critical_missing():
    fsm = SystemStateMachine(required_controllers=[], moveit_required=False)
    snap = _snapshot(clock_ok=False)
    decision = fsm.decide(
        SystemInputs(snapshot=snap, startup_expired=True, ever_ready=False)
    )
    assert decision.fatal_reason is not None


def test_drop_after_ready_sets_error_state():
    fsm = SystemStateMachine(required_controllers=[], moveit_required=False)
    snap = _snapshot(camera_ok=False)
    decision = fsm.decide(
        SystemInputs(snapshot=snap, startup_expired=False, ever_ready=True)
    )
    assert decision.state == SystemState.WAITING_CAMERA
    assert decision.fatal_reason is not None


def test_drop_after_ready_critical_missing_sets_fatal_reason():
    fsm = SystemStateMachine(required_controllers=[], moveit_required=False)
    snap = _snapshot(clock_ok=False)
    decision = fsm.decide(
        SystemInputs(snapshot=snap, startup_expired=False, ever_ready=True)
    )
    assert decision.state == SystemState.WAITING_GAZEBO
    assert decision.fatal_reason is not None
