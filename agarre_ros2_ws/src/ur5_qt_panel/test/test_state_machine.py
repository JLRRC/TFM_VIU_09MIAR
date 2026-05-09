#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_state_machine.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Tests unitarios de la logica de evaluacion de estado del panel."""
from __future__ import annotations

import time

from ur5_qt_panel.panel_external_state import (
    _fresh_external_state_from_worker,
    _worker_external_state_snapshot,
    apply_external_system_state,
    external_state_active,
    resolve_external_state,
)
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
        moveit_required: bool = False,
        moveit_state: MoveItState = MoveItState.READY,
    ):
        self._external_state = ui_state
        self._external_state_reason = ui_reason
        self._external_state_last = time.time() - float(ui_age_sec)
        worker_wall = 0.0
        if worker_state:
            worker_wall = time.monotonic() - float(worker_age_sec)
        self.ros_worker = _RosWorkerStub(worker_state, worker_reason, worker_wall)
        self._moveit_required = moveit_required
        self._moveit_state = moveit_state

    def _moveit_not_ready_reason(self) -> str:
        return "moveit_not_ready"


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


def test_state_machine_active_but_target_none_waits() -> None:
    # active=True, target=None → line 29: return WAITING_GAZEBO
    panel = _PanelStub(active=True, target=None, reason="no_state")
    decision = PanelStateMachine().decide_external(panel)
    assert decision.state == SystemState.WAITING_GAZEBO


def test_state_machine_non_fatal_target_passes_through() -> None:
    # active=True, target=READY (not ERROR_FATAL) → line 32
    panel = _PanelStub(active=True, target=SystemState.READY_VISION, reason="all ok")
    decision = PanelStateMachine().decide_external(panel)
    assert decision.state == SystemState.READY_VISION
    assert decision.reason == "all ok"
    assert decision.fatal_reason is None


# ---------------------------------------------------------------------------
# PanelStateEvaluator.is_ready_level (lines 73-77)
# ---------------------------------------------------------------------------

def test_is_ready_level_basic_includes_ready_basic() -> None:
    assert PanelStateEvaluator.is_ready_level(SystemState.READY_BASIC, "basic") is True
    assert PanelStateEvaluator.is_ready_level(SystemState.READY_MOVEIT, "basic") is True
    assert PanelStateEvaluator.is_ready_level(SystemState.WAITING_GAZEBO, "basic") is False


def test_is_ready_level_vision_excludes_ready_basic() -> None:
    assert PanelStateEvaluator.is_ready_level(SystemState.READY_VISION, "vision") is True
    assert PanelStateEvaluator.is_ready_level(SystemState.READY_BASIC, "vision") is False


def test_is_ready_level_other_only_moveit() -> None:
    assert PanelStateEvaluator.is_ready_level(SystemState.READY_MOVEIT, "other") is True
    assert PanelStateEvaluator.is_ready_level(SystemState.READY_VISION, "other") is False


# ---------------------------------------------------------------------------
# PanelStateEvaluator.resolve — extra branches (lines 94, 99-101, 107)
# ---------------------------------------------------------------------------

def test_evaluator_bridge_not_running() -> None:
    snap = _snapshot(bridge_running=False)
    state, reason = PanelStateEvaluator.resolve(snap)
    assert state == SystemState.READY_BASIC
    assert "Bridge" in reason


def test_evaluator_objects_not_settled() -> None:
    snap = _snapshot(objects_settled=False)
    state, reason = PanelStateEvaluator.resolve(snap)
    assert state == SystemState.READY_BASIC
    assert "estabilizados" in reason


def test_evaluator_calibration_pending() -> None:
    snap = _snapshot(calibration_ready=False)
    state, reason = PanelStateEvaluator.resolve(snap)
    assert state == SystemState.READY_BASIC
    assert "Calibración" in reason


def test_evaluator_ready_vision_no_moveit() -> None:
    # moveit_required=False, all gates pass → READY_VISION (line 107)
    snap = _snapshot(moveit_required=False)
    state, reason = PanelStateEvaluator.resolve(snap)
    assert state == SystemState.READY_VISION
    assert "visión" in reason


# ---------------------------------------------------------------------------
# _worker_external_state_snapshot — all None-return paths (lines 19,22,25-26,29,32-33,35)
# ---------------------------------------------------------------------------

class _NoWorkerPanel:
    pass


class _NoSnapshotWorkerPanel:
    class ros_worker:
        pass  # no system_state_snapshot method


class _RaisingWorkerPanel:
    class ros_worker:
        def system_state_snapshot(self):
            raise RuntimeError("hardware_fault")


class _EmptyStateWorkerPanel:
    class ros_worker:
        def system_state_snapshot(self):
            return "", "", time.monotonic()


class _BadWallWorkerPanel:
    class ros_worker:
        def system_state_snapshot(self):
            return "READY", "ok", "not_a_number"


class _ZeroWallWorkerPanel:
    class ros_worker:
        def system_state_snapshot(self):
            return "READY", "ok", 0.0


def test_worker_snapshot_no_ros_worker_returns_none() -> None:
    assert _worker_external_state_snapshot(_NoWorkerPanel()) is None


def test_worker_snapshot_no_snapshot_method_returns_none() -> None:
    assert _worker_external_state_snapshot(_NoSnapshotWorkerPanel()) is None


def test_worker_snapshot_raises_returns_none() -> None:
    assert _worker_external_state_snapshot(_RaisingWorkerPanel()) is None


def test_worker_snapshot_empty_state_returns_none() -> None:
    assert _worker_external_state_snapshot(_EmptyStateWorkerPanel()) is None


def test_worker_snapshot_bad_wall_returns_none() -> None:
    assert _worker_external_state_snapshot(_BadWallWorkerPanel()) is None


def test_worker_snapshot_zero_wall_returns_none() -> None:
    assert _worker_external_state_snapshot(_ZeroWallWorkerPanel()) is None


# ---------------------------------------------------------------------------
# external_state_active — fresh UI state path (line 51)
# ---------------------------------------------------------------------------

def test_external_state_active_fresh_ui_state_returns_true() -> None:
    # ui_age_sec=0.1 < EXTERNAL_STATE_LIVENESS_SEC=2.0 → line 51: return True
    panel = _ExternalStatePanelStub(
        ui_state="READY",
        ui_reason="fresh",
        ui_age_sec=0.1,
    )
    assert external_state_active(panel) is True


# ---------------------------------------------------------------------------
# resolve_external_state — no worker → uses direct state (lines 61-62, 65)
# ---------------------------------------------------------------------------

def test_resolve_external_state_no_worker_unknown_state_returns_none() -> None:
    # worker stale, ui_state not in EXTERNAL_STATE_MAP → target=None → (None, "")
    panel = _ExternalStatePanelStub(
        ui_state="UNKNOWN_BOGUS_STATE",
        ui_reason="bogus",
        ui_age_sec=0.1,
        worker_state="",  # no worker state
    )
    target, reason = resolve_external_state(panel)
    assert target is None
    assert reason == ""


def test_resolve_external_state_no_worker_uses_direct_state() -> None:
    # worker stale/absent → uses panel._external_state directly
    panel = _ExternalStatePanelStub(
        ui_state="READY",
        ui_reason="direct",
        ui_age_sec=0.5,
        worker_state="",  # no fresh worker state
    )
    target, reason = resolve_external_state(panel)
    assert target == SystemState.READY_VISION


# ---------------------------------------------------------------------------
# _worker_external_state_snapshot — proper-instance stubs (lines 32-33, 35)
# ---------------------------------------------------------------------------

class _BadWallPanelProper:
    """Panel with a worker that returns a non-numeric wall — covers lines 32-33."""
    def __init__(self):
        self.ros_worker = _RosWorkerStub("READY", "ok", "not_a_number")  # type: ignore[arg-type]


class _ZeroWallPanelProper:
    """Panel with a worker that returns wall=0.0 — covers line 35."""
    def __init__(self):
        self.ros_worker = _RosWorkerStub("READY", "ok", 0.0)


def test_worker_snapshot_bad_wall_proper_returns_none() -> None:
    # float("not_a_number") raises → except: return None (lines 32-33)
    assert _worker_external_state_snapshot(_BadWallPanelProper()) is None


def test_worker_snapshot_zero_wall_proper_returns_none() -> None:
    # wall_f = 0.0 ≤ 0.0 → return None (line 35)
    assert _worker_external_state_snapshot(_ZeroWallPanelProper()) is None


# ---------------------------------------------------------------------------
# _fresh_external_state_from_worker — stale snapshot (line 45)
# ---------------------------------------------------------------------------

def test_fresh_external_state_from_worker_stale_returns_none() -> None:
    # worker_age_sec=5.0 > EXTERNAL_STATE_LIVENESS_SEC=2.0 → return None (line 45)
    panel = _ExternalStatePanelStub(
        worker_state="READY",
        worker_reason="stale",
        worker_age_sec=5.0,
    )
    assert _fresh_external_state_from_worker(panel) is None


def test_fresh_external_state_from_worker_fresh_returns_state() -> None:
    panel = _ExternalStatePanelStub(
        worker_state="READY",
        worker_reason="ok",
        worker_age_sec=0.1,
    )
    result = _fresh_external_state_from_worker(panel)
    assert result is not None
    assert result[0] == "READY"


# ---------------------------------------------------------------------------
# resolve_external_state — moveit_required branches (lines 67-70)
# ---------------------------------------------------------------------------

def test_resolve_external_state_moveit_required_not_ready() -> None:
    # target=READY_VISION, moveit_required=True, moveit NOT READY → line 68-69
    panel = _ExternalStatePanelStub(
        ui_state="READY",
        ui_reason="ready",
        ui_age_sec=0.5,
        moveit_required=True,
        moveit_state=MoveItState.STARTING,
    )
    target, reason = resolve_external_state(panel)
    assert target == SystemState.READY_VISION
    assert reason == "moveit_not_ready"


def test_resolve_external_state_moveit_required_and_ready() -> None:
    # target=READY_VISION, moveit_required=True, moveit READY → line 70
    panel = _ExternalStatePanelStub(
        ui_state="READY",
        ui_reason="ready",
        ui_age_sec=0.5,
        moveit_required=True,
        moveit_state=MoveItState.READY,
    )
    target, reason = resolve_external_state(panel)
    assert target == SystemState.READY_MOVEIT


# ---------------------------------------------------------------------------
# apply_external_system_state (lines 75-89)
# ---------------------------------------------------------------------------

class _ApplyStatePanelStub:
    """Minimal panel stub for testing apply_external_system_state."""
    def __init__(self, ui_state: str, moveit_required: bool = False):
        self._external_state = ui_state
        self._external_state_reason = "reason"
        self._moveit_required = moveit_required
        self._moveit_state = MoveItState.READY
        self._system_error_reason = ""
        self.fatal_called = False
        self.state_set = None
        self.pose_sub_ensured = False
        self.pose_info_watch_started = False
        self.tf_ready_timer_started = False

    def _moveit_not_ready_reason(self) -> str:
        return "moveit_not_ready"

    def _trigger_fatal(self, reason: str) -> None:
        self.fatal_called = True

    def _set_system_state(self, state, reason) -> None:
        self.state_set = state

    def _ensure_pose_subscription(self) -> None:
        self.pose_sub_ensured = True

    def _start_pose_info_watch(self) -> None:
        self.pose_info_watch_started = True

    def _start_tf_ready_timer(self) -> None:
        self.tf_ready_timer_started = True


def test_apply_external_system_state_unknown_returns_early() -> None:
    # target=None → return (lines 76-77)
    panel = _ApplyStatePanelStub("UNKNOWN_BOGUS_STATE")
    apply_external_system_state(panel)
    assert panel.state_set is None
    assert not panel.fatal_called


def test_apply_external_system_state_fatal_triggers_fatal() -> None:
    # target=ERROR_FATAL → _trigger_fatal (lines 78-80)
    panel = _ApplyStatePanelStub("ERROR_FATAL")
    apply_external_system_state(panel)
    assert panel.fatal_called
    assert panel.state_set is None


def test_apply_external_system_state_error_sets_reason() -> None:
    # target=ERROR → sets _system_error_reason (lines 81-82, 85)
    panel = _ApplyStatePanelStub("ERROR")
    apply_external_system_state(panel)
    assert panel.state_set == SystemState.ERROR
    assert not panel.fatal_called


def test_apply_external_system_state_ready_vision_starts_subscriptions() -> None:
    # target=READY_VISION → clears error reason + starts subscriptions (lines 84-89)
    panel = _ApplyStatePanelStub("READY")
    panel._system_error_reason = "old_error"
    apply_external_system_state(panel)
    assert panel.state_set == SystemState.READY_VISION
    assert panel._system_error_reason == ""
    assert panel.pose_sub_ensured
    assert panel.pose_info_watch_started
    assert panel.tf_ready_timer_started


def test_apply_external_system_state_waiting_gazebo_no_subscriptions() -> None:
    # target=WAITING_GAZEBO → _set_system_state but no subscription calls (line 85, not 87-89)
    panel = _ApplyStatePanelStub("WAITING_GAZEBO")
    apply_external_system_state(panel)
    assert panel.state_set == SystemState.WAITING_GAZEBO
    assert not panel.pose_sub_ensured
    assert not panel.pose_info_watch_started
    assert not panel.tf_ready_timer_started
