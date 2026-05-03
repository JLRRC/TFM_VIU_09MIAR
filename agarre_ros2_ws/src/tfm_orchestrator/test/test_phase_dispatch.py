"""F5-step1: tests unitarios del dispatch puro de fases.

El módulo ``phase_dispatch`` extrae la lógica común a Node legacy y
LifecycleNode canónico. Estos tests inyectan callers mock vía
``PhaseDispatchContext.service_caller`` / ``action_caller`` para
verificar el contrato fase → service/action sin levantar ROS.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

import pytest

# Estos imports SÍ requieren rclpy en runtime para resolver mensajes.
# Si no estuviera disponible (entorno offline), los tests se skip.
rclpy_available = True
try:
    from ur5_panel_interfaces.action import PickPlace, PlanToPose  # noqa: F401
    from ur5_panel_interfaces.srv import (  # noqa: F401
        Attach as AttachSrv,
        Close as CloseSrv,
        Detach as DetachSrv,
        Open as OpenSrv,
        SelectObject as SelectObjectSrv,
    )
except Exception:
    rclpy_available = False

pytestmark = pytest.mark.skipif(
    not rclpy_available,
    reason="ur5_panel_interfaces / rclpy no disponibles en este entorno",
)

from tfm_orchestrator.phase_dispatch import (  # noqa: E402
    PhaseDispatchContext,
    build_plan_to_pose_goal_for_approach,
    build_plan_to_pose_goal_for_lift,
    build_plan_to_pose_goal_for_transport,
    dispatch_phase,
)
from tfm_orchestrator.pick_fsm import PickContext, PickPhase  # noqa: E402
from tfm_orchestrator.service_clients import (  # noqa: E402
    ActionCallResult,
    PhaseServiceMap,
    ServiceCallResult,
)


# ---------------------------------------------------------------------------
# Helpers de tests
# ---------------------------------------------------------------------------


@dataclass
class _RecordedCall:
    kind: str  # "service" | "action"
    msg_type_name: str
    name: str
    request: Any


@dataclass
class _MockCallerSpec:
    """Programa qué resultado devolver para cada call y graba lo recibido."""

    service_results: Dict[str, ServiceCallResult] = field(default_factory=dict)
    action_results: Dict[str, ActionCallResult] = field(default_factory=dict)
    recorded: List[_RecordedCall] = field(default_factory=list)

    def make_service_caller(self):
        def _caller(node, srv_type, name, request, **_kwargs):
            self.recorded.append(
                _RecordedCall("service", srv_type.__name__, name, request)
            )
            return self.service_results.get(
                name, ServiceCallResult(success=True, reason="mock_default_ok")
            )
        return _caller

    def make_action_caller(self):
        def _caller(node, action_type, name, goal, **_kwargs):
            self.recorded.append(
                _RecordedCall("action", action_type.__name__, name, goal)
            )
            return self.action_results.get(
                name, ActionCallResult(success=True, reason="mock_default_ok")
            )
        return _caller


def _ctx_for(spec: _MockCallerSpec, **overrides) -> PhaseDispatchContext:
    return PhaseDispatchContext(
        node=object(),
        service_map=PhaseServiceMap(),
        client_cache={},
        service_caller=spec.make_service_caller(),
        action_caller=spec.make_action_caller(),
        **overrides,
    )


def _pick_ctx(name: str = "box_red", drop=(0.5, 0.0, 0.05)) -> PickContext:
    return PickContext(object_name=name, drop_xyz_world=drop)


# ---------------------------------------------------------------------------
# SELECT_OBJECT
# ---------------------------------------------------------------------------


def test_select_object_calls_select_object_service_with_object_name():
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.SELECT_OBJECT, _pick_ctx("widget"))
    assert ok is True
    assert reason.startswith("select_object:")
    assert len(spec.recorded) == 1
    rec = spec.recorded[0]
    assert rec.kind == "service"
    assert rec.msg_type_name == "SelectObject"
    assert rec.name == "/panel/select_object"
    assert rec.request.name == "widget"


def test_select_object_propagates_failure():
    spec = _MockCallerSpec(
        service_results={
            "/panel/select_object": ServiceCallResult(
                success=False, reason="object_not_found"
            )
        }
    )
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.SELECT_OBJECT, _pick_ctx("missing"))
    assert ok is False
    assert "object_not_found" in reason


# ---------------------------------------------------------------------------
# APPROACH (PlanToPose action)
# ---------------------------------------------------------------------------


def test_approach_calls_plan_to_pose_action():
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.APPROACH, _pick_ctx())
    assert ok is True
    assert reason.startswith("approach:")
    assert len(spec.recorded) == 1
    rec = spec.recorded[0]
    assert rec.kind == "action"
    assert rec.msg_type_name == "PlanToPose"
    assert rec.name == "/orchestrator/plan_to_pose"
    # Goal del approach: pose neutra + ee_frame correcto + no cartesian.
    assert rec.request.ee_frame == "rg2_pinch_center"
    assert rec.request.cartesian is False


def test_approach_failure_propagates():
    spec = _MockCallerSpec(
        action_results={
            "/orchestrator/plan_to_pose": ActionCallResult(
                success=False, reason="planner_failed"
            )
        }
    )
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.APPROACH, _pick_ctx())
    assert ok is False
    assert "planner_failed" in reason


# ---------------------------------------------------------------------------
# GRASP (Close + Attach)
# ---------------------------------------------------------------------------


def test_grasp_calls_close_then_attach_in_order():
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.GRASP, _pick_ctx("cube"))
    assert ok is True
    assert reason.startswith("grasp_attach:")
    assert len(spec.recorded) == 2
    # Orden: Close primero, Attach después.
    assert spec.recorded[0].msg_type_name == "Close"
    assert spec.recorded[1].msg_type_name == "Attach"
    assert spec.recorded[1].request.object_name == "cube"


def test_grasp_aborts_on_close_failure_without_calling_attach():
    spec = _MockCallerSpec(
        service_results={
            "/gripper/close": ServiceCallResult(success=False, reason="motor_stall")
        }
    )
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.GRASP, _pick_ctx())
    assert ok is False
    assert "motor_stall" in reason
    # Solo se llamó a Close — Attach NO se intentó.
    assert len(spec.recorded) == 1
    assert spec.recorded[0].msg_type_name == "Close"


def test_grasp_propagates_attach_failure():
    spec = _MockCallerSpec(
        service_results={
            "/orchestrator/attach": ServiceCallResult(
                success=False, reason="object_too_far"
            )
        }
    )
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.GRASP, _pick_ctx())
    assert ok is False
    assert "object_too_far" in reason


# ---------------------------------------------------------------------------
# LIFT
# ---------------------------------------------------------------------------


def test_lift_calls_plan_to_pose_with_cartesian_true():
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.LIFT, _pick_ctx())
    assert ok is True
    assert reason.startswith("lift:")
    assert spec.recorded[0].msg_type_name == "PlanToPose"
    # LIFT usa cartesian=True (movimiento lineal vertical).
    assert spec.recorded[0].request.cartesian is True


# ---------------------------------------------------------------------------
# TRANSPORT
# ---------------------------------------------------------------------------


def test_transport_uses_drop_xyz_from_pick_context():
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(
        dctx, PickPhase.TRANSPORT, _pick_ctx(drop=(0.7, -0.2, 0.10))
    )
    assert ok is True
    assert reason.startswith("transport:")
    pose = spec.recorded[0].request.target_pose_base
    assert pose.position.x == pytest.approx(0.7)
    assert pose.position.y == pytest.approx(-0.2)
    assert pose.position.z == pytest.approx(0.10)


# ---------------------------------------------------------------------------
# RELEASE (Detach + Open)
# ---------------------------------------------------------------------------


def test_release_calls_detach_then_open_in_order():
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.RELEASE, _pick_ctx("cylinder"))
    assert ok is True
    assert reason.startswith("release_open:")
    assert len(spec.recorded) == 2
    assert spec.recorded[0].msg_type_name == "Detach"
    assert spec.recorded[0].request.object_name == "cylinder"
    assert spec.recorded[1].msg_type_name == "Open"


def test_release_aborts_on_detach_failure_without_calling_open():
    spec = _MockCallerSpec(
        service_results={
            "/orchestrator/detach": ServiceCallResult(
                success=False, reason="not_attached"
            )
        }
    )
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.RELEASE, _pick_ctx())
    assert ok is False
    assert "not_attached" in reason
    assert len(spec.recorded) == 1
    assert spec.recorded[0].msg_type_name == "Detach"


def test_release_propagates_open_failure():
    spec = _MockCallerSpec(
        service_results={
            "/gripper/open": ServiceCallResult(success=False, reason="open_timeout")
        }
    )
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.RELEASE, _pick_ctx())
    assert ok is False
    assert "open_timeout" in reason


# ---------------------------------------------------------------------------
# Goal builders puros
# ---------------------------------------------------------------------------


def test_build_plan_to_pose_goal_for_approach_returns_neutral_pose():
    goal = build_plan_to_pose_goal_for_approach(_pick_ctx())
    assert goal.target_pose_base.position.x == 0.0
    assert goal.target_pose_base.position.y == 0.0
    assert goal.target_pose_base.position.z == 0.0
    assert goal.ee_frame == "rg2_pinch_center"
    assert goal.cartesian is False


def test_build_plan_to_pose_goal_for_lift_uses_z_offset():
    goal = build_plan_to_pose_goal_for_lift(_pick_ctx())
    assert goal.target_pose_base.position.z == pytest.approx(0.20)
    assert goal.cartesian is True


def test_build_plan_to_pose_goal_for_transport_uses_drop_xyz():
    ctx = _pick_ctx(drop=(0.3, 0.4, 0.15))
    goal = build_plan_to_pose_goal_for_transport(ctx)
    assert goal.target_pose_base.position.x == pytest.approx(0.3)
    assert goal.target_pose_base.position.y == pytest.approx(0.4)
    assert goal.target_pose_base.position.z == pytest.approx(0.15)


# ---------------------------------------------------------------------------
# Fases sin dispatch (no-op)
# ---------------------------------------------------------------------------


def test_initial_snapshot_is_noop():
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.INITIAL_SNAPSHOT, _pick_ctx())
    assert ok is True
    assert "no_op" in reason
    assert spec.recorded == []


def test_home_initial_is_noop():
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.HOME_INITIAL, _pick_ctx())
    assert ok is True
    assert "no_op" in reason
    assert spec.recorded == []
