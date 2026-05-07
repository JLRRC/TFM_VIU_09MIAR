"""F5-step1: tests unitarios del dispatch puro de fases.

El módulo ``phase_dispatch`` extrae la lógica común a Node legacy y
LifecycleNode canónico. Estos tests inyectan callers mock vía
``PhaseDispatchContext.service_caller`` / ``action_caller`` para
verificar el contrato fase → service/action sin levantar ROS.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List

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
    pose_msg_to_tuple7,
    try_resolve_object_pose_world,
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


def test_select_object_is_internal_only_no_service_call():
    """B-iter1 (2026-05-03): SELECT_OBJECT NO llama al panel.

    El object_name ya viaja en el goal de PickPlace; las fases siguientes
    (APPROACH/GRASP/etc) consumen ctx.object_name directamente. Eliminada
    la llamada a /panel/select_object que creaba dependencia circular
    orchestrator↔panel.
    """
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.SELECT_OBJECT, _pick_ctx("widget"))
    assert ok is True
    assert reason == "select_object:internal_ok:object=widget"
    assert len(spec.recorded) == 0, (
        "SELECT_OBJECT no debe llamar a ningún service externo "
        "(B-iter1: orchestrator independiente del panel)"
    )


def test_select_object_rejects_empty_object_name():
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.SELECT_OBJECT, _pick_ctx(""))
    assert ok is False
    assert reason == "select_object:empty_object_name_in_goal"
    assert len(spec.recorded) == 0


def test_select_object_strips_whitespace_object_name():
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.SELECT_OBJECT, _pick_ctx("  widget  "))
    assert ok is True
    assert reason == "select_object:internal_ok:object=widget"


# ---------------------------------------------------------------------------
# INITIAL_SNAPSHOT / HOME_INITIAL (B-iter2: no-op explícitos)
# ---------------------------------------------------------------------------


def test_initial_snapshot_is_explicit_noop_with_object_name():
    """B-iter2: INITIAL_SNAPSHOT no llama services; emite marker semántico."""
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.INITIAL_SNAPSHOT, _pick_ctx("widget"))
    assert ok is True
    assert reason == "initial_snapshot:scaffold_ok:object=widget"
    assert len(spec.recorded) == 0


def test_initial_snapshot_handles_empty_object_name():
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.INITIAL_SNAPSHOT, _pick_ctx(""))
    assert ok is True
    assert reason == "initial_snapshot:scaffold_ok:object=none"
    assert len(spec.recorded) == 0


def test_home_initial_is_explicit_noop():
    """B-iter2: HOME_INITIAL no llama services; emite marker semántico."""
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.HOME_INITIAL, _pick_ctx())
    assert ok is True
    assert reason == "home_initial:scaffold_ok"
    assert len(spec.recorded) == 0


# ---------------------------------------------------------------------------
# APPROACH (PlanToPose action)
# ---------------------------------------------------------------------------


def test_approach_calls_plan_to_pose_action():
    """F5-step3: APPROACH sin hint dispara resolver + plan_to_pose.
    El mock por defecto devuelve payload=None ⇒ resolver retorna None ⇒
    plan_to_pose se llama con placeholder (pose neutra)."""
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.APPROACH, _pick_ctx())
    assert ok is True
    assert reason.startswith("approach:")
    # 2 calls: resolver (service) + plan_to_pose (action).
    assert len(spec.recorded) == 2
    assert spec.recorded[0].kind == "service"
    assert spec.recorded[0].msg_type_name == "ResolveObjectPoseWorld"
    rec = spec.recorded[1]
    assert rec.kind == "action"
    assert rec.msg_type_name == "PlanToPose"
    assert rec.name == "/orchestrator/plan_to_pose"
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


def _attach_payload_close(tcp_obj_dist_m: float = 0.020):
    """B-iter8: helper para construir un Attach response payload con
    tcp_obj_dist_m dentro de tolerancia para que el gate pase."""
    from types import SimpleNamespace
    return SimpleNamespace(
        success=True,
        message="attached",
        method="drop_anchor",
        tcp_obj_dist_m=float(tcp_obj_dist_m),
    )


def test_grasp_calls_close_then_attach_in_order():
    spec = _MockCallerSpec(
        service_results={
            "/orchestrator/attach": ServiceCallResult(
                success=True, reason="attached",
                payload=_attach_payload_close(0.020),
            )
        }
    )
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


def test_grasp_fails_when_attach_distance_gate_too_far():
    """B-iter8: el gate detecta el bug 'drop_anchor placebo' donde el backend
    retorna success=True pero TCP a >5cm del objeto."""
    spec = _MockCallerSpec(
        service_results={
            "/orchestrator/attach": ServiceCallResult(
                success=True, reason="attached",
                payload=_attach_payload_close(1.093),  # del log live
            )
        }
    )
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.GRASP, _pick_ctx())
    assert ok is False
    assert "grasp_attach_gate" in reason
    assert "too_far" in reason


def test_grasp_fails_when_attach_distance_unmeasured():
    """B-iter8: si el payload no expone tcp_obj_dist_m, el gate falla seguro."""
    spec = _MockCallerSpec()  # default payload=None
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.GRASP, _pick_ctx())
    assert ok is False
    assert "unmeasured" in reason


# ---------------------------------------------------------------------------
# LIFT
# ---------------------------------------------------------------------------


# 2026-05-07: tests adaptados al fix world→base_link en phase_dispatch.py.
# Offset estático del UR5 (urdf:47): base_link en world está en (-0.85, 0, 0.850).
# Por tanto: pos_base = pos_world - (-0.85, 0, 0.850).
_BASE_LINK_IN_WORLD = (-0.85, 0.0, 0.850)


def _world_to_base(x_world, y_world, z_world):
    return (
        x_world - _BASE_LINK_IN_WORLD[0],
        y_world - _BASE_LINK_IN_WORLD[1],
        z_world - _BASE_LINK_IN_WORLD[2],
    )


def test_lift_calls_plan_to_pose_with_cartesian_false_20260507():
    """2026-05-07: LIFT cambió a cartesian=False (joint planning más
    robusto que cartesian para el LIFT con peso del objeto)."""
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.LIFT, _pick_ctx())
    assert ok is True
    assert reason.startswith("lift:")
    assert spec.recorded[0].msg_type_name == "PlanToPose"
    assert spec.recorded[0].request.cartesian is False


# ---------------------------------------------------------------------------
# TRANSPORT
# ---------------------------------------------------------------------------


def test_transport_uses_drop_xyz_from_pick_context():
    """2026-05-07: drop_xyz_world se convierte a base_link antes de poner
    en target_pose_base (el server lee target_pose_base como base_link)."""
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    drop_world = (0.7, -0.2, 0.10)
    ok, reason = dispatch_phase(
        dctx, PickPhase.TRANSPORT, _pick_ctx(drop=drop_world)
    )
    assert ok is True
    assert reason.startswith("transport:")
    pose = spec.recorded[0].request.target_pose_base
    expected = _world_to_base(*drop_world)
    assert pose.position.x == pytest.approx(expected[0])
    assert pose.position.y == pytest.approx(expected[1])
    assert pose.position.z == pytest.approx(expected[2])


def test_transport_retries_on_first_failure_then_succeeds(monkeypatch):
    """B-iter9: TRANSPORT reintenta el plan_to_pose ante fallo transitorio."""
    # No esperar back-off real en tests.
    import tfm_orchestrator.retry as retry_mod
    sleeps = []
    monkeypatch.setattr(retry_mod.time, "sleep", sleeps.append)

    # Mock que falla 1ª llamada y ok 2ª.
    calls = {"n": 0}

    def _action_caller(node, action_type, name, goal, **_kwargs):
        calls["n"] += 1
        if calls["n"] == 1:
            return ActionCallResult(success=False, reason="planning_transient")
        return ActionCallResult(success=True, reason="ok")

    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    # Override action_caller (bypass spec).
    dctx.action_caller = _action_caller

    ok, reason = dispatch_phase(dctx, PickPhase.TRANSPORT, _pick_ctx())
    assert ok is True
    assert reason == "transport:ok|attempts=2"
    assert calls["n"] == 2
    assert sleeps == [1.0]  # back-off antes del 2º intento


def test_transport_fails_after_exhausting_retries(monkeypatch):
    """B-iter9: TRANSPORT falla con detalle si todos los intentos abortan."""
    import tfm_orchestrator.retry as retry_mod
    sleeps = []
    monkeypatch.setattr(retry_mod.time, "sleep", sleeps.append)

    def _action_caller(node, action_type, name, goal, **_kwargs):
        return ActionCallResult(success=False, reason="moveit_err:PLANNING_FAILED")

    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    dctx.action_caller = _action_caller

    ok, reason = dispatch_phase(dctx, PickPhase.TRANSPORT, _pick_ctx())
    assert ok is False
    assert "moveit_err:PLANNING_FAILED" in reason
    assert "attempts=2" in reason
    # Default max_attempts=2 → 1 back-off entre intentos.
    assert len(sleeps) == 1


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


def test_build_plan_to_pose_goal_for_approach_returns_neutral_pose_when_no_hint():
    goal = build_plan_to_pose_goal_for_approach(_pick_ctx())
    assert goal.target_pose_base.position.x == 0.0
    assert goal.target_pose_base.position.y == 0.0
    assert goal.target_pose_base.position.z == 0.0
    assert goal.ee_frame == "rg2_pinch_center"
    assert goal.cartesian is False


def test_build_plan_to_pose_goal_for_lift_no_hint_uses_placeholder_20260507():
    """2026-05-07: sin object_pose_world_hint, LIFT cae a placeholder
    (0.40, 0, 0.40) en base_link y cartesian=False."""
    goal = build_plan_to_pose_goal_for_lift(_pick_ctx())
    assert goal.target_pose_base.position.x == pytest.approx(0.40)
    assert goal.target_pose_base.position.y == pytest.approx(0.0)
    assert goal.target_pose_base.position.z == pytest.approx(0.40)
    assert goal.cartesian is False


def test_build_plan_to_pose_goal_for_lift_with_hint_uses_world_to_base_20260507():
    """2026-05-07: con hint, LIFT usa pose objeto + lift_z 0.30, convertido
    world→base_link."""
    hint_world = (0.4, -0.1, 0.03)
    ctx = PickContext(
        object_name="box",
        drop_xyz_world=(0.5, 0, 0.05),
        object_pose_world_hint=hint_world,
    )
    goal = build_plan_to_pose_goal_for_lift(ctx)
    expected = _world_to_base(hint_world[0], hint_world[1], hint_world[2] + 0.30)
    assert goal.target_pose_base.position.x == pytest.approx(expected[0])
    assert goal.target_pose_base.position.y == pytest.approx(expected[1])
    assert goal.target_pose_base.position.z == pytest.approx(expected[2])


def test_build_plan_to_pose_goal_for_transport_uses_drop_xyz_world_to_base_20260507():
    """2026-05-07: drop_xyz_world se convierte a base_link en el goal."""
    drop_world = (0.3, 0.4, 0.15)
    ctx = _pick_ctx(drop=drop_world)
    goal = build_plan_to_pose_goal_for_transport(ctx)
    expected = _world_to_base(*drop_world)
    assert goal.target_pose_base.position.x == pytest.approx(expected[0])
    assert goal.target_pose_base.position.y == pytest.approx(expected[1])
    assert goal.target_pose_base.position.z == pytest.approx(expected[2])


# ---------------------------------------------------------------------------
# Fases sin dispatch (no-op)
# ---------------------------------------------------------------------------


def test_initial_snapshot_is_noop():
    """Histórico: F5-step1 garantizaba "no_op". B-iter2 (2026-05-03) cambió a
    marker semántico explícito; el contrato esencial sigue: ok=True + no service calls."""
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.INITIAL_SNAPSHOT, _pick_ctx())
    assert ok is True
    assert reason.startswith("initial_snapshot:")
    assert spec.recorded == []


def test_home_initial_is_noop():
    """Histórico: ver comentario en test_initial_snapshot_is_noop."""
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ok, reason = dispatch_phase(dctx, PickPhase.HOME_INITIAL, _pick_ctx())
    assert ok is True
    assert reason.startswith("home_initial:")
    assert spec.recorded == []


# ---------------------------------------------------------------------------
# F5-step2: object_pose_world_hint propagado al goal de APPROACH
# ---------------------------------------------------------------------------


def _pick_ctx_with_hint(
    name: str = "box_red",
    drop=(0.5, 0.0, 0.05),
    hint=(0.4, -0.1, 0.03),
) -> PickContext:
    return PickContext(
        object_name=name,
        drop_xyz_world=drop,
        object_pose_world_hint=hint,
    )


def test_approach_uses_object_hint_with_default_z_clearance_world_to_base_20260507():
    """2026-05-07: APPROACH convierte hint world → base_link y añade
    clearance Z (0.10 default)."""
    hint_world = (0.4, -0.1, 0.03)
    ctx = _pick_ctx_with_hint(hint=hint_world)
    goal = build_plan_to_pose_goal_for_approach(ctx)
    pose = goal.target_pose_base
    expected = _world_to_base(hint_world[0], hint_world[1], hint_world[2] + 0.10)
    assert pose.position.x == pytest.approx(expected[0])
    assert pose.position.y == pytest.approx(expected[1])
    assert pose.position.z == pytest.approx(expected[2])
    # Quat identity (hint era tuple3 → completed con identity).
    assert pose.orientation.w == pytest.approx(1.0)


def test_approach_respects_custom_z_clearance_world_to_base_20260507():
    """2026-05-07: ver test_approach_uses_object_hint."""
    hint_world = (0.5, 0.2, 0.10)
    ctx = _pick_ctx_with_hint(hint=hint_world)
    goal = build_plan_to_pose_goal_for_approach(ctx, z_clearance_m=0.05)
    expected_z = hint_world[2] + 0.05 - _BASE_LINK_IN_WORLD[2]
    assert goal.target_pose_base.position.z == pytest.approx(expected_z)


def test_approach_uses_hint_quat_when_tuple7():
    # Quat 90° around Z: (0, 0, sin(45°), cos(45°)) ≈ (0, 0, 0.707, 0.707).
    ctx = PickContext(
        object_name="box",
        drop_xyz_world=(0.0, 0.0, 0.0),
        object_pose_world_hint=(0.3, 0.4, 0.05, 0.0, 0.0, 0.7071, 0.7071),
    )
    goal = build_plan_to_pose_goal_for_approach(ctx)
    assert goal.target_pose_base.orientation.x == pytest.approx(0.0)
    assert goal.target_pose_base.orientation.y == pytest.approx(0.0)
    assert goal.target_pose_base.orientation.z == pytest.approx(0.7071)
    assert goal.target_pose_base.orientation.w == pytest.approx(0.7071)


def test_approach_falls_back_to_neutral_when_hint_is_zero_pose():
    """Convención "no hint": pos zero + quat identity → placeholder."""
    ctx = PickContext(
        object_name="box",
        drop_xyz_world=(0.0, 0.0, 0.0),
        object_pose_world_hint=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    )
    goal = build_plan_to_pose_goal_for_approach(ctx)
    # Placeholder F5-step1: pose neutra origen.
    assert goal.target_pose_base.position.x == 0.0
    assert goal.target_pose_base.position.y == 0.0
    assert goal.target_pose_base.position.z == 0.0


def test_dispatch_approach_propagates_hint_via_dispatch_context_world_to_base_20260507():
    """2026-05-07: dispatch_phase pasa el clearance al builder y world→base
    se aplica en el goal recibido."""
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec, approach_z_clearance_m=0.07)
    hint_world = (0.6, 0.1, 0.04)
    ctx = _pick_ctx_with_hint(hint=hint_world)
    ok, reason = dispatch_phase(dctx, PickPhase.APPROACH, ctx)
    assert ok is True
    assert reason.startswith("approach:")
    rec_goal = spec.recorded[0].request
    expected = _world_to_base(hint_world[0], hint_world[1], hint_world[2] + 0.07)
    assert rec_goal.target_pose_base.position.x == pytest.approx(expected[0])
    assert rec_goal.target_pose_base.position.y == pytest.approx(expected[1])
    assert rec_goal.target_pose_base.position.z == pytest.approx(expected[2])


# ---------------------------------------------------------------------------
# F5-step2: pose_msg_to_tuple7
# ---------------------------------------------------------------------------


def test_pose_msg_to_tuple7_none_returns_none():
    assert pose_msg_to_tuple7(None) is None


def test_pose_msg_to_tuple7_pose_returns_seven_tuple():
    from geometry_msgs.msg import Pose, Point, Quaternion
    pose = Pose(
        position=Point(x=0.1, y=0.2, z=0.3),
        orientation=Quaternion(x=0.4, y=0.5, z=0.6, w=0.7),
    )
    assert pose_msg_to_tuple7(pose) == (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7)


def test_pose_msg_to_tuple7_invalid_object_returns_none():
    assert pose_msg_to_tuple7("not_a_pose") is None
    assert pose_msg_to_tuple7(object()) is None


# ---------------------------------------------------------------------------
# F5-step3: try_resolve_object_pose_world + APPROACH usa el resolver
# ---------------------------------------------------------------------------


def test_try_resolve_returns_none_when_object_name_empty():
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    assert try_resolve_object_pose_world(dctx, "") is None
    assert try_resolve_object_pose_world(dctx, "   ") is None


def test_try_resolve_returns_none_when_service_fails():
    """Server devuelve success=False ⇒ resolver devuelve None (fallback)."""
    spec = _MockCallerSpec(
        service_results={
            "/orchestrator/resolve_object_pose_world": ServiceCallResult(
                success=False, reason="object_not_in_gazebo"
            )
        }
    )
    dctx = _ctx_for(spec)
    assert try_resolve_object_pose_world(dctx, "missing_object") is None


def test_try_resolve_returns_tuple7_when_success():
    """Server devuelve pose válida ⇒ resolver normaliza a tuple7."""
    from geometry_msgs.msg import Pose, Point, Quaternion
    from types import SimpleNamespace
    payload = SimpleNamespace(
        pose_world=Pose(
            position=Point(x=0.4, y=0.1, z=0.03),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        success=True,
        detail="ok",
    )
    spec = _MockCallerSpec(
        service_results={
            "/orchestrator/resolve_object_pose_world": ServiceCallResult(
                success=True, reason="ok", payload=payload,
            )
        }
    )
    dctx = _ctx_for(spec)
    pose = try_resolve_object_pose_world(dctx, "box_red")
    assert pose == (0.4, 0.1, 0.03, 0.0, 0.0, 0.0, 1.0)


def test_try_resolve_returns_none_when_caller_raises():
    """Excepción inesperada en el caller ⇒ resolver devuelve None."""
    def _raising_caller(*_args, **_kwargs):
        raise RuntimeError("simulated rclpy error")

    dctx = PhaseDispatchContext(
        node=object(),
        service_map=PhaseServiceMap(),
        client_cache={},
        service_caller=_raising_caller,
        action_caller=lambda *a, **k: ActionCallResult(success=True, reason="ok"),
    )
    assert try_resolve_object_pose_world(dctx, "box") is None


def test_dispatch_approach_uses_resolver_when_no_hint():
    """APPROACH sin hint ⇒ llama al resolver, recibe pose, usa esa pose
    para el goal de PlanToPose con clearance Z."""
    from geometry_msgs.msg import Pose, Point, Quaternion
    from types import SimpleNamespace

    resolved_payload = SimpleNamespace(
        pose_world=Pose(
            position=Point(x=0.6, y=-0.1, z=0.04),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
        success=True,
        detail="resolved_from_gz",
    )
    spec = _MockCallerSpec(
        service_results={
            "/orchestrator/resolve_object_pose_world": ServiceCallResult(
                success=True, reason="ok", payload=resolved_payload,
            )
        }
    )
    dctx = _ctx_for(spec)
    # PickContext SIN hint (None).
    ctx = PickContext(object_name="box_red", drop_xyz_world=(0.0, 0.0, 0.0))
    ok, reason = dispatch_phase(dctx, PickPhase.APPROACH, ctx)
    assert ok is True
    assert reason.startswith("approach:")

    # 2 calls: resolver + plan_to_pose.
    kinds = [r.kind for r in spec.recorded]
    assert kinds == ["service", "action"]
    plan_goal = spec.recorded[1].request
    # 2026-05-07: pose resuelta (0.6, -0.1, 0.04) + clearance 0.10 → world
    # (0.6, -0.1, 0.14), convertido a base_link.
    expected = _world_to_base(0.6, -0.1, 0.04 + 0.10)
    assert plan_goal.target_pose_base.position.x == pytest.approx(expected[0])
    assert plan_goal.target_pose_base.position.y == pytest.approx(expected[1])
    assert plan_goal.target_pose_base.position.z == pytest.approx(expected[2])


def test_dispatch_approach_falls_back_to_placeholder_when_resolver_fails():
    """APPROACH sin hint y resolver falla ⇒ usa placeholder (pose 0,0,0)."""
    spec = _MockCallerSpec(
        service_results={
            "/orchestrator/resolve_object_pose_world": ServiceCallResult(
                success=False, reason="server_unavailable"
            )
        }
    )
    dctx = _ctx_for(spec)
    ctx = PickContext(object_name="box_red", drop_xyz_world=(0.0, 0.0, 0.0))
    ok, reason = dispatch_phase(dctx, PickPhase.APPROACH, ctx)
    assert ok is True
    # plan_to_pose se llama con placeholder porque resolver falló.
    plan_goal = spec.recorded[-1].request
    assert plan_goal.target_pose_base.position.x == 0.0
    assert plan_goal.target_pose_base.position.y == 0.0
    assert plan_goal.target_pose_base.position.z == 0.0


def test_dispatch_approach_skips_resolver_when_hint_provided():
    """Si el ctx YA tiene hint, no se invoca al resolver — sólo plan_to_pose."""
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ctx = _pick_ctx_with_hint(hint=(0.5, 0.0, 0.05))
    ok, reason = dispatch_phase(dctx, PickPhase.APPROACH, ctx)
    assert ok is True
    # Sólo 1 call: plan_to_pose. NO resolver.
    assert len(spec.recorded) == 1
    assert spec.recorded[0].kind == "action"


def test_dispatch_approach_skips_resolver_when_object_name_empty():
    """Sin hint y sin object_name ⇒ no se invoca resolver, va a placeholder."""
    spec = _MockCallerSpec()
    dctx = _ctx_for(spec)
    ctx = PickContext(object_name="", drop_xyz_world=(0.0, 0.0, 0.0))
    ok, reason = dispatch_phase(dctx, PickPhase.APPROACH, ctx)
    assert ok is True
    # Sólo plan_to_pose con placeholder.
    assert len(spec.recorded) == 1
    assert spec.recorded[0].kind == "action"
