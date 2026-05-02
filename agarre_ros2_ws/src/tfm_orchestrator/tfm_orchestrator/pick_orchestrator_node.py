#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/pick_orchestrator_node.py
# Contenido: F5/F6 — nodo ROS 2 que aloja PickPlace.action y orquesta el flujo.
"""Pick orchestrator node (LEGACY F5/F6) — hosts PickPlace.action.

.. deprecated:: F11 (2026-05-01)
    Este nodo se mantiene como compatibilidad F5/F6 con clientes y tests
    existentes. **El punto de entrada canónico desde F9 es**
    ``pick_orchestrator_lifecycle`` (LifecycleNode con configure/activate/
    deactivate/cleanup/shutdown). Migra los clientes nuevos a la versión
    lifecycle. Ver ``docs/architecture.md`` §3.2 y ``docs/LIFECYCLE.md``.

Este nodo aloja la action ``PickPlace.action``. Cliente externo (panel
Qt o cualquier otro) envía un goal con ``object_name`` + ``drop_xyz_world``
y recibe feedback estructurado por fase.

El FSM puro (``pick_fsm.PickContext``) vive separado para ser
testeable sin ROS. El nodo lo instancia, lo avanza por las fases, y
en cada transición publica feedback al cliente de la action.

Esta primera versión (F5 mínimo viable) implementa **stubs** para
cada fase: registra el avance, espera 200ms, avanza. La integración
real con services (Open/Close/SetWidth/Attach/Detach/WorldToBase/
ComputeApproachPose) se hará en F6.

Uso:
    ros2 run tfm_orchestrator pick_orchestrator
    ros2 action send_goal /pick_place ur5_panel_interfaces/action/PickPlace \\
        "{object_name: box_red, drop_xyz_world: {x: 0.5, y: 0.0, z: 0.05}}"
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ur5_panel_interfaces.action import PickPlace, PlanToPose
from ur5_panel_interfaces.srv import (
    Attach as AttachSrv,
    Close as CloseSrv,
    Detach as DetachSrv,
    Open as OpenSrv,
    SelectObject as SelectObjectSrv,
    WorldToBase as WorldToBaseSrv,
)

from .pick_fsm import PickContext, PickPhase
from .phase_timings import PhaseTimings
from .service_clients import (
    ActionCallResult,
    PhaseServiceMap,
    ServiceCallResult,
    call_action_with_timeout,
    call_service_with_timeout,
)


_PHASE_DELAY_SEC = 0.2  # F5 stub — usado solo si use_stubs=True.


class PickOrchestratorNode(Node):
    """Action server que orquesta pick & place via FSM puro."""

    def __init__(self) -> None:
        super().__init__("pick_orchestrator")
        self.get_logger().warning(
            "[ORCHESTRATOR][DEPRECATED] pick_orchestrator (F5/F6 Node legacy) está "
            "deprecado desde F11. Usa 'ros2 run tfm_orchestrator pick_orchestrator_lifecycle' "
            "(LifecycleNode F9). Ver docs/LIFECYCLE.md."
        )
        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            PickPlace,
            "/pick_place",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        # F6: parámetros runtime.
        self.declare_parameter("use_stubs", True)
        self.declare_parameter("service_discovery_timeout_sec", 2.0)
        self.declare_parameter("service_call_timeout_sec", 10.0)
        self._use_stubs = bool(self.get_parameter("use_stubs").value)
        self._discovery_timeout = float(
            self.get_parameter("service_discovery_timeout_sec").value
        )
        self._call_timeout = float(
            self.get_parameter("service_call_timeout_sec").value
        )
        self._cancel_requested: bool = False
        self._service_map = PhaseServiceMap()
        self._client_cache: dict = {}
        self.get_logger().info(
            "[ORCHESTRATOR] ready, action=/pick_place "
            f"use_stubs={self._use_stubs} "
            f"phases={[p.value for p in [PickPhase.INITIAL_SNAPSHOT, PickPhase.HOME_INITIAL, PickPhase.SELECT_OBJECT, PickPhase.APPROACH, PickPhase.GRASP, PickPhase.LIFT, PickPhase.TRANSPORT, PickPhase.RELEASE]]}"
        )

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, goal_request) -> GoalResponse:
        obj = str(getattr(goal_request, "object_name", "") or "").strip()
        if not obj:
            self.get_logger().warning("[ORCHESTRATOR] goal rejected: object_name vacío")
            return GoalResponse.REJECT
        self.get_logger().info(
            f"[ORCHESTRATOR] goal accepted: object={obj} "
            f"drop=({goal_request.drop_xyz_world.x:.3f}, "
            f"{goal_request.drop_xyz_world.y:.3f}, "
            f"{goal_request.drop_xyz_world.z:.3f})"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self._cancel_requested = True
        self.get_logger().info("[ORCHESTRATOR] cancel requested by client")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> PickPlace.Result:
        """Avanza el FSM por las fases. Cada transición publica feedback."""
        request = goal_handle.request
        ctx = PickContext(
            object_name=str(request.object_name),
            drop_xyz_world=(
                float(request.drop_xyz_world.x),
                float(request.drop_xyz_world.y),
                float(request.drop_xyz_world.z),
            ),
        )
        self._cancel_requested = False
        start_mono = time.monotonic()
        timings = PhaseTimings()
        timings.mark_session_start(clock_now=start_mono)
        result = PickPlace.Result()

        # Happy path completo. Cada paso es un stub F5 — F6 substituirá
        # por service calls. INITIAL_SNAPSHOT/HOME_INITIAL añadidos en
        # F12-step2c.1 (paridad con el lifecycle node).
        next_phases = [
            (PickPhase.INITIAL_SNAPSHOT, "initial_snapshot_stub"),
            (PickPhase.HOME_INITIAL, "home_initial_stub"),
            (PickPhase.SELECT_OBJECT, "select_object_stub"),
            (PickPhase.APPROACH, "approach_stub"),
            (PickPhase.GRASP, "grasp_stub"),
            (PickPhase.LIFT, "lift_stub"),
            (PickPhase.TRANSPORT, "transport_stub"),
            (PickPhase.RELEASE, "release_stub"),
            (PickPhase.DONE, "done"),
        ]

        for dst, detail in next_phases:
            if self._cancel_requested or goal_handle.is_cancel_requested:
                ctx.abort("client_canceled")
                self._publish_feedback(goal_handle, ctx)
                goal_handle.canceled()
                result.success = False
                result.reason = "canceled"
                result.duration_sec = time.monotonic() - start_mono
                result.cycles_completed = 0
                timings.mark_session_end(clock_now=time.monotonic())
                self._log_timings(timings, outcome="canceled")
                return result

            try:
                ctx.advance(dst, detail=detail)
            except ValueError as exc:
                ctx.fail(str(exc))
                self._publish_feedback(goal_handle, ctx)
                goal_handle.abort()
                result.success = False
                result.reason = f"invalid_transition: {exc}"
                result.duration_sec = time.monotonic() - start_mono
                result.cycles_completed = 0
                timings.mark_session_end(clock_now=time.monotonic())
                self._log_timings(timings, outcome="invalid_transition")
                return result

            self._publish_feedback(goal_handle, ctx)

            # F6: ejecutar la acción real de la fase. Si use_stubs,
            # mantiene comportamiento F5 (sleep). Si no, llama a los
            # services apropiados; ante fallo, marca FAILED y aborta.
            if dst != PickPhase.DONE:
                timings.mark_start(dst.value, clock_now=time.monotonic(), detail=detail)
                phase_ok, phase_reason = self._execute_phase(dst, ctx)
                timings.mark_end(
                    dst.value,
                    clock_now=time.monotonic(),
                    success=phase_ok,
                    detail=phase_reason,
                )
                if not phase_ok:
                    ctx.fail(phase_reason)
                    self._publish_feedback(goal_handle, ctx)
                    goal_handle.abort()
                    result.success = False
                    result.reason = phase_reason
                    result.duration_sec = time.monotonic() - start_mono
                    result.cycles_completed = 0
                    timings.mark_session_end(clock_now=time.monotonic())
                    self._log_timings(timings, outcome="failed")
                    return result

        goal_handle.succeed()
        result.success = True
        result.reason = "ok"
        result.duration_sec = time.monotonic() - start_mono
        result.cycles_completed = 1
        timings.mark_session_end(clock_now=time.monotonic())
        self._log_timings(timings, outcome="ok")
        self.get_logger().info(
            f"[ORCHESTRATOR] result success=True duration={result.duration_sec:.2f}s"
        )
        return result

    def _log_timings(self, timings: PhaseTimings, *, outcome: str) -> None:
        """Loguea el snapshot de PhaseTimings con prefijo [PERF]."""
        import json
        snap = timings.snapshot()
        self.get_logger().info(
            f"[PERF][PICK_PLACE] outcome={outcome} "
            f"total_duration_sec={snap.get('total_duration_sec')} "
            f"phases_completed={snap.get('phases_completed')} "
            f"phases_failed={snap.get('phases_failed')}"
        )
        try:
            self.get_logger().info(
                f"[PERF][PICK_PLACE][SNAPSHOT] {json.dumps(snap)}"
            )
        except Exception as exc:
            self.get_logger().warning(
                f"[PERF][PICK_PLACE] snapshot serialize failed: {exc}"
            )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _execute_phase(self, phase: PickPhase, ctx: PickContext) -> tuple[bool, str]:
        """F6: ejecuta la lógica real de la fase via service calls.

        Si use_stubs=True (default), mantiene el sleep stub de F5.

        Devuelve ``(success, reason)``. reason describe el resultado
        para logging y feedback. Si no es success, el caller fallará
        el FSM con esa reason.
        """
        if self._use_stubs:
            time.sleep(_PHASE_DELAY_SEC)
            return True, f"{phase.value}_stub_ok"

        # F6 real: cada fase tiene su service call.
        try:
            return self._dispatch_phase_service(phase, ctx)
        except Exception as exc:
            return False, f"phase_dispatch_exception:{type(exc).__name__}:{exc}"

    def _dispatch_phase_service(
        self, phase: PickPhase, ctx: PickContext
    ) -> tuple[bool, str]:
        """Mapea fase → service call. Devuelve ``(ok, reason)``."""
        common_kwargs = dict(
            discovery_timeout_sec=self._discovery_timeout,
            call_timeout_sec=self._call_timeout,
            client_cache=self._client_cache,
        )

        if phase == PickPhase.SELECT_OBJECT:
            req = SelectObjectSrv.Request()
            req.name = ctx.object_name
            r = call_service_with_timeout(
                self, SelectObjectSrv,
                self._service_map.select_object, req, **common_kwargs,
            )
            return r.success, f"select_object:{r.reason}"

        if phase == PickPhase.APPROACH:
            # F6.3: usar PlanToPose.action con clearance Z sobre el
            # objeto. Sin acceso al ComputeApproachPose service todavía,
            # construimos un goal con un offset Z fijo (el server real
            # ya hará el computo de pose en su lado).
            goal = self._build_plan_to_pose_goal_for_approach(ctx)
            r = self._call_plan_to_pose(goal)
            return r.success, f"approach:{r.reason}"

        if phase == PickPhase.GRASP:
            # 1) Cerrar gripper.
            req_close = CloseSrv.Request()
            r_close = call_service_with_timeout(
                self, CloseSrv,
                self._service_map.gripper_close, req_close, **common_kwargs,
            )
            if not r_close.success:
                return False, f"grasp_close:{r_close.reason}"
            # 2) Attach lógico al objeto seleccionado.
            req_attach = AttachSrv.Request()
            req_attach.object_name = ctx.object_name
            r_att = call_service_with_timeout(
                self, AttachSrv,
                self._service_map.attach, req_attach, **common_kwargs,
            )
            return r_att.success, f"grasp_attach:{r_att.reason}"

        if phase == PickPhase.LIFT:
            # F6.3: Z+ relativo desde el frame actual.
            goal = self._build_plan_to_pose_goal_for_lift(ctx)
            r = self._call_plan_to_pose(goal)
            return r.success, f"lift:{r.reason}"

        if phase == PickPhase.TRANSPORT:
            # F6.3: planning a drop_xyz_world (en world). El planner del
            # action server se encargará de world→base si lo necesita.
            goal = self._build_plan_to_pose_goal_for_transport(ctx)
            r = self._call_plan_to_pose(goal)
            return r.success, f"transport:{r.reason}"

        if phase == PickPhase.RELEASE:
            # 1) Detach.
            req_det = DetachSrv.Request()
            req_det.object_name = ctx.object_name
            r_det = call_service_with_timeout(
                self, DetachSrv,
                self._service_map.detach, req_det, **common_kwargs,
            )
            if not r_det.success:
                return False, f"release_detach:{r_det.reason}"
            # 2) Abrir gripper.
            req_open = OpenSrv.Request()
            r_open = call_service_with_timeout(
                self, OpenSrv,
                self._service_map.gripper_open, req_open, **common_kwargs,
            )
            return r_open.success, f"release_open:{r_open.reason}"

        # Cualquier otra fase no debería entrar aquí (filtramos DONE
        # en el caller). Si entra, no fallamos — devolvemos ok inerte.
        return True, f"{phase.value}_no_op"

    # ------------------------------------------------------------------
    # F6.3 — PlanToPose action goal builders + caller
    # ------------------------------------------------------------------

    def _call_plan_to_pose(self, goal: PlanToPose.Goal) -> ActionCallResult:
        """Wrapper común para invocar el PlanToPose action server."""
        return call_action_with_timeout(
            self,
            PlanToPose,
            self._service_map.plan_to_pose_action,
            goal,
            discovery_timeout_sec=self._discovery_timeout,
            accept_timeout_sec=max(self._discovery_timeout, 3.0),
            result_timeout_sec=max(self._call_timeout, 30.0),
            client_cache=self._client_cache,
        )

    def _build_plan_to_pose_goal_for_approach(self, ctx: PickContext) -> PlanToPose.Goal:
        """Goal placeholder — el server real hará computo de pose.

        Por ahora envía pose neutra (origen) como target; el server real
        debería resolver la pose de aproach del objeto desde su nombre.
        F6.4 (panel cliente) o F6.5 (server real) refinarán esto.
        """
        from geometry_msgs.msg import Pose, Point, Quaternion
        goal = PlanToPose.Goal()
        goal.target_pose_base = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        goal.ee_frame = "rg2_pinch_center"
        goal.cartesian = False
        goal.timeout_sec = 0.0
        return goal

    def _build_plan_to_pose_goal_for_lift(self, ctx: PickContext) -> PlanToPose.Goal:
        """Goal de lift: Z+ relativo (placeholder)."""
        from geometry_msgs.msg import Pose, Point, Quaternion
        goal = PlanToPose.Goal()
        goal.target_pose_base = Pose(
            position=Point(x=0.0, y=0.0, z=0.20),  # 20cm arriba (placeholder)
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        goal.ee_frame = "rg2_pinch_center"
        goal.cartesian = True  # lift suele ser cartesiano lineal
        goal.timeout_sec = 0.0
        return goal

    def _build_plan_to_pose_goal_for_transport(self, ctx: PickContext) -> PlanToPose.Goal:
        """Goal de transport: drop_xyz_world del request original."""
        from geometry_msgs.msg import Pose, Point, Quaternion
        goal = PlanToPose.Goal()
        goal.target_pose_base = Pose(
            position=Point(
                x=float(ctx.drop_xyz_world[0]),
                y=float(ctx.drop_xyz_world[1]),
                z=float(ctx.drop_xyz_world[2]),
            ),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        goal.ee_frame = "rg2_pinch_center"
        goal.cartesian = False
        goal.timeout_sec = 0.0
        return goal

    def _publish_feedback(self, goal_handle, ctx: PickContext) -> None:
        snap = ctx.feedback_snapshot()
        fb = PickPlace.Feedback()
        fb.current_phase = str(snap["current_phase"])
        fb.progress = float(snap["progress"])
        fb.phase_index = int(snap["phase_index"])
        fb.detail = str(snap["detail"])
        try:
            goal_handle.publish_feedback(fb)
        except Exception as exc:
            self.get_logger().warning(
                f"[ORCHESTRATOR] feedback publish failed: {exc}"
            )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = PickOrchestratorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
