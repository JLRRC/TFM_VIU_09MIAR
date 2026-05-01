#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/pick_orchestrator_node.py
# Contenido: F5/F6 — nodo ROS 2 que aloja PickPlace.action y orquesta el flujo.
"""Pick orchestrator node — hosts PickPlace.action.

Este nodo es el **punto de entrada canónico** del flujo pick & place
en el sistema ROS 2. Cliente externo (panel Qt o cualquier otro)
envía un goal ``PickPlace.action`` con ``object_name`` + ``drop_xyz_world``
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

from ur5_panel_interfaces.action import PickPlace
from ur5_panel_interfaces.srv import (
    Attach as AttachSrv,
    Close as CloseSrv,
    Detach as DetachSrv,
    Open as OpenSrv,
    SelectObject as SelectObjectSrv,
    WorldToBase as WorldToBaseSrv,
)

from .pick_fsm import PickContext, PickPhase
from .service_clients import (
    PhaseServiceMap,
    ServiceCallResult,
    call_service_with_timeout,
)


_PHASE_DELAY_SEC = 0.2  # F5 stub — usado solo si use_stubs=True.


class PickOrchestratorNode(Node):
    """Action server que orquesta pick & place via FSM puro."""

    def __init__(self) -> None:
        super().__init__("pick_orchestrator")
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
            f"phases={[p.value for p in [PickPhase.SELECT_OBJECT, PickPhase.APPROACH, PickPhase.GRASP, PickPhase.LIFT, PickPhase.TRANSPORT, PickPhase.RELEASE]]}"
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
        result = PickPlace.Result()

        # Happy path completo. Cada paso es un stub F5 — F6 substituirá
        # por service calls.
        next_phases = [
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
                return result

            self._publish_feedback(goal_handle, ctx)

            # F6: ejecutar la acción real de la fase. Si use_stubs,
            # mantiene comportamiento F5 (sleep). Si no, llama a los
            # services apropiados; ante fallo, marca FAILED y aborta.
            if dst != PickPhase.DONE:
                phase_ok, phase_reason = self._execute_phase(dst, ctx)
                if not phase_ok:
                    ctx.fail(phase_reason)
                    self._publish_feedback(goal_handle, ctx)
                    goal_handle.abort()
                    result.success = False
                    result.reason = phase_reason
                    result.duration_sec = time.monotonic() - start_mono
                    result.cycles_completed = 0
                    return result

        goal_handle.succeed()
        result.success = True
        result.reason = "ok"
        result.duration_sec = time.monotonic() - start_mono
        result.cycles_completed = 1
        self.get_logger().info(
            f"[ORCHESTRATOR] result success=True duration={result.duration_sec:.2f}s"
        )
        return result

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
            # Sin un service real para "approach exec"; F6.2 añadirá
            # PlanToPose.action. Por ahora devolvemos ok para no
            # bloquear (es un placeholder honesto: el orchestrator
            # NO puede ejecutar approach todavía sin planner).
            return True, "approach_placeholder_no_planner_yet"

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
            # Placeholder: futuro PlanToPose.action.
            return True, "lift_placeholder_no_planner_yet"

        if phase == PickPhase.TRANSPORT:
            # Placeholder: requiere WorldToBase + PlanToPose.action.
            return True, "transport_placeholder_no_planner_yet"

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
