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

from .pick_fsm import PickContext, PickPhase


_PHASE_DELAY_SEC = 0.2  # F5 stub — F6 reemplazará por service calls reales.


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
        self._cancel_requested: bool = False
        self.get_logger().info(
            "[ORCHESTRATOR] ready, action=/pick_place "
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

            # F5 stub: pausa entre fases. F6 reemplazará por service calls.
            time.sleep(_PHASE_DELAY_SEC)

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
