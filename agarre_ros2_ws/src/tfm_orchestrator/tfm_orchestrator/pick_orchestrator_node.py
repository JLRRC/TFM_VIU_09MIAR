#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/pick_orchestrator_node.py
# Contenido: F5/F6 — nodo ROS 2 que aloja PickPlace.action y orquesta el flujo.
"""Pick orchestrator node (HUÉRFANO — pendiente de borrado).

.. deprecated:: F11 (2026-05-01) — entry_point eliminado en F1 (2026-05-10).

    Este archivo está HUÉRFANO desde la auditoría F1 del 2026-05-10:
    el ``console_script`` ``pick_orchestrator`` se eliminó de
    ``setup.py`` y ningún launch ni test lo invoca. El punto de
    entrada canónico es ``pick_orchestrator_lifecycle``
    (``pick_orchestrator_lifecycle_node.py``, LifecycleNode con
    configure/activate/deactivate/cleanup/shutdown).

    Pendiente: borrar este archivo en un commit separado con
    ``git rm pick_orchestrator_node.py`` cuando se confirme que ningún
    workflow externo lo importa.
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

from .phase_dispatch import PhaseDispatchContext, dispatch_phase, pose_msg_to_tuple7
from .pick_fsm import PickContext, PickPhase
from .phase_timings import PhaseTimings
from .service_clients import (
    PhaseServiceMap,
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
            object_pose_world_hint=pose_msg_to_tuple7(
                getattr(request, "object_pose_world_hint", None)
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
        """F5-step1: delega al módulo puro ``phase_dispatch`` (compartido
        con el LifecycleNode canónico). Esto garantiza paridad funcional
        sin duplicación."""
        dctx = PhaseDispatchContext(
            node=self,
            service_map=self._service_map,
            client_cache=self._client_cache,
            discovery_timeout_sec=self._discovery_timeout,
            call_timeout_sec=self._call_timeout,
        )
        return dispatch_phase(dctx, phase, ctx)

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
