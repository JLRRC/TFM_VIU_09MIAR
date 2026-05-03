#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/pick_orchestrator_lifecycle_node.py
# Contenido: F9 — Variante LifecycleNode del pick_orchestrator (managed node ROS 2).
"""Pick orchestrator como LifecycleNode (F9).

Variante managed del orchestrator que permite control externo de
estados (configure / activate / deactivate / cleanup / shutdown) via
``ros2 lifecycle`` o el lifecycle_msgs API.

Uso:
    ros2 run tfm_orchestrator pick_orchestrator_lifecycle
    ros2 lifecycle set /pick_orchestrator_lifecycle configure
    ros2 lifecycle set /pick_orchestrator_lifecycle activate
    # ... ahora acepta goals en /pick_place
    ros2 lifecycle set /pick_orchestrator_lifecycle deactivate
    ros2 lifecycle set /pick_orchestrator_lifecycle cleanup
    ros2 lifecycle set /pick_orchestrator_lifecycle shutdown

Diferencias con ``pick_orchestrator_node.py`` (F5/F6):

* Hereda de ``rclpy.lifecycle.LifecycleNode`` en lugar de ``Node``.
* Action server `/pick_place` se crea en ``on_configure`` y se
  habilita/deshabilita en activate/deactivate.
* Goals se rechazan con razón ``node_not_active:<state>`` cuando el
  nodo no está en ACTIVE.
* Recursos se liberan en ``on_cleanup`` permitiendo re-configurar.

La lógica de FSM puro (PickContext / PickPhase) y la ejecución de
fases (service_clients, action calls, PhaseTimings) son idénticas al
nodo F5/F6 — sólo cambia el lifecycle del nodo.
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

from ur5_panel_interfaces.action import PickPlace, PlanToPose
from ur5_panel_interfaces.srv import (
    Attach as AttachSrv,
    Close as CloseSrv,
    Detach as DetachSrv,
    Open as OpenSrv,
    SelectObject as SelectObjectSrv,
)

from .lifecycle_helpers import (
    LifecycleState,
    OrchestratorLifecycleResources,
    reject_reason_for_state,
)
from .phase_dispatch import PhaseDispatchContext, dispatch_phase, pose_msg_to_tuple7
from .phase_timings import PhaseTimings
from .pick_fsm import PickContext, PickPhase
from .service_clients import (
    ActionCallResult,
    PhaseServiceMap,
    call_action_with_timeout,
    call_service_with_timeout,
)


_PHASE_DELAY_SEC = 0.2  # F5 stub — usado solo si use_stubs=True.


class PickOrchestratorLifecycleNode(LifecycleNode):
    """Action server `/pick_place` con ciclo de vida managed.

    Estados:
      - UNCONFIGURED (default): no hay action_server; goals serán rechazados.
      - INACTIVE (post-configure): action_server creado, params cargados,
        clients cacheados; pero ``_accepts_goals=False`` rechaza requests.
      - ACTIVE (post-activate): acepta goals normalmente.
      - INACTIVE (post-deactivate): rechaza goals; recursos siguen vivos.
      - UNCONFIGURED (post-cleanup): action_server destruido; recursos liberados.
      - FINALIZED (post-shutdown): nodo no usable.
    """

    def __init__(self) -> None:
        super().__init__("pick_orchestrator_lifecycle")
        self._cb_group = ReentrantCallbackGroup()
        self._action_server: Optional[ActionServer] = None
        self._service_map: Optional[PhaseServiceMap] = None
        self._client_cache: dict = {}
        self._cancel_requested: bool = False
        self._accepts_goals: bool = False
        self._current_state: LifecycleState = LifecycleState.UNCONFIGURED
        self._use_stubs: bool = True
        self._discovery_timeout: float = 2.0
        self._call_timeout: float = 10.0
        self._resources = OrchestratorLifecycleResources()
        # B-iter5 (2026-05-03): TF buffer + JointState cache para INITIAL_SNAPSHOT real.
        # Init en on_configure (con node ya inicializado).
        self._tf_buffer = None
        self._tf_listener = None
        self._joint_state_cache = None
        self._joint_state_sub = None
        self._snapshot_base_frame: str = "base_link"
        self._snapshot_tcp_frame: str = "rg2_tcp"
        self._snapshot_tf_timeout: float = 0.5
        self._snapshot_require_object_pose: bool = True
        self.get_logger().info(
            "[ORCHESTRATOR_LC] instantiated (UNCONFIGURED) — "
            "use `ros2 lifecycle set <node> configure` para inicializar"
        )

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Crea action_server, carga params, prepara clients cache."""
        try:
            self.declare_parameter("use_stubs", True)
            self.declare_parameter("service_discovery_timeout_sec", 2.0)
            self.declare_parameter("service_call_timeout_sec", 10.0)
            # B-iter5: params del INITIAL_SNAPSHOT real.
            self.declare_parameter("snapshot_base_frame", "base_link")
            self.declare_parameter("snapshot_tcp_frame", "rg2_tcp")
            self.declare_parameter("snapshot_tf_timeout_sec", 0.5)
            self.declare_parameter("snapshot_require_object_pose", True)
        except Exception:
            # Re-configure scenario: parameters already declared, skip.
            pass
        self._use_stubs = bool(self.get_parameter("use_stubs").value)
        self._discovery_timeout = float(
            self.get_parameter("service_discovery_timeout_sec").value
        )
        self._call_timeout = float(self.get_parameter("service_call_timeout_sec").value)
        self._snapshot_base_frame = str(
            self.get_parameter("snapshot_base_frame").value or "base_link"
        )
        self._snapshot_tcp_frame = str(
            self.get_parameter("snapshot_tcp_frame").value or "rg2_tcp"
        )
        self._snapshot_tf_timeout = float(
            self.get_parameter("snapshot_tf_timeout_sec").value
        )
        self._snapshot_require_object_pose = bool(
            self.get_parameter("snapshot_require_object_pose").value
        )
        self._service_map = PhaseServiceMap()
        self._client_cache = {}
        self._cancel_requested = False
        # B-iter5: inicializa TF buffer + JointState subscription para snapshot real.
        try:
            from sensor_msgs.msg import JointState
            from tf2_ros import Buffer, TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._joint_state_cache = None
            self._joint_state_sub = self.create_subscription(
                JointState,
                "/joint_states",
                self._on_joint_state,
                10,
                callback_group=self._cb_group,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"[ORCHESTRATOR_LC] snapshot infra init failed: "
                f"{type(exc).__name__}: {exc} (INITIAL_SNAPSHOT degradará a scaffold)"
            )
            self._tf_buffer = None
            self._joint_state_sub = None
        self._action_server = ActionServer(
            self,
            PickPlace,
            "/pick_place",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        self._accepts_goals = False  # active gating happens in on_activate
        self._current_state = LifecycleState.INACTIVE
        self._resources = OrchestratorLifecycleResources(
            has_action_server=True,
            has_service_map=True,
            params_loaded=True,
            accepts_goals=False,
            config_summary={
                "use_stubs": self._use_stubs,
                "discovery_timeout": self._discovery_timeout,
                "call_timeout": self._call_timeout,
            },
        )
        invariant = self._resources.configure_invariant()
        if invariant is not None:
            self.get_logger().error(
                f"[ORCHESTRATOR_LC] on_configure invariant failed: {invariant}"
            )
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info(
            f"[ORCHESTRATOR_LC] on_configure OK use_stubs={self._use_stubs}"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Habilita aceptación de goals."""
        self._accepts_goals = True
        self._current_state = LifecycleState.ACTIVE
        self._resources.accepts_goals = True
        invariant = self._resources.activate_invariant()
        if invariant is not None:
            self.get_logger().error(
                f"[ORCHESTRATOR_LC] on_activate invariant failed: {invariant}"
            )
            self._accepts_goals = False
            self._resources.accepts_goals = False
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info("[ORCHESTRATOR_LC] on_activate OK — accepting goals")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Rechaza nuevos goals pero mantiene recursos vivos."""
        self._accepts_goals = False
        self._current_state = LifecycleState.INACTIVE
        self._resources.accepts_goals = False
        invariant = self._resources.deactivate_invariant()
        if invariant is not None:
            self.get_logger().error(
                f"[ORCHESTRATOR_LC] on_deactivate invariant failed: {invariant}"
            )
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info(
            "[ORCHESTRATOR_LC] on_deactivate OK — rejecting new goals"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Destruye action_server y libera recursos."""
        if self._action_server is not None:
            try:
                self._action_server.destroy()
            except Exception as exc:
                self.get_logger().warning(
                    f"[ORCHESTRATOR_LC] action_server destroy failed: {exc}"
                )
            self._action_server = None
        self._service_map = None
        self._client_cache = {}
        self._accepts_goals = False
        self._current_state = LifecycleState.UNCONFIGURED
        self._resources = OrchestratorLifecycleResources()
        invariant = self._resources.cleanup_invariant()
        if invariant is not None:
            self.get_logger().error(
                f"[ORCHESTRATOR_LC] on_cleanup invariant failed: {invariant}"
            )
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info("[ORCHESTRATOR_LC] on_cleanup OK — resources released")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Cleanup terminal."""
        if self._action_server is not None:
            try:
                self._action_server.destroy()
            except Exception:
                pass
            self._action_server = None
        self._current_state = LifecycleState.FINALIZED
        self.get_logger().info("[ORCHESTRATOR_LC] on_shutdown OK")
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Action callbacks (gated by lifecycle state)
    # ------------------------------------------------------------------

    def _goal_callback(self, goal_request) -> GoalResponse:
        if not self._accepts_goals:
            reason = reject_reason_for_state(self._current_state) or "node_not_active"
            self.get_logger().warning(
                f"[ORCHESTRATOR_LC] goal rejected: {reason}"
            )
            return GoalResponse.REJECT
        obj = str(getattr(goal_request, "object_name", "") or "").strip()
        if not obj:
            self.get_logger().warning("[ORCHESTRATOR_LC] goal rejected: object_name vacío")
            return GoalResponse.REJECT
        self.get_logger().info(
            f"[ORCHESTRATOR_LC] goal accepted: object={obj} "
            f"drop=({goal_request.drop_xyz_world.x:.3f}, "
            f"{goal_request.drop_xyz_world.y:.3f}, "
            f"{goal_request.drop_xyz_world.z:.3f})"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self._cancel_requested = True
        self.get_logger().info("[ORCHESTRATOR_LC] cancel requested by client")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> PickPlace.Result:
        """Misma lógica que pick_orchestrator_node._execute_callback."""
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
                return result

            self._publish_feedback(goal_handle, ctx)

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
                    return result

        goal_handle.succeed()
        result.success = True
        result.reason = "ok"
        result.duration_sec = time.monotonic() - start_mono
        result.cycles_completed = 1
        timings.mark_session_end(clock_now=time.monotonic())
        self.get_logger().info(
            f"[ORCHESTRATOR_LC] result success=True duration={result.duration_sec:.2f}s"
        )
        return result

    def _execute_phase(self, phase: PickPhase, ctx: PickContext) -> tuple[bool, str]:
        if self._use_stubs:
            time.sleep(_PHASE_DELAY_SEC)
            return True, f"{phase.value}_stub_ok"
        # B-iter5: INITIAL_SNAPSHOT real (intercepta antes del dispatch genérico).
        if phase == PickPhase.INITIAL_SNAPSHOT:
            try:
                return self._capture_initial_snapshot_real(ctx)
            except Exception as exc:
                return False, (
                    f"initial_snapshot_exception:{type(exc).__name__}:{exc}"
                )
        try:
            return self._dispatch_phase_service(phase, ctx)
        except Exception as exc:
            return False, f"phase_dispatch_exception:{type(exc).__name__}:{exc}"

    def _dispatch_phase_service(
        self, phase: PickPhase, ctx: PickContext
    ) -> tuple[bool, str]:
        """F5-step1: delega al módulo puro phase_dispatch para paridad
        funcional con pick_orchestrator_node.py (Node legacy)."""
        dctx = PhaseDispatchContext(
            node=self,
            service_map=self._service_map or PhaseServiceMap(),
            client_cache=self._client_cache,
            discovery_timeout_sec=self._discovery_timeout,
            call_timeout_sec=self._call_timeout,
        )
        return dispatch_phase(dctx, phase, ctx)

    # ------------------------------------------------------------------
    # B-iter5 (2026-05-03) — INITIAL_SNAPSHOT real
    # ------------------------------------------------------------------

    def _on_joint_state(self, msg) -> None:
        """Cachea el último JointState para el snapshot."""
        self._joint_state_cache = msg

    def _capture_initial_snapshot_real(self, ctx: PickContext) -> tuple[bool, str]:
        """Captura TF + joint_state + object_pose y muta el ctx con los valores.

        Llamado desde _execute_phase cuando dst=INITIAL_SNAPSHOT y
        use_stubs=False. Si la infra de snapshot no se inicializó (TF/JointState
        sub fallaron en on_configure), degrada a no-op (scaffold) y devuelve OK
        para no bloquear el resto del FSM.
        """
        from .initial_snapshot import capture_initial_snapshot
        from ur5_panel_interfaces.srv import ResolveObjectPoseWorld
        from .service_clients import call_service_with_timeout

        if self._tf_buffer is None or self._joint_state_sub is None:
            # Infra no disponible — degrada a scaffold.
            return True, "initial_snapshot:scaffold_fallback"

        # Resolver object pose vía service caller (cached).
        def _resolve(name: str):
            req = ResolveObjectPoseWorld.Request()
            req.object_name = name
            r = call_service_with_timeout(
                self,
                ResolveObjectPoseWorld,
                self._service_map.resolve_object_pose_world,
                req,
                discovery_timeout_sec=0.3,
                call_timeout_sec=self._call_timeout,
                client_cache=self._client_cache,
            )
            # call_service_with_timeout devuelve ServiceCallResult con .payload
            # (la response real). Adaptamos para el helper.
            payload = getattr(r, "payload", None)
            if payload is None:
                # Construimos un fake response con success=False para que el
                # helper lo trate como error sin excepción.
                from types import SimpleNamespace
                return SimpleNamespace(
                    success=False,
                    detail=getattr(r, "reason", "service_unavailable"),
                    pose_world=None,
                )
            return payload

        result = capture_initial_snapshot(
            object_name=ctx.object_name,
            tf_lookup=self._tf_buffer.lookup_transform,
            joint_state_msg=self._joint_state_cache,
            resolve_object_pose=_resolve,
            base_frame=self._snapshot_base_frame,
            tcp_frame=self._snapshot_tcp_frame,
            tf_timeout_sec=self._snapshot_tf_timeout,
            require_object_pose=self._snapshot_require_object_pose,
        )

        # Mutar ctx con los capturados (aunque success=False, parciales sirven).
        # PickContext es @dataclass mutable, los campos snapshot son Optional.
        ctx.initial_tcp_pose_base = result.tcp_pose_base
        ctx.initial_joint_positions = result.joint_positions
        ctx.initial_object_pose_world = result.object_pose_world

        if result.success:
            self.get_logger().info(
                f"[ORCHESTRATOR_LC][INITIAL_SNAPSHOT] {result.reason} "
                f"tcp={result.tcp_pose_base} "
                f"joints={result.joint_positions} "
                f"object={result.object_pose_world}"
            )
        else:
            self.get_logger().warning(
                f"[ORCHESTRATOR_LC][INITIAL_SNAPSHOT] partial: {result.reason}"
            )
        return result.success, f"initial_snapshot:{result.reason}"

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
                f"[ORCHESTRATOR_LC] feedback publish failed: {exc}"
            )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = PickOrchestratorLifecycleNode()
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
