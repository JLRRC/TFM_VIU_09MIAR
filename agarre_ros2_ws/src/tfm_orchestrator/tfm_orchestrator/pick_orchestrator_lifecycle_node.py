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

Características clave (LifecycleNode canónico desde F9):

* Hereda de ``rclpy.lifecycle.LifecycleNode``.
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

from ur5_panel_interfaces.action import PickPlace

from .lifecycle_helpers import (
    LifecycleState,
    OrchestratorLifecycleResources,
    reject_reason_for_state,
)
from .phase_dispatch import PhaseDispatchContext, dispatch_phase, pose_msg_to_tuple7
from .phase_timings import PhaseTimings
from .pick_fsm import PickContext, PickPhase
from .run_id import format_log_line, generate_run_id
from .service_clients import (
    PhaseServiceMap,
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
        # F19 audit-v4 (2026-05-08): ReentrantCallbackGroup intencional.
        # ActionServer.execute_callback ya está serializado por contract
        # (un solo goal activo a la vez). ReentrantCallbackGroup permite
        # que las llamadas a /pick_phase + ActionClient async dentro del
        # execute_callback no se bloqueen mutuamente. MutuallyExclusiveCBG
        # introduciría deadlocks porque cada call síncrona dentro del
        # execute esperaría su propio callback group lock.
        # Verificado live: T35 × 3 ciclos verde con esta config.
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
        # B-iter6 (2026-05-03): ActionClient FollowJointTrajectory para HOME_INITIAL real.
        # Init en on_configure.
        self._fjt_client = None
        self._home_action_name: str = "/joint_trajectory_controller/follow_joint_trajectory"
        # F1.7 audit-v4 (2026-05-08): defaults relajados tras live test 3 ciclos.
        # 0.10 rad / 5.0 s eran demasiado tight para Gazebo Sim — el controller
        # bajo gz_ros2_control no rastrea linear interp con esa precisión y
        # aborta con PATH_TOLERANCE_VIOLATED al primer instante. 1.0 rad / 10s
        # da slack suficiente sin invalidar el "home reached" físicamente
        # (HOME positions tienen separación >> 1.0 rad entre joints).
        self._home_duration_sec: float = 10.0
        self._home_position_tol_rad: float = 1.0
        self._home_goal_time_tol_sec: float = 5.0
        self._home_result_timeout_sec: float = 30.0
        # 2026-05-06: auto_activate honrado al instantiation. El factory
        # (runtime_nodes_factory.py) pasa auto_activate=True por defecto.
        # Sin esto el orchestrator queda UNCONFIGURED y el dispatcher cae
        # al legacy run_pick_demo.
        self.declare_parameter("auto_activate", False)
        self._auto_activate: bool = bool(
            self.get_parameter("auto_activate").get_parameter_value().bool_value
        )
        if self._auto_activate:
            self.get_logger().info(
                "[ORCHESTRATOR_LC] instantiated (UNCONFIGURED) — auto_activate=True; "
                "scheduling configure+activate via timer (rclpy spin requerido)"
            )
            # Timer one-shot para ejecutar las transitions una vez el nodo
            # esté siendo spinneado. trigger_configure/activate síncronos
            # bloquearían si se llaman fuera del executor.
            self._auto_activate_timer = self.create_timer(
                0.2, self._auto_activate_callback
            )
        else:
            self.get_logger().info(
                "[ORCHESTRATOR_LC] instantiated (UNCONFIGURED) — "
                "use `ros2 lifecycle set <node> configure` para inicializar"
            )

    def _auto_activate_callback(self) -> None:
        """Dispara configure + activate una sola vez tras instantiation."""
        try:
            self._auto_activate_timer.cancel()
        except Exception:
            pass
        try:
            self.trigger_configure()
            self.trigger_activate()
            self.get_logger().info(
                "[ORCHESTRATOR_LC] auto_activate completed — node should be ACTIVE"
            )
        except Exception as exc:
            self.get_logger().error(
                f"[ORCHESTRATOR_LC] auto_activate failed: {exc}"
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
            # B-iter6: params del HOME_INITIAL real.
            self.declare_parameter(
                "home_action_name",
                "/joint_trajectory_controller/follow_joint_trajectory",
            )
            # F1.7 audit-v4 defaults relajados (ver __init__ docstring).
            self.declare_parameter("home_duration_sec", 10.0)
            self.declare_parameter("home_position_tol_rad", 1.0)
            self.declare_parameter("home_goal_time_tol_sec", 5.0)
            self.declare_parameter("home_result_timeout_sec", 30.0)
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
        self._home_action_name = str(
            self.get_parameter("home_action_name").value
            or "/joint_trajectory_controller/follow_joint_trajectory"
        )
        self._home_duration_sec = float(
            self.get_parameter("home_duration_sec").value
        )
        self._home_position_tol_rad = float(
            self.get_parameter("home_position_tol_rad").value
        )
        self._home_goal_time_tol_sec = float(
            self.get_parameter("home_goal_time_tol_sec").value
        )
        self._home_result_timeout_sec = float(
            self.get_parameter("home_result_timeout_sec").value
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
        # B-iter6: ActionClient FollowJointTrajectory para HOME_INITIAL real.
        try:
            from rclpy.action import ActionClient
            from control_msgs.action import FollowJointTrajectory
            self._fjt_client = ActionClient(
                self,
                FollowJointTrajectory,
                self._home_action_name,
                callback_group=self._cb_group,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"[ORCHESTRATOR_LC] FJT client init failed: "
                f"{type(exc).__name__}: {exc} (HOME_INITIAL degradará a scaffold)"
            )
            self._fjt_client = None
        self._action_server = ActionServer(
            self,
            PickPlace,
            "/pick_place",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )
        # F5 (auditoría 2026-05-10): PICK_RUN_ID publisher.
        # `/pick/run_id` es String latched (transient_local + reliable)
        # — cualquier nodo que se conecte después recibe el ID actual
        # de la sesión activa. evidence_logger lo consume.
        try:
            from rclpy.qos import (
                DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
            )
            from std_msgs.msg import String as StringMsg
            qos_latched = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            )
            self._run_id_pub = self.create_publisher(
                StringMsg, "/pick/run_id", qos_latched
            )
            self._run_id_msg_cls = StringMsg
        except Exception as exc:
            self.get_logger().warning(
                f"[ORCHESTRATOR_LC] PICK_RUN_ID publisher init failed: "
                f"{type(exc).__name__}: {exc} (logs sin run_id)"
            )
            self._run_id_pub = None
            self._run_id_msg_cls = None
        self._current_run_id: Optional[str] = None
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
        # F5: liberar publisher de PICK_RUN_ID y limpiar ID activo.
        if getattr(self, "_run_id_pub", None) is not None:
            try:
                self.destroy_publisher(self._run_id_pub)
            except Exception:
                pass
            self._run_id_pub = None
            self._run_id_msg_cls = None
        self._current_run_id = None
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

    def _run_log(
        self, phase: str, status: str, msg: str, **extra: object
    ) -> None:
        """Loguea una línea con el formato canónico PICK_RUN_ID.

        F5 (auditoría 2026-05-10). Si no hay run_id activo (caso raro:
        log fuera de un goal), usa ``00000000`` como sentinela.
        Mapea el status a get_logger().info/warn/error según convenga.
        """
        run_id = self._current_run_id or "00000000"
        line = format_log_line(run_id, phase, "orch", status, msg, **extra)
        if status in ("FAILED",):
            self.get_logger().error(line)
        elif status in ("WARN",):
            self.get_logger().warning(line)
        else:
            self.get_logger().info(line)

    def _publish_current_run_id(self) -> None:
        """Publica ``self._current_run_id`` en ``/pick/run_id`` (latched)."""
        if self._run_id_pub is None or self._run_id_msg_cls is None:
            return
        if not self._current_run_id:
            return
        try:
            msg = self._run_id_msg_cls()
            msg.data = self._current_run_id
            self._run_id_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warning(
                f"[ORCHESTRATOR_LC] publish /pick/run_id failed: "
                f"{type(exc).__name__}: {exc}"
            )

    def _execute_callback(self, goal_handle) -> PickPlace.Result:
        """Avanza el FSM por las fases hasta DONE/FAILED/ABORTED."""
        # F5: nuevo PICK_RUN_ID por sesión (un goal == una sesión).
        self._current_run_id = generate_run_id()
        self._publish_current_run_id()

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
        self._run_log(
            "SESSION", "STARTED", "pick_place_goal_received",
            object=ctx.object_name,
            drop_xyz_world=f"{ctx.drop_xyz_world}",
        )

        next_phases = [
            (PickPhase.INITIAL_SNAPSHOT, "initial_snapshot_stub"),
            (PickPhase.HOME_INITIAL, "home_initial_stub"),
            (PickPhase.SELECT_OBJECT, "select_object_stub"),
            (PickPhase.APPROACH, "approach_stub"),
            # 2026-05-07: GRASP_DOWN insertado entre APPROACH y GRASP para
            # que las pinzas físicas RG2 toquen el objeto antes del attach.
            # Sin esto, attach lógico funciona pero las pinzas quedan a 12cm.
            (PickPhase.GRASP_DOWN, "grasp_down_stub"),
            (PickPhase.GRASP, "grasp_stub"),
            (PickPhase.LIFT, "lift_stub"),
            (PickPhase.TRANSPORT, "transport_stub"),
            (PickPhase.RELEASE, "release_stub"),
            (PickPhase.DONE, "done"),
        ]

        for dst, detail in next_phases:
            self.get_logger().info(
                f"[ORCHESTRATOR_LC] phase_loop entering phase={dst.value}"
            )
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

                # B-iter7: callback de feedback intermedio. Las fases de
                # ejecución larga (HOME_INITIAL, MoveIt) lo invocan con
                # sub_progress (0..1) cada vez que el inner action emite
                # feedback. Convertimos a progress global interpolado y
                # publicamos PickPlace.Feedback.
                from .phase_progress import interpolate_phase_progress

                def _publish_intermediate(sub_0_1: float) -> None:
                    try:
                        global_progress = interpolate_phase_progress(
                            ctx.current_phase, sub_0_1
                        )
                        fb = PickPlace.Feedback()
                        fb.current_phase = str(ctx.current_phase.value)
                        fb.progress = float(global_progress)
                        fb.phase_index = int(ctx.index())
                        fb.detail = f"intermediate:sub={float(sub_0_1):.2f}"
                        goal_handle.publish_feedback(fb)
                    except Exception:
                        pass  # nunca dejar que el feedback rompa la fase

                self.get_logger().info(
                    f"[ORCHESTRATOR_LC] phase_loop calling _execute_phase phase={dst.value}"
                )
                phase_ok, phase_reason = self._execute_phase(
                    dst, ctx, intermediate_publisher=_publish_intermediate,
                )
                self.get_logger().info(
                    f"[ORCHESTRATOR_LC] phase_loop _execute_phase returned phase={dst.value} ok={phase_ok} reason={phase_reason}"
                )
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
        self._run_log(
            "SESSION", "FINISHED", "pick_place_goal_done",
            success=True,
            duration_sec=f"{result.duration_sec:.2f}",
        )
        return result

    def _execute_phase(
        self,
        phase: PickPhase,
        ctx: PickContext,
        *,
        intermediate_publisher=None,
    ) -> tuple[bool, str]:
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
        # B-iter6: HOME_INITIAL real (intercepta antes del dispatch genérico).
        if phase == PickPhase.HOME_INITIAL:
            try:
                return self._execute_home_initial_real(
                    ctx, intermediate_publisher=intermediate_publisher,
                )
            except Exception as exc:
                return False, (
                    f"home_initial_exception:{type(exc).__name__}:{exc}"
                )
        try:
            return self._dispatch_phase_service(phase, ctx)
        except Exception as exc:
            return False, f"phase_dispatch_exception:{type(exc).__name__}:{exc}"

    def _dispatch_phase_service(
        self, phase: PickPhase, ctx: PickContext
    ) -> tuple[bool, str]:
        """Delega al módulo puro phase_dispatch."""
        dctx = PhaseDispatchContext(
            node=self,
            service_map=self._service_map or PhaseServiceMap(),
            client_cache=self._client_cache,
            discovery_timeout_sec=self._discovery_timeout,
            call_timeout_sec=self._call_timeout,
            # 2026-05-07 ronda 24: action_result_timeout 300→500s.
            # F1.12 audit-v4 (2026-05-08): subido 500→700s. Con retry sleep
            # 8→20s en plan_to_pose_server, el chain interno (60s first +
            # 20s sleep + 400s retry = 480s) deja sólo 20s margen vs 500
            # outer. Cycles 2/3 v6 hitean outer timeout. 700s da margen
            # cómodo de 220s.
            action_result_timeout_sec=700.0,
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

        # B-iter10: now_sec en ROS time para que las ages calculadas sean
        # consistentes con header.stamp de los msgs (use_sim_time aware).
        try:
            now_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
        except Exception:
            now_sec = None

        result = capture_initial_snapshot(
            object_name=ctx.object_name,
            tf_lookup=self._tf_buffer.lookup_transform,
            joint_state_msg=self._joint_state_cache,
            resolve_object_pose=_resolve,
            base_frame=self._snapshot_base_frame,
            tcp_frame=self._snapshot_tcp_frame,
            tf_timeout_sec=self._snapshot_tf_timeout,
            require_object_pose=self._snapshot_require_object_pose,
            now_sec=now_sec,
        )

        # Mutar ctx con los capturados (aunque success=False, parciales sirven).
        # PickContext es @dataclass mutable, los campos snapshot son Optional.
        ctx.initial_tcp_pose_base = result.tcp_pose_base
        ctx.initial_joint_positions = result.joint_positions
        ctx.initial_object_pose_world = result.object_pose_world

        # B-iter10: freshness gate informativo (no bloquea si stale, sólo
        # adjunta reason). Para abortar en stale, set
        # snapshot_require_fresh_sources=true (param futuro).
        from .pose_consistency import evaluate_pose_freshness_gate

        fresh_ok, fresh_reason = evaluate_pose_freshness_gate(
            result.tcp_tf_age_sec,
            result.joint_state_age_sec,
        )
        full_reason = f"initial_snapshot:{result.reason}|{fresh_reason}"

        if result.success:
            level = self.get_logger().info if fresh_ok else self.get_logger().warning
            level(
                f"[ORCHESTRATOR_LC][INITIAL_SNAPSHOT] {result.reason} "
                f"tcp={result.tcp_pose_base} "
                f"joints={result.joint_positions} "
                f"object={result.object_pose_world} "
                f"freshness={fresh_reason}"
            )
        else:
            self.get_logger().warning(
                f"[ORCHESTRATOR_LC][INITIAL_SNAPSHOT] partial: {result.reason} "
                f"freshness={fresh_reason}"
            )
        return result.success, full_reason

    # ------------------------------------------------------------------
    # B-iter6 (2026-05-03) — HOME_INITIAL real
    # ------------------------------------------------------------------

    def _execute_home_initial_real(
        self,
        ctx: PickContext,
        *,
        intermediate_publisher=None,
    ) -> tuple[bool, str]:
        """Mueve el robot a HOME via FollowJointTrajectory.action directo.

        Llamado desde _execute_phase cuando dst=HOME_INITIAL y use_stubs=False.
        Si el ActionClient FJT no está disponible (init falló en on_configure),
        degrada a scaffold (no-op success) para no bloquear el resto del FSM.

        B-iter7: si ``intermediate_publisher`` no es None, lo invoca con
        sub_progress (0..1) cada feedback recibido del FJT para que el
        cliente del PickPlace.action vea progreso intra-fase.
        """
        import threading
        from .home_initial import (
            build_follow_joint_trajectory_goal,
            build_home_joint_trajectory,
            parse_fjt_result,
        )
        from .phase_progress import fjt_progress_from_feedback

        if self._fjt_client is None:
            return True, "home_initial:scaffold_fallback"

        # Wait for server (timeout corto: si el controller no está listo,
        # mejor degradar que bloquear el ciclo entero).
        if not self._fjt_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warning(
                f"[ORCHESTRATOR_LC][HOME_INITIAL] FJT server unavailable: "
                f"{self._home_action_name}"
            )
            return False, f"home_initial:fjt_server_unavailable:{self._home_action_name}"

        jt = build_home_joint_trajectory(duration_sec=self._home_duration_sec)
        goal = build_follow_joint_trajectory_goal(
            jt,
            position_tol_rad=self._home_position_tol_rad,
            goal_time_tol_sec=self._home_goal_time_tol_sec,
        )

        self.get_logger().info(
            f"[ORCHESTRATOR_LC][HOME_INITIAL] sending goal "
            f"home_positions=[0,-π/2,0,-π/2,0,0] duration={self._home_duration_sec:.1f}s "
            f"pos_tol={self._home_position_tol_rad:.3f}rad"
        )

        # B-iter7: feedback_callback que estima sub_progress desde
        # desired/actual del FJT y publica feedback intermedio del PickPlace.
        def _fjt_feedback_cb(fb_msg) -> None:
            if intermediate_publisher is None:
                return
            try:
                sub = fjt_progress_from_feedback(fb_msg)
                intermediate_publisher(sub)
            except Exception:
                pass

        send_future = self._fjt_client.send_goal_async(
            goal, feedback_callback=_fjt_feedback_cb,
        )
        send_event = threading.Event()
        send_future.add_done_callback(lambda _f: send_event.set())
        send_event.wait(timeout=3.0)
        if not send_future.done():
            return False, "home_initial:fjt_goal_send_timeout"

        gh = send_future.result()
        if gh is None or not getattr(gh, "accepted", False):
            return False, "home_initial:fjt_goal_rejected"

        result_future = gh.get_result_async()
        result_event = threading.Event()
        result_future.add_done_callback(lambda _f: result_event.set())
        result_event.wait(timeout=float(max(1.0, self._home_result_timeout_sec)))
        if not result_future.done():
            return False, (
                f"home_initial:fjt_result_timeout:{self._home_result_timeout_sec:.1f}s"
            )

        wrapper = result_future.result()
        ok, reason = parse_fjt_result(wrapper)
        if ok:
            self.get_logger().info(
                f"[ORCHESTRATOR_LC][HOME_INITIAL] success reason={reason}"
            )
        else:
            self.get_logger().warning(
                f"[ORCHESTRATOR_LC][HOME_INITIAL] failed reason={reason}"
            )
        return ok, f"home_initial:{reason}"

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
    executor = MultiThreadedExecutor(num_threads=2)  # iter4-bis: limitar threads (antes default = cpu_count = 8)
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
