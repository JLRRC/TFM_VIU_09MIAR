#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/plan_to_pose_server.py
# Contenido: F6.5 — action server PlanToPose (stub que delega en plan_to_pose_logic).
"""PlanToPose action server (stub).

Hospeda ``PlanToPose.action`` en ``/orchestrator/plan_to_pose``.
Acepta cualquier goal válido y simula planning + ejecución
publicando feedback intermedio. Devuelve éxito tras un breve sleep.

Esta versión F6.5 NO invoca al bridge MoveIt — el wiring real con
``/desired_grasp`` queda para F6.6. La razón es desacoplar la cadena
del orchestrator (el orchestrator ya espera este action y debe
poder probarse end-to-end sin Gazebo).

La lógica pura vive en ``plan_to_pose_logic.py`` y es 100% testeable
sin ROS.

Uso:
    ros2 run ur5_tools plan_to_pose_server
    ros2 run ur5_tools plan_to_pose_server --ros-args -p step_delay_sec:=0.05
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import threading
import uuid

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

# F1.23 LIVE (2026-05-08): SwitchController para reset del joint_trajectory_controller
# tras retry-fail (BUG_CONTROLLER_FEEDBACK_HANG mitigation deeper).
try:
    from controller_manager_msgs.srv import SwitchController
except Exception:
    SwitchController = None  # type: ignore[misc,assignment]

from ur5_panel_interfaces.action import PlanToPose

from .plan_to_pose_logic import (
    PlanToPoseGoal,
    PlanToPoseResult,
    encode_request_frame,
    execute_stub,
    feedback_sequence,
    parse_bridge_result,
    result_matches_request,
    validate_goal,
)


class PlanToPoseServer(Node):
    """Action server PlanToPose (stub F6.5)."""

    def __init__(self) -> None:
        super().__init__("plan_to_pose_server")
        self._init_declare_and_read_params()
        self._init_setup_tf_and_clients()
        self._init_setup_action_server_and_bridge()

    # ------------------------------------------------------------------
    # __init__ helpers (extraídos en F1.24-refactor T15, 2026-05-08)
    # ------------------------------------------------------------------

    def _init_declare_and_read_params(self) -> None:
        """Declara y lee TODOS los parámetros ROS de este nodo.

        Sin side effects ROS más allá de declare_parameter / get_parameter.
        Setea atributos ``self._<name>``. Refactor offline 1:1 (no cambia
        defaults ni nombres de parámetro respecto a la versión inline).
        """
        # --- Action server / scaffolding ---
        self.declare_parameter("action_name", "/orchestrator/plan_to_pose")
        self.declare_parameter("step_delay_sec", 0.10)
        self.declare_parameter("planning_steps", 3)
        self.declare_parameter("executing_steps", 4)
        # --- Real bridge (F6.6 opt-in) ---
        self.declare_parameter("use_real_bridge", False)
        self.declare_parameter("bridge_pose_topic", "/desired_grasp")
        self.declare_parameter("bridge_result_topic", "/desired_grasp/result")
        self.declare_parameter("bridge_base_frame", "base_link")
        self.declare_parameter("bridge_result_timeout_sec", 60.0)
        # --- Mode resolution (B-iter3, 2026-05-03) ---
        # Valores: "STUB", "REAL_BRIDGE", "MOVEIT_DIRECT".
        # Backwards compat: mode="" + use_real_bridge=true ⇒ "REAL_BRIDGE";
        # mode="" + use_real_bridge=false ⇒ "STUB".
        self.declare_parameter("mode", "")
        # --- MoveIt direct (B-iter3 / B-iter14, 2026-05-03) ---
        self.declare_parameter("moveit_action_name", "/move_action")
        self.declare_parameter("moveit_group_name", "manipulator")
        self.declare_parameter("moveit_tip_link_override", "rg2_tcp")
        self.declare_parameter("moveit_planner_id", "")
        # 2026-05-07: planning_time 5→15s, result_timeout 30→60s — APPROACH
        # timeout sistemático con 5s desde pose post-HOME_INITIAL no canónica.
        self.declare_parameter("moveit_planning_time_sec", 15.0)
        self.declare_parameter("moveit_position_tol_m", 0.005)
        self.declare_parameter("moveit_orientation_tol_rad", 0.05)
        self.declare_parameter("moveit_result_timeout_sec", 60.0)
        # --- FJT directo (F1.24 H9+H10+H11+H14 LIVE, 2026-05-08) ---
        # bypass MoveIt vía /compute_ik + envío directo a FJT action,
        # cierra BUG_CONTROLLER_FEEDBACK_HANG (T35 × 3 cycles verde).
        self.declare_parameter("bypass_moveit_for_short_paths", False)
        self.declare_parameter(
            "fjt_direct_action_name",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("fjt_direct_compute_ik_service", "/compute_ik")
        self.declare_parameter("fjt_direct_duration_sec", 6.0)
        self.declare_parameter("fjt_direct_ik_timeout_sec", 2.0)
        self.declare_parameter("fjt_direct_result_timeout_sec", 30.0)
        # H14: 0.3 rad ≈ 17° — absorbe tracking errors transitorios sin
        # permitir desviaciones peligrosas. 0.0 = no enviar.
        self.declare_parameter("fjt_direct_path_tolerance_rad", 0.3)

        # --- Read into attributes ---
        self._action_name = str(
            self.get_parameter("action_name").value or "/orchestrator/plan_to_pose"
        ).strip()
        self._step_delay = float(self.get_parameter("step_delay_sec").value)
        self._planning_steps = int(self.get_parameter("planning_steps").value)
        self._executing_steps = int(self.get_parameter("executing_steps").value)

        self._use_real_bridge = bool(self.get_parameter("use_real_bridge").value)
        self._bridge_pose_topic = str(
            self.get_parameter("bridge_pose_topic").value or "/desired_grasp"
        ).strip()
        self._bridge_result_topic = str(
            self.get_parameter("bridge_result_topic").value or "/desired_grasp/result"
        ).strip()
        self._bridge_base_frame = str(
            self.get_parameter("bridge_base_frame").value or "base_link"
        ).strip() or "base_link"
        self._bridge_result_timeout = float(
            self.get_parameter("bridge_result_timeout_sec").value
        )

        _raw_mode = str(self.get_parameter("mode").value or "").strip().upper()
        if _raw_mode in {"STUB", "REAL_BRIDGE", "MOVEIT_DIRECT"}:
            self._mode = _raw_mode
        else:
            self._mode = "REAL_BRIDGE" if self._use_real_bridge else "STUB"

        self._moveit_action_name = str(
            self.get_parameter("moveit_action_name").value or "/move_action"
        ).strip() or "/move_action"
        self._moveit_group_name = str(
            self.get_parameter("moveit_group_name").value or "manipulator"
        ).strip() or "manipulator"
        self._moveit_planner_id = str(
            self.get_parameter("moveit_planner_id").value or ""
        ).strip()
        self._moveit_tip_link_override = str(
            self.get_parameter("moveit_tip_link_override").value or ""
        ).strip()
        self._moveit_planning_time = float(
            self.get_parameter("moveit_planning_time_sec").value
        )
        self._moveit_position_tol = float(
            self.get_parameter("moveit_position_tol_m").value
        )
        self._moveit_orientation_tol = float(
            self.get_parameter("moveit_orientation_tol_rad").value
        )
        self._moveit_result_timeout = float(
            self.get_parameter("moveit_result_timeout_sec").value
        )

        self._bypass_moveit_for_short_paths = bool(
            self.get_parameter("bypass_moveit_for_short_paths").value
        )
        self._fjt_direct_action_name = str(
            self.get_parameter("fjt_direct_action_name").value
        )
        self._fjt_direct_ik_service = str(
            self.get_parameter("fjt_direct_compute_ik_service").value
        )
        self._fjt_direct_duration = float(
            self.get_parameter("fjt_direct_duration_sec").value
        )
        self._fjt_direct_ik_timeout = float(
            self.get_parameter("fjt_direct_ik_timeout_sec").value
        )
        self._fjt_direct_result_timeout = float(
            self.get_parameter("fjt_direct_result_timeout_sec").value
        )
        self._fjt_direct_path_tolerance_rad = float(
            self.get_parameter("fjt_direct_path_tolerance_rad").value
        )

    def _init_setup_tf_and_clients(self) -> None:
        """Inicializa TF buffer/listener + lazy clients y subs.

        Side effects ROS: TransformListener subscribe, switch_controller
        client create, /joint_states subscription. Atributos lazy
        (``_moveit_action_client``, ``_fjt_direct_action_client``,
        ``_fjt_direct_ik_client``) quedan en None — se inicializan en
        primer uso.
        """
        # ActionClient /move_action (lazy en _execute_moveit_direct).
        self._moveit_action_client = None

        # F1.22 LIVE: TF buffer para verificar pose post-FIRST_ATTEMPT_TIMEOUT.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # F1.23 LIVE: cliente SwitchController para restart del JTC.
        self._switch_controller_client = None
        if SwitchController is not None:
            try:
                self._switch_controller_client = self.create_client(
                    SwitchController,
                    "/controller_manager/switch_controller",
                )
            except Exception as exc:
                self.get_logger().warning(
                    f"[PLAN_TO_POSE] no se pudo crear cliente switch_controller: {exc}"
                )

        # F1.24 / H9 LIVE: estado lazy del path FJT directo.
        self._fjt_direct_action_client = None
        self._fjt_direct_ik_client = None
        self._latest_joint_state = None
        self._joint_state_lock = threading.Lock()
        try:
            from sensor_msgs.msg import JointState as _JointState
            self.create_subscription(
                _JointState,
                "/joint_states",
                self._on_joint_state,
                10,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"[PLAN_TO_POSE] no se pudo crear sub /joint_states: {exc}"
            )

    def _init_setup_action_server_and_bridge(self) -> None:
        """Crea action server + (opcional) wiring del REAL_BRIDGE + log final."""
        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            PlanToPose,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        # F6.6: bridge wiring sólo si REAL_BRIDGE.
        self._bridge_pose_pub = None
        self._bridge_result_sub = None
        self._bridge_result_lock = threading.Lock()
        self._bridge_pending_uuid: Optional[str] = None
        self._bridge_pending_event = threading.Event()
        self._bridge_pending_text: Optional[str] = None
        self._next_request_id = 0

        if self._mode == "REAL_BRIDGE":
            self._bridge_pose_pub = self.create_publisher(
                PoseStamped, self._bridge_pose_topic, 10
            )
            self._bridge_result_sub = self.create_subscription(
                String,
                self._bridge_result_topic,
                self._on_bridge_result,
                10,
                callback_group=self._cb_group,
            )

        self.get_logger().info(
            f"[PLAN_TO_POSE] ready, action={self._action_name} mode={self._mode} "
            f"planning={self._planning_steps} executing={self._executing_steps} "
            f"step_delay={self._step_delay}s"
        )
        if self._mode == "REAL_BRIDGE":
            self.get_logger().info(
                f"[PLAN_TO_POSE] bridge pose_topic={self._bridge_pose_topic} "
                f"result_topic={self._bridge_result_topic} "
                f"base_frame={self._bridge_base_frame} "
                f"result_timeout={self._bridge_result_timeout:.1f}s"
            )
        elif self._mode == "MOVEIT_DIRECT":
            self.get_logger().info(
                f"[PLAN_TO_POSE] moveit_direct action={self._moveit_action_name} "
                f"group={self._moveit_group_name} planner='{self._moveit_planner_id}' "
                f"planning_time={self._moveit_planning_time:.1f}s "
                f"pos_tol={self._moveit_position_tol:.4f}m "
                f"ori_tol={self._moveit_orientation_tol:.3f}rad "
                f"result_timeout={self._moveit_result_timeout:.1f}s"
            )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, goal_request) -> GoalResponse:
        # Pre-validación rápida usando lógica pura.
        parsed = self._parse_goal(goal_request)
        ok, reason = validate_goal(parsed)
        if not ok:
            self.get_logger().warning(f"[PLAN_TO_POSE] goal rejected: {reason}")
            return GoalResponse.REJECT
        self.get_logger().info(
            f"[PLAN_TO_POSE] goal accepted: ee_frame={parsed.ee_frame} "
            f"cartesian={parsed.cartesian} target=({parsed.target_xyz[0]:.3f}, "
            f"{parsed.target_xyz[1]:.3f}, {parsed.target_xyz[2]:.3f})"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("[PLAN_TO_POSE] cancel requested")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> PlanToPose.Result:
        request = goal_handle.request
        parsed = self._parse_goal(request)
        start_mono = time.monotonic()

        # Publicar feedback determinístico de la lógica pura.
        for fb in feedback_sequence(
            parsed,
            n_planning_steps=self._planning_steps,
            n_executing_steps=self._executing_steps,
        ):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = PlanToPose.Result()
                result.success = False
                result.reason = "canceled"
                result.duration_sec = time.monotonic() - start_mono
                result.attempts = 1
                return result
            self._publish_feedback(goal_handle, fb)
            time.sleep(max(0.0, self._step_delay))

        # Ejecutar según modo:
        #  - STUB: lógica pura (instantáneo, success).
        #  - REAL_BRIDGE: publica PoseStamped a /desired_grasp y espera panel.
        #  - MOVEIT_DIRECT (B-iter3): cliente directo a /move_action de MoveIt.
        duration = time.monotonic() - start_mono
        if self._mode == "MOVEIT_DIRECT":
            outcome = self._execute_moveit_direct(parsed, start_mono)
        elif self._mode == "REAL_BRIDGE":
            outcome = self._execute_real_bridge(parsed, start_mono)
        else:
            outcome = execute_stub(parsed, duration_sec=duration)

        result = PlanToPose.Result()
        result.success = bool(outcome.success)
        result.reason = str(outcome.reason)
        result.final_pose_base = self._final_pose_msg(outcome)
        result.duration_sec = float(outcome.duration_sec)
        result.attempts = int(outcome.attempts)

        if outcome.success:
            goal_handle.succeed()
            self.get_logger().info(
                f"[PLAN_TO_POSE] success duration={result.duration_sec:.2f}s"
            )
        else:
            goal_handle.abort()
            self.get_logger().warning(
                f"[PLAN_TO_POSE] aborted reason={result.reason}"
            )
        return result

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_goal(request) -> PlanToPoseGoal:
        return PlanToPoseGoal(
            target_xyz=(
                float(request.target_pose_base.position.x),
                float(request.target_pose_base.position.y),
                float(request.target_pose_base.position.z),
            ),
            target_quat_xyzw=(
                float(request.target_pose_base.orientation.x),
                float(request.target_pose_base.orientation.y),
                float(request.target_pose_base.orientation.z),
                float(request.target_pose_base.orientation.w),
            ),
            ee_frame=str(request.ee_frame or "").strip() or "rg2_pinch_center",
            cartesian=bool(request.cartesian),
            timeout_sec=float(request.timeout_sec or 0.0),
        )

    @staticmethod
    def _final_pose_msg(outcome):
        from geometry_msgs.msg import Pose, Point, Quaternion
        return Pose(
            position=Point(
                x=float(outcome.final_xyz[0]),
                y=float(outcome.final_xyz[1]),
                z=float(outcome.final_xyz[2]),
            ),
            orientation=Quaternion(
                x=float(outcome.final_quat_xyzw[0]),
                y=float(outcome.final_quat_xyzw[1]),
                z=float(outcome.final_quat_xyzw[2]),
                w=float(outcome.final_quat_xyzw[3]),
            ),
        )

    # ------------------------------------------------------------------
    # F6.6 — wiring real con el bridge MoveIt
    # ------------------------------------------------------------------

    def _execute_real_bridge(
        self, goal: PlanToPoseGoal, start_mono: float
    ) -> PlanToPoseResult:
        """Publica el goal al bridge y espera el result correlado por UUID."""
        from .plan_to_pose_logic import normalize_quat
        if self._bridge_pose_pub is None:
            return PlanToPoseResult(
                success=False,
                reason="bridge_publisher_not_initialized",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=0,
            )

        # Generar UUID único para correlación + frame_id codificado.
        request_uuid = uuid.uuid4().hex
        self._next_request_id += 1
        request_id = int(self._next_request_id)
        frame_id = encode_request_frame(
            self._bridge_base_frame,
            request_id,
            request_uuid,
            phase_label="PLAN_TO_POSE",
        )

        # Construir PoseStamped y publicar.
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        try:
            msg.header.stamp = self.get_clock().now().to_msg()
        except Exception:
            pass
        msg.pose.position.x = float(goal.target_xyz[0])
        msg.pose.position.y = float(goal.target_xyz[1])
        msg.pose.position.z = float(goal.target_xyz[2])
        msg.pose.orientation.x = float(goal.target_quat_xyzw[0])
        msg.pose.orientation.y = float(goal.target_quat_xyzw[1])
        msg.pose.orientation.z = float(goal.target_quat_xyzw[2])
        msg.pose.orientation.w = float(goal.target_quat_xyzw[3])

        # Armar pending antes de publicar (evita race con result que llega rápido).
        with self._bridge_result_lock:
            self._bridge_pending_uuid = request_uuid
            self._bridge_pending_text = None
            self._bridge_pending_event.clear()

        try:
            self._bridge_pose_pub.publish(msg)
        except Exception as exc:
            return PlanToPoseResult(
                success=False,
                reason=f"bridge_publish_exception:{type(exc).__name__}:{exc}",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=1,
            )

        self.get_logger().info(
            f"[PLAN_TO_POSE][BRIDGE] published rid={request_id} uid={request_uuid} "
            f"target=({goal.target_xyz[0]:.3f},{goal.target_xyz[1]:.3f},{goal.target_xyz[2]:.3f})"
        )

        # Esperar result. El callback _on_bridge_result setea
        # _bridge_pending_event cuando el UUID coincida.
        timeout = float(max(1.0, self._bridge_result_timeout))
        ok_wait = self._bridge_pending_event.wait(timeout=timeout)
        with self._bridge_result_lock:
            text = self._bridge_pending_text
            self._bridge_pending_uuid = None  # release slot

        if not ok_wait or text is None:
            return PlanToPoseResult(
                success=False,
                reason=f"bridge_result_timeout:{timeout:.1f}s",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=1,
            )

        success, reason, _uid = parse_bridge_result(text)
        ok = bool(success) if success is not None else False
        return PlanToPoseResult(
            success=ok,
            reason=reason or ("ok" if ok else "bridge_unknown"),
            final_xyz=goal.target_xyz,
            final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
            duration_sec=time.monotonic() - start_mono,
            attempts=1,
        )

    # ------------------------------------------------------------------
    # B-iter3 (2026-05-03) — modo MOVEIT_DIRECT
    # ------------------------------------------------------------------

    def _execute_moveit_direct(
        self, goal: PlanToPoseGoal, start_mono: float
    ) -> PlanToPoseResult:
        """Llama directamente a /move_action de MoveIt 2 (sin bridge al panel).

        Flujo:
          0. F1.24 / H9 LIVE (2026-05-08): si bypass_moveit_for_short_paths,
             intentar primero FJT directo (sin pasar por simple_controller_manager).
             Si IK ok y controller responde → success. Si falla, falls back a MoveIt.
          1. Construye MoveGroup.Goal con PositionConstraint + OrientationConstraint
             sobre ee_frame del request.
          2. wait_for_server / send_goal / wait result (event-based).
          3. Decodifica MoveItErrorCodes.val a (success, reason).
        """
        from rclpy.action import ActionClient as _ActionClient
        from moveit_msgs.action import MoveGroup
        from .plan_to_pose_logic import normalize_quat
        from .plan_to_pose_moveit_direct import (
            build_move_group_goal,
            parse_move_group_result,
        )

        # F1.24 / H9 LIVE: intento prioritario FJT directo (bypass MoveIt).
        if self._bypass_moveit_for_short_paths:
            self.get_logger().info(
                "[PLAN_TO_POSE][FJT_DIRECT] intento prioritario "
                "(bypass_moveit_for_short_paths=true)"
            )
            fjt_result = self._execute_fjt_direct(goal, start_mono)
            if fjt_result is not None and fjt_result.success:
                return fjt_result
            # None = pre-condiciones no cumplidas (joint_state ausente, IK fail,
            # controller no ready) — fall through al path MoveIt original.
            if fjt_result is not None and not fjt_result.success:
                # FJT respondió pero falló (path tolerance, etc) — devolver el fail.
                return fjt_result
            self.get_logger().info(
                "[PLAN_TO_POSE][FJT_DIRECT] fallback al path MoveIt"
            )

        if self._moveit_action_client is None:
            self._moveit_action_client = _ActionClient(
                self, MoveGroup, self._moveit_action_name,
                callback_group=self._cb_group,
            )

        # Wait for server (timeout corto para no bloquear pick si MoveIt no listo).
        if not self._moveit_action_client.wait_for_server(timeout_sec=3.0):
            return PlanToPoseResult(
                success=False,
                reason=f"moveit_action_server_unavailable:{self._moveit_action_name}",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=0,
            )

        # B-iter14: si moveit_tip_link_override está set, usarlo para las
        # constraints (debe ser tip_link del SRDF del group). El ee_frame
        # del goal sigue siendo semántico (no se cambia el target — pero
        # las constraints apuntan al link correcto del kinematic chain).
        effective_ee_frame = (
            self._moveit_tip_link_override
            if self._moveit_tip_link_override
            else goal.ee_frame
        )
        # F1.18 audit-v4 (2026-05-08): heurística per-fase delegada a
        # classify_phase_by_target_z (función pura, testeable offline).
        # TRANSPORT (drop pose, target Z < 0.05 base_link) usa scaling=0.5 +
        # first_attempt_timeout=240s; resto de fases (APPROACH/GRASP_DOWN/LIFT)
        # mantienen scaling=0.25 + first_attempt_timeout=120s.
        from ur5_tools.plan_to_pose_moveit_direct import classify_phase_by_target_z
        phase_tuning = classify_phase_by_target_z(goal.target_xyz)
        is_transport_phase = phase_tuning.phase_label == "TRANSPORT"
        mg_goal = build_move_group_goal(
            goal.target_xyz,
            goal.target_quat_xyzw,
            ee_frame=effective_ee_frame,
            base_frame=self._bridge_base_frame,
            group_name=self._moveit_group_name,
            planner_id=self._moveit_planner_id,
            planning_time_sec=self._moveit_planning_time,
            position_tol_m=self._moveit_position_tol,
            orientation_tol_rad=self._moveit_orientation_tol,
            velocity_scaling_factor=phase_tuning.velocity_scaling,
            acceleration_scaling_factor=phase_tuning.acceleration_scaling,
        )

        self.get_logger().info(
            "[PLAN_TO_POSE][MOVEIT_DIRECT] sending goal "
            f"target=({goal.target_xyz[0]:.3f},{goal.target_xyz[1]:.3f},{goal.target_xyz[2]:.3f}) "
            f"ee_frame={goal.ee_frame} group={self._moveit_group_name}"
        )

        send_future = self._moveit_action_client.send_goal_async(mg_goal)
        send_event = threading.Event()
        send_future.add_done_callback(lambda _f: send_event.set())
        send_event.wait(timeout=3.0)
        if not send_future.done():
            return PlanToPoseResult(
                success=False,
                reason="moveit_goal_send_timeout",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=1,
            )

        gh = send_future.result()
        if gh is None or not getattr(gh, "accepted", False):
            return PlanToPoseResult(
                success=False,
                reason="moveit_goal_rejected",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=1,
            )

        # F1.7 audit-v4 (2026-05-08): first-attempt timeout corto para detectar
        # hang del bridge (race condition controller_manager). Si MoveIt no
        # devuelve resultado en _MOVEIT_FIRST_ATTEMPT_TIMEOUT_SEC, asumimos
        # que el controller no conectó y disparamos el retry con el timeout
        # completo. Sin esto, el primer attempt bloquea hasta
        # _moveit_result_timeout completo (400s) y el retry NUNCA fira porque
        # el reason resultante es "moveit_result_timeout" no "CONTROL_FAILED".
        # F1.12 audit-v4 (2026-05-08): primero subido 60→120s, pero rompió
        # APPROACH por outer orchestrator timeout (500s). Revertido a 60s.
        # El retry sleep es la palanca correcta — no first_attempt.
        # F1.17 audit-v4 (2026-05-08): outer timeout subido a 700s (F1.12),
        # cabe 120 + 20 + 120 = 260s por retry_with_backoff, ×2 = 520s + 3s
        # backoff = 523s < 700s. Subiendo 60→120s para tolerar OMPL más
        # lento en targets alejados (cycle 3 box_green a 0.75m falló por
        # OMPL flaky con 60s).
        # F1.18 audit-v4 (2026-05-08): timeout per-fase delegado a
        # classify_phase_by_target_z. TRANSPORT 240s, resto 120s. Outer 700s
        # acomoda un retry de TRANSPORT (240+20+240=500s) o ×2 cycles cortos.
        first_attempt_timeout = min(
            phase_tuning.first_attempt_timeout_sec,
            float(max(1.0, self._moveit_result_timeout)),
        )

        result_future = gh.get_result_async()
        result_event = threading.Event()
        result_future.add_done_callback(lambda _f: result_event.set())
        result_event.wait(timeout=first_attempt_timeout)
        if not result_future.done():
            # F1.7 audit-v4: primer attempt hang → tratar como CONTROL_FAILED
            # para disparar retry. Cancelar la goal pendiente para liberar
            # el server.
            self.get_logger().warning(
                f"[PLAN_TO_POSE][MOVEIT_DIRECT] first attempt hang "
                f"timeout={first_attempt_timeout:.1f}s — cancelando goal y "
                "disparando retry (probable bridge race condition)"
            )
            try:
                cancel_future = gh.cancel_goal_async()
                cancel_event = threading.Event()
                cancel_future.add_done_callback(lambda _f: cancel_event.set())
                cancel_event.wait(timeout=2.0)
            except Exception:
                pass
            ok = False
            reason = f"FIRST_ATTEMPT_TIMEOUT:{first_attempt_timeout:.1f}s"
            wrapper = None  # type: ignore[assignment]

            # F1.22 LIVE (2026-05-08): BUG_CONTROLLER_FEEDBACK_HANG mitigation.
            # Tras FIRST_ATTEMPT_TIMEOUT, verificar si el robot ya alcanzó el
            # target (TF lookup). Si está dentro de moveit_position_tol *
            # tf_check_factor, considerar success — el controller ejecutó la
            # trayectoria pero el feedback "Goal reached" no llegó al move_group.
            try:
                tf_target_pos = self._lookup_ee_position_in_base(
                    ee_frame=effective_ee_frame,
                    base_frame=self._bridge_base_frame,
                    timeout_sec=1.0,
                )
            except Exception as exc:
                tf_target_pos = None
                self.get_logger().warning(
                    f"[PLAN_TO_POSE][MOVEIT_DIRECT] TF lookup post-timeout fail: "
                    f"{type(exc).__name__}: {exc}"
                )
            if tf_target_pos is not None:
                tf_check_tol = max(0.05, self._moveit_position_tol * 5.0)
                dx = float(tf_target_pos[0]) - float(goal.target_xyz[0])
                dy = float(tf_target_pos[1]) - float(goal.target_xyz[1])
                dz = float(tf_target_pos[2]) - float(goal.target_xyz[2])
                dist = (dx * dx + dy * dy + dz * dz) ** 0.5
                self.get_logger().info(
                    f"[PLAN_TO_POSE][MOVEIT_DIRECT] post-timeout TF check "
                    f"actual=({tf_target_pos[0]:.3f},{tf_target_pos[1]:.3f},"
                    f"{tf_target_pos[2]:.3f}) target=({goal.target_xyz[0]:.3f},"
                    f"{goal.target_xyz[1]:.3f},{goal.target_xyz[2]:.3f}) "
                    f"dist={dist:.4f}m tol={tf_check_tol:.4f}m"
                )
                if dist <= tf_check_tol:
                    self.get_logger().info(
                        f"[PLAN_TO_POSE][MOVEIT_DIRECT] feedback_hang_recovered "
                        f"(robot at target post-timeout, dist={dist:.4f}m"
                        f"<={tf_check_tol:.4f}m) — returning success "
                        f"reason=feedback_hang_recovered_via_tf"
                    )
                    return PlanToPoseResult(
                        success=True,
                        reason=(
                            f"feedback_hang_recovered_via_tf"
                            f"|orig={reason}|dist={dist:.4f}m"
                        ),
                        final_xyz=goal.target_xyz,
                        final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                        duration_sec=time.monotonic() - start_mono,
                        attempts=1,
                    )
        else:
            wrapper = result_future.result()
            ok, reason = parse_move_group_result(wrapper)
        if ok:
            self.get_logger().info(
                f"[PLAN_TO_POSE][MOVEIT_DIRECT] success reason={reason}"
            )
            return PlanToPoseResult(
                success=ok,
                reason=reason,
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=1,
            )

        # 2026-05-07 fix bug bridge: si CONTROL_FAILED (race condition de
        # MoveIt simple_controller_manager con joint_trajectory_controller
        # action server), reintentar UNA vez tras esperar 8s. El error
        # "Action client not connected to action server" se da en el primer
        # goal cuando MoveIt aún no terminó de detectar el controller.
        if (
            "CONTROL_FAILED" in reason
            or "TIMED_OUT" in reason
            or "FIRST_ATTEMPT_TIMEOUT" in reason
        ):
            # F1.23 LIVE (2026-05-08): tras FIRST_ATTEMPT_TIMEOUT, hacer
            # restart del joint_trajectory_controller ANTES del retry.
            # Hipótesis: el controller queda en estado "ghost-busy" tras
            # un cancel mal procesado por simple_controller_manager. El
            # restart fuerza una transición de estado limpia.
            if "FIRST_ATTEMPT_TIMEOUT" in reason:
                self.get_logger().warning(
                    "[PLAN_TO_POSE][MOVEIT_DIRECT] reset joint_trajectory_controller "
                    "antes del retry (BUG_CONTROLLER_FEEDBACK_HANG mitigation)"
                )
                self._restart_joint_trajectory_controller(timeout_sec=3.0)
            # F1.12 audit-v4 (2026-05-08): retry sleep 8 → 20s. 8s era OK
            # para el race condition de startup pero post-GRASP attach el
            # controller_manager pausa más tiempo antes de aceptar un nuevo
            # trajectory. 20s da margen adecuado.
            self.get_logger().warning(
                f"[PLAN_TO_POSE][MOVEIT_DIRECT] failed reason={reason} — "
                "intentando retry tras 20s (race condition controller_manager)"
            )
            time.sleep(20.0)
            send_future_retry = self._moveit_action_client.send_goal_async(mg_goal)
            retry_send_event = threading.Event()
            send_future_retry.add_done_callback(lambda _f: retry_send_event.set())
            retry_send_event.wait(timeout=3.0)
            if send_future_retry.done():
                gh_retry = send_future_retry.result()
                if gh_retry is not None and getattr(gh_retry, "accepted", False):
                    result_future_retry = gh_retry.get_result_async()
                    retry_result_event = threading.Event()
                    result_future_retry.add_done_callback(
                        lambda _f: retry_result_event.set()
                    )
                    # F1.23 LIVE (2026-05-08): retry timeout cap a
                    # first_attempt_timeout para detectar feedback hang
                    # rápidamente y devolver via TF check si robot llegó.
                    retry_timeout_effective = float(
                        min(first_attempt_timeout,
                            max(1.0, self._moveit_result_timeout))
                    )
                    retry_result_event.wait(timeout=retry_timeout_effective)
                    if result_future_retry.done():
                        wrapper_retry = result_future_retry.result()
                        ok_retry, reason_retry = parse_move_group_result(wrapper_retry)
                        if ok_retry:
                            self.get_logger().info(
                                f"[PLAN_TO_POSE][MOVEIT_DIRECT] retry success "
                                f"reason={reason_retry}"
                            )
                            return PlanToPoseResult(
                                success=True,
                                reason=f"{reason_retry}|retry_after_{reason}",
                                final_xyz=goal.target_xyz,
                                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                                duration_sec=time.monotonic() - start_mono,
                                attempts=2,
                            )
                        reason = f"{reason_retry}|retry_failed"
                    else:
                        # F1.23 LIVE (2026-05-08): retry también colgó por
                        # feedback hang. Cancel + TF check final.
                        try:
                            cancel_r = gh_retry.cancel_goal_async()
                            evt_cr = threading.Event()
                            cancel_r.add_done_callback(lambda _f: evt_cr.set())
                            evt_cr.wait(timeout=2.0)
                        except Exception:
                            pass
                        reason = f"{reason}|retry_first_attempt_hang"
            else:
                reason = f"{reason}|retry_send_timeout"

            # F1.23 LIVE (2026-05-08): tras todos los intentos fallidos,
            # último TF check — si el robot sí llegó al target durante
            # alguno de los attempts (y solo el feedback se perdió),
            # devolver success.
            try:
                final_tf = self._lookup_ee_position_in_base(
                    ee_frame=effective_ee_frame,
                    base_frame=self._bridge_base_frame,
                    timeout_sec=1.5,
                )
            except Exception:
                final_tf = None
            if final_tf is not None:
                final_tol = max(0.05, self._moveit_position_tol * 5.0)
                fdx = float(final_tf[0]) - float(goal.target_xyz[0])
                fdy = float(final_tf[1]) - float(goal.target_xyz[1])
                fdz = float(final_tf[2]) - float(goal.target_xyz[2])
                fdist = (fdx * fdx + fdy * fdy + fdz * fdz) ** 0.5
                self.get_logger().info(
                    f"[PLAN_TO_POSE][MOVEIT_DIRECT] final TF check "
                    f"actual=({final_tf[0]:.3f},{final_tf[1]:.3f},"
                    f"{final_tf[2]:.3f}) target=({goal.target_xyz[0]:.3f},"
                    f"{goal.target_xyz[1]:.3f},{goal.target_xyz[2]:.3f}) "
                    f"dist={fdist:.4f}m tol={final_tol:.4f}m"
                )
                if fdist <= final_tol:
                    self.get_logger().info(
                        "[PLAN_TO_POSE][MOVEIT_DIRECT] feedback_hang_recovered_final "
                        "(robot at target post-retry, feedback lost) — success"
                    )
                    return PlanToPoseResult(
                        success=True,
                        reason=(
                            f"feedback_hang_recovered_final"
                            f"|orig={reason}|dist={fdist:.4f}m"
                        ),
                        final_xyz=goal.target_xyz,
                        final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                        duration_sec=time.monotonic() - start_mono,
                        attempts=2,
                    )

        self.get_logger().warning(
            f"[PLAN_TO_POSE][MOVEIT_DIRECT] failed reason={reason}"
        )
        return PlanToPoseResult(
            success=ok,
            reason=reason,
            final_xyz=goal.target_xyz,
            final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
            duration_sec=time.monotonic() - start_mono,
            attempts=1,
        )

    def _on_joint_state(self, msg) -> None:
        """Cachea el último JointState para usar como seed IK."""
        with self._joint_state_lock:
            self._latest_joint_state = msg

    def _get_latest_joint_state(self):
        """Devuelve copia del último JointState o None."""
        with self._joint_state_lock:
            return self._latest_joint_state

    def _execute_fjt_direct(self, goal: PlanToPoseGoal, start_mono: float) -> Optional["PlanToPoseResult"]:
        """F1.24 / H9 LIVE (2026-05-08): bypass MoveIt en APPROACH.

        Flujo:
        1. Lee joint_state actual (seed IK).
        2. Llama /compute_ik para obtener target joints de la pose XYZ.
        3. Construye JointTrajectory de 2 puntos (current → target).
        4. Envía al /joint_trajectory_controller/follow_joint_trajectory action
           directamente (sin pasar por simple_controller_manager → evita bug).
        5. Espera "Goal reached, success!" del controller.

        Returns:
            PlanToPoseResult si se ejecutó (success o failure),
            None si pre-condiciones no satisfechas (caller debe fallback a MoveIt).
        """
        from .fjt_direct_helpers import (
            build_fjt_trajectory_multi_point,
            build_fjt_trajectory_two_point,
            build_ik_request,
            parse_ik_result,
        )
        from .plan_to_pose_logic import normalize_quat
        from .plan_to_pose_moveit_direct import classify_phase_by_target_z
        import math as _math

        # F1.24 LIVE (2026-05-08): tuning per-distancia para FJT_DIRECT.
        # Calcula distancia desde current pose (seed) al target XYZ. Si > 0.4m
        # → trayectoria larga (TRANSPORT/destino lejos): duration=25s, timeout=120s
        #   + multi-waypoint trajectory (H11) para evitar path_tolerance fail.
        # Si <= 0.4m → fase corta (APPROACH/GRASP_DOWN/LIFT): duration=8s, timeout=30s
        #   + 2-point trajectory simple.
        # El criterio basado en target Z (classify_phase_by_target_z) NO funciona
        # cuando el target está alto (e.g. cesta a Z=0.25 base): falsamente
        # clasifica como "OTHER" pese a la distancia grande.
        try:
            cur_x, cur_y, cur_z = (
                float(seed_positions[0]) * 0,  # placeholder; usaremos lookup TF
                0.0, 0.0,
            )
            # Mejor: usar TF lookup del ee_frame actual (ya tenemos el helper).
            tf_pos = self._lookup_ee_position_in_base(
                ee_frame=ee_frame,
                base_frame=self._bridge_base_frame,
                timeout_sec=0.5,
            )
            if tf_pos is not None:
                dx = float(tf_pos[0]) - float(goal.target_xyz[0])
                dy = float(tf_pos[1]) - float(goal.target_xyz[1])
                dz = float(tf_pos[2]) - float(goal.target_xyz[2])
                dist_to_target = _math.sqrt(dx * dx + dy * dy + dz * dz)
            else:
                dist_to_target = 0.5  # fallback conservador → larga
        except Exception:
            dist_to_target = 0.5

        is_long_traj = dist_to_target > 0.4
        if is_long_traj:
            fjt_duration_eff = max(self._fjt_direct_duration, 25.0)
            fjt_result_timeout_eff = max(self._fjt_direct_result_timeout, 120.0)
        else:
            fjt_duration_eff = max(float(self._fjt_direct_duration), 8.0)
            fjt_result_timeout_eff = max(float(self._fjt_direct_result_timeout), 30.0)
        self.get_logger().info(
            f"[PLAN_TO_POSE][FJT_DIRECT] dist_to_target={dist_to_target:.3f}m "
            f"duration={fjt_duration_eff:.1f}s timeout={fjt_result_timeout_eff:.1f}s "
            f"multi_waypoint={is_long_traj}"
        )
        # 1. Joint state actual.
        js = self._get_latest_joint_state()
        if js is None or not getattr(js, "name", None) or not getattr(js, "position", None):
            self.get_logger().warning(
                "[PLAN_TO_POSE][FJT_DIRECT] no joint_state cached — fallback a MoveIt"
            )
            return None

        # Filtrar solo joints del UR5 arm (en el orden esperado por el controller).
        ur5_joints = (
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        )
        name_to_pos = {str(n): float(p) for n, p in zip(js.name, js.position)}
        seed_positions = []
        for jn in ur5_joints:
            if jn not in name_to_pos:
                self.get_logger().warning(
                    f"[PLAN_TO_POSE][FJT_DIRECT] joint {jn} no en /joint_states — fallback"
                )
                return None
            seed_positions.append(name_to_pos[jn])

        # 2. Cliente /compute_ik (lazy).
        if self._fjt_direct_ik_client is None:
            try:
                from moveit_msgs.srv import GetPositionIK
                self._fjt_direct_ik_client = self.create_client(
                    GetPositionIK, self._fjt_direct_ik_service
                )
            except Exception as exc:
                self.get_logger().warning(
                    f"[PLAN_TO_POSE][FJT_DIRECT] cliente IK no disponible: {exc}"
                )
                return None
        if not self._fjt_direct_ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                "[PLAN_TO_POSE][FJT_DIRECT] /compute_ik service no ready — fallback"
            )
            return None

        ee_frame = (
            self._moveit_tip_link_override
            if self._moveit_tip_link_override
            else goal.ee_frame
        )
        ik_req = build_ik_request(
            target_xyz=goal.target_xyz,
            target_quat_xyzw=goal.target_quat_xyzw,
            ee_frame=ee_frame,
            base_frame=self._bridge_base_frame,
            group_name=self._moveit_group_name,
            current_joints=seed_positions,
            joint_names=list(ur5_joints),
            timeout_sec=self._fjt_direct_ik_timeout,
            avoid_collisions=False,
        )
        ik_future = self._fjt_direct_ik_client.call_async(ik_req)
        ik_event = threading.Event()
        ik_future.add_done_callback(lambda _f: ik_event.set())
        ik_event.wait(timeout=float(self._fjt_direct_ik_timeout) + 1.0)
        if not ik_future.done():
            self.get_logger().warning(
                "[PLAN_TO_POSE][FJT_DIRECT] IK timeout — fallback a MoveIt"
            )
            return None
        ok_ik, reason_ik, target_joints = parse_ik_result(
            ik_future.result(), list(ur5_joints)
        )
        if not ok_ik or target_joints is None:
            self.get_logger().info(
                f"[PLAN_TO_POSE][FJT_DIRECT] IK failed: {reason_ik} — fallback a MoveIt"
            )
            return None

        self.get_logger().info(
            f"[PLAN_TO_POSE][FJT_DIRECT] IK OK joints="
            f"({target_joints[0]:+.3f},{target_joints[1]:+.3f},"
            f"{target_joints[2]:+.3f},{target_joints[3]:+.3f},"
            f"{target_joints[4]:+.3f},{target_joints[5]:+.3f}) "
            f"target=({goal.target_xyz[0]:.3f},{goal.target_xyz[1]:.3f},"
            f"{goal.target_xyz[2]:.3f})"
        )

        # 3. Build trajectory (per-distance F1.24 LIVE).
        # H11: multi-waypoint para distancias largas (TRANSPORT) — evita
        # path_tolerance_violation por aceleración brusca con solo 2 puntos.
        if is_long_traj:
            jt = build_fjt_trajectory_multi_point(
                joint_names=list(ur5_joints),
                start_positions=seed_positions,
                target_positions=target_joints,
                num_intermediate_points=8,
                total_duration_sec=fjt_duration_eff,
            )
        else:
            jt = build_fjt_trajectory_two_point(
                joint_names=list(ur5_joints),
                start_positions=seed_positions,
                target_positions=target_joints,
                duration_sec=fjt_duration_eff,
            )

        # 4. FJT action client (lazy).
        if self._fjt_direct_action_client is None:
            try:
                from rclpy.action import ActionClient as _ActionClient
                from control_msgs.action import FollowJointTrajectory
                self._fjt_direct_action_client = _ActionClient(
                    self,
                    FollowJointTrajectory,
                    self._fjt_direct_action_name,
                )
            except Exception as exc:
                self.get_logger().warning(
                    f"[PLAN_TO_POSE][FJT_DIRECT] FJT action client error: {exc}"
                )
                return None
        if not self._fjt_direct_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning(
                f"[PLAN_TO_POSE][FJT_DIRECT] FJT action server "
                f"{self._fjt_direct_action_name} no ready — fallback"
            )
            return None

        from control_msgs.action import FollowJointTrajectory as _FJT
        fjt_goal = _FJT.Goal()
        fjt_goal.trajectory = jt
        # F1.24 H14 (2026-05-08): path_tolerance generoso para absorber
        # tracking errors transitorios. 0.0 = no enviar (default controller).
        if self._fjt_direct_path_tolerance_rad > 0.0:
            from .fjt_direct_helpers import build_fjt_path_tolerances
            fjt_goal.path_tolerance = build_fjt_path_tolerances(
                joint_names=list(ur5_joints),
                position_tolerance_rad=self._fjt_direct_path_tolerance_rad,
            )

        send_future = self._fjt_direct_action_client.send_goal_async(fjt_goal)
        send_event = threading.Event()
        send_future.add_done_callback(lambda _f: send_event.set())
        send_event.wait(timeout=3.0)
        if not send_future.done():
            self.get_logger().warning(
                "[PLAN_TO_POSE][FJT_DIRECT] FJT goal send timeout — fallback"
            )
            return None

        gh = send_future.result()
        if gh is None or not getattr(gh, "accepted", False):
            self.get_logger().warning(
                "[PLAN_TO_POSE][FJT_DIRECT] FJT goal rejected — fallback"
            )
            return None

        # 5. Esperar result (timeout per-fase F1.24 LIVE).
        result_future = gh.get_result_async()
        result_event = threading.Event()
        result_future.add_done_callback(lambda _f: result_event.set())
        result_event.wait(timeout=fjt_result_timeout_eff)
        if not result_future.done():
            self.get_logger().warning(
                "[PLAN_TO_POSE][FJT_DIRECT] FJT result timeout — fallback"
            )
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
            return None

        wrapper = result_future.result()
        ec = getattr(getattr(wrapper, "result", None), "error_code", 0)
        ec_val = int(ec) if isinstance(ec, int) else int(getattr(ec, "val", 0)) if hasattr(ec, "val") else 0
        # 0 = SUCCESSFUL en FollowJointTrajectory error_code.
        if ec_val == 0:
            self.get_logger().info(
                "[PLAN_TO_POSE][FJT_DIRECT] success (bypass MoveIt OK)"
            )
            return PlanToPoseResult(
                success=True,
                reason="fjt_direct:SUCCESSFUL",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=1,
            )
        # FJT error code != 0 (e.g. -1 INVALID_GOAL, -3 PATH_TOLERANCE_VIOLATED).
        self.get_logger().warning(
            f"[PLAN_TO_POSE][FJT_DIRECT] FJT error_code={ec_val} — fallback a MoveIt"
        )
        return None

    def _restart_joint_trajectory_controller(self, *, timeout_sec: float = 3.0) -> bool:
        """F1.23 LIVE (2026-05-08): deactivate + activate joint_trajectory_controller.

        Mitigación del bug BUG_CONTROLLER_FEEDBACK_HANG cuando retry también
        falla con FIRST_ATTEMPT_TIMEOUT. Hipótesis: el controller queda en
        un estado "ghost-busy" tras un cancel mal procesado por el bridge
        simple_controller_manager. El restart fuerza una transición de
        estado limpia.

        Returns:
            True si deactivate + activate fueron OK; False en cualquier fail.
        """
        if SwitchController is None or self._switch_controller_client is None:
            self.get_logger().warning(
                "[PLAN_TO_POSE] switch_controller no disponible; skip restart"
            )
            return False
        if not self._switch_controller_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                "[PLAN_TO_POSE] switch_controller service not ready"
            )
            return False
        # Step 1: deactivate
        deactivate_req = SwitchController.Request()
        deactivate_req.deactivate_controllers = ["joint_trajectory_controller"]
        deactivate_req.activate_controllers = []
        deactivate_req.strictness = SwitchController.Request.BEST_EFFORT
        deactivate_req.activate_asap = False
        try:
            future_d = self._switch_controller_client.call_async(deactivate_req)
            event_d = threading.Event()
            future_d.add_done_callback(lambda _f: event_d.set())
            event_d.wait(timeout=float(timeout_sec))
            if not future_d.done():
                self.get_logger().warning("[PLAN_TO_POSE] deactivate timeout")
                return False
            resp_d = future_d.result()
            if resp_d is None or not bool(getattr(resp_d, "ok", False)):
                self.get_logger().warning(
                    f"[PLAN_TO_POSE] deactivate failed: ok=False "
                    f"msg={getattr(resp_d, 'message', '?')}"
                )
                # Continuar igual al activate (BEST_EFFORT)
        except Exception as exc:
            self.get_logger().warning(
                f"[PLAN_TO_POSE] deactivate exception: {type(exc).__name__}: {exc}"
            )
            return False
        # Pause breve para que el controller release recursos
        time.sleep(1.0)
        # Step 2: activate
        activate_req = SwitchController.Request()
        activate_req.deactivate_controllers = []
        activate_req.activate_controllers = ["joint_trajectory_controller"]
        activate_req.strictness = SwitchController.Request.BEST_EFFORT
        activate_req.activate_asap = True
        try:
            future_a = self._switch_controller_client.call_async(activate_req)
            event_a = threading.Event()
            future_a.add_done_callback(lambda _f: event_a.set())
            event_a.wait(timeout=float(timeout_sec))
            if not future_a.done():
                self.get_logger().warning("[PLAN_TO_POSE] activate timeout")
                return False
            resp_a = future_a.result()
            if resp_a is None or not bool(getattr(resp_a, "ok", False)):
                self.get_logger().warning(
                    f"[PLAN_TO_POSE] activate failed: ok=False "
                    f"msg={getattr(resp_a, 'message', '?')}"
                )
                return False
            self.get_logger().info(
                "[PLAN_TO_POSE] joint_trajectory_controller restarted OK"
            )
            return True
        except Exception as exc:
            self.get_logger().warning(
                f"[PLAN_TO_POSE] activate exception: {type(exc).__name__}: {exc}"
            )
            return False

    def _lookup_ee_position_in_base(
        self,
        *,
        ee_frame: str,
        base_frame: str,
        timeout_sec: float = 1.0,
    ):
        """F1.22 LIVE (2026-05-08): lookup posición ee_frame en base_frame.

        Usado tras FIRST_ATTEMPT_TIMEOUT para detectar el bug
        BUG_CONTROLLER_FEEDBACK_HANG: el robot llegó al target pero el
        controller feedback no llegó al move_group.

        Returns:
            Tuple[float, float, float] con (x, y, z) en base_frame, o None
            si TF lookup falla.
        """
        try:
            ts = self._tf_buffer.lookup_transform(
                str(base_frame),
                str(ee_frame),
                rclpy.time.Time(),
                timeout=Duration(seconds=float(timeout_sec)),
            )
            tx = float(ts.transform.translation.x)
            ty = float(ts.transform.translation.y)
            tz = float(ts.transform.translation.z)
            return (tx, ty, tz)
        except Exception:
            return None

    def _on_bridge_result(self, msg) -> None:
        """Callback de /desired_grasp/result. Despierta el ejecutor si UUID OK."""
        text = str(getattr(msg, "data", "") or "")
        if not text:
            return
        with self._bridge_result_lock:
            expected = self._bridge_pending_uuid
            if expected is None:
                return  # no hay request pendiente
            if not result_matches_request(text, expected):
                # Result de otra request (concurrencia). Ignorar.
                return
            self._bridge_pending_text = text
        self._bridge_pending_event.set()

    # ------------------------------------------------------------------

    def _publish_feedback(self, goal_handle, fb):
        msg = PlanToPose.Feedback()
        msg.current_state = str(fb.current_state)
        msg.progress = float(fb.progress)
        msg.attempts = int(fb.attempts)
        msg.detail = str(fb.detail)
        try:
            goal_handle.publish_feedback(msg)
        except Exception as exc:
            self.get_logger().warning(
                f"[PLAN_TO_POSE] feedback publish failed: {exc}"
            )


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = PlanToPoseServer()
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
