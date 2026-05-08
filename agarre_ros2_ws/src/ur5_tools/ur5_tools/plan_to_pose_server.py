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
        self.declare_parameter("action_name", "/orchestrator/plan_to_pose")
        self.declare_parameter("step_delay_sec", 0.10)
        self.declare_parameter("planning_steps", 3)
        self.declare_parameter("executing_steps", 4)
        # F6.6: opt-in real bridge wiring.
        self.declare_parameter("use_real_bridge", False)
        self.declare_parameter("bridge_pose_topic", "/desired_grasp")
        self.declare_parameter("bridge_result_topic", "/desired_grasp/result")
        self.declare_parameter("bridge_base_frame", "base_link")
        self.declare_parameter("bridge_result_timeout_sec", 60.0)
        # B-iter3 (2026-05-03): modo MOVEIT_DIRECT — bypassa el bridge al
        # panel y llama directamente a /move_action de MoveIt 2.
        # Valores: "STUB", "REAL_BRIDGE", "MOVEIT_DIRECT".
        # Backwards compat: si mode="" (default) y use_real_bridge=true,
        # mode efectivo = "REAL_BRIDGE"; si use_real_bridge=false, "STUB".
        self.declare_parameter("mode", "")
        self.declare_parameter("moveit_action_name", "/move_action")
        self.declare_parameter("moveit_group_name", "manipulator")
        # B-iter14 (2026-05-03): forzar el tip_link real del SRDF para
        # las constraints de MoveIt. El ee_frame del goal (semántico)
        # puede ser distinto (rg2_pinch_center) pero MoveIt requiere que
        # las constraints apunten a un link del kinematic chain del group.
        self.declare_parameter("moveit_tip_link_override", "rg2_tcp")
        self.declare_parameter("moveit_planner_id", "")
        # 2026-05-07: subido planning_time 5→15 y result_timeout 30→60.
        # Validación live ronda 9 mostró APPROACH timeout sistemático con 5s
        # cuando el robot empieza desde pose post HOME_INITIAL no canónica.
        # 15s es generoso pero válido para una sesión live; el orchestrator
        # ya espera el result_timeout completo antes de fallar.
        self.declare_parameter("moveit_planning_time_sec", 15.0)
        self.declare_parameter("moveit_position_tol_m", 0.005)
        self.declare_parameter("moveit_orientation_tol_rad", 0.05)
        self.declare_parameter("moveit_result_timeout_sec", 60.0)

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

        # B-iter3: resolución del modo efectivo.
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
        # ActionClient para /move_action (lazy init en _execute_moveit_direct).
        self._moveit_action_client = None

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

        # F6.6: bridge wiring (publisher + subscription + correlation).
        # Sólo se inicializa si use_real_bridge=true (evita topics extra
        # innecesarios en modo stub).
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
        _MOVEIT_FIRST_ATTEMPT_TIMEOUT_SEC = 120.0
        first_attempt_timeout = min(
            _MOVEIT_FIRST_ATTEMPT_TIMEOUT_SEC,
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
                    retry_result_event.wait(
                        timeout=float(max(1.0, self._moveit_result_timeout))
                    )
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
                reason = f"{reason}|retry_send_timeout"

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
