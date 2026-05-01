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
from typing import Optional

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

        if self._use_real_bridge:
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

        mode = "REAL_BRIDGE" if self._use_real_bridge else "STUB"
        self.get_logger().info(
            f"[PLAN_TO_POSE] ready, action={self._action_name} mode={mode} "
            f"planning={self._planning_steps} executing={self._executing_steps} "
            f"step_delay={self._step_delay}s"
        )
        if self._use_real_bridge:
            self.get_logger().info(
                f"[PLAN_TO_POSE] bridge pose_topic={self._bridge_pose_topic} "
                f"result_topic={self._bridge_result_topic} "
                f"base_frame={self._bridge_base_frame} "
                f"result_timeout={self._bridge_result_timeout:.1f}s"
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

        # Ejecutar:
        # - Si use_real_bridge=true: publica PoseStamped a /desired_grasp
        #   con frame_id codificado y espera result correlando UUID.
        # - Si false: stub de la lógica pura.
        duration = time.monotonic() - start_mono
        if self._use_real_bridge:
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
