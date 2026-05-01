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

from ur5_panel_interfaces.action import PlanToPose

from .plan_to_pose_logic import (
    PlanToPoseGoal,
    execute_stub,
    feedback_sequence,
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

        self._action_name = str(
            self.get_parameter("action_name").value or "/orchestrator/plan_to_pose"
        ).strip()
        self._step_delay = float(self.get_parameter("step_delay_sec").value)
        self._planning_steps = int(self.get_parameter("planning_steps").value)
        self._executing_steps = int(self.get_parameter("executing_steps").value)

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
        self.get_logger().info(
            f"[PLAN_TO_POSE] ready, action={self._action_name} "
            f"planning={self._planning_steps} executing={self._executing_steps} "
            f"step_delay={self._step_delay}s [STUB — no real planner yet]"
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

        # Ejecutar (stub).
        duration = time.monotonic() - start_mono
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
