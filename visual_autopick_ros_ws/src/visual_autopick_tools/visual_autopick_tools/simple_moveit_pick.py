#!/usr/bin/env python3
"""Simple MoveIt pick executor.

Uses moveit_py to plan and execute arm phases.
Phases: HOME | APPROACH | DOWN | CLOSE | LIFT | RESET | ABORT
"""
from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

try:
    from moveit.core.robot_state import RobotState
    from moveit.planning import MoveItPy, PlanningComponent
    _MOVEIT_OK = True
except ImportError:
    _MOVEIT_OK = False

# Gripper commands
_GRIPPER_OPEN  = [0.0, 0.0]
_GRIPPER_CLOSE = [0.4, 0.4]

# Pre-grasp pose in world frame (over the object)
_PREGRASP_X, _PREGRASP_Y, _PREGRASP_Z = -0.42, 0.00, 1.03
_GRASP_X,    _GRASP_Y,    _GRASP_Z    = -0.42, 0.00, 0.895
_LIFT_X,     _LIFT_Y,     _LIFT_Z     = -0.42, 0.00, 1.10


def _pose_msg(x: float, y: float, z: float):
    from geometry_msgs.msg import Pose
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    # Gripper pointing down: rotate 180° around Y
    p.orientation.x = 0.0
    p.orientation.y = 1.0
    p.orientation.z = 0.0
    p.orientation.w = 0.0
    return p


class SimpleMoveItPick(Node):
    """Execute pick phases via MoveItPy."""

    def __init__(self) -> None:
        super().__init__("simple_moveit_pick")

        self._gripper_pub = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )
        self._status_pub = self.create_publisher(String, "/visual_autopick/status", 10)

        self._moveit: Optional[object] = None
        self._arm: Optional[object] = None

        if _MOVEIT_OK:
            try:
                self._moveit = MoveItPy(node_name="simple_moveit_pick_mv")
                self._arm = self._moveit.get_planning_component("manipulator")
                self.get_logger().info("MoveItPy initialized.")
            except Exception as e:
                self.get_logger().error(f"MoveItPy init failed: {e}")
        else:
            self.get_logger().error("moveit_py not available. MOVEIT mode disabled.")

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(f"[STATUS] {text}")

    def _send_gripper(self, positions: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = positions
        self._gripper_pub.publish(msg)

    def _plan_and_execute(self, pose, mode: str, phase: str) -> bool:
        if self._arm is None:
            self.get_logger().error("MoveIt arm not initialized.")
            return False
        try:
            self._arm.set_start_state_to_current_state()
            self._arm.set_goal_state(pose_stamped_msg=pose, pose_link="rg2_tcp")
            plan_result = self._arm.plan()
            if not plan_result:
                self.get_logger().warn(f"[{mode}:{phase}] Planning failed.")
                return False
            robot_trajectory = plan_result.trajectory
            self._moveit.execute(robot_trajectory, controllers=[])
            return True
        except Exception as e:
            self.get_logger().error(f"[{mode}:{phase}] Execute error: {e}")
            return False

    def _go_home(self, mode: str) -> None:
        if self._arm is None:
            return
        try:
            self._arm.set_start_state_to_current_state()
            self._arm.set_goal_state(configuration_name="home")
            plan_result = self._arm.plan()
            if plan_result:
                self._moveit.execute(plan_result.trajectory, controllers=[])
        except Exception as e:
            self.get_logger().error(f"[{mode}:HOME] Error: {e}")

    def execute_phase(self, phase: str, mode: str = "MOVEIT") -> None:
        from geometry_msgs.msg import PoseStamped
        phase = phase.upper().strip()
        self._publish_status(f"{mode}:{phase}:RUNNING")

        try:
            if phase == "HOME":
                self._send_gripper(_GRIPPER_OPEN)
                if _MOVEIT_OK and self._arm:
                    self._go_home(mode)
                    time.sleep(1.0)
                self._publish_status(f"{mode}:{phase}:DONE")

            elif phase == "APPROACH":
                ps = PoseStamped()
                ps.header.frame_id = "world"
                ps.pose = _pose_msg(_PREGRASP_X, _PREGRASP_Y, _PREGRASP_Z)
                ok = self._plan_and_execute(ps, mode, phase)
                time.sleep(1.0)
                self._publish_status(f"{mode}:{phase}:{'DONE' if ok else 'FAILED'}")

            elif phase == "DOWN":
                ps = PoseStamped()
                ps.header.frame_id = "world"
                ps.pose = _pose_msg(_GRASP_X, _GRASP_Y, _GRASP_Z)
                ok = self._plan_and_execute(ps, mode, phase)
                time.sleep(1.0)
                self._publish_status(f"{mode}:{phase}:{'DONE' if ok else 'FAILED'}")

            elif phase == "CLOSE":
                self._send_gripper(_GRIPPER_CLOSE)
                time.sleep(1.5)
                self._publish_status(f"{mode}:{phase}:DONE")

            elif phase == "LIFT":
                ps = PoseStamped()
                ps.header.frame_id = "world"
                ps.pose = _pose_msg(_LIFT_X, _LIFT_Y, _LIFT_Z)
                ok = self._plan_and_execute(ps, mode, phase)
                time.sleep(1.0)
                self._publish_status(f"{mode}:{phase}:{'DONE' if ok else 'FAILED'}")

            elif phase in ("RESET", "ABORT"):
                self._send_gripper(_GRIPPER_OPEN)
                if _MOVEIT_OK and self._arm:
                    self._go_home(mode)
                time.sleep(1.0)
                self._publish_status(f"{mode}:IDLE")

            else:
                self.get_logger().warn(f"Unknown phase: {phase}")
                self._publish_status(f"{mode}:{phase}:UNKNOWN")

        except Exception as e:
            self._publish_status(f"{mode}:{phase}:ERROR:{e}")
            self.get_logger().error(f"Phase {phase} error: {e}")

    def run_full_sequence(self, mode: str = "MOVEIT") -> None:
        for phase in ("HOME", "APPROACH", "DOWN", "CLOSE", "LIFT"):
            self.execute_phase(phase, mode=mode)


def main(args=None) -> None:
    import argparse
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--phase", default=None)
    ap.add_argument("--full", action="store_true")
    parsed, ros_args = ap.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = SimpleMoveItPick()

    if parsed.phase:
        node.execute_phase(parsed.phase)
    elif parsed.full:
        node.run_full_sequence()
    else:
        try:
            rclpy.spin(node)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

    node.destroy_node()
    try:
        rclpy.try_shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
