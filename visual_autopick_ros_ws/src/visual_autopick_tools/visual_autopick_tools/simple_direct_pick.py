#!/usr/bin/env python3
"""Simple direct pick executor using joint trajectories (no MoveIt).

Phases: HOME | APPROACH | DOWN | CLOSE | LIFT | RESET

Joint positions are pre-computed for the fixed object pose at (-0.42, 0.00, 0.876)
with base_link at (-0.85, 0.0, 0.850) in world frame.
"""
from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Pre-computed joint configs for the known scene geometry.
# All angles in radians.
_ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Safe home position (upright, elbows in)
_HOME_ANGLES    = [0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0]
# Approach: over the object at ~15cm height
_APPROACH_ANGLES = [-0.18, -1.55, 1.48, -1.52, -1.57, 0.0]
# Down: gripper near object surface
_DOWN_ANGLES    = [-0.18, -1.35, 1.70, -1.95, -1.57, 0.0]
# Lift: same XY as approach but raised
_LIFT_ANGLES    = [-0.18, -1.55, 1.48, -1.52, -1.57, 0.0]

_GRIPPER_OPEN   = [0.0, 0.0]   # rg2_finger_joint1, rg2_finger_joint2
_GRIPPER_CLOSE  = [0.4, 0.4]   # partial close for cylinder radius 0.025m


def _make_arm_traj(angles: list[float], duration_sec: float) -> JointTrajectory:
    traj = JointTrajectory()
    traj.joint_names = _ARM_JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = angles
    pt.velocities = [0.0] * len(angles)
    pt.time_from_start.sec = int(duration_sec)
    pt.time_from_start.nanosec = int((duration_sec % 1.0) * 1e9)
    traj.points = [pt]
    return traj


class SimpleDirectPick(Node):
    """Execute arm phases via JointTrajectory and gripper via Float64MultiArray."""

    PHASES = ("HOME", "APPROACH", "DOWN", "CLOSE", "LIFT", "RESET", "ABORT")

    def __init__(self) -> None:
        super().__init__("simple_direct_pick")

        self._arm_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )
        self._gripper_pub = self.create_publisher(
            Float64MultiArray,
            "/gripper_controller/commands",
            10,
        )
        self._status_pub = self.create_publisher(String, "/visual_autopick/status", 10)

        self._use_attach = False  # VISUAL_AUTOPICK_USE_ATTACH env var

        import os
        if os.environ.get("VISUAL_AUTOPICK_USE_ATTACH", "0").strip() in ("1", "true", "yes"):
            self._use_attach = True

        self.get_logger().info("SimpleDirectPick ready.")

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(f"[STATUS] {text}")

    def _send_arm(self, angles: list[float], duration: float = 3.0) -> None:
        traj = _make_arm_traj(angles, duration)
        self._arm_pub.publish(traj)

    def _send_gripper(self, positions: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = positions
        self._gripper_pub.publish(msg)

    def execute_phase(self, phase: str, mode: str = "DIRECTO") -> None:
        phase = phase.upper().strip()
        self._publish_status(f"{mode}:{phase}:RUNNING")
        try:
            if phase == "HOME":
                self._send_arm(_HOME_ANGLES, duration=4.0)
                self._send_gripper(_GRIPPER_OPEN)
                time.sleep(4.5)
                self._publish_status(f"{mode}:{phase}:DONE")

            elif phase == "APPROACH":
                self._send_arm(_APPROACH_ANGLES, duration=3.5)
                time.sleep(4.0)
                self._publish_status(f"{mode}:{phase}:DONE")

            elif phase == "DOWN":
                self._send_arm(_DOWN_ANGLES, duration=2.0)
                time.sleep(2.5)
                self._publish_status(f"{mode}:{phase}:DONE")

            elif phase == "CLOSE":
                self._send_gripper(_GRIPPER_CLOSE)
                time.sleep(1.5)
                self._publish_status(f"{mode}:{phase}:DONE")

            elif phase == "LIFT":
                self._send_arm(_LIFT_ANGLES, duration=2.5)
                time.sleep(3.0)
                self._publish_status(f"{mode}:{phase}:DONE")

            elif phase in ("RESET", "ABORT"):
                self._send_arm(_HOME_ANGLES, duration=4.0)
                self._send_gripper(_GRIPPER_OPEN)
                time.sleep(4.5)
                self._publish_status(f"{mode}:IDLE")

            else:
                self.get_logger().warn(f"Unknown phase: {phase}")
                self._publish_status(f"{mode}:{phase}:UNKNOWN")
        except Exception as e:
            self._publish_status(f"{mode}:{phase}:ERROR:{e}")
            self.get_logger().error(f"Phase {phase} error: {e}")

    def run_full_sequence(self, mode: str = "DIRECTO") -> None:
        for phase in ("HOME", "APPROACH", "DOWN", "CLOSE", "LIFT"):
            self.execute_phase(phase, mode=mode)


def main(args=None) -> None:
    import argparse
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--phase", default=None)
    ap.add_argument("--full", action="store_true")
    parsed, ros_args = ap.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = SimpleDirectPick()

    if parsed.phase:
        # Run single phase then exit
        node.execute_phase(parsed.phase)
    elif parsed.full:
        node.run_full_sequence()
    else:
        # Stay alive waiting for external calls
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
