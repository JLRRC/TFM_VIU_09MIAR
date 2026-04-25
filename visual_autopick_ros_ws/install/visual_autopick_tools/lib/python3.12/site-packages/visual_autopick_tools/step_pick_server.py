#!/usr/bin/env python3
"""Step pick server — executes pick phases on command.

Subscribe: /visual_autopick/step_command  (std_msgs/String)
  Format:  MODE:PHASE
  Example: DIRECTO:HOME  |  MOVEIT:APPROACH  |  DIRECTO:ABORT

Publish:  /visual_autopick/status  (std_msgs/String)
  Format:  MODE:PHASE:STATE  or  MODE:IDLE

Supported phases: HOME | APPROACH | DOWN | CLOSE | LIFT | RESET | ABORT
Supported modes:  DIRECTO | MOVEIT
"""
from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

# Object that the visual_autopick stack picks up.
_PICK_OBJECT = "pick_demo"
# Gripper attach/detach topic prefix (must match gripper_attach_backend config).
_GRIPPER_PREFIX = "/gripper"

# ── Pre-computed joint configs ────────────────────────────────────────────────
_ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
_HOME_ANGLES     = [0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0]
_APPROACH_ANGLES = [-0.18, -1.55, 1.48, -1.52, -1.57, 0.0]
_DOWN_ANGLES     = [-0.18, -1.35, 1.70, -1.95, -1.57, 0.0]
_LIFT_ANGLES     = [-0.18, -1.55, 1.48, -1.52, -1.57, 0.0]
_GRIPPER_OPEN    = [0.0, 0.0]
_GRIPPER_CLOSE   = [0.4, 0.4]

_PHASE_ORDER = ["HOME", "APPROACH", "DOWN", "CLOSE", "LIFT"]


def _make_arm_traj(angles: list, duration: float) -> JointTrajectory:
    traj = JointTrajectory()
    traj.joint_names = _ARM_JOINTS
    pt = JointTrajectoryPoint()
    pt.positions = list(angles)
    pt.velocities = [0.0] * len(angles)
    pt.time_from_start.sec = int(duration)
    pt.time_from_start.nanosec = int((duration % 1.0) * 1e9)
    traj.points = [pt]
    return traj


class StepPickServer(Node):
    def __init__(self) -> None:
        super().__init__("step_pick_server")

        self._arm_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory", 10
        )
        self._gripper_pub = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )
        self._attach_pub = self.create_publisher(
            Empty, f"{_GRIPPER_PREFIX}/{_PICK_OBJECT}/attach", 10
        )
        self._detach_pub = self.create_publisher(
            Empty, f"{_GRIPPER_PREFIX}/{_PICK_OBJECT}/detach", 10
        )
        self._status_pub = self.create_publisher(
            String, "/visual_autopick/status", 10
        )
        self._cmd_sub = self.create_subscription(
            String, "/visual_autopick/step_command", self._on_command, 10
        )

        self._current_mode = "DIRECTO"
        self._current_phase_idx = -1  # -1 = IDLE
        self._busy = False
        self._lock = threading.Lock()

        self._status_timer = self.create_timer(0.5, self._publish_status_periodic)

        # Enviar HOME automáticamente al arrancar, una sola vez,
        # con delay para dar tiempo a que los spawners activen los controllers.
        self._startup_home_done = False
        self._startup_timer = self.create_timer(10.0, self._startup_home)
        self.get_logger().info("StepPickServer ready. Listening on /visual_autopick/step_command")

    # ── Startup HOME ─────────────────────────────────────────────────────────

    def _startup_home(self) -> None:
        if self._startup_home_done:
            return
        self._startup_home_done = True
        self._startup_timer.cancel()
        self.get_logger().info("[STARTUP] Enviando robot a HOME")
        self._arm_pub.publish(_make_arm_traj(_HOME_ANGLES, duration=5.0))
        self._publish_status("DIRECTO:HOME:STARTUP")

    # ── Status ──────────────────────────────────────────────────────────────

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(f"[STATUS] {text}")

    def _publish_status_periodic(self) -> None:
        with self._lock:
            if self._busy:
                return
            if self._current_phase_idx < 0:
                status = f"{self._current_mode}:IDLE"
            elif self._current_phase_idx >= len(_PHASE_ORDER):
                status = f"{self._current_mode}:DONE"
            else:
                phase = _PHASE_ORDER[self._current_phase_idx]
                status = f"{self._current_mode}:{phase}:READY"
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    # ── Command callback ─────────────────────────────────────────────────────

    def _on_command(self, msg: String) -> None:
        data = msg.data.strip().upper()
        self.get_logger().info(f"[CMD] {data}")

        parts = data.split(":", 1)
        if len(parts) != 2:
            self.get_logger().warn(f"Invalid command format: '{data}'. Expected MODE:PHASE")
            return

        mode, phase = parts[0], parts[1]
        if mode not in ("DIRECTO", "MOVEIT"):
            self.get_logger().warn(f"Unknown mode: {mode}")
            return

        with self._lock:
            if self._busy and phase not in ("ABORT", "RESET"):
                self.get_logger().warn("Busy executing previous command; ignoring.")
                return

        thread = threading.Thread(
            target=self._execute_phase, args=(mode, phase), daemon=True
        )
        thread.start()

    # ── Phase execution ──────────────────────────────────────────────────────

    def _execute_phase(self, mode: str, phase: str) -> None:
        with self._lock:
            self._busy = True
            self._current_mode = mode

        self._publish_status(f"{mode}:{phase}:RUNNING")

        try:
            if mode == "DIRECTO":
                self._execute_direct(phase, mode)
            else:
                self._execute_moveit(phase, mode)
        except Exception as e:
            self._publish_status(f"{mode}:{phase}:ERROR")
            self.get_logger().error(f"Error in {mode}:{phase}: {e}")
        finally:
            with self._lock:
                self._busy = False

    def _execute_direct(self, phase: str, mode: str) -> None:
        if phase == "HOME":
            self._send_arm(_HOME_ANGLES, 4.0)
            self._send_gripper(_GRIPPER_OPEN)
            time.sleep(4.5)
            with self._lock:
                self._current_phase_idx = 0
            self._publish_status(f"{mode}:HOME:DONE")

        elif phase == "APPROACH":
            self._send_arm(_APPROACH_ANGLES, 3.5)
            time.sleep(4.0)
            with self._lock:
                self._current_phase_idx = 1
            self._publish_status(f"{mode}:APPROACH:DONE")

        elif phase == "DOWN":
            self._send_arm(_DOWN_ANGLES, 2.0)
            time.sleep(2.5)
            with self._lock:
                self._current_phase_idx = 2
            self._publish_status(f"{mode}:DOWN:DONE")

        elif phase == "CLOSE":
            self._send_gripper(_GRIPPER_CLOSE)
            time.sleep(1.0)
            # Trigger Gazebo attach so the object is held by the gripper.
            self._attach_pub.publish(Empty())
            time.sleep(0.8)
            with self._lock:
                self._current_phase_idx = 3
            self._publish_status(f"{mode}:CLOSE:DONE")

        elif phase == "LIFT":
            self._send_arm(_LIFT_ANGLES, 2.5)
            time.sleep(3.0)
            with self._lock:
                self._current_phase_idx = 4
            self._publish_status(f"{mode}:LIFT:DONE")

        elif phase in ("RESET", "ABORT"):
            self._send_arm(_HOME_ANGLES, 4.0)
            self._send_gripper(_GRIPPER_OPEN)
            self._detach_pub.publish(Empty())
            time.sleep(4.5)
            with self._lock:
                self._current_phase_idx = -1
            self._publish_status(f"{mode}:IDLE")

        elif phase == "NEXT":
            with self._lock:
                next_idx = self._current_phase_idx + 1
            if 0 <= next_idx < len(_PHASE_ORDER):
                next_phase = _PHASE_ORDER[next_idx]
                self._execute_direct(next_phase, mode)
            else:
                self.get_logger().info("No next phase (sequence complete or not started).")

        else:
            self.get_logger().warn(f"Unknown phase: {phase}")
            self._publish_status(f"{mode}:{phase}:UNKNOWN")

    def _execute_moveit(self, phase: str, mode: str) -> None:
        # MoveIt execution delegates to direct joint control as fallback
        # (full MoveIt requires move_group to be running).
        # When move_group is available, simple_moveit_pick can be called here.
        self.get_logger().info(f"[MOVEIT] phase={phase} — using joint fallback (move_group not integrated in server yet)")
        self._execute_direct(phase, mode)

    # ── Hardware publishers ──────────────────────────────────────────────────

    def _send_arm(self, angles: list, duration: float) -> None:
        traj = _make_arm_traj(angles, duration)
        self._arm_pub.publish(traj)

    def _send_gripper(self, positions: list) -> None:
        msg = Float64MultiArray()
        msg.data = list(positions)
        self._gripper_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StepPickServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    node.destroy_node()
    try:
        rclpy.try_shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
