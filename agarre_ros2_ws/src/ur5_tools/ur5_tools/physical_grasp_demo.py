#!/usr/bin/env python3
"""Demo fisica minima de agarre para pick_demo.

El nodo expone ``/physical_grasp_demo/run`` (std_srvs/Trigger) y tambien
puede ejecutarse una vez con ``ros2 run ur5_tools physical_grasp_demo``.
No usa set_pose ni attach: mueve el TCP con MoveIt, cierra la RG2 por su
controller y comprueba la pose del objeto en Gazebo.
"""
from __future__ import annotations

import math
import os
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


_PREFIX = "[AGARRE_MOVEIT_SIMPLE]"
_UR5_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
_GRIPPER_JOINTS = ("rg2_finger_joint1", "rg2_finger_joint2")


@dataclass(frozen=True)
class DemoPose:
    """Pose target del TCP en world."""

    x: float
    y: float
    z: float


def _q_mult(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = (float(v) for v in a)
    bx, by, bz, bw = (float(v) for v in b)
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _q_conj(q: Sequence[float]) -> Tuple[float, float, float, float]:
    return (-float(q[0]), -float(q[1]), -float(q[2]), float(q[3]))


def _q_norm(q: Sequence[float]) -> Tuple[float, float, float, float]:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def _rotate_vec(q: Sequence[float], p: Sequence[float]) -> Tuple[float, float, float]:
    qn = _q_norm(q)
    pq = (float(p[0]), float(p[1]), float(p[2]), 0.0)
    out = _q_mult(_q_mult(qn, pq), _q_conj(qn))
    return (out[0], out[1], out[2])


def _transform_point(tf: TransformStamped, xyz: Sequence[float]) -> Tuple[float, float, float]:
    t = tf.transform.translation
    r = tf.transform.rotation
    q = (r.x, r.y, r.z, r.w)
    rx, ry, rz = _rotate_vec(q, xyz)
    return (rx + float(t.x), ry + float(t.y), rz + float(t.z))


class PhysicalGraspDemo(Node):
    """Backend del flujo Mesa -> MoveIt -> RG2 close -> lift."""

    def __init__(self) -> None:
        super().__init__("physical_grasp_demo")
        self._cb_group = ReentrantCallbackGroup()
        self._declare_params()
        self._read_params()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._moveit_client = None
        self._fjt_client = None
        self._ik_client = None
        self._joint_state_lock = threading.Lock()
        self._latest_joint_state: Optional[JointState] = None
        self._run_lock = threading.Lock()
        self._events: list[str] = []
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
            callback_group=self._cb_group,
        )
        self._gripper_pub = self.create_publisher(
            Float64MultiArray,
            self._gripper_cmd_topic,
            10,
        )
        self.create_service(
            Trigger,
            "/physical_grasp_demo/run",
            self._on_run_service,
            callback_group=self._cb_group,
        )
        self._log_event(
            f"{_PREFIX}[READY] service=/physical_grasp_demo/run "
            f"tcp={self._ee_frame} object=pick_demo"
        )

    def _declare_params(self) -> None:
        self.declare_parameter("autostart", True)
        self.declare_parameter("keep_alive", False)
        self.declare_parameter("object_x", -0.414)
        self.declare_parameter("object_y", 0.0)
        self.declare_parameter("object_z", 0.875)
        self.declare_parameter("pregrasp_offset_z", 0.12)
        self.declare_parameter("grasp_offset_z", 0.02)
        self.declare_parameter("lift_offset_z", 0.14)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "rg2_pinch_center")
        self.declare_parameter("moveit_action_name", "/move_action")
        self.declare_parameter("moveit_group_name", "manipulator")
        self.declare_parameter("moveit_planning_time_sec", 15.0)
        self.declare_parameter("moveit_result_timeout_sec", 120.0)
        self.declare_parameter("moveit_position_tol_m", 0.008)
        self.declare_parameter("moveit_orientation_tol_rad", 0.10)
        self.declare_parameter("gripper_cmd_topic", "/gripper_controller/commands")
        self.declare_parameter("gripper_open_m", 0.0425)
        self.declare_parameter("gripper_closed_m", 0.0)
        self.declare_parameter("gripper_joint2_sign", 1.0)
        self.declare_parameter("gripper_settle_sec", 1.2)
        self.declare_parameter(
            "fjt_action_name",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("compute_ik_service", "/compute_ik")

    def _read_params(self) -> None:
        self._object_world = DemoPose(
            float(self.get_parameter("object_x").value),
            float(self.get_parameter("object_y").value),
            float(self.get_parameter("object_z").value),
        )
        self._pregrasp = DemoPose(
            self._object_world.x,
            self._object_world.y,
            self._object_world.z + float(self.get_parameter("pregrasp_offset_z").value),
        )
        self._grasp = DemoPose(
            self._object_world.x,
            self._object_world.y,
            self._object_world.z + float(self.get_parameter("grasp_offset_z").value),
        )
        self._lift = DemoPose(
            self._object_world.x,
            self._object_world.y,
            self._object_world.z + float(self.get_parameter("lift_offset_z").value),
        )
        self._world_frame = str(self.get_parameter("world_frame").value or "world")
        self._base_frame = str(self.get_parameter("base_frame").value or "base_link")
        self._ee_frame = str(self.get_parameter("ee_frame").value or "rg2_pinch_center")
        self._moveit_action_name = str(
            self.get_parameter("moveit_action_name").value or "/move_action"
        )
        self._moveit_group_name = str(
            self.get_parameter("moveit_group_name").value or "manipulator"
        )
        self._moveit_planning_time = float(
            self.get_parameter("moveit_planning_time_sec").value
        )
        self._moveit_result_timeout = float(
            self.get_parameter("moveit_result_timeout_sec").value
        )
        self._moveit_position_tol = float(
            self.get_parameter("moveit_position_tol_m").value
        )
        self._moveit_orientation_tol = float(
            self.get_parameter("moveit_orientation_tol_rad").value
        )
        self._gripper_cmd_topic = str(
            self.get_parameter("gripper_cmd_topic").value
            or "/gripper_controller/commands"
        )
        self._gripper_open = float(self.get_parameter("gripper_open_m").value)
        self._gripper_closed = float(self.get_parameter("gripper_closed_m").value)
        self._gripper_joint2_sign = float(
            self.get_parameter("gripper_joint2_sign").value
        )
        self._gripper_settle_sec = float(self.get_parameter("gripper_settle_sec").value)
        self._fjt_action_name = str(self.get_parameter("fjt_action_name").value)
        self._compute_ik_service = str(self.get_parameter("compute_ik_service").value)

    def _on_joint_state(self, msg: JointState) -> None:
        with self._joint_state_lock:
            self._latest_joint_state = msg

    def _on_run_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        ok, message = self.run_demo()
        response.success = bool(ok)
        response.message = str(message)
        return response

    def _log_event(self, text: str) -> None:
        line = str(text)
        self._events.append(line)
        self.get_logger().info(line)

    def _fail(self, phase: str, reason: str) -> Tuple[bool, str]:
        self._log_event(f"{_PREFIX}[FAIL] phase={phase} reason={reason}")
        return False, "\n".join(self._events)

    def run_demo(self) -> Tuple[bool, str]:
        if not self._run_lock.acquire(blocking=False):
            return False, f"{_PREFIX}[FAIL] phase=START reason=already_running"
        self._events = []
        try:
            return self._run_demo_locked()
        finally:
            self._run_lock.release()

    def _run_demo_locked(self) -> Tuple[bool, str]:
        self._log_event(f"{_PREFIX}[BUTTON_BACKEND][START]")
        self._log_runtime_interfaces()

        ok, reason = self._verify_tf()
        if not ok:
            return self._fail("TF", reason)

        quat_base = self._current_tcp_quat_base()
        if quat_base is None:
            return self._fail("TF", "base_link_to_rg2_pinch_center_orientation_unavailable")
        self._log_event(
            f"{_PREFIX}[POSES] object_world=({self._object_world.x:.3f},"
            f"{self._object_world.y:.3f},{self._object_world.z:.3f}) "
            f"pregrasp_z={self._pregrasp.z:.3f} grasp_z={self._grasp.z:.3f} "
            f"lift_z={self._lift.z:.3f}"
        )

        before_z = self._capture_pose_info(
            "/tmp/physical_grasp_demo_pose_before.txt", "before"
        )
        self._command_gripper(self._gripper_closed, "GRIPPER_PREP_CLOSE")
        time.sleep(0.5)

        ok, reason = self._moveit_phase("PREGRASP", self._pregrasp, quat_base)
        if not ok:
            return self._fail("PREGRASP", reason)

        self._command_gripper(self._gripper_open, "GRIPPER_OPEN")
        time.sleep(0.5)

        ok, reason = self._moveit_phase("GRASP_DOWN", self._grasp, quat_base)
        if not ok:
            return self._fail("GRASP_DOWN", reason)

        ok, reason = self._close_gripper()
        if not ok:
            return self._fail("GRIPPER_CLOSE", reason)

        after_close_z = self._capture_pose_info(
            "/tmp/physical_grasp_demo_pose_after_close.txt", "after_close"
        )
        ok, reason = self._moveit_phase("LIFT", self._lift, quat_base)
        if not ok:
            self._log_event(f"{_PREFIX}[FALLBACK_FJT] phase=LIFT reason={reason}")
            ok, reason = self._fjt_lift(self._lift, quat_base)
            if not ok:
                return self._fail("LIFT", reason)

        after_lift_z = self._capture_pose_info(
            "/tmp/physical_grasp_demo_pose_after_lift.txt", "after_lift"
        )
        self._log_event(f"{_PREFIX}[HOLD] no_place=true no_cesta=true")
        if before_z is not None and after_lift_z is not None:
            dz = after_lift_z - before_z
            self._log_event(
                f"{_PREFIX}[OBJECT_LIFT] before_z={before_z:.4f} "
                f"after_close_z={after_close_z if after_close_z is not None else 'unknown'} "
                f"after_lift_z={after_lift_z:.4f} dz={dz:.4f}"
            )
            if dz < 0.05:
                self._log_event(
                    f"{_PREFIX}[DIAG] cause=object_not_lifted "
                    "candidates=contact/friction/gripper_alignment/collision"
                )
                return self._fail("PHYSICS", f"object_not_lifted dz={dz:.4f}")
        else:
            self._log_event(
                f"{_PREFIX}[OBJECT_LIFT] before_z={before_z} after_lift_z={after_lift_z}"
            )
        self._log_event(f"{_PREFIX}[DONE]")
        return True, "\n".join(self._events)

    def _log_runtime_interfaces(self) -> None:
        self._log_event(
            f"{_PREFIX}[GRIPPER_CONTROLLER] cmd_topic={self._gripper_cmd_topic} "
            f"joints={','.join(_GRIPPER_JOINTS)}"
        )
        try:
            out = subprocess.run(
                ["ros2", "control", "list_controllers"],
                text=True,
                capture_output=True,
                timeout=20.0,
                check=False,
            )
            line = " | ".join((out.stdout or out.stderr or "").splitlines()[:8])
            self._log_event(f"{_PREFIX}[ROS2_CONTROL] {line or 'no_output'}")
        except Exception as exc:
            self._log_event(f"{_PREFIX}[ROS2_CONTROL] unavailable reason={exc}")
        topics = [
            name for name, _types in self.get_topic_names_and_types()
            if any(k in name.lower() for k in ("gripper", "rg2", "controller"))
        ]
        actions = []
        try:
            actions = [
                name for name, _types in self.get_action_names_and_types()
                if any(k in name.lower() for k in ("gripper", "trajectory", "rg2"))
            ]
        except Exception:
            actions = []
        self._log_event(f"{_PREFIX}[TOPICS] candidates={topics[:12]}")
        self._log_event(f"{_PREFIX}[ACTIONS] candidates={actions[:12]}")

    def _verify_tf(self) -> Tuple[bool, str]:
        checks = (
            (self._base_frame, self._ee_frame, "base_link->rg2_pinch_center"),
            (self._world_frame, self._base_frame, "world->base_link"),
            (self._world_frame, self._ee_frame, "world->rg2_pinch_center"),
        )
        for target, source, label in checks:
            try:
                self._tf_buffer.lookup_transform(
                    target,
                    source,
                    Time(),
                    timeout=Duration(seconds=2.0),
                )
                self._log_event(f"{_PREFIX}[TF][OK] {label}")
            except TransformException as exc:
                self._log_event(f"{_PREFIX}[TF][FAIL] {label} reason={exc}")
                return False, f"{label}:{exc}"
        return True, "ok"

    def _current_tcp_quat_base(self) -> Optional[Tuple[float, float, float, float]]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._ee_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException:
            return None
        q = tf.transform.rotation
        return _q_norm((q.x, q.y, q.z, q.w))

    def _world_pose_to_base_xyz(self, pose: DemoPose) -> Tuple[float, float, float]:
        tf = self._tf_buffer.lookup_transform(
            self._base_frame,
            self._world_frame,
            Time(),
            timeout=Duration(seconds=1.0),
        )
        return _transform_point(tf, (pose.x, pose.y, pose.z))

    def _moveit_phase(
        self,
        phase: str,
        pose_world: DemoPose,
        quat_base: Tuple[float, float, float, float],
    ) -> Tuple[bool, str]:
        self._log_event(f"{_PREFIX}[{phase}][START]")
        target_base = self._world_pose_to_base_xyz(pose_world)
        ok, reason = self._send_moveit_goal(target_base, quat_base)
        tag = "OK" if ok else "FAIL"
        self._log_event(
            f"{_PREFIX}[{phase}][{tag}] target_base=({target_base[0]:.3f},"
            f"{target_base[1]:.3f},{target_base[2]:.3f}) reason={reason}"
        )
        return ok, reason

    def _send_moveit_goal(
        self,
        target_base: Tuple[float, float, float],
        quat_base: Tuple[float, float, float, float],
    ) -> Tuple[bool, str]:
        from moveit_msgs.action import MoveGroup
        from .plan_to_pose_moveit_direct import build_move_group_goal, parse_move_group_result

        if self._moveit_client is None:
            self._moveit_client = ActionClient(
                self,
                MoveGroup,
                self._moveit_action_name,
                callback_group=self._cb_group,
            )
        if not self._moveit_client.wait_for_server(timeout_sec=5.0):
            return False, f"moveit_action_unavailable:{self._moveit_action_name}"
        goal = build_move_group_goal(
            target_base,
            quat_base,
            ee_frame=self._ee_frame,
            base_frame=self._base_frame,
            group_name=self._moveit_group_name,
            planning_time_sec=self._moveit_planning_time,
            position_tol_m=self._moveit_position_tol,
            orientation_tol_rad=self._moveit_orientation_tol,
            velocity_scaling_factor=0.20,
            acceleration_scaling_factor=0.20,
        )
        send_future = self._moveit_client.send_goal_async(goal)
        if not self._wait_future(send_future, 10.0):
            return False, "moveit_goal_send_timeout"
        gh = send_future.result()
        if gh is None or not getattr(gh, "accepted", False):
            return False, "moveit_goal_rejected"
        result_future = gh.get_result_async()
        if not self._wait_future(result_future, self._moveit_result_timeout):
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
            return False, f"moveit_result_timeout:{self._moveit_result_timeout:.1f}s"
        ok, reason = parse_move_group_result(result_future.result())
        if ok:
            return True, reason
        recovered, dist = self._target_reached_by_tf(target_base)
        if recovered:
            return True, f"tf_recovered_after_{reason}:dist={dist:.4f}"
        return False, reason

    def _target_reached_by_tf(
        self,
        target_base: Tuple[float, float, float],
        *,
        tol_m: float = 0.025,
    ) -> Tuple[bool, float]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._ee_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException:
            return False, float("inf")
        p = tf.transform.translation
        dx = float(p.x) - float(target_base[0])
        dy = float(p.y) - float(target_base[1])
        dz = float(p.z) - float(target_base[2])
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        return dist <= max(0.003, float(tol_m)), dist

    @staticmethod
    def _wait_future(future, timeout_sec: float) -> bool:
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        event.wait(timeout=max(0.1, float(timeout_sec)))
        return bool(future.done())

    def _command_gripper(self, target: float, phase: str) -> None:
        msg = Float64MultiArray()
        msg.data = [float(target), float(target) * self._gripper_joint2_sign]
        for _idx in range(3):
            self._gripper_pub.publish(msg)
            time.sleep(0.08)
        self._log_event(
            f"{_PREFIX}[{phase}] target={target:.4f} topic={self._gripper_cmd_topic} "
            f"data={list(msg.data)}"
        )

    def _close_gripper(self) -> Tuple[bool, str]:
        before = self._joint_values(_GRIPPER_JOINTS)
        self._log_event(f"{_PREFIX}[GRIPPER_CLOSE][before={before}]")
        self._command_gripper(self._gripper_closed, "GRIPPER_CLOSE_CMD")
        time.sleep(max(0.1, self._gripper_settle_sec))
        after = self._joint_values(_GRIPPER_JOINTS)
        self._log_event(f"{_PREFIX}[GRIPPER_CLOSE][after={after}]")
        if before is None or after is None:
            return False, "joint_states_missing_for_gripper"
        delta = max(abs(float(a) - float(b)) for a, b in zip(after, before))
        if delta < 1e-4 and max(abs(float(v)) for v in after) > 0.005:
            return False, "gripper_joint_states_did_not_change"
        return True, "ok"

    def _joint_values(self, names: Sequence[str]) -> Optional[list[float]]:
        with self._joint_state_lock:
            msg = self._latest_joint_state
        if msg is None:
            return None
        index = {str(n): i for i, n in enumerate(msg.name)}
        out = []
        for name in names:
            idx = index.get(str(name))
            if idx is None or idx >= len(msg.position):
                return None
            out.append(float(msg.position[idx]))
        return out

    def _fjt_lift(
        self,
        pose_world: DemoPose,
        quat_base: Tuple[float, float, float, float],
    ) -> Tuple[bool, str]:
        target_base = self._world_pose_to_base_xyz(pose_world)
        seed = self._joint_values(_UR5_JOINTS)
        if seed is None:
            return False, "fjt_no_joint_state_seed"
        from moveit_msgs.srv import GetPositionIK
        from control_msgs.action import FollowJointTrajectory
        from .fjt_direct_helpers import (
            build_fjt_path_tolerances,
            build_fjt_trajectory_two_point,
            build_ik_request,
            parse_ik_result,
        )

        if self._ik_client is None:
            self._ik_client = self.create_client(
                GetPositionIK,
                self._compute_ik_service,
                callback_group=self._cb_group,
            )
        if not self._ik_client.wait_for_service(timeout_sec=3.0):
            return False, f"ik_service_unavailable:{self._compute_ik_service}"
        ik_req = build_ik_request(
            target_xyz=target_base,
            target_quat_xyzw=quat_base,
            ee_frame=self._ee_frame,
            base_frame=self._base_frame,
            group_name=self._moveit_group_name,
            current_joints=seed,
            joint_names=_UR5_JOINTS,
            timeout_sec=5.0,
            avoid_collisions=False,
        )
        ik_future = self._ik_client.call_async(ik_req)
        if not self._wait_future(ik_future, 7.0):
            return False, "ik_timeout"
        ok, reason, target_joints = parse_ik_result(ik_future.result(), _UR5_JOINTS)
        if not ok or target_joints is None:
            return False, reason
        traj = build_fjt_trajectory_two_point(
            joint_names=_UR5_JOINTS,
            start_positions=seed,
            target_positions=target_joints,
            duration_sec=6.0,
        )
        if self._fjt_client is None:
            self._fjt_client = ActionClient(
                self,
                FollowJointTrajectory,
                self._fjt_action_name,
                callback_group=self._cb_group,
            )
        if not self._fjt_client.wait_for_server(timeout_sec=3.0):
            return False, f"fjt_action_unavailable:{self._fjt_action_name}"
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.path_tolerance = build_fjt_path_tolerances(
            joint_names=_UR5_JOINTS,
            position_tolerance_rad=0.3,
        )
        send_future = self._fjt_client.send_goal_async(goal)
        if not self._wait_future(send_future, 4.0):
            return False, "fjt_goal_send_timeout"
        gh = send_future.result()
        if gh is None or not getattr(gh, "accepted", False):
            return False, "fjt_goal_rejected"
        result_future = gh.get_result_async()
        if not self._wait_future(result_future, 30.0):
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
            return False, "fjt_result_timeout"
        wrapper = result_future.result()
        ec = getattr(getattr(wrapper, "result", None), "error_code", 0)
        ec_val = int(getattr(ec, "val", ec if isinstance(ec, int) else 0))
        if ec_val != 0:
            return False, f"fjt_error_code={ec_val}"
        self._log_event(f"{_PREFIX}[LIFT][OK] fallback=fjt_direct")
        return True, "fjt_direct:SUCCESS"

    def _capture_pose_info(self, path: str, label: str) -> Optional[float]:
        z_value: Optional[float] = None
        try:
            env = os.environ.copy()
            if not env.get("GZ_PARTITION"):
                ws_candidates = [
                    Path(env.get("WS_DIR", "")) if env.get("WS_DIR") else None,
                    Path.cwd(),
                    Path(__file__).resolve().parents[3],
                ]
                for ws_dir in ws_candidates:
                    if ws_dir is None:
                        continue
                    part_file = ws_dir / "log" / "gz_partition.txt"
                    if part_file.exists():
                        env["GZ_PARTITION"] = part_file.read_text(
                            encoding="utf-8"
                        ).strip()
                        break
            out = subprocess.run(
                [
                    "gz",
                    "topic",
                    "-e",
                    "-n",
                    "1",
                    "-t",
                    "/world/ur5_mesa_objetos/pose/info",
                ],
                text=True,
                capture_output=True,
                env=env,
                timeout=20.0,
                check=False,
            )
            raw = out.stdout or out.stderr or ""
            block = self._extract_pick_demo_block(raw)
            Path(path).write_text(block or raw or "no_pose_info\n", encoding="utf-8")
            z_value = self._extract_z(block or raw)
            self._log_event(f"{_PREFIX}[GAZEBO_POSE] label={label} path={path} z={z_value}")
        except Exception as exc:
            Path(path).write_text(f"error={exc}\n", encoding="utf-8")
            self._log_event(f"{_PREFIX}[GAZEBO_POSE] label={label} path={path} error={exc}")
        return z_value

    @staticmethod
    def _extract_pick_demo_block(raw: str) -> str:
        lines = raw.splitlines()
        hit = -1
        for idx, line in enumerate(lines):
            if 'name: "pick_demo"' in line or "name: 'pick_demo'" in line:
                hit = idx
                break
        if hit < 0:
            return ""
        start = max(0, hit - 2)
        end = min(len(lines), hit + 20)
        return "\n".join(lines[start:end]) + "\n"

    @staticmethod
    def _extract_z(text: str) -> Optional[float]:
        lines = text.splitlines()
        for idx, line in enumerate(lines):
            if "z:" not in line:
                continue
            try:
                return float(line.split("z:", 1)[1].strip().split()[0])
            except Exception:
                continue
        return None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PhysicalGraspDemo()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        autostart = bool(node.get_parameter("autostart").value)
        keep_alive = bool(node.get_parameter("keep_alive").value)
        if autostart:
            ok, message = node.run_demo()
            if not ok:
                node.get_logger().error(message)
            if not keep_alive:
                return
        while rclpy.ok():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()
            spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
