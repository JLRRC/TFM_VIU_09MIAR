#!/usr/bin/env python3
# URL: agarre_ros2_ws/src/ur5_tools/ur5_tools/validate_dh_vs_tf.py
# Summary: Compare UR5 DH FK against live TF to detect kinematic model divergence.
"""Validate that the numeric DH FK matches the TF published by robot_state_publisher.

Usage (after source install/setup.bash):
    ros2 run ur5_tools validate_dh_vs_tf
    ros2 run ur5_tools validate_dh_vs_tf --ros-args -p fail_threshold_m:=0.005

The node reads one /joint_states sample, computes FK, queries TF and prints
the discrepancy.  Exit code 0 = all errors under threshold; 1 = divergence.

Checked transforms:
  base_link -> tool0             (DH gives base_link_inertia, flipped XY for base_link)
  base_link -> rg2_pinch_center  (DH + URDF offset 0.175 in tool0-local Z)
"""

from __future__ import annotations

import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import tf2_ros

_UR5_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Standard UR5 CB3 DH parameters (same as ur5_kinematics.py)
_A = [0.0, -0.425, -0.39225, 0.0, 0.0, 0.0]
_D = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
_ALPHA = [math.pi / 2.0, 0.0, 0.0, math.pi / 2.0, -math.pi / 2.0, 0.0]

# URDF canonical offset tool0 → rg2_pinch_center in tool0-local frame
_PINCH_OFFSET_TOOL0_LOCAL = (0.0, 0.0, 0.175)

FAIL_THRESHOLD_DEFAULT_M = 0.005


def _dh(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [[ct, -st * ca, st * sa, a * ct],
         [st,  ct * ca, -ct * sa, a * st],
         [0.0, sa, ca, d],
         [0.0, 0.0, 0.0, 1.0]],
        dtype=float,
    )


def _fk(q: list[float]) -> tuple[np.ndarray, np.ndarray]:
    """Returns (position_3, rotation_3x3) in base_link_inertia frame."""
    t = np.eye(4, dtype=float)
    for i in range(6):
        t = t @ _dh(_A[i], _D[i], _ALPHA[i], q[i])
    return t[:3, 3].copy(), t[:3, :3].copy()


def _fk_base_link(q: list[float]) -> tuple[np.ndarray, np.ndarray]:
    """FK result converted from base_link_inertia to base_link (negate X, Y)."""
    pos_inertia, rot_inertia = _fk(q)
    # base_link_inertia has Rz(π) relative to base_link (standard ur_description).
    # Negating X and Y converts positions from base_link_inertia to base_link.
    pos_base = np.array([-pos_inertia[0], -pos_inertia[1], pos_inertia[2]])
    # Rotation: Rz(π) * rot_inertia  [columns 0,1 negate; col 2 unchanged]
    rot_base = rot_inertia.copy()
    rot_base[:, 0] = -rot_inertia[:, 0]
    rot_base[:, 1] = -rot_inertia[:, 1]
    return pos_base, rot_base


def _pinch_center_from_tool0(pos_tool0: np.ndarray, rot_tool0: np.ndarray) -> np.ndarray:
    """Apply URDF offset (0,0,0.175) in tool0-local frame to get rg2_pinch_center."""
    offset_local = np.array(_PINCH_OFFSET_TOOL0_LOCAL, dtype=float)
    return pos_tool0 + rot_tool0 @ offset_local


def _tf_lookup(buf: tf2_ros.Buffer, target: str, source: str, timeout: float = 0.5):
    """Return position (x,y,z) from TF or None."""
    try:
        tf = buf.lookup_transform(target, source, rclpy.time.Time(), rclpy.duration.Duration(seconds=timeout))
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z], dtype=float)
    except Exception:
        return None


def _err3d(a: np.ndarray, b: np.ndarray) -> tuple[np.ndarray, float]:
    diff = a - b
    return diff, float(np.linalg.norm(diff))


class ValidateDhVsTf(Node):
    def __init__(self, fail_threshold_m: float):
        super().__init__("validate_dh_vs_tf")
        self._fail_threshold_m = fail_threshold_m
        self._joints: dict[str, float] = {}
        self._done = False
        self._result_ok = False

        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        self._sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            qos_profile_sensor_data,
        )
        self.get_logger().info("[DH_TF_CHECK] waiting for /joint_states and TF...")

    def _on_joint_states(self, msg: JointState) -> None:
        if self._done:
            return
        joint_map = dict(zip(msg.name, msg.position))
        q = [joint_map.get(n) for n in _UR5_JOINT_NAMES]
        if any(v is None for v in q):
            return
        q = [float(v) for v in q]

        # Wait until TF is available
        pos_tool0_tf = _tf_lookup(self._tf_buf, "base_link", "tool0")
        pos_pinch_tf = _tf_lookup(self._tf_buf, "base_link", "rg2_pinch_center")
        if pos_tool0_tf is None or pos_pinch_tf is None:
            return

        self._done = True

        # DH FK
        pos_tool0_dh, rot_tool0_dh = _fk_base_link(q)
        pos_pinch_dh = _pinch_center_from_tool0(pos_tool0_dh, rot_tool0_dh)

        # tool0 comparison
        diff_t0, err_t0 = _err3d(pos_tool0_dh, pos_tool0_tf)
        # rg2_pinch_center comparison
        diff_pc, err_pc = _err3d(pos_pinch_dh, pos_pinch_tf)

        ok_t0 = err_t0 <= self._fail_threshold_m
        ok_pc = err_pc <= self._fail_threshold_m
        overall_ok = ok_t0 and ok_pc

        thr = self._fail_threshold_m
        self.get_logger().info(
            f"[DH_TF_CHECK] joints={[f'{v:.3f}' for v in q]}"
        )
        self.get_logger().info(
            f"[DH_TF_CHECK] tool0       DH=({pos_tool0_dh[0]:.4f},{pos_tool0_dh[1]:.4f},{pos_tool0_dh[2]:.4f})"
            f"  TF=({pos_tool0_tf[0]:.4f},{pos_tool0_tf[1]:.4f},{pos_tool0_tf[2]:.4f})"
            f"  err_xyz=({diff_t0[0]:.4f},{diff_t0[1]:.4f},{diff_t0[2]:.4f})"
            f"  err_3d={err_t0:.4f}m  {'OK' if ok_t0 else f'FAIL (>{thr:.3f}m)'}"
        )
        self.get_logger().info(
            f"[DH_TF_CHECK] rg2_pinch   DH=({pos_pinch_dh[0]:.4f},{pos_pinch_dh[1]:.4f},{pos_pinch_dh[2]:.4f})"
            f"  TF=({pos_pinch_tf[0]:.4f},{pos_pinch_tf[1]:.4f},{pos_pinch_tf[2]:.4f})"
            f"  err_xyz=({diff_pc[0]:.4f},{diff_pc[1]:.4f},{diff_pc[2]:.4f})"
            f"  err_3d={err_pc:.4f}m  {'OK' if ok_pc else f'FAIL (>{thr:.3f}m)'}"
        )

        if overall_ok:
            self.get_logger().info(
                f"[DH_TF_CHECK] OK — DH and TF agree within {thr*1000:.1f} mm"
            )
        else:
            self.get_logger().error(
                "[DH_TF_CHECK] FAIL — DH/TF diverge. Check: "
                "base_link_inertia Rz(π) negate, DH D-params, URDF wrist chain, "
                "NEGATE_XY sign, or robot_state_publisher topic."
            )

        self._result_ok = overall_ok


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ValidateDhVsTf(fail_threshold_m=FAIL_THRESHOLD_DEFAULT_M)

    # Read fail_threshold from ROS param if declared
    node.declare_parameter("fail_threshold_m", FAIL_THRESHOLD_DEFAULT_M)
    thr = node.get_parameter("fail_threshold_m").get_parameter_value().double_value
    node._fail_threshold_m = float(thr)

    deadline = time.time() + 10.0
    while rclpy.ok() and not node._done and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node._done:
        node.get_logger().error("[DH_TF_CHECK] TIMEOUT — could not get joint_states or TF within 10 s")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    ok = node._result_ok
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)
