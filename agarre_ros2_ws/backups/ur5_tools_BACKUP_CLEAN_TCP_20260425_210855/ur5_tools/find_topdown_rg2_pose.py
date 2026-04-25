#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/find_topdown_rg2_pose.py
# Contenido: Herramienta de búsqueda de pose top-down para el objeto pick_demo.
# Uso breve: Script independiente de diagnóstico; no modifica la secuencia de pick.
"""Search for a valid top-down IK pose for the pick_demo object.

This script is diagnostic only.  It reads the current joint state and the
pick_demo object world position, then searches for IK solutions that place
rg2_pinch_center at the object and orient tool0+Z ≈ -Z_world (top-down).

Usage:
    ros2 run ur5_tools find_topdown_rg2_pose
    ros2 run ur5_tools find_topdown_rg2_pose --execute-test-pose

Logs tagged [TOPDOWN_SEARCH] are emitted to the ROS 2 logger and stdout.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
import tf2_ros

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_LOG = "[TOPDOWN_SEARCH]"

# UR5 base in Gazebo world frame (from ur5.urdf.xacro <origin xyz="-0.85 0 0.850">)
_ROBOT_BASE_WORLD_X = -0.85
_ROBOT_BASE_WORLD_Z = 0.850

# rg2_pinch_center offset from tool0 in tool0-local Z: loaded from URDF canonical geometry.
from ur5_tools.gripper_geometry import load_gripper_geometry as _load_geom_topdown, RG2_PINCH_CENTER_FRAME as _PC_TOPDOWN
_PINCH_OFFSET_Z = _load_geom_topdown().z_for_frame(_PC_TOPDOWN)

# DH parameters (UR5 CB3, same as ur5_kinematics.py)
_A = [0.0, -0.425, -0.39225, 0.0, 0.0, 0.0]
_D = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
_ALPHA = [math.pi / 2.0, 0.0, 0.0, math.pi / 2.0, -math.pi / 2.0, 0.0]

_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Top-down target rotation for tool0 in base_link frame:
#   Column 0 = tool0+X = (1, 0, 0)
#   Column 1 = tool0+Y = (0, -1, 0)
#   Column 2 = tool0+Z = (0, 0, -1)  ← pointing down
_R_TOPDOWN = np.array(
    [[1.0,  0.0,  0.0],
     [0.0, -1.0,  0.0],
     [0.0,  0.0, -1.0]],
    dtype=float,
)

# Dot product threshold to classify tool0+Z ≈ -Zworld as top-down
_TOPDOWN_DOT_MIN = 0.90

# Error tolerance to accept an IK solution as valid
_IK_ERR_TOL = 0.050


# ---------------------------------------------------------------------------
# DH forward kinematics
# ---------------------------------------------------------------------------

def _dh(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array(
        [[ct, -st * ca,  st * sa, a * ct],
         [st,  ct * ca, -ct * sa, a * st],
         [0.0,      sa,       ca,      d],
         [0.0,     0.0,      0.0,    1.0]],
        dtype=float,
    )


def _fk(q: List[float]) -> Tuple[np.ndarray, np.ndarray]:
    """FK for tool0 in DH-base frame.  Returns (pos_xyz, rot_3x3)."""
    t = np.eye(4, dtype=float)
    for i in range(6):
        t = t @ _dh(_A[i], _D[i], _ALPHA[i], q[i])
    return t[:3, 3].copy(), t[:3, :3].copy()


def _ik(
    target_pos: np.ndarray,
    target_rot: np.ndarray,
    seed: List[float],
    *,
    pos_weight: float = 1.0,
    rot_weight: float = 0.5,
    joint_weight: float = 0.1,
    max_iter: int = 400,
) -> Tuple[np.ndarray, float, bool]:
    """Numeric IK.  Returns (q, err_norm, success)."""
    q0 = np.asarray(seed, dtype=float)

    def _wrap(v: np.ndarray) -> np.ndarray:
        return (v + math.pi) % (2.0 * math.pi) - math.pi

    def _err(q: np.ndarray) -> np.ndarray:
        pos, rot = _fk(list(q))
        dp = (pos - target_pos) * pos_weight
        r_err = rot.T @ target_rot
        rv = Rotation.from_matrix(r_err).as_rotvec() * rot_weight
        dq = _wrap(q - q0) * joint_weight
        return np.concatenate([dp, rv, dq])

    bounds = ([-2.0 * math.pi] * 6, [2.0 * math.pi] * 6)
    res = least_squares(_err, q0, bounds=bounds, max_nfev=max_iter)
    return res.x, float(np.linalg.norm(res.fun[:3])), bool(res.success)


# ---------------------------------------------------------------------------
# Seed generation
# ---------------------------------------------------------------------------

def _build_seeds(q_live: List[float]) -> List[Tuple[str, List[float]]]:
    """Return labelled candidate seeds for top-down IK search."""
    seeds: List[Tuple[str, List[float]]] = []

    # Seed A: flip wrist_2 sign (main maneuver for top-down regime)
    q_a = list(q_live)
    q_a[4] = -q_a[4]
    seeds.append(("flip_wrist2", q_a))

    # Seed B: force wrist_2 = +π/2 directly
    q_b = list(q_live)
    q_b[4] = math.pi / 2.0
    seeds.append(("wrist2_pos90", q_b))

    # Seed C: force wrist_2 = +π/2, wrist_3 offset +π
    q_c = list(q_live)
    q_c[4] = math.pi / 2.0
    q_c[5] = q_c[5] + math.pi
    seeds.append(("wrist2_pos90_w3_flip", q_c))

    # Seed D: flip wrist_2, flip wrist_1 sign
    q_d = list(q_live)
    q_d[4] = -q_d[4]
    q_d[3] = -q_d[3]
    seeds.append(("flip_wrist2_flip_wrist1", q_d))

    # Seed E: standard home-like top-down start
    seeds.append(("home_like", [
        0.0, -math.pi / 2, 0.0, -math.pi / 2, math.pi / 2, 0.0
    ]))

    # Seed F: near-vertical elbow, positive wrist_2
    seeds.append(("vertical_elbow", [
        q_live[0], -2.0, 2.0, -0.5, math.pi / 2, 0.0
    ]))

    return seeds


# ---------------------------------------------------------------------------
# Orientation analysis
# ---------------------------------------------------------------------------

def _regime_from_rot(rot: np.ndarray) -> Tuple[str, float]:
    """Determine grip regime from tool0 rotation matrix (in world/base frame)."""
    tool0_z = rot[:, 2]
    dot_down = float(-tool0_z[2])    # dot with -Z_world
    dot_up = float(tool0_z[2])       # dot with +Z_world
    if dot_down >= _TOPDOWN_DOT_MIN:
        return "TOP_DOWN", dot_down
    if dot_up >= _TOPDOWN_DOT_MIN:
        return "BOTTOM_UP", dot_up
    return "LATERAL", max(dot_down, dot_up)


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class FindTopdownPoseNode(Node):
    def __init__(self, execute_test: bool) -> None:
        super().__init__("find_topdown_rg2_pose")
        self._execute_test = execute_test
        self._joint_state: Optional[JointState] = None
        self._sub = self.create_subscription(
            JointState, "/joint_states", self._js_cb, qos_profile_sensor_data
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def _js_cb(self, msg: JointState) -> None:
        self._joint_state = msg

    def _log(self, msg: str) -> None:
        self.get_logger().info(msg)
        print(msg, flush=True)

    def _get_live_joints(self) -> Optional[List[float]]:
        js = self._joint_state
        if js is None:
            return None
        name_to_pos = dict(zip(js.name, js.position))
        q = []
        for name in _JOINT_NAMES:
            if name not in name_to_pos:
                return None
            q.append(float(name_to_pos[name]))
        return q

    def _get_object_world(self) -> Optional[np.ndarray]:
        """Get pick_demo object position in world frame via TF (rg2_pinch_center fallback)."""
        # Try rg2_pinch_center as a proxy for current TCP world pos
        # Real object pos is read from TF frame 'pick_demo' if attached, else from JSON
        import os
        import pathlib

        json_candidates = [
            pathlib.Path(__file__).parents[4] / "scripts" / "object_positions.json",
            pathlib.Path("/home/laboratorio/TFM/agarre_ros2_ws/scripts/object_positions.json"),
        ]
        for p in json_candidates:
            if p.exists():
                try:
                    data = json.loads(p.read_text())
                    coords = data.get("pick_demo")
                    if coords and len(coords) >= 3:
                        self._log(
                            f"{_LOG} object_source=json path={p} "
                            f"pos_world=({coords[0]:.4f},{coords[1]:.4f},{coords[2]:.4f})"
                        )
                        return np.array([float(coords[0]), float(coords[1]), float(coords[2])])
                except Exception as exc:
                    self._log(f"{_LOG} json_read_error={exc}")

        # Fallback: try TF rg2_pinch_center → world
        try:
            tf = self._tf_buffer.lookup_transform("world", "rg2_pinch_center", rclpy.time.Time())
            t = tf.transform.translation
            return np.array([float(t.x), float(t.y), float(t.z)])
        except Exception:
            pass
        return None

    def run(self) -> int:
        self._log(f"{_LOG} starting search execute_test={self._execute_test}")

        # Wait for joint state
        deadline = time.time() + 5.0
        while time.time() < deadline and self._joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        q_live = self._get_live_joints()
        if q_live is None:
            self._log(f"{_LOG} status=ABORT reason=joint_state_unavailable")
            return 1

        self._log(
            f"{_LOG} live_joints=[{', '.join(f'{v:.4f}' for v in q_live)}]"
        )

        # Get object world position
        obj_world = self._get_object_world()
        if obj_world is None:
            self._log(f"{_LOG} status=ABORT reason=object_world_unavailable")
            return 1

        # Convert object world position to DH base frame (= base_link)
        # base_link origin in world = (-0.85, 0, 0.850), same orientation
        obj_base = np.array([
            obj_world[0] - _ROBOT_BASE_WORLD_X,
            float(obj_world[1]),
            obj_world[2] - _ROBOT_BASE_WORLD_Z,
        ])

        # For top-down grasp, rg2_pinch_center is at obj_base.
        # rg2_pinch_center = tool0_pos + R_tool0 * (0,0,_PINCH_OFFSET_Z)
        # With R_TOPDOWN: R * (0,0,_PINCH_OFFSET_Z) = (0,0,-_PINCH_OFFSET_Z) [tool0+Z = -Zworld]
        # So: tool0_target = obj_base - (0,0,-_PINCH_OFFSET_Z) = obj_base + (0,0,_PINCH_OFFSET_Z)
        tool0_target = obj_base.copy()
        tool0_target[2] += _PINCH_OFFSET_Z

        self._log(
            f"{_LOG} obj_world=({obj_world[0]:.4f},{obj_world[1]:.4f},{obj_world[2]:.4f}) "
            f"obj_base=({obj_base[0]:.4f},{obj_base[1]:.4f},{obj_base[2]:.4f}) "
            f"tool0_target=({tool0_target[0]:.4f},{tool0_target[1]:.4f},{tool0_target[2]:.4f}) "
            f"target_regime=TOP_DOWN"
        )

        # Current FK for reference
        fk_pos, fk_rot = _fk(q_live)
        cur_regime, cur_dot = _regime_from_rot(fk_rot)
        self._log(
            f"{_LOG} current_fk "
            f"tool0_base=({fk_pos[0]:.4f},{fk_pos[1]:.4f},{fk_pos[2]:.4f}) "
            f"tool0_z_world=({fk_rot[0,2]:.3f},{fk_rot[1,2]:.3f},{fk_rot[2,2]:.3f}) "
            f"regime={cur_regime} dot={cur_dot:.3f}"
        )

        # Search across seeds
        seeds = _build_seeds(q_live)
        best: Optional[Tuple[List[float], float, str]] = None  # (q, err, label)

        for label, seed in seeds:
            try:
                q_sol, pos_err, ok = _ik(
                    tool0_target, _R_TOPDOWN, seed,
                    pos_weight=1.0, rot_weight=0.5, joint_weight=0.05,
                    max_iter=600,
                )
                sol_pos, sol_rot = _fk(list(q_sol))
                regime, dot = _regime_from_rot(sol_rot)
                rg2_pinch_sol = sol_pos + sol_rot[:, 2] * _PINCH_OFFSET_Z
                pinch_err = float(np.linalg.norm(rg2_pinch_sol - obj_base))
                is_topdown = regime == "TOP_DOWN"
                candidate_ok = is_topdown and pos_err < _IK_ERR_TOL
                self._log(
                    f"{_LOG} candidate label={label} "
                    f"seed=[{', '.join(f'{v:.4f}' for v in seed)}] "
                    f"solution=[{', '.join(f'{v:.4f}' for v in q_sol)}] "
                    f"pos_err_m={pos_err:.4f} pinch_err_m={pinch_err:.4f} "
                    f"tool0_z_world=({sol_rot[0,2]:.3f},{sol_rot[1,2]:.3f},{sol_rot[2,2]:.3f}) "
                    f"regime={regime} dot_topdown={dot:.3f} "
                    f"ik_converged={str(ok).lower()} verdict={'VALID' if candidate_ok else 'INVALID'}"
                )
                if candidate_ok:
                    if best is None or pos_err < best[1]:
                        best = (list(q_sol), pos_err, label)
            except Exception as exc:
                self._log(f"{_LOG} candidate label={label} verdict=EXCEPTION reason={exc!s}")

        if best is None:
            self._log(
                f"{_LOG} status=NO_SOLUTION "
                "reason=no_candidate_satisfied_topdown_and_err_tol "
                f"err_tol={_IK_ERR_TOL:.3f} dot_min={_TOPDOWN_DOT_MIN:.2f}"
            )
            return 1

        q_best, best_err, best_label = best
        fk_best_pos, fk_best_rot = _fk(q_best)
        rg2_pinch_best = fk_best_pos + fk_best_rot[:, 2] * _PINCH_OFFSET_Z
        regime_best, dot_best = _regime_from_rot(fk_best_rot)
        self._log(
            f"{_LOG} status=SOLUTION_FOUND "
            f"best_label={best_label} pos_err_m={best_err:.4f} "
            f"q_deg=[{', '.join(f'{math.degrees(v):.2f}' for v in q_best)}] "
            f"tool0_base=({fk_best_pos[0]:.4f},{fk_best_pos[1]:.4f},{fk_best_pos[2]:.4f}) "
            f"rg2_pinch_base=({rg2_pinch_best[0]:.4f},{rg2_pinch_best[1]:.4f},{rg2_pinch_best[2]:.4f}) "
            f"obj_base=({obj_base[0]:.4f},{obj_base[1]:.4f},{obj_base[2]:.4f}) "
            f"regime={regime_best} dot_topdown={dot_best:.3f}"
        )

        if self._execute_test:
            self._log(
                f"{_LOG} execute_test=true "
                "note=NOT_IMPLEMENTED_publish_JointTrajectory_manually"
            )

        return 0


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Find top-down IK pose for pick_demo")
    parser.add_argument(
        "--execute-test-pose",
        action="store_true",
        help="(placeholder) Execute the found pose on the real robot",
    )
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args or None)
    node = FindTopdownPoseNode(execute_test=args.execute_test_pose)
    try:
        # Spin briefly to receive joint state, then run search
        deadline = time.time() + 6.0
        while time.time() < deadline and node._joint_state is None:
            rclpy.spin_once(node, timeout_sec=0.2)
        code = node.run()
    except KeyboardInterrupt:
        code = 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
