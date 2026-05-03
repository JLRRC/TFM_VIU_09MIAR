"""GeometryMixin: helpers de pose <-> matriz, FK + IK seeding para APPROACH.

Extraído de ``ur5_tools/ur5_moveit_bridge.py`` (líneas 3252-3596 originales):

* Constante ``_HOME_JOINTS_SEED`` (referencia elbow-down para IK).
* Wrappers ``_pose_to_matrix`` / ``_matrix_to_pose`` que delegan en
  ``moveit_bridge_utils``.
* ``_robot_state_frame_transform_matrix`` (FK auxiliary).
* ``_compute_approach_ik_seeded`` (IK con seeds priorizadas — current_state
  primero, home como fallback; selecciona la solución que minimiza fk_err
  y delta-joint manteniendo la rama visual viva).

Sin estado propio. La clase concreta debe heredar también de
``rclpy.node.Node`` y de ``JointStateHelpersMixin`` (provee
``_normalize_joint_position`` y constantes de joints).
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from geometry_msgs.msg import Pose, PoseStamped

from ..moveit_bridge_utils import matrix_to_pose, pose_to_matrix

try:
    from moveit.core.robot_state import RobotState as MoveItRobotState  # type: ignore
except Exception:  # pragma: no cover - moveit_py opcional
    MoveItRobotState = None  # type: ignore


class GeometryMixin:
    """Pose<->matrix helpers + IK seeding para APPROACH."""

    # HOME joint values (elbow-down reference for IK seeding)
    _HOME_JOINTS_SEED = {
        "shoulder_pan_joint": 0.0,
        "shoulder_lift_joint": -1.5708,
        "elbow_joint": 0.0,
        "wrist_1_joint": 0.0,
        "wrist_2_joint": 0.0,
        "wrist_3_joint": 0.0,
    }

    @staticmethod
    def _pose_to_matrix(pose: Pose) -> np.ndarray:
        return pose_to_matrix(pose)

    @staticmethod
    def _matrix_to_pose(T: np.ndarray) -> Any:
        return matrix_to_pose(T)

    def _robot_state_frame_transform_matrix(
        self,
        state: "MoveItRobotState",
        frame_name: str,
    ) -> tuple["np.ndarray | None", str]:
        try:
            transform = state.get_frame_transform(frame_name)
        except Exception as exc:
            return None, f"get_frame_transform_failed:{frame_name}:{type(exc).__name__}:{exc}"
        if transform is None:
            return None, f"transform_missing:{frame_name}"
        try:
            transform_np = np.asarray(transform, dtype=float).reshape(4, 4)
        except Exception as exc:
            return None, f"transform_parse_failed:{frame_name}:{type(exc).__name__}:{exc}"
        return transform_np, f"transform_ok:{frame_name}"

    def _eval_ik_seed_candidate(
        self,
        *,
        robot_model,
        seed_label: str,
        seed_positions,
        group_joint_names,
        score_reference_map: dict,
        seed_reference_label: str,
        ik_tip: str,
        tcp_target_pose,
        T_base_ee_target,
        T_model_tcp_target,
        candidate_fk_tol_m: float,
        candidate_max_joint_delta_rad: float,
        candidate_sum_joint_delta_rad: float,
        joint_delta_fn,
    ):
        """F3-step31a: evalua un candidato IK con un seed dado (~115 LOC).

        Llama set_from_ik con el seed, normaliza wrap-around joints, evalua
        FK error vs target + max/sum joint delta. Devuelve
        (candidate_score, ik_state, solved_map) si valido, None si rechazado.
        """
        ik_state = MoveItRobotState(robot_model)
        ik_state.set_joint_group_positions(self._group_name, seed_positions)
        ik_state.update()
        ok = ik_state.set_from_ik(
            self._group_name,
            tcp_target_pose,
            ik_tip,
            1.0,
        )
        if not ok:
            self.get_logger().warning(
                f"[APPROACH_IK_SEED] IK solve failed seed={seed_label} "
                f"tip={ik_tip} target_frame={self._base_frame or 'base_link'} "
                f"tcp_target_model=({T_model_tcp_target[0,3]:.3f},"
                f"{T_model_tcp_target[1,3]:.3f},{T_model_tcp_target[2,3]:.3f})"
            )
            return None
        ik_state.update()
        solved = list(ik_state.get_joint_group_positions(self._group_name))
        wrap_adjusted: list[str] = []
        for jdx, jname in enumerate(group_joint_names):
            if jname not in self._WRAPAROUND_JOINTS:
                continue
            ref_val = score_reference_map.get(jname)
            if ref_val is None:
                continue
            raw_val = float(solved[jdx])
            adjusted = raw_val + (
                math.tau * round((float(ref_val) - raw_val) / math.tau)
            )
            if abs(adjusted - raw_val) > 1e-6:
                solved[jdx] = adjusted
                wrap_adjusted.append(f"{jname}:{raw_val:.3f}->{adjusted:.3f}")
        if wrap_adjusted:
            ik_state.set_joint_group_positions(
                self._group_name,
                np.asarray(solved, dtype=float),
            )
            ik_state.update()
            self.get_logger().info(
                "[APPROACH_IK_SEED] wrap-normalized continuous joints "
                f"seed={seed_label} adjusted={','.join(wrap_adjusted)}"
            )
        solved_map = dict(zip(group_joint_names, solved))
        T_model_base_candidate, base_candidate_detail = (
            self._robot_state_frame_transform_matrix(ik_state, self._base_frame)
        )
        T_model_ee_candidate, ee_candidate_detail = (
            self._robot_state_frame_transform_matrix(ik_state, self._ee_frame)
        )
        fk_err = float("inf")
        fk_dx = float("nan")
        fk_dy = float("nan")
        fk_dz = float("nan")
        if T_model_base_candidate is not None and T_model_ee_candidate is not None:
            T_base_ee_candidate = (
                np.linalg.inv(T_model_base_candidate) @ T_model_ee_candidate
            )
            fk_dx = float(T_base_ee_candidate[0, 3] - T_base_ee_target[0, 3])
            fk_dy = float(T_base_ee_candidate[1, 3] - T_base_ee_target[1, 3])
            fk_dz = float(T_base_ee_candidate[2, 3] - T_base_ee_target[2, 3])
            fk_err = math.sqrt((fk_dx * fk_dx) + (fk_dy * fk_dy) + (fk_dz * fk_dz))
        deltas: list[float] = []
        for joint_name in group_joint_names:
            delta = joint_delta_fn(score_reference_map, solved_map, joint_name)
            if delta is not None:
                deltas.append(float(delta))
        max_delta = max(deltas) if deltas else float("inf")
        sum_delta = sum(deltas) if deltas else float("inf")
        elbow_val = solved_map.get("elbow_joint", None)
        elbow_abs = abs(float(elbow_val)) if elbow_val is not None else float("inf")
        branch_hint = (
            "elbow_down_like"
            if elbow_val is not None and elbow_abs <= (math.pi / 3)
            else "elbow_wide"
        )
        joints_str = " ".join(
            f"{name}={solved_map[name]:.3f}"
            for name in group_joint_names
            if name in solved_map
        )
        self.get_logger().info(
            f"[APPROACH_IK_SEED] candidate seed={seed_label} "
            f"score_ref={seed_reference_label} elbow="
            f"{f'{elbow_val:.3f}' if elbow_val is not None else 'n/a'} "
            f"branch={branch_hint} compared={len(deltas)} "
            f"fk_err={fk_err:.3f} "
            f"delta=({fk_dx:.3f},{fk_dy:.3f},{fk_dz:.3f}) "
            f"max_delta={max_delta:.3f} sum_delta={sum_delta:.3f} {joints_str}"
        )
        if not math.isfinite(fk_err):
            self.get_logger().warning(
                f"[APPROACH_IK_SEED] rejecting seed={seed_label} "
                f"reason=fk_unavailable base={base_candidate_detail} ee={ee_candidate_detail}"
            )
            return None
        if fk_err > candidate_fk_tol_m:
            self.get_logger().warning(
                f"[APPROACH_IK_SEED] rejecting seed={seed_label} "
                f"reason=fk_mismatch fk_err={fk_err:.3f} tol={candidate_fk_tol_m:.3f} "
                f"delta=({fk_dx:.3f},{fk_dy:.3f},{fk_dz:.3f})"
            )
            return None
        if max_delta > candidate_max_joint_delta_rad or sum_delta > candidate_sum_joint_delta_rad:
            self.get_logger().warning(
                f"[APPROACH_IK_SEED] rejecting seed={seed_label} "
                f"reason=branch_too_far max_delta={max_delta:.3f} "
                f"sum_delta={sum_delta:.3f} "
                f"limits=({candidate_max_joint_delta_rad:.3f},{candidate_sum_joint_delta_rad:.3f})"
            )
            return None
        candidate_score = (
            fk_err,
            max_delta,
            sum_delta,
            elbow_abs,
            0 if seed_label == "current_state" else 1,
        )
        return candidate_score, ik_state, solved_map

    def _compute_approach_ik_seeded(
        self,
        target: PoseStamped,
    ) -> "MoveItRobotState | None":
        """
        Compute IK for APPROACH using seeds prioritized by the live arm state.

        Strategy:
        - The SRDF kinematic chain tip is 'rg2_tcp', not 'rg2_pinch_center'.
        - set_from_ik only works with the registered chain tip.
        - We compute the rg2_tcp pose that places rg2_pinch_center at the target,
          using the fixed T_tcp->pinch offset obtained from FK.
        - Then solve IK with tip='rg2_tcp' using the live arm state first and
          HOME as fallback, selecting the valid solution closest to the current
          joints so execution stays on the visually consistent branch.
        """
        if MoveItRobotState is None or self._moveit_py is None:
            return None
        try:
            target_base = target
            if str(target.header.frame_id or "").strip() != str(self._base_frame or "").strip():
                target_base = self._ensure_base_frame(target)
            if target_base is None:
                self.get_logger().warning(
                    "[APPROACH_IK_SEED] target transform to base frame unavailable "
                    f"source={str(target.header.frame_id or 'n/a')}"
                )
                return None
            robot_model = self._moveit_py.get_robot_model()
            if robot_model is None:
                return None
            joint_model_group = robot_model.get_joint_model_group(self._group_name)
            if joint_model_group is None:
                return None
            group_joint_names = list(
                getattr(joint_model_group, "active_joint_model_names", None)
                or getattr(joint_model_group, "joint_model_names", None)
                or self._START_STATE_JOINTS
            )
            current_joint_map: dict[str, float] = {}
            raw_names = list(self._joint_state_last_names or [])
            raw_positions = list(self._joint_state_last_positions or [])
            if raw_names and len(raw_positions) >= len(raw_names):
                for name, pos in zip(raw_names, raw_positions):
                    jname = str(name or "").strip()
                    if not jname:
                        continue
                    try:
                        value = float(pos)
                    except (TypeError, ValueError):
                        continue
                    if not math.isfinite(value):
                        continue
                    if jname in self._WRAPAROUND_JOINTS:
                        value = self._normalize_joint_position(jname, value)
                    current_joint_map[jname] = value
            home_joint_map = {
                str(name): float(self._HOME_JOINTS_SEED.get(name, 0.0))
                for name in group_joint_names
            }
            seed_candidates: list[tuple[str, np.ndarray]] = []
            has_live_seed = all(name in current_joint_map for name in group_joint_names)
            if has_live_seed:
                seed_candidates.append(
                    (
                        "current_state",
                        np.asarray(
                            [current_joint_map[name] for name in group_joint_names],
                            dtype=float,
                        ),
                    )
                )
            else:
                seed_candidates.append(
                    (
                        "home",
                        np.asarray(
                            [home_joint_map.get(name, 0.0) for name in group_joint_names],
                            dtype=float,
                        ),
                    )
                )
            seed_reference_label = (
                "current_state"
                if seed_candidates and seed_candidates[0][0] == "current_state"
                else "home"
            )
            score_reference_map = (
                current_joint_map if seed_reference_label == "current_state" else home_joint_map
            )
            ref_state = MoveItRobotState(robot_model)
            ref_state.set_joint_group_positions(self._group_name, seed_candidates[0][1])
            ref_state.update()
            if has_live_seed:
                self.get_logger().info(
                    "[APPROACH_IK_SEED] live joint seed available; "
                    "disabling home-seeded explicit APPROACH fallback to keep the live branch"
                )

            # Find the IK chain tip registered in the SRDF/kinematics (rg2_tcp).
            # get_frame_transform() is expressed in the robot-model root frame
            # (world in this workspace because of the SRDF virtual joint).
            # Convert targets/poses explicitly instead of assuming base_link.
            ik_tip = "rg2_tcp"
            T_model_base, base_detail = self._robot_state_frame_transform_matrix(
                ref_state,
                self._base_frame,
            )
            T_model_tcp, tcp_detail = self._robot_state_frame_transform_matrix(
                ref_state,
                ik_tip,
            )
            T_model_ee, ee_detail = self._robot_state_frame_transform_matrix(
                ref_state,
                self._ee_frame,
            )
            if T_model_base is None or T_model_tcp is None or T_model_ee is None:
                self.get_logger().warning(
                    "[APPROACH_IK_SEED] get_frame_transform failed "
                    f"base={base_detail} tcp={tcp_detail} ee={ee_detail}"
                )
                return None
            # Fixed offset: T_tcp->ee (constant, independent of joint config)
            T_tcp_ee = np.linalg.inv(T_model_tcp) @ T_model_ee

            # Compute the required tcp pose in model-root coordinates so that
            # ee_frame reaches the base_link target requested by the panel.
            T_base_ee_target = self._pose_to_matrix(target_base.pose)
            T_model_ee_target = T_model_base @ T_base_ee_target
            T_model_tcp_target = T_model_ee_target @ np.linalg.inv(T_tcp_ee)
            tcp_target_pose = self._matrix_to_pose(T_model_tcp_target)
            candidate_fk_tol_m = max(
                0.005,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_FK_ERR_M",
                    0.012,
                ),
            )
            candidate_max_joint_delta_rad = max(
                0.25,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_JOINT_DELTA_RAD",
                    0.90,
                ),
            )
            candidate_sum_joint_delta_rad = max(
                candidate_max_joint_delta_rad,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_IK_MAX_JOINT_DELTA_SUM_RAD",
                    2.75,
                ),
            )

            def _joint_delta(
                ref_map: dict[str, float],
                solved_map: dict[str, float],
                joint_name: str,
            ) -> float | None:
                if joint_name not in ref_map or joint_name not in solved_map:
                    return None
                ref_val = float(ref_map[joint_name])
                solved_val = float(solved_map[joint_name])
                if joint_name in self._WRAPAROUND_JOINTS:
                    ref_val = self._normalize_joint_position(joint_name, ref_val)
                    solved_val = self._normalize_joint_position(joint_name, solved_val)
                    return abs(
                        math.atan2(
                            math.sin(solved_val - ref_val),
                            math.cos(solved_val - ref_val),
                        )
                    )
                return abs(solved_val - ref_val)

            best_state: "MoveItRobotState | None" = None
            best_seed = ""
            best_score: tuple[float, float, float, int] | None = None
            best_solved_map: dict[str, float] = {}
            for seed_label, seed_positions in seed_candidates:
                result = self._eval_ik_seed_candidate(
                    robot_model=robot_model,
                    seed_label=seed_label,
                    seed_positions=seed_positions,
                    group_joint_names=group_joint_names,
                    score_reference_map=score_reference_map,
                    seed_reference_label=seed_reference_label,
                    ik_tip=ik_tip,
                    tcp_target_pose=tcp_target_pose,
                    T_base_ee_target=T_base_ee_target,
                    T_model_tcp_target=T_model_tcp_target,
                    candidate_fk_tol_m=candidate_fk_tol_m,
                    candidate_max_joint_delta_rad=candidate_max_joint_delta_rad,
                    candidate_sum_joint_delta_rad=candidate_sum_joint_delta_rad,
                    joint_delta_fn=_joint_delta,
                )
                if result is None:
                    continue
                candidate_score, ik_state, solved_map = result
                if best_score is None or candidate_score < best_score:
                    best_score = candidate_score
                    best_state = ik_state
                    best_seed = seed_label
                    best_solved_map = solved_map
            if best_state is None:
                self.get_logger().warning(
                    f"[APPROACH_IK_SEED] no valid seeded IK candidate "
                    f"tip={ik_tip} target_frame={self._base_frame or 'base_link'} "
                    f"tcp_target_model=({T_model_tcp_target[0,3]:.3f},"
                    f"{T_model_tcp_target[1,3]:.3f},{T_model_tcp_target[2,3]:.3f})"
                )
                return None
            selected_elbow = best_solved_map.get("elbow_joint", None)
            selected_joints = " ".join(
                f"{name}={best_solved_map[name]:.3f}"
                for name in group_joint_names
                if name in best_solved_map
            )
            self.get_logger().info(
                f"[APPROACH_IK_SEED] selected seed={best_seed} "
                f"score_ref={seed_reference_label} elbow="
                f"{f'{selected_elbow:.3f}' if selected_elbow is not None else 'n/a'} "
                f"fk_err={best_score[0]:.3f} "
                f"max_delta={best_score[1]:.3f} sum_delta={best_score[2]:.3f} {selected_joints}"
            )
            return best_state
        except Exception as exc:
            self.get_logger().warning(
                f"[APPROACH_IK_SEED] exception during IK seed compute: {type(exc).__name__}: {exc}"
            )
            return None
