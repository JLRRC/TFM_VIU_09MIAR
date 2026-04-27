"""GoalValidationMixin: comprobación de joint-goal y EE-goal.

Extraído de ``ur5_tools/ur5_moveit_bridge.py`` (líneas 1348-1731 originales).
No mantiene estado propio; todo accede a atributos del nodo principal vía
``self``. La clase concreta debe heredar también de ``rclpy.node.Node``.
"""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.time import Time
from trajectory_msgs.msg import JointTrajectory

try:
    from moveit.core.robot_state import RobotState as MoveItRobotState  # type: ignore
except Exception:  # pragma: no cover - moveit_py opcional en CI
    MoveItRobotState = None  # type: ignore


class GoalValidationMixin:
    """Funciones puras de comprobación de goals (joint y EE)."""

    def _joint_goal_reached(self, jt: JointTrajectory, tol_rad: float = 0.08) -> tuple[bool, str]:
        names = list(getattr(jt, "joint_names", []) or [])
        points = list(getattr(jt, "points", []) or [])
        if not names or not points:
            return False, "goal_check_missing_joint_data"
        target_pos = list(getattr(points[-1], "positions", []) or [])
        if len(target_pos) != len(names):
            return False, "goal_check_target_size_mismatch"

        state_names = list(self._joint_state_last_names or [])
        state_pos = list(self._joint_state_last_positions or [])
        if not state_names or len(state_pos) < len(state_names):
            return False, "goal_check_joint_state_unavailable"

        current = {
            str(name): float(pos)
            for name, pos in zip(state_names, state_pos)
            if str(name or "").strip()
        }
        max_err = 0.0
        worst_joint = ""
        for idx, jname in enumerate(names):
            if jname not in current:
                return False, f"goal_check_missing_joint:{jname}"
            try:
                tgt = float(target_pos[idx])
                cur = float(current[jname])
            except Exception:
                return False, f"goal_check_non_numeric:{jname}"
            if jname in self._WRAPAROUND_JOINTS:
                tgt = self._normalize_joint_position(jname, tgt)
                cur = self._normalize_joint_position(jname, cur)
                err = abs(math.atan2(math.sin(cur - tgt), math.cos(cur - tgt)))
            else:
                err = abs(cur - tgt)
            if err > max_err:
                max_err = err
                worst_joint = jname
            if err > tol_rad:
                return False, f"goal_check_not_reached:{jname}:err={err:.4f}:tol={tol_rad:.4f}"
        return True, f"goal_reached:max_err={max_err:.4f}:joint={worst_joint or 'n/a'}:tol={tol_rad:.4f}"

    def _wait_joint_goal_reached(
        self,
        jt: JointTrajectory,
        *,
        settle_timeout_sec: float = 1.5,
        tol_rad: float = 0.08,
    ) -> tuple[bool, str]:
        deadline = time.monotonic() + max(0.1, float(settle_timeout_sec))
        last_detail = "goal_check_timeout"
        while time.monotonic() <= deadline:
            ok, detail = self._joint_goal_reached(jt, tol_rad=tol_rad)
            if ok:
                return True, detail
            last_detail = detail
            time.sleep(0.05)
        return False, last_detail

    def _joint_motion_since_vector(
        self,
        reference_vec: list[float] | tuple[float, ...] | None,
    ) -> tuple[float | None, str]:
        if reference_vec is None:
            return None, "joint_motion_reference_missing"
        current_vec, reason = self._current_arm_joint_vector()
        if current_vec is None:
            return None, reason
        if len(current_vec) != len(reference_vec):
            return None, "joint_motion_size_mismatch"
        max_delta = 0.0
        worst_joint = ""
        for idx, jname in enumerate(self._START_STATE_JOINTS):
            cur = float(current_vec[idx])
            ref = float(reference_vec[idx])
            if jname in self._WRAPAROUND_JOINTS:
                delta = abs(math.atan2(math.sin(cur - ref), math.cos(cur - ref)))
            else:
                delta = abs(cur - ref)
            if delta > max_delta:
                max_delta = float(delta)
                worst_joint = str(jname)
        return max_delta, f"joint_motion max_delta={max_delta:.4f} joint={worst_joint or 'n/a'}"

    def _ee_target_reached(
        self,
        target_pose: PoseStamped,
        *,
        tol_m: float = 0.10,
    ) -> tuple[bool, str]:
        if target_pose is None:
            return False, "ee_goal_missing_target_pose"
        try:
            target_base = target_pose
            if str(target_pose.header.frame_id or "").strip() != str(self._base_frame or "").strip():
                target_base = self._ensure_base_frame(target_pose)
            if target_base is None:
                return False, "ee_goal_target_tf_unavailable"
            now = Time()
            if not self.tf_buffer.can_transform(
                self._base_frame,
                self._ee_frame,
                now,
                timeout=Duration(seconds=0.1),
            ):
                return False, "ee_goal_current_tf_unavailable"
            transform = self.tf_buffer.lookup_transform(
                self._base_frame,
                self._ee_frame,
                now,
            )
            tcp = transform.transform.translation
            tgt = target_base.pose.position
            dx = float(tcp.x) - float(tgt.x)
            dy = float(tcp.y) - float(tgt.y)
            dz = float(tcp.z) - float(tgt.z)
            dist = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
            if dist <= float(tol_m):
                return (
                    True,
                    "ee_target_reached:"
                    f"dist={dist:.4f}:tol={float(tol_m):.4f}:"
                    f"dx={dx:.4f}:dy={dy:.4f}:dz={dz:.4f}",
                )
            return (
                False,
                "ee_target_not_reached:"
                f"dist={dist:.4f}:tol={float(tol_m):.4f}:"
                f"dx={dx:.4f}:dy={dy:.4f}:dz={dz:.4f}",
            )
        except Exception as exc:
            return False, f"ee_goal_check_exc:{type(exc).__name__}:{exc}"

    def _wait_ee_target_reached(
        self,
        target_pose: PoseStamped,
        *,
        settle_timeout_sec: float = 1.0,
        tol_m: float = 0.10,
    ) -> tuple[bool, str]:
        deadline = time.monotonic() + max(0.1, float(settle_timeout_sec))
        last_detail = "ee_goal_check_timeout"
        while time.monotonic() <= deadline:
            ok, detail = self._ee_target_reached(target_pose, tol_m=tol_m)
            if ok:
                return True, detail
            last_detail = detail
            time.sleep(0.05)
        return False, last_detail

    def _joint_goal_success_consistent_with_ee(
        self,
        *,
        target_pose: PoseStamped | None,
        ee_target_tol_m: float,
        settle_timeout_sec: float,
        joint_detail: str,
        action_name: str,
        source_label: str,
    ) -> tuple[bool, str]:
        if target_pose is None:
            return True, "ee_goal_not_required"
        ee_ok, ee_detail = self._wait_ee_target_reached(
            target_pose,
            settle_timeout_sec=settle_timeout_sec,
            tol_m=ee_target_tol_m,
        )
        if ee_ok:
            return True, ee_detail
        self.get_logger().warning(
            "[BRIDGE_EXEC] joint goal reached but ee target still not reached; "
            f"rejecting success source={source_label} action={action_name} "
            f"joint_detail={joint_detail} ee_detail={ee_detail}"
        )
        return False, ee_detail

    def _planned_ee_pose_from_joint_trajectory(
        self,
        jt: JointTrajectory | None,
    ) -> tuple[PoseStamped | None, str]:
        if jt is None:
            return None, "planned_endpoint_missing_joint_trajectory"
        if self._moveit_py is None:
            return None, "planned_endpoint_moveit_py_unavailable"
        if MoveItRobotState is None:
            return None, "planned_endpoint_robot_state_import_unavailable"
        points = list(getattr(jt, "points", []) or [])
        if not points:
            return None, "planned_endpoint_trajectory_empty"
        joint_names = list(getattr(jt, "joint_names", []) or [])
        final_positions = list(getattr(points[-1], "positions", []) or [])
        if not joint_names or not final_positions:
            return None, "planned_endpoint_joint_data_missing"
        if len(final_positions) < len(joint_names):
            return (
                None,
                f"planned_endpoint_joint_data_short:{len(final_positions)}/{len(joint_names)}",
            )
        joint_map: dict[str, float] = {}
        for name, pos in zip(
            list(self._joint_state_last_names or []),
            list(self._joint_state_last_positions or []),
        ):
            jname = str(name or "").strip()
            if not jname:
                continue
            try:
                value = float(pos)
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                joint_map[jname] = self._normalize_joint_position(jname, value)
        for name, pos in zip(joint_names, final_positions):
            jname = str(name or "").strip()
            if not jname:
                continue
            try:
                value = float(pos)
            except (TypeError, ValueError):
                return None, f"planned_endpoint_joint_non_numeric:{jname}"
            if not math.isfinite(value):
                return None, f"planned_endpoint_joint_non_finite:{jname}"
            joint_map[jname] = self._normalize_joint_position(jname, value)
        try:
            robot_model = self._moveit_py.get_robot_model()
        except Exception as exc:
            return None, f"planned_endpoint_robot_model_unavailable:{type(exc).__name__}:{exc}"
        if robot_model is None:
            return None, "planned_endpoint_robot_model_none"
        try:
            joint_model_group = robot_model.get_joint_model_group(self._group_name)
        except Exception as exc:
            return None, f"planned_endpoint_joint_model_group_unavailable:{type(exc).__name__}:{exc}"
        if joint_model_group is None:
            return None, f"planned_endpoint_joint_model_group_missing:{self._group_name}"
        group_joint_names = list(
            getattr(joint_model_group, "active_joint_model_names", None)
            or getattr(joint_model_group, "joint_model_names", None)
            or self._START_STATE_JOINTS
        )
        missing = [j for j in group_joint_names if j not in joint_map]
        if missing:
            return None, f"planned_endpoint_missing_required:{','.join(missing)}"
        try:
            state = MoveItRobotState(robot_model)
            state.set_joint_group_positions(
                self._group_name,
                np.asarray([joint_map[name] for name in group_joint_names], dtype=float),
            )
            state.update()
        except Exception as exc:
            return None, f"planned_endpoint_fk_failed:{type(exc).__name__}:{exc}"
        base_transform_np, base_detail = self._robot_state_frame_transform_matrix(
            state,
            self._base_frame,
        )
        if base_transform_np is None:
            return None, f"planned_endpoint_base_transform_failed:{base_detail}"
        ee_transform_np, ee_detail = self._robot_state_frame_transform_matrix(
            state,
            self._ee_frame,
        )
        if ee_transform_np is None:
            return None, f"planned_endpoint_transform_failed:{ee_detail}"
        try:
            transform_np = np.linalg.inv(base_transform_np) @ ee_transform_np
        except Exception as exc:
            return None, f"planned_endpoint_relative_transform_failed:{type(exc).__name__}:{exc}"
        pose = PoseStamped()
        pose.header.frame_id = str(self._base_frame or "base_link")
        pose.pose = self._matrix_to_pose(transform_np)
        return (
            pose,
            "planned_endpoint_fk_ok:"
            f"frame={self._base_frame or 'base_link'}:"
            f"x={pose.pose.position.x:.4f}:y={pose.pose.position.y:.4f}:z={pose.pose.position.z:.4f}",
        )

    def _planned_trajectory_target_consistent(
        self,
        trajectory: Any,
        *,
        target_pose: PoseStamped | None,
        tol_m: float,
        phase_label: str | None = None,
        request_uuid: str = "",
    ) -> tuple[bool, str]:
        if target_pose is None:
            return True, "planned_endpoint_target_not_required"
        target_base = target_pose
        if str(target_pose.header.frame_id or "").strip() != str(self._base_frame or "").strip():
            target_base = self._ensure_base_frame(target_pose)
        if target_base is None:
            return False, "planned_endpoint_target_tf_unavailable"
        jt = self._extract_joint_trajectory_msg(trajectory)
        if jt is None:
            return False, "planned_endpoint_joint_trajectory_unavailable"
        planned_pose, planned_detail = self._planned_ee_pose_from_joint_trajectory(jt)
        if planned_pose is None:
            return False, planned_detail
        tgt = target_base.pose.position
        planned = planned_pose.pose.position
        dx = float(planned.x) - float(tgt.x)
        dy = float(planned.y) - float(tgt.y)
        dz = float(planned.z) - float(tgt.z)
        dist = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
        detail = (
            ("planned_ee_target_reached:" if dist <= float(tol_m) else "planned_ee_target_not_reached:")
            + f"dist={dist:.4f}:tol={float(tol_m):.4f}:"
            + f"dx={dx:.4f}:dy={dy:.4f}:dz={dz:.4f}:"
            + f"planned=({float(planned.x):.4f},{float(planned.y):.4f},{float(planned.z):.4f}):"
            + f"target=({float(tgt.x):.4f},{float(tgt.y):.4f},{float(tgt.z):.4f})"
        )
        log_msg = (
            "[PICK][MOVEIT][PLAN_FK] "
            f"phase={phase_label or 'n/a'} request_uuid={request_uuid or 'n/a'} "
            f"ee_frame={self._ee_frame or 'n/a'} "
            f"success={str(dist <= float(tol_m)).lower()} {detail} "
            f"fk_source={planned_detail}"
        )
        if dist <= float(tol_m):
            self.get_logger().info(log_msg)
            return True, detail
        self.get_logger().warning(log_msg)
        return False, detail

    def _feedback_goal_reached(
        self,
        feedback_msg: Any,
        *,
        target_joint_names: list[str],
        target_joint_positions: list[float],
        tol_rad: float,
    ) -> tuple[bool, str]:
        try:
            feedback = getattr(feedback_msg, "feedback", feedback_msg)
        except Exception:
            feedback = feedback_msg
        if feedback is None:
            return False, "feedback_missing"
        try:
            names = list(getattr(feedback, "joint_names", []) or [])
            actual = list(getattr(getattr(feedback, "actual", None), "positions", []) or [])
        except Exception as exc:
            return False, f"feedback_parse_exc:{type(exc).__name__}:{exc}"
        if not names or not actual or not target_joint_names or not target_joint_positions:
            return False, "feedback_joint_data_missing"
        target_map = {}
        for idx, jname in enumerate(target_joint_names):
            if idx >= len(target_joint_positions):
                break
            target_map[str(jname or "").strip()] = float(target_joint_positions[idx])
        max_err = 0.0
        worst_joint = ""
        usable = 0
        for idx in range(min(len(names), len(actual))):
            jname = str(names[idx] or "").strip()
            if not jname or jname not in target_map:
                continue
            try:
                act = float(actual[idx])
                des = float(target_map[jname])
            except Exception:
                return False, f"feedback_non_numeric:{jname or idx}"
            usable += 1
            if jname in self._WRAPAROUND_JOINTS:
                des = self._normalize_joint_position(jname, des)
                act = self._normalize_joint_position(jname, act)
                err = abs(math.atan2(math.sin(act - des), math.cos(act - des)))
            else:
                err = abs(act - des)
            if err > max_err:
                max_err = err
                worst_joint = jname
            if err > float(tol_rad):
                return False, (
                    f"feedback_goal_not_reached:{jname}:err={err:.4f}:tol={float(tol_rad):.4f}"
                )
        if usable <= 0:
            return False, "feedback_joint_match_empty"
        return True, (
            f"feedback_goal_reached:max_err={max_err:.4f}:joint={worst_joint or 'n/a'}:"
            f"tol={float(tol_rad):.4f}"
        )
