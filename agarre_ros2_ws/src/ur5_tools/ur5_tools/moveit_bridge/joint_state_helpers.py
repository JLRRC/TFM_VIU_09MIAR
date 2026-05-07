"""JointStateHelpersMixin: cache de joint_state, robot_state builders, scaling.

Extraído de ``ur5_tools/ur5_moveit_bridge.py`` (líneas 674-1048 originales).
Incluye:

* Callback ``_joint_state_cb`` y cache.
* Constantes de joints (PATH_CONSTRAINT, APPROACH_PATH_CONSTRAINT,
  START_STATE, WRAPAROUND).
* Helper ``_normalize_joint_position`` (convertido a ``classmethod`` para
  que funcione independientemente de la clase concreta que herede el mixin).
* Constraints builder ``_build_joint_path_constraints``.
* Robot state builders (MoveItPy + msg).
* Move group scaling configurator.
* Joint state ready/settled checks.

Sin estado propio; todo accede a atributos del nodo principal vía ``self``.
La clase concreta debe heredar también de ``rclpy.node.Node``.
"""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

from .params import get_moveit_bridge_params as _get_moveit_bridge_params

from moveit_msgs.msg import Constraints, JointConstraint, RobotState as MoveItRobotStateMsg
from sensor_msgs.msg import JointState

try:
    from moveit.core.robot_state import RobotState as MoveItRobotState  # type: ignore
except Exception:  # pragma: no cover - moveit_py opcional en CI
    MoveItRobotState = None  # type: ignore


class JointStateHelpersMixin:
    """Joint-state cache + planning start state + scaling + readiness."""

    _PATH_CONSTRAINT_JOINTS = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
    )

    _APPROACH_PATH_CONSTRAINT_JOINTS = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        # wrist_2_joint removed: MESA has wrist_2 ~ -1.58 rad but APPROACH IK needs
        # wrist_2 ~ 0; the +/-0.35 rad window doesn't span the gap. Constraining pan
        # and lift keeps APPROACH near the live branch without blocking the valid
        # wrist transition needed to center the gripper visually on the object.
    )

    _START_STATE_JOINTS = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )

    _WRAPAROUND_JOINTS = {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    }

    def _joint_state_cb(self, msg: JointState) -> None:
        stamp = getattr(msg, "header", None)
        ts = getattr(stamp, "stamp", None)
        sec = int(getattr(ts, "sec", 0) or 0)
        nsec = int(getattr(ts, "nanosec", 0) or 0)
        self._joint_state_stamp_ns = (sec * 1_000_000_000) + nsec
        self._joint_state_recv_mono = time.monotonic()
        names = list(getattr(msg, "name", []) or [])
        positions = list(getattr(msg, "position", []) or [])
        self._joint_state_names = len(names)
        self._joint_state_positions = len(positions)
        if names and len(positions) >= len(names):
            self._joint_state_last_names = names
            self._joint_state_last_positions = positions[:len(names)]

    @classmethod
    def _normalize_joint_position(cls, joint_name: str, value: float) -> float:
        if joint_name not in cls._WRAPAROUND_JOINTS:
            return value
        return math.atan2(math.sin(value), math.cos(value))

    def _is_skip_constraints_request(self, request_uuid: str) -> bool:
        req = str(request_uuid or "").strip().lower()
        return req.startswith("test_touch:") or req.startswith("skip_constraints:")

    def _build_joint_path_constraints(
        self,
        request_uuid: str = "",
        phase_label: str | None = None,
        tol_override: float | None = None,
    ) -> Constraints | None:
        """Build joint path constraints to prevent IK configuration flipping."""
        if self._is_skip_constraints_request(request_uuid):
            self.get_logger().info(
                "[BRIDGE_CONSTRAINT] skip constraints for tagged request_uuid="
                f"{request_uuid or 'n/a'}"
            )
            self.get_logger().info(
                "[PICK][MOVEIT][CONSTRAINTS] "
                f"phase={str(phase_label or '').strip().upper() or 'n/a'} "
                f"request_uuid={request_uuid or 'n/a'} applied=false "
                "reason=tagged_skip_constraints"
            )
            return None
        phase_upper = str(phase_label or "").strip().upper()
        if phase_upper == "APPROACH":
            if _get_moveit_bridge_params().approach_skip_constraints:
                self.get_logger().info(
                    "[BRIDGE_CONSTRAINT] skip constraints for APPROACH "
                    f"phase={phase_upper} env=1"
                )
                self.get_logger().info(
                    "[PICK][MOVEIT][CONSTRAINTS] "
                    f"phase={phase_upper} request_uuid={request_uuid or 'n/a'} "
                    "applied=false reason=approach_skip_env env=1"
                )
                return None
        tol = float(tol_override) if tol_override is not None else self._path_constraint_joint_tol
        if phase_upper == "APPROACH" and tol_override is None:
            tol = max(
                0.05,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_PATH_CONSTRAINT_TOL_RAD",
                    min(float(self._path_constraint_joint_tol), 0.35),
                ),
            )
        if tol <= 0.0:
            return None
        names = self._joint_state_last_names
        positions = self._joint_state_last_positions
        if not names or not positions:
            return None
        joint_map = dict(zip(names, positions))
        constraints = Constraints()
        constraints.name = "ik_config_lock"
        constrained = []
        constraint_joints = self._PATH_CONSTRAINT_JOINTS
        if phase_upper == "APPROACH":
            constraint_joints = self._APPROACH_PATH_CONSTRAINT_JOINTS
        for jname in constraint_joints:
            if jname not in joint_map:
                continue
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = self._normalize_joint_position(jname, joint_map[jname])
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            constrained.append((jname, jc.position))
        if not constrained:
            return None
        self.get_logger().info(
            f"[BRIDGE_CONSTRAINT] joints={[n for n, _ in constrained]} "
            f"positions={[f'{p:.3f}' for _, p in constrained]} "
            f"tol={tol:.3f}"
        )
        self.get_logger().info(
            "[PICK][MOVEIT][CONSTRAINTS] "
            f"phase={phase_upper or 'n/a'} request_uuid={request_uuid or 'n/a'} "
            f"applied=true joints={[n for n, _ in constrained]} tol_rad={tol:.3f}"
        )
        return constraints

    def _build_start_robot_state(self) -> tuple[Any | None, str]:
        names = list(self._joint_state_last_names or [])
        positions = list(self._joint_state_last_positions or [])
        if not names or not positions:
            return None, "joint_state_cache_empty"
        if len(positions) < len(names):
            return None, f"joint_state_cache_short:{len(positions)}/{len(names)}"
        if self._moveit_py is None:
            return None, "moveit_py_unavailable"
        if MoveItRobotState is None:
            return None, "moveit_robot_state_import_unavailable"
        joint_map: dict[str, float] = {}
        for name, pos in zip(names, positions):
            jname = str(name or "").strip()
            if not jname:
                continue
            try:
                value = float(pos)
            except (TypeError, ValueError):
                return None, f"joint_state_invalid_value:{jname}"
            if not math.isfinite(value):
                return None, f"joint_state_non_finite:{jname}"
            joint_map[jname] = self._normalize_joint_position(jname, value)
        try:
            robot_model = self._moveit_py.get_robot_model()
        except Exception as exc:
            return None, f"robot_model_unavailable:{type(exc).__name__}:{exc}"
        if robot_model is None:
            return None, "robot_model_none"
        try:
            joint_model_group = robot_model.get_joint_model_group(self._group_name)
        except Exception as exc:
            return None, f"joint_model_group_unavailable:{type(exc).__name__}:{exc}"
        if joint_model_group is None:
            return None, f"joint_model_group_missing:{self._group_name}"
        group_joint_names = list(
            getattr(joint_model_group, "active_joint_model_names", None)
            or getattr(joint_model_group, "joint_model_names", None)
            or []
        )
        if not group_joint_names:
            group_joint_names = list(self._START_STATE_JOINTS)
        missing = [j for j in group_joint_names if j not in joint_map]
        if missing:
            return None, f"joint_state_missing_required:{','.join(missing)}"
        try:
            state = MoveItRobotState(robot_model)
            state.set_joint_group_positions(
                self._group_name,
                np.asarray([joint_map[name] for name in group_joint_names], dtype=float),
            )
            state.update()
        except Exception as exc:
            return None, f"robot_state_build_failed:{type(exc).__name__}:{exc}"
        arm_positions = ",".join(
            f"{name}={joint_map[name]:.3f}" for name in self._START_STATE_JOINTS
        )
        return state, f"joint_state_cache_ok:{arm_positions}"

    def _set_planning_start_state_from_joint_state(self) -> tuple[bool, str]:
        if self._planning_component is None:
            return False, "planning_component_unavailable"
        state, reason = self._build_start_robot_state()
        if state is None:
            return False, reason
        try:
            self._planning_component.set_start_state(robot_state=state)
        except TypeError:
            try:
                self._planning_component.set_start_state(state)
            except TypeError:
                self._planning_component.set_start_state(None, state)
        except Exception as exc:
            return False, f"set_start_state_failed:{type(exc).__name__}:{exc}"
        return True, reason

    def _build_start_robot_state_msg(self) -> tuple[MoveItRobotStateMsg | None, str]:
        names = list(self._joint_state_last_names or [])
        positions = list(self._joint_state_last_positions or [])
        if not names or not positions:
            return None, "joint_state_cache_empty"
        if len(positions) < len(names):
            return None, f"joint_state_cache_short:{len(positions)}/{len(names)}"

        joint_state = JointState()
        for name, pos in zip(names, positions):
            jname = str(name or "").strip()
            if not jname:
                continue
            try:
                value = float(pos)
            except (TypeError, ValueError):
                return None, f"joint_state_invalid_value:{jname}"
            if not math.isfinite(value):
                return None, f"joint_state_non_finite:{jname}"
            joint_state.name.append(jname)
            joint_state.position.append(self._normalize_joint_position(jname, value))

        if not joint_state.name or not joint_state.position:
            return None, "joint_state_cache_empty_after_filter"

        state = MoveItRobotStateMsg()
        state.joint_state = joint_state
        state.is_diff = False
        arm_positions = ",".join(
            f"{name}={joint_state.position[joint_state.name.index(name)]:.3f}"
            for name in self._START_STATE_JOINTS
            if name in joint_state.name
        )
        return state, f"joint_state_cache_ok:{arm_positions or 'subset'}"

    def _configure_move_group_scaling(self, group: Any) -> None:
        if group is None:
            return
        try:
            if hasattr(group, "set_max_velocity_scaling_factor"):
                group.set_max_velocity_scaling_factor(float(self._max_velocity_scaling))
            if hasattr(group, "set_max_acceleration_scaling_factor"):
                group.set_max_acceleration_scaling_factor(float(self._max_acceleration_scaling))
        except Exception as exc:
            self.get_logger().warning(
                "[BRIDGE_CFG] no se pudo aplicar scaling en MoveGroup "
                f"type={type(exc).__name__} err={exc}"
            )

    def _joint_state_ready_status(self) -> tuple[bool, str]:
        names = int(self._joint_state_names or 0)
        positions = int(self._joint_state_positions or 0)
        if names <= 0:
            return False, "joint_names_empty"
        if positions <= 0:
            return False, "joint_positions_empty"
        if positions < names:
            return False, f"joint_positions_short:{positions}/{names}"
        if self._joint_state_recv_mono <= 0.0:
            return False, "joint_state_never_received"
        recv_age = max(0.0, time.monotonic() - float(self._joint_state_recv_mono))
        if recv_age > float(self._joint_state_valid_max_age_sec):
            return False, f"joint_state_stale age={recv_age:.2f}s"
        available = set(self._joint_state_last_names or [])
        missing = [j for j in self._START_STATE_JOINTS if j not in available]
        if missing:
            return False, f"joint_state_missing_required:{','.join(missing)}"
        return True, f"joint_state_ok names={names} positions={positions} age={recv_age:.2f}s"

    def _wait_for_valid_joint_state(self, timeout_sec: float) -> tuple[bool, str]:
        timeout = max(0.1, float(timeout_sec))
        deadline = time.monotonic() + timeout
        last_reason = "joint_state_wait_start"
        while time.monotonic() <= deadline:
            ok, reason = self._joint_state_ready_status()
            if ok:
                return True, reason
            last_reason = reason
            time.sleep(0.05)
        return False, f"{last_reason} timeout={timeout:.2f}s"

    def _current_arm_joint_vector(self) -> tuple[list[float], str] | tuple[None, str]:
        names = list(self._joint_state_last_names or [])
        positions = list(self._joint_state_last_positions or [])
        if not names or len(positions) < len(names):
            return None, "joint_state_cache_incomplete"
        current_map = {
            str(name): float(pos)
            for name, pos in zip(names, positions)
            if str(name or "").strip()
        }
        missing = [j for j in self._START_STATE_JOINTS if j not in current_map]
        if missing:
            return None, f"joint_state_missing_required:{','.join(missing)}"
        vec: list[float] = []
        for jname in self._START_STATE_JOINTS:
            value = float(current_map[jname])
            if jname in self._WRAPAROUND_JOINTS:
                value = self._normalize_joint_position(jname, value)
            vec.append(value)
        return vec, "joint_state_vector_ok"

    def _wait_for_joint_state_settled(
        self,
        *,
        timeout_sec: float,
        stable_sec: float = 0.25,
        tol_rad: float = 0.02,
    ) -> tuple[bool, str]:
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        stable_since = 0.0
        last_vec: list[float] | None = None
        last_delta = float("inf")
        last_reason = "joint_state_settle_wait_start"
        while time.monotonic() <= deadline:
            ok, reason = self._joint_state_ready_status()
            if not ok:
                last_reason = reason
                stable_since = 0.0
                time.sleep(0.05)
                continue
            vec, vec_reason = self._current_arm_joint_vector()
            if vec is None:
                last_reason = vec_reason
                stable_since = 0.0
                time.sleep(0.05)
                continue
            now = time.monotonic()
            if last_vec is None:
                last_vec = list(vec)
                stable_since = now
                last_delta = float("inf")
                time.sleep(0.05)
                continue
            deltas: list[float] = []
            for idx, jname in enumerate(self._START_STATE_JOINTS):
                cur = float(vec[idx])
                prev = float(last_vec[idx])
                if jname in self._WRAPAROUND_JOINTS:
                    delta = abs(math.atan2(math.sin(cur - prev), math.cos(cur - prev)))
                else:
                    delta = abs(cur - prev)
                deltas.append(delta)
            last_delta = max(deltas) if deltas else 0.0
            last_vec = list(vec)
            if last_delta <= float(tol_rad):
                if stable_since <= 0.0:
                    stable_since = now
                if (now - stable_since) >= max(0.05, float(stable_sec)):
                    return True, f"joint_state_settled max_delta={last_delta:.4f} tol={float(tol_rad):.4f}"
            else:
                stable_since = 0.0
            last_reason = f"joint_state_not_settled max_delta={last_delta:.4f} tol={float(tol_rad):.4f}"
            time.sleep(0.05)
        return False, f"{last_reason} timeout={max(0.1, float(timeout_sec)):.2f}s"
