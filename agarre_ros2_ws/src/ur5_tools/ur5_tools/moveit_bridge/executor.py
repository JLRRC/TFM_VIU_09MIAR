"""ExecutorMixin: ejecucion via FollowJointTrajectory action client.

Extraído de ``ur5_tools/ur5_moveit_bridge.py`` (líneas 827-2176 originales).

Contiene un único método ``_execute_joint_trajectory_action`` (~1.350 L)
que se invoca a sí mismo recursivamente para retries (slow_factor,
goal_time_tolerance, APPROACH replan). Internamente define un
``feedback_cb`` anidado y maneja:

* Pre-conditions de action server, joint_state ready, controller list.
* Goal preparation (cold-start hold, midpoint blending — vía
  ``TrajectoryPrepMixin._prepare_joint_trajectory_for_controller``).
* Goal sending y feedback streaming.
* Goal acceptance + status polling con timeouts híbridos sim/wall.
* Detección de tolerance violations y retry con slow factor.
* Detección de APPROACH terminal-timeout y signal de replan al planner.
* Validación final (joint goal + EE goal + feedback goal) post-success.
* Cleanup de timeouts compartidos via _pose_lock.

Dependencias principales (vía ``self``):

* ``ControllerManagementMixin``: ensure FJT client, active controllers,
  wait_for_expected_controller_action.
* ``GoalValidationMixin``: joint/EE/feedback goal reached helpers.
* ``JointStateHelpersMixin``: ready/settled, current arm vector, normalize.
* ``TrajectoryPrepMixin``: prepare_joint_trajectory_for_controller,
  scale_joint_trajectory_timing, joint_trajectory_duration_sec.

Sin estado propio. La clase concreta debe heredar también de
``rclpy.node.Node``.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any

from .params import get_moveit_bridge_params as _get_moveit_bridge_params

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory


@dataclass(frozen=True)
class _FjtPollingThresholds:
    """Bundle de tolerancias y umbrales para el polling loop FJT.

    Computado una vez al inicio de _execute_joint_trajectory_action, leído
    repetidamente en el loop. Inmutable para evitar drift accidental.
    """
    goal_check_tol_rad: float
    goal_check_settle_sec: float
    goal_check_poll_sec: float
    feedback_goal_check_tol_rad: float
    feedback_goal_check_settle_sec: float
    allow_feedback_early_success: bool
    allow_joint_early_success: bool
    allow_ee_early_success: bool
    ee_goal_check_tol_m: float
    ee_goal_check_settle_sec: float
    micro_goal_profile: bool
    micro_retry_start_sec: float
    micro_retry_scale: float
    approach_stall_retry_enabled: bool
    approach_stall_retry_start_sec: float
    approach_stall_min_motion_rad: float
    approach_stall_retry_scale: float
    approach_long_retry_enabled: bool
    approach_long_retry_start_sec: float
    approach_long_retry_scale: float
    approach_long_retry_joint_tol_rad: float
    approach_long_retry_ee_tol_m: float
    approach_replan_max_attempts: int
    goal_check_start_sec: float


class ExecutorMixin:
    """FollowJointTrajectory execution loop con retries y validación final."""

    def _compute_basic_check_thresholds(self) -> tuple[float, float, float, float, float]:
        """Tolerancias/settle/poll del joint goal y feedback."""
        goal_check_tol_rad = max(
            0.05,
            self._env_float("PANEL_MOVEIT_BRIDGE_GOAL_CHECK_TOL_RAD", 0.12),
        )
        goal_check_settle_sec = max(
            0.25,
            self._env_float("PANEL_MOVEIT_BRIDGE_GOAL_CHECK_SETTLE_SEC", 0.45),
        )
        goal_check_poll_sec = max(
            0.15,
            self._env_float("PANEL_MOVEIT_BRIDGE_GOAL_CHECK_POLL_SEC", 0.40),
        )
        feedback_goal_check_tol_rad = max(
            goal_check_tol_rad,
            self._env_float("PANEL_MOVEIT_BRIDGE_FEEDBACK_GOAL_TOL_RAD", 0.14),
        )
        feedback_goal_check_settle_sec = max(
            0.20,
            self._env_float("PANEL_MOVEIT_BRIDGE_FEEDBACK_GOAL_SETTLE_SEC", 0.35),
        )
        return (
            goal_check_tol_rad,
            goal_check_settle_sec,
            goal_check_poll_sec,
            feedback_goal_check_tol_rad,
            feedback_goal_check_settle_sec,
        )

    def _compute_ee_check_thresholds(
        self,
        *,
        ee_target_tol_m: float | None,
        phase_label_upper: str,
        goal_check_settle_sec: float,
    ) -> tuple[float, float]:
        """Tolerancia EE y settle, con overrides APPROACH/PRE_GRASP."""
        ee_goal_check_tol_m = max(
            0.02,
            float(ee_target_tol_m)
            if ee_target_tol_m is not None
            else self._env_float("PANEL_MOVEIT_BRIDGE_EE_TARGET_TOL_M", 0.10),
        )
        if phase_label_upper == "APPROACH":
            ee_goal_check_tol_m = max(
                ee_goal_check_tol_m,
                self._env_float("PANEL_MOVEIT_BRIDGE_APPROACH_EE_TARGET_TOL_M", 0.10),
            )
        if phase_label_upper == "PRE_GRASP":
            ee_goal_check_tol_m = max(
                ee_goal_check_tol_m,
                self._env_float("PANEL_MOVEIT_BRIDGE_PREGRASP_EE_TARGET_TOL_M", 0.12),
            )
        ee_goal_check_settle_sec = max(
            0.15,
            self._env_float("PANEL_MOVEIT_BRIDGE_EE_TARGET_SETTLE_SEC", goal_check_settle_sec),
        )
        if phase_label_upper == "APPROACH":
            ee_goal_check_settle_sec = min(
                float(ee_goal_check_settle_sec),
                max(0.15, self._env_float("PANEL_MOVEIT_BRIDGE_APPROACH_EE_TARGET_SETTLE_SEC", 0.25)),
            )
            self.get_logger().info(
                "[BRIDGE_EXEC] approach early-close profile "
                f"ee_goal_check_tol_m={float(ee_goal_check_tol_m):.3f} "
                f"ee_goal_check_settle_sec={float(ee_goal_check_settle_sec):.3f}"
            )
        if phase_label_upper == "PRE_GRASP":
            ee_goal_check_settle_sec = min(
                float(ee_goal_check_settle_sec),
                max(0.15, self._env_float("PANEL_MOVEIT_BRIDGE_PREGRASP_EE_TARGET_SETTLE_SEC", 0.20)),
            )
            self.get_logger().info(
                "[BRIDGE_EXEC] pregrasp early-close profile "
                f"ee_goal_check_tol_m={float(ee_goal_check_tol_m):.3f} "
                f"ee_goal_check_settle_sec={float(ee_goal_check_settle_sec):.3f}"
            )
        return ee_goal_check_tol_m, ee_goal_check_settle_sec

    def _compute_retry_thresholds(
        self,
        *,
        phase_label_upper: str,
        prepared_traj_sec: float,
        start_joint_vec: Any,
        retry_on_tolerance_violation: bool,
        approach_replan_attempt: int,
        goal_check_tol_rad: float,
        ee_goal_check_tol_m: float,
    ) -> dict[str, Any]:
        """Bundle de umbrales de retry: micro + APPROACH stall + APPROACH long."""
        micro_goal_profile = phase_label_upper == "GRASP_DOWN_MICRO_4"
        micro_retry_start_sec = max(
            10.0,
            min(
                30.0,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_MICRO_RETRY_START_SEC",
                    max(18.0, float(prepared_traj_sec) * 0.80),
                ),
            ),
        )
        micro_retry_scale = max(1.2, self._env_float("PANEL_MOVEIT_BRIDGE_MICRO_RETRY_SCALE", 1.6))
        approach_stall_retry_enabled = (
            phase_label_upper == "APPROACH"
            and retry_on_tolerance_violation
            and bool(start_joint_vec is not None)
        )
        approach_stall_retry_start_sec = max(
            8.0, self._env_float("PANEL_MOVEIT_BRIDGE_APPROACH_STALL_RETRY_START_SEC", 12.0)
        )
        approach_stall_min_motion_rad = max(
            0.01, self._env_float("PANEL_MOVEIT_BRIDGE_APPROACH_STALL_MIN_MOTION_RAD", 0.05)
        )
        approach_stall_retry_scale = max(
            1.2, self._env_float("PANEL_MOVEIT_BRIDGE_APPROACH_STALL_RETRY_SCALE", 1.7)
        )
        approach_long_retry_enabled = (
            phase_label_upper == "APPROACH"
            and retry_on_tolerance_violation
            and int(approach_replan_attempt) <= 0
        )
        approach_long_retry_start_sec = max(
            30.0, self._env_float("PANEL_MOVEIT_BRIDGE_APPROACH_LONG_RETRY_START_SEC", 55.0)
        )
        approach_long_retry_scale = max(
            1.2, self._env_float("PANEL_MOVEIT_BRIDGE_APPROACH_LONG_RETRY_SCALE", 1.6)
        )
        approach_long_retry_joint_tol_rad = max(
            goal_check_tol_rad,
            self._env_float("PANEL_MOVEIT_BRIDGE_APPROACH_LONG_RETRY_JOINT_TOL_RAD", 0.35),
        )
        approach_long_retry_ee_tol_m = max(
            ee_goal_check_tol_m,
            self._env_float("PANEL_MOVEIT_BRIDGE_APPROACH_LONG_RETRY_EE_TOL_M", 0.18),
        )
        approach_replan_max_attempts = max(
            1,
            int(round(self._env_float("PANEL_MOVEIT_BRIDGE_APPROACH_REPLAN_MAX_ATTEMPTS", 2.0))),
        )
        return dict(
            micro_goal_profile=micro_goal_profile,
            micro_retry_start_sec=micro_retry_start_sec,
            micro_retry_scale=micro_retry_scale,
            approach_stall_retry_enabled=approach_stall_retry_enabled,
            approach_stall_retry_start_sec=approach_stall_retry_start_sec,
            approach_stall_min_motion_rad=approach_stall_min_motion_rad,
            approach_stall_retry_scale=approach_stall_retry_scale,
            approach_long_retry_enabled=approach_long_retry_enabled,
            approach_long_retry_start_sec=approach_long_retry_start_sec,
            approach_long_retry_scale=approach_long_retry_scale,
            approach_long_retry_joint_tol_rad=approach_long_retry_joint_tol_rad,
            approach_long_retry_ee_tol_m=approach_long_retry_ee_tol_m,
            approach_replan_max_attempts=approach_replan_max_attempts,
        )

    def _compute_goal_check_start_sec(
        self,
        *,
        prepared_traj_sec: float,
        target_pose: Any,
        phase_label_upper: str,
        cold_start_first_goal: bool,
    ) -> float:
        goal_check_start_sec = max(
            4.0, min(25.0, max(4.0, float(prepared_traj_sec) * 0.60))
        )
        if target_pose is not None:
            goal_check_start_sec = min(
                float(goal_check_start_sec),
                max(4.0, self._env_float("PANEL_MOVEIT_BRIDGE_EARLY_TARGET_CHECK_START_SEC", 8.0)),
            )
        if phase_label_upper == "PRE_GRASP":
            goal_check_start_sec = min(
                float(goal_check_start_sec),
                max(4.0, self._env_float(
                    "PANEL_MOVEIT_BRIDGE_PREGRASP_EARLY_TARGET_CHECK_START_SEC", 6.0
                )),
            )
        if cold_start_first_goal:
            goal_check_start_sec = min(
                float(goal_check_start_sec),
                max(4.0, self._env_float("PANEL_MOVEIT_BRIDGE_FIRST_GOAL_CHECK_START_SEC", 10.0)),
            )
        return goal_check_start_sec

    def _compute_fjt_polling_thresholds(
        self,
        *,
        ee_target_tol_m: float | None,
        phase_label_upper: str,
        target_pose: Any,
        cold_start_first_goal: bool,
        prepared_traj_sec: float,
        start_joint_vec: Any,
        retry_on_tolerance_violation: bool,
        approach_replan_attempt: int,
    ) -> _FjtPollingThresholds:
        (
            goal_check_tol_rad,
            goal_check_settle_sec,
            goal_check_poll_sec,
            feedback_goal_check_tol_rad,
            feedback_goal_check_settle_sec,
        ) = self._compute_basic_check_thresholds()
        _mb_params = _get_moveit_bridge_params()
        ee_goal_check_tol_m, ee_goal_check_settle_sec = self._compute_ee_check_thresholds(
            ee_target_tol_m=ee_target_tol_m,
            phase_label_upper=phase_label_upper,
            goal_check_settle_sec=goal_check_settle_sec,
        )
        retry = self._compute_retry_thresholds(
            phase_label_upper=phase_label_upper,
            prepared_traj_sec=prepared_traj_sec,
            start_joint_vec=start_joint_vec,
            retry_on_tolerance_violation=retry_on_tolerance_violation,
            approach_replan_attempt=approach_replan_attempt,
            goal_check_tol_rad=goal_check_tol_rad,
            ee_goal_check_tol_m=ee_goal_check_tol_m,
        )
        goal_check_start_sec = self._compute_goal_check_start_sec(
            prepared_traj_sec=prepared_traj_sec,
            target_pose=target_pose,
            phase_label_upper=phase_label_upper,
            cold_start_first_goal=cold_start_first_goal,
        )
        return _FjtPollingThresholds(
            goal_check_tol_rad=goal_check_tol_rad,
            goal_check_settle_sec=goal_check_settle_sec,
            goal_check_poll_sec=goal_check_poll_sec,
            feedback_goal_check_tol_rad=feedback_goal_check_tol_rad,
            feedback_goal_check_settle_sec=feedback_goal_check_settle_sec,
            allow_feedback_early_success=_mb_params.allow_feedback_early_success,
            allow_joint_early_success=_mb_params.allow_joint_early_success,
            allow_ee_early_success=_mb_params.allow_ee_early_success,
            ee_goal_check_tol_m=ee_goal_check_tol_m,
            ee_goal_check_settle_sec=ee_goal_check_settle_sec,
            goal_check_start_sec=goal_check_start_sec,
            **retry,
        )

    def _make_fjt_feedback_cb(
        self,
        *,
        joint_names: list[str],
        target_joint_positions: list[float],
    ) -> tuple[dict[str, Any], threading.Lock, Any]:
        feedback_lock = threading.Lock()
        feedback_state: dict[str, Any] = {
            "last_mono": 0.0,
            "count": 0,
            "best_detail": "feedback_never_received",
            "best_max_err": float("inf"),
            "stable_since": 0.0,
        }

        def _feedback_cb(feedback_msg: Any) -> None:
            ok = False
            detail = "feedback_goal_timeout"
            max_err = float("inf")
            try:
                ok, detail = self._feedback_goal_reached(
                    feedback_msg,
                    target_joint_names=joint_names,
                    target_joint_positions=target_joint_positions,
                    tol_rad=max(
                        0.08,
                        self._env_float("PANEL_MOVEIT_BRIDGE_FEEDBACK_GOAL_TOL_RAD", 0.14),
                    ),
                )
                try:
                    feedback = getattr(feedback_msg, "feedback", feedback_msg)
                except Exception:
                    feedback = feedback_msg
                names = list(getattr(feedback, "joint_names", []) or [])
                actual = list(getattr(getattr(feedback, "actual", None), "positions", []) or [])
                target_map = {
                    str(jname or "").strip(): float(pos)
                    for jname, pos in zip(joint_names, target_joint_positions)
                    if str(jname or "").strip()
                }
                usable = min(len(names), len(actual))
                if usable > 0:
                    errs = []
                    for idx in range(usable):
                        jname = str(names[idx] or "").strip()
                        if not jname or jname not in target_map:
                            continue
                        des = float(target_map[jname])
                        act = float(actual[idx])
                        if jname in self._WRAPAROUND_JOINTS:
                            des = self._normalize_joint_position(jname, des)
                            act = self._normalize_joint_position(jname, act)
                            err = abs(math.atan2(math.sin(act - des), math.cos(act - des)))
                        else:
                            err = abs(act - des)
                        errs.append(err)
                    if errs:
                        max_err = max(errs)
            except Exception as exc:
                detail = f"feedback_eval_exc:{type(exc).__name__}:{exc}"
            now_mono = time.monotonic()
            with feedback_lock:
                feedback_state["last_mono"] = now_mono
                feedback_state["count"] = int(feedback_state.get("count", 0)) + 1
                feedback_state["last_detail"] = detail
                feedback_state["last_ok"] = bool(ok)
                feedback_state["last_max_err"] = float(max_err)
                best = float(feedback_state.get("best_max_err", float("inf")))
                if max_err < best:
                    feedback_state["best_max_err"] = float(max_err)
                    feedback_state["best_detail"] = detail
                if ok:
                    stable_since = float(feedback_state.get("stable_since", 0.0) or 0.0)
                    if stable_since <= 0.0:
                        feedback_state["stable_since"] = now_mono
                else:
                    feedback_state["stable_since"] = 0.0

        return feedback_state, feedback_lock, _feedback_cb

    def _build_fjt_goal_with_tolerances(
        self,
        jt: JointTrajectory,
        *,
        path_tol_override_rad: float | None,
        effective_goal_time_tol_sec: float,
    ) -> tuple[Any, list[str], list[float]]:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        joint_names: list[str] = []
        target_joint_positions: list[float] = []
        try:
            joint_names = list(getattr(jt, "joint_names", []) or [])
            target_joint_positions = list(getattr((list(getattr(jt, "points", []) or [])[-1]), "positions", []) or [])
            path_tol = (
                float(path_tol_override_rad)
                if path_tol_override_rad is not None
                else float(self._controller_path_tolerance_rad)
            )
            goal_tol = float(self._controller_goal_tolerance_rad)
            goal_time_tol = float(effective_goal_time_tol_sec)
            path_tol_floor = 0.05
            if self._force_fjt_direct_for_walltime_sim:
                path_tol_floor = max(
                    path_tol_floor,
                    self._env_float("PANEL_MOVEIT_BRIDGE_WRAP_PATH_TOL_RAD", 6.50),
                )
            if joint_names and path_tol >= 0.0:
                goal.path_tolerance = [
                    JointTolerance(name=str(jn), position=float(max(path_tol_floor, path_tol)))
                    for jn in joint_names
                ]
            if joint_names and goal_tol >= 0.0:
                goal.goal_tolerance = [
                    JointTolerance(name=str(jn), position=float(max(0.05, goal_tol)))
                    for jn in joint_names
                ]
            if goal_time_tol >= 0.0:
                total = max(0.0, float(goal_time_tol))
                sec = int(total)
                nsec = int(round((total - sec) * 1_000_000_000.0))
                if nsec >= 1_000_000_000:
                    sec += 1
                    nsec -= 1_000_000_000
                goal.goal_time_tolerance.sec = sec
                goal.goal_time_tolerance.nanosec = nsec
        except Exception as tol_exc:
            self.get_logger().warning(
                "[BRIDGE_EXEC] no se pudieron fijar tolerancias FJT explicitas: "
                f"{type(tol_exc).__name__}: {tol_exc}"
            )
        return goal, joint_names, target_joint_positions

    def _maybe_scale_approach_replan_traj(
        self,
        jt: JointTrajectory,
        *,
        prepared_traj_sec: float,
        phase_label_upper: str,
        approach_replan_attempt: int,
    ) -> tuple[JointTrajectory, float]:
        if phase_label_upper != "APPROACH" or int(approach_replan_attempt) < 1:
            return jt, prepared_traj_sec
        approach_replan_min_traj_sec = max(
            30.0,
            self._env_float(
                "PANEL_MOVEIT_BRIDGE_APPROACH_REPLAN_MIN_TRAJ_SEC",
                45.0,
            ),
        )
        if prepared_traj_sec + 1e-6 >= approach_replan_min_traj_sec:
            return jt, prepared_traj_sec
        replan_scale = max(
            1.0,
            float(approach_replan_min_traj_sec) / max(0.001, float(prepared_traj_sec)),
        )
        jt = self._scale_joint_trajectory_timing(jt, scale=replan_scale)
        prepared_traj_sec = self._joint_trajectory_duration_sec(jt)
        self.get_logger().warning(
            "[BRIDGE_EXEC] approach replan trajectory duration raised "
            f"attempt={int(approach_replan_attempt)} "
            f"scale={replan_scale:.2f} min_traj_sec={float(approach_replan_min_traj_sec):.1f} "
            f"prepared_traj_sec={float(prepared_traj_sec):.3f}"
        )
        return jt, prepared_traj_sec

    def _compute_effective_goal_time_tol_sec(
        self,
        *,
        goal_time_override_sec: float | None,
        phase_label_upper: str,
        approach_replan_attempt: int,
    ) -> float:
        effective = (
            float(goal_time_override_sec)
            if goal_time_override_sec is not None
            else float(self._controller_goal_time_tolerance_sec)
        )
        if phase_label_upper == "APPROACH":
            effective = max(
                effective,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_GOAL_TIME_TOL_SEC",
                    75.0,
                ),
            )
            if int(approach_replan_attempt) >= 1:
                effective = max(
                    effective,
                    self._env_float(
                        "PANEL_MOVEIT_BRIDGE_APPROACH_REPLAN_GOAL_TIME_TOL_SEC",
                        120.0,
                    ),
                )
        return float(effective)

    def _execute_joint_trajectory_action(
        self,
        jt: JointTrajectory,
        *,
        timeout_sec: float = 8.0,
        retry_on_tolerance_violation: bool = True,
        path_tol_override_rad: float | None = None,
        goal_time_override_sec: float | None = None,
        target_pose: PoseStamped | None = None,
        ee_target_tol_m: float | None = None,
        phase_label: str | None = None,
        approach_replan_attempt: int = 0,
    ) -> tuple[bool, str, dict[str, Any]]:
        cold_start_first_goal = False
        try:
            with self._pose_lock:
                cold_start_first_goal = bool(self._first_controller_goal_pending)
        except Exception:
            cold_start_first_goal = False
        phase_label_upper = str(phase_label or "").upper()
        approach_max_total_timeout_sec = float(
            self._env_float(
                "PANEL_MOVEIT_BRIDGE_APPROACH_MAX_TOTAL_TIMEOUT_SEC",
                600.0,  # FIX: aumentado de 120 a 600s para sim con RTF bajo
            )
        )
        effective_goal_time_tol_sec = self._compute_effective_goal_time_tol_sec(
            goal_time_override_sec=goal_time_override_sec,
            phase_label_upper=phase_label_upper,
            approach_replan_attempt=approach_replan_attempt,
        )
        if retry_on_tolerance_violation and self._force_fjt_direct_for_walltime_sim:
            pre_scale = 2.0
            jt = self._scale_joint_trajectory_timing(jt, scale=pre_scale)
            self.get_logger().info(
                "[BRIDGE_EXEC] pre-scaling controller trajectory "
                f"scale={pre_scale:.1f} reason=sim_tracking_margin"
            )
        jt = self._prepare_joint_trajectory_for_controller(
            jt,
            force_cold_start_hold=cold_start_first_goal,
        )
        prepared_traj_sec = self._joint_trajectory_duration_sec(jt)
        jt, prepared_traj_sec = self._maybe_scale_approach_replan_traj(
            jt,
            prepared_traj_sec=prepared_traj_sec,
            phase_label_upper=phase_label_upper,
            approach_replan_attempt=approach_replan_attempt,
        )
        prepared_timeout = self._fjt_timeout_for_trajectory(
            prepared_traj_sec,
            extra_margin_sec=8.0,
            minimum_sec=max(8.0, float(timeout_sec)),
            first_goal=cold_start_first_goal,
            controller_goal_time_sec_override=effective_goal_time_tol_sec,
        )
        if prepared_timeout > float(timeout_sec) + 1e-6:
            self.get_logger().info(
                "[BRIDGE_EXEC] adjusted action timeout after controller prep "
                f"from={float(timeout_sec):.3f}s to={float(prepared_timeout):.3f}s "
                f"traj_sec={float(prepared_traj_sec):.3f}"
            )
            timeout_sec = float(prepared_timeout)
        if phase_label_upper == "APPROACH" and float(timeout_sec) > approach_max_total_timeout_sec:
            self.get_logger().warning(
                "[BRIDGE_EXEC] APPROACH timeout capped "
                f"original={float(timeout_sec):.1f} max={approach_max_total_timeout_sec:.1f}"
            )
            timeout_sec = float(approach_max_total_timeout_sec)
        ready, action_name, action_names, candidates = self._wait_for_expected_controller_action(
            timeout_sec=1.5
        )
        if not ready or not action_name:
            # F3-step41a (2026-05-08): meta + reason delegados a no_server_meta
            # (puros, testeables offline).
            from .no_server_meta import build_no_server_meta, build_no_server_reason
            meta = build_no_server_meta(
                expected_action=self._controller_action_name,
                available_actions=action_names,
                candidates=candidates,
            )
            reason = build_no_server_reason(
                expected_action=self._controller_action_name,
                available_actions=action_names,
                candidates=candidates,
            )
            return False, reason, meta

        client = self._ensure_fjt_action_client()
        if client is None:
            return False, "fjt_action_client_unavailable", {"action": action_name}
        goal, joint_names, target_joint_positions = self._build_fjt_goal_with_tolerances(
            jt,
            path_tol_override_rad=path_tol_override_rad,
            effective_goal_time_tol_sec=effective_goal_time_tol_sec,
        )
        feedback_state, feedback_lock, _feedback_cb = self._make_fjt_feedback_cb(
            joint_names=joint_names,
            target_joint_positions=target_joint_positions,
        )

        send_future = client.send_goal_async(goal, feedback_callback=_feedback_cb)
        if not self._wait_future_done(send_future, timeout_sec=min(2.0, timeout_sec)):
            return False, "fjt_goal_send_timeout", {"action": action_name}
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return (
                False,
                "fjt_goal_rejected",
                {"action": action_name, "accepted": bool(getattr(goal_handle, "accepted", False))},
            )
        if cold_start_first_goal:
            try:
                with self._pose_lock:
                    self._first_controller_goal_pending = False
            except Exception:
                pass
        start_joint_vec, start_joint_reason = self._current_arm_joint_vector()
        if start_joint_vec is None:
            self.get_logger().warning(
                "[BRIDGE_EXEC] start joint snapshot unavailable "
                f"action={action_name} reason={start_joint_reason}"
            )

        exec_deadline_mono = time.monotonic() + max(1.0, float(timeout_sec))
        with self._pose_lock:
            self._active_exec_timeout_sec = float(timeout_sec)
            self._active_exec_timeout_deadline_mono = float(exec_deadline_mono)
        try:
            result_future = goal_handle.get_result_async()
            _th = self._compute_fjt_polling_thresholds(
                ee_target_tol_m=ee_target_tol_m,
                phase_label_upper=phase_label_upper,
                target_pose=target_pose,
                cold_start_first_goal=cold_start_first_goal,
                prepared_traj_sec=prepared_traj_sec,
                start_joint_vec=start_joint_vec,
                retry_on_tolerance_violation=retry_on_tolerance_violation,
                approach_replan_attempt=approach_replan_attempt,
            )
            goal_check_tol_rad = _th.goal_check_tol_rad
            goal_check_settle_sec = _th.goal_check_settle_sec
            goal_check_poll_sec = _th.goal_check_poll_sec
            feedback_goal_check_tol_rad = _th.feedback_goal_check_tol_rad
            feedback_goal_check_settle_sec = _th.feedback_goal_check_settle_sec
            allow_feedback_early_success = _th.allow_feedback_early_success
            allow_joint_early_success = _th.allow_joint_early_success
            allow_ee_early_success = _th.allow_ee_early_success
            ee_goal_check_tol_m = _th.ee_goal_check_tol_m
            ee_goal_check_settle_sec = _th.ee_goal_check_settle_sec
            micro_goal_profile = _th.micro_goal_profile
            micro_retry_start_sec = _th.micro_retry_start_sec
            micro_retry_scale = _th.micro_retry_scale
            approach_stall_retry_enabled = _th.approach_stall_retry_enabled
            approach_stall_retry_start_sec = _th.approach_stall_retry_start_sec
            approach_stall_min_motion_rad = _th.approach_stall_min_motion_rad
            approach_stall_retry_scale = _th.approach_stall_retry_scale
            approach_long_retry_enabled = _th.approach_long_retry_enabled
            approach_long_retry_start_sec = _th.approach_long_retry_start_sec
            approach_long_retry_scale = _th.approach_long_retry_scale
            approach_long_retry_joint_tol_rad = _th.approach_long_retry_joint_tol_rad
            approach_long_retry_ee_tol_m = _th.approach_long_retry_ee_tol_m
            approach_replan_max_attempts = _th.approach_replan_max_attempts
            goal_check_start_sec = _th.goal_check_start_sec
            result_wait_deadline = time.monotonic() + max(1.0, float(timeout_sec))
            result_wait_started = time.monotonic()
            last_goal_check_mono = 0.0
            last_ee_check_mono = 0.0
            last_feedback_check_mono = 0.0
            while not result_future.done():
                now_mono = time.monotonic()
                if now_mono >= result_wait_deadline:
                    break
                try:
                    gh_status = int(
                        getattr(goal_handle, "status", GoalStatus.STATUS_UNKNOWN)
                        or GoalStatus.STATUS_UNKNOWN
                    )
                    if gh_status == GoalStatus.STATUS_SUCCEEDED:
                        consistent_with_ee, ee_consistency_detail = (
                            self._joint_goal_success_consistent_with_ee(
                                target_pose=target_pose,
                                ee_target_tol_m=ee_goal_check_tol_m,
                                settle_timeout_sec=ee_goal_check_settle_sec,
                                joint_detail="gh_status_succeeded",
                                action_name=action_name,
                                source_label="goal_handle_status_succeeded",
                            )
                        )
                        if consistent_with_ee:
                            self.get_logger().info(
                                "[BRIDGE_EXEC] FJT succeeded via goal_handle.status; retorno directo "
                                f"action={action_name} gh_status={gh_status} "
                                f"elapsed={now_mono - result_wait_started:.1f}s"
                            )
                            return True, f"fjt_gh_status_succeeded:{action_name}", {
                                "action": action_name,
                                "status_text": "GH_STATUS_SUCCEEDED",
                                "elapsed_sec": round(now_mono - result_wait_started, 1),
                                "ee_goal_check": ee_consistency_detail,
                            }
                        self.get_logger().warning(
                            "[BRIDGE_EXEC] goal_handle.status=SUCCEEDED but ee target is still inconsistent; "
                            f"keeping wait action={action_name} ee_detail={ee_consistency_detail}"
                        )
                    if gh_status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
                        self.get_logger().info(
                            "[BRIDGE_EXEC] FJT terminal via goal_handle.status; saliendo loop "
                            f"action={action_name} gh_status={gh_status} "
                            f"elapsed={now_mono - result_wait_started:.1f}s"
                        )
                        break
                except Exception:
                    pass
                if (
                    allow_feedback_early_success
                    and
                    (now_mono - result_wait_started) >= max(3.0, goal_check_start_sec * 0.5)
                    and (now_mono - last_feedback_check_mono) >= goal_check_poll_sec
                ):
                    last_feedback_check_mono = now_mono
                    with feedback_lock:
                        feedback_count = int(feedback_state.get("count", 0) or 0)
                        feedback_last_mono = float(feedback_state.get("last_mono", 0.0) or 0.0)
                        feedback_stable_since = float(
                            feedback_state.get("stable_since", 0.0) or 0.0
                        )
                        feedback_detail = str(
                            feedback_state.get("last_detail")
                            or feedback_state.get("best_detail")
                            or "feedback_never_received"
                        )
                    if feedback_count > 0 and feedback_stable_since > 0.0:
                        stable_for = max(0.0, now_mono - feedback_stable_since)
                        if stable_for >= feedback_goal_check_settle_sec:
                            consistent_with_ee, ee_consistency_detail = (
                                self._joint_goal_success_consistent_with_ee(
                                    target_pose=target_pose,
                                    ee_target_tol_m=ee_goal_check_tol_m,
                                    settle_timeout_sec=ee_goal_check_settle_sec,
                                    joint_detail=feedback_detail,
                                    action_name=action_name,
                                    source_label="feedback_goal_reached_before_result",
                                )
                            )
                            if not consistent_with_ee:
                                continue
                            meta = {
                                "action": action_name,
                                "status_text": "FEEDBACK_GOAL_REACHED_BEFORE_RESULT",
                                "feedback_goal_check": feedback_detail,
                                "ee_goal_check": ee_consistency_detail,
                                "feedback_goal_check_tol_rad": round(
                                    float(feedback_goal_check_tol_rad), 4
                                ),
                            }
                            self.get_logger().info(
                                "[BRIDGE_EXEC] FollowJointTrajectory feedback goal reached before terminal result "
                                f"action={action_name} detail={feedback_detail}"
                            )
                            return True, f"fjt_feedback_goal_reached_before_result:{feedback_detail}", meta
                    if (
                        feedback_count > 0
                        and feedback_last_mono > 0.0
                        and (now_mono - feedback_last_mono)
                        >= max(
                            4.0,
                            self._env_float(
                                "PANEL_MOVEIT_BRIDGE_FEEDBACK_STALE_FAIL_SEC",
                                8.0,
                            ),
                        )
                    ):
                        joint_near, joint_near_detail = self._joint_goal_reached(
                            jt,
                            tol_rad=max(goal_check_tol_rad, feedback_goal_check_tol_rad),
                        )
                        ee_near = False
                        ee_near_detail = "ee_goal_not_checked"
                        if target_pose is not None:
                            ee_near, ee_near_detail = self._ee_target_reached(
                                target_pose,
                                tol_m=ee_goal_check_tol_m,
                            )
                        motion_delta, motion_detail = self._joint_motion_since_vector(
                            start_joint_vec,
                        )
                        if joint_near:
                            meta = {
                                "action": action_name,
                                "status_text": "FEEDBACK_STALE_GOAL_REACHED",
                                "feedback_goal_check": feedback_detail,
                                "joint_goal_check": joint_near_detail,
                                "feedback_count": feedback_count,
                            }
                            self.get_logger().warning(
                                "[BRIDGE_EXEC] FollowJointTrajectory feedback stale "
                                "pero el goal articular ya esta alcanzado; aceptando success "
                                f"action={action_name} detail={joint_near_detail}"
                            )
                            return True, (
                                "fjt_feedback_stale_goal_reached:"
                                f"{feedback_detail};{joint_near_detail}"
                            ), meta
                        if target_pose is not None and ee_near:
                            meta = {
                                "action": action_name,
                                "status_text": "FEEDBACK_STALE_EE_TARGET_REACHED",
                                "feedback_goal_check": feedback_detail,
                                "ee_goal_check": ee_near_detail,
                                "feedback_count": feedback_count,
                            }
                            self.get_logger().warning(
                                "[BRIDGE_EXEC] FollowJointTrajectory feedback stale "
                                "pero el ee target ya esta alcanzado; aceptando success "
                                f"action={action_name} detail={ee_near_detail}"
                            )
                            return True, (
                                "fjt_feedback_stale_ee_target_reached:"
                                f"{feedback_detail};{ee_near_detail}"
                            ), meta
                        try:
                            goal_handle.cancel_goal_async()
                        except Exception:
                            pass
                        meta = {
                            "action": action_name,
                            "status_text": "FEEDBACK_STALE",
                            "feedback_goal_check": feedback_detail,
                            "feedback_count": feedback_count,
                            "joint_goal_check": joint_near_detail,
                            "ee_goal_check": ee_near_detail,
                            "joint_motion": motion_detail,
                        }
                        if (
                            phase_label_upper == "APPROACH"
                            and int(approach_replan_attempt) < int(approach_replan_max_attempts)
                        ):
                            meta["status_text"] = "APPROACH_REPLAN_FROM_CURRENT_STATE"
                            self.get_logger().warning(
                                "[BRIDGE_EXEC] FollowJointTrajectory feedback stale during APPROACH; "
                                "replanificando desde el estado actual "
                                f"attempt={int(approach_replan_attempt) + 1}/"
                                f"{int(approach_replan_max_attempts)} "
                                f"feedback_detail={feedback_detail} "
                                f"joint_detail={joint_near_detail} "
                                f"ee_detail={ee_near_detail} "
                                f"{motion_detail}"
                            )
                            return (
                                False,
                                "fjt_approach_replan_from_current_state:"
                                f"feedback_stale:{feedback_detail};"
                                f"{joint_near_detail};{ee_near_detail};{motion_detail}",
                                meta,
                            )
                        self.get_logger().warning(
                            "[BRIDGE_EXEC] FollowJointTrajectory feedback stale; cerrando fallo terminal "
                            f"action={action_name} detail={feedback_detail} "
                            f"joint_detail={joint_near_detail} ee_detail={ee_near_detail} "
                            f"{motion_detail}"
                        )
                        return (
                            False,
                            "fjt_feedback_stale:"
                            f"{feedback_detail};{joint_near_detail};{ee_near_detail};{motion_detail}",
                            meta,
                        )
                if (
                    allow_joint_early_success
                    and
                    (now_mono - result_wait_started) >= goal_check_start_sec
                    and (now_mono - last_goal_check_mono) >= goal_check_poll_sec
                ):
                    last_goal_check_mono = now_mono
                    reached_early, reached_early_detail = self._wait_joint_goal_reached(
                        jt,
                        settle_timeout_sec=goal_check_settle_sec,
                        tol_rad=goal_check_tol_rad,
                    )
                    if reached_early:
                        consistent_with_ee, ee_consistency_detail = (
                            self._joint_goal_success_consistent_with_ee(
                                target_pose=target_pose,
                                ee_target_tol_m=ee_goal_check_tol_m,
                                settle_timeout_sec=ee_goal_check_settle_sec,
                                joint_detail=reached_early_detail,
                                action_name=action_name,
                                source_label="goal_reached_before_result",
                            )
                        )
                        if not consistent_with_ee:
                            continue
                        meta = {
                            "action": action_name,
                            "status_text": "GOAL_REACHED_BEFORE_RESULT",
                            "goal_check": reached_early_detail,
                            "ee_goal_check": ee_consistency_detail,
                            "goal_check_tol_rad": round(float(goal_check_tol_rad), 4),
                        }
                        self.get_logger().info(
                            "[BRIDGE_EXEC] FollowJointTrajectory goal reached before terminal result "
                            f"action={action_name} detail={reached_early_detail}"
                        )
                        return True, f"fjt_goal_reached_before_result:{reached_early_detail}", meta
                if (
                    allow_ee_early_success
                    and
                    target_pose is not None
                    and (now_mono - result_wait_started) >= goal_check_start_sec
                    and (now_mono - last_ee_check_mono) >= goal_check_poll_sec
                ):
                    last_ee_check_mono = now_mono
                    ee_reached, ee_reached_detail = self._wait_ee_target_reached(
                        target_pose,
                        settle_timeout_sec=ee_goal_check_settle_sec,
                        tol_m=ee_goal_check_tol_m,
                    )
                    if ee_reached:
                        meta = {
                            "action": action_name,
                            "status_text": "EE_TARGET_REACHED_BEFORE_RESULT",
                            "ee_goal_check": ee_reached_detail,
                            "ee_goal_check_tol_m": round(float(ee_goal_check_tol_m), 4),
                        }
                        self.get_logger().info(
                            "[BRIDGE_EXEC] FollowJointTrajectory ee target reached before terminal result "
                            f"action={action_name} detail={ee_reached_detail}"
                        )
                        return True, f"fjt_ee_target_reached_before_result:{ee_reached_detail}", meta
                if (
                    retry_on_tolerance_violation
                    and micro_goal_profile
                    and (now_mono - result_wait_started) >= micro_retry_start_sec
                ):
                    try:
                        goal_handle.cancel_goal_async()
                    except Exception:
                        pass
                    slowed = self._scale_joint_trajectory_timing(jt, scale=micro_retry_scale)
                    retry_timeout = max(
                        float(timeout_sec),
                        self._joint_trajectory_duration_sec(slowed) + 12.0,
                    )
                    retry_goal_time = max(
                        12.0,
                        min(30.0, self._joint_trajectory_duration_sec(slowed) + 6.0),
                    )
                    self.get_logger().warning(
                        "[BRIDGE_EXEC] micro-goal still_waiting early retry "
                        f"phase={phase_label or 'n/a'} elapsed={now_mono - result_wait_started:.1f}s "
                        f"scale={micro_retry_scale:.2f} retry_timeout_sec={retry_timeout:.1f} "
                        f"retry_goal_time_tol={retry_goal_time:.1f}"
                    )
                    return self._execute_joint_trajectory_action(
                        slowed,
                        timeout_sec=retry_timeout,
                        retry_on_tolerance_violation=False,
                        path_tol_override_rad=path_tol_override_rad,
                        goal_time_override_sec=retry_goal_time,
                        target_pose=target_pose,
                        ee_target_tol_m=ee_target_tol_m,
                        phase_label=phase_label,
                        approach_replan_attempt=approach_replan_attempt,
                    )
                if (
                    approach_stall_retry_enabled
                    and (now_mono - result_wait_started) >= approach_stall_retry_start_sec
                ):
                    with feedback_lock:
                        feedback_count = int(feedback_state.get("count", 0) or 0)
                        feedback_last_detail = str(
                            feedback_state.get("last_detail")
                            or feedback_state.get("best_detail")
                            or "feedback_never_received"
                        )
                    motion_delta, motion_detail = self._joint_motion_since_vector(start_joint_vec)
                    if (
                        feedback_count <= 0
                        and motion_delta is not None
                        and float(motion_delta) < float(approach_stall_min_motion_rad)
                    ):
                        try:
                            goal_handle.cancel_goal_async()
                        except Exception:
                            pass
                        slowed = self._scale_joint_trajectory_timing(
                            jt,
                            scale=approach_stall_retry_scale,
                        )
                        retry_timeout = max(
                            float(timeout_sec) * approach_stall_retry_scale,
                            self._joint_trajectory_duration_sec(slowed) + 16.0,
                        )
                        if phase_label_upper == "APPROACH" and retry_timeout > approach_max_total_timeout_sec:
                            retry_timeout = float(approach_max_total_timeout_sec)
                        retry_goal_time = max(
                            float(effective_goal_time_tol_sec),
                            self._env_float(
                                "PANEL_MOVEIT_BRIDGE_APPROACH_STALL_RETRY_GOAL_TIME_SEC",
                                90.0,
                            ),
                        )
                        self.get_logger().warning(
                            "[BRIDGE_EXEC] approach stall early retry "
                            f"action={action_name} elapsed={now_mono - result_wait_started:.1f}s "
                            f"{motion_detail} feedback_count={feedback_count} "
                            f"feedback_detail={feedback_last_detail} "
                            f"scale={approach_stall_retry_scale:.2f} "
                            f"retry_timeout_sec={retry_timeout:.1f} "
                            f"retry_goal_time_tol={retry_goal_time:.1f}"
                        )
                        return self._execute_joint_trajectory_action(
                            slowed,
                            timeout_sec=retry_timeout,
                            retry_on_tolerance_violation=False,
                            path_tol_override_rad=path_tol_override_rad,
                            goal_time_override_sec=retry_goal_time,
                            target_pose=target_pose,
                            ee_target_tol_m=ee_target_tol_m,
                            phase_label=phase_label,
                            approach_replan_attempt=approach_replan_attempt,
                        )
                if (
                    approach_long_retry_enabled
                    and (now_mono - result_wait_started) >= approach_long_retry_start_sec
                ):
                    joint_near, joint_near_detail = self._joint_goal_reached(
                        jt,
                        tol_rad=approach_long_retry_joint_tol_rad,
                    )
                    ee_near = False
                    ee_near_detail = "ee_goal_not_checked"
                    if target_pose is not None:
                        ee_near, ee_near_detail = self._ee_target_reached(
                            target_pose,
                            tol_m=approach_long_retry_ee_tol_m,
                        )
                    if not joint_near and not ee_near:
                        with feedback_lock:
                            feedback_count = int(feedback_state.get("count", 0) or 0)
                            feedback_last_detail = str(
                                feedback_state.get("last_detail")
                                or feedback_state.get("best_detail")
                                or "feedback_never_received"
                            )
                        motion_delta, motion_detail = self._joint_motion_since_vector(start_joint_vec)
                        try:
                            goal_handle.cancel_goal_async()
                        except Exception:
                            pass
                        slowed = self._scale_joint_trajectory_timing(
                            jt,
                            scale=approach_long_retry_scale,
                        )
                        retry_goal_time = max(
                            float(effective_goal_time_tol_sec),
                            self._env_float(
                                "PANEL_MOVEIT_BRIDGE_APPROACH_LONG_RETRY_GOAL_TIME_SEC",
                                120.0,
                            ),
                        )
                        retry_timeout = max(
                            float(timeout_sec),
                            self._joint_trajectory_duration_sec(slowed)
                            + float(retry_goal_time)
                            + 16.0,
                        )
                        if phase_label_upper == "APPROACH" and retry_timeout > approach_max_total_timeout_sec:
                            retry_timeout = float(approach_max_total_timeout_sec)
                        approach_internal_replan_enabled = _get_moveit_bridge_params().approach_internal_replan
                        if not (phase_label_upper == "APPROACH" and approach_internal_replan_enabled):
                            self.get_logger().warning(
                                "[BRIDGE_EXEC] approach long-wait terminal failure "
                                f"action={action_name} elapsed={now_mono - result_wait_started:.1f}s "
                                f"joint_detail={joint_near_detail} ee_detail={ee_near_detail} "
                                f"{motion_detail} feedback_count={feedback_count} "
                                f"feedback_detail={feedback_last_detail} "
                                f"scale={approach_long_retry_scale:.2f} "
                                f"retry_timeout_sec={retry_timeout:.1f} "
                                f"retry_goal_time_tol={retry_goal_time:.1f}"
                            )
                            return (
                                False,
                                "fjt_approach_long_wait_terminal:"
                                f"{joint_near_detail};{ee_near_detail};{motion_detail}",
                                {
                                    "action": action_name,
                                    "status_text": "APPROACH_LONG_WAIT_TERMINAL",
                                    "joint_goal_check": joint_near_detail,
                                    "ee_goal_check": ee_near_detail,
                                    "joint_motion": motion_detail,
                                    "feedback_goal_check": feedback_last_detail,
                                    "retry_timeout_sec": round(float(retry_timeout), 3),
                                    "retry_goal_time_tol_sec": round(float(retry_goal_time), 3),
                                },
                            )
                        self.get_logger().warning(
                            "[BRIDGE_EXEC] approach long-wait requires replan from current state "
                            f"action={action_name} elapsed={now_mono - result_wait_started:.1f}s "
                            f"joint_detail={joint_near_detail} ee_detail={ee_near_detail} "
                            f"{motion_detail} feedback_count={feedback_count} "
                            f"feedback_detail={feedback_last_detail} "
                            f"scale={approach_long_retry_scale:.2f} "
                            f"retry_timeout_sec={retry_timeout:.1f} "
                            f"retry_goal_time_tol={retry_goal_time:.1f}"
                        )
                        return (
                            False,
                            "fjt_approach_replan_from_current_state:"
                            f"{joint_near_detail};{ee_near_detail};{motion_detail}",
                            {
                                "action": action_name,
                                "status_text": "APPROACH_REPLAN_FROM_CURRENT_STATE",
                                "joint_goal_check": joint_near_detail,
                                "ee_goal_check": ee_near_detail,
                                "joint_motion": motion_detail,
                                "feedback_goal_check": feedback_last_detail,
                                "retry_timeout_sec": round(float(retry_timeout), 3),
                                "retry_goal_time_tol_sec": round(float(retry_goal_time), 3),
                            },
                        )
                time.sleep(0.05)
            if not result_future.done():
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                if target_pose is not None:
                    ee_reached_after_timeout, ee_reached_after_timeout_detail = self._wait_ee_target_reached(
                        target_pose,
                        settle_timeout_sec=1.5,
                        tol_m=ee_goal_check_tol_m,
                    )
                    if ee_reached_after_timeout:
                        meta = {
                            "action": action_name,
                            "status_text": "TIMEOUT_EE_TARGET_REACHED",
                            "timeout_sec": round(float(timeout_sec), 3),
                            "ee_goal_check": ee_reached_after_timeout_detail,
                        }
                        self.get_logger().warning(
                            "[BRIDGE_EXEC] FollowJointTrajectory TIMEOUT pero ee target alcanzado "
                            f"action={action_name} detail={ee_reached_after_timeout_detail}"
                        )
                        return True, f"fjt_timeout_ee_target_reached:{ee_reached_after_timeout_detail}", meta
                reached, reached_detail = self._wait_joint_goal_reached(
                    jt,
                    settle_timeout_sec=1.5,
                    tol_rad=goal_check_tol_rad,
                )
                if reached:
                    consistent_with_ee, ee_consistency_detail = (
                        self._joint_goal_success_consistent_with_ee(
                            target_pose=target_pose,
                            ee_target_tol_m=ee_goal_check_tol_m,
                            settle_timeout_sec=1.0,
                            joint_detail=reached_detail,
                            action_name=action_name,
                            source_label="timeout_goal_reached",
                        )
                    )
                    if not consistent_with_ee:
                        reached = False
                    else:
                        meta = {
                            "action": action_name,
                            "status_text": "TIMEOUT_GOAL_REACHED",
                            "timeout_sec": round(float(timeout_sec), 3),
                            "goal_check": reached_detail,
                            "ee_goal_check": ee_consistency_detail,
                        }
                        self.get_logger().warning(
                            "[BRIDGE_EXEC] FollowJointTrajectory TIMEOUT pero goal alcanzado "
                            f"action={action_name} detail={reached_detail}"
                        )
                        return True, f"fjt_timeout_goal_reached:{reached_detail}", meta
                diag = self._exec_diagnostics()
                detail = self._diag_to_message(diag)
                meta = {
                    "action": action_name,
                    "status_text": "TIMEOUT",
                    "timeout_sec": round(float(timeout_sec), 3),
                    "goal_check": reached_detail,
                    "diag_reason": diag.get("reason"),
                    "joint_state_age_sec": diag.get("joint_state_age_sec"),
                    "controller_action_available": diag.get("controller_action_available"),
                    "active_controllers": ",".join(diag.get("active_controllers") or []) or "none",
                }
                self.get_logger().warning(
                    "[BRIDGE_EXEC] FollowJointTrajectory TIMEOUT "
                    f"action={action_name} detail={detail}"
                )
                return False, f"fjt_result_timeout:{detail}", meta

            wrapped = result_future.result()
            status = int(
                getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN) or GoalStatus.STATUS_UNKNOWN
            )
            status_text = self._goal_status_text(status)
            result = getattr(wrapped, "result", None)
            error_code = None
            error_string = ""
            if result is not None:
                try:
                    error_code = int(getattr(result, "error_code", 0))
                except Exception:
                    error_code = None
                try:
                    error_string = str(getattr(result, "error_string", "") or "")
                except Exception:
                    error_string = ""
            success_code = int(getattr(FollowJointTrajectory.Result, "SUCCESSFUL", 0))
            ok = status == GoalStatus.STATUS_SUCCEEDED and (
                error_code is None or int(error_code) == success_code
            )
            meta = {
                "action": action_name,
                "status": status,
                "status_text": status_text,
                "error_code": error_code,
                "error_string": error_string or "n/a",
            }
            detail = (
                f"fjt_status={status_text};error_code={error_code if error_code is not None else 'n/a'};"
                f"error_string={error_string or 'n/a'};action={action_name}"
            )
            if ok:
                consistent_with_ee, ee_consistency_detail = (
                    self._joint_goal_success_consistent_with_ee(
                        target_pose=target_pose,
                        ee_target_tol_m=ee_goal_check_tol_m,
                        settle_timeout_sec=ee_goal_check_settle_sec,
                        joint_detail=detail,
                        action_name=action_name,
                        source_label="follow_joint_trajectory_result",
                    )
                )
                if consistent_with_ee:
                    meta["ee_goal_check"] = ee_consistency_detail
                    self.get_logger().info(f"[BRIDGE_EXEC] FollowJointTrajectory OK ({detail})")
                    return True, f"fjt_execute_ok:{detail}", meta
                meta["ee_goal_check"] = ee_consistency_detail
                if (
                    phase_label_upper == "APPROACH"
                    and int(approach_replan_attempt) < int(approach_replan_max_attempts)
                ):
                    meta["status_text"] = "APPROACH_REPLAN_FROM_CURRENT_STATE"
                    self.get_logger().warning(
                        "[BRIDGE_EXEC] FollowJointTrajectory returned SUCCEEDED "
                        "but ee target stayed away during APPROACH; requesting replan "
                        f"attempt={int(approach_replan_attempt) + 1}/{int(approach_replan_max_attempts)} "
                        f"action={action_name} ee_detail={ee_consistency_detail}"
                    )
                    return (
                        False,
                        "fjt_approach_replan_from_current_state:"
                        f"{detail};{ee_consistency_detail}",
                        meta,
                    )
                meta["status_text"] = "SUCCEEDED_BUT_EE_TARGET_NOT_REACHED"
                self.get_logger().warning(
                    "[BRIDGE_EXEC] FollowJointTrajectory returned SUCCEEDED "
                    "but ee target stayed away; rejecting success "
                    f"action={action_name} ee_detail={ee_consistency_detail}"
                )
                return (
                    False,
                    f"fjt_succeeded_but_ee_target_not_reached:{detail};{ee_consistency_detail}",
                    meta,
                )
            if (
                retry_on_tolerance_violation
                and int(error_code or 0) == -4
                and "path tolerance" in (error_string or "").lower()
            ):
                reached_after_abort, reached_after_abort_detail = self._wait_joint_goal_reached(
                    jt,
                    settle_timeout_sec=1.0,
                    tol_rad=goal_check_tol_rad,
                )
                ee_reached_after_abort = False
                ee_reached_after_abort_detail = "ee_goal_not_checked"
                if target_pose is not None:
                    ee_reached_after_abort, ee_reached_after_abort_detail = self._wait_ee_target_reached(
                        target_pose,
                        settle_timeout_sec=1.0,
                        tol_m=ee_goal_check_tol_m,
                    )
                if reached_after_abort:
                    consistent_with_ee, ee_consistency_detail = (
                        self._joint_goal_success_consistent_with_ee(
                            target_pose=target_pose,
                            ee_target_tol_m=ee_goal_check_tol_m,
                            settle_timeout_sec=1.0,
                            joint_detail=reached_after_abort_detail,
                            action_name=action_name,
                            source_label="aborted_goal_reached",
                        )
                    )
                    if consistent_with_ee:
                        meta["status_text"] = "ABORTED_GOAL_REACHED"
                        meta["goal_check"] = reached_after_abort_detail
                        meta["ee_goal_check"] = ee_consistency_detail
                        self.get_logger().warning(
                            "[BRIDGE_EXEC] FollowJointTrajectory path tolerance violation "
                            "pero el goal articular ya esta alcanzado; aceptando success "
                            f"action={action_name} detail={reached_after_abort_detail}"
                        )
                        return True, f"fjt_aborted_but_goal_reached:{reached_after_abort_detail}", meta
                if target_pose is not None and ee_reached_after_abort:
                    meta["status_text"] = "ABORTED_EE_TARGET_REACHED"
                    meta["ee_goal_check"] = ee_reached_after_abort_detail
                    self.get_logger().warning(
                        "[BRIDGE_EXEC] FollowJointTrajectory path tolerance violation "
                        "pero el ee target ya esta alcanzado; aceptando success "
                        f"action={action_name} detail={ee_reached_after_abort_detail}"
                    )
                    return True, f"fjt_aborted_but_ee_target_reached:{ee_reached_after_abort_detail}", meta
                if (
                    phase_label_upper == "APPROACH"
                    and int(approach_replan_attempt) < int(approach_replan_max_attempts)
                ):
                    approach_internal_replan_enabled = _get_moveit_bridge_params().approach_internal_replan
                    if not approach_internal_replan_enabled:
                        meta["status_text"] = "APPROACH_PATH_TOL_TERMINAL"
                        meta["joint_goal_check"] = reached_after_abort_detail
                        meta["ee_goal_check"] = ee_reached_after_abort_detail
                        self.get_logger().warning(
                            "[BRIDGE_EXEC] APPROACH abort por path tolerance; "
                            "replan interno deshabilitado, devolviendo fallo terminal "
                            f"attempt={int(approach_replan_attempt) + 1}/{int(approach_replan_max_attempts)} "
                            f"joint_check={reached_after_abort_detail} "
                            f"ee_check={ee_reached_after_abort_detail}"
                        )
                        return (
                            False,
                            "fjt_approach_path_tolerance_terminal:"
                            f"{reached_after_abort_detail};{ee_reached_after_abort_detail}",
                            meta,
                        )
                    self.get_logger().warning(
                        "[BRIDGE_EXEC] APPROACH abort por path tolerance; "
                        "replanificando desde el estado actual "
                        f"attempt={int(approach_replan_attempt) + 1}/{int(approach_replan_max_attempts)} "
                        f"joint_check={reached_after_abort_detail} "
                        f"ee_check={ee_reached_after_abort_detail}"
                    )
                    return (
                        False,
                        "fjt_approach_replan_from_current_state:"
                        f"path_tolerance:{reached_after_abort_detail};{ee_reached_after_abort_detail}",
                        {
                            **meta,
                            "status_text": "APPROACH_REPLAN_FROM_CURRENT_STATE",
                            "joint_goal_check": reached_after_abort_detail,
                            "ee_goal_check": ee_reached_after_abort_detail,
                        },
                    )
                slow_factor = 2.0
                slowed = self._scale_joint_trajectory_timing(jt, scale=slow_factor)
                retry_timeout = max(
                    float(timeout_sec) * slow_factor,
                    self._joint_trajectory_duration_sec(slowed) + 8.0,
                )
                retry_path_tol = max(
                    3.8,
                    float(path_tol_override_rad)
                    if path_tol_override_rad is not None
                    else float(self._controller_path_tolerance_rad)
                    if float(self._controller_path_tolerance_rad) >= 0.0
                    else 0.0,
                )
                retry_goal_time = max(
                    45.0,
                    float(goal_time_override_sec)
                    if goal_time_override_sec is not None
                    else float(self._controller_goal_time_tolerance_sec),
                )
                self.get_logger().warning(
                    "[BRIDGE_EXEC] FollowJointTrajectory retry por path tolerance violation "
                    f"scale={slow_factor:.1f} timeout_sec={retry_timeout:.1f} "
                    f"path_tol={retry_path_tol:.3f} goal_time_tol={retry_goal_time:.1f}"
                )
                return self._execute_joint_trajectory_action(
                    slowed,
                    timeout_sec=retry_timeout,
                    retry_on_tolerance_violation=False,
                    path_tol_override_rad=retry_path_tol,
                    goal_time_override_sec=retry_goal_time,
                    target_pose=target_pose,
                    ee_target_tol_m=ee_target_tol_m,
                    approach_replan_attempt=approach_replan_attempt,
                )
            if (
                retry_on_tolerance_violation
                and int(error_code or 0) == -5
                and "goal_time_tolerance" in (error_string or "").lower()
            ):
                reached_after_goal_time, reached_after_goal_time_detail = self._wait_joint_goal_reached(
                    jt,
                    settle_timeout_sec=1.0,
                    tol_rad=goal_check_tol_rad,
                )
                ee_reached_after_goal_time = False
                ee_reached_after_goal_time_detail = "ee_goal_not_checked"
                if target_pose is not None:
                    ee_reached_after_goal_time, ee_reached_after_goal_time_detail = self._wait_ee_target_reached(
                        target_pose,
                        settle_timeout_sec=1.0,
                        tol_m=ee_goal_check_tol_m,
                    )
                self.get_logger().warning(
                    "[BRIDGE_EXEC] goal_time_tolerance diagnostics "
                    f"action={action_name} "
                    f"joint_check={reached_after_goal_time_detail} "
                    f"ee_check={ee_reached_after_goal_time_detail} "
                    f"goal_time_tol={float(effective_goal_time_tol_sec):.3f}"
                )
                if reached_after_goal_time:
                    meta["status_text"] = "GOAL_TIME_TOLERANCE_GOAL_REACHED"
                    meta["goal_check"] = reached_after_goal_time_detail
                    self.get_logger().warning(
                        "[BRIDGE_EXEC] FollowJointTrajectory goal_time_tolerance "
                        "pero el goal articular ya esta alcanzado; aceptando success "
                        f"action={action_name} detail={reached_after_goal_time_detail}"
                    )
                    return True, f"fjt_goal_time_but_goal_reached:{reached_after_goal_time_detail}", meta
                if target_pose is not None:
                    if ee_reached_after_goal_time:
                        meta["status_text"] = "GOAL_TIME_TOLERANCE_EE_TARGET_REACHED"
                        meta["ee_goal_check"] = ee_reached_after_goal_time_detail
                        self.get_logger().warning(
                            "[BRIDGE_EXEC] FollowJointTrajectory goal_time_tolerance "
                            "pero el ee target ya esta alcanzado; aceptando success "
                            f"action={action_name} detail={ee_reached_after_goal_time_detail}"
                        )
                        return True, f"fjt_goal_time_but_ee_target_reached:{ee_reached_after_goal_time_detail}", meta
                if (
                    phase_label_upper == "APPROACH"
                    and int(approach_replan_attempt) < int(approach_replan_max_attempts)
                ):
                    approach_internal_replan_enabled = _get_moveit_bridge_params().approach_internal_replan
                    if not approach_internal_replan_enabled:
                        meta["status_text"] = "APPROACH_GOAL_TIME_TERMINAL"
                        meta["joint_goal_check"] = reached_after_goal_time_detail
                        meta["ee_goal_check"] = ee_reached_after_goal_time_detail
                        self.get_logger().warning(
                            "[BRIDGE_EXEC] APPROACH goal_time_tolerance; "
                            "replan interno deshabilitado, devolviendo fallo terminal "
                            f"attempt={int(approach_replan_attempt) + 1}/{int(approach_replan_max_attempts)} "
                            f"joint_check={reached_after_goal_time_detail} "
                            f"ee_check={ee_reached_after_goal_time_detail}"
                        )
                        return (
                            False,
                            "fjt_approach_goal_time_terminal:"
                            f"{reached_after_goal_time_detail};"
                            f"{ee_reached_after_goal_time_detail}",
                            meta,
                        )
                    self.get_logger().warning(
                        "[BRIDGE_EXEC] APPROACH goal_time_tolerance; "
                        "replanificando desde el estado actual "
                        f"attempt={int(approach_replan_attempt) + 1}/{int(approach_replan_max_attempts)} "
                        f"joint_check={reached_after_goal_time_detail} "
                        f"ee_check={ee_reached_after_goal_time_detail}"
                    )
                    return (
                        False,
                        "fjt_approach_replan_from_current_state:"
                        f"goal_time_tolerance:{reached_after_goal_time_detail};"
                        f"{ee_reached_after_goal_time_detail}",
                        {
                            **meta,
                            "status_text": "APPROACH_REPLAN_FROM_CURRENT_STATE",
                            "joint_goal_check": reached_after_goal_time_detail,
                            "ee_goal_check": ee_reached_after_goal_time_detail,
                        },
                    )
                slow_factor = 2.0
                slowed = self._scale_joint_trajectory_timing(jt, scale=slow_factor)
                retry_timeout = max(
                    float(timeout_sec) * slow_factor,
                    self._joint_trajectory_duration_sec(slowed) + 12.0,
                )
                # APPROACH specific: enforce maximum timeout to prevent infinite retries
                if phase_label_upper == "APPROACH":
                    max_approach_timeout = float(approach_max_total_timeout_sec)
                    if retry_timeout > max_approach_timeout:
                        self.get_logger().warning(
                            "[BRIDGE_EXEC] APPROACH retry_timeout capped "
                            f"original={retry_timeout:.1f} max={max_approach_timeout:.1f}"
                        )
                        retry_timeout = max_approach_timeout
                retry_goal_time = max(
                    float(effective_goal_time_tol_sec),
                    self._env_float(
                        "PANEL_MOVEIT_BRIDGE_APPROACH_GOAL_TIME_RETRY_SEC",
                        90.0 if phase_label_upper == "APPROACH" else float(effective_goal_time_tol_sec),
                    ),
                )
                self.get_logger().warning(
                    "[BRIDGE_EXEC] FollowJointTrajectory retry por goal_time_tolerance "
                    f"scale={slow_factor:.1f} timeout_sec={retry_timeout:.1f} "
                    f"goal_time_tol={retry_goal_time:.1f}"
                )
                return self._execute_joint_trajectory_action(
                    slowed,
                    timeout_sec=retry_timeout,
                    retry_on_tolerance_violation=False,
                    goal_time_override_sec=retry_goal_time,
                    target_pose=target_pose,
                    ee_target_tol_m=ee_target_tol_m,
                    phase_label=phase_label,
                    approach_replan_attempt=approach_replan_attempt,
                )
            self.get_logger().warning(f"[BRIDGE_EXEC] FollowJointTrajectory FAIL ({detail})")
            return False, f"fjt_aborted:{detail}", meta
        finally:
            with self._pose_lock:
                current_deadline = float(getattr(self, "_active_exec_timeout_deadline_mono", 0.0) or 0.0)
                if current_deadline <= 0.0 or abs(current_deadline - exec_deadline_mono) <= 1.0:
                    self._active_exec_timeout_sec = 0.0
                    self._active_exec_timeout_deadline_mono = 0.0
