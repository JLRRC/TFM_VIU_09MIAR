"""MoveItPyPlannerMixin: backend principal MoveItPy + init.

Extraído de ``ur5_tools/ur5_moveit_bridge.py`` (líneas 2855-3562 originales):

* ``_plan_with_moveit_py`` (~563 L): pipeline completo plan+execute con
  MoveItPy. Maneja APPROACH IK seeded, path constraints con relaxed retry,
  endpoint validation, dry run, force FJT direct para walltime sim,
  fallback FJT directo, fallback topic publish, replan desde current
  state.
* ``_init_moveit_py`` (~146 L): construcción de ``MoveItConfigsBuilder``,
  saneo de QoS overrides y ``planning_pipelines``, default planner_id,
  ``trajectory_execution`` defaults, instancia ``MoveItPy`` y
  ``PlanningComponent``.

Sin estado propio. Requiere que la clase concreta herede también de
``rclpy.node.Node`` y de los mixins previos
(``JointStateHelpersMixin``, ``GeometryMixin``, ``ControllerManagementMixin``,
``GoalValidationMixin``, ``TrajectoryPrepMixin``).
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from .params import get_moveit_bridge_params as _get_moveit_bridge_params

from geometry_msgs.msg import PoseStamped
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.msg import Constraints

try:
    from moveit.planning import MoveItPy, PlanningComponent  # type: ignore
except Exception:  # pragma: no cover - moveit_py opcional
    MoveItPy = None  # type: ignore
    PlanningComponent = None  # type: ignore


class MoveItPyPlannerMixin:
    """Plan + execute backend MoveItPy (con APPROACH-aware retries) + init."""

    def _plan_with_moveit_py(
        self,
        target: PoseStamped,
        request_uuid: str = "",
        ee_target_tol_m: float | None = None,
        phase_label: str | None = None,
        _replan_from_current_state_attempt: int = 0,
    ) -> tuple[bool, str, bool, bool]:
        if not self._planning_component or not self._moveit_py:
            if self._moveit_py_init_error:
                self.get_logger().warning(
                    f"MoveItPy no inicializado: {self._moveit_py_init_error}"
                )
            else:
                self.get_logger().warning(
                    "MoveItPy inicializando; reintenta en unos segundos."
                )
            return False, "moveit_py_not_ready", False, False
        try:
            start_state_ok, start_state_reason = self._set_planning_start_state_from_joint_state()
            if start_state_ok:
                self.get_logger().info(
                    f"[BRIDGE_START_STATE] source=joint_states detail={start_state_reason}"
                )
            else:
                self.get_logger().warning(
                    "[BRIDGE_START_STATE] source=joint_states unavailable "
                    f"detail={start_state_reason}; fallback=current_state_monitor"
                )
                self._planning_component.set_start_state_to_current_state()
            phase_upper_goal = str(phase_label or "").strip().upper()
            _approach_ik_state = None
            if phase_upper_goal == "APPROACH":
                _approach_ik_state = self._compute_approach_ik_seeded(target)
            use_explicit_joint_goal = _approach_ik_state is not None
            if _approach_ik_state is not None:
                # Joint-space goal: no IK branch ambiguity, OMPL plans to explicit joints.
                self._planning_component.set_goal_state(robot_state=_approach_ik_state)
                self.get_logger().info(
                    "[APPROACH_IK_SEED] Using joint-space goal (selected seeded IK) for APPROACH"
                )
            else:
                try:
                    self._planning_component.set_goal_state(
                        pose_stamped_msg=target,
                        pose_link=self._ee_frame,
                    )
                except TypeError:
                    # Fallback for older MoveItPy bindings.
                    self._planning_component.set_goal_state(target, self._ee_frame)
            path_constraints = None
            if not use_explicit_joint_goal:
                path_constraints = self._build_joint_path_constraints(
                    request_uuid=request_uuid,
                    phase_label=phase_label,
                )
            elif phase_upper_goal == "APPROACH":
                self.get_logger().info(
                    "[APPROACH_IK_SEED] skipping path constraints for explicit joint-space APPROACH goal"
                )
            if path_constraints is not None:
                self._planning_component.set_path_constraints(path_constraints)
            try:
                plan = self._planning_component.plan()
            finally:
                if path_constraints is not None:
                    self._planning_component.set_path_constraints(Constraints())
            # Fallback: if plan failed WITH constraints, first retry APPROACH
            # with a relaxed shoulder_pan tolerance before disabling constraints.
            if path_constraints is not None and plan is not None:
                _pc_ok = self._plan_success_ok(getattr(plan, "success", None))
                _pc_traj = getattr(plan, "trajectory", None) is not None
                _pc_ec = self._plan_error_code_val(plan)
                _pc_ec_bad = _pc_ec is not None and _pc_ec != 1
                if not _pc_ok or not _pc_traj or _pc_ec_bad:
                    phase_upper = str(phase_label or "").strip().upper()
                    relaxed_retry_allowed = (
                        phase_upper == "APPROACH"
                        and _get_moveit_bridge_params().approach_relaxed_constraint_retry
                    )
                    if relaxed_retry_allowed:
                        strict_tol = max(
                            0.05,
                            self._env_float(
                                "PANEL_MOVEIT_BRIDGE_APPROACH_PATH_CONSTRAINT_TOL_RAD",
                                min(float(self._path_constraint_joint_tol), 0.35),
                            ),
                        )
                        relaxed_tol = max(
                            strict_tol,
                            self._env_float(
                                "PANEL_MOVEIT_BRIDGE_APPROACH_RELAXED_PATH_CONSTRAINT_TOL_RAD",
                                0.60,
                            ),
                        )
                        if relaxed_tol > strict_tol + 1e-6:
                            self.get_logger().warning(
                                "[BRIDGE_CONSTRAINT] plan failed with strict APPROACH constraints; "
                                f"retrying with relaxed tol={relaxed_tol:.3f} "
                                f"success_ok={_pc_ok} traj={_pc_traj} error_code={_pc_ec}"
                            )
                            start_state_ok, start_state_reason = (
                                self._set_planning_start_state_from_joint_state()
                            )
                            if start_state_ok:
                                self.get_logger().info(
                                    "[BRIDGE_START_STATE] relaxed retry source=joint_states "
                                    f"detail={start_state_reason}"
                                )
                            else:
                                self.get_logger().warning(
                                    "[BRIDGE_START_STATE] relaxed retry fallback=current_state_monitor "
                                    f"detail={start_state_reason}"
                                )
                                self._planning_component.set_start_state_to_current_state()
                            relaxed_constraints = self._build_joint_path_constraints(
                                request_uuid=request_uuid,
                                phase_label=phase_label,
                                tol_override=relaxed_tol,
                            )
                            if relaxed_constraints is not None:
                                self._planning_component.set_path_constraints(relaxed_constraints)
                                try:
                                    plan = self._planning_component.plan()
                                finally:
                                    self._planning_component.set_path_constraints(Constraints())
                                _pc_ok = self._plan_success_ok(getattr(plan, "success", None))
                                _pc_traj = getattr(plan, "trajectory", None) is not None
                                _pc_ec = self._plan_error_code_val(plan)
                                _pc_ec_bad = _pc_ec is not None and _pc_ec != 1
                                if _pc_ok and _pc_traj and not _pc_ec_bad:
                                    path_constraints = relaxed_constraints
                    if not _pc_ok or not _pc_traj or _pc_ec_bad:
                        self.get_logger().warning(
                            "[BRIDGE_CONSTRAINT] plan failed with constraints; "
                            f"retrying WITHOUT constraints (fallback) "
                            f"success_ok={_pc_ok} traj={_pc_traj} error_code={_pc_ec}"
                        )
                        self.get_logger().warning(
                            "[PICK][MOVEIT][DIVERGENCE] "
                            f"phase={phase_label or 'n/a'} request_uuid={request_uuid or 'n/a'} "
                            "kind=constraints_fallback_disabled_for_replan "
                            f"success_ok={_pc_ok} traj={_pc_traj} error_code={_pc_ec if _pc_ec is not None else 'n/a'}"
                        )
                        start_state_ok, start_state_reason = (
                            self._set_planning_start_state_from_joint_state()
                        )
                        if start_state_ok:
                            self.get_logger().info(
                                "[BRIDGE_START_STATE] retry source=joint_states "
                                f"detail={start_state_reason}"
                            )
                        else:
                            self.get_logger().warning(
                                "[BRIDGE_START_STATE] retry fallback=current_state_monitor "
                                f"detail={start_state_reason}"
                            )
                            self._planning_component.set_start_state_to_current_state()
                        plan = self._planning_component.plan()
        except Exception as exc:
            self.get_logger().warning(f"Planificación MoveItPy fallida: {exc}")
            return False, f"plan_exception:{exc}", False, False
        if plan is None:
            self._log_bridge_status(
                "[BRIDGE_STATUS] plan_fail backend=moveit_py reason=empty",
                level="warn",
            )
            self.get_logger().warning("Planificación MoveItPy fallida (plan vacío).")
            return False, "plan_empty", False, False
        trajectory = getattr(plan, "trajectory", None)
        success = getattr(plan, "success", None)
        success_code = self._plan_success_code(success)
        success_ok = self._plan_success_ok(success)
        # Also check error_code (catches ValidateSolution INVALID_MOTION_PLAN)
        ec_val = self._plan_error_code_val(plan)
        if ec_val is not None and ec_val != 1:
            self.get_logger().warning(
                f"[BRIDGE_STATUS] plan rejected: error_code={ec_val} "
                f"(success={success!r})"
            )
            success_ok = False
        if (not success_ok) or trajectory is None:
            fail_reason = "invalid_plan_status" if not success_ok else "no_trajectory"
            self.get_logger().warning(
                "[PICK][MOVEIT][PLAN_RESULT] "
                f"phase={phase_label or 'n/a'} request_uuid={request_uuid or 'n/a'} "
                f"backend=moveit_py success=false reason={fail_reason} "
                f"success_code={success_code} error_code={ec_val if ec_val is not None else 'n/a'} "
                f"constraints_applied={str(path_constraints is not None).lower()}"
            )
            self._log_bridge_status(
                f"[BRIDGE_STATUS] plan_fail backend=moveit_py reason={fail_reason}",
                level="warn",
            )
            self.get_logger().warning(
                "Planificación MoveItPy fallida "
                f"(success={success!r}, success_code={success_code}, "
                f"trajectory={'none' if trajectory is None else type(trajectory).__name__})."
            )
            if not success_ok:
                return False, f"plan_failed:success_code={success_code}", False, False
            return False, "plan_no_trajectory", False, False
        phase_upper = str(phase_label or "").strip().upper()
        plan_ee_target_tol_m = (
            float(ee_target_tol_m)
            if ee_target_tol_m is not None
            else self._env_float("PANEL_MOVEIT_BRIDGE_PLAN_EE_TARGET_TOL_M", 0.08)
        )
        if phase_upper == "APPROACH":
            plan_ee_target_tol_m = min(
                float(plan_ee_target_tol_m),
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_PLAN_EE_TARGET_TOL_M",
                    0.08,
                ),
            )
        elif phase_upper == "PRE_GRASP":
            plan_ee_target_tol_m = min(
                float(plan_ee_target_tol_m),
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_PREGRASP_PLAN_EE_TARGET_TOL_M",
                    0.10,
                ),
            )
        plan_ee_target_tol_m = max(0.02, float(plan_ee_target_tol_m))
        plan_endpoint_ok, plan_endpoint_detail = self._planned_trajectory_target_consistent(
            trajectory,
            target_pose=target,
            tol_m=plan_ee_target_tol_m,
            phase_label=phase_label,
            request_uuid=request_uuid,
        )
        if not plan_endpoint_ok:
            self._log_bridge_status(
                "[BRIDGE_STATUS] plan_fail backend=moveit_py reason=plan_endpoint_target_mismatch",
                level="warn",
            )
            self.get_logger().warning(
                "[PICK][MOVEIT][PLAN_RESULT] "
                f"phase={phase_label or 'n/a'} request_uuid={request_uuid or 'n/a'} "
                "backend=moveit_py success=false reason=plan_endpoint_target_mismatch "
                f"detail={plan_endpoint_detail}"
            )
            return False, f"plan_endpoint_target_mismatch:{plan_endpoint_detail}", False, False
        self._log_bridge_status("[BRIDGE_STATUS] plan_ok backend=moveit_py")
        self.get_logger().info("Planificación MoveItPy OK.")
        self.get_logger().info(
            "[PICK][MOVEIT][PLAN_RESULT] "
            f"phase={phase_label or 'n/a'} request_uuid={request_uuid or 'n/a'} "
            f"backend=moveit_py success=true constraints_applied={str(path_constraints is not None).lower()} "
            f"trajectory_type={type(trajectory).__name__ if trajectory is not None else 'none'}"
        )
        if self._dry_run_plan_only:
            self._log_bridge_status(
                "[BRIDGE_STATUS] exec_skip backend=moveit_py reason=dry_run_plan_only"
            )
            self.get_logger().warning(
                "[BRIDGE_DRY_RUN] plan generated and execution skipped (dry_run_plan_only=true)."
            )
            return True, "dry_run_plan_only", True, False
        self.get_logger().info(
            "[BRIDGE_EXEC] "
            f"backend=moveit_py ee_link={self._ee_frame} "
            f"controller={self._controller_name} action={self._controller_action_name}"
        )
        action_ready, matched_action, action_names, checked_candidates = (
            self._wait_for_expected_controller_action(
            timeout_sec=2.5
            )
        )
        if not action_ready:
            available = ",".join(sorted(action_names)) if action_names else "none"
            checked = ",".join(checked_candidates) if checked_candidates else "none"
            active = ",".join(self._active_controllers(timeout_sec=0.6)) or "none"
            detail = (
                f"expected_action={self._controller_action_name};"
                f"checked_candidates={checked};"
                f"available_actions={available}"
                f";active_controllers={active};"
                f"controller_manager={self._controller_manager_name}"
            )
            self._log_bridge_status(
                "[BRIDGE_STATUS] exec_precheck_fail backend=moveit_py reason=no_action_server "
                f"detail={detail}",
                level="warn",
            )
            self.get_logger().warning(
                "Action server no detectado por precheck; se intentara execute igualmente. "
                f"{detail}"
            )
        else:
            self.get_logger().info(
                f"[BRIDGE_EXEC] action server detectado: {matched_action}"
            )
        # When MoveItPy is forced to wall-time (sim-time QoS crash workaround),
        # execute through FJT directly to avoid false ABORTED from temporal validation.
        if (
            self._use_sim_time
            and not self._moveit_py_use_sim_time
            and self._force_fjt_direct_for_walltime_sim
        ):
            jt_direct = self._extract_joint_trajectory_msg(trajectory)
            if jt_direct is not None:
                traj_sec = self._joint_trajectory_duration_sec(jt_direct)
                fjt_timeout = self._fjt_timeout_for_trajectory(
                    traj_sec,
                    extra_margin_sec=8.0,
                    minimum_sec=8.0,
                )
                fjt_ok, fjt_msg, fjt_meta = self._execute_joint_trajectory_action(
                    jt_direct,
                    timeout_sec=fjt_timeout,
                    target_pose=target,
                    ee_target_tol_m=ee_target_tol_m,
                    phase_label=phase_label,
                    approach_replan_attempt=_replan_from_current_state_attempt,
                )
                fjt_detail = self._result_meta_to_message(fjt_meta)
                approach_replan_max_attempts = max(
                    1,
                    int(
                        round(
                            self._env_float(
                                "PANEL_MOVEIT_BRIDGE_APPROACH_REPLAN_MAX_ATTEMPTS",
                                2.0,
                            )
                        )
                    ),
                )
                if fjt_ok:
                    self._log_bridge_status(
                        "[BRIDGE_STATUS] exec_ok backend=moveit_py mode=fjt_direct_time_domain"
                    )
                    return (
                        True,
                        f"exec_ok_fjt_direct:{fjt_msg};fjt_meta={fjt_detail}",
                        True,
                        True,
                    )
                fjt_status_text = str((fjt_meta or {}).get("status_text", "") or "").upper()
                phase_upper = str(phase_label or "").strip().upper()
                should_replan_from_current = (
                    phase_upper == "APPROACH"
                    and _replan_from_current_state_attempt < approach_replan_max_attempts
                    and fjt_status_text in ("APPROACH_REPLAN_FROM_CURRENT_STATE", "TIMEOUT")
                )
                if (
                    should_replan_from_current
                ):
                    settle_ok, settle_reason = self._wait_for_joint_state_settled(
                        timeout_sec=2.0,
                        stable_sec=0.35,
                        tol_rad=0.03,
                    )
                    replan_reason = (
                        "timeout_terminal"
                        if fjt_status_text == "TIMEOUT"
                        else "approach_replan_signal"
                    )
                    joint_goal_check = str((fjt_meta or {}).get("joint_goal_check") or "n/a")
                    ee_goal_check = str((fjt_meta or {}).get("ee_goal_check") or "n/a")
                    goal_check = str((fjt_meta or {}).get("goal_check") or "n/a")
                    feedback_goal_check = str((fjt_meta or {}).get("feedback_goal_check") or "n/a")
                    joint_motion = str((fjt_meta or {}).get("joint_motion") or "n/a")
                    self.get_logger().warning(
                        "[BRIDGE_EXEC] replanificando APPROACH desde el estado actual "
                        f"attempt={_replan_from_current_state_attempt + 1} "
                        f"reason={replan_reason} status={fjt_status_text or 'n/a'} "
                        f"joint_goal_check={joint_goal_check} ee_goal_check={ee_goal_check} "
                        f"goal_check={goal_check} feedback_goal_check={feedback_goal_check} "
                        f"joint_motion={joint_motion} "
                        f"joint_state_settled={str(bool(settle_ok)).lower()} "
                        f"detail={settle_reason}"
                    )
                    return self._plan_with_moveit_py(
                        target,
                        request_uuid=request_uuid,
                        ee_target_tol_m=ee_target_tol_m,
                        phase_label=phase_label,
                        _replan_from_current_state_attempt=_replan_from_current_state_attempt + 1,
                    )
                if (
                    phase_upper == "APPROACH"
                    and _replan_from_current_state_attempt >= approach_replan_max_attempts
                ):
                    self.get_logger().warning(
                        "[BRIDGE_EXEC] APPROACH replan attempt ya consumido; "
                        "evitando topic publish fallback y devolviendo fallo directo "
                        f"status={fjt_status_text or 'n/a'}"
                    )
                    return (
                        False,
                        f"exec_failed_fjt_direct:{fjt_msg};fjt_meta={fjt_detail}",
                        True,
                        False,
                    )
                topic_timeout = self._fjt_timeout_for_trajectory(
                    traj_sec,
                    extra_margin_sec=4.0,
                    minimum_sec=8.0,
                )
                topic_timeout = max(8.0, min(18.0, topic_timeout * 0.25))
                if self._publish_planned_joint_trajectory(jt_direct):
                    reached, reached_detail = self._wait_joint_goal_reached(
                        self._prepare_joint_trajectory_for_controller(jt_direct),
                        settle_timeout_sec=topic_timeout,
                        tol_rad=0.10,
                    )
                    if reached:
                        consistent_with_ee, ee_consistency_detail = (
                            self._joint_goal_success_consistent_with_ee(
                                target_pose=target,
                                ee_target_tol_m=max(0.04, float(ee_target_tol_m or 0.0)),
                                settle_timeout_sec=min(2.0, topic_timeout),
                                joint_detail=reached_detail,
                                action_name=self._controller_action_name,
                                source_label="topic_publish_goal_check",
                            )
                        )
                        if not consistent_with_ee:
                            self.get_logger().warning(
                                "[BRIDGE_EXEC] topic publish fallback reached joint goal "
                                "but ee target stayed away; rejecting logical success "
                                f"goal_detail={reached_detail} ee_detail={ee_consistency_detail}"
                            )
                        else:
                            self._log_bridge_status(
                                "[BRIDGE_STATUS] exec_fallback backend=moveit_py mode=topic_publish_goal_check"
                            )
                            return (
                                True,
                                (
                                    "exec_fallback_topic_goal_reached:"
                                    f"{fjt_msg};goal_check={reached_detail};"
                                    f"ee_goal_check={ee_consistency_detail};fjt_meta={fjt_detail}"
                                ),
                                True,
                                True,
                            )
                    self.get_logger().warning(
                        "[BRIDGE_EXEC] topic publish fallback no alcanzo goal "
                        f"detail={reached_detail}"
                    )
                return (
                    False,
                    f"exec_failed_fjt_direct:{fjt_msg};fjt_meta={fjt_detail}",
                    True,
                    False,
                )
        try:
            exec_ok, result, timeout_detail = self._execute_moveit_py_with_timeout(trajectory)
            if not exec_ok:
                return False, f"exec_failed:{timeout_detail}", True, False
        except Exception as exc:
            diag = self._exec_diagnostics()
            detail = self._diag_to_message(diag)
            self._log_bridge_status(
                f"[BRIDGE_STATUS] exec_fail backend=moveit_py reason=exception detail={detail}",
                level="warn",
            )
            self.get_logger().warning(
                "Ejecución MoveItPy excepción: "
                f"{exc} | diag={json.dumps(diag, ensure_ascii=True)}"
            )
            return False, f"exec_exception:{exc};{detail}", True, False
        result_meta = self._describe_execute_result(result)
        if bool(result):
            if target is not None:
                exec_ee_tol_m = max(
                    0.02,
                    float(ee_target_tol_m)
                    if ee_target_tol_m is not None
                    else self._env_float("PANEL_MOVEIT_BRIDGE_EE_TARGET_TOL_M", 0.10),
                )
                exec_ee_ok, exec_ee_detail = self._wait_ee_target_reached(
                    target,
                    settle_timeout_sec=max(
                        0.25,
                        self._env_float("PANEL_MOVEIT_BRIDGE_EE_TARGET_SETTLE_SEC", 0.45),
                    ),
                    tol_m=exec_ee_tol_m,
                )
                if not exec_ee_ok:
                    self._log_bridge_status(
                        "[BRIDGE_STATUS] exec_fail backend=moveit_py reason=ee_target_not_reached_after_execute",
                        level="warn",
                    )
                    self.get_logger().warning(
                        "Ejecución MoveItPy devolvió success pero el ee target no quedó alcanzado. "
                        f"detail={exec_ee_detail} result={json.dumps(result_meta, ensure_ascii=True)}"
                    )
                    return False, f"exec_succeeded_but_ee_target_not_reached:{exec_ee_detail}", True, False
                result_meta["ee_goal_check"] = exec_ee_detail
            self._log_bridge_status("[BRIDGE_STATUS] exec_ok backend=moveit_py")
            self.get_logger().info(
                "Ejecución MoveItPy completada. "
                f"result={json.dumps(result_meta, ensure_ascii=True)}"
            )
            return True, "exec_ok", True, True
        diag = self._exec_diagnostics()
        detail = self._diag_to_message(diag)
        result_detail = self._result_meta_to_message(result_meta)
        self._log_bridge_status(
            f"[BRIDGE_STATUS] exec_fail backend=moveit_py reason={diag['reason']} detail={detail}",
            level="warn",
        )
        self.get_logger().warning(
            "Ejecución MoveItPy fallida. "
            f"diag={json.dumps(diag, ensure_ascii=True)} "
            f"result={json.dumps(result_meta, ensure_ascii=True)}"
        )
        # Fallback robusto: si la validación temporal de MoveIt falla, ejecutar por
        # el controlador con la trayectoria ya planificada evita falso negativo.
        jt = self._extract_joint_trajectory_msg(trajectory)
        if jt is not None:
            traj_sec = self._joint_trajectory_duration_sec(jt)
            fjt_timeout = self._fjt_timeout_for_trajectory(
                traj_sec,
                extra_margin_sec=8.0,
                minimum_sec=8.0,
            )
            fjt_ok, fjt_msg, fjt_meta = self._execute_joint_trajectory_action(
                jt,
                timeout_sec=fjt_timeout,
                target_pose=target,
                ee_target_tol_m=ee_target_tol_m,
                phase_label=phase_label,
            )
            fjt_detail = self._result_meta_to_message(fjt_meta)
            if fjt_ok:
                self._log_bridge_status(
                    "[BRIDGE_STATUS] exec_fallback backend=moveit_py mode=fjt_action"
                )
                return (
                    True,
                    f"exec_fallback_fjt:{detail};{fjt_msg};execute_result={result_detail};fjt_meta={fjt_detail}",
                    True,
                    True,
                )
            self.get_logger().warning(
                "Fallback FollowJointTrajectory fallido: "
                f"{fjt_msg} meta={json.dumps(fjt_meta, ensure_ascii=True)}"
            )
            return (
                False,
                f"exec_failed:{detail};{fjt_msg};execute_result={result_detail};fjt_meta={fjt_detail}",
                True,
                False,
            )

        published = self._publish_planned_joint_trajectory(trajectory)
        if published:
            self._log_bridge_status(
                "[BRIDGE_STATUS] exec_fallback backend=moveit_py mode=topic_publish"
            )
            return True, f"exec_fallback_topic_publish:{detail};execute_result={result_detail}", True, True
        return False, f"exec_failed:{detail};execute_result={result_detail};fallback=no_joint_trajectory", True, False

    def _init_moveit_py(self) -> None:
        try:
            moveit_share = get_package_share_directory(self._moveit_config_pkg)
            ur5_description_share = get_package_share_directory(self._description_pkg)
            srdf_path = (
                Path(self._srdf_path)
                if self._srdf_path
                else (Path(moveit_share) / "config" / "ur5.srdf")
            )
            urdf_path = (
                Path(self._urdf_xacro_path)
                if self._urdf_xacro_path
                else (Path(ur5_description_share) / "urdf" / "ur5.urdf.xacro")
            )
            kinematics_path = (
                Path(self._kinematics_yaml)
                if self._kinematics_yaml
                else (Path(moveit_share) / "config" / "kinematics.yaml")
            )
            joint_limits_path = (
                Path(self._joint_limits_yaml)
                if self._joint_limits_yaml
                else (Path(moveit_share) / "config" / "joint_limits.yaml")
            )
            controllers_path = (
                Path(self._moveit_controllers_yaml)
                if self._moveit_controllers_yaml
                else (Path(moveit_share) / "config" / "moveit_controllers.yaml")
            )
            self._load_controller_contract(controllers_path)
            moveit_config = (
                MoveItConfigsBuilder(
                    self._robot_name, package_name=self._moveit_config_pkg
                )
                .robot_description(
                    file_path=str(urdf_path),
                    mappings={"ur_type": self._ur_type, "name": self._robot_name},
                )
                .robot_description_semantic(file_path=str(srdf_path))
                .robot_description_kinematics(file_path=str(kinematics_path))
                .joint_limits(file_path=str(joint_limits_path))
                .planning_pipelines(
                    pipelines=self._planning_pipelines,
                    default_planning_pipeline=self._default_planning_pipeline,
                )
                .trajectory_execution(file_path=str(controllers_path))
                .to_moveit_configs()
            )
            config_dict = moveit_config.to_dict()
            # Strip QoS overrides that can fail on /clock in some RMW setups.
            config_dict = {
                k: v
                for k, v in config_dict.items()
                if not str(k).startswith("qos_overrides.")
            }
            config_dict = self._strip_qos_overrides(config_dict)
            pipeline_names = config_dict.get("planning_pipelines", [])
            if isinstance(pipeline_names, list):
                config_dict["planning_pipelines"] = {
                    "pipeline_names": pipeline_names,
                    "default_planning_pipeline": self._default_planning_pipeline,
                    "namespace": "",
                }
                for pipeline in pipeline_names:
                    pipeline_cfg = config_dict.get(pipeline)
                    if (
                        isinstance(pipeline_cfg, dict)
                        and "planning_plugin" not in pipeline_cfg
                        and "planning_plugins" in pipeline_cfg
                        and isinstance(pipeline_cfg["planning_plugins"], list)
                        and pipeline_cfg["planning_plugins"]
                    ):
                        pipeline_cfg["planning_plugin"] = pipeline_cfg[
                            "planning_plugins"
                        ][0]
            if "default_planning_pipeline" not in config_dict:
                config_dict["default_planning_pipeline"] = (
                    self._default_planning_pipeline
                )
            plan_request_params = config_dict.get("plan_request_params")
            if not isinstance(plan_request_params, dict):
                plan_request_params = {}
            if not plan_request_params.get("planning_pipeline"):
                plan_request_params["planning_pipeline"] = (
                    self._default_planning_pipeline
                )
            if not plan_request_params.get("planner_id"):
                pipeline_cfg = config_dict.get(self._default_planning_pipeline, {})
                planner_configs = (
                    pipeline_cfg.get("planner_configs", {})
                    if isinstance(pipeline_cfg, dict)
                    else {}
                )
                if isinstance(planner_configs, dict) and planner_configs:
                    if "RRTConnectkConfigDefault" in planner_configs:
                        plan_request_params["planner_id"] = "RRTConnectkConfigDefault"
                    else:
                        plan_request_params["planner_id"] = next(
                            iter(planner_configs.keys())
                        )
            plan_request_params.setdefault("planning_time", 3.0)
            plan_request_params.setdefault("planning_attempts", 3)
            plan_request_params.setdefault(
                "max_velocity_scaling_factor", float(self._max_velocity_scaling)
            )
            plan_request_params.setdefault(
                "max_acceleration_scaling_factor", float(self._max_acceleration_scaling)
            )
            config_dict["plan_request_params"] = plan_request_params
            config_dict["use_sim_time"] = self._moveit_py_use_sim_time
            traj_exec = config_dict.get("trajectory_execution")
            if not isinstance(traj_exec, dict):
                traj_exec = {}
            traj_exec.setdefault("allowed_execution_duration_scaling", 2.5)
            traj_exec.setdefault("allowed_goal_duration_margin", 1.5)
            config_dict["trajectory_execution"] = traj_exec
            self._moveit_py = MoveItPy(
                node_name="ur5_moveit_py", config_dict=config_dict
            )
            try:
                self._planning_component = PlanningComponent(
                    self._group_name,
                    self._moveit_py,
                    self._default_planning_pipeline,
                )
            except TypeError:
                # Older MoveItPy bindings do not accept planning pipeline id in constructor.
                self._planning_component = PlanningComponent(
                    self._group_name, self._moveit_py
                )
                if hasattr(self._planning_component, "set_planning_pipeline_id"):
                    try:
                        self._planning_component.set_planning_pipeline_id(
                            self._default_planning_pipeline
                        )
                    except Exception as exc:
                        self.get_logger().warning(
                            "No se pudo fijar planning pipeline "
                            f"'{self._default_planning_pipeline}': {exc}"
                        )
            self._moveit_py_ready = True
            self.get_logger().info("MoveItPy backend activo.")
        except (PackageNotFoundError, FileNotFoundError, Exception) as exc:
            self._moveit_py_init_error = exc
            self.get_logger().error(f"MoveItPy init fallida: {exc}")
