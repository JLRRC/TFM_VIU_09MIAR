"""FjtLifecycleMixin: ciclo de vida del goal FollowJointTrajectory.

Extraído de :mod:`ur5_tools.moveit_bridge.executor` (audit-v4.1, 2026-05-08)
como continuación de F5-iter1/2/3. Los 3 helpers ya estaban extraídos
in-place en `executor.py` desde commits 3b31f45 / f645c44 / b9fbd7b, pero
permanecían dentro del mismo archivo, dejando `executor.py` en 1.663 LOC
(+117 sobre el baseline v4 de 1.546). Este módulo materializa la
extracción real moviendo los 3 helpers a un mixin propio:

* ``_prepare_fjt_execution``  — fase de preparación (cold-start, rescaling,
  timeout cap APPROACH, goal_time_tolerance efectivo).
* ``_send_and_accept_fjt_goal`` — build + send goal + wait for accepted.
* ``_setup_post_accept_state`` — captura ``start_joint_vec``, calcula el
  deadline monotónico y propaga al lock.

Los métodos siguen siendo métodos de instancia y dependen del resto de
mixins de ``UR5MoveItBridge`` vía ``self`` (sin cambios de contrato).
La cadena MRO es la responsable de resolver las dependencias.
"""

from __future__ import annotations

import time
from typing import Any

from trajectory_msgs.msg import JointTrajectory


class FjtLifecycleMixin:
    """Métodos de ciclo de vida del goal FJT extraídos de ExecutorMixin.

    Sin estado propio. Requiere los siguientes métodos/atributos vía
    ``self`` (provistos por otros mixins de ``UR5MoveItBridge``):

    * ``self._pose_lock``
    * ``self._first_controller_goal_pending``
    * ``self._active_exec_timeout_sec`` / ``_active_exec_timeout_deadline_mono``
    * ``self._force_fjt_direct_for_walltime_sim``
    * ``self._env_float`` (FromControllerManagementMixin / params)
    * ``self._compute_effective_goal_time_tol_sec``
    * ``self._scale_joint_trajectory_timing`` (TrajectoryPrepMixin)
    * ``self._prepare_joint_trajectory_for_controller`` (TrajectoryPrepMixin)
    * ``self._joint_trajectory_duration_sec`` (TrajectoryPrepMixin)
    * ``self._maybe_scale_approach_replan_traj``
    * ``self._fjt_timeout_for_trajectory``
    * ``self._ensure_fjt_action_client``
    * ``self._build_fjt_goal_with_tolerances``
    * ``self._make_fjt_feedback_cb``
    * ``self._wait_future_done``
    * ``self._current_arm_joint_vector`` (JointStateHelpersMixin)
    * ``self.get_logger`` (rclpy Node)
    """

    def _setup_post_accept_state(
        self,
        *,
        action_name: str,
        timeout_sec: float,
    ) -> tuple[Any, float]:
        """F5-iter3 audit-v4 (2026-05-08): post-accept state setup.

        Captura el start_joint_vec, calcula el deadline monotónico, y
        actualiza los slots de active_exec_timeout en el lock.

        Returns:
            (start_joint_vec, exec_deadline_mono). start_joint_vec puede
            ser None si la captura falló (warning ya emitido).
        """
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
        return start_joint_vec, exec_deadline_mono

    def _send_and_accept_fjt_goal(
        self,
        jt: JointTrajectory,
        *,
        action_name: str,
        path_tol_override_rad: float | None,
        effective_goal_time_tol_sec: float,
        timeout_sec: float,
        cold_start_first_goal: bool,
    ) -> tuple[
        Any,                # goal_handle | None
        str,                # reason ("" if accepted, else canonical fail reason)
        dict[str, Any],     # meta
        Any,                # feedback_state
        Any,                # feedback_lock
        list[str],          # joint_names
        list[float],        # target_joint_positions
    ]:
        """F5-iter2 audit-v4 (2026-05-08): build + send goal, wait for accept.

        Extrae el bloque ``client = ensure ... client.send_goal_async ... accepted``
        del monolito ``_execute_joint_trajectory_action`` (lines 640-668).

        Returns:
            (goal_handle, reason, meta, feedback_state, feedback_lock,
             joint_names, target_joint_positions). Si reason != "" el caller
             debe propagar (False, reason, meta) y abortar.
        """
        client = self._ensure_fjt_action_client()
        if client is None:
            return (
                None, "fjt_action_client_unavailable",
                {"action": action_name}, None, None, [], [],
            )
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
            return (
                None, "fjt_goal_send_timeout", {"action": action_name},
                None, None, [], [],
            )
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return (
                None,
                "fjt_goal_rejected",
                {
                    "action": action_name,
                    "accepted": bool(getattr(goal_handle, "accepted", False)),
                },
                None, None, [], [],
            )
        if cold_start_first_goal:
            try:
                with self._pose_lock:
                    self._first_controller_goal_pending = False
            except Exception:
                pass
        return (
            goal_handle, "", {}, feedback_state, feedback_lock,
            joint_names, target_joint_positions,
        )

    def _prepare_fjt_execution(
        self,
        jt: JointTrajectory,
        *,
        timeout_sec: float,
        retry_on_tolerance_violation: bool,
        goal_time_override_sec: float | None,
        phase_label_upper: str,
        approach_replan_attempt: int,
    ) -> tuple[JointTrajectory, float, float, bool, float]:
        """F5 audit-v4 (2026-05-08): extract preparation phase from
        _execute_joint_trajectory_action.

        Returns:
            (jt_prepared, timeout_sec_capped, effective_goal_time_tol_sec,
             cold_start_first_goal, prepared_traj_sec)
        """
        cold_start_first_goal = False
        try:
            with self._pose_lock:
                cold_start_first_goal = bool(self._first_controller_goal_pending)
        except Exception:
            cold_start_first_goal = False
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
        return (
            jt,
            float(timeout_sec),
            effective_goal_time_tol_sec,
            cold_start_first_goal,
            float(prepared_traj_sec),
        )
