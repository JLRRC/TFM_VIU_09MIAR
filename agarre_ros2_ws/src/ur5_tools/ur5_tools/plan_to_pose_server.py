#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/plan_to_pose_server.py
# Contenido: F6.5 — action server PlanToPose (stub que delega en plan_to_pose_logic).
"""PlanToPose action server (stub).

Hospeda ``PlanToPose.action`` en ``/orchestrator/plan_to_pose``.
Acepta cualquier goal válido y simula planning + ejecución
publicando feedback intermedio. Devuelve éxito tras un breve sleep.

Esta versión F6.5 NO invoca al bridge MoveIt — el wiring real con
``/desired_grasp`` queda para F6.6. La razón es desacoplar la cadena
del orchestrator (el orchestrator ya espera este action y debe
poder probarse end-to-end sin Gazebo).

La lógica pura vive en ``plan_to_pose_logic.py`` y es 100% testeable
sin ROS.

Uso:
    ros2 run ur5_tools plan_to_pose_server
    ros2 run ur5_tools plan_to_pose_server --ros-args -p step_delay_sec:=0.05
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

import threading
import uuid

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

# F1.23 LIVE (2026-05-08): SwitchController para reset del joint_trajectory_controller
# tras retry-fail (BUG_CONTROLLER_FEEDBACK_HANG mitigation deeper).
try:
    from controller_manager_msgs.srv import SwitchController
except Exception:
    SwitchController = None  # type: ignore[misc,assignment]

from ur5_panel_interfaces.action import PlanToPose

from .plan_to_pose_helpers import (
    extract_ordered_joint_positions as _pure_extract_ordered_joint_positions,
    parse_plan_to_pose_request as _pure_parse_plan_to_pose_request,
    select_traj_duration_and_timeout as _pure_select_traj_duration_and_timeout,
)
from .plan_to_pose_logic import (
    PlanToPoseGoal,
    PlanToPoseResult,
    encode_request_frame,
    execute_stub,
    feedback_sequence,
    parse_bridge_result,
    result_matches_request,
    validate_goal,
)


class PlanToPoseServer(Node):
    """Action server PlanToPose (stub F6.5)."""

    def __init__(self) -> None:
        super().__init__("plan_to_pose_server")
        self._init_declare_and_read_params()
        self._init_setup_tf_and_clients()
        self._init_setup_action_server_and_bridge()

    # ------------------------------------------------------------------
    # __init__ helpers (extraídos en F1.24-refactor T15, 2026-05-08)
    # ------------------------------------------------------------------

    def _init_declare_and_read_params(self) -> None:
        """Declara y lee TODOS los parámetros ROS de este nodo.

        Sin side effects ROS más allá de declare_parameter / get_parameter.
        Setea atributos ``self._<name>``. Refactor offline 1:1 (no cambia
        defaults ni nombres de parámetro respecto a la versión inline).
        """
        # --- Action server / scaffolding ---
        self.declare_parameter("action_name", "/orchestrator/plan_to_pose")
        self.declare_parameter("step_delay_sec", 0.10)
        self.declare_parameter("planning_steps", 3)
        self.declare_parameter("executing_steps", 4)
        # --- Real bridge (F6.6 opt-in) ---
        self.declare_parameter("use_real_bridge", False)
        self.declare_parameter("bridge_pose_topic", "/desired_grasp")
        self.declare_parameter("bridge_result_topic", "/desired_grasp/result")
        self.declare_parameter("bridge_base_frame", "base_link")
        self.declare_parameter("bridge_result_timeout_sec", 60.0)
        # --- Mode resolution (B-iter3, 2026-05-03) ---
        # Valores: "STUB", "REAL_BRIDGE", "MOVEIT_DIRECT".
        # Backwards compat: mode="" + use_real_bridge=true ⇒ "REAL_BRIDGE";
        # mode="" + use_real_bridge=false ⇒ "STUB".
        self.declare_parameter("mode", "")
        # --- MoveIt direct (B-iter3 / B-iter14, 2026-05-03) ---
        self.declare_parameter("moveit_action_name", "/move_action")
        self.declare_parameter("moveit_group_name", "manipulator")
        self.declare_parameter("moveit_tip_link_override", "rg2_tcp")
        self.declare_parameter("moveit_planner_id", "")
        # 2026-05-07: planning_time 5→15s, result_timeout 30→60s — APPROACH
        # timeout sistemático con 5s desde pose post-HOME_INITIAL no canónica.
        self.declare_parameter("moveit_planning_time_sec", 15.0)
        self.declare_parameter("moveit_position_tol_m", 0.005)
        self.declare_parameter("moveit_orientation_tol_rad", 0.05)
        self.declare_parameter("moveit_goal_send_timeout_sec", 15.0)
        self.declare_parameter("moveit_result_timeout_sec", 60.0)
        # --- FJT directo (F1.24 H9+H10+H11+H14 LIVE, 2026-05-08) ---
        # bypass MoveIt vía /compute_ik + envío directo a FJT action,
        # cierra BUG_CONTROLLER_FEEDBACK_HANG (T35 × 3 cycles verde).
        self.declare_parameter("bypass_moveit_for_short_paths", False)
        self.declare_parameter(
            "fjt_direct_action_name",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("fjt_direct_compute_ik_service", "/compute_ik")
        self.declare_parameter("fjt_direct_duration_sec", 6.0)
        # F1.24 H14b (2026-05-09): IK timeout 2.0→5.0s. T35 × 5 stress reveló
        # que post-relaunch /compute_ik puede tardar 2-4s en converger
        # (TRAC-IK seed-dependent), causando err_val=-21 (TIMED_OUT) y
        # fallback al path MoveIt que dispara el bug original. 5s da margen
        # suficiente sin penalizar el caso normal.
        self.declare_parameter("fjt_direct_ik_timeout_sec", 5.0)
        self.declare_parameter("fjt_direct_result_timeout_sec", 30.0)
        self.declare_parameter("fjt_direct_max_bypass_dist_m", 1.2)
        self.declare_parameter("fjt_direct_max_joint_delta_rad", 3.5)
        # H14: 0.3 rad ≈ 17° — absorbe tracking errors transitorios sin
        # permitir desviaciones peligrosas. 0.0 = no enviar.
        self.declare_parameter("fjt_direct_path_tolerance_rad", 0.3)

        # --- Read into attributes ---
        self._action_name = str(
            self.get_parameter("action_name").value or "/orchestrator/plan_to_pose"
        ).strip()
        self._step_delay = float(self.get_parameter("step_delay_sec").value)
        self._planning_steps = int(self.get_parameter("planning_steps").value)
        self._executing_steps = int(self.get_parameter("executing_steps").value)

        self._use_real_bridge = bool(self.get_parameter("use_real_bridge").value)
        self._bridge_pose_topic = str(
            self.get_parameter("bridge_pose_topic").value or "/desired_grasp"
        ).strip()
        self._bridge_result_topic = str(
            self.get_parameter("bridge_result_topic").value or "/desired_grasp/result"
        ).strip()
        self._bridge_base_frame = str(
            self.get_parameter("bridge_base_frame").value or "base_link"
        ).strip() or "base_link"
        self._bridge_result_timeout = float(
            self.get_parameter("bridge_result_timeout_sec").value
        )

        _raw_mode = str(self.get_parameter("mode").value or "").strip().upper()
        if _raw_mode in {"STUB", "REAL_BRIDGE", "MOVEIT_DIRECT"}:
            self._mode = _raw_mode
        else:
            self._mode = "REAL_BRIDGE" if self._use_real_bridge else "STUB"
        # F7 audit (2026-05-10): STUB y REAL_BRIDGE están deprecados;
        # MOVEIT_DIRECT es el modo canónico desde B-iter3-bis. Los modos
        # legacy se mantienen para compatibilidad de tests offline pero
        # emiten warning en runtime para señalar la deprecación.
        if self._mode in {"STUB", "REAL_BRIDGE"}:
            self.get_logger().warning(
                f"[PLAN_TO_POSE][DEPRECATED] mode={self._mode} es legacy. "
                "Usa mode=MOVEIT_DIRECT (default desde 2026-05-03)."
            )

        self._moveit_action_name = str(
            self.get_parameter("moveit_action_name").value or "/move_action"
        ).strip() or "/move_action"
        self._moveit_group_name = str(
            self.get_parameter("moveit_group_name").value or "manipulator"
        ).strip() or "manipulator"
        self._moveit_planner_id = str(
            self.get_parameter("moveit_planner_id").value or ""
        ).strip()
        self._moveit_tip_link_override = str(
            self.get_parameter("moveit_tip_link_override").value or ""
        ).strip()
        self._moveit_planning_time = float(
            self.get_parameter("moveit_planning_time_sec").value
        )
        self._moveit_position_tol = float(
            self.get_parameter("moveit_position_tol_m").value
        )
        self._moveit_orientation_tol = float(
            self.get_parameter("moveit_orientation_tol_rad").value
        )
        self._moveit_goal_send_timeout = float(
            self.get_parameter("moveit_goal_send_timeout_sec").value
        )
        self._moveit_result_timeout = float(
            self.get_parameter("moveit_result_timeout_sec").value
        )

        self._bypass_moveit_for_short_paths = bool(
            self.get_parameter("bypass_moveit_for_short_paths").value
        )
        self._fjt_direct_action_name = str(
            self.get_parameter("fjt_direct_action_name").value
        )
        self._fjt_direct_ik_service = str(
            self.get_parameter("fjt_direct_compute_ik_service").value
        )
        self._fjt_direct_duration = float(
            self.get_parameter("fjt_direct_duration_sec").value
        )
        self._fjt_direct_ik_timeout = float(
            self.get_parameter("fjt_direct_ik_timeout_sec").value
        )
        self._fjt_direct_result_timeout = float(
            self.get_parameter("fjt_direct_result_timeout_sec").value
        )
        self._fjt_direct_max_bypass_dist = float(
            self.get_parameter("fjt_direct_max_bypass_dist_m").value
        )
        self._fjt_direct_max_joint_delta = float(
            self.get_parameter("fjt_direct_max_joint_delta_rad").value
        )
        self._fjt_direct_path_tolerance_rad = float(
            self.get_parameter("fjt_direct_path_tolerance_rad").value
        )

    def _init_setup_tf_and_clients(self) -> None:
        """Inicializa TF buffer/listener + lazy clients y subs.

        Side effects ROS: TransformListener subscribe, switch_controller
        client create, /joint_states subscription. Atributos lazy
        (``_moveit_action_client``, ``_fjt_direct_action_client``,
        ``_fjt_direct_ik_client``) quedan en None — se inicializan en
        primer uso.
        """
        # ActionClient /move_action (lazy en _execute_moveit_direct).
        self._moveit_action_client = None

        # F1.22 LIVE: TF buffer para verificar pose post-FIRST_ATTEMPT_TIMEOUT.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # F1.23 LIVE: cliente SwitchController para restart del JTC.
        self._switch_controller_client = None
        if SwitchController is not None:
            try:
                self._switch_controller_client = self.create_client(
                    SwitchController,
                    "/controller_manager/switch_controller",
                )
            except Exception as exc:
                self.get_logger().warning(
                    f"[PLAN_TO_POSE] no se pudo crear cliente switch_controller: {exc}"
                )

        # F1.24 / H9 LIVE: estado lazy del path FJT directo.
        self._fjt_direct_action_client = None
        self._fjt_direct_ik_client = None
        self._latest_joint_state = None
        self._joint_state_lock = threading.Lock()
        try:
            from sensor_msgs.msg import JointState as _JointState
            self.create_subscription(
                _JointState,
                "/joint_states",
                self._on_joint_state,
                10,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"[PLAN_TO_POSE] no se pudo crear sub /joint_states: {exc}"
            )

    def _init_setup_action_server_and_bridge(self) -> None:
        """Crea action server + (opcional) wiring del REAL_BRIDGE + log final."""
        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            PlanToPose,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        # F6.6: bridge wiring sólo si REAL_BRIDGE.
        self._bridge_pose_pub = None
        self._bridge_result_sub = None
        self._bridge_result_lock = threading.Lock()
        self._bridge_pending_uuid: Optional[str] = None
        self._bridge_pending_event = threading.Event()
        self._bridge_pending_text: Optional[str] = None
        self._next_request_id = 0

        if self._mode == "REAL_BRIDGE":
            self._bridge_pose_pub = self.create_publisher(
                PoseStamped, self._bridge_pose_topic, 10
            )
            self._bridge_result_sub = self.create_subscription(
                String,
                self._bridge_result_topic,
                self._on_bridge_result,
                10,
                callback_group=self._cb_group,
            )

        self.get_logger().info(
            f"[PLAN_TO_POSE] ready, action={self._action_name} mode={self._mode} "
            f"planning={self._planning_steps} executing={self._executing_steps} "
            f"step_delay={self._step_delay}s"
        )
        if self._mode == "REAL_BRIDGE":
            self.get_logger().info(
                f"[PLAN_TO_POSE] bridge pose_topic={self._bridge_pose_topic} "
                f"result_topic={self._bridge_result_topic} "
                f"base_frame={self._bridge_base_frame} "
                f"result_timeout={self._bridge_result_timeout:.1f}s"
            )
        elif self._mode == "MOVEIT_DIRECT":
            self.get_logger().info(
                f"[PLAN_TO_POSE] moveit_direct action={self._moveit_action_name} "
                f"group={self._moveit_group_name} planner='{self._moveit_planner_id}' "
                f"planning_time={self._moveit_planning_time:.1f}s "
                f"pos_tol={self._moveit_position_tol:.4f}m "
                f"ori_tol={self._moveit_orientation_tol:.3f}rad "
                f"goal_send_timeout={self._moveit_goal_send_timeout:.1f}s "
                f"result_timeout={self._moveit_result_timeout:.1f}s"
            )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, goal_request) -> GoalResponse:
        # Pre-validación rápida usando lógica pura.
        parsed = self._parse_goal(goal_request)
        ok, reason = validate_goal(parsed)
        if not ok:
            self.get_logger().warning(f"[PLAN_TO_POSE] goal rejected: {reason}")
            return GoalResponse.REJECT
        self.get_logger().info(
            f"[PLAN_TO_POSE] goal accepted: ee_frame={parsed.ee_frame} "
            f"cartesian={parsed.cartesian} target=({parsed.target_xyz[0]:.3f}, "
            f"{parsed.target_xyz[1]:.3f}, {parsed.target_xyz[2]:.3f})"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("[PLAN_TO_POSE] cancel requested")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle) -> PlanToPose.Result:
        request = goal_handle.request
        parsed = self._parse_goal(request)
        start_mono = time.monotonic()

        # Publicar feedback determinístico de la lógica pura.
        for fb in feedback_sequence(
            parsed,
            n_planning_steps=self._planning_steps,
            n_executing_steps=self._executing_steps,
        ):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = PlanToPose.Result()
                result.success = False
                result.reason = "canceled"
                result.duration_sec = time.monotonic() - start_mono
                result.attempts = 1
                return result
            self._publish_feedback(goal_handle, fb)
            time.sleep(max(0.0, self._step_delay))

        # Ejecutar según modo:
        #  - STUB: lógica pura (instantáneo, success).
        #  - REAL_BRIDGE: publica PoseStamped a /desired_grasp y espera panel.
        #  - MOVEIT_DIRECT (B-iter3): cliente directo a /move_action de MoveIt.
        duration = time.monotonic() - start_mono
        if self._mode == "MOVEIT_DIRECT":
            outcome = self._execute_moveit_direct(parsed, start_mono)
        elif self._mode == "REAL_BRIDGE":
            outcome = self._execute_real_bridge(parsed, start_mono)
        else:
            outcome = execute_stub(parsed, duration_sec=duration)

        result = PlanToPose.Result()
        result.success = bool(outcome.success)
        result.reason = str(outcome.reason)
        result.final_pose_base = self._final_pose_msg(outcome)
        result.duration_sec = float(outcome.duration_sec)
        result.attempts = int(outcome.attempts)

        if outcome.success:
            goal_handle.succeed()
            self.get_logger().info(
                f"[PLAN_TO_POSE] success duration={result.duration_sec:.2f}s"
            )
        else:
            goal_handle.abort()
            self.get_logger().warning(
                f"[PLAN_TO_POSE] aborted reason={result.reason}"
            )
        return result

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_goal(request) -> PlanToPoseGoal:
        """Wrapper sobre helper puro (F10 audit 2026-05-10)."""
        target_xyz, target_quat, ee_frame, cartesian, timeout_sec = (
            _pure_parse_plan_to_pose_request(request)
        )
        return PlanToPoseGoal(
            target_xyz=target_xyz,
            target_quat_xyzw=target_quat,
            ee_frame=ee_frame,
            cartesian=cartesian,
            timeout_sec=timeout_sec,
        )

    @staticmethod
    def _final_pose_msg(outcome):
        from geometry_msgs.msg import Pose, Point, Quaternion
        return Pose(
            position=Point(
                x=float(outcome.final_xyz[0]),
                y=float(outcome.final_xyz[1]),
                z=float(outcome.final_xyz[2]),
            ),
            orientation=Quaternion(
                x=float(outcome.final_quat_xyzw[0]),
                y=float(outcome.final_quat_xyzw[1]),
                z=float(outcome.final_quat_xyzw[2]),
                w=float(outcome.final_quat_xyzw[3]),
            ),
        )

    # ------------------------------------------------------------------
    # F6.6 — wiring real con el bridge MoveIt
    # ------------------------------------------------------------------

    def _execute_real_bridge(
        self, goal: PlanToPoseGoal, start_mono: float
    ) -> PlanToPoseResult:
        """Publica el goal al bridge y espera el result correlado por UUID.

        Action 12 (audit 2026-05-10): lógica extraída a
        ``plan_to_pose_real_bridge.execute_real_bridge`` para descongestionar
        el monolito (1507 LOC). Comportamiento idéntico.
        """
        from .plan_to_pose_real_bridge import execute_real_bridge
        return execute_real_bridge(self, goal, start_mono)

    # ------------------------------------------------------------------
    # B-iter3 (2026-05-03) — modo MOVEIT_DIRECT
    # ------------------------------------------------------------------

    def _execute_moveit_direct(
        self, goal: PlanToPoseGoal, start_mono: float
    ) -> PlanToPoseResult:
        """Llama directamente a /move_action de MoveIt 2 (sin bridge al panel).

        Flujo:
          0. F1.24 / H9 LIVE (2026-05-08): si bypass_moveit_for_short_paths,
             intentar primero FJT directo (sin pasar por simple_controller_manager).
             Si IK ok y controller responde → success. Si falla, falls back a MoveIt.
          1. Construye MoveGroup.Goal con PositionConstraint + OrientationConstraint
             sobre ee_frame del request.
          2. wait_for_server / send_goal / wait result (event-based).
          3. Decodifica MoveItErrorCodes.val a (success, reason).

        Refactor T15 (2026-05-09): split en 4 sub-helpers
        (_moveit_try_fjt_bypass, _moveit_send_first_attempt,
        _moveit_post_timeout_tf_check, _moveit_retry_after_failure,
        _moveit_final_tf_recovery). Función orquestadora <100 LOC.
        """
        from .plan_to_pose_logic import normalize_quat
        from .plan_to_pose_moveit_direct import parse_move_group_result

        # 0. FJT directo bypass.
        bypass = self._moveit_try_fjt_bypass(goal, start_mono)
        if bypass is not None:
            return bypass

        # 1. Build + send first attempt (lazy action client + tuning).
        sent = self._moveit_send_first_attempt(goal, start_mono)
        if isinstance(sent, PlanToPoseResult):
            return sent  # falla pre-result (server unavailable / send timeout / rejected)
        gh, mg_goal, first_attempt_timeout, effective_ee_frame = sent

        # 2. Esperar primer resultado (con first_attempt_timeout).
        result_future = gh.get_result_async()
        result_event = threading.Event()
        result_future.add_done_callback(lambda _f: result_event.set())
        result_event.wait(timeout=first_attempt_timeout)
        if not result_future.done():
            self.get_logger().warning(
                f"[PLAN_TO_POSE][MOVEIT_DIRECT] first attempt hang "
                f"timeout={first_attempt_timeout:.1f}s — cancelando goal y "
                "disparando retry (probable bridge race condition)"
            )
            try:
                cancel_future = gh.cancel_goal_async()
                cancel_event = threading.Event()
                cancel_future.add_done_callback(lambda _f: cancel_event.set())
                cancel_event.wait(timeout=2.0)
            except Exception:
                pass
            ok = False
            reason = f"FIRST_ATTEMPT_TIMEOUT:{first_attempt_timeout:.1f}s"
            # F1.22 LIVE: TF check post-timeout — si robot llegó, recoverar.
            recovered = self._moveit_post_timeout_tf_check(
                goal=goal,
                start_mono=start_mono,
                effective_ee_frame=effective_ee_frame,
                reason=reason,
            )
            if recovered is not None:
                return recovered
        else:
            wrapper = result_future.result()
            ok, reason = parse_move_group_result(wrapper)

        if ok:
            self.get_logger().info(
                f"[PLAN_TO_POSE][MOVEIT_DIRECT] success reason={reason}"
            )
            return PlanToPoseResult(
                success=ok,
                reason=reason,
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=1,
            )

        # 3. Retry path (CONTROL_FAILED / TIMED_OUT / FIRST_ATTEMPT_TIMEOUT).
        if (
            "CONTROL_FAILED" in reason
            or "TIMED_OUT" in reason
            or "FIRST_ATTEMPT_TIMEOUT" in reason
        ):
            retry = self._moveit_retry_after_failure(
                mg_goal=mg_goal,
                goal=goal,
                start_mono=start_mono,
                first_attempt_timeout=first_attempt_timeout,
                reason=reason,
            )
            if retry is not None and retry.success:
                return retry
            if retry is not None:
                reason = retry.reason

            # 4. Final TF recovery — si el robot sí llegó pese a todos los aborts.
            recovered_final = self._moveit_final_tf_recovery(
                goal=goal,
                start_mono=start_mono,
                effective_ee_frame=effective_ee_frame,
                reason=reason,
            )
            if recovered_final is not None:
                return recovered_final

        self.get_logger().warning(
            f"[PLAN_TO_POSE][MOVEIT_DIRECT] failed reason={reason}"
        )
        return PlanToPoseResult(
            success=ok,
            reason=reason,
            final_xyz=goal.target_xyz,
            final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
            duration_sec=time.monotonic() - start_mono,
            attempts=1,
        )

    def _moveit_try_fjt_bypass(
        self, goal: PlanToPoseGoal, start_mono: float
    ) -> Optional["PlanToPoseResult"]:
        """F1.24 / H9 LIVE: intento prioritario FJT directo.
        Returns:
            PlanToPoseResult si FJT directo respondió (success o fail terminal).
            None si pre-condiciones no cumplidas → caller debe seguir con MoveIt.
        """
        if not self._bypass_moveit_for_short_paths:
            return None
        self.get_logger().info(
            "[PLAN_TO_POSE][FJT_DIRECT] intento prioritario "
            "(bypass_moveit_for_short_paths=true)"
        )
        fjt_result = self._execute_fjt_direct(goal, start_mono)
        if fjt_result is not None:
            # Sí ejecutó (success o fail terminal — no fallback).
            return fjt_result
        # None = pre-condiciones no cumplidas → fall through al path MoveIt.
        self.get_logger().info(
            "[PLAN_TO_POSE][FJT_DIRECT] fallback al path MoveIt"
        )
        return None

    def _moveit_send_first_attempt(
        self, goal: PlanToPoseGoal, start_mono: float
    ):
        """Lazy ActionClient + wait_for_server + build_move_group_goal + send +
        wait_send. Returns:
            (gh, mg_goal, first_attempt_timeout, effective_ee_frame) en éxito.
            PlanToPoseResult en cualquier fallo previo al primer result wait.
        """
        from rclpy.action import ActionClient as _ActionClient
        from moveit_msgs.action import MoveGroup
        from .plan_to_pose_logic import normalize_quat
        from .plan_to_pose_moveit_direct import (
            build_move_group_goal,
            classify_phase_by_target_z,
        )

        if self._moveit_action_client is None:
            self._moveit_action_client = _ActionClient(
                self, MoveGroup, self._moveit_action_name,
                callback_group=self._cb_group,
            )
        if not self._moveit_action_client.wait_for_server(timeout_sec=3.0):
            return PlanToPoseResult(
                success=False,
                reason=f"moveit_action_server_unavailable:{self._moveit_action_name}",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=0,
            )
        effective_ee_frame = (
            self._moveit_tip_link_override
            if self._moveit_tip_link_override
            else goal.ee_frame
        )
        # F1.18 audit-v4: heurística per-fase (TRANSPORT scaling=0.5 + timeout=240s,
        # resto scaling=0.25 + timeout=120s).
        phase_tuning = classify_phase_by_target_z(goal.target_xyz)
        mg_goal = build_move_group_goal(
            goal.target_xyz,
            goal.target_quat_xyzw,
            ee_frame=effective_ee_frame,
            base_frame=self._bridge_base_frame,
            group_name=self._moveit_group_name,
            planner_id=self._moveit_planner_id,
            planning_time_sec=self._moveit_planning_time,
            position_tol_m=self._moveit_position_tol,
            orientation_tol_rad=self._moveit_orientation_tol,
            velocity_scaling_factor=phase_tuning.velocity_scaling,
            acceleration_scaling_factor=phase_tuning.acceleration_scaling,
        )
        self.get_logger().info(
            "[PLAN_TO_POSE][MOVEIT_DIRECT] sending goal "
            f"target=({goal.target_xyz[0]:.3f},{goal.target_xyz[1]:.3f},"
            f"{goal.target_xyz[2]:.3f}) "
            f"ee_frame={goal.ee_frame} group={self._moveit_group_name}"
        )
        send_future = self._moveit_action_client.send_goal_async(mg_goal)
        send_event = threading.Event()
        send_future.add_done_callback(lambda _f: send_event.set())
        send_event.wait(timeout=max(3.0, self._moveit_goal_send_timeout))
        if not send_future.done():
            return PlanToPoseResult(
                success=False,
                reason="moveit_goal_send_timeout",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=1,
            )
        gh = send_future.result()
        if gh is None or not getattr(gh, "accepted", False):
            return PlanToPoseResult(
                success=False,
                reason="moveit_goal_rejected",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=1,
            )
        first_attempt_timeout = min(
            phase_tuning.first_attempt_timeout_sec,
            float(max(1.0, self._moveit_result_timeout)),
        )
        return gh, mg_goal, first_attempt_timeout, effective_ee_frame

    def _moveit_post_timeout_tf_check(
        self,
        *,
        goal: PlanToPoseGoal,
        start_mono: float,
        effective_ee_frame: str,
        reason: str,
    ) -> Optional["PlanToPoseResult"]:
        """F1.22 LIVE: tras FIRST_ATTEMPT_TIMEOUT, verifica si el robot ya alcanzó
        el target via TF lookup. Si sí, devuelve PlanToPoseResult success
        ``feedback_hang_recovered_via_tf``. None si no recovery."""
        from .plan_to_pose_logic import normalize_quat

        try:
            tf_target_pos = self._lookup_ee_position_in_base(
                ee_frame=effective_ee_frame,
                base_frame=self._bridge_base_frame,
                timeout_sec=1.0,
            )
        except Exception as exc:
            self.get_logger().warning(
                f"[PLAN_TO_POSE][MOVEIT_DIRECT] TF lookup post-timeout fail: "
                f"{type(exc).__name__}: {exc}"
            )
            return None
        if tf_target_pos is None:
            return None
        tf_check_tol = max(0.05, self._moveit_position_tol * 5.0)
        dx = float(tf_target_pos[0]) - float(goal.target_xyz[0])
        dy = float(tf_target_pos[1]) - float(goal.target_xyz[1])
        dz = float(tf_target_pos[2]) - float(goal.target_xyz[2])
        dist = (dx * dx + dy * dy + dz * dz) ** 0.5
        self.get_logger().info(
            f"[PLAN_TO_POSE][MOVEIT_DIRECT] post-timeout TF check "
            f"actual=({tf_target_pos[0]:.3f},{tf_target_pos[1]:.3f},"
            f"{tf_target_pos[2]:.3f}) target=({goal.target_xyz[0]:.3f},"
            f"{goal.target_xyz[1]:.3f},{goal.target_xyz[2]:.3f}) "
            f"dist={dist:.4f}m tol={tf_check_tol:.4f}m"
        )
        if dist > tf_check_tol:
            return None
        self.get_logger().info(
            f"[PLAN_TO_POSE][MOVEIT_DIRECT] feedback_hang_recovered "
            f"(robot at target post-timeout, dist={dist:.4f}m"
            f"<={tf_check_tol:.4f}m) — returning success"
        )
        return PlanToPoseResult(
            success=True,
            reason=(
                f"feedback_hang_recovered_via_tf"
                f"|orig={reason}|dist={dist:.4f}m"
            ),
            final_xyz=goal.target_xyz,
            final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
            duration_sec=time.monotonic() - start_mono,
            attempts=1,
        )

    def _moveit_retry_after_failure(
        self,
        *,
        mg_goal,
        goal: PlanToPoseGoal,
        start_mono: float,
        first_attempt_timeout: float,
        reason: str,
    ) -> Optional["PlanToPoseResult"]:
        """F1.23 LIVE: controller restart + retry MoveIt goal tras 20s sleep.
        Returns:
            PlanToPoseResult(success=True) si retry funcionó.
            PlanToPoseResult(success=False, reason=...) si retry falló pero
            quedó info útil (caller hará final TF check).
            None si nada que reportar.
        """
        from .plan_to_pose_logic import normalize_quat
        from .plan_to_pose_moveit_direct import parse_move_group_result

        if "FIRST_ATTEMPT_TIMEOUT" in reason:
            self.get_logger().warning(
                "[PLAN_TO_POSE][MOVEIT_DIRECT] reset joint_trajectory_controller "
                "antes del retry (BUG_CONTROLLER_FEEDBACK_HANG mitigation)"
            )
            self._restart_joint_trajectory_controller(timeout_sec=3.0)
        self.get_logger().warning(
            f"[PLAN_TO_POSE][MOVEIT_DIRECT] failed reason={reason} — "
            "intentando retry tras 20s (race condition controller_manager)"
        )
        time.sleep(20.0)
        send_future_retry = self._moveit_action_client.send_goal_async(mg_goal)
        retry_send_event = threading.Event()
        send_future_retry.add_done_callback(lambda _f: retry_send_event.set())
        retry_send_event.wait(timeout=max(3.0, self._moveit_goal_send_timeout))
        if not send_future_retry.done():
            return PlanToPoseResult(
                success=False,
                reason=f"{reason}|retry_send_timeout",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=2,
            )
        gh_retry = send_future_retry.result()
        if gh_retry is None or not getattr(gh_retry, "accepted", False):
            return PlanToPoseResult(
                success=False,
                reason=f"{reason}|retry_rejected",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=2,
            )
        result_future_retry = gh_retry.get_result_async()
        retry_result_event = threading.Event()
        result_future_retry.add_done_callback(lambda _f: retry_result_event.set())
        retry_timeout_effective = float(
            min(first_attempt_timeout, max(1.0, self._moveit_result_timeout))
        )
        retry_result_event.wait(timeout=retry_timeout_effective)
        if not result_future_retry.done():
            try:
                cancel_r = gh_retry.cancel_goal_async()
                evt_cr = threading.Event()
                cancel_r.add_done_callback(lambda _f: evt_cr.set())
                evt_cr.wait(timeout=2.0)
            except Exception:
                pass
            return PlanToPoseResult(
                success=False,
                reason=f"{reason}|retry_first_attempt_hang",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=2,
            )
        wrapper_retry = result_future_retry.result()
        ok_retry, reason_retry = parse_move_group_result(wrapper_retry)
        if ok_retry:
            self.get_logger().info(
                f"[PLAN_TO_POSE][MOVEIT_DIRECT] retry success reason={reason_retry}"
            )
            return PlanToPoseResult(
                success=True,
                reason=f"{reason_retry}|retry_after_{reason}",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=2,
            )
        return PlanToPoseResult(
            success=False,
            reason=f"{reason_retry}|retry_failed",
            final_xyz=goal.target_xyz,
            final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
            duration_sec=time.monotonic() - start_mono,
            attempts=2,
        )

    def _moveit_final_tf_recovery(
        self,
        *,
        goal: PlanToPoseGoal,
        start_mono: float,
        effective_ee_frame: str,
        reason: str,
    ) -> Optional["PlanToPoseResult"]:
        """F1.23 LIVE: tras todos los intentos fallidos, último TF check.
        Si el robot llegó al target durante alguno de los attempts, devolver
        success ``feedback_hang_recovered_final``. None si no recovery."""
        from .plan_to_pose_logic import normalize_quat

        try:
            final_tf = self._lookup_ee_position_in_base(
                ee_frame=effective_ee_frame,
                base_frame=self._bridge_base_frame,
                timeout_sec=1.5,
            )
        except Exception:
            return None
        if final_tf is None:
            return None
        final_tol = max(0.05, self._moveit_position_tol * 5.0)
        fdx = float(final_tf[0]) - float(goal.target_xyz[0])
        fdy = float(final_tf[1]) - float(goal.target_xyz[1])
        fdz = float(final_tf[2]) - float(goal.target_xyz[2])
        fdist = (fdx * fdx + fdy * fdy + fdz * fdz) ** 0.5
        self.get_logger().info(
            f"[PLAN_TO_POSE][MOVEIT_DIRECT] final TF check "
            f"actual=({final_tf[0]:.3f},{final_tf[1]:.3f},"
            f"{final_tf[2]:.3f}) target=({goal.target_xyz[0]:.3f},"
            f"{goal.target_xyz[1]:.3f},{goal.target_xyz[2]:.3f}) "
            f"dist={fdist:.4f}m tol={final_tol:.4f}m"
        )
        if fdist > final_tol:
            return None
        self.get_logger().info(
            "[PLAN_TO_POSE][MOVEIT_DIRECT] feedback_hang_recovered_final "
            "(robot at target post-retry, feedback lost) — success"
        )
        return PlanToPoseResult(
            success=True,
            reason=(
                f"feedback_hang_recovered_final"
                f"|orig={reason}|dist={fdist:.4f}m"
            ),
            final_xyz=goal.target_xyz,
            final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
            duration_sec=time.monotonic() - start_mono,
            attempts=2,
        )

    def _on_joint_state(self, msg) -> None:
        """Cachea el último JointState para usar como seed IK."""
        with self._joint_state_lock:
            self._latest_joint_state = msg

    def _get_latest_joint_state(self):
        """Devuelve copia del último JointState o None."""
        with self._joint_state_lock:
            return self._latest_joint_state

    # UR5 arm joints (orden esperado por el controller).
    _UR5_JOINTS = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )

    def _execute_fjt_direct(
        self, goal: PlanToPoseGoal, start_mono: float
    ) -> Optional["PlanToPoseResult"]:
        """F1.24 / H9+H10+H11+H14 LIVE (2026-05-08): bypass MoveIt vía FJT directo.

        Flujo:
        1. Resuelve ee_frame y extrae seed positions del joint_state cacheado.
        2. Calcula traj params per-distancia (TF lookup ee_frame → target).
        3. Llama ``/compute_ik`` para obtener target joints (normalizados a [-π, π]).
        4. Construye JointTrajectory (2-point para fases cortas, multi-waypoint
           para distancias > 0.4m a.k.a. TRANSPORT).
        5. Envía al ``/joint_trajectory_controller/follow_joint_trajectory``
           con path_tolerance=H14 y espera "Goal reached, success!".

        Returns:
            PlanToPoseResult si se ejecutó (success o failure),
            None si pre-condiciones no satisfechas (caller debe fallback a MoveIt).

        Refactor T15 (2026-05-09): split en 4 sub-helpers
        (_fjt_extract_seed_positions, _fjt_compute_traj_params,
        _fjt_call_compute_ik, _fjt_send_and_wait_result). Fix bug latente
        F1.24 donde ``seed_positions`` se referenciaba antes de definirse,
        haciendo que ``dist_to_target=0.5`` siempre (multi-waypoint always-on).
        """
        # 1. Seed.
        seed_positions = self._fjt_extract_seed_positions()
        if seed_positions is None:
            return None

        # 2. ee_frame efectivo.
        ee_frame = (
            self._moveit_tip_link_override
            if self._moveit_tip_link_override
            else goal.ee_frame
        )

        # 3. Distance-based tuning (TF lookup ee_frame).
        fjt_duration_eff, fjt_result_timeout_eff, is_long_traj, dist_to_target = (
            self._fjt_compute_traj_params(goal, ee_frame)
        )
        if dist_to_target > max(0.0, self._fjt_direct_max_bypass_dist):
            self.get_logger().info(
                "[PLAN_TO_POSE][FJT_DIRECT] skip bypass: "
                f"dist_to_target={dist_to_target:.3f}m > "
                f"max_bypass={self._fjt_direct_max_bypass_dist:.3f}m"
            )
            return None

        # 4. IK síncrono → target joints normalizados.
        target_joints = self._fjt_call_compute_ik(goal, ee_frame, seed_positions)
        if target_joints is None:
            return None
        max_delta = max(
            abs(float(t) - float(s))
            for s, t in zip(seed_positions, target_joints)
        )
        if max_delta > max(0.0, self._fjt_direct_max_joint_delta):
            self.get_logger().warning(
                "[PLAN_TO_POSE][FJT_DIRECT] IK branch rejected before FJT: "
                f"max_joint_delta={max_delta:.3f}rad > "
                f"limit={self._fjt_direct_max_joint_delta:.3f}rad"
            )
            return None

        # 5. Build trajectory (per-distancia: H11 multi-waypoint si is_long_traj).
        jt = self._fjt_build_trajectory(
            seed_positions=seed_positions,
            target_joints=target_joints,
            is_long_traj=is_long_traj,
            duration_sec=fjt_duration_eff,
        )

        # 6. Send + wait + parse FJT result.
        return self._fjt_send_and_wait_result(
            jt=jt,
            goal=goal,
            start_mono=start_mono,
            result_timeout_sec=fjt_result_timeout_eff,
        )

    def _fjt_extract_seed_positions(self) -> Optional[list]:
        """Lee joint_state cacheado y devuelve seed positions ordenadas.

        F10 (auditoría 2026-05-10): la lógica pura de reordenación vive
        en ``plan_to_pose_helpers.extract_ordered_joint_positions``.
        Aquí solo convertimos el msg ROS a payload dict y manejamos el
        logging cuando falta algo.
        """
        js = self._get_latest_joint_state()
        payload = None
        if js is not None:
            payload = {
                "name": getattr(js, "name", None) or [],
                "position": getattr(js, "position", None) or [],
            }
        positions, missing = _pure_extract_ordered_joint_positions(
            payload, self._UR5_JOINTS
        )
        if positions is None:
            if missing == "no_payload":
                self.get_logger().warning(
                    "[PLAN_TO_POSE][FJT_DIRECT] no joint_state cached — "
                    "fallback a MoveIt"
                )
            else:
                self.get_logger().warning(
                    f"[PLAN_TO_POSE][FJT_DIRECT] joint {missing} no en "
                    "/joint_states — fallback"
                )
        return positions

    def _fjt_compute_traj_params(
        self, goal: PlanToPoseGoal, ee_frame: str
    ) -> tuple:
        """Calcula (duration_sec, result_timeout_sec, is_long_traj, dist).

        F10 (auditoría 2026-05-10): la política de selección
        duración/timeout vive en ``plan_to_pose_helpers.
        select_traj_duration_and_timeout``. Aquí solo hacemos el TF
        lookup y el log.
        """
        import math as _math

        try:
            tf_pos = self._lookup_ee_position_in_base(
                ee_frame=ee_frame,
                base_frame=self._bridge_base_frame,
                timeout_sec=0.5,
            )
        except Exception:
            tf_pos = None
        if tf_pos is not None:
            dx = float(tf_pos[0]) - float(goal.target_xyz[0])
            dy = float(tf_pos[1]) - float(goal.target_xyz[1])
            dz = float(tf_pos[2]) - float(goal.target_xyz[2])
            dist = _math.sqrt(dx * dx + dy * dy + dz * dz)
        else:
            dist = 0.5  # conservador → ruta larga
        duration_eff, timeout_eff, is_long = (
            _pure_select_traj_duration_and_timeout(
                dist_m=dist,
                default_duration_sec=self._fjt_direct_duration,
                default_timeout_sec=self._fjt_direct_result_timeout,
            )
        )
        self.get_logger().info(
            f"[PLAN_TO_POSE][FJT_DIRECT] dist_to_target={dist:.3f}m "
            f"duration={duration_eff:.1f}s timeout={timeout_eff:.1f}s "
            f"multi_waypoint={is_long}"
        )
        return duration_eff, timeout_eff, is_long, dist

    def _fjt_call_compute_ik(
        self,
        goal: PlanToPoseGoal,
        ee_frame: str,
        seed_positions: list,
    ) -> Optional[list]:
        """Llama ``/compute_ik`` con timeout configurable y devuelve target
        joints normalizados a [-π, π] (H10). Devuelve None y loguea si falla."""
        from .fjt_direct_helpers import build_ik_request, parse_ik_result

        # Cliente lazy.
        if self._fjt_direct_ik_client is None:
            try:
                from moveit_msgs.srv import GetPositionIK
                self._fjt_direct_ik_client = self.create_client(
                    GetPositionIK, self._fjt_direct_ik_service
                )
            except Exception as exc:
                self.get_logger().warning(
                    f"[PLAN_TO_POSE][FJT_DIRECT] cliente IK no disponible: {exc}"
                )
                return None
        if not self._fjt_direct_ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                "[PLAN_TO_POSE][FJT_DIRECT] /compute_ik service no ready — fallback"
            )
            return None
        ik_req = build_ik_request(
            target_xyz=goal.target_xyz,
            target_quat_xyzw=goal.target_quat_xyzw,
            ee_frame=ee_frame,
            base_frame=self._bridge_base_frame,
            group_name=self._moveit_group_name,
            current_joints=seed_positions,
            joint_names=list(self._UR5_JOINTS),
            timeout_sec=self._fjt_direct_ik_timeout,
            avoid_collisions=False,
        )
        ik_future = self._fjt_direct_ik_client.call_async(ik_req)
        ik_event = threading.Event()
        ik_future.add_done_callback(lambda _f: ik_event.set())
        ik_event.wait(timeout=float(self._fjt_direct_ik_timeout) + 1.0)
        if not ik_future.done():
            self.get_logger().warning(
                "[PLAN_TO_POSE][FJT_DIRECT] IK timeout — fallback a MoveIt"
            )
            return None
        ok_ik, reason_ik, target_joints = parse_ik_result(
            ik_future.result(), list(self._UR5_JOINTS)
        )
        if not ok_ik or target_joints is None:
            self.get_logger().info(
                f"[PLAN_TO_POSE][FJT_DIRECT] IK failed: {reason_ik} — fallback a MoveIt"
            )
            return None
        self.get_logger().info(
            "[PLAN_TO_POSE][FJT_DIRECT] IK OK joints=("
            f"{target_joints[0]:+.3f},{target_joints[1]:+.3f},"
            f"{target_joints[2]:+.3f},{target_joints[3]:+.3f},"
            f"{target_joints[4]:+.3f},{target_joints[5]:+.3f}) "
            f"target=({goal.target_xyz[0]:.3f},{goal.target_xyz[1]:.3f},"
            f"{goal.target_xyz[2]:.3f})"
        )
        return target_joints

    def _fjt_build_trajectory(
        self,
        *,
        seed_positions: list,
        target_joints: list,
        is_long_traj: bool,
        duration_sec: float,
    ):
        """Wrapper sobre ``build_fjt_trajectory_*``: multi-waypoint si la
        trayectoria es larga (TRANSPORT), 2-point si corta."""
        from .fjt_direct_helpers import (
            build_fjt_trajectory_multi_point,
            build_fjt_trajectory_two_point,
        )

        joint_names = list(self._UR5_JOINTS)
        if is_long_traj:
            return build_fjt_trajectory_multi_point(
                joint_names=joint_names,
                start_positions=seed_positions,
                target_positions=target_joints,
                num_intermediate_points=8,
                total_duration_sec=duration_sec,
            )
        return build_fjt_trajectory_two_point(
            joint_names=joint_names,
            start_positions=seed_positions,
            target_positions=target_joints,
            duration_sec=duration_sec,
        )

    def _fjt_send_and_wait_result(
        self,
        *,
        jt,
        goal: PlanToPoseGoal,
        start_mono: float,
        result_timeout_sec: float,
    ) -> Optional["PlanToPoseResult"]:
        """Envía el FJT goal (con H14 path_tolerance), espera resultado y
        decodifica error_code. Devuelve PlanToPoseResult(success=True) si
        error_code==0, None en cualquier otro caso (caller fallback)."""
        from .plan_to_pose_logic import normalize_quat

        # Action client lazy.
        if self._fjt_direct_action_client is None:
            try:
                from rclpy.action import ActionClient as _ActionClient
                from control_msgs.action import FollowJointTrajectory
                self._fjt_direct_action_client = _ActionClient(
                    self,
                    FollowJointTrajectory,
                    self._fjt_direct_action_name,
                )
            except Exception as exc:
                self.get_logger().warning(
                    f"[PLAN_TO_POSE][FJT_DIRECT] FJT action client error: {exc}"
                )
                return None
        if not self._fjt_direct_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning(
                f"[PLAN_TO_POSE][FJT_DIRECT] FJT action server "
                f"{self._fjt_direct_action_name} no ready — fallback"
            )
            return None
        from control_msgs.action import FollowJointTrajectory as _FJT
        fjt_goal = _FJT.Goal()
        fjt_goal.trajectory = jt
        if self._fjt_direct_path_tolerance_rad > 0.0:
            from .fjt_direct_helpers import build_fjt_path_tolerances
            fjt_goal.path_tolerance = build_fjt_path_tolerances(
                joint_names=list(self._UR5_JOINTS),
                position_tolerance_rad=self._fjt_direct_path_tolerance_rad,
            )

        # Send goal.
        send_future = self._fjt_direct_action_client.send_goal_async(fjt_goal)
        send_event = threading.Event()
        send_future.add_done_callback(lambda _f: send_event.set())
        send_event.wait(timeout=3.0)
        if not send_future.done():
            self.get_logger().warning(
                "[PLAN_TO_POSE][FJT_DIRECT] FJT goal send timeout — fallback"
            )
            return None
        gh = send_future.result()
        if gh is None or not getattr(gh, "accepted", False):
            self.get_logger().warning(
                "[PLAN_TO_POSE][FJT_DIRECT] FJT goal rejected — fallback"
            )
            return None

        # Wait result.
        result_future = gh.get_result_async()
        result_event = threading.Event()
        result_future.add_done_callback(lambda _f: result_event.set())
        result_event.wait(timeout=result_timeout_sec)
        if not result_future.done():
            self.get_logger().warning(
                "[PLAN_TO_POSE][FJT_DIRECT] FJT result timeout — fallback"
            )
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
            return None

        wrapper = result_future.result()
        ec = getattr(getattr(wrapper, "result", None), "error_code", 0)
        if isinstance(ec, int):
            ec_val = int(ec)
        elif hasattr(ec, "val"):
            ec_val = int(getattr(ec, "val", 0))
        else:
            ec_val = 0
        if ec_val == 0:
            self.get_logger().info(
                "[PLAN_TO_POSE][FJT_DIRECT] success (bypass MoveIt OK)"
            )
            return PlanToPoseResult(
                success=True,
                reason="fjt_direct:SUCCESSFUL",
                final_xyz=goal.target_xyz,
                final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
                duration_sec=time.monotonic() - start_mono,
                attempts=1,
            )
        self.get_logger().warning(
            f"[PLAN_TO_POSE][FJT_DIRECT] FJT error_code={ec_val} — fallback a MoveIt"
        )
        return None

    def _restart_joint_trajectory_controller(self, *, timeout_sec: float = 3.0) -> bool:
        """F1.23 LIVE (2026-05-08): deactivate + activate joint_trajectory_controller.

        Mitigación del bug BUG_CONTROLLER_FEEDBACK_HANG cuando retry también
        falla con FIRST_ATTEMPT_TIMEOUT. Hipótesis: el controller queda en
        un estado "ghost-busy" tras un cancel mal procesado por el bridge
        simple_controller_manager. El restart fuerza una transición de
        estado limpia.

        Returns:
            True si deactivate + activate fueron OK; False en cualquier fail.
        """
        if SwitchController is None or self._switch_controller_client is None:
            self.get_logger().warning(
                "[PLAN_TO_POSE] switch_controller no disponible; skip restart"
            )
            return False
        if not self._switch_controller_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                "[PLAN_TO_POSE] switch_controller service not ready"
            )
            return False
        # Step 1: deactivate
        deactivate_req = SwitchController.Request()
        deactivate_req.deactivate_controllers = ["joint_trajectory_controller"]
        deactivate_req.activate_controllers = []
        deactivate_req.strictness = SwitchController.Request.BEST_EFFORT
        deactivate_req.activate_asap = False
        try:
            future_d = self._switch_controller_client.call_async(deactivate_req)
            event_d = threading.Event()
            future_d.add_done_callback(lambda _f: event_d.set())
            event_d.wait(timeout=float(timeout_sec))
            if not future_d.done():
                self.get_logger().warning("[PLAN_TO_POSE] deactivate timeout")
                return False
            resp_d = future_d.result()
            if resp_d is None or not bool(getattr(resp_d, "ok", False)):
                self.get_logger().warning(
                    f"[PLAN_TO_POSE] deactivate failed: ok=False "
                    f"msg={getattr(resp_d, 'message', '?')}"
                )
                # Continuar igual al activate (BEST_EFFORT)
        except Exception as exc:
            self.get_logger().warning(
                f"[PLAN_TO_POSE] deactivate exception: {type(exc).__name__}: {exc}"
            )
            return False
        # Pause breve para que el controller release recursos
        time.sleep(1.0)
        # Step 2: activate
        activate_req = SwitchController.Request()
        activate_req.deactivate_controllers = []
        activate_req.activate_controllers = ["joint_trajectory_controller"]
        activate_req.strictness = SwitchController.Request.BEST_EFFORT
        activate_req.activate_asap = True
        try:
            future_a = self._switch_controller_client.call_async(activate_req)
            event_a = threading.Event()
            future_a.add_done_callback(lambda _f: event_a.set())
            event_a.wait(timeout=float(timeout_sec))
            if not future_a.done():
                self.get_logger().warning("[PLAN_TO_POSE] activate timeout")
                return False
            resp_a = future_a.result()
            if resp_a is None or not bool(getattr(resp_a, "ok", False)):
                self.get_logger().warning(
                    f"[PLAN_TO_POSE] activate failed: ok=False "
                    f"msg={getattr(resp_a, 'message', '?')}"
                )
                return False
            self.get_logger().info(
                "[PLAN_TO_POSE] joint_trajectory_controller restarted OK"
            )
            return True
        except Exception as exc:
            self.get_logger().warning(
                f"[PLAN_TO_POSE] activate exception: {type(exc).__name__}: {exc}"
            )
            return False

    def _lookup_ee_position_in_base(
        self,
        *,
        ee_frame: str,
        base_frame: str,
        timeout_sec: float = 1.0,
    ):
        """F1.22 LIVE (2026-05-08): lookup posición ee_frame en base_frame.

        Usado tras FIRST_ATTEMPT_TIMEOUT para detectar el bug
        BUG_CONTROLLER_FEEDBACK_HANG: el robot llegó al target pero el
        controller feedback no llegó al move_group.

        Returns:
            Tuple[float, float, float] con (x, y, z) en base_frame, o None
            si TF lookup falla.
        """
        try:
            ts = self._tf_buffer.lookup_transform(
                str(base_frame),
                str(ee_frame),
                rclpy.time.Time(),
                timeout=Duration(seconds=float(timeout_sec)),
            )
            tx = float(ts.transform.translation.x)
            ty = float(ts.transform.translation.y)
            tz = float(ts.transform.translation.z)
            return (tx, ty, tz)
        except Exception:
            return None

    def _on_bridge_result(self, msg) -> None:
        """Callback de /desired_grasp/result. Despierta el ejecutor si UUID OK."""
        text = str(getattr(msg, "data", "") or "")
        if not text:
            return
        with self._bridge_result_lock:
            expected = self._bridge_pending_uuid
            if expected is None:
                return  # no hay request pendiente
            if not result_matches_request(text, expected):
                # Result de otra request (concurrencia). Ignorar.
                return
            self._bridge_pending_text = text
        self._bridge_pending_event.set()

    # ------------------------------------------------------------------

    def _publish_feedback(self, goal_handle, fb):
        msg = PlanToPose.Feedback()
        msg.current_state = str(fb.current_state)
        msg.progress = float(fb.progress)
        msg.attempts = int(fb.attempts)
        msg.detail = str(fb.detail)
        try:
            goal_handle.publish_feedback(msg)
        except Exception as exc:
            self.get_logger().warning(
                f"[PLAN_TO_POSE] feedback publish failed: {exc}"
            )


# F3.2 audit (2026-05-10): main() movido a plan_to_pose_runtime para
# evitar ciclo de imports y mantener server.py centrado en la clase.
# El entry_point setup.py apunta ahora a plan_to_pose_runtime:main.
