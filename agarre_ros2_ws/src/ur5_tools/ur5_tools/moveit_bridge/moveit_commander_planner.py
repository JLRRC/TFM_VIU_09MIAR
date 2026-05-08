"""MoveItCommanderMixin: planificación vía moveit_commander + cartesian path.

Extraído de ``ur5_tools/ur5_moveit_bridge.py`` (líneas 4315-4550 originales).

Tres métodos:

* ``_plan_with_moveit_commander``: backend de planificación tradicional.
* ``_get_cartesian_group``: lazy MoveGroupCommander para cartesian.
* ``_plan_cartesian``: invoca ``/compute_cartesian_path`` y ejecuta vía FJT.

Sin estado propio; todo accede a atributos del nodo principal vía ``self``.
La clase concreta debe heredar también de ``rclpy.node.Node``.
"""

from __future__ import annotations

import json
import os
import time

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetCartesianPath

try:
    from moveit_commander.move_group import MoveGroupCommander  # type: ignore
except Exception:  # pragma: no cover - moveit_commander opcional
    MoveGroupCommander = None  # type: ignore


# F2-step5 (audit-v4 2026-05-08): env reads del cartesian path centralizadas
# como constantes module-level. Override en runtime via env var (no hot-reload):
#   MOVEIT_BRIDGE_CARTESIAN_MAX_STEP=<float>
#   MOVEIT_BRIDGE_CARTESIAN_JUMP_THRESHOLD=<float>
#   MOVEIT_BRIDGE_CARTESIAN_AVOID_COLLISIONS=true|false
def _read_float_env(name: str, default: float) -> float:
    raw = os.environ.get(name, "")
    if not raw:
        return default
    try:
        return float(raw)
    except (TypeError, ValueError):
        return default


def _read_bool_env(name: str, default: bool) -> bool:
    raw = os.environ.get(name, "true" if default else "false").strip().lower()
    return raw in ("1", "true", "yes", "on")


_CART_MAX_STEP_DEFAULT = _read_float_env("MOVEIT_BRIDGE_CARTESIAN_MAX_STEP", 0.005)
_CART_JUMP_THRESHOLD_DEFAULT = _read_float_env(
    "MOVEIT_BRIDGE_CARTESIAN_JUMP_THRESHOLD", 5.0
)
_CART_AVOID_COLLISIONS_DEFAULT = _read_bool_env(
    "MOVEIT_BRIDGE_CARTESIAN_AVOID_COLLISIONS", False
)


class MoveItCommanderMixin:
    """Plan + execute vía moveit_commander y servicio /compute_cartesian_path."""

    def _plan_with_moveit_commander(self, target: PoseStamped) -> tuple[bool, str, bool, bool]:
        if not self._move_group:
            self.get_logger().error("moveit_commander no inicializado.")
            return False, "moveit_commander_not_ready", False, False
        try:
            eef = str(self._move_group.get_end_effector_link() or "")
        except Exception:
            eef = ""
        self.get_logger().info(
            "[BRIDGE_EXEC] "
            f"backend=moveit_commander ee_link_param={self._ee_frame} "
            f"ee_link_move_group={eef or 'n/a'} controller={self._controller_name} "
            f"action={self._controller_action_name}"
        )
        self.get_logger().info(
            "[BRIDGE_PLAN] USING scaling "
            f"v={self._max_velocity_scaling:.2f} a={self._max_acceleration_scaling:.2f}"
        )
        self._move_group.set_pose_target(target.pose)
        plan = self._move_group.plan()
        trajectory = self._extract_trajectory(plan)
        if trajectory is None:
            self._log_bridge_status(
                "[BRIDGE_STATUS] plan_fail backend=moveit_commander reason=no_trajectory",
                level="warn",
            )
            self.get_logger().warning("Planificación con MoveIt fallida.")
            self._move_group.clear_pose_targets()
            return False, "plan_no_trajectory", False, False

        self._log_bridge_status("[BRIDGE_STATUS] plan_ok backend=moveit_commander")
        self.get_logger().info("Planificación con MoveIt OK.")
        plan_endpoint_ok, plan_endpoint_detail = self._planned_trajectory_target_consistent(
            trajectory,
            target_pose=target,
            tol_m=max(
                0.02,
                self._env_float("PANEL_MOVEIT_BRIDGE_PLAN_EE_TARGET_TOL_M", 0.08),
            ),
        )
        if not plan_endpoint_ok:
            self._move_group.clear_pose_targets()
            self._log_bridge_status(
                "[BRIDGE_STATUS] plan_fail backend=moveit_commander reason=plan_endpoint_target_mismatch",
                level="warn",
            )
            self.get_logger().warning(
                "Planificación con MoveIt rechazada por endpoint geométricamente incoherente. "
                f"detail={plan_endpoint_detail}"
            )
            return False, f"plan_endpoint_target_mismatch:{plan_endpoint_detail}", False, False
        if self._dry_run_plan_only:
            self._log_bridge_status(
                "[BRIDGE_STATUS] exec_skip backend=moveit_commander reason=dry_run_plan_only"
            )
            self.get_logger().warning(
                "[BRIDGE_DRY_RUN] plan generated and execution skipped (dry_run_plan_only=true)."
            )
            self._move_group.clear_pose_targets()
            return True, "dry_run_plan_only", True, False
        success = self._move_group.execute(trajectory, wait=True)
        if success:
            self._log_bridge_status("[BRIDGE_STATUS] exec_ok backend=moveit_commander")
            self.get_logger().info("Ejecución MoveIt completada.")
            out = (True, "exec_ok", True, True)
        else:
            diag = self._exec_diagnostics()
            detail = self._diag_to_message(diag)
            self._log_bridge_status(
                f"[BRIDGE_STATUS] exec_fail backend=moveit_commander reason={diag['reason']} detail={detail}",
                level="warn",
            )
            self.get_logger().warning(
                "Ejecución MoveIt fallida. "
                f"diag={json.dumps(diag, ensure_ascii=True)}"
            )
            published = self._publish_planned_joint_trajectory(trajectory)
            if published:
                self._log_bridge_status(
                    "[BRIDGE_STATUS] exec_fallback backend=moveit_commander mode=topic_publish"
                )
                out = (True, f"exec_fallback_topic_publish:{detail}", True, True)
            else:
                out = (False, f"exec_failed:{detail}", True, False)
        self._move_group.clear_pose_targets()
        return out

    def _get_cartesian_group(self) -> "MoveGroupCommander | None":
        if self._backend == "moveit_commander" and self._move_group:
            return self._move_group
        if MoveGroupCommander is None:
            self.get_logger().warning("moveit_commander no disponible para cartesian.")
            return None
        if self._cartesian_group is None:
            try:
                self._cartesian_group = MoveGroupCommander(self._group_name)
                self._cartesian_group.set_pose_reference_frame(self._base_frame)
                self._configure_move_group_scaling(self._cartesian_group)
            except Exception as exc:
                self.get_logger().warning(f"MoveGroupCommander cartesian fallo: {exc}")
                self._cartesian_group = None
        return self._cartesian_group

    def _plan_cartesian(self, target: PoseStamped) -> tuple[bool, str, bool, bool]:
        if self._cartesian_client is None:
            self.get_logger().warning("Cartesian: cliente no disponible.")
            return False, "cartesian_client_unavailable", False, False
        if not self._cartesian_client.wait_for_service(timeout_sec=2.0):
            self._log_bridge_status(
                "[BRIDGE_STATUS] cartesian_fail reason=service_unavailable",
                level="warn",
            )
            self.get_logger().warning(
                "Cartesian: servicio /compute_cartesian_path no disponible."
            )
            return False, "cartesian_service_unavailable", False, False
        req = GetCartesianPath.Request()
        req.header = target.header
        req.group_name = self._group_name
        req.link_name = self._ee_frame
        req.waypoints = [target.pose]
        # FIX 2026-05-04 (bug GRASP_DOWN cartesian fraction=0.000):
        # max_step=0.005 + jump_threshold=0.0 + avoid_collisions=True
        # rechazaban el cartesian path al intentar bajar el TCP al objeto.
        # MoveIt computaba 1 waypoint (start) y se detenía con fraction=0.
        #
        # jump_threshold=0.0 = "no permitir NINGÚN salto en joint space".
        # Para grasp con TCP cerca del target (28mm de descenso), esto
        # es demasiado restrictivo. jump_threshold=5.0 permite los saltos
        # típicos de cartesian descent sin permitir teleporte.
        #
        # avoid_collisions=False permite que el TCP haga contacto con el
        # objeto (necesario para grasp). Override con env var:
        #   MOVEIT_BRIDGE_CARTESIAN_AVOID_COLLISIONS=true
        #   MOVEIT_BRIDGE_CARTESIAN_JUMP_THRESHOLD=<float>
        #   MOVEIT_BRIDGE_CARTESIAN_MAX_STEP=<float>
        # F2-step5 (audit-v4): defaults leídos a nivel módulo (constantes).
        req.max_step = _CART_MAX_STEP_DEFAULT
        req.jump_threshold = _CART_JUMP_THRESHOLD_DEFAULT
        req.avoid_collisions = _CART_AVOID_COLLISIONS_DEFAULT
        start_state_msg, start_state_reason = self._build_start_robot_state_msg()
        if start_state_msg is not None:
            req.start_state = start_state_msg
            self.get_logger().info(
                f"[BRIDGE_START_STATE] cartesian detail={start_state_reason}"
            )
        else:
            self.get_logger().warning(
                "[BRIDGE_START_STATE] cartesian unavailable "
                f"detail={start_state_reason}; fallback=current_state_monitor"
            )
        if hasattr(req, "max_velocity_scaling_factor"):
            req.max_velocity_scaling_factor = float(self._max_velocity_scaling)
        if hasattr(req, "max_acceleration_scaling_factor"):
            req.max_acceleration_scaling_factor = float(self._max_acceleration_scaling)
        self.get_logger().info(
            "[BRIDGE_CART] USING scaling "
            f"v={self._max_velocity_scaling:.2f} a={self._max_acceleration_scaling:.2f}"
        )
        future = self._cartesian_client.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            self._log_bridge_status(
                "[BRIDGE_STATUS] cartesian_fail reason=timeout", level="warn"
            )
            self.get_logger().warning("Cartesian: timeout esperando respuesta.")
            return False, "cartesian_timeout", False, False
        resp = future.result()
        if resp is None:
            self._log_bridge_status(
                "[BRIDGE_STATUS] cartesian_fail reason=empty_response",
                level="warn",
            )
            self.get_logger().warning("Cartesian: respuesta vacia.")
            return False, "cartesian_empty_response", False, False
        fraction = float(resp.fraction)
        self.get_logger().info(f"Cartesian path fraction={fraction:.3f}")
        if fraction < 0.99:
            self._log_bridge_status(
                "[BRIDGE_STATUS] cartesian_fail reason=low_fraction",
                level="warn",
            )
            self.get_logger().warning(
                f"Cartesian path incompleto (fraction={fraction:.3f})."
            )
            return False, f"cartesian_low_fraction:{fraction:.3f}", False, False
        traj = resp.solution
        if not traj.joint_trajectory.points:
            self._log_bridge_status(
                "[BRIDGE_STATUS] cartesian_fail reason=empty_trajectory",
                level="warn",
            )
            self.get_logger().warning("Cartesian: trayectoria vacia.")
            return False, "cartesian_empty_trajectory", False, False
        final_tfs = traj.joint_trajectory.points[-1].time_from_start
        final_sec = float(final_tfs.sec) + float(final_tfs.nanosec) / 1_000_000_000.0
        self._log_bridge_status("[BRIDGE_STATUS] cartesian_ok")
        self.get_logger().info(
            f"Cartesian planning OK (servicio). points={len(traj.joint_trajectory.points)} "
            f"duration={final_sec:.3f}s"
        )
        cartesian_plan_ok, cartesian_plan_detail = self._planned_trajectory_target_consistent(
            traj.joint_trajectory,
            target_pose=target,
            tol_m=max(
                0.02,
                self._env_float("PANEL_MOVEIT_BRIDGE_PLAN_EE_TARGET_TOL_M", 0.08),
            ),
            phase_label="CARTESIAN",
        )
        if not cartesian_plan_ok:
            self._log_bridge_status(
                "[BRIDGE_STATUS] cartesian_fail reason=plan_endpoint_target_mismatch",
                level="warn",
            )
            self.get_logger().warning(
                "Cartesian planning rechazado por endpoint geométricamente incoherente. "
                f"detail={cartesian_plan_detail}"
            )
            return False, f"cartesian_plan_endpoint_target_mismatch:{cartesian_plan_detail}", False, False
        if self._dry_run_plan_only:
            self._log_bridge_status(
                "[BRIDGE_STATUS] exec_skip backend=moveit_py reason=dry_run_plan_only_cartesian"
            )
            self.get_logger().warning(
                "[BRIDGE_DRY_RUN] cartesian plan generated and execution skipped "
                "(dry_run_plan_only=true)."
            )
            return True, "dry_run_plan_only", True, False
        fjt_timeout = self._fjt_timeout_for_trajectory(
            final_sec,
            extra_margin_sec=4.0,
            minimum_sec=8.0,
        )
        fjt_ok, fjt_msg, fjt_meta = self._execute_joint_trajectory_action(
            traj.joint_trajectory,
            timeout_sec=fjt_timeout,
            target_pose=target,
        )
        if fjt_ok:
            self._log_bridge_status(
                "[BRIDGE_STATUS] exec_ok backend=moveit_py mode=fjt_direct_cartesian"
            )
            return True, f"cartesian_exec_ok:{fjt_msg}", True, True
        fjt_detail = ",".join(f"{k}={v}" for k, v in fjt_meta.items())
        self._log_bridge_status(
            "[BRIDGE_STATUS] exec_fail backend=moveit_py reason=cartesian_fjt_failed",
            level="warn",
        )
        return False, f"cartesian_exec_failed:{fjt_msg};fjt_meta={fjt_detail}", True, False
