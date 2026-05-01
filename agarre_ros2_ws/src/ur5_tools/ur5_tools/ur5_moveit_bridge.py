# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/ur5_moveit_bridge.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
"""Node that bridges a grasp pose to MoveIt planning/execution for the UR5."""

from __future__ import annotations

from collections import deque
import gc
import json
import os
from pathlib import Path
import sys
import threading
import time
import traceback
from typing import Any

try:
    from moveit.planning import MoveItPy, PlanningComponent  # type: ignore
except Exception as exc:  # pragma: no cover
    MoveItPy = None  # type: ignore
    PlanningComponent = None  # type: ignore
    _MOVEIT_PY_IMPORT_ERROR = exc
else:
    _MOVEIT_PY_IMPORT_ERROR = None

try:
    from moveit.core.robot_state import RobotState as MoveItRobotState  # type: ignore
except Exception:  # pragma: no cover
    MoveItRobotState = None  # type: ignore

try:
    import moveit_commander  # type: ignore
    from moveit_commander.move_group import MoveGroupCommander  # type: ignore
    from moveit_commander.robot_trajectory import RobotTrajectory  # type: ignore
except Exception as exc:  # pragma: no cover
    moveit_commander = None  # type: ignore
    MoveGroupCommander = None  # type: ignore
    RobotTrajectory = None  # type: ignore
    _MOVEIT_COMMANDER_IMPORT_ERROR = exc
else:
    _MOVEIT_COMMANDER_IMPORT_ERROR = None
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetCartesianPath
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)
from trajectory_msgs.msg import JointTrajectory
from controller_manager_msgs.srv import ListControllers

from .moveit_bridge.controller_management import ControllerManagementMixin
from .moveit_bridge.log_formatters import (
    format_busy_message as _fmt_busy_message,
    format_exec_start_log as _fmt_exec_start_log,
    format_pick_request_log as _fmt_pick_request_log,
    format_recv_log as _fmt_recv_log,
    format_rx_log as _fmt_rx_log,
    format_target_log as _fmt_target_log,
)
from .moveit_bridge.queue_helpers import (
    compute_request_stamp_ns as _compute_request_stamp_ns,
    compute_settle_params as _compute_settle_params,
    is_invalid_business_frame as _is_invalid_business_frame,
    is_stale_request as _is_stale_request,
)
from .moveit_bridge.executor import ExecutorMixin
from .moveit_bridge.geometry import GeometryMixin
from .moveit_bridge.goal_validation import GoalValidationMixin
from .moveit_bridge.joint_state_helpers import JointStateHelpersMixin
from .moveit_bridge.moveit_commander_planner import MoveItCommanderMixin
from .moveit_bridge.moveit_py_planner import MoveItPyPlannerMixin
from .moveit_bridge.trajectory_prep import TrajectoryPrepMixin
from .param_utils import read_float_param, read_str_list_param, read_str_param
from .moveit_bridge_utils import (
    bridge_env_float,
    parse_request_meta,
    plan_success_code,
    plan_success_ok,
    plan_error_code_val,
    describe_execute_result,
    result_meta_to_message,
    diag_to_message,
    goal_status_text,
    wait_future_done,
)

_BRIDGE_CODE_REV = "2026-04-20-approach-replan-v1"


class UR5MoveItBridge(
    MoveItPyPlannerMixin,
    MoveItCommanderMixin,
    GeometryMixin,
    TrajectoryPrepMixin,
    ExecutorMixin,
    JointStateHelpersMixin,
    ControllerManagementMixin,
    GoalValidationMixin,
    LifecycleNode,
):
    """Subscribes to grasp poses and drives MoveIt planning/execution.

    F13 (2026-05-01): migrado a ``LifecycleNode`` (observable). La
    inicialización completa permanece en ``__init__`` por la
    complejidad del backend MoveItPy + 8 mixins. Las transiciones
    lifecycle retornan SUCCESS sin re-crear recursos para preservar
    operatividad. Esto permite que ``system_state_manager`` coordine
    globalmente. Una segregación estricta de recursos a
    on_configure/on_activate queda como F13b.

    El parámetro ``auto_activate`` (default True) preserva el
    comportamiento de los launch existentes.
    """

    def _log_bridge_status(self, message: str, *, level: str = "info") -> None:
        if level == "warn":
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def __init__(self) -> None:
        super().__init__("ur5_moveit_bridge")
        # F13 lifecycle: auto-activate por defecto preserva backward-compat.
        if not self.has_parameter("auto_activate"):
            self.declare_parameter("auto_activate", True)
        self.declare_parameter("backend", "auto")
        self.declare_parameter("move_group", "manipulator")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "rg2_tcp")
        self.declare_parameter("result_topic", "/desired_grasp/result")
        self.declare_parameter("heartbeat_topic", "/ur5_moveit_bridge/heartbeat")
        self.declare_parameter("heartbeat_rate_hz", 2.0)
        # Contract: topics may come from any frame; bridge normalizes into base_frame.
        self.declare_parameter("pose_topics", ["/desired_grasp", "/grasp_pose"])
        self.declare_parameter("cartesian_pose_topics", ["/desired_grasp_cartesian"])
        self.declare_parameter("robot_name", "ur5_rg2")
        self.declare_parameter("ur_type", "ur5")
        self.declare_parameter("moveit_config_pkg", "ur5_moveit_config")
        self.declare_parameter("description_pkg", "ur5_description")
        self.declare_parameter("srdf_path", "")
        self.declare_parameter("urdf_xacro_path", "")
        self.declare_parameter("kinematics_yaml", "")
        self.declare_parameter("joint_limits_yaml", "")
        self.declare_parameter("moveit_controllers_yaml", "")
        self.declare_parameter("planning_pipelines", ["ompl"])
        self.declare_parameter("default_planning_pipeline", "ompl")
        self.declare_parameter("min_plan_interval_sec", 0.5)
        self.declare_parameter("execute_timeout_sec", 30.0)
        self.declare_parameter("request_timeout_sec", 35.0)
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("moveit_py_use_sim_time", False)
        self.declare_parameter("allow_unsafe_moveit_py_sim_time", False)
        self.declare_parameter("force_fjt_direct_for_walltime_sim", False)
        self.declare_parameter("require_request_id", True)
        self.declare_parameter("drop_pending_on_tagged_request", True)
        self.declare_parameter("stale_request_ttl_sec", 20.0)
        self.declare_parameter("dry_run_plan_only", False)
        self.declare_parameter("joint_state_valid_timeout_sec", 2.0)
        self.declare_parameter("joint_state_valid_max_age_sec", 1.0)
        self.declare_parameter("unwrap_continuous_joints", False)
        self.declare_parameter("max_velocity_scaling_factor", 0.30)
        self.declare_parameter("max_acceleration_scaling_factor", 0.30)
        self.declare_parameter("path_constraint_joint_tolerance_rad", 0.0)
        self.declare_parameter("controller_path_tolerance_rad", -1.0)
        self.declare_parameter("controller_goal_tolerance_rad", -1.0)
        self.declare_parameter("controller_goal_time_tolerance_sec", -1.0)
        self.declare_parameter("controller_expected_goal_time_sec", 12.0)
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)

        self._backend_pref = read_str_param(self, "backend", "auto").strip().lower()
        self._group_name = read_str_param(self, "move_group", "manipulator")
        self._base_frame = read_str_param(self, "base_frame", "base_link")
        self._ee_frame = read_str_param(self, "ee_frame", "rg2_tcp")
        self._result_topic = read_str_param(
            self, "result_topic", "/desired_grasp/result"
        )
        self._heartbeat_topic = read_str_param(
            self, "heartbeat_topic", "/ur5_moveit_bridge/heartbeat"
        )
        self._heartbeat_rate_hz = read_float_param(
            self, "heartbeat_rate_hz", 2.0, min_value=0.2
        )
        self._pose_topics = read_str_list_param(
            self,
            "pose_topics",
            default=["/desired_grasp", "/grasp_pose"],
        )
        self._cartesian_pose_topics = read_str_list_param(
            self,
            "cartesian_pose_topics",
            default=["/desired_grasp_cartesian"],
        )
        self._robot_name = read_str_param(self, "robot_name", "ur5_rg2")
        self._ur_type = read_str_param(self, "ur_type", "ur5")
        self._moveit_config_pkg = read_str_param(
            self,
            "moveit_config_pkg",
            "ur5_moveit_config",
        )
        self._description_pkg = read_str_param(
            self, "description_pkg", "ur5_description"
        )
        self._srdf_path = read_str_param(self, "srdf_path", "")
        self._urdf_xacro_path = read_str_param(self, "urdf_xacro_path", "")
        self._kinematics_yaml = read_str_param(self, "kinematics_yaml", "")
        self._joint_limits_yaml = read_str_param(self, "joint_limits_yaml", "")
        self._moveit_controllers_yaml = read_str_param(
            self, "moveit_controllers_yaml", ""
        )
        self._planning_pipelines = read_str_list_param(
            self,
            "planning_pipelines",
            default=["ompl"],
        )
        self._default_planning_pipeline = read_str_param(
            self, "default_planning_pipeline", "ompl"
        )
        self._min_plan_interval = read_float_param(
            self,
            "min_plan_interval_sec",
            0.5,
            min_value=0.0,
        )
        self._execute_timeout_sec = read_float_param(
            self,
            "execute_timeout_sec",
            30.0,
            min_value=1.0,
        )
        self._request_timeout_sec = read_float_param(
            self,
            "request_timeout_sec",
            35.0,
            min_value=2.0,
        )
        self._controller_manager_name = read_str_param(
            self, "controller_manager", "/controller_manager"
        ).strip()
        self._require_request_id = bool(self.get_parameter("require_request_id").value)
        self._drop_pending_on_tagged_request = bool(
            self.get_parameter("drop_pending_on_tagged_request").value
        )
        self._stale_request_ttl_sec = read_float_param(
            self,
            "stale_request_ttl_sec",
            20.0,
            min_value=0.0,
        )
        self._dry_run_plan_only = bool(
            self.get_parameter("dry_run_plan_only").value
        )
        self._joint_state_valid_timeout_sec = read_float_param(
            self,
            "joint_state_valid_timeout_sec",
            2.0,
            min_value=0.1,
        )
        self._joint_state_valid_max_age_sec = read_float_param(
            self,
            "joint_state_valid_max_age_sec",
            1.0,
            min_value=0.05,
        )
        self._unwrap_continuous_joints = bool(
            self.get_parameter("unwrap_continuous_joints").value
        )
        self._max_velocity_scaling = read_float_param(
            self,
            "max_velocity_scaling_factor",
            0.30,
            min_value=0.0,
            max_value=1.0,
        )
        self._max_acceleration_scaling = read_float_param(
            self,
            "max_acceleration_scaling_factor",
            0.30,
            min_value=0.0,
            max_value=1.0,
        )
        if self._max_velocity_scaling <= 0.0:
            self._max_velocity_scaling = 0.30
        if self._max_acceleration_scaling <= 0.0:
            self._max_acceleration_scaling = 0.30
        self._path_constraint_joint_tol = read_float_param(
            self,
            "path_constraint_joint_tolerance_rad",
            0.0,
            min_value=0.0,
        )
        self._controller_path_tolerance_rad = float(
            self.get_parameter("controller_path_tolerance_rad").value
        )
        self._controller_goal_tolerance_rad = float(
            self.get_parameter("controller_goal_tolerance_rad").value
        )
        self._controller_goal_time_tolerance_sec = float(
            self.get_parameter("controller_goal_time_tolerance_sec").value
        )
        self._controller_expected_goal_time_sec = read_float_param(
            self,
            "controller_expected_goal_time_sec",
            12.0,
            min_value=0.0,
        )
        self._use_sim_time = bool(self.get_parameter("use_sim_time").value)
        requested_moveit_py_sim = bool(
            self.get_parameter("moveit_py_use_sim_time").value
        )
        self._force_fjt_direct_for_walltime_sim = bool(
            self.get_parameter("force_fjt_direct_for_walltime_sim").value
        )
        self._moveit_py_use_sim_time = bool(requested_moveit_py_sim)
        allow_unsafe_sim_time = bool(
            self.get_parameter("allow_unsafe_moveit_py_sim_time").value
        )
        if self._use_sim_time and self._moveit_py_use_sim_time and not allow_unsafe_sim_time:
            self.get_logger().warning(
                "moveit_py_use_sim_time=true solicitado con use_sim_time=true; "
                "forzando false para evitar crash qos_overrides./clock.subscription.durability "
                "(set allow_unsafe_moveit_py_sim_time=true bajo tu responsabilidad)."
            )
            self._moveit_py_use_sim_time = False
        if self._use_sim_time and not self._moveit_py_use_sim_time:
            if not self._force_fjt_direct_for_walltime_sim:
                self._force_fjt_direct_for_walltime_sim = True
                self.get_logger().warning(
                    "use_sim_time=true pero moveit_py_use_sim_time=false; "
                    "auto-activando force_fjt_direct_for_walltime_sim para evitar "
                    "abort/timeout falsos en execute y delegar la ejecucion al "
                    "action server FollowJointTrajectory."
                )
            else:
                self.get_logger().warning(
                    "use_sim_time=true pero moveit_py_use_sim_time=false; "
                    "usando force_fjt_direct_for_walltime_sim para evitar "
                    "validacion temporal inconsistente en execute."
                )
            controller_goal_time_sec = max(
                0.0,
                float(self._controller_expected_goal_time_sec),
                float(self._controller_goal_time_tolerance_sec),
            )
            retry_cushion_sec = max(
                30.0,
                (2.0 * controller_goal_time_sec) + 10.0,
            )
            min_request_timeout = max(
                90.0,
                float(self._execute_timeout_sec) + retry_cushion_sec,
            )
            if float(self._request_timeout_sec) < min_request_timeout:
                self._request_timeout_sec = float(min_request_timeout)
                self.get_logger().warning(
                    "ajustando request_timeout_sec para cubrir planificacion + "
                    "FollowJointTrajectory en dominio temporal mixto: "
                    f"request_timeout_sec={self._request_timeout_sec:.1f}"
                )
        self.get_logger().info(
            "Bridge config: "
            f"rev={_BRIDGE_CODE_REV} "
            f"backend_pref={self._backend_pref or 'auto'} group={self._group_name} "
            f"base={self._base_frame} ee={self._ee_frame} result_topic={self._result_topic} "
            f"execute_timeout_sec={self._execute_timeout_sec:.1f} "
            f"request_timeout_sec={self._request_timeout_sec:.1f} "
            f"use_sim_time={str(self._use_sim_time).lower()} "
            f"moveit_py_use_sim_time={str(self._moveit_py_use_sim_time).lower()} "
            f"force_fjt_direct_for_walltime_sim={str(self._force_fjt_direct_for_walltime_sim).lower()} "
            f"require_request_id={str(self._require_request_id).lower()} "
            f"drop_pending_on_tagged_request={str(self._drop_pending_on_tagged_request).lower()} "
            f"stale_request_ttl_sec={self._stale_request_ttl_sec:.1f} "
            f"dry_run_plan_only={str(self._dry_run_plan_only).lower()}"
        )
        self.get_logger().info(
            "[BRIDGE_CFG] USING scaling "
            f"v={self._max_velocity_scaling:.2f} a={self._max_acceleration_scaling:.2f} "
            f"joint_state_timeout={self._joint_state_valid_timeout_sec:.2f}s "
            f"joint_state_max_age={self._joint_state_valid_max_age_sec:.2f}s "
            f"unwrap_continuous_joints={str(self._unwrap_continuous_joints).lower()} "
            f"path_constraint_joint_tol={self._path_constraint_joint_tol:.3f}rad "
            f"controller_path_tol={self._controller_path_tolerance_rad:.3f}rad "
            f"controller_goal_tol={self._controller_goal_tolerance_rad:.3f}rad "
            f"controller_goal_time_tol={self._controller_goal_time_tolerance_sec:.3f}s "
            f"controller_expected_goal_time={self._controller_expected_goal_time_sec:.3f}s"
        )

        self._backend = None
        self._moveit_py = None
        self._planning_component = None
        self._move_group = None
        self._moveit_py_init_thread: threading.Thread | None = None
        self._cartesian_group = None
        self._cartesian_client = None
        self._traj_pub = None
        self._moveit_py_ready = False
        self._moveit_py_init_error = None
        self._controller_name = "joint_trajectory_controller"
        self._controller_action_ns = "follow_joint_trajectory"
        self._controller_action_name = "/joint_trajectory_controller/follow_joint_trajectory"
        self._controller_config_source = "default"
        self._action_clients: dict[str, ActionClient] = {}
        self._action_client_lock = threading.Lock()
        self._fjt_action_name = ""
        self._fjt_client: ActionClient | None = None
        self._fjt_prime_timer = None
        self._list_controllers_client = None
        self._joint_state_stamp_ns = 0
        self._joint_state_recv_mono = 0.0
        self._joint_state_names = 0
        self._joint_state_positions = 0
        self._joint_state_last_names: list[str] = []
        self._joint_state_last_positions: list[float] = []
        if self._backend_pref and self._backend_pref not in (
            "auto",
            "moveit_py",
            "moveit_commander",
        ):
            self.get_logger().warning(
                f"backend desconocido '{self._backend_pref}', usando auto."
            )
            self._backend_pref = "auto"

        if self._backend_pref == "moveit_py":
            if MoveItPy is None or PlanningComponent is None:
                raise RuntimeError("MoveItPy solicitado pero no disponible")
            self._backend = "moveit_py"
            self.get_logger().info("MoveItPy backend seleccionado; inicializando...")
        elif self._backend_pref == "moveit_commander":
            if moveit_commander is None:
                raise RuntimeError("moveit_commander solicitado pero no disponible")
            self._backend = "moveit_commander"
            self._move_group = MoveGroupCommander(self._group_name)
            self._move_group.set_pose_reference_frame(self._base_frame)
            self._configure_move_group_scaling(self._move_group)
            self.get_logger().info("moveit_commander backend activo.")
        elif moveit_commander is not None:
            self._backend = "moveit_commander"
            self._move_group = MoveGroupCommander(self._group_name)
            self._move_group.set_pose_reference_frame(self._base_frame)
            self._configure_move_group_scaling(self._move_group)
            self.get_logger().info("moveit_commander backend activo.")
        elif MoveItPy is not None and PlanningComponent is not None:
            self._backend = "moveit_py"
            self.get_logger().info("MoveItPy backend seleccionado; inicializando...")
        else:
            self.get_logger().error(
                "MoveIt Python no disponible. "
                f"moveit_py: {_MOVEIT_PY_IMPORT_ERROR} "
                f"moveit_commander: {_MOVEIT_COMMANDER_IMPORT_ERROR}"
            )
            raise RuntimeError("MoveIt Python no disponible")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._pose_lock = threading.Lock()
        self._pose_queue: deque[
            tuple[int, str, PoseStamped, bool, bool, int, float]
        ] = deque()
        self._command_seq = 0
        self._active_request_id = 0
        self._active_request_uuid = ""
        self._active_request_started_mono = 0.0
        self._active_exec_timeout_sec = 0.0
        self._active_exec_timeout_deadline_mono = 0.0
        self._last_plan_time = 0.0
        self._first_controller_goal_pending = True
        self._plan_event = threading.Event()
        self._shutdown = False
        self._last_pose_log = 0.0
        self._tf_ready = threading.Event()
        self._pose_subscriptions = []
        self._qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        topics = self._pose_topics or ["/desired_grasp", "/grasp_pose"]
        cartesian_topics = self._cartesian_pose_topics or ["/desired_grasp_cartesian"]
        all_topics = list(dict.fromkeys(list(topics) + list(cartesian_topics)))
        for topic in all_topics:
            is_cartesian = topic in cartesian_topics
            self._pose_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    lambda msg, topic_name=topic, cart=is_cartesian: self._pose_callback(msg, cart, topic_name),
                    self._qos_cmd,
                )
            )
        self.get_logger().info(
            "Suscrito a pose topics: "
            f"{topics} (cartesian: {cartesian_topics}) qos={self._qos_summary(self._qos_cmd)}"
        )
        self._traj_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self._result_pub = self.create_publisher(String, self._result_topic, self._qos_cmd)
        self._heartbeat_pub = self.create_publisher(Bool, self._heartbeat_topic, self._qos_cmd)
        self._heartbeat_seq = 0
        self._sim_wall_rate_samples: deque[float] = deque(maxlen=24)
        self._sim_wall_last_sample: tuple[float, float] | None = None
        self.get_logger().info(
            f"[BRIDGE][PUB_RESULT] configured topic={self._result_topic} qos={self._qos_summary(self._qos_cmd)}"
        )
        self.get_logger().info(
            f"[BRIDGE][HEARTBEAT] topic={self._heartbeat_topic} "
            f"rate_hz={self._heartbeat_rate_hz:.2f} qos={self._qos_summary(self._qos_cmd)}"
        )
        self._cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self._list_controllers_client = self.create_client(
            ListControllers,
            self._controller_manager_service_name("list_controllers"),
        )
        self._joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_cb,
            20,
        )
        if self._backend == "moveit_py":
            self._moveit_py_init_thread = threading.Thread(
                target=self._init_moveit_py,
                daemon=True,
                name="ur5_moveit_py_init",
            )
            self._moveit_py_init_thread.start()
        self.create_timer(0.5, self._poll_tf_ready)
        self.create_timer(max(0.05, 1.0 / max(0.2, self._heartbeat_rate_hz)), self._publish_heartbeat)
        self._worker_thread = threading.Thread(target=self._plan_worker, daemon=True)
        self._worker_thread.start()
        # Prime the action client from the executor thread to avoid runtime races.
        self._fjt_prime_timer = self.create_timer(0.5, self._prime_fjt_action_client)
        self.get_logger().info("UR5 MoveIt bridge listo.")

    def _publish_heartbeat(self) -> None:
        try:
            msg = Bool()
            msg.data = True
            self._heartbeat_pub.publish(msg)
            self._heartbeat_seq += 1
            self._update_sim_wall_rate_estimate()
        except Exception as exc:
            self.get_logger().warning(
                f"[BRIDGE][EXCEPTION] callback=heartbeat_publish type={type(exc).__name__} err={exc}"
            )

    def _update_sim_wall_rate_estimate(self) -> None:
        if not self._use_sim_time:
            return
        try:
            sim_now = float(self.get_clock().now().nanoseconds or 0) / 1_000_000_000.0
        except Exception:
            return
        if sim_now <= 0.0:
            return
        wall_now = time.monotonic()
        prev = self._sim_wall_last_sample
        self._sim_wall_last_sample = (wall_now, sim_now)
        if prev is None:
            return
        prev_wall, prev_sim = prev
        delta_wall = max(0.0, wall_now - prev_wall)
        delta_sim = sim_now - prev_sim
        if delta_wall < 0.2 or delta_sim < 0.0:
            return
        rate = delta_sim / delta_wall if delta_wall > 0.0 else 0.0
        if 0.02 <= rate <= 5.0:
            self._sim_wall_rate_samples.append(float(rate))

    def _estimated_sim_seconds_per_wall_second(self) -> float:
        samples = list(self._sim_wall_rate_samples)
        if not samples:
            return 1.0
        samples.sort()
        mid = len(samples) // 2
        if len(samples) % 2:
            return max(0.05, float(samples[mid]))
        return max(0.05, float(samples[mid - 1] + samples[mid]) / 2.0)

    def _timeout_sim_seconds_per_wall_second(
        self,
        *,
        first_goal: bool = False,
    ) -> tuple[float, str]:
        raw = float(self._estimated_sim_seconds_per_wall_second())
        samples = len(self._sim_wall_rate_samples)
        if not (
            self._use_sim_time
            and not self._moveit_py_use_sim_time
            and self._force_fjt_direct_for_walltime_sim
        ):
            return raw, f"raw={raw:.3f};samples={samples};mixed_time=false"
        min_samples = max(
            1,
            int(round(self._env_float("PANEL_MOVEIT_BRIDGE_SIM_RATE_MIN_SAMPLES", 3.0))),
        )
        fallback = max(
            0.05,
            self._env_float("PANEL_MOVEIT_BRIDGE_SIM_PER_WALL_FALLBACK", 0.45),
        )
        if first_goal and samples < min_samples:
            effective = min(raw, fallback) if raw > 0.0 else fallback
            return (
                float(effective),
                (
                    f"warmup_first_goal raw={raw:.3f};samples={samples};"
                    f"min_samples={min_samples};fallback={fallback:.3f}"
                ),
            )
        return raw, f"raw={raw:.3f};samples={samples}"

    def _fjt_timeout_for_trajectory(
        self,
        traj_sec: float,
        *,
        extra_margin_sec: float,
        minimum_sec: float = 8.0,
        first_goal: bool = False,
        controller_goal_time_sec_override: float | None = None,
    ) -> float:
        controller_goal_time_sec = max(
            0.0,
            float(self._controller_expected_goal_time_sec),
            (
                float(controller_goal_time_sec_override)
                if controller_goal_time_sec_override is not None
                else float(self._controller_goal_time_tolerance_sec)
            ),
        )
        traj_and_goal_sec = float(traj_sec) + controller_goal_time_sec
        timeout_sec = max(
            float(minimum_sec),
            float(self._execute_timeout_sec) + controller_goal_time_sec + float(extra_margin_sec),
            traj_and_goal_sec + float(extra_margin_sec),
        )
        if (
            self._use_sim_time
            and not self._moveit_py_use_sim_time
            and self._force_fjt_direct_for_walltime_sim
            and float(traj_sec) > 0.0
        ):
            sim_rate, sim_rate_detail = self._timeout_sim_seconds_per_wall_second(
                first_goal=first_goal,
            )
            scaled_timeout = (traj_and_goal_sec / max(0.25, float(sim_rate))) + float(extra_margin_sec)
            timeout_sec = max(timeout_sec, scaled_timeout)
            self.get_logger().info(
                "[BRIDGE_EXEC] timeout adjusted for sim/wall skew "
                f"traj_sec={float(traj_sec):.3f} controller_goal_time_sec={controller_goal_time_sec:.3f} "
                f"sim_per_wall={float(sim_rate):.3f} detail={sim_rate_detail} "
                f"timeout_sec={float(timeout_sec):.3f}"
            )
        return float(timeout_sec)

    def _effective_request_timeout_sec(self) -> float:
        timeout_sec = max(2.0, float(self._request_timeout_sec))
        if (
            self._use_sim_time
            and not self._moveit_py_use_sim_time
            and self._force_fjt_direct_for_walltime_sim
        ):
            sim_rate, sim_rate_detail = self._timeout_sim_seconds_per_wall_second()
            controller_goal_time_sec = max(
                0.0,
                float(self._controller_expected_goal_time_sec),
                float(self._controller_goal_time_tolerance_sec),
            )
            retry_cushion_sec = max(
                30.0,
                (2.0 * controller_goal_time_sec) + 10.0,
            )
            scaled_timeout = (timeout_sec / max(0.25, float(sim_rate))) + retry_cushion_sec
            timeout_sec = max(timeout_sec, scaled_timeout)
            self.get_logger().info(
                "[BRIDGE_EXEC] request timeout adjusted for sim/wall skew "
                f"base_timeout_sec={float(self._request_timeout_sec):.3f} "
                f"sim_per_wall={float(sim_rate):.3f} detail={sim_rate_detail} "
                f"retry_cushion_sec={float(retry_cushion_sec):.3f} "
                f"timeout_sec={float(timeout_sec):.3f}"
            )
        return float(timeout_sec)

    @staticmethod
    def _env_float(name: str, default: float) -> float:
        return bridge_env_float(name, default)

    def _topic_endpoint_counts(self, topic: str) -> tuple[int, int]:
        pubs = 0
        subs = 0
        try:
            pubs = len(self.get_publishers_info_by_topic(topic))
        except Exception:
            pubs = 0
        try:
            subs = len(self.get_subscriptions_info_by_topic(topic))
        except Exception:
            subs = 0
        return pubs, subs

    def _exec_diagnostics(self) -> dict[str, Any]:
        now_ns = int(self.get_clock().now().nanoseconds)
        js_stamp_ns = int(self._joint_state_stamp_ns or 0)
        js_age_sec = None
        if js_stamp_ns > 0:
            js_age_sec = (now_ns - js_stamp_ns) / 1_000_000_000.0
        action_names = self._available_action_names()
        expected_action = str(self._controller_action_name or "")
        candidates = self._controller_action_candidates(action_names)
        has_expected_action = False
        matched_action = ""
        client = self._ensure_fjt_action_client()
        if client is not None:
            try:
                if client.wait_for_server(timeout_sec=0.05):
                    has_expected_action = True
                    matched_action = self._normalize_action_name(expected_action)
            except Exception:
                pass
        traj_topic = f"/{self._controller_name}/joint_trajectory".replace("//", "/")
        traj_pubs, traj_subs = self._topic_endpoint_counts(traj_topic)
        active_controllers = self._active_controllers(timeout_sec=0.2)
        reason = "execute_returned_false"
        if not has_expected_action:
            reason = "no_follow_joint_trajectory_action"
        elif js_age_sec is not None and js_age_sec > 1.0:
            reason = "joint_state_stale_or_clock_mismatch"
        return {
            "reason": reason,
            "backend": str(self._backend or "unknown"),
            "use_sim_time": bool(self._use_sim_time),
            "ee_link": str(self._ee_frame or ""),
            "controller": str(self._controller_name or ""),
            "controller_action": expected_action,
            "controller_action_matched": matched_action,
            "controller_action_candidates": candidates,
            "controller_manager": self._controller_manager_name,
            "active_controllers": active_controllers,
            "controller_action_available": bool(has_expected_action),
            "joint_state_stamp_ns": js_stamp_ns,
            "joint_state_age_sec": js_age_sec,
            "joint_state_names": int(self._joint_state_names),
            "joint_state_last_recv_mono_age_sec": (
                (time.monotonic() - self._joint_state_recv_mono)
                if self._joint_state_recv_mono > 0.0
                else None
            ),
            "trajectory_topic": traj_topic,
            "trajectory_topic_pubs": int(traj_pubs),
            "trajectory_topic_subs": int(traj_subs),
        }

    @staticmethod
    def _diag_to_message(diag: dict[str, Any]) -> str:
        return diag_to_message(diag)

    @staticmethod
    def _describe_execute_result(result: Any) -> dict[str, Any]:
        return describe_execute_result(result)

    @staticmethod
    def _plan_success_code(success: Any) -> int | None:
        return plan_success_code(success)

    @classmethod
    def _plan_success_ok(cls, success: Any) -> bool:
        return plan_success_ok(success)

    @staticmethod
    def _plan_error_code_val(plan) -> int | None:
        return plan_error_code_val(plan)

    @staticmethod
    def _result_meta_to_message(meta: dict[str, Any]) -> str:
        return result_meta_to_message(meta)

    @staticmethod
    def _goal_status_text(status: int) -> str:
        return goal_status_text(status)

    @staticmethod
    def _wait_future_done(future, timeout_sec: float) -> bool:
        return wait_future_done(future, timeout_sec)

    def _execute_moveit_py_with_timeout(self, trajectory) -> tuple[bool, Any, str]:
        if self._moveit_py is None:
            return False, None, "moveit_py_not_ready"
        done = threading.Event()
        holder: dict[str, Any] = {}
        error: dict[str, Exception] = {}

        def _runner() -> None:
            try:
                try:
                    holder["result"] = self._moveit_py.execute(trajectory, controllers=[])
                except TypeError:
                    holder["result"] = self._moveit_py.execute(trajectory)
            except Exception as exc:
                error["exc"] = exc
            finally:
                done.set()

        thread = threading.Thread(target=_runner, daemon=True, name="moveit_py_execute")
        thread.start()
        if not done.wait(timeout=max(1.0, float(self._execute_timeout_sec))):
            detail = (
                f"execute_timeout timeout_sec={self._execute_timeout_sec:.1f} "
                f"controller={self._controller_name} action={self._controller_action_name}"
            )
            self._log_bridge_status(
                f"[BRIDGE_STATUS] exec_fail backend=moveit_py reason=execute_timeout detail={detail}",
                level="warn",
            )
            self.get_logger().warning(
                "[BRIDGE_EXEC] MoveItPy execute timeout; devolviendo fallo para evitar espera infinita. "
                f"{detail}"
            )
            return False, None, detail
        if "exc" in error:
            raise error["exc"]
        return True, holder.get("result"), ""

    @staticmethod
    def _qos_summary(qos: QoSProfile) -> str:
        reliability = "RELIABLE" if qos.reliability == ReliabilityPolicy.RELIABLE else "BEST_EFFORT"
        durability = "VOLATILE" if qos.durability == DurabilityPolicy.VOLATILE else "TRANSIENT_LOCAL"
        history = "KEEP_LAST" if qos.history == HistoryPolicy.KEEP_LAST else "KEEP_ALL"
        return f"{reliability}/{durability}/{history}@depth={qos.depth}"

    @staticmethod
    def _parse_request_meta(frame_raw: str) -> tuple[str, int | None, str, dict[str, Any]]:
        return parse_request_meta(frame_raw)

    def _pose_callback(self, msg: PoseStamped, cartesian: bool = False, topic_name: str = "") -> None:
        try:
            now = time.monotonic()
            frame_raw = str(getattr(msg.header, "frame_id", "") or "")
            frame_clean, req_from_msg, req_uuid, req_meta = self._parse_request_meta(frame_raw)
            if not frame_clean:
                frame_clean = self._base_frame
            msg.header.frame_id = frame_clean
            req_stamp_ns = _compute_request_stamp_ns(msg.header.stamp)
            if _is_invalid_business_frame(frame_clean, self._base_frame):
                self._command_seq += 1
                rejected_request_id = int(self._command_seq)
                rx_ts_us = int(time.time() * 1_000_000)
                self.get_logger().warning(
                    _fmt_rx_log(
                        ts_us=rx_ts_us,
                        request_id=req_from_msg if req_from_msg is not None else rejected_request_id,
                        request_uuid=req_uuid,
                        frame_id=frame_clean,
                        pose=msg.pose.position,
                        accepted=False,
                        reason="invalid_business_frame",
                    )
                )
                self._publish_result(
                    request_id=rejected_request_id,
                    request_uuid=req_uuid,
                    target=msg,
                    request_stamp_ns=req_stamp_ns,
                    cartesian=cartesian,
                    success=False,
                    plan_ok=False,
                    exec_ok=False,
                    message="invalid_business_frame:base",
                )
                self.get_logger().error(
                    "[BRIDGE][P0] rejected_invalid_business_frame "
                    f"topic={topic_name or 'n/a'} frame_raw={frame_raw or 'n/a'} "
                    f"required={self._base_frame}"
                )
                return
            ee_target_tol_m = None
            try:
                if "tol_m" in req_meta:
                    ee_target_tol_m = float(req_meta["tol_m"])
            except Exception:
                ee_target_tol_m = None
            phase_label = str(req_meta.get("phase_label") or "").strip() or None
            if self._require_request_id and req_from_msg is None:
                self._command_seq += 1
                rejected_request_id = int(self._command_seq)
                rx_ts_us = int(time.time() * 1_000_000)
                self.get_logger().warning(
                    _fmt_rx_log(
                        ts_us=rx_ts_us,
                        request_id=rejected_request_id,
                        request_uuid=req_uuid,
                        frame_id=frame_clean,
                        pose=msg.pose.position,
                        accepted=False,
                        reason="missing_request_id",
                    )
                )
                self._publish_result(
                    request_id=rejected_request_id,
                    request_uuid=req_uuid,
                    target=msg,
                    request_stamp_ns=req_stamp_ns,
                    cartesian=cartesian,
                    success=False,
                    plan_ok=False,
                    exec_ok=False,
                    message="missing_request_id",
                )
                self.get_logger().warning(
                    "[BRIDGE][RECV] rejected_missing_request_id "
                    f"topic={topic_name or 'n/a'} frame_raw={frame_raw or 'n/a'} "
                    f"assigned_request_id={rejected_request_id} pos=({msg.pose.position.x:.3f},"
                    f"{msg.pose.position.y:.3f},{msg.pose.position.z:.3f})"
                )
                return
            with self._pose_lock:
                if req_from_msg is not None:
                    request_id = int(req_from_msg)
                    self._command_seq = max(int(self._command_seq), int(request_id))
                else:
                    self._command_seq += 1
                    request_id = int(self._command_seq)
                active_request_id = int(getattr(self, "_active_request_id", 0) or 0)
                active_request_uuid = str(getattr(self, "_active_request_uuid", "") or "")
                active_started = float(getattr(self, "_active_request_started_mono", 0.0) or 0.0)
                if req_from_msg is not None and active_request_id > 0 and active_request_id != request_id:
                    busy_age = max(0.0, time.monotonic() - active_started) if active_started > 0.0 else 0.0
                    busy_message = _fmt_busy_message(
                        active_request_id=active_request_id,
                        active_request_uuid=active_request_uuid,
                        active_age_sec=busy_age,
                    )
                    self.get_logger().warning(
                        "[BRIDGE][QUEUE] reject_overlapping_request "
                        f"request_id={request_id} active_request_id={active_request_id} "
                        f"active_request_uuid={active_request_uuid or 'n/a'} active_age={busy_age:.2f}s"
                    )
                    rx_ts_us = int(time.time() * 1_000_000)
                    self.get_logger().warning(
                        _fmt_rx_log(
                            ts_us=rx_ts_us,
                            request_id=request_id,
                            request_uuid=req_uuid,
                            frame_id=frame_clean,
                            pose=msg.pose.position,
                            accepted=False,
                            reason="bridge_busy",
                        )
                    )
                    self._publish_result(
                        request_id=request_id,
                        request_uuid=req_uuid,
                        target=msg,
                        request_stamp_ns=req_stamp_ns,
                        cartesian=cartesian,
                        success=False,
                        plan_ok=False,
                        exec_ok=False,
                        message=busy_message,
                    )
                    return
                dropped_pending = 0
                if req_from_msg is not None and self._drop_pending_on_tagged_request:
                    dropped_pending = len(self._pose_queue)
                    if dropped_pending > 0:
                        self._pose_queue.clear()
                self._pose_queue.append(
                    (
                        request_id,
                        str(req_uuid or ""),
                        msg,
                        cartesian,
                        msg.header.frame_id != self._base_frame,
                        ee_target_tol_m,
                        phase_label,
                        req_stamp_ns,
                        now,
                    )
                )
            rx_ts_us = int(time.time() * 1_000_000)
            self.get_logger().info(
                _fmt_rx_log(
                    ts_us=rx_ts_us,
                    request_id=request_id,
                    request_uuid=req_uuid,
                    frame_id=msg.header.frame_id,
                    pose=msg.pose.position,
                    accepted=True,
                )
            )
            self.get_logger().info(
                _fmt_pick_request_log(
                    ts_us=rx_ts_us,
                    request_id=request_id,
                    request_uuid=req_uuid,
                    frame_id=msg.header.frame_id,
                    pose=msg.pose.position,
                    cartesian=cartesian,
                    phase_label=phase_label,
                    ee_target_tol_m=ee_target_tol_m,
                    accepted=True,
                )
            )
            self._plan_event.set()
            pos = msg.pose.position
            if dropped_pending > 0:
                self.get_logger().warning(
                    "[BRIDGE][QUEUE] dropped_pending "
                    f"count={dropped_pending} reason=tagged_request request_id={request_id}"
                )
            self.get_logger().info(
                _fmt_recv_log(
                    label="CARTESIAN" if cartesian else "POSE",
                    topic_name=topic_name,
                    request_id=request_id,
                    frame_id=msg.header.frame_id,
                    pose=pos,
                    stamp_sec=msg.header.stamp.sec,
                    stamp_nanosec=msg.header.stamp.nanosec,
                    frame_raw=frame_raw,
                    request_uuid=req_uuid,
                    ee_target_tol_m=ee_target_tol_m,
                    phase_label=phase_label,
                )
            )
            if now - self._last_pose_log > 2.0:
                needs_tf = msg.header.frame_id != self._base_frame
                self.get_logger().info(
                    f"Pose recibida: id={request_id} frame={msg.header.frame_id} "
                    f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
                    f"base_frame={self._base_frame} needs_tf={needs_tf}"
                )
                self._last_pose_log = now
            if cartesian:
                pos = msg.pose.position
                self.get_logger().info(
                    f"Pose cartesiana recibida: frame={msg.header.frame_id} "
                    f"pos=({pos.x:.3f},{pos.y:.3f},{pos.z:.3f})"
                )
        except Exception as exc:
            self.get_logger().error(
                "[BRIDGE][EXCEPTION] callback=pose_callback "
                f"type={type(exc).__name__} err={exc}\n{traceback.format_exc()}"
            )

    def _ensure_base_frame(self, msg: PoseStamped) -> PoseStamped | None:
        if msg.header.frame_id == self._base_frame:
            return msg

        try:
            now = Time()
            if not self.tf_buffer.can_transform(
                self._base_frame,
                msg.header.frame_id,
                now,
                timeout=Duration(seconds=0.5),
            ):
                raise LookupException(
                    f"TF {msg.header.frame_id}->{self._base_frame} no disponible"
                )
            transform = self.tf_buffer.lookup_transform(
                self._base_frame, msg.header.frame_id, now
            )
            converted = do_transform_pose_stamped(msg, transform)
            pos = converted.pose.position
            tr = transform.transform.translation
            self.get_logger().info(
                "[BRIDGE][TF] "
                f"source={msg.header.frame_id} target={self._base_frame} "
                f"tf_t=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f}) "
                f"out=({pos.x:.3f},{pos.y:.3f},{pos.z:.3f})"
            )
            return converted
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warning(
                f"No se pudo transformar pose a {self._base_frame}: {exc}"
            )
            return None

    def _poll_tf_ready(self) -> None:
        if self._tf_ready.is_set():
            return
        try:
            now = Time()
            if self.tf_buffer.can_transform(
                self._base_frame, self._ee_frame, now, timeout=Duration(seconds=0.1)
            ):
                self._tf_ready.set()
                self.get_logger().info(
                    f"TF listo: {self._base_frame} <-> {self._ee_frame} disponible."
                )
        except Exception:
            return

    def _dispatch_plan_request(
        self,
        target: PoseStamped,
        cartesian: bool,
        request_uuid: str = "",
        ee_target_tol_m: float | None = None,
        phase_label: str | None = None,
    ) -> tuple[bool, str, bool, bool]:
        if cartesian:
            return self._plan_cartesian(target)
        if self._backend == "moveit_py":
            return self._plan_with_moveit_py(
                target,
                request_uuid=request_uuid,
                ee_target_tol_m=ee_target_tol_m,
                phase_label=phase_label,
            )
        return self._plan_with_moveit_commander(target)

    def _dispatch_plan_request_with_timeout(
        self,
        *,
        request_id: int,
        request_uuid: str,
        target: PoseStamped,
        cartesian: bool,
        ee_target_tol_m: float | None = None,
        phase_label: str | None = None,
    ) -> tuple[bool, str, bool, bool]:
        done = threading.Event()
        holder: dict[str, tuple[bool, str, bool, bool]] = {}
        error: dict[str, Exception] = {}

        def _runner() -> None:
            try:
                holder["result"] = self._dispatch_plan_request(
                    target,
                    cartesian,
                    request_uuid=request_uuid,
                    ee_target_tol_m=ee_target_tol_m,
                    phase_label=phase_label,
                )
            except Exception as exc:
                error["exc"] = exc
            finally:
                done.set()

        thread = threading.Thread(target=_runner, daemon=True, name=f"plan_request_{request_id}")
        thread.start()
        timeout_sec = self._effective_request_timeout_sec()
        phase_upper = str(phase_label or "").strip().upper()
        if phase_upper == "APPROACH":
            approach_max_total_timeout_sec = max(
                20.0,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_MAX_TOTAL_TIMEOUT_SEC",
                    600.0,  # FIX: aumentado de 120 a 600s para sim con RTF bajo
                ),
            )
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
            approach_request_cushion_sec = max(
                10.0,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_REQUEST_TIMEOUT_CUSHION_SEC",
                    20.0,
                ),
            )
            approach_budget_timeout_sec = (
                float(approach_max_total_timeout_sec)
                * float(approach_replan_max_attempts + 1)
                + float(approach_request_cushion_sec)
            )
            if float(timeout_sec) < float(approach_budget_timeout_sec):
                self.get_logger().info(
                    "[BRIDGE_EXEC] request timeout elevated for APPROACH "
                    f"from={float(timeout_sec):.1f}s to={float(approach_budget_timeout_sec):.1f}s "
                    f"attempts={int(approach_replan_max_attempts) + 1} "
                    f"per_attempt_cap={float(approach_max_total_timeout_sec):.1f}s"
                )
                timeout_sec = float(approach_budget_timeout_sec)
        start_wall_us = int(time.time() * 1_000_000)
        self.get_logger().info(
            f"[BRIDGE][DISPATCH_START] request_id={request_id} "
            f"request_uuid={request_uuid or 'n/a'} timeout_sec={timeout_sec:.1f} "
            f"start_wall_us={start_wall_us} cartesian={cartesian}"
        )
        start_mono = time.monotonic()
        deadline_mono = start_mono + float(timeout_sec)
        while True:
            remaining = max(0.1, deadline_mono - time.monotonic())
            if done.wait(timeout=min(0.5, remaining)):
                break
            now_mono = time.monotonic()
            if now_mono < deadline_mono:
                continue
            with self._pose_lock:
                active_request_id = int(getattr(self, "_active_request_id", 0) or 0)
                active_exec_deadline_mono = float(
                    getattr(self, "_active_exec_timeout_deadline_mono", 0.0) or 0.0
                )
                active_exec_timeout_sec = float(
                    getattr(self, "_active_exec_timeout_sec", 0.0) or 0.0
                )
            if active_request_id == int(request_id) and active_exec_deadline_mono > (deadline_mono + 0.5):
                old_deadline = deadline_mono
                deadline_mono = active_exec_deadline_mono + 2.0
                old_timeout_sec = max(0.0, old_deadline - start_mono)
                new_timeout_sec = max(0.0, deadline_mono - start_mono)
                self.get_logger().info(
                    "[BRIDGE] extending request wait to match active controller execution "
                    f"request_id={request_id} old_timeout_sec={old_timeout_sec:.1f} "
                    f"new_timeout_sec={new_timeout_sec:.1f} exec_timeout_sec={active_exec_timeout_sec:.1f}"
                )
                continue
            detail = f"request_id={request_id} timeout_sec={timeout_sec:.1f} cartesian={cartesian}"
            self._log_bridge_status(
                f"[BRIDGE_STATUS] exec_fail backend={self._backend} reason=execute_timeout detail={detail}",
                level="warn",
            )
            self.get_logger().warning(
                "[BRIDGE] request timeout; publicando fallo para evitar espera infinita. "
                f"{detail}"
            )
            return False, f"execute_timeout:{detail}", False, False
        if "exc" in error:
            raise error["exc"]
        end_wall_us = int(time.time() * 1_000_000)
        result = holder.get("result", (False, "empty_result", False, False))
        elapsed_us = max(0, end_wall_us - start_wall_us)
        self.get_logger().info(
            f"[BRIDGE][DISPATCH_END] request_id={request_id} "
            f"request_uuid={request_uuid or 'n/a'} "
            f"end_wall_us={end_wall_us} elapsed_us={elapsed_us} "
            f"success={result[0]} plan_ok={result[2]} exec_ok={result[3]}"
        )
        return result

    def _plan_worker(self) -> None:
        while rclpy.ok() and not self._shutdown:
            try:
                if not self._plan_event.wait(timeout=0.2):
                    continue
                while rclpy.ok() and not self._shutdown:
                    with self._pose_lock:
                        if not self._pose_queue:
                            self._plan_event.clear()
                            break
                        (
                            request_id,
                            request_uuid,
                            target,
                            cartesian,
                            needs_tf,
                            ee_target_tol_m,
                            phase_label,
                            request_stamp_ns,
                            queued_mono,
                        ) = self._pose_queue.popleft()
                    is_stale, queued_age = _is_stale_request(
                        float(queued_mono),
                        time.monotonic(),
                        float(self._stale_request_ttl_sec),
                    )
                    if is_stale:
                        self._publish_result(
                            request_id=request_id,
                            request_uuid=request_uuid,
                            target=target,
                            request_stamp_ns=request_stamp_ns,
                            cartesian=cartesian,
                            success=False,
                            plan_ok=False,
                            exec_ok=False,
                            message=f"stale_request_dropped:age={queued_age:.2f}s",
                        )
                        self.get_logger().warning(
                            "[BRIDGE][QUEUE] stale_request_dropped "
                            f"request_id={request_id} request_uuid={request_uuid or 'n/a'} "
                            f"age={queued_age:.2f}s"
                        )
                        continue
                    with self._pose_lock:
                        self._active_request_id = int(request_id)
                        self._active_request_uuid = str(request_uuid or "")
                        self._active_request_started_mono = time.monotonic()
                    if needs_tf:
                        resolved = self._ensure_base_frame(target)
                        if resolved is None:
                            self._publish_result(
                                request_id=request_id,
                                request_uuid=request_uuid,
                                target=target,
                                request_stamp_ns=request_stamp_ns,
                                cartesian=cartesian,
                                success=False,
                                plan_ok=False,
                                exec_ok=False,
                                message=f"tf_unavailable:{target.header.frame_id}->{self._base_frame}",
                            )
                            continue
                        target = resolved
                    js_ok, js_reason = self._wait_for_valid_joint_state(
                        timeout_sec=self._joint_state_valid_timeout_sec
                    )
                    if not js_ok:
                        self._publish_result(
                            request_id=request_id,
                            request_uuid=request_uuid,
                            target=target,
                            request_stamp_ns=request_stamp_ns,
                            cartesian=cartesian,
                            success=False,
                            plan_ok=False,
                            exec_ok=False,
                            message=f"joint_state_not_ready:{js_reason}",
                        )
                        self._log_bridge_status(
                            "[BRIDGE_STATUS] plan_fail backend="
                            f"{self._backend} reason=joint_state_not_ready detail={js_reason}",
                            level="warn",
                        )
                        continue
                    settle_timeout_sec, settle_stable_sec, settle_tol_rad = _compute_settle_params(
                        self._env_float,
                        float(self._joint_state_valid_timeout_sec),
                    )
                    settled_ok, settled_reason = self._wait_for_joint_state_settled(
                        timeout_sec=settle_timeout_sec,
                        stable_sec=settle_stable_sec,
                        tol_rad=settle_tol_rad,
                    )
                    if settled_ok:
                        self.get_logger().info(
                            "[BRIDGE_EXEC] joint_state_settled "
                            f"request_id={request_id} detail={settled_reason} "
                            f"cfg(timeout={settle_timeout_sec:.2f}s stable={settle_stable_sec:.2f}s tol={settle_tol_rad:.4f})"
                        )
                    else:
                        self.get_logger().warning(
                            "[BRIDGE_EXEC] joint_state_settle_timeout "
                            f"request_id={request_id} detail={settled_reason} "
                            f"cfg(timeout={settle_timeout_sec:.2f}s stable={settle_stable_sec:.2f}s tol={settle_tol_rad:.4f})"
                        )
                    elapsed = time.monotonic() - self._last_plan_time
                    if self._min_plan_interval > 0.0 and elapsed < self._min_plan_interval:
                        time.sleep(self._min_plan_interval - elapsed)
                    self.get_logger().info(
                        _fmt_exec_start_log(
                            request_id=request_id,
                            cartesian=cartesian,
                            frame_id=target.header.frame_id,
                            ee_frame=self._ee_frame,
                            base_frame=self._base_frame,
                            pose=target.pose.position,
                        )
                    )
                    self.get_logger().info(
                        f"Planificando id={request_id} frame={target.header.frame_id} "
                        f"(cartesian={cartesian}, ee_link={self._ee_frame})"
                    )
                    self.get_logger().info(
                        _fmt_target_log(
                            request_id=request_id,
                            request_uuid=request_uuid,
                            phase_label=phase_label,
                            frame_id=target.header.frame_id,
                            pose=target.pose.position,
                            cartesian=cartesian,
                            ee_frame=self._ee_frame,
                        )
                    )
                    success = False
                    plan_ok = False
                    exec_ok = False
                    message = "bridge_internal_error"
                    try:
                        success, message, plan_ok, exec_ok = self._dispatch_plan_request_with_timeout(
                            request_id=request_id,
                            request_uuid=request_uuid,
                            target=target,
                            cartesian=cartesian,
                            ee_target_tol_m=ee_target_tol_m,
                            phase_label=phase_label,
                        )
                    except Exception as exc:
                        success = False
                        plan_ok = False
                        exec_ok = False
                        message = f"bridge_internal_exception:{type(exc).__name__}:{exc}"
                        self.get_logger().error(
                            f"[BRIDGE_STATUS] exec_fail backend={self._backend} reason=internal_exception "
                            f"request_id={request_id} detail={message}"
                        )
                        self.get_logger().error(
                            "[BRIDGE][EXCEPTION] callback=plan_worker "
                            f"request_id={request_id} type={type(exc).__name__} err={exc}\n"
                            f"{traceback.format_exc()}"
                        )
                    finally:
                        try:
                            pub_ts_us = int(time.time() * 1_000_000)
                            self.get_logger().info(
                                f"[BRIDGE][RESULT_DIAG] request_id={request_id} "
                                f"request_uuid={request_uuid or 'n/a'} "
                                f"about_to_publish_result pub_ts_us={pub_ts_us} "
                                f"success={success} plan_ok={plan_ok} exec_ok={exec_ok}"
                            )
                            self._publish_result(
                                request_id=request_id,
                                request_uuid=request_uuid,
                                target=target,
                                request_stamp_ns=request_stamp_ns,
                                cartesian=cartesian,
                                success=success,
                                plan_ok=plan_ok,
                                exec_ok=exec_ok,
                                message=message,
                            )
                            pub_ts_us_after = int(time.time() * 1_000_000)
                            self.get_logger().info(
                                f"[BRIDGE][RESULT_PUBLISHED] request_id={request_id} "
                                f"request_uuid={request_uuid or 'n/a'} "
                                f"pub_elapsed_us={pub_ts_us_after - pub_ts_us} pub_ts_us={pub_ts_us_after}"
                            )
                        except Exception as pub_exc:
                            self.get_logger().error(
                                "[BRIDGE][PUB_RESULT] failed "
                                f"request_id={request_id} topic={self._result_topic} err={pub_exc}"
                            )
                        finally:
                            with self._pose_lock:
                                if int(getattr(self, "_active_request_id", 0) or 0) == int(request_id):
                                    self._active_request_id = 0
                                    self._active_request_uuid = ""
                                    self._active_request_started_mono = 0.0
                                    self._active_exec_timeout_sec = 0.0
                                    self._active_exec_timeout_deadline_mono = 0.0
                        self._last_plan_time = time.monotonic()
            except Exception as exc:
                self.get_logger().error(
                    "[BRIDGE][EXCEPTION] callback=plan_worker_loop "
                    f"type={type(exc).__name__} err={exc}\n{traceback.format_exc()}"
                )
                time.sleep(0.1)

    def _publish_result(
        self,
        *,
        request_id: int,
        request_uuid: str = "",
        target: PoseStamped,
        request_stamp_ns: int = 0,
        cartesian: bool,
        success: bool,
        plan_ok: bool,
        exec_ok: bool,
        message: str,
    ) -> None:
        stamp = getattr(target.header, "stamp", None)
        stamp_sec = int(getattr(stamp, "sec", 0) or 0)
        stamp_nsec = int(getattr(stamp, "nanosec", 0) or 0)
        target_stamp_ns = (stamp_sec * 1_000_000_000) + stamp_nsec
        if int(request_stamp_ns) > 0:
            target_stamp_ns = int(request_stamp_ns)
        payload = {
            "request_id": int(request_id),
            "request_uuid": str(request_uuid or ""),
            "success": bool(success),
            "plan_ok": bool(plan_ok),
            "exec_ok": bool(exec_ok),
            "message": str(message),
            "backend": str(self._backend or "unknown"),
            "cartesian": bool(cartesian),
            "frame_id": str(target.header.frame_id or ""),
            "ee_link": str(self._ee_frame or ""),
            "target_stamp_ns": int(target_stamp_ns),
        }
        out = String()
        out.data = json.dumps(payload, ensure_ascii=True)
        self._result_pub.publish(out)
        self.get_logger().info(
            "[BRIDGE][TX] "
            f"request_id={request_id} request_uuid={request_uuid or 'n/a'} "
            f"stamp_ns={target_stamp_ns} success={str(bool(success)).lower()}"
        )
        self.get_logger().info(
            "[BRIDGE][PUB_RESULT] "
            f"label={'CARTESIAN' if cartesian else 'POSE'} request_id={request_id} "
            f"request_uuid={request_uuid or 'n/a'} "
            f"success={str(bool(success)).lower()} plan_ok={str(bool(plan_ok)).lower()} "
            f"exec_ok={str(bool(exec_ok)).lower()} msg={message or 'n/a'} topic={self._result_topic}"
        )
        self.get_logger().info(
            "[PICK][MOVEIT][EXEC_RESULT] "
            f"request_id={request_id} request_uuid={request_uuid or 'n/a'} "
            f"frame={str(target.header.frame_id or 'n/a')} ee_frame={self._ee_frame or 'n/a'} "
            f"success={str(bool(success)).lower()} plan_ok={str(bool(plan_ok)).lower()} "
            f"exec_ok={str(bool(exec_ok)).lower()} msg={message or 'n/a'}"
        )
        self.get_logger().info(
            "[BRIDGE_RESULT] "
            f"request_id={request_id} success={str(bool(success)).lower()} "
            f"request_uuid={request_uuid or 'n/a'} "
            f"plan_ok={str(bool(plan_ok)).lower()} exec_ok={str(bool(exec_ok)).lower()} "
            f"message={message or 'n/a'} topic={self._result_topic}"
        )
        self.get_logger().info(
            "[MOVEIT_BRIDGE][TX_RESULT] "
            f"ts_us={int(time.time() * 1_000_000)} req_id={request_id} req_uuid={request_uuid or 'n/a'} "
            f"success={str(bool(success)).lower()} reason={message or 'n/a'} "
            f"ee={self._ee_frame or 'n/a'} frame={str(target.header.frame_id or 'n/a')} "
            f"finalized={str(bool(success) or (not bool(plan_ok) or not bool(exec_ok))).lower()}"
        )

    @staticmethod
    def _strip_qos_overrides(data: object) -> object:
        if isinstance(data, dict):
            data = dict(data)
            if "qos_overrides" in data:
                data.pop("qos_overrides", None)
            for key in list(data.keys()):
                if "qos_overrides" in str(key):
                    data.pop(key, None)
            for key, value in list(data.items()):
                data[key] = UR5MoveItBridge._strip_qos_overrides(value)
        elif isinstance(data, list):
            data = [UR5MoveItBridge._strip_qos_overrides(item) for item in data]
        return data

    @staticmethod
    def _extract_trajectory(plan) -> RobotTrajectory | None:
        if plan is None:
            return None
        if isinstance(plan, tuple):
            if len(plan) >= 2:
                outcome = plan[0]
                trajectory = plan[1]
                if outcome:
                    return (
                        trajectory if isinstance(trajectory, RobotTrajectory) else None
                    )
            return None
        if isinstance(plan, RobotTrajectory):
            return plan
        return None

    def _publish_planned_joint_trajectory(self, trajectory) -> bool:
        if self._traj_pub is None:
            self.get_logger().warning("Fallback execution cancelada: traj_pub no disponible.")
            return False
        try:
            jt = self._extract_joint_trajectory_msg(trajectory)
            if jt is None:
                self.get_logger().warning(
                    "Fallback execution fallida: no se pudo extraer JointTrajectory del plan."
                )
                return False
            jt = self._prepare_joint_trajectory_for_controller(jt)
            points = getattr(jt, "points", None)
            if not points:
                self.get_logger().warning(
                    "Fallback execution fallida: trayectoria vacía."
                )
                return False
            self._traj_pub.publish(jt)
            self.get_logger().info(
                "Fallback execution: JointTrajectory publicada al controlador."
            )
            return True
        except Exception as exc:
            self.get_logger().warning(
                f"Fallback execution fallida (topic publish): {exc}"
            )
            return False

    def _load_controller_contract(self, controllers_path: Path) -> None:
        self._controller_config_source = str(controllers_path)
        try:
            import yaml  # type: ignore
        except Exception:
            self.get_logger().warning(
                f"Controller contract yaml no disponible; usando defaults ({controllers_path})"
            )
            return
        try:
            data = yaml.safe_load(controllers_path.read_text(encoding="utf-8")) or {}
            scm = data.get("moveit_simple_controller_manager", {}) or {}
            names = scm.get("controller_names", []) or []
            controller = str(names[0] if names else "joint_trajectory_controller")
            cfg = scm.get(controller, {}) or {}
            action_ns = str(cfg.get("action_ns", "follow_joint_trajectory"))
            self._controller_name = controller
            self._controller_action_ns = action_ns
            self._controller_action_name = (
                f"/{self._controller_name}/{self._controller_action_ns}".replace("//", "/")
            )
            # Action clients cache depends on action name candidates.
            self._action_clients.clear()
            self._destroy_fjt_action_client()
            self.get_logger().info(
                "Controller contract: "
                f"controller={self._controller_name} action={self._controller_action_name} "
                f"source={controllers_path}"
            )
        except Exception as exc:
            self.get_logger().warning(
                f"No se pudo leer moveit_controllers.yaml ({controllers_path}): {exc}"
            )

    def shutdown(self) -> None:
        self._shutdown = True
        self._plan_event.set()
        # FASE 8: Join worker thread to avoid zombie/orphan threads.
        if hasattr(self, "_worker_thread") and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=3.0)
        if (
            self._moveit_py_init_thread is not None
            and self._moveit_py_init_thread.is_alive()
        ):
            self._moveit_py_init_thread.join(timeout=3.0)
        try:
            if self._fjt_prime_timer is not None:
                self._fjt_prime_timer.cancel()
        except Exception:
            pass
        self._destroy_fjt_action_client()
        self._release_moveit_backend()

    def _release_moveit_backend(self) -> None:
        """Release MoveIt objects while the ROS context is still alive."""
        try:
            if self._move_group is not None:
                try:
                    self._move_group.stop()
                except Exception:
                    pass
                try:
                    self._move_group.clear_pose_targets()
                except Exception:
                    pass
        except Exception:
            pass
        self._planning_component = None
        self._move_group = None
        self._cartesian_group = None
        self._moveit_py_ready = False
        self._moveit_py_init_error = None
        if self._moveit_py is not None:
            self.get_logger().info(
                "[BRIDGE] liberando backend MoveItPy antes de destroy_node/shutdown."
            )
            try:
                self._moveit_py.shutdown()
            except Exception as exc:
                self.get_logger().warning(
                    "[BRIDGE] MoveItPy shutdown() fallo: "
                    f"{type(exc).__name__}: {exc}"
                )
        self._moveit_py = None
        try:
            gc.collect()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Lifecycle transitions (F13 — observable, sin re-creación de recursos)
    # ------------------------------------------------------------------

    def on_configure(self, _state) -> TransitionCallbackReturn:
        self.get_logger().info("[LIFECYCLE] UR5MoveItBridge configured")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state) -> TransitionCallbackReturn:
        self.get_logger().info("[LIFECYCLE] UR5MoveItBridge activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, _state) -> TransitionCallbackReturn:
        self.get_logger().info("[LIFECYCLE] UR5MoveItBridge deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state) -> TransitionCallbackReturn:
        self.get_logger().info("[LIFECYCLE] UR5MoveItBridge cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state) -> TransitionCallbackReturn:
        return self.on_cleanup(_state)


def main(args=None) -> None:
    node: UR5MoveItBridge | None = None
    fast_exit_on_sigint = False
    rclpy.init(args=args)
    if MoveItPy is None and moveit_commander is None:
        raise SystemExit("MoveIt Python no disponible. Instala ros-jazzy-moveit-py.")
    if moveit_commander is not None:
        moveit_commander.roscpp_initialize(sys.argv)
    node = UR5MoveItBridge()
    # F13 lifecycle: auto-activate por defecto preserva backward-compat.
    if bool(node.get_parameter("auto_activate").value):
        try:
            node.trigger_configure()
            node.trigger_activate()
        except Exception as exc:
            node.get_logger().error(f"[LIFECYCLE] auto_activate failed: {exc}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[BRIDGE] detenido por usuario.")
        fast_exit_on_sigint = bool(getattr(node, "_backend", "") == "moveit_py")
        if fast_exit_on_sigint:
            node.get_logger().warning(
                "[BRIDGE] fast-exit en SIGINT para evitar segfault conocido "
                "de MoveItPy durante teardown."
            )
    except Exception as exc:
        # External shutdown paths can raise executor/context exceptions; keep
        # process exit clean while leaving evidence in log.
        node.get_logger().warning(
            "[BRIDGE][EXCEPTION] main_loop "
            f"type={type(exc).__name__} err={exc}\n{traceback.format_exc()}"
        )
    finally:
        if fast_exit_on_sigint:
            try:
                sys.stdout.flush()
                sys.stderr.flush()
            except Exception:
                pass
            os._exit(0)
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if moveit_commander is not None:
            moveit_commander.roscpp_shutdown()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
