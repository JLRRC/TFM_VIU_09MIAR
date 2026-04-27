# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/ur5_moveit_bridge.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
"""Node that bridges a grasp pose to MoveIt planning/execution for the UR5."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
import gc
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import traceback
from typing import Any

import numpy as np

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
from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)
from geometry_msgs.msg import PoseStamped
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_msgs.msg import Constraints, JointConstraint, RobotState as MoveItRobotStateMsg
from moveit_msgs.srv import GetCartesianPath
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
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
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from controller_manager_msgs.srv import ListControllers

from .moveit_bridge.controller_management import ControllerManagementMixin
from .moveit_bridge.geometry import GeometryMixin
from .moveit_bridge.goal_validation import GoalValidationMixin
from .moveit_bridge.joint_state_helpers import JointStateHelpersMixin
from .moveit_bridge.moveit_commander_planner import MoveItCommanderMixin
from .moveit_bridge.moveit_py_planner import MoveItPyPlannerMixin
from .moveit_bridge.trajectory_prep import TrajectoryPrepMixin
from .param_utils import read_float_param, read_str_list_param, read_str_param
from .moveit_bridge_utils import (
    bridge_env_float,
    normalize_action_name,
    parse_request_meta,
    plan_success_code,
    plan_success_ok,
    plan_error_code_val,
    describe_execute_result,
    result_meta_to_message,
    diag_to_message,
    goal_status_text,
    wait_future_done,
    joint_trajectory_duration_sec,
    joint_trajectory_initial_segment_max_delta,
    scale_joint_trajectory_timing,
    pose_to_matrix,
    matrix_to_pose,
)

_BRIDGE_CODE_REV = "2026-04-20-approach-replan-v1"


class UR5MoveItBridge(
    MoveItPyPlannerMixin,
    MoveItCommanderMixin,
    GeometryMixin,
    TrajectoryPrepMixin,
    JointStateHelpersMixin,
    ControllerManagementMixin,
    GoalValidationMixin,
    Node,
):
    """Subscribes to grasp poses and drives MoveIt planning/execution."""

    def _log_bridge_status(self, message: str, *, level: str = "info") -> None:
        if level == "warn":
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def __init__(self) -> None:
        super().__init__("ur5_moveit_bridge")
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
        effective_goal_time_tol_sec = (
            float(goal_time_override_sec)
            if goal_time_override_sec is not None
            else float(self._controller_goal_time_tolerance_sec)
        )
        if phase_label_upper == "APPROACH":
            effective_goal_time_tol_sec = max(
                effective_goal_time_tol_sec,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_GOAL_TIME_TOL_SEC",
                    75.0,
                ),
            )
            if int(approach_replan_attempt) >= 1:
                effective_goal_time_tol_sec = max(
                    effective_goal_time_tol_sec,
                    self._env_float(
                        "PANEL_MOVEIT_BRIDGE_APPROACH_REPLAN_GOAL_TIME_TOL_SEC",
                        120.0,
                    ),
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
        if phase_label_upper == "APPROACH" and int(approach_replan_attempt) >= 1:
            approach_replan_min_traj_sec = max(
                30.0,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_REPLAN_MIN_TRAJ_SEC",
                    45.0,
                ),
            )
            if prepared_traj_sec + 1e-6 < approach_replan_min_traj_sec:
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
            available = ",".join(sorted(action_names)) if action_names else "none"
            checked = ",".join(candidates) if candidates else "none"
            meta = {
                "action": self._controller_action_name,
                "status_text": "NO_SERVER",
                "error_string": f"checked={checked} available={available}",
            }
            return (
                False,
                (
                    "fjt_no_action_server:"
                    f"expected_action={self._controller_action_name};"
                    f"checked_candidates={checked};available_actions={available}"
                ),
                meta,
            )

        client = self._ensure_fjt_action_client()
        if client is None:
            return False, "fjt_action_client_unavailable", {"action": action_name}
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt
        try:
            joint_names = list(getattr(jt, "joint_names", []) or [])
            target_joint_positions = list(getattr((list(getattr(jt, "points", []) or [])[-1]), "positions", []) or [])
            path_tol = (
                float(path_tol_override_rad)
                if path_tol_override_rad is not None
                else float(self._controller_path_tolerance_rad)
            )
            goal_tol = float(self._controller_goal_tolerance_rad)
            goal_time_tol = (
                float(effective_goal_time_tol_sec)
            )
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
            ee_goal_check_tol_m = max(
                0.02,
                float(ee_target_tol_m)
                if ee_target_tol_m is not None
                else self._env_float("PANEL_MOVEIT_BRIDGE_EE_TARGET_TOL_M", 0.10),
            )
            if phase_label_upper == "APPROACH":
                ee_goal_check_tol_m = max(
                    ee_goal_check_tol_m,
                    self._env_float(
                        "PANEL_MOVEIT_BRIDGE_APPROACH_EE_TARGET_TOL_M",
                        0.10,
                    ),
                )
            if phase_label_upper == "PRE_GRASP":
                ee_goal_check_tol_m = max(
                    ee_goal_check_tol_m,
                    self._env_float(
                        "PANEL_MOVEIT_BRIDGE_PREGRASP_EE_TARGET_TOL_M",
                        0.12,
                    ),
                )
            ee_goal_check_settle_sec = max(
                0.15,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_EE_TARGET_SETTLE_SEC",
                    goal_check_settle_sec,
                ),
            )
            if phase_label_upper == "APPROACH":
                ee_goal_check_settle_sec = min(
                    float(ee_goal_check_settle_sec),
                    max(
                        0.15,
                        self._env_float(
                            "PANEL_MOVEIT_BRIDGE_APPROACH_EE_TARGET_SETTLE_SEC",
                            0.25,
                        ),
                    ),
                )
                self.get_logger().info(
                    "[BRIDGE_EXEC] approach early-close profile "
                    f"ee_goal_check_tol_m={float(ee_goal_check_tol_m):.3f} "
                    f"ee_goal_check_settle_sec={float(ee_goal_check_settle_sec):.3f}"
                )
            if phase_label_upper == "PRE_GRASP":
                ee_goal_check_settle_sec = min(
                    float(ee_goal_check_settle_sec),
                    max(
                        0.15,
                        self._env_float(
                            "PANEL_MOVEIT_BRIDGE_PREGRASP_EE_TARGET_SETTLE_SEC",
                            0.20,
                        ),
                    ),
                )
                self.get_logger().info(
                    "[BRIDGE_EXEC] pregrasp early-close profile "
                    f"ee_goal_check_tol_m={float(ee_goal_check_tol_m):.3f} "
                    f"ee_goal_check_settle_sec={float(ee_goal_check_settle_sec):.3f}"
                )
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
            micro_retry_scale = max(
                1.2,
                self._env_float("PANEL_MOVEIT_BRIDGE_MICRO_RETRY_SCALE", 1.6),
            )
            approach_stall_retry_enabled = (
                phase_label_upper == "APPROACH"
                and retry_on_tolerance_violation
                and bool(start_joint_vec is not None)
            )
            approach_stall_retry_start_sec = max(
                8.0,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_STALL_RETRY_START_SEC",
                    12.0,
                ),
            )
            approach_stall_min_motion_rad = max(
                0.01,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_STALL_MIN_MOTION_RAD",
                    0.05,
                ),
            )
            approach_stall_retry_scale = max(
                1.2,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_STALL_RETRY_SCALE",
                    1.7,
                ),
            )
            approach_long_retry_enabled = (
                phase_label_upper == "APPROACH"
                and retry_on_tolerance_violation
                and int(approach_replan_attempt) <= 0
            )
            approach_long_retry_start_sec = max(
                30.0,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_LONG_RETRY_START_SEC",
                    55.0,
                ),
            )
            approach_long_retry_scale = max(
                1.2,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_LONG_RETRY_SCALE",
                    1.6,
                ),
            )
            approach_long_retry_joint_tol_rad = max(
                goal_check_tol_rad,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_LONG_RETRY_JOINT_TOL_RAD",
                    0.35,
                ),
            )
            approach_long_retry_ee_tol_m = max(
                ee_goal_check_tol_m,
                self._env_float(
                    "PANEL_MOVEIT_BRIDGE_APPROACH_LONG_RETRY_EE_TOL_M",
                    0.18,
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
            goal_check_start_sec = max(
                4.0,
                min(
                    25.0,
                    max(
                        4.0,
                        float(prepared_traj_sec) * 0.60,
                    ),
                ),
            )
            if target_pose is not None:
                goal_check_start_sec = min(
                    float(goal_check_start_sec),
                    max(
                        4.0,
                        self._env_float(
                            "PANEL_MOVEIT_BRIDGE_EARLY_TARGET_CHECK_START_SEC",
                            8.0,
                        ),
                    ),
                )
            if phase_label_upper == "PRE_GRASP":
                goal_check_start_sec = min(
                    float(goal_check_start_sec),
                    max(
                        4.0,
                        self._env_float(
                            "PANEL_MOVEIT_BRIDGE_PREGRASP_EARLY_TARGET_CHECK_START_SEC",
                            6.0,
                        ),
                    ),
                )
            if cold_start_first_goal:
                goal_check_start_sec = min(
                    float(goal_check_start_sec),
                    max(
                        4.0,
                        self._env_float(
                            "PANEL_MOVEIT_BRIDGE_FIRST_GOAL_CHECK_START_SEC",
                            10.0,
                        ),
                    ),
                )
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
                        approach_internal_replan_enabled = str(
                            os.environ.get(
                                "PANEL_MOVEIT_BRIDGE_APPROACH_INTERNAL_REPLAN",
                                "1",
                            )
                        ).strip().lower() not in ("0", "false", "no", "off")
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
                    approach_internal_replan_enabled = str(
                        os.environ.get(
                            "PANEL_MOVEIT_BRIDGE_APPROACH_INTERNAL_REPLAN",
                            "1",
                        )
                        or "0"
                    ).strip().lower() not in ("0", "false", "no", "off")
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
                    approach_internal_replan_enabled = str(
                        os.environ.get(
                            "PANEL_MOVEIT_BRIDGE_APPROACH_INTERNAL_REPLAN",
                            "1",
                        )
                        or "0"
                    ).strip().lower() not in ("0", "false", "no", "off")
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
            req_stamp_ns = (
                int(getattr(msg.header.stamp, "sec", 0) or 0) * 1_000_000_000
                + int(getattr(msg.header.stamp, "nanosec", 0) or 0)
            )
            if frame_clean in ("base", "/base") and self._base_frame == "base_link":
                self._command_seq += 1
                rejected_request_id = int(self._command_seq)
                rx_ts_us = int(time.time() * 1_000_000)
                self.get_logger().warning(
                    "[MOVEIT_BRIDGE][RX] "
                    f"ts_us={rx_ts_us} req_id={req_from_msg if req_from_msg is not None else rejected_request_id} "
                    f"req_uuid={req_uuid or 'n/a'} frame={frame_clean or 'n/a'} "
                    f"pose=({msg.pose.position.x:.3f},{msg.pose.position.y:.3f},{msg.pose.position.z:.3f}) "
                    "accepted=false reason=invalid_business_frame"
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
                    "[MOVEIT_BRIDGE][RX] "
                    f"ts_us={rx_ts_us} req_id={rejected_request_id} req_uuid={req_uuid or 'n/a'} "
                    f"frame={frame_clean or 'n/a'} "
                    f"pose=({msg.pose.position.x:.3f},{msg.pose.position.y:.3f},{msg.pose.position.z:.3f}) "
                    "accepted=false reason=missing_request_id"
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
                    busy_message = (
                        "bridge_busy:"
                        f"active_request_id={active_request_id};"
                        f"active_request_uuid={active_request_uuid or 'n/a'};"
                        f"active_age={busy_age:.2f}s"
                    )
                    self.get_logger().warning(
                        "[BRIDGE][QUEUE] reject_overlapping_request "
                        f"request_id={request_id} active_request_id={active_request_id} "
                        f"active_request_uuid={active_request_uuid or 'n/a'} active_age={busy_age:.2f}s"
                    )
                    rx_ts_us = int(time.time() * 1_000_000)
                    self.get_logger().warning(
                        "[MOVEIT_BRIDGE][RX] "
                        f"ts_us={rx_ts_us} req_id={request_id} req_uuid={req_uuid or 'n/a'} "
                        f"frame={frame_clean or 'n/a'} "
                        f"pose=({msg.pose.position.x:.3f},{msg.pose.position.y:.3f},{msg.pose.position.z:.3f}) "
                        "accepted=false reason=bridge_busy"
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
                "[MOVEIT_BRIDGE][RX] "
                f"ts_us={rx_ts_us} req_id={request_id} req_uuid={req_uuid or 'n/a'} "
                f"frame={msg.header.frame_id or 'n/a'} "
                f"pose=({msg.pose.position.x:.3f},{msg.pose.position.y:.3f},{msg.pose.position.z:.3f}) "
                "accepted=true"
            )
            self.get_logger().info(
                "[PICK][MOVEIT][REQUEST] "
                f"ts_us={rx_ts_us} request_id={request_id} request_uuid={req_uuid or 'n/a'} "
                f"frame={msg.header.frame_id or 'n/a'} "
                f"pose=({msg.pose.position.x:.3f},{msg.pose.position.y:.3f},{msg.pose.position.z:.3f}) "
                f"cartesian={str(bool(cartesian)).lower()} phase={phase_label or 'n/a'} "
                f"ee_target_tol_m={ee_target_tol_m if ee_target_tol_m is not None else 'n/a'} "
                "accepted=true"
            )
            self._plan_event.set()
            pos = msg.pose.position
            if dropped_pending > 0:
                self.get_logger().warning(
                    "[BRIDGE][QUEUE] dropped_pending "
                    f"count={dropped_pending} reason=tagged_request request_id={request_id}"
                )
            self.get_logger().info(
                "[BRIDGE][RECV] "
                f"label={'CARTESIAN' if cartesian else 'POSE'} "
                f"topic={topic_name or 'n/a'} request_id={request_id} frame={msg.header.frame_id} "
                f"pos=({pos.x:.3f},{pos.y:.3f},{pos.z:.3f}) "
                f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
                f"frame_raw={frame_raw or 'n/a'} request_uuid={req_uuid or 'n/a'} "
                f"ee_target_tol_m={ee_target_tol_m if ee_target_tol_m is not None else 'n/a'} "
                f"phase={phase_label or 'n/a'}"
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
                    if self._stale_request_ttl_sec > 0.0:
                        queued_age = max(0.0, time.monotonic() - float(queued_mono))
                        if queued_age > float(self._stale_request_ttl_sec):
                            self._publish_result(
                                request_id=request_id,
                                request_uuid=request_uuid,
                                target=target,
                                request_stamp_ns=request_stamp_ns,
                                cartesian=cartesian,
                                success=False,
                                plan_ok=False,
                                exec_ok=False,
                                message=(
                                    f"stale_request_dropped:age={queued_age:.2f}s"
                                ),
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
                    settle_timeout_sec = max(
                        0.4,
                        self._env_float(
                            "PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_TIMEOUT_SEC",
                            min(1.5, max(0.4, float(self._joint_state_valid_timeout_sec))),
                        ),
                    )
                    settle_stable_sec = max(
                        0.05,
                        self._env_float("PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_STABLE_SEC", 0.25),
                    )
                    settle_tol_rad = max(
                        0.005,
                        self._env_float("PANEL_MOVEIT_BRIDGE_JOINT_SETTLE_TOL_RAD", 0.02),
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
                        "[BRIDGE][EXEC_START] "
                        f"request_id={request_id} "
                        f"label={'CARTESIAN' if cartesian else 'POSE'} "
                        f"frame={target.header.frame_id} ee_link={self._ee_frame} "
                        f"base_frame={self._base_frame} "
                        f"pos=({target.pose.position.x:.3f},{target.pose.position.y:.3f},{target.pose.position.z:.3f})"
                    )
                    self.get_logger().info(
                        f"Planificando id={request_id} frame={target.header.frame_id} "
                        f"(cartesian={cartesian}, ee_link={self._ee_frame})"
                    )
                    self.get_logger().info(
                        "[PICK][MOVEIT][TARGET] "
                        f"request_id={request_id} request_uuid={request_uuid or 'n/a'} "
                        f"phase={phase_label or 'n/a'} frame={target.header.frame_id or 'n/a'} "
                        f"pose=({target.pose.position.x:.3f},{target.pose.position.y:.3f},{target.pose.position.z:.3f}) "
                        f"cartesian={str(bool(cartesian)).lower()} ee_frame={self._ee_frame or 'n/a'}"
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


def main(args=None) -> None:
    node: UR5MoveItBridge | None = None
    fast_exit_on_sigint = False
    rclpy.init(args=args)
    if MoveItPy is None and moveit_commander is None:
        raise SystemExit("MoveIt Python no disponible. Instala ros-jazzy-moveit-py.")
    if moveit_commander is not None:
        moveit_commander.roscpp_initialize(sys.argv)
    node = UR5MoveItBridge()
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
