# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/ur5_moveit_bridge.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
"""Node that bridges a grasp pose to MoveIt planning/execution for the UR5."""

from __future__ import annotations

from collections import deque
import json
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
from moveit_msgs.msg import Constraints, JointConstraint
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
from controller_manager_msgs.srv import ListControllers

from .param_utils import read_float_param, read_str_list_param, read_str_param


class UR5MoveItBridge(Node):
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
        self.declare_parameter("require_request_id", True)
        self.declare_parameter("drop_pending_on_tagged_request", True)
        self.declare_parameter("stale_request_ttl_sec", 20.0)
        self.declare_parameter("dry_run_plan_only", False)
        self.declare_parameter("joint_state_valid_timeout_sec", 2.0)
        self.declare_parameter("joint_state_valid_max_age_sec", 1.0)
        self.declare_parameter("max_velocity_scaling_factor", 0.30)
        self.declare_parameter("max_acceleration_scaling_factor", 0.30)
        self.declare_parameter("path_constraint_joint_tolerance_rad", 0.0)
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
        self._use_sim_time = bool(self.get_parameter("use_sim_time").value)
        requested_moveit_py_sim = bool(
            self.get_parameter("moveit_py_use_sim_time").value
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
            self.get_logger().warning(
                "use_sim_time=true pero moveit_py_use_sim_time=false; "
                "puede aparecer validacion temporal inconsistente en execute."
            )
        self.get_logger().info(
            "Bridge config: "
            f"backend_pref={self._backend_pref or 'auto'} group={self._group_name} "
            f"base={self._base_frame} ee={self._ee_frame} result_topic={self._result_topic} "
            f"execute_timeout_sec={self._execute_timeout_sec:.1f} "
            f"request_timeout_sec={self._request_timeout_sec:.1f} "
            f"use_sim_time={str(self._use_sim_time).lower()} "
            f"moveit_py_use_sim_time={str(self._moveit_py_use_sim_time).lower()} "
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
            f"path_constraint_joint_tol={self._path_constraint_joint_tol:.3f}rad"
        )

        self._backend = None
        self._moveit_py = None
        self._planning_component = None
        self._move_group = None
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
        self._last_plan_time = 0.0
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
            threading.Thread(target=self._init_moveit_py, daemon=True).start()
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
        except Exception as exc:
            self.get_logger().warning(
                f"[BRIDGE][EXCEPTION] callback=heartbeat_publish type={type(exc).__name__} err={exc}"
            )

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

    _ARM_JOINTS = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
    )

    def _is_test_touch_request(self, request_uuid: str) -> bool:
        req = str(request_uuid or "").strip().lower()
        return req.startswith("test_touch:")

    def _build_joint_path_constraints(self, request_uuid: str = "") -> Constraints | None:
        """Build joint path constraints to prevent IK configuration flipping.

        Constrains shoulder_pan, shoulder_lift, and elbow joints
        within +/- tolerance_rad of their current positions so the planner
        stays in the same arm configuration.
        """
        if self._is_test_touch_request(request_uuid):
            self.get_logger().info(
                "[BRIDGE_CONSTRAINT] skip constraints for TEST_TOUCH request_uuid="
                f"{request_uuid or 'n/a'}"
            )
            return None
        tol = self._path_constraint_joint_tol
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
        for jname in self._ARM_JOINTS:
            if jname not in joint_map:
                continue
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = joint_map[jname]
            jc.tolerance_above = tol
            jc.tolerance_below = tol
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
            constrained.append((jname, joint_map[jname]))
        if not constrained:
            return None
        self.get_logger().info(
            f"[BRIDGE_CONSTRAINT] joints={[n for n, _ in constrained]} "
            f"positions={[f'{p:.3f}' for _, p in constrained]} "
            f"tol={tol:.3f}"
        )
        return constraints

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

    def _available_action_names(self) -> list[str]:
        try:
            names = self.get_action_names_and_types()
            return [str(name) for name, _types in names]
        except Exception:
            return []

    @staticmethod
    def _normalize_action_name(name: str) -> str:
        raw = str(name or "").strip()
        if not raw:
            return ""
        if not raw.startswith("/"):
            raw = f"/{raw}"
        return raw.replace("//", "/")

    def _destroy_fjt_action_client(self) -> None:
        with self._action_client_lock:
            client = self._fjt_client
            self._fjt_client = None
            self._fjt_action_name = ""
        if client is None:
            return
        try:
            client.destroy()
        except Exception:
            pass

    def _ensure_fjt_action_client(self) -> ActionClient | None:
        action_name = self._normalize_action_name(self._controller_action_name)
        if not action_name:
            return None
        with self._action_client_lock:
            if self._fjt_client is not None and self._fjt_action_name == action_name:
                return self._fjt_client
            old_client = self._fjt_client
            self._fjt_client = None
            self._fjt_action_name = ""
            if old_client is not None:
                try:
                    old_client.destroy()
                except Exception:
                    pass
            try:
                client = ActionClient(self, FollowJointTrajectory, action_name)
            except Exception as exc:
                self.get_logger().warning(
                    "[BRIDGE_EXEC] no se pudo crear ActionClient "
                    f"action={action_name}: {type(exc).__name__}: {exc}"
                )
                return None
            self._fjt_client = client
            self._fjt_action_name = action_name
            return client

    def _prime_fjt_action_client(self) -> None:
        try:
            self._ensure_fjt_action_client()
            timer = self._fjt_prime_timer
            if timer is not None:
                try:
                    timer.cancel()
                except Exception:
                    pass
                self._fjt_prime_timer = None
        except Exception as exc:
            self.get_logger().warning(
                "[BRIDGE_EXEC] prime ActionClient fallido "
                f"type={type(exc).__name__} err={exc}"
            )

    def _controller_manager_service_name(self, service: str) -> str:
        base = (self._controller_manager_name or "/controller_manager").strip()
        if not base:
            base = "/controller_manager"
        if not base.startswith("/"):
            base = f"/{base}"
        base = base.rstrip("/")
        return f"{base}/{service}"

    def _controller_action_candidates(self, action_names: list[str]) -> list[str]:
        candidates: list[str] = []
        seen: set[str] = set()
        controller = str(self._controller_name or "joint_trajectory_controller").strip("/")
        action_ns = str(self._controller_action_ns or "follow_joint_trajectory").strip("/")
        expected = str(self._controller_action_name or "").strip()
        base_cm = (self._controller_manager_name or "/controller_manager").strip().rstrip("/")
        if not base_cm.startswith("/"):
            base_cm = f"/{base_cm}"
        base_cm_ns = base_cm.removesuffix("/controller_manager")
        for raw in (
            expected,
            f"/{controller}/{action_ns}",
            f"/controller_manager/{controller}/{action_ns}",
            f"{base_cm}/{controller}/{action_ns}",
            f"{base_cm_ns}/{controller}/{action_ns}" if base_cm_ns else "",
        ):
            name = str(raw or "").strip()
            if not name:
                continue
            if not name.startswith("/"):
                name = f"/{name}"
            name = name.replace("//", "/")
            if name in seen:
                continue
            seen.add(name)
            candidates.append(name)
        suffix = f"/{controller}/{action_ns}"
        for name in action_names:
            norm = f"/{name.lstrip('/')}"
            if norm.endswith(suffix) and norm not in seen:
                seen.add(norm)
                candidates.append(norm)
        return candidates

    def _active_controllers(self, timeout_sec: float = 0.8) -> list[str]:
        client = self._list_controllers_client
        if client is None:
            return []
        if not client.wait_for_service(timeout_sec=min(0.2, timeout_sec)):
            return []
        req = ListControllers.Request()
        fut = client.call_async(req)
        deadline = time.monotonic() + max(0.1, timeout_sec)
        while time.monotonic() < deadline:
            if fut.done():
                break
            time.sleep(0.05)
        if not fut.done():
            return []
        try:
            resp = fut.result()
        except Exception:
            return []
        out: list[str] = []
        for c in getattr(resp, "controller", []) or []:
            try:
                if str(getattr(c, "state", "")).lower() == "active":
                    out.append(str(getattr(c, "name", "")))
            except Exception:
                continue
        return out

    def _wait_for_expected_controller_action(
        self, timeout_sec: float = 2.0
    ) -> tuple[bool, str, list[str], list[str]]:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        last_actions: list[str] = []
        last_candidates: list[str] = []
        while time.monotonic() <= deadline:
            last_actions = self._available_action_names()
            last_candidates = self._controller_action_candidates(last_actions)
            client = self._ensure_fjt_action_client()
            if client is not None:
                try:
                    if client.wait_for_server(timeout_sec=0.2):
                        matched = self._normalize_action_name(self._controller_action_name)
                        return True, matched, last_actions, last_candidates
                except Exception as exc:
                    self.get_logger().warning(
                        "[BRIDGE_EXEC] wait_for_server fallido "
                        f"action={self._controller_action_name}: {type(exc).__name__}: {exc}"
                    )
                    return False, "", last_actions, last_candidates
            time.sleep(0.1)
        return False, "", last_actions, last_candidates

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
        js_age = diag.get("joint_state_age_sec")
        js_age_txt = "n/a" if js_age is None else f"{float(js_age):.3f}"
        return (
            f"{diag.get('reason')};backend={diag.get('backend')};"
            f"use_sim_time={str(diag.get('use_sim_time')).lower()};"
            f"controller={diag.get('controller')};"
            f"action={diag.get('controller_action')};"
            f"action_matched={diag.get('controller_action_matched') or 'none'};"
            f"action_available={str(diag.get('controller_action_available')).lower()};"
            f"controller_manager={diag.get('controller_manager')};"
            f"active_controllers={','.join(diag.get('active_controllers') or []) or 'none'};"
            f"joint_state_age_sec={js_age_txt};"
            f"trajectory_topic_pubs={diag.get('trajectory_topic_pubs')};"
            f"trajectory_topic_subs={diag.get('trajectory_topic_subs')}"
        )

    @staticmethod
    def _describe_execute_result(result: Any) -> dict[str, Any]:
        if result is None:
            return {"type": "NoneType"}
        meta: dict[str, Any] = {"type": type(result).__name__}
        for key in ("success", "status", "message", "error_code", "val"):
            try:
                value = getattr(result, key)
            except Exception:
                continue
            if value is not None:
                meta[key] = value
        return meta

    @staticmethod
    def _plan_success_code(success: Any) -> int | None:
        if success is None:
            return None
        if isinstance(success, bool):
            return 1 if success else 0
        if isinstance(success, (int, float)):
            return int(success)
        for attr in ("val", "value", "code"):
            try:
                if hasattr(success, attr):
                    return int(getattr(success, attr))
            except Exception:
                continue
        return None

    @classmethod
    def _plan_success_ok(cls, success: Any) -> bool:
        # moveit_msgs/MoveItErrorCodes.SUCCESS == 1
        code = cls._plan_success_code(success)
        if code is not None:
            return int(code) == 1
        if isinstance(success, str):
            token = success.strip().upper()
            if token in ("SUCCESS", "SUCCEEDED", "OK"):
                return True
            if token in ("FAIL", "FAILED", "ERROR", "ABORTED"):
                return False
        if success is None:
            # Some bindings omit explicit success when trajectory is valid.
            return True
        return bool(success)

    @staticmethod
    def _plan_error_code_val(plan) -> int | None:
        """Return integer MoveIt error code from *plan*.error_code, or None."""
        ec = getattr(plan, "error_code", None)
        if ec is None:
            return None
        # moveit_msgs/MoveItErrorCodes has .val; raw int also possible.
        val = getattr(ec, "val", None)
        if val is not None:
            return int(val)
        if isinstance(ec, (int, float)):
            return int(ec)
        return None

    @staticmethod
    def _result_meta_to_message(meta: dict[str, Any]) -> str:
        pieces: list[str] = []
        for key in (
            "type",
            "success",
            "status",
            "status_text",
            "message",
            "error_code",
            "error_string",
            "action",
            "accepted",
            "val",
        ):
            if key in meta:
                pieces.append(f"{key}={meta[key]}")
        return ",".join(pieces) if pieces else "none"

    @staticmethod
    def _goal_status_text(status: int) -> str:
        mapping = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        return mapping.get(int(status), f"STATUS_{int(status)}")

    @staticmethod
    def _wait_future_done(future, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(0.1, timeout_sec)
        while time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.02)
        return bool(future.done())

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
        self, jt: JointTrajectory, *, timeout_sec: float = 8.0
    ) -> tuple[bool, str, dict[str, Any]]:
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
        send_future = client.send_goal_async(goal)
        if not self._wait_future_done(send_future, timeout_sec=min(2.0, timeout_sec)):
            return False, "fjt_goal_send_timeout", {"action": action_name}
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return (
                False,
                "fjt_goal_rejected",
                {"action": action_name, "accepted": bool(getattr(goal_handle, "accepted", False))},
            )

        result_future = goal_handle.get_result_async()
        if not self._wait_future_done(result_future, timeout_sec=max(1.0, timeout_sec)):
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            diag = self._exec_diagnostics()
            detail = self._diag_to_message(diag)
            meta = {
                "action": action_name,
                "status_text": "TIMEOUT",
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
        status = int(getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN) or GoalStatus.STATUS_UNKNOWN)
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
            self.get_logger().info(f"[BRIDGE_EXEC] FollowJointTrajectory OK ({detail})")
            return True, f"fjt_execute_ok:{detail}", meta
        self.get_logger().warning(f"[BRIDGE_EXEC] FollowJointTrajectory FAIL ({detail})")
        return False, f"fjt_aborted:{detail}", meta

    def _extract_joint_trajectory_msg(self, trajectory) -> JointTrajectory | None:
        if trajectory is None:
            return None
        queue = [trajectory]
        visited = set()
        while queue:
            item = queue.pop(0)
            if item is None:
                continue
            item_id = id(item)
            if item_id in visited:
                continue
            visited.add(item_id)

            if isinstance(item, JointTrajectory):
                if getattr(item, "points", None):
                    return item
                continue

            jt = getattr(item, "joint_trajectory", None)
            if isinstance(jt, JointTrajectory) and getattr(jt, "points", None):
                return jt

            for attr in (
                "trajectory",
                "_trajectory",
                "robot_trajectory",
                "_robot_trajectory",
                "_msg",
                "msg",
            ):
                try:
                    nested = getattr(item, attr, None)
                except Exception:
                    nested = None
                if nested is not None:
                    queue.append(nested)

            for meth in ("get_robot_trajectory_msg", "to_msg"):
                fn = getattr(item, meth, None)
                if callable(fn):
                    try:
                        nested = fn()
                    except Exception:
                        nested = None
                    if nested is not None:
                        queue.append(nested)

        self.get_logger().warning(
            "Fallback execution: no se pudo extraer JointTrajectory "
            f"(trajectory_type={type(trajectory).__name__})"
        )
        return None

    @staticmethod
    def _qos_summary(qos: QoSProfile) -> str:
        reliability = "RELIABLE" if qos.reliability == ReliabilityPolicy.RELIABLE else "BEST_EFFORT"
        durability = "VOLATILE" if qos.durability == DurabilityPolicy.VOLATILE else "TRANSIENT_LOCAL"
        history = "KEEP_LAST" if qos.history == HistoryPolicy.KEEP_LAST else "KEEP_ALL"
        return f"{reliability}/{durability}/{history}@depth={qos.depth}"

    @staticmethod
    def _parse_request_meta(frame_raw: str) -> tuple[str, int | None, str]:
        raw = str(frame_raw or "").strip()
        if not raw:
            return "", None, ""
        parts = [p.strip() for p in raw.split("|")]
        frame = parts[0] if parts else raw
        req_id = None
        req_uuid = ""
        for token in parts[1:]:
            key, sep, value = token.partition("=")
            if sep != "=":
                continue
            k = key.strip().lower()
            v = value.strip()
            if not v:
                continue
            if k in ("rid", "request_id"):
                try:
                    req_id = int(v)
                except Exception:
                    req_id = None
            elif k in ("uid", "uuid", "request_uuid"):
                req_uuid = v
        return frame, req_id, req_uuid

    def _pose_callback(self, msg: PoseStamped, cartesian: bool = False, topic_name: str = "") -> None:
        try:
            now = time.monotonic()
            frame_raw = str(getattr(msg.header, "frame_id", "") or "")
            frame_clean, req_from_msg, req_uuid = self._parse_request_meta(frame_raw)
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
            if self._require_request_id and req_from_msg is None:
                self._command_seq += 1
                rejected_request_id = int(self._command_seq)
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
                        req_stamp_ns,
                        now,
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
                "[BRIDGE][RECV] "
                f"label={'CARTESIAN' if cartesian else 'POSE'} "
                f"topic={topic_name or 'n/a'} request_id={request_id} frame={msg.header.frame_id} "
                f"pos=({pos.x:.3f},{pos.y:.3f},{pos.z:.3f}) "
                f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} "
                f"frame_raw={frame_raw or 'n/a'} request_uuid={req_uuid or 'n/a'}"
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
    ) -> tuple[bool, str, bool, bool]:
        if cartesian:
            return self._plan_cartesian(target)
        if self._backend == "moveit_py":
            return self._plan_with_moveit_py(target, request_uuid=request_uuid)
        return self._plan_with_moveit_commander(target)

    def _dispatch_plan_request_with_timeout(
        self,
        *,
        request_id: int,
        request_uuid: str,
        target: PoseStamped,
        cartesian: bool,
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
                )
            except Exception as exc:
                error["exc"] = exc
            finally:
                done.set()

        thread = threading.Thread(target=_runner, daemon=True, name=f"plan_request_{request_id}")
        thread.start()
        timeout_sec = max(2.0, float(self._request_timeout_sec))
        if not done.wait(timeout_sec):
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
        return holder.get("result", (False, "empty_result", False, False))

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
                        except Exception as pub_exc:
                            self.get_logger().error(
                                "[BRIDGE][PUB_RESULT] failed "
                                f"request_id={request_id} topic={self._result_topic} err={pub_exc}"
                            )
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
            "[BRIDGE_RESULT] "
            f"request_id={request_id} success={str(bool(success)).lower()} "
            f"request_uuid={request_uuid or 'n/a'} "
            f"plan_ok={str(bool(plan_ok)).lower()} exec_ok={str(bool(exec_ok)).lower()} "
            f"message={message or 'n/a'} topic={self._result_topic}"
        )

    def _plan_with_moveit_py(
        self,
        target: PoseStamped,
        request_uuid: str = "",
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
            self._planning_component.set_start_state_to_current_state()
            try:
                self._planning_component.set_goal_state(
                    pose_stamped_msg=target,
                    pose_link=self._ee_frame,
                )
            except TypeError:
                # Fallback for older MoveItPy bindings.
                self._planning_component.set_goal_state(target, self._ee_frame)
            path_constraints = self._build_joint_path_constraints(request_uuid=request_uuid)
            if path_constraints is not None:
                self._planning_component.set_path_constraints(path_constraints)
            try:
                plan = self._planning_component.plan()
            finally:
                if path_constraints is not None:
                    self._planning_component.set_path_constraints(Constraints())
            # Fallback: if plan failed WITH constraints, retry WITHOUT them
            if path_constraints is not None and plan is not None:
                _pc_ok = self._plan_success_ok(getattr(plan, "success", None))
                _pc_traj = getattr(plan, "trajectory", None) is not None
                _pc_ec = self._plan_error_code_val(plan)
                _pc_ec_bad = _pc_ec is not None and _pc_ec != 1
                if not _pc_ok or not _pc_traj or _pc_ec_bad:
                    self.get_logger().warning(
                        "[BRIDGE_CONSTRAINT] plan failed with constraints; "
                        f"retrying WITHOUT constraints (fallback) "
                        f"success_ok={_pc_ok} traj={_pc_traj} error_code={_pc_ec}"
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
        self._log_bridge_status("[BRIDGE_STATUS] plan_ok backend=moveit_py")
        self.get_logger().info("Planificación MoveItPy OK.")
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
        if self._use_sim_time and not self._moveit_py_use_sim_time:
            jt_direct = self._extract_joint_trajectory_msg(trajectory)
            if jt_direct is not None:
                fjt_timeout = max(8.0, float(self._execute_timeout_sec) + 4.0)
                fjt_ok, fjt_msg, fjt_meta = self._execute_joint_trajectory_action(
                    jt_direct, timeout_sec=fjt_timeout
                )
                fjt_detail = self._result_meta_to_message(fjt_meta)
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
            fjt_timeout = max(8.0, float(self._execute_timeout_sec) + 4.0)
            fjt_ok, fjt_msg, fjt_meta = self._execute_joint_trajectory_action(
                jt,
                timeout_sec=fjt_timeout,
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

    def _get_cartesian_group(self) -> MoveGroupCommander | None:
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
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True
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
        if self._dry_run_plan_only:
            self._log_bridge_status(
                "[BRIDGE_STATUS] exec_skip backend=moveit_py reason=dry_run_plan_only_cartesian"
            )
            self.get_logger().warning(
                "[BRIDGE_DRY_RUN] cartesian plan generated and execution skipped "
                "(dry_run_plan_only=true)."
            )
            return True, "dry_run_plan_only", True, False
        fjt_timeout = max(8.0, float(self._execute_timeout_sec) + 4.0, final_sec + 4.0)
        fjt_ok, fjt_msg, fjt_meta = self._execute_joint_trajectory_action(
            traj.joint_trajectory, timeout_sec=fjt_timeout
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
        self._destroy_fjt_action_client()


def main(args=None) -> None:
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
    except Exception as exc:
        # External shutdown paths can raise executor/context exceptions; keep
        # process exit clean while leaving evidence in log.
        node.get_logger().warning(
            "[BRIDGE][EXCEPTION] main_loop "
            f"type={type(exc).__name__} err={exc}\n{traceback.format_exc()}"
        )
    finally:
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
