#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/system_state_manager.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_tools/ur5_tools/system_state_manager.py
# Summary: Publishes a global system state for the UR5 stack.
"""Publish global system readiness state for the UR5 stack."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
import time
from typing import Dict, List, Optional, Tuple

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from .param_utils import read_float_param, read_str_list_param, read_str_param


class SystemState(str, Enum):
    BOOTING = "BOOTING"
    WAITING_GAZEBO = "WAITING_GAZEBO"
    WAITING_BRIDGE = "WAITING_BRIDGE"
    WAITING_CONTROLLERS = "WAITING_CONTROLLERS"
    WAITING_TF = "WAITING_TF"
    WAITING_CAMERA = "WAITING_CAMERA"
    WAITING_MOVEIT = "WAITING_MOVEIT"
    READY = "READY"
    ERROR = "ERROR"
    ERROR_FATAL = "ERROR_FATAL"


@dataclass
class DependencySnapshot:
    clock_ok: bool
    clock_age: float
    pose_ok: bool
    pose_age: float
    pose_model_seen: bool
    pose_entities_seen: bool
    camera_ok: bool
    camera_age: float
    tf_ok: bool
    tf_reason: str
    controllers_ready: bool
    controllers_reason: str
    missing_controllers: List[str]
    moveit_ready: bool


@dataclass(frozen=True)
class SystemInputs:
    snapshot: DependencySnapshot
    startup_expired: bool
    ever_ready: bool


class SystemStateMachine:
    def __init__(
        self, *, required_controllers: List[str], moveit_required: bool
    ) -> None:
        self._required_controllers = required_controllers
        self._moveit_required = moveit_required

    def decide(self, inputs: SystemInputs) -> StateDecision:
        next_state, next_reason = self._next_state(inputs.snapshot)
        if self._critical_missing(inputs.snapshot) and inputs.startup_expired:
            critical_reason = self._critical_reason(inputs.snapshot)
            return StateDecision(
                state=next_state,
                reason=next_reason,
                fatal_reason=f"critical dependency missing: {critical_reason}",
            )
        if inputs.ever_ready and next_state != SystemState.READY:
            if self._critical_missing(inputs.snapshot):
                critical_reason = self._critical_reason(inputs.snapshot)
                return StateDecision(
                    state=next_state,
                    reason=next_reason,
                    fatal_reason=f"drop: {critical_reason}",
                )
            return StateDecision(state=SystemState.ERROR, reason=f"drop: {next_reason}")
        return StateDecision(state=next_state, reason=next_reason)

    @staticmethod
    def _critical_missing(snapshot: DependencySnapshot) -> bool:
        tf_in_grace = str(snapshot.tf_reason).startswith("grace:")
        tf_critical = snapshot.controllers_ready and (not snapshot.tf_ok) and (not tf_in_grace)
        return not (
            snapshot.clock_ok
            and snapshot.pose_ok
            and snapshot.camera_ok
            and (not tf_critical)
        )

    @staticmethod
    def _critical_reason(snapshot: DependencySnapshot) -> str:
        if not snapshot.clock_ok:
            if snapshot.clock_age == float("inf"):
                return "/clock no disponible"
            return f"/clock age={snapshot.clock_age:.2f}s"
        if not snapshot.pose_ok:
            if snapshot.pose_age == float("inf"):
                return "pose/info no disponible"
            return f"pose/info age={snapshot.pose_age:.2f}s"
        if snapshot.controllers_ready and (not snapshot.tf_ok):
            return f"tf={snapshot.tf_reason}"
        if not snapshot.camera_ok:
            if snapshot.camera_age == float("inf"):
                return "camera sin frames válidos"
            return f"camera age={snapshot.camera_age:.2f}s"
        return "ok"

    def _next_state(self, snapshot: DependencySnapshot) -> Tuple[SystemState, str]:
        if not snapshot.clock_ok:
            return SystemState.WAITING_GAZEBO, f"/clock age={snapshot.clock_age:.2f}s"
        if not snapshot.pose_ok:
            return (
                SystemState.WAITING_BRIDGE,
                f"pose/info age={snapshot.pose_age:.2f}s model={snapshot.pose_model_seen}",
            )
        if not snapshot.controllers_ready:
            reason = snapshot.controllers_reason
            if not reason:
                if snapshot.missing_controllers:
                    reason = "controllers missing: " + ", ".join(snapshot.missing_controllers)
                else:
                    reason = "controllers no listos"
            return SystemState.WAITING_CONTROLLERS, reason
        if not snapshot.tf_ok:
            return SystemState.WAITING_TF, f"tf={snapshot.tf_reason}"
        if not snapshot.camera_ok:
            if snapshot.camera_age == float("inf"):
                return SystemState.WAITING_CAMERA, "camera sin frames válidos"
            return SystemState.WAITING_CAMERA, f"camera age={snapshot.camera_age:.2f}s"
        if self._moveit_required and not snapshot.moveit_ready:
            return SystemState.WAITING_MOVEIT, "move_group no listo"
        return SystemState.READY, "Sistema listo"


@dataclass(frozen=True)
class StateDecision:
    state: SystemState
    reason: str
    fatal_reason: Optional[str] = None


class SystemStateManager(Node):
    """Track system readiness and publish /system_state + /system_diag."""

    def __init__(self) -> None:
        super().__init__("system_state_manager")
        self.declare_parameter("world_name", "ur5_mesa_objetos")
        self.declare_parameter("model_name", "ur5_rg2")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("ee_frame", "tool0")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("camera_topic", "/camera_overhead/image")
        self.declare_parameter("camera_required", True)
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter(
            "required_controllers",
            [
                "joint_state_broadcaster",
                "joint_trajectory_controller",
                "gripper_controller",
            ],
        )
        self.declare_parameter("moveit_required", False)
        self.declare_parameter(
            "moveit_service_names",
            [
                "/get_planning_scene",
                "/move_group/get_planning_scene",
            ],
        )
        self.declare_parameter("moveit_check_sec", 1.0)
        self.declare_parameter("clock_timeout_sec", 1.5)
        self.declare_parameter("pose_timeout_sec", 1.0)
        self.declare_parameter("camera_timeout_sec", 1.5)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("tf_drop_grace_sec", 4.0)
        self.declare_parameter("controller_check_sec", 1.0)
        self.declare_parameter("controller_future_timeout_sec", 2.0)
        self.declare_parameter("state_publish_hz", 2.0)
        self.declare_parameter("startup_timeout_sec", 0.0)

        self._world_name = read_str_param(self, "world_name", "ur5_mesa_objetos")
        self._model_name = read_str_param(self, "model_name", "ur5_rg2")
        self._base_frame = read_str_param(self, "base_frame", "base_link")
        self._world_frame = read_str_param(self, "world_frame", "world")
        self._ee_frame = read_str_param(self, "ee_frame", "tool0")
        pose_topic = read_str_param(self, "pose_topic", "")
        self._pose_topic = pose_topic or f"/world/{self._world_name}/pose/info"
        self._camera_topic = read_str_param(
            self, "camera_topic", "/camera_overhead/image"
        )
        self._camera_required = bool(self.get_parameter("camera_required").value)
        self._controller_manager = read_str_param(
            self,
            "controller_manager",
            "/controller_manager",
        )
        self._required_controllers = read_str_list_param(self, "required_controllers")
        self._moveit_required = bool(self.get_parameter("moveit_required").value)
        self._moveit_services = read_str_list_param(self, "moveit_service_names")
        self._moveit_check_sec = read_float_param(
            self,
            "moveit_check_sec",
            1.0,
            min_value=0.2,
        )
        self._clock_timeout = read_float_param(
            self, "clock_timeout_sec", 1.5, min_value=0.1
        )
        self._pose_timeout = read_float_param(
            self, "pose_timeout_sec", 1.0, min_value=0.1
        )
        self._camera_timeout = read_float_param(
            self, "camera_timeout_sec", 1.5, min_value=0.1
        )
        self._tf_timeout = read_float_param(self, "tf_timeout_sec", 0.2, min_value=0.05)
        self._tf_drop_grace = read_float_param(
            self,
            "tf_drop_grace_sec",
            4.0,
            min_value=0.0,
        )
        self._controller_check_sec = read_float_param(
            self,
            "controller_check_sec",
            1.0,
            min_value=0.1,
        )
        self._controller_future_timeout = read_float_param(
            self, "controller_future_timeout_sec", 2.0, min_value=0.2
        )
        self._state_hz = read_float_param(self, "state_publish_hz", 2.0, min_value=0.5)
        self._startup_timeout = read_float_param(
            self, "startup_timeout_sec", 0.0, min_value=0.0
        )

        self._start_wall = self._now()
        self._last_clock_wall = 0.0
        self._last_clock_msg_ns: Optional[int] = None
        self._last_clock_advance_wall = 0.0
        self._pose_last_wall = 0.0
        self._pose_model_seen = False
        self._pose_entities_seen = False
        self._camera_last_wall = 0.0
        self._camera_last_stamp_ns: Optional[int] = None
        self._camera_last_advance_wall = 0.0
        self._camera_frames = 0
        self._controllers_state: Dict[str, str] = {}
        self._controllers_ready = False
        self._controllers_reason = "controller check pending"
        self._last_controller_check = 0.0
        self._controller_future = None
        self._controller_future_start = 0.0
        self._moveit_ready = False
        self._moveit_last_check = 0.0
        self._moveit_last_ok = False
        self._ever_ready = False
        self._state = SystemState.BOOTING.value
        self._reason = "boot"
        self._fatal_latched = False
        self._tf_missing_since = 0.0
        self._fsm = SystemStateMachine(
            required_controllers=self._required_controllers,
            moveit_required=self._moveit_required,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._state_pub = self.create_publisher(String, "/system_state", 10)
        self._diag_pub = self.create_publisher(String, "/system_diag", 10)

        self.create_subscription(
            Clock,
            "/clock",
            self._on_clock,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TFMessage,
            self._pose_topic,
            self._on_pose_info,
            qos_profile_sensor_data,
        )
        if self._camera_topic:
            self.create_subscription(
                Image,
                self._camera_topic,
                self._on_camera_image,
                qos_profile_sensor_data,
            )
        else:
            if self._camera_required:
                self._fatal("camera_topic vacío; cámaras obligatorias")
                return
            self.get_logger().warn(
                "camera_topic vacío; cámaras deshabilitadas por configuración"
            )

        self._controller_client = self.create_client(
            ListControllers,
            f"{self._controller_manager}/list_controllers",
        )

        period = 1.0 / self._state_hz
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"SystemStateManager listo: pose={self._pose_topic} camera={self._camera_topic} "
            f"ee={self._ee_frame} controllers={len(self._required_controllers)} "
            f"moveit={self._moveit_required}"
        )

    @staticmethod
    def _now() -> float:
        return time.monotonic()

    def _on_clock(self, _msg: Clock) -> None:
        now = self._now()
        self._last_clock_wall = now
        try:
            clock_ns = int(_msg.clock.sec) * 1_000_000_000 + int(_msg.clock.nanosec)
        except Exception:
            clock_ns = None
        if clock_ns is None:
            return
        if self._last_clock_msg_ns is None:
            self._last_clock_msg_ns = clock_ns
            return
        if clock_ns > self._last_clock_msg_ns:
            self._last_clock_advance_wall = now
        self._last_clock_msg_ns = clock_ns

    def _on_pose_info(self, msg: TFMessage) -> None:
        now = self._now()
        if getattr(msg, "transforms", []):
            self._pose_entities_seen = True
        found = False
        for tf in getattr(msg, "transforms", []):
            name = self._name_from_tf(tf)
            if not name:
                continue
            if name == self._model_name or name.startswith(f"{self._model_name}::"):
                found = True
                break
            if self._model_name and self._model_name in name:
                found = True
                break
        if found:
            self._pose_model_seen = True
        if getattr(msg, "transforms", []):
            self._pose_last_wall = now

    def _on_camera_image(self, _msg: Image) -> None:
        now = self._now()
        self._camera_last_wall = now
        self._camera_frames += 1
        stamp = getattr(_msg, "header", None)
        if stamp is None:
            return
        try:
            stamp_ns = int(stamp.stamp.sec) * 1_000_000_000 + int(stamp.stamp.nanosec)
        except Exception:
            return
        if stamp_ns <= 0:
            return
        if self._camera_last_stamp_ns is None or stamp_ns > self._camera_last_stamp_ns:
            self._camera_last_advance_wall = now
        self._camera_last_stamp_ns = stamp_ns

    def _name_from_tf(self, tf: TransformStamped) -> str:
        child = getattr(tf, "child_frame_id", "") or ""
        if child:
            return child
        header = getattr(tf, "header", None)
        return getattr(header, "frame_id", "") if header else ""

    def _clock_ok(self) -> Tuple[bool, float]:
        if self._last_clock_wall <= 0.0:
            return False, float("inf")
        age = self._now() - self._last_clock_wall
        if age > self._clock_timeout:
            return False, age
        if self._last_clock_advance_wall <= 0.0:
            return False, float("inf")
        advance_age = self._now() - self._last_clock_advance_wall
        return advance_age <= self._clock_timeout, advance_age

    def _pose_ok(self) -> Tuple[bool, float]:
        if self._pose_last_wall <= 0.0:
            return False, float("inf")
        age = self._now() - self._pose_last_wall
        ok = (
            self._pose_model_seen
            and self._pose_entities_seen
            and age <= self._pose_timeout
        )
        return ok, age

    def _camera_ok(self) -> Tuple[bool, float]:
        if not self._camera_required:
            return True, 0.0
        if self._camera_last_wall <= 0.0:
            return False, float("inf")
        if self._camera_last_advance_wall <= 0.0:
            return False, float("inf")
        age = self._now() - self._camera_last_advance_wall
        return age <= self._camera_timeout, age

    @staticmethod
    def _normalize_controller_name(name: str) -> str:
        raw = str(name or "").strip().lstrip("/")
        return raw

    def _state_for_required_controller(self, required_name: str) -> str:
        target = self._normalize_controller_name(required_name)
        if not target:
            return ""
        for name, state in self._controllers_state.items():
            norm = self._normalize_controller_name(name)
            if norm == target or norm.endswith("/" + target):
                return str(state or "")
        return ""

    def _evaluate_controllers_status(self) -> Tuple[bool, List[str], str]:
        if not self._required_controllers:
            return True, [], "sin controladores requeridos"
        missing: List[str] = []
        not_active: List[str] = []
        for required in self._required_controllers:
            state = self._state_for_required_controller(required)
            if not state:
                missing.append(required)
                continue
            if str(state).strip().lower() != "active":
                not_active.append(f"{required}:{state}")
        if missing:
            return False, missing, "controllers missing: " + ", ".join(missing)
        if not_active:
            return False, [], "controllers not active: " + ", ".join(not_active)
        return True, [], "controllers activos"

    def _update_controllers(self) -> None:
        now = self._now()
        if (now - self._last_controller_check) < self._controller_check_sec:
            return
        self._last_controller_check = now
        if not self._controller_client.service_is_ready():
            self._controllers_ready = False
            self._controllers_reason = "controller_manager/list_controllers no disponible"
            if not self._controllers_state:
                self._controllers_state = {}
            return
        if self._controller_future is not None and not self._controller_future.done():
            if (
                self._controller_future_start
                and (now - self._controller_future_start)
                > self._controller_future_timeout
            ):
                self._controller_future = None
                self._controllers_ready = False
                self._controllers_reason = "list_controllers timeout"
                self._controllers_state = {}
            else:
                self._controllers_reason = "list_controllers pending"
            return
        req = ListControllers.Request()
        self._controller_future = self._controller_client.call_async(req)
        self._controller_future_start = now

    def _consume_controllers(self) -> None:
        future = self._controller_future
        if future is None or not future.done():
            return
        self._controller_future = None
        self._controller_future_start = 0.0
        try:
            result = future.result()
        except Exception:
            self._controllers_ready = False
            self._controllers_reason = "list_controllers call failed"
            return
        states: Dict[str, str] = {}
        for ctrl in result.controller:
            states[str(ctrl.name)] = str(ctrl.state)
        self._controllers_state = states
        ready, _missing, reason = self._evaluate_controllers_status()
        self._controllers_ready = ready
        self._controllers_reason = reason

    def _tf_ok(self) -> Tuple[bool, str]:
        timeout = Duration(seconds=self._tf_timeout)
        try:
            if not self._tf_buffer.can_transform(
                self._world_frame, self._base_frame, rclpy.time.Time(), timeout=timeout
            ):
                return False, "world->base"
            tf = self._tf_buffer.lookup_transform(
                self._world_frame, self._base_frame, rclpy.time.Time(), timeout=timeout
            )
            if self._is_identity(tf):
                return False, "world->base identity"
            if self._ee_frame:
                if not self._tf_buffer.can_transform(
                    self._base_frame, self._ee_frame, rclpy.time.Time(), timeout=timeout
                ):
                    return False, "base->ee"
        except Exception:
            return False, "tf_error"
        return True, "ok"

    @staticmethod
    def _is_identity(tf: TransformStamped) -> bool:
        t = tf.transform.translation
        r = tf.transform.rotation
        if abs(t.x) > 1e-4 or abs(t.y) > 1e-4 or abs(t.z) > 1e-4:
            return False
        if (
            abs(r.x) > 1e-4
            or abs(r.y) > 1e-4
            or abs(r.z) > 1e-4
            or abs(r.w - 1.0) > 1e-4
        ):
            return False
        return True

    def _moveit_ok(self) -> bool:
        if not self._moveit_required:
            return True
        now = self._now()
        if (now - self._moveit_last_check) < self._moveit_check_sec:
            return self._moveit_last_ok
        try:
            services = self.get_service_names_and_types()
        except Exception:
            self._moveit_last_check = now
            self._moveit_last_ok = False
            return False
        for name, _types in services:
            if name in self._moveit_services:
                self._moveit_last_check = now
                self._moveit_last_ok = True
                return True
            if name.endswith("/get_planning_scene"):
                self._moveit_last_check = now
                self._moveit_last_ok = True
                return True
        self._moveit_last_check = now
        self._moveit_last_ok = False
        return False

    def _set_state(self, state: SystemState, reason: str) -> None:
        if state.value == self._state and reason == self._reason:
            return
        self._state = state.value
        self._reason = reason
        if state == SystemState.READY:
            self._ever_ready = True
        if state != SystemState.ERROR_FATAL:
            self.get_logger().info(f"STATE {state.value} ({reason})")

    def _fatal(self, reason: str) -> None:
        if self._fatal_latched:
            return
        self._fatal_latched = True
        self._set_state(SystemState.ERROR_FATAL, reason)
        self._publish_state()
        self.get_logger().error(f"ERROR_FATAL: {reason}")
        try:
            self.destroy_node()
        except Exception:
            pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass

    def _publish_state(self) -> None:
        self._state_pub.publish(String(data=self._state))
        diag = {
            "reason": self._reason,
            "state": self._state,
            "clock_age_sec": None,
            "pose_age_sec": None,
            "camera_age_sec": None,
            "controllers_ready": self._controllers_ready,
            "controllers_reason": self._controllers_reason,
            "moveit_ready": self._moveit_ready,
            "pose_model_seen": self._pose_model_seen,
            "pose_entities_seen": self._pose_entities_seen,
            "controllers": self._controllers_state,
            "fatal": self._fatal_latched,
        }
        clock_ok, clock_age = self._clock_ok()
        pose_ok, pose_age = self._pose_ok()
        camera_ok, camera_age = self._camera_ok()
        diag["clock_age_sec"] = None if clock_age == float("inf") else float(clock_age)
        diag["pose_age_sec"] = None if pose_age == float("inf") else float(pose_age)
        diag["camera_age_sec"] = (
            None if camera_age == float("inf") else float(camera_age)
        )
        diag["clock_ok"] = clock_ok
        diag["pose_ok"] = pose_ok
        diag["camera_ok"] = camera_ok
        self._diag_pub.publish(String(data=json.dumps(diag)))

    def _tick(self) -> None:
        if self._fatal_latched:
            return
        self._update_controllers()
        self._consume_controllers()
        snapshot = self._collect_snapshot()
        inputs = SystemInputs(
            snapshot=snapshot,
            startup_expired=self._startup_expired(),
            ever_ready=self._ever_ready,
        )
        decision = self._fsm.decide(inputs)
        if decision.fatal_reason:
            self._fatal(decision.fatal_reason)
            return
        self._set_state(decision.state, decision.reason)
        self._publish_state()

    def _collect_snapshot(self) -> DependencySnapshot:
        clock_ok, clock_age = self._clock_ok()
        pose_ok, pose_age = self._pose_ok()
        camera_ok, camera_age = self._camera_ok()
        tf_ok, tf_reason = self._tf_ok()
        now = self._now()
        if tf_ok:
            self._tf_missing_since = 0.0
        elif self._controllers_ready and self._tf_drop_grace > 0.0:
            if self._tf_missing_since <= 0.0:
                self._tf_missing_since = now
            missing_age = now - self._tf_missing_since
            if missing_age < self._tf_drop_grace:
                # Keep TF as non-critical for brief outages to avoid false ERROR_FATAL.
                tf_reason = f"grace:{tf_reason} age={missing_age:.2f}s"
        self._moveit_ready = self._moveit_ok()
        ready_eval, missing, eval_reason = self._evaluate_controllers_status()
        if self._controller_client.service_is_ready() and self._controllers_state:
            self._controllers_ready = ready_eval
            self._controllers_reason = eval_reason
        if not self._controller_client.service_is_ready():
            missing = []
        return DependencySnapshot(
            clock_ok=clock_ok,
            clock_age=clock_age,
            pose_ok=pose_ok,
            pose_age=pose_age,
            pose_model_seen=self._pose_model_seen,
            pose_entities_seen=self._pose_entities_seen,
            camera_ok=camera_ok,
            camera_age=camera_age,
            tf_ok=tf_ok,
            tf_reason=tf_reason,
            controllers_ready=self._controllers_ready,
            controllers_reason=self._controllers_reason,
            missing_controllers=missing,
            moveit_ready=self._moveit_ready,
        )

    def _startup_expired(self) -> bool:
        if self._startup_timeout <= 0.0:
            return True
        return (self._now() - self._start_wall) >= self._startup_timeout


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemStateManager()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.try_shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
