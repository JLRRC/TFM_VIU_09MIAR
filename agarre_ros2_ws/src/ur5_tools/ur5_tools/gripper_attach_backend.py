#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_backend.py
# Contenido: Codigo de herramientas, bridges y servicios auxiliares del stack UR5.
# Uso breve: Se usa en build con colcon y como nodos/servicios ROS 2 del sistema.
"""Attach backend for gripper topics with physical object motion in Gazebo."""

from __future__ import annotations

from dataclasses import dataclass
from functools import partial
import math
import os
import subprocess
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Empty
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

try:
    from ros_gz_interfaces.msg import Entity as GzEntity
    from ros_gz_interfaces.srv import SetEntityPose
except Exception:  # pragma: no cover - optional at runtime
    GzEntity = None
    SetEntityPose = None


DEFAULT_OBJECTS = [
    "pick_demo",
    "box_blue",
    "box_green",
    "box_lightblue",
    "box_red",
    "box_yellow",
    "cross_cyan",
    "cyl_gray",
    "cyl_green",
    "cyl_orange",
    "cyl_purple",
]


@dataclass
class PoseSample:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    stamp_ns: int


@dataclass
class AttachedTarget:
    name: str
    offset_x: float
    offset_y: float
    offset_z: float
    qx: float
    qy: float
    qz: float
    qw: float
    attach_stamp_ns: int


class GripperAttachBackend(Node):
    """Backend that keeps attached objects physically following the gripper TCP."""

    def __init__(self) -> None:
        super().__init__("gripper_attach_backend")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("gripper_prefix", "/gripper")
        self.declare_parameter("drop_anchor_prefix", "/drop_anchor")
        self.declare_parameter("object_names", DEFAULT_OBJECTS)
        self.declare_parameter("attach_mode", "follow_tcp")
        self.declare_parameter("world_name", "ur5_mesa_objetos")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("tcp_frame", "rg2_tcp")
        self.declare_parameter("set_pose_service", "")
        self.declare_parameter("follow_rate_hz", 20.0)
        self.declare_parameter("max_pose_age_sec", 1.5)
        self.declare_parameter("service_timeout_sec", 0.2)
        self.declare_parameter("gz_cli_fallback", True)
        self.declare_parameter("gz_service_timeout_ms", 400)
        self.declare_parameter("gz_cmd_timeout_sec", 0.9)
        self.declare_parameter("attach_initial_queue_retries", 4)
        self.declare_parameter("attach_retry_sleep_sec", 0.06)
        self.declare_parameter("attach_max_dist_m", 0.15)
        self.declare_parameter("ws_dir", "")

        self._gripper_prefix = str(
            self.get_parameter("gripper_prefix").value or "/gripper"
        ).strip("/")
        self._drop_anchor_prefix = str(
            self.get_parameter("drop_anchor_prefix").value or "/drop_anchor"
        ).strip("/")
        names_raw = self.get_parameter("object_names").value
        object_names = [str(v).strip() for v in (names_raw or []) if str(v).strip()]
        self._object_names = sorted(set(object_names))
        self._attach_mode = str(
            self.get_parameter("attach_mode").value or "follow_tcp"
        ).strip()
        self._world_name = str(
            self.get_parameter("world_name").value or "ur5_mesa_objetos"
        ).strip()
        self._world_frame = str(
            self.get_parameter("world_frame").value or "world"
        ).strip() or "world"
        self._tcp_frame = str(self.get_parameter("tcp_frame").value or "rg2_tcp").strip()
        self._pose_topic = str(self.get_parameter("pose_topic").value or "").strip()
        if not self._pose_topic:
            self._pose_topic = f"/world/{self._world_name}/pose/info"
        self._set_pose_service = str(
            self.get_parameter("set_pose_service").value or ""
        ).strip()
        self._follow_rate_hz = max(
            2.0, float(self.get_parameter("follow_rate_hz").value or 20.0)
        )
        self._max_pose_age_sec = max(
            0.05, float(self.get_parameter("max_pose_age_sec").value or 0.6)
        )
        self._service_timeout_sec = max(
            0.05, float(self.get_parameter("service_timeout_sec").value or 0.2)
        )
        self._gz_cli_fallback = bool(self.get_parameter("gz_cli_fallback").value)
        self._gz_service_timeout_ms = max(
            100, int(self.get_parameter("gz_service_timeout_ms").value or 400)
        )
        self._gz_cmd_timeout_sec = max(
            0.3, float(self.get_parameter("gz_cmd_timeout_sec").value or 2.0)
        )
        self._attach_initial_queue_retries = max(
            1, int(self.get_parameter("attach_initial_queue_retries").value or 4)
        )
        self._attach_retry_sleep_sec = max(
            0.01, float(self.get_parameter("attach_retry_sleep_sec").value or 0.06)
        )
        self._attach_max_dist_m = max(
            0.01, float(self.get_parameter("attach_max_dist_m").value or 0.15)
        )
        self._ws_dir = str(self.get_parameter("ws_dir").value or "").strip()
        if not self._ws_dir:
            self._ws_dir = os.environ.get(
                "WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws")
            )

        self._qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._gripper_state_pubs: Dict[str, object] = {}
        self._drop_detach_pubs: Dict[str, object] = {}
        self._drop_attach_pubs: Dict[str, object] = {}

        self._subs = []
        for name in self._object_names:
            grip_attach = f"/{self._gripper_prefix}/{name}/attach".replace("//", "/")
            grip_detach = f"/{self._gripper_prefix}/{name}/detach".replace("//", "/")
            grip_state = f"/{self._gripper_prefix}/{name}/state".replace("//", "/")
            self._gripper_state_pubs[name] = self.create_publisher(
                Bool, grip_state, self._qos
            )
            self._subs.append(
                self.create_subscription(
                    Empty,
                    grip_attach,
                    partial(self._on_gripper_attach, name=name, src_topic=grip_attach),
                    self._qos,
                )
            )
            self._subs.append(
                self.create_subscription(
                    Empty,
                    grip_detach,
                    partial(self._on_gripper_detach, name=name, src_topic=grip_detach),
                    self._qos,
                )
            )
            drop_attach = (
                f"/{self._drop_anchor_prefix}/{name}/attach".replace("//", "/")
            )
            drop_detach = (
                f"/{self._drop_anchor_prefix}/{name}/detach".replace("//", "/")
            )
            self._drop_attach_pubs[name] = self.create_publisher(
                Empty, drop_attach, self._qos
            )
            self._drop_detach_pubs[name] = self.create_publisher(
                Empty, drop_detach, self._qos
            )

        self._pose_cache: Dict[str, PoseSample] = {}
        self._pose_sub = self.create_subscription(
            TFMessage,
            self._pose_topic,
            self._on_pose_info,
            self._qos,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._attached: Dict[str, AttachedTarget] = {}
        self._set_pose_client = None
        self._set_pose_future = None
        self._set_pose_pending: Optional[Tuple[str, PoseSample]] = None
        self._last_apply_log_ts = 0.0
        self._last_exception_log_ts = 0.0
        self._last_stale_warn_ts = 0.0
        self._gz_set_pose_service: Optional[str] = None

        self._follow_timer = self.create_timer(
            1.0 / self._follow_rate_hz,
            self._follow_attached_objects,
        )

        self.get_logger().info(
            "[ATTACH_BACKEND] ready "
            f"objects={','.join(self._object_names)} "
            f"mode={self._attach_mode} "
            f"gripper_prefix=/{self._gripper_prefix} "
            f"pose_topic={self._pose_topic} tcp_frame={self._tcp_frame}"
        )

    def _publish_state(self, name: str, attached: bool) -> None:
        pub = self._gripper_state_pubs.get(name)
        if pub is None:
            return
        msg = Bool()
        msg.data = bool(attached)
        pub.publish(msg)

    def _lookup_pose(self, key: str) -> Optional[PoseSample]:
        if key in self._pose_cache:
            return self._pose_cache[key]
        alt = key.lstrip("/")
        if alt in self._pose_cache:
            return self._pose_cache[alt]
        scoped = f"ur5_rg2::{alt}"
        if scoped in self._pose_cache:
            return self._pose_cache[scoped]
        suffix = f"::{alt}"
        for name, pose in self._pose_cache.items():
            if name.endswith(suffix):
                return pose
        return None

    def _lookup_tcp_pose(self) -> Optional[PoseSample]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame,
                self._tcp_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
            tr = tf.transform.translation
            rot = tf.transform.rotation
            stamp = tf.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            return PoseSample(
                x=float(tr.x),
                y=float(tr.y),
                z=float(tr.z),
                qx=float(rot.x),
                qy=float(rot.y),
                qz=float(rot.z),
                qw=float(rot.w),
                stamp_ns=stamp_ns,
            )
        except Exception:
            pass
        return self._lookup_pose(self._tcp_frame)

    def _on_pose_info(self, msg: TFMessage) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        for tf in getattr(msg, "transforms", []) or []:
            name = str(getattr(tf, "child_frame_id", "") or "").strip()
            if not name:
                continue
            stamp = getattr(tf, "header", None)
            stamp_msg = getattr(stamp, "stamp", None)
            if stamp_msg is not None:
                stamp_ns = int(stamp_msg.sec) * 1_000_000_000 + int(stamp_msg.nanosec)
            else:
                stamp_ns = now_ns
            tr = tf.transform.translation
            rot = tf.transform.rotation
            sample = PoseSample(
                x=float(tr.x),
                y=float(tr.y),
                z=float(tr.z),
                qx=float(rot.x),
                qy=float(rot.y),
                qz=float(rot.z),
                qw=float(rot.w),
                stamp_ns=stamp_ns,
            )
            self._pose_cache[name] = sample
            if "::" in name:
                model = name.split("::")[0].strip()
                if model:
                    # Keep model alias updated to latest sample; setdefault would freeze stale values.
                    self._pose_cache[model] = sample

    def _pose_age_ok(self, pose: PoseSample) -> bool:
        age = self._pose_age_sec(pose)
        return -0.25 <= age <= self._max_pose_age_sec

    def _pose_age_sec(self, pose: PoseSample) -> float:
        now_ns = int(self.get_clock().now().nanoseconds)
        if pose.stamp_ns <= 0 or now_ns <= 0:
            return float("inf")
        return (now_ns - pose.stamp_ns) / 1_000_000_000.0

    def _resolve_set_pose_service(self) -> Optional[str]:
        if self._set_pose_service:
            return self._set_pose_service
        expected = f"/world/{self._world_name}/set_entity_pose"
        services = self.get_service_names_and_types()
        for name, types in services:
            if name == expected and any("SetEntityPose" in t for t in (types or [])):
                self._set_pose_service = expected
                return self._set_pose_service
        for name, types in services:
            if f"/world/{self._world_name}/" not in name:
                continue
            if any("SetEntityPose" in t for t in (types or [])):
                self._set_pose_service = name
                return self._set_pose_service
        return None

    def _ensure_set_pose_client(self) -> bool:
        if SetEntityPose is None:
            return False
        # Re-resolve dynamically because Gazebo services can appear after this node starts.
        if not self._set_pose_service:
            resolved = self._resolve_set_pose_service()
            if resolved:
                self._set_pose_service = resolved
                self.get_logger().info(
                    f"[ATTACH_BACKEND] set_pose_service resolved={resolved}"
                )
        if not self._set_pose_service:
            return False
        if (
            self._set_pose_client is None
            or getattr(self._set_pose_client, "srv_name", "") != self._set_pose_service
        ):
            self._set_pose_client = self.create_client(
                SetEntityPose, self._set_pose_service
            )
        return self._set_pose_client.wait_for_service(timeout_sec=self._service_timeout_sec)

    def _gz_env_prefix(self) -> str:
        exports: List[str] = []
        gz_partition = os.environ.get("GZ_PARTITION", "").strip()
        if not gz_partition:
            part_file = os.path.join(self._ws_dir, "log", "gz_partition.txt")
            try:
                with open(part_file, "r", encoding="utf-8") as f:
                    gz_partition = f.read().strip()
            except Exception:
                gz_partition = ""
        if gz_partition:
            exports.append(f"export GZ_PARTITION='{gz_partition}'")
        gz_ip = os.environ.get("GZ_IP", "").strip()
        if gz_ip:
            exports.append(f"export GZ_IP='{gz_ip}'")
        if not exports:
            return ""
        return " ; ".join(exports) + " ; "

    def _set_pose_via_gz_cli(self, name: str, pose: PoseSample) -> Tuple[bool, str]:
        req = (
            f'name: "{name}" '
            f"position {{x: {float(pose.x)} y: {float(pose.y)} z: {float(pose.z)}}} "
            f"orientation {{x: {float(pose.qx)} y: {float(pose.qy)} z: {float(pose.qz)} w: {float(pose.qw)}}}"
        )
        env_prefix = self._gz_env_prefix()
        world_scope = f"/world/{self._world_name}"
        candidates: List[str] = []
        if self._gz_set_pose_service:
            candidates.append(self._gz_set_pose_service)
        candidates.extend(
            [
                f"{world_scope}/set_pose",
                f"{world_scope}/set_pose/blocking",
                f"{world_scope}/set_entity_pose",
            ]
        )
        dedup: List[str] = []
        for svc in candidates:
            if svc and svc not in dedup:
                dedup.append(svc)
        last_detail = "gz_set_pose_service_unavailable"
        for svc in dedup:
            cmd = (
                f"{env_prefix}gz service -s {svc} "
                "--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean "
                f"--timeout {int(self._gz_service_timeout_ms)} --req '{req}'"
            )
            try:
                res = subprocess.run(
                    ["bash", "-lc", cmd],
                    text=True,
                    capture_output=True,
                    timeout=self._gz_cmd_timeout_sec,
                )
            except Exception as exc:
                last_detail = f"gz_service_exception:{exc}"
                continue
            if res.returncode != 0:
                err = (res.stderr or "").strip()
                last_detail = f"gz_service_rc={res.returncode}:{err or 'n/a'}"
                continue
            out = (res.stdout or "").strip()
            ok = ("data: true" in out.lower()) or ("true" == out.lower())
            if not ok:
                err = (res.stderr or "").strip()
                last_detail = f"gz_service_false:{err or out or 'empty'}"
                continue
            if self._gz_set_pose_service != svc:
                self._gz_set_pose_service = svc
                self.get_logger().info(
                    f"[ATTACH_BACKEND] gz_set_pose_service resolved={svc}"
                )
            return True, "gz_service_ok"
        return False, last_detail

    def _queue_set_pose(self, name: str, pose: PoseSample) -> bool:
        if self._ensure_set_pose_client():
            if self._set_pose_future is not None and not self._set_pose_future.done():
                return False
            req = SetEntityPose.Request()
            req.entity.name = name
            req.entity.type = GzEntity.MODEL if GzEntity is not None else 2
            req.pose.position.x = float(pose.x)
            req.pose.position.y = float(pose.y)
            req.pose.position.z = float(pose.z)
            req.pose.orientation.x = float(pose.qx)
            req.pose.orientation.y = float(pose.qy)
            req.pose.orientation.z = float(pose.qz)
            req.pose.orientation.w = float(pose.qw)
            self._set_pose_pending = (name, pose)
            self._set_pose_future = self._set_pose_client.call_async(req)
            self._set_pose_future.add_done_callback(self._on_set_pose_done)
            return True
        ok, detail = self._set_pose_via_gz_cli(name, pose)
        now = time.time()
        if (now - self._last_apply_log_ts) >= 0.8:
            self.get_logger().info(
                f"[ATTACH_BACKEND] gazebo_attach_applied={str(ok).lower()} "
                f"object={name} method={self._attach_mode} detail={detail}"
            )
            self._last_apply_log_ts = now
        return ok

    def _queue_set_pose_with_retry(self, name: str, pose: PoseSample) -> bool:
        attempts = max(1, int(self._attach_initial_queue_retries))
        for idx in range(attempts):
            if self._queue_set_pose(name, pose):
                return True
            if idx + 1 < attempts:
                time.sleep(self._attach_retry_sleep_sec)
        return False

    def _on_set_pose_done(self, future) -> None:
        pending = self._set_pose_pending
        self._set_pose_pending = None
        self._set_pose_future = None
        if pending is None:
            return
        name, _pose = pending
        ok = False
        detail = "unknown"
        try:
            result = future.result()
            ok = bool(getattr(result, "success", False))
            detail = str(getattr(result, "status_message", "") or "n/a")
        except Exception as exc:
            detail = f"exception:{exc}"
            now = time.time()
            if (now - self._last_exception_log_ts) >= 1.0:
                self.get_logger().exception(
                    f"[ATTACH_BACKEND] [BRIDGE][EXCEPTION] object={name} err={exc}"
                )
                self._last_exception_log_ts = now
        now = time.time()
        if (now - self._last_apply_log_ts) >= 0.8:
            self.get_logger().info(
                f"[ATTACH_BACKEND] gazebo_attach_applied={str(ok).lower()} "
                f"object={name} method={self._attach_mode} detail={detail}"
            )
            self._last_apply_log_ts = now

    def _follow_attached_objects(self) -> None:
        if self._attach_mode != "follow_tcp":
            return
        if not self._attached:
            return
        tcp_pose = self._lookup_tcp_pose()
        if tcp_pose is None:
            return
        tcp_age = self._pose_age_sec(tcp_pose)
        hard_age = max(3.0, self._max_pose_age_sec * 3.0)
        if tcp_age > hard_age:
            return
        if tcp_age > self._max_pose_age_sec:
            now = time.time()
            if (now - self._last_stale_warn_ts) >= 0.7:
                self.get_logger().warning(
                    f"[ATTACH_BACKEND] stale_tcp_pose_but_following age={tcp_age:.3f}s "
                    f"max={self._max_pose_age_sec:.3f}s hard={hard_age:.3f}s"
                )
                self._last_stale_warn_ts = now
        for name, target in list(self._attached.items()):
            desired = PoseSample(
                x=float(tcp_pose.x + target.offset_x),
                y=float(tcp_pose.y + target.offset_y),
                z=float(tcp_pose.z + target.offset_z),
                qx=float(target.qx),
                qy=float(target.qy),
                qz=float(target.qz),
                qw=float(target.qw),
                stamp_ns=int(tcp_pose.stamp_ns),
            )
            if self._queue_set_pose(name, desired):
                break

    def _on_gripper_attach(self, _msg: Empty, *, name: str, src_topic: str) -> None:
        self.get_logger().info(
            f"[ATTACH_BACKEND] attach_request_received object={name} src={src_topic} mode={self._attach_mode}"
        )
        if self._attach_mode != "follow_tcp":
            pub = self._drop_attach_pubs.get(name)
            if pub is None:
                self.get_logger().error(
                    f"[ATTACH_BACKEND] missing drop_attach publisher object={name}"
                )
                self._publish_state(name, False)
                return
            pub.publish(Empty())
            self.get_logger().info(
                f"[ATTACH_BACKEND] relay attach object={name} "
                f"dst=/{self._drop_anchor_prefix}/{name}/attach method=drop_anchor_relay"
            )
            self._publish_state(name, True)
            return

        # Ensure the object is not still constrained to world drop anchor.
        drop_detach_pub = self._drop_detach_pubs.get(name)
        if drop_detach_pub is not None:
            drop_detach_pub.publish(Empty())
            self.get_logger().info(
                f"[ATTACH_BACKEND] relay detach object={name} "
                f"dst=/{self._drop_anchor_prefix}/{name}/detach method=drop_anchor_release"
            )
            time.sleep(0.05)
        else:
            self.get_logger().warning(
                f"[ATTACH_BACKEND] drop_anchor detach publisher missing object={name}"
            )

        obj_pose = self._lookup_pose(name)
        tcp_pose = self._lookup_tcp_pose()
        if obj_pose is None or tcp_pose is None:
            self.get_logger().error(
                "[ATTACH_BACKEND] gazebo_attach_applied=false "
                f"object={name} method=follow_tcp detail=missing_pose "
                f"obj_pose={str(obj_pose is not None).lower()} tcp_pose={str(tcp_pose is not None).lower()}"
            )
            self._publish_state(name, False)
            return
        obj_age = self._pose_age_sec(obj_pose)
        tcp_age = self._pose_age_sec(tcp_pose)
        hard_age = max(3.0, self._max_pose_age_sec * 3.0)
        if obj_age > hard_age or tcp_age > hard_age:
            self.get_logger().error(
                "[ATTACH_BACKEND] gazebo_attach_applied=false "
                f"object={name} method=follow_tcp detail=stale_pose_hard "
                f"obj_age={obj_age:.3f}s tcp_age={tcp_age:.3f}s hard={hard_age:.3f}s"
            )
            self._publish_state(name, False)
            return
        if obj_age > self._max_pose_age_sec or tcp_age > self._max_pose_age_sec:
            self.get_logger().warning(
                "[ATTACH_BACKEND] stale_pose_soft_accept "
                f"object={name} obj_age={obj_age:.3f}s tcp_age={tcp_age:.3f}s "
                f"max={self._max_pose_age_sec:.3f}s"
            )
        if (not self._ensure_set_pose_client()) and (not self._gz_cli_fallback):
            self.get_logger().error(
                "[ATTACH_BACKEND] gazebo_attach_applied=false "
                f"object={name} method=follow_tcp detail=set_pose_backend_unavailable"
            )
            self._publish_state(name, False)
            return
        # FASE 7: Distance gate — reject attach if TCP is too far from object.
        attach_dist = math.sqrt(
            (obj_pose.x - tcp_pose.x) ** 2
            + (obj_pose.y - tcp_pose.y) ** 2
            + (obj_pose.z - tcp_pose.z) ** 2
        )
        if attach_dist > self._attach_max_dist_m:
            self.get_logger().error(
                "[ATTACH_BACKEND] gazebo_attach_applied=false "
                f"object={name} method=follow_tcp detail=distance_too_large "
                f"dist={attach_dist:.4f}m max={self._attach_max_dist_m:.4f}m "
                f"tcp=({tcp_pose.x:.3f},{tcp_pose.y:.3f},{tcp_pose.z:.3f}) "
                f"obj=({obj_pose.x:.3f},{obj_pose.y:.3f},{obj_pose.z:.3f})"
            )
            self._publish_state(name, False)
            return
        attached = AttachedTarget(
            name=name,
            offset_x=float(obj_pose.x - tcp_pose.x),
            offset_y=float(obj_pose.y - tcp_pose.y),
            offset_z=float(obj_pose.z - tcp_pose.z),
            qx=float(obj_pose.qx),
            qy=float(obj_pose.qy),
            qz=float(obj_pose.qz),
            qw=float(obj_pose.qw),
            attach_stamp_ns=int(self.get_clock().now().nanoseconds),
        )
        self._attached[name] = attached
        self._publish_state(name, True)
        self.get_logger().info(
            f"[ATTACH_BACKEND] gazebo_attach_applied=true object={name} method=follow_tcp "
            f"dist={attach_dist:.4f}m max={self._attach_max_dist_m:.4f}m "
            f"offset=({attached.offset_x:.3f},{attached.offset_y:.3f},{attached.offset_z:.3f})"
        )
        desired = PoseSample(
            x=float(tcp_pose.x + attached.offset_x),
            y=float(tcp_pose.y + attached.offset_y),
            z=float(tcp_pose.z + attached.offset_z),
            qx=float(attached.qx),
            qy=float(attached.qy),
            qz=float(attached.qz),
            qw=float(attached.qw),
            stamp_ns=int(tcp_pose.stamp_ns),
        )
        if not self._queue_set_pose_with_retry(name, desired):
            self.get_logger().warning(
                "[ATTACH_BACKEND] initial_set_pose_not_queued "
                f"object={name} method=follow_tcp "
                f"retries={self._attach_initial_queue_retries} "
                "detail=will_retry_on_follow_timer"
            )

    def _on_gripper_detach(self, _msg: Empty, *, name: str, src_topic: str) -> None:
        self.get_logger().info(
            f"[ATTACH_BACKEND] detach_request_received object={name} src={src_topic} mode={self._attach_mode}"
        )
        if self._attach_mode != "follow_tcp":
            pub = self._drop_detach_pubs.get(name)
            if pub is None:
                self.get_logger().error(
                    f"[ATTACH_BACKEND] missing drop_detach publisher object={name}"
                )
                self._publish_state(name, False)
                return
            pub.publish(Empty())
            self.get_logger().info(
                f"[ATTACH_BACKEND] relay detach object={name} "
                f"dst=/{self._drop_anchor_prefix}/{name}/detach method=drop_anchor_relay"
            )
            self._publish_state(name, False)
            return
        self._attached.pop(name, None)
        self._publish_state(name, False)
        self.get_logger().info(
            f"[ATTACH_BACKEND] gazebo_detach_applied=true object={name} method=follow_tcp"
        )


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GripperAttachBackend()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("[ATTACH_BACKEND] stopped")
    finally:
        try:
            node.destroy_node()
        except BaseException:
            pass
        try:
            rclpy.try_shutdown()
        except BaseException:
            pass


if __name__ == "__main__":
    main()
