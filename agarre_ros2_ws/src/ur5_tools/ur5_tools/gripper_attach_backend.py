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
    coherence_breach_count: int = 0


def _quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    inv = 1.0 / norm
    return (x * inv, y * inv, z * inv, w * inv)


def _quat_inverse(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = _quat_normalize(q)
    return (-x, -y, -z, w)


def _quat_multiply_raw(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        (aw * bx) + (ax * bw) + (ay * bz) - (az * by),
        (aw * by) - (ax * bz) + (ay * bw) + (az * bx),
        (aw * bz) + (ax * by) - (ay * bx) + (az * bw),
        (aw * bw) - (ax * bx) - (ay * by) - (az * bz),
    )


def _quat_multiply(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    return _quat_normalize(
        _quat_multiply_raw(a, b)
    )


def _rotate_vector(
    q: Tuple[float, float, float, float],
    v: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    qn = _quat_normalize(q)
    vx, vy, vz = v
    vec_q = (vx, vy, vz, 0.0)
    rx, ry, rz, _rw = _quat_multiply_raw(
        _quat_multiply_raw(qn, vec_q),
        _quat_inverse(qn),
    )
    return (rx, ry, rz)


class GripperAttachBackend(Node):
    """Backend that keeps attached objects physically following the gripper TCP."""

    def __init__(self) -> None:
        super().__init__("gripper_attach_backend")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("gripper_prefix", "/gripper")
        self.declare_parameter("drop_anchor_prefix", "/drop_anchor")
        self.declare_parameter("tool_anchor_prefix", "/gripper_anchor")
        self.declare_parameter("object_names", DEFAULT_OBJECTS)
        self.declare_parameter("attach_mode", "detachable_joint")
        self.declare_parameter("world_name", "ur5_mesa_objetos")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("tcp_frame", "rg2_tcp")
        self.declare_parameter("set_pose_service", "")
        self.declare_parameter("follow_rate_hz", 20.0)
        self.declare_parameter("max_pose_age_sec", 1.5)
        self.declare_parameter("service_timeout_sec", 0.2)
        self.declare_parameter("set_pose_future_timeout_sec", 1.0)
        self.declare_parameter("gz_cli_fallback", True)
        self.declare_parameter("gz_service_timeout_ms", 400)
        self.declare_parameter("gz_cmd_timeout_sec", 0.9)
        self.declare_parameter("attach_initial_queue_retries", 4)
        self.declare_parameter("attach_retry_sleep_sec", 0.06)
        self.declare_parameter("attach_max_dist_m", 0.15)
        self.declare_parameter("follow_break_dist_m", 0.18)
        self.declare_parameter("follow_break_consecutive", 3)
        self.declare_parameter("ws_dir", "")
        self.declare_parameter("startup_detach_tool_anchors", True)
        self.declare_parameter("startup_detach_max_attempts", 12)
        self.declare_parameter("startup_detach_period_sec", 0.5)
        self.declare_parameter("detachable_shadow_follow", True)

        self._gripper_prefix = str(
            self.get_parameter("gripper_prefix").value or "/gripper"
        ).strip("/")
        self._drop_anchor_prefix = str(
            self.get_parameter("drop_anchor_prefix").value or "/drop_anchor"
        ).strip("/")
        self._tool_anchor_prefix = str(
            self.get_parameter("tool_anchor_prefix").value or "/gripper_anchor"
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
        self._base_frame = str(
            self.get_parameter("base_frame").value or "base_link"
        ).strip() or "base_link"
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
        self._set_pose_future_timeout_sec = max(
            0.2, float(self.get_parameter("set_pose_future_timeout_sec").value or 1.0)
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
        self._follow_break_dist_m = max(
            self._attach_max_dist_m,
            float(self.get_parameter("follow_break_dist_m").value or 0.18),
        )
        self._follow_break_consecutive = max(
            1, int(self.get_parameter("follow_break_consecutive").value or 3)
        )
        self._ws_dir = str(self.get_parameter("ws_dir").value or "").strip()
        if not self._ws_dir:
            self._ws_dir = os.environ.get(
                "WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws")
            )
        self._startup_detach_tool_anchors = bool(
            self.get_parameter("startup_detach_tool_anchors").value
        )
        self._startup_detach_max_attempts = max(
            1, int(self.get_parameter("startup_detach_max_attempts").value or 12)
        )
        self._startup_detach_period_sec = max(
            0.1, float(self.get_parameter("startup_detach_period_sec").value or 0.5)
        )
        self._detachable_shadow_follow = bool(
            self.get_parameter("detachable_shadow_follow").value
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
        self._tool_detach_pubs: Dict[str, object] = {}
        self._tool_attach_pubs: Dict[str, object] = {}
        self._drop_anchor_states: Dict[str, bool] = {}

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
            tool_attach = (
                f"/{self._tool_anchor_prefix}/{name}/attach".replace("//", "/")
            )
            tool_detach = (
                f"/{self._tool_anchor_prefix}/{name}/detach".replace("//", "/")
            )
            self._tool_attach_pubs[name] = self.create_publisher(
                Empty, tool_attach, self._qos
            )
            self._tool_detach_pubs[name] = self.create_publisher(
                Empty, tool_detach, self._qos
            )
            drop_state = f"/{self._drop_anchor_prefix}/{name}/state".replace("//", "/")
            self._subs.append(
                self.create_subscription(
                    Bool,
                    drop_state,
                    partial(self._on_drop_anchor_state, name=name),
                    self._qos,
                )
            )

        self._pose_cache: Dict[str, PoseSample] = {}
        self._pose_sub = self.create_subscription(
            TFMessage,
            self._pose_topic,
            self._on_pose_info,
            self._qos,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        # The follow timer can block on Gazebo pose updates; keep TF subscriptions on
        # a dedicated listener thread so the buffer does not stale during lifts.
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._attached: Dict[str, AttachedTarget] = {}
        self._set_pose_client = None
        self._set_pose_future = None
        self._set_pose_pending: Optional[Tuple[str, PoseSample]] = None
        self._set_pose_future_start_ts = 0.0
        self._last_apply_log_ts = 0.0
        self._last_exception_log_ts = 0.0
        self._last_stale_warn_ts = 0.0
        self._last_coherence_warn_ts = 0.0
        self._last_set_pose_timeout_log_ts = 0.0
        self._gz_set_pose_service: Optional[str] = None
        self._last_tcp_pose_source = "none"
        self._last_tcp_pose_diag: Dict[str, float | str | bool] = {}
        self._startup_detach_attempts_left = int(self._startup_detach_max_attempts)
        self._startup_detach_sent = 0

        self._follow_timer = self.create_timer(
            1.0 / self._follow_rate_hz,
            self._follow_attached_objects,
        )
        self._startup_detach_timer = None
        if self._startup_detach_tool_anchors and self._tool_detach_pubs:
            self._startup_detach_timer = self.create_timer(
                self._startup_detach_period_sec,
                self._startup_detach_tool_anchors_once_ready,
            )

        self.get_logger().info(
            "[ATTACH_BACKEND] ready "
            f"objects={','.join(self._object_names)} "
            f"mode={self._attach_mode} "
            f"gripper_prefix=/{self._gripper_prefix} "
            f"tool_anchor_prefix=/{self._tool_anchor_prefix} "
            f"pose_topic={self._pose_topic} base_frame={self._base_frame} tcp_frame={self._tcp_frame}"
        )

    def _publish_state(self, name: str, attached: bool) -> None:
        pub = self._gripper_state_pubs.get(name)
        if pub is None:
            return
        msg = Bool()
        msg.data = bool(attached)
        pub.publish(msg)

    def _on_drop_anchor_state(self, msg: Bool, *, name: str) -> None:
        self._drop_anchor_states[name] = bool(getattr(msg, "data", False))

    def _force_drop_anchor_detach(self, name: str) -> None:
        pub = self._drop_detach_pubs.get(name)
        if pub is None:
            self.get_logger().warning(
                f"[ATTACH_BACKEND] drop_anchor detach publisher missing object={name}"
            )
            return
        state_hint = bool(self._drop_anchor_states.get(name, False))
        # The drop_anchor state topic can report false while the joint is still
        # constraining the object on the table. Publish detach proactively.
        for _idx in range(2):
            pub.publish(Empty())
            time.sleep(0.05)
        self.get_logger().info(
            f"[ATTACH_BACKEND] relay detach object={name} "
            f"dst=/{self._drop_anchor_prefix}/{name}/detach "
            "method=drop_anchor_force_release "
            f"state_hint={str(state_hint).lower()}"
        )

    def _startup_detach_tool_anchors_once_ready(self) -> None:
        timer = self._startup_detach_timer
        if timer is None:
            return
        if self._startup_detach_attempts_left <= 0:
            timer.cancel()
            self._startup_detach_timer = None
            if self._startup_detach_sent <= 0:
                self.get_logger().warning(
                    "[ATTACH_BACKEND] startup_tool_detach skipped "
                    "detail=no_tool_anchor_subscribers"
                )
            return

        self._startup_detach_attempts_left -= 1
        ready = []
        for name, pub in self._tool_detach_pubs.items():
            try:
                sub_count = int(pub.get_subscription_count())
            except Exception:
                sub_count = 0
            if sub_count > 0:
                ready.append((name, pub))
        if not ready:
            return

        detach_sent = 0
        for name, pub in ready:
            pub.publish(Empty())
            self._publish_state(name, False)
            detach_sent += 1
        self._startup_detach_sent += detach_sent
        self.get_logger().info(
            "[ATTACH_BACKEND] startup_tool_detach "
            f"sent={detach_sent} ready={len(ready)}/{len(self._tool_detach_pubs)} "
            f"attempts_left={self._startup_detach_attempts_left}"
        )
        if len(ready) >= len(self._tool_detach_pubs):
            timer.cancel()
            self._startup_detach_timer = None

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

    def _lookup_tf_pose(
        self,
        parent_frame: str,
        child_frame: str,
        *,
        timeout_sec: float = 0.05,
    ) -> Optional[PoseSample]:
        try:
            tf = self._tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
                Time(),
                timeout=Duration(seconds=timeout_sec),
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
            return None

    def _compose_pose(
        self,
        parent_pose: PoseSample,
        child_pose: PoseSample,
    ) -> PoseSample:
        parent_q = _quat_normalize(
            (
                float(parent_pose.qx),
                float(parent_pose.qy),
                float(parent_pose.qz),
                float(parent_pose.qw),
            )
        )
        rel_q = _quat_normalize(
            (
                float(child_pose.qx),
                float(child_pose.qy),
                float(child_pose.qz),
                float(child_pose.qw),
            )
        )
        rel_pos_world = _rotate_vector(
            parent_q,
            (float(child_pose.x), float(child_pose.y), float(child_pose.z)),
        )
        world_q = _quat_multiply(parent_q, rel_q)
        return PoseSample(
            x=float(parent_pose.x + rel_pos_world[0]),
            y=float(parent_pose.y + rel_pos_world[1]),
            z=float(parent_pose.z + rel_pos_world[2]),
            qx=float(world_q[0]),
            qy=float(world_q[1]),
            qz=float(world_q[2]),
            qw=float(world_q[3]),
            stamp_ns=max(int(parent_pose.stamp_ns), int(child_pose.stamp_ns)),
        )

    def _lookup_tcp_pose(self) -> Optional[PoseSample]:
        tf_pose = self._lookup_tf_pose(self._world_frame, self._tcp_frame)
        base_chain_pose: Optional[PoseSample] = None
        world_base: Optional[PoseSample] = None
        base_tcp: Optional[PoseSample] = None
        if (
            self._base_frame
            and self._base_frame != self._world_frame
            and self._base_frame != self._tcp_frame
        ):
            world_base = self._lookup_tf_pose(self._world_frame, self._base_frame)
            base_tcp = self._lookup_tf_pose(self._base_frame, self._tcp_frame)
            if world_base is not None and base_tcp is not None:
                try:
                    base_chain_pose = self._compose_pose(world_base, base_tcp)
                except Exception:
                    base_chain_pose = None
        cached_pose = self._lookup_pose(self._tcp_frame)
        named_candidates = [
            ("base_chain", base_chain_pose),
            ("world_tcp", tf_pose),
            ("cache_pose", cached_pose),
        ]
        candidates = [(name, pose) for name, pose in named_candidates if pose is not None]
        diag: Dict[str, float | str | bool] = {
            "world_base_ok": world_base is not None,
            "base_tcp_ok": base_tcp is not None,
        }
        for name, pose in named_candidates:
            diag[f"{name}_ok"] = pose is not None
            diag[f"{name}_age"] = float("inf") if pose is None else float(self._pose_age_sec(pose))
        self._last_tcp_pose_diag = diag
        if not candidates:
            self._last_tcp_pose_source = "none"
            return None
        best_name, best_pose = min(candidates, key=lambda item: self._pose_age_sec(item[1]))
        self._last_tcp_pose_source = str(best_name)
        return best_pose

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
        preferred = [
            f"{world_scope}/set_entity_pose",
            f"{world_scope}/set_pose/blocking",
            f"{world_scope}/set_pose",
        ]
        cached = (self._gz_set_pose_service or "").strip()
        if cached.endswith("/set_entity_pose") or cached.endswith("/set_pose/blocking"):
            candidates.append(cached)
        candidates.extend(preferred)
        if cached and cached not in candidates:
            candidates.append(cached)
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
                age = 0.0
                if self._set_pose_future_start_ts > 0.0:
                    age = max(0.0, time.time() - self._set_pose_future_start_ts)
                if age < self._set_pose_future_timeout_sec:
                    return False
                pending_name = name
                if self._set_pose_pending is not None:
                    pending_name = self._set_pose_pending[0]
                try:
                    self._set_pose_future.cancel()
                except Exception:
                    pass
                self._set_pose_future = None
                self._set_pose_pending = None
                self._set_pose_future_start_ts = 0.0
                now = time.time()
                if (now - self._last_set_pose_timeout_log_ts) >= 0.8:
                    self.get_logger().warning(
                        "[ATTACH_BACKEND] set_pose_future_stale "
                        f"object={pending_name} age={age:.3f}s "
                        f"timeout={self._set_pose_future_timeout_sec:.3f}s "
                        "action=cancel_and_retry"
                    )
                    self._last_set_pose_timeout_log_ts = now
                if self._gz_cli_fallback:
                    ok, detail = self._set_pose_via_gz_cli(name, pose)
                    if (now - self._last_apply_log_ts) >= 0.8:
                        self.get_logger().info(
                            f"[ATTACH_BACKEND] gazebo_attach_applied={str(ok).lower()} "
                            f"object={name} method={self._attach_mode} detail={detail}"
                        )
                        self._last_apply_log_ts = now
                    if ok:
                        return True
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
            self._set_pose_future_start_ts = time.time()
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

    def _drop_attached_target(
        self,
        name: str,
        *,
        detail: str,
        dist: Optional[float] = None,
    ) -> None:
        self._attached.pop(name, None)
        if self._attach_mode == "detachable_joint":
            pub = self._tool_detach_pubs.get(name)
            if pub is not None:
                pub.publish(Empty())
        self._publish_state(name, False)
        dist_txt = "n/a" if dist is None else f"{dist:.4f}m"
        self.get_logger().warning(
            "[ATTACH_BACKEND] follow_detached "
            f"object={name} detail={detail} dist={dist_txt} "
            f"max={self._follow_break_dist_m:.4f}m "
            f"breaches_required={self._follow_break_consecutive}"
        )

    def _on_set_pose_done(self, future) -> None:
        pending = self._set_pose_pending
        self._set_pose_pending = None
        self._set_pose_future = None
        self._set_pose_future_start_ts = 0.0
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
        shadow_follow = (
            self._attach_mode == "follow_tcp"
            or (self._attach_mode == "detachable_joint" and self._detachable_shadow_follow)
        )
        if not shadow_follow:
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
        soft_stale = tcp_age > self._max_pose_age_sec
        if tcp_age > self._max_pose_age_sec:
            now = time.time()
            if (now - self._last_stale_warn_ts) >= 0.7:
                diag = dict(self._last_tcp_pose_diag or {})
                diag_txt = (
                    f"src={self._last_tcp_pose_source} "
                    f"base_chain_ok={diag.get('base_chain_ok')} base_chain_age={diag.get('base_chain_age')} "
                    f"world_tcp_ok={diag.get('world_tcp_ok')} world_tcp_age={diag.get('world_tcp_age')} "
                    f"cache_ok={diag.get('cache_pose_ok')} cache_age={diag.get('cache_pose_age')} "
                    f"world_base_ok={diag.get('world_base_ok')} base_tcp_ok={diag.get('base_tcp_ok')}"
                )
                self.get_logger().warning(
                    f"[ATTACH_BACKEND] stale_tcp_pose_soft_follow age={tcp_age:.3f}s "
                    f"max={self._max_pose_age_sec:.3f}s hard={hard_age:.3f}s {diag_txt}"
                )
                self._last_stale_warn_ts = now
        for name, target in list(self._attached.items()):
            obj_pose = self._lookup_pose(name)
            if (not soft_stale) and obj_pose is not None and self._pose_age_ok(obj_pose):
                obj_tcp_dist = math.sqrt(
                    (obj_pose.x - tcp_pose.x) ** 2
                    + (obj_pose.y - tcp_pose.y) ** 2
                    + (obj_pose.z - tcp_pose.z) ** 2
                )
                if obj_tcp_dist > self._follow_break_dist_m:
                    target.coherence_breach_count += 1
                    now = time.time()
                    if (
                        target.coherence_breach_count >= self._follow_break_consecutive
                    ):
                        self._drop_attached_target(
                            name,
                            detail="coherence_lost",
                            dist=obj_tcp_dist,
                        )
                        continue
                    if (now - self._last_coherence_warn_ts) >= 0.5:
                        self.get_logger().warning(
                            "[ATTACH_BACKEND] follow_coherence_warning "
                            f"object={name} dist={obj_tcp_dist:.4f}m "
                            f"max={self._follow_break_dist_m:.4f}m "
                            f"breach={target.coherence_breach_count}/"
                            f"{self._follow_break_consecutive}"
                        )
                        self._last_coherence_warn_ts = now
                else:
                    target.coherence_breach_count = 0
            else:
                target.coherence_breach_count = 0
            desired = PoseSample(
                x=0.0,
                y=0.0,
                z=0.0,
                qx=0.0,
                qy=0.0,
                qz=0.0,
                qw=1.0,
                stamp_ns=int(tcp_pose.stamp_ns),
            )
            tcp_q = _quat_normalize(
                (float(tcp_pose.qx), float(tcp_pose.qy), float(tcp_pose.qz), float(tcp_pose.qw))
            )
            rel_pos_world = _rotate_vector(
                tcp_q,
                (float(target.offset_x), float(target.offset_y), float(target.offset_z)),
            )
            rel_q_world = _quat_multiply(
                tcp_q,
                (float(target.qx), float(target.qy), float(target.qz), float(target.qw)),
            )
            desired.x = float(tcp_pose.x + rel_pos_world[0])
            desired.y = float(tcp_pose.y + rel_pos_world[1])
            desired.z = float(tcp_pose.z + rel_pos_world[2])
            desired.qx = float(rel_q_world[0])
            desired.qy = float(rel_q_world[1])
            desired.qz = float(rel_q_world[2])
            desired.qw = float(rel_q_world[3])
            if self._queue_set_pose(name, desired):
                break

    def _activate_follow_attachment(self, name: str, *, method: str) -> bool:
        obj_pose = self._lookup_pose(name)
        tcp_pose = self._lookup_tcp_pose()
        if obj_pose is None or tcp_pose is None:
            self.get_logger().error(
                "[ATTACH_BACKEND] gazebo_attach_applied=false "
                f"object={name} method={method} detail=missing_pose "
                f"obj_pose={str(obj_pose is not None).lower()} tcp_pose={str(tcp_pose is not None).lower()}"
            )
            self._publish_state(name, False)
            return False
        obj_age = self._pose_age_sec(obj_pose)
        tcp_age = self._pose_age_sec(tcp_pose)
        hard_age = max(3.0, self._max_pose_age_sec * 3.0)
        if obj_age > hard_age or tcp_age > hard_age:
            self.get_logger().error(
                "[ATTACH_BACKEND] gazebo_attach_applied=false "
                f"object={name} method={method} detail=stale_pose_hard "
                f"obj_age={obj_age:.3f}s tcp_age={tcp_age:.3f}s hard={hard_age:.3f}s"
            )
            self._publish_state(name, False)
            return False
        if obj_age > self._max_pose_age_sec or tcp_age > self._max_pose_age_sec:
            self.get_logger().warning(
                "[ATTACH_BACKEND] stale_pose_soft_accept "
                f"object={name} obj_age={obj_age:.3f}s tcp_age={tcp_age:.3f}s "
                f"max={self._max_pose_age_sec:.3f}s"
            )
        if (not self._ensure_set_pose_client()) and (not self._gz_cli_fallback):
            self.get_logger().error(
                "[ATTACH_BACKEND] gazebo_attach_applied=false "
                f"object={name} method={method} detail=set_pose_backend_unavailable"
            )
            self._publish_state(name, False)
            return False
        attach_dist = math.sqrt(
            (obj_pose.x - tcp_pose.x) ** 2
            + (obj_pose.y - tcp_pose.y) ** 2
            + (obj_pose.z - tcp_pose.z) ** 2
        )
        if attach_dist > self._attach_max_dist_m:
            self.get_logger().error(
                "[ATTACH_BACKEND] gazebo_attach_applied=false "
                f"object={name} method={method} detail=distance_too_large "
                f"dist={attach_dist:.4f}m max={self._attach_max_dist_m:.4f}m "
                f"tcp=({tcp_pose.x:.3f},{tcp_pose.y:.3f},{tcp_pose.z:.3f}) "
                f"obj=({obj_pose.x:.3f},{obj_pose.y:.3f},{obj_pose.z:.3f})"
            )
            self._publish_state(name, False)
            return False
        tcp_q = _quat_normalize(
            (float(tcp_pose.qx), float(tcp_pose.qy), float(tcp_pose.qz), float(tcp_pose.qw))
        )
        obj_q = _quat_normalize(
            (float(obj_pose.qx), float(obj_pose.qy), float(obj_pose.qz), float(obj_pose.qw))
        )
        tcp_q_inv = _quat_inverse(tcp_q)
        rel_pos = _rotate_vector(
            tcp_q_inv,
            (
                float(obj_pose.x - tcp_pose.x),
                float(obj_pose.y - tcp_pose.y),
                float(obj_pose.z - tcp_pose.z),
            ),
        )
        rel_q = _quat_multiply(tcp_q_inv, obj_q)
        attached = AttachedTarget(
            name=name,
            offset_x=float(rel_pos[0]),
            offset_y=float(rel_pos[1]),
            offset_z=float(rel_pos[2]),
            qx=float(rel_q[0]),
            qy=float(rel_q[1]),
            qz=float(rel_q[2]),
            qw=float(rel_q[3]),
            attach_stamp_ns=int(self.get_clock().now().nanoseconds),
        )
        self._attached[name] = attached
        self._publish_state(name, True)
        self.get_logger().info(
            f"[ATTACH_BACKEND] gazebo_attach_applied=true object={name} method={method} "
            f"dist={attach_dist:.4f}m max={self._attach_max_dist_m:.4f}m "
            f"rel_pos=({attached.offset_x:.3f},{attached.offset_y:.3f},{attached.offset_z:.3f}) "
            f"rel_q=({attached.qx:.3f},{attached.qy:.3f},{attached.qz:.3f},{attached.qw:.3f})"
        )
        rel_pos_world = _rotate_vector(
            tcp_q,
            (float(attached.offset_x), float(attached.offset_y), float(attached.offset_z)),
        )
        rel_q_world = _quat_multiply(
            tcp_q,
            (float(attached.qx), float(attached.qy), float(attached.qz), float(attached.qw)),
        )
        desired = PoseSample(
            x=float(tcp_pose.x + rel_pos_world[0]),
            y=float(tcp_pose.y + rel_pos_world[1]),
            z=float(tcp_pose.z + rel_pos_world[2]),
            qx=float(rel_q_world[0]),
            qy=float(rel_q_world[1]),
            qz=float(rel_q_world[2]),
            qw=float(rel_q_world[3]),
            stamp_ns=int(tcp_pose.stamp_ns),
        )
        if not self._queue_set_pose_with_retry(name, desired):
            self.get_logger().warning(
                "[ATTACH_BACKEND] initial_set_pose_not_queued "
                f"object={name} method={method} "
                f"retries={self._attach_initial_queue_retries} "
                "detail=will_retry_on_follow_timer"
            )
        return True

    def _on_gripper_attach(self, _msg: Empty, *, name: str, src_topic: str) -> None:
        self.get_logger().info(
            f"[ATTACH_BACKEND] attach_request_received object={name} src={src_topic} mode={self._attach_mode}"
        )
        if self._attach_mode != "follow_tcp":
            self._force_drop_anchor_detach(name)
            pub = self._tool_attach_pubs.get(name)
            if pub is None:
                self.get_logger().error(
                    f"[ATTACH_BACKEND] missing tool_attach publisher object={name}"
                )
                self._publish_state(name, False)
                return
            pub.publish(Empty())
            self.get_logger().info(
                f"[ATTACH_BACKEND] relay attach object={name} "
                f"dst=/{self._tool_anchor_prefix}/{name}/attach method=tool_anchor_relay"
            )
            if self._detachable_shadow_follow:
                self.get_logger().info(
                    f"[ATTACH_BACKEND] tool_anchor relay object={name} "
                    "detail=starting_shadow_follow"
                )
                self._activate_follow_attachment(name, method="detachable_joint_shadow_follow")
            else:
                self._publish_state(name, True)
            return

        # The drop anchor can remain physically attached even if its state topic
        # already reports false. Release it proactively before follow attach.
        self._force_drop_anchor_detach(name)
        self._activate_follow_attachment(name, method="follow_tcp")

    def _on_gripper_detach(self, _msg: Empty, *, name: str, src_topic: str) -> None:
        self.get_logger().info(
            f"[ATTACH_BACKEND] detach_request_received object={name} src={src_topic} mode={self._attach_mode}"
        )
        if self._attach_mode != "follow_tcp":
            self._attached.pop(name, None)
            pub = self._tool_detach_pubs.get(name)
            if pub is None:
                self.get_logger().error(
                    f"[ATTACH_BACKEND] missing tool_detach publisher object={name}"
                )
                self._publish_state(name, False)
                return
            pub.publish(Empty())
            self.get_logger().info(
                f"[ATTACH_BACKEND] relay detach object={name} "
                f"dst=/{self._tool_anchor_prefix}/{name}/detach method=tool_anchor_relay"
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
