#!/usr/bin/env python3
"""Sync Gazebo models into MoveIt PlanningScene."""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
import time
from typing import Dict, List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener


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

DEFAULT_TOUCH_LINKS = [
    "rg2_tcp",
    "tool0",
    "flange",
    "wrist_3_link",
]


@dataclass(frozen=True)
class PrimitiveSpec:
    primitive_type: int
    dimensions: Tuple[float, ...]
    local_pose: Tuple[float, float, float, float, float, float, float]


@dataclass(frozen=True)
class ModelGeometry:
    name: str
    model_pose: Tuple[float, float, float, float, float, float, float]
    primitives: Tuple[PrimitiveSpec, ...]
    is_static: bool


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


def _strip_ns(tag: str) -> str:
    if "}" in tag:
        return tag.split("}", 1)[1]
    return tag


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quat_multiply(
    lhs: Tuple[float, float, float, float],
    rhs: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        (lw * rx) + (lx * rw) + (ly * rz) - (lz * ry),
        (lw * ry) - (lx * rz) + (ly * rw) + (lz * rx),
        (lw * rz) + (lx * ry) - (ly * rx) + (lz * rw),
        (lw * rw) - (lx * rx) - (ly * ry) - (lz * rz),
    )


def _quat_conjugate(quat: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    qx, qy, qz, qw = quat
    return (-qx, -qy, -qz, qw)


def _rotate_vector(
    quat: Tuple[float, float, float, float],
    vector: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    vx, vy, vz = vector
    vec_quat = (vx, vy, vz, 0.0)
    rotated = _quat_multiply(_quat_multiply(quat, vec_quat), _quat_conjugate(quat))
    return (rotated[0], rotated[1], rotated[2])


def _compose_pose(
    parent: Tuple[float, float, float, float, float, float, float],
    child: Tuple[float, float, float, float, float, float, float],
) -> Tuple[float, float, float, float, float, float, float]:
    px, py, pz, pqx, pqy, pqz, pqw = parent
    cx, cy, cz, cqx, cqy, cqz, cqw = child
    ox, oy, oz = _rotate_vector((pqx, pqy, pqz, pqw), (cx, cy, cz))
    qx, qy, qz, qw = _quat_multiply((pqx, pqy, pqz, pqw), (cqx, cqy, cqz, cqw))
    return (px + ox, py + oy, pz + oz, qx, qy, qz, qw)


def _parse_pose_text(text: str) -> Tuple[float, float, float, float, float, float, float]:
    values = [part for part in (text or "").split() if part]
    if len(values) < 6:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    try:
        x, y, z, rr, pp, yy = (float(values[idx]) for idx in range(6))
    except Exception:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    qx, qy, qz, qw = _quat_from_rpy(rr, pp, yy)
    return (x, y, z, qx, qy, qz, qw)


def _tuple_to_pose(values: Tuple[float, float, float, float, float, float, float]) -> Pose:
    pose = Pose()
    pose.position.x = float(values[0])
    pose.position.y = float(values[1])
    pose.position.z = float(values[2])
    pose.orientation.x = float(values[3])
    pose.orientation.y = float(values[4])
    pose.orientation.z = float(values[5])
    pose.orientation.w = float(values[6])
    return pose


def _world_file_default(world_name: str) -> str:
    ws_dir = os.environ.get("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws"))
    return os.path.join(ws_dir, "worlds", f"{world_name}.sdf")


class PlanningSceneSync(Node):
    """Populate PlanningScene with Gazebo table and movable objects."""

    def __init__(self) -> None:
        super().__init__("planning_scene_sync")
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("world_name", "ur5_mesa_objetos")
        self.declare_parameter("world_file", "")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "rg2_tcp")
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("apply_service", "/apply_planning_scene")
        self.declare_parameter("table_model_name", "mesa_pro")
        self.declare_parameter("object_names", DEFAULT_OBJECTS)
        self.declare_parameter("attach_prefix", "/gripper")
        self.declare_parameter("touch_links", DEFAULT_TOUCH_LINKS)
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("max_pose_age_sec", 1.5)
        self.declare_parameter("lookup_timeout_sec", 0.15)

        self._world_name = str(self.get_parameter("world_name").value or "ur5_mesa_objetos").strip()
        self._world_frame = str(self.get_parameter("world_frame").value or "world").strip() or "world"
        self._base_frame = str(self.get_parameter("base_frame").value or "base_link").strip() or "base_link"
        self._ee_frame = str(self.get_parameter("ee_frame").value or "rg2_tcp").strip() or "rg2_tcp"
        self._pose_topic = str(self.get_parameter("pose_topic").value or "").strip()
        if not self._pose_topic:
            self._pose_topic = f"/world/{self._world_name}/pose/info"
        self._apply_service_name = str(self.get_parameter("apply_service").value or "/apply_planning_scene").strip() or "/apply_planning_scene"
        self._table_model_name = str(self.get_parameter("table_model_name").value or "mesa_pro").strip() or "mesa_pro"
        object_values = self.get_parameter("object_names").value or DEFAULT_OBJECTS
        self._object_names = [str(value).strip() for value in object_values if str(value).strip()]
        attach_prefix = str(self.get_parameter("attach_prefix").value or "/gripper").strip()
        self._attach_prefix = attach_prefix.strip("/") or "gripper"
        touch_values = self.get_parameter("touch_links").value or DEFAULT_TOUCH_LINKS
        self._touch_links = [str(value).strip() for value in touch_values if str(value).strip()]
        if self._ee_frame not in self._touch_links:
            self._touch_links.append(self._ee_frame)
        self._publish_period_sec = max(0.1, float(self.get_parameter("publish_period_sec").value or 0.5))
        self._max_pose_age_sec = max(0.05, float(self.get_parameter("max_pose_age_sec").value or 1.5))
        self._lookup_timeout_sec = max(0.02, float(self.get_parameter("lookup_timeout_sec").value or 0.15))
        self._world_file = str(self.get_parameter("world_file").value or "").strip()
        if not self._world_file:
            self._world_file = _world_file_default(self._world_name)

        self._pose_cache: Dict[str, PoseSample] = {}
        self._attached_state: Dict[str, bool] = {name: False for name in self._object_names}
        self._last_signature = ""
        self._last_modes: Dict[str, str] = {}
        self._last_service_log = 0.0
        self._last_attach_warn: Dict[str, float] = {}
        self._apply_future = None

        self._qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pose_sub = self.create_subscription(
            TFMessage,
            self._pose_topic,
            self._on_pose_info,
            self._qos,
        )
        self._attach_subs = []
        for name in self._object_names:
            topic = f"/{self._attach_prefix}/{name}/state".replace("//", "/")
            self._attach_subs.append(
                self.create_subscription(
                    Bool,
                    topic,
                    lambda msg, object_name=name: self._on_attach_state(msg, object_name),
                    self._qos,
                )
            )

        self._tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._apply_client = self.create_client(ApplyPlanningScene, self._apply_service_name)

        self._table_geometry, self._object_geometry = self._load_geometry()
        self._timer = self.create_timer(self._publish_period_sec, self._timer_cb)

        self.get_logger().info(
            "[SCENE_SYNC] ready "
            f"pose_topic={self._pose_topic} apply_service={self._apply_service_name} "
            f"table={self._table_model_name} objects={','.join(self._object_names)}"
        )

    def _load_geometry(self) -> Tuple[Optional[ModelGeometry], Dict[str, ModelGeometry]]:
        if not os.path.isfile(self._world_file):
            self.get_logger().warning(f"[SCENE_SYNC] world file no encontrado: {self._world_file}")
            return None, {}
        try:
            tree = ET.parse(self._world_file)
            root = tree.getroot()
        except Exception as exc:
            self.get_logger().error(f"[SCENE_SYNC] no se pudo parsear world file: {exc}")
            return None, {}

        for elem in root.iter():
            elem.tag = _strip_ns(elem.tag)

        interest = set(self._object_names)
        interest.add(self._table_model_name)
        table_geometry: Optional[ModelGeometry] = None
        object_geometry: Dict[str, ModelGeometry] = {}
        world_elem = root.find("world")
        if world_elem is None:
            return None, {}
        for model in world_elem.findall("model"):
            name = (model.attrib.get("name") or "").strip()
            if name not in interest:
                continue
            geometry = self._parse_model_geometry(model, name)
            if geometry is None:
                continue
            if name == self._table_model_name:
                table_geometry = geometry
            else:
                object_geometry[name] = geometry
        return table_geometry, object_geometry

    def _parse_model_geometry(self, model: ET.Element, name: str) -> Optional[ModelGeometry]:
        model_pose = _parse_pose_text(model.findtext("pose") or "")
        static_text = (model.findtext("static") or "false").strip().lower()
        is_static = static_text in ("1", "true", "yes")
        primitives: List[PrimitiveSpec] = []
        for link in model.findall("link"):
            link_pose = _parse_pose_text(link.findtext("pose") or "")
            for collision in link.findall("collision"):
                geom = collision.find("geometry")
                if geom is None:
                    continue
                collision_pose = _parse_pose_text(collision.findtext("pose") or "")
                local_pose = _compose_pose(link_pose, collision_pose)
                box = geom.find("box")
                cylinder = geom.find("cylinder")
                sphere = geom.find("sphere")
                if box is not None:
                    size_text = box.findtext("size") or ""
                    parts = [part for part in size_text.split() if part]
                    if len(parts) != 3:
                        continue
                    try:
                        dims = tuple(float(part) for part in parts)
                    except Exception:
                        continue
                    primitives.append(
                        PrimitiveSpec(
                            primitive_type=SolidPrimitive.BOX,
                            dimensions=dims,
                            local_pose=local_pose,
                        )
                    )
                    continue
                if cylinder is not None:
                    try:
                        radius = float(cylinder.findtext("radius") or 0.0)
                        length = float(cylinder.findtext("length") or 0.0)
                    except Exception:
                        continue
                    if radius <= 0.0 or length <= 0.0:
                        continue
                    primitives.append(
                        PrimitiveSpec(
                            primitive_type=SolidPrimitive.CYLINDER,
                            dimensions=(length, radius),
                            local_pose=local_pose,
                        )
                    )
                    continue
                if sphere is not None:
                    try:
                        radius = float(sphere.findtext("radius") or 0.0)
                    except Exception:
                        continue
                    if radius <= 0.0:
                        continue
                    primitives.append(
                        PrimitiveSpec(
                            primitive_type=SolidPrimitive.SPHERE,
                            dimensions=(radius,),
                            local_pose=local_pose,
                        )
                    )
        if not primitives:
            self.get_logger().warning(f"[SCENE_SYNC] modelo sin collisions compatibles: {name}")
            return None
        return ModelGeometry(
            name=name,
            model_pose=model_pose,
            primitives=tuple(primitives),
            is_static=is_static,
        )

    def _on_pose_info(self, msg: TFMessage) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        for tf in getattr(msg, "transforms", []) or []:
            child_name = str(getattr(tf, "child_frame_id", "") or "").strip()
            if not child_name:
                continue
            stamp = getattr(getattr(tf, "header", None), "stamp", None)
            if stamp is not None:
                stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            else:
                stamp_ns = now_ns
            sample = PoseSample(
                x=float(tf.transform.translation.x),
                y=float(tf.transform.translation.y),
                z=float(tf.transform.translation.z),
                qx=float(tf.transform.rotation.x),
                qy=float(tf.transform.rotation.y),
                qz=float(tf.transform.rotation.z),
                qw=float(tf.transform.rotation.w),
                stamp_ns=stamp_ns,
            )
            self._pose_cache[child_name] = sample
            if "::" in child_name:
                model_name = child_name.split("::", 1)[0].strip()
                if model_name:
                    self._pose_cache[model_name] = sample

    def _on_attach_state(self, msg: Bool, object_name: str) -> None:
        self._attached_state[object_name] = bool(msg.data)

    def _lookup_pose(self, name: str) -> Optional[PoseSample]:
        if name in self._pose_cache:
            return self._pose_cache[name]
        alt = name.lstrip("/")
        if alt in self._pose_cache:
            return self._pose_cache[alt]
        scoped = f"ur5_rg2::{alt}"
        if scoped in self._pose_cache:
            return self._pose_cache[scoped]
        suffix = f"::{alt}"
        for key, sample in self._pose_cache.items():
            if key.endswith(suffix):
                return sample
        return None

    def _pose_age_ok(self, sample: PoseSample) -> bool:
        now_ns = int(self.get_clock().now().nanoseconds)
        if now_ns <= 0 or sample.stamp_ns <= 0:
            return True
        age_sec = float(now_ns - sample.stamp_ns) / 1_000_000_000.0
        return -0.25 <= age_sec <= self._max_pose_age_sec

    def _collision_from_geometry(
        self,
        geometry: ModelGeometry,
        model_pose: Tuple[float, float, float, float, float, float, float],
        frame_id: str,
        object_id: Optional[str] = None,
    ) -> CollisionObject:
        collision = CollisionObject()
        collision.id = object_id or geometry.name
        collision.header.frame_id = frame_id
        collision.operation = CollisionObject.ADD
        for spec in geometry.primitives:
            primitive = SolidPrimitive()
            primitive.type = spec.primitive_type
            primitive.dimensions = list(spec.dimensions)
            collision.primitives.append(primitive)
            world_pose = _compose_pose(model_pose, spec.local_pose)
            collision.primitive_poses.append(_tuple_to_pose(world_pose))
        return collision

    def _transform_collision(
        self,
        collision: CollisionObject,
        target_frame: str,
    ) -> Optional[CollisionObject]:
        if collision.header.frame_id == target_frame:
            return collision
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame,
                collision.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._lookup_timeout_sec),
            )
        except Exception:
            return None
        transformed = CollisionObject()
        transformed.id = collision.id
        transformed.header.frame_id = target_frame
        transformed.operation = collision.operation
        for primitive, pose in zip(collision.primitives, collision.primitive_poses):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = collision.header.frame_id
            pose_stamped.pose = pose
            pose_target = do_transform_pose_stamped(pose_stamped, tf)
            transformed.primitives.append(primitive)
            transformed.primitive_poses.append(pose_target.pose)
        return transformed

    def _collision_remove(self, object_id: str) -> CollisionObject:
        collision = CollisionObject()
        collision.id = object_id
        collision.header.frame_id = self._world_frame
        collision.operation = CollisionObject.REMOVE
        return collision

    def _attached_remove(self, object_id: str) -> AttachedCollisionObject:
        attached = AttachedCollisionObject()
        attached.link_name = self._ee_frame
        attached.object.id = object_id
        attached.object.operation = CollisionObject.REMOVE
        return attached

    def _attached_add(self, collision_world: CollisionObject) -> Optional[AttachedCollisionObject]:
        collision_ee = self._transform_collision(collision_world, self._ee_frame)
        if collision_ee is None:
            return None
        attached = AttachedCollisionObject()
        attached.link_name = self._ee_frame
        attached.touch_links = list(dict.fromkeys(self._touch_links))
        attached.object = collision_ee
        attached.object.operation = CollisionObject.ADD
        return attached

    def _geometry_pose_tuple(self, sample: PoseSample) -> Tuple[float, float, float, float, float, float, float]:
        return (sample.x, sample.y, sample.z, sample.qx, sample.qy, sample.qz, sample.qw)

    def _warn_attach_fallback(self, name: str) -> None:
        now = time.monotonic()
        last = self._last_attach_warn.get(name, 0.0)
        if (now - last) < 2.0:
            return
        self._last_attach_warn[name] = now
        self.get_logger().warning(
            f"[SCENE_SYNC] attach fallback a world collision para {name}; TF {self._world_frame}->{self._ee_frame} no disponible"
        )

    def _build_scene_request(self) -> Tuple[ApplyPlanningScene.Request, str]:
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        signature_parts: List[str] = []
        next_modes: Dict[str, str] = {}

        if self._table_geometry is not None:
            table_collision = self._collision_from_geometry(
                self._table_geometry,
                self._table_geometry.model_pose,
                self._world_frame,
            )
            scene.world.collision_objects.append(table_collision)
            signature_parts.append(
                f"table:{self._table_geometry.name}:{len(self._table_geometry.primitives)}"
            )

        for name in self._object_names:
            geometry = self._object_geometry.get(name)
            previous_mode = self._last_modes.get(name, "unknown")
            if geometry is None:
                if previous_mode == "world":
                    scene.world.collision_objects.append(self._collision_remove(name))
                elif previous_mode == "attached":
                    scene.robot_state.attached_collision_objects.append(self._attached_remove(name))
                signature_parts.append(f"{name}:nogeom")
                next_modes[name] = "absent"
                continue

            sample = self._lookup_pose(name)
            if sample is None or not self._pose_age_ok(sample):
                if previous_mode == "world":
                    scene.world.collision_objects.append(self._collision_remove(name))
                elif previous_mode == "attached":
                    scene.robot_state.attached_collision_objects.append(self._attached_remove(name))
                signature_parts.append(f"{name}:absent")
                next_modes[name] = "absent"
                continue

            world_pose = self._geometry_pose_tuple(sample)
            collision_world = self._collision_from_geometry(
                geometry,
                world_pose,
                self._world_frame,
                object_id=name,
            )
            attached = bool(self._attached_state.get(name, False))
            if attached:
                attached_object = self._attached_add(collision_world)
                if attached_object is not None:
                    if previous_mode == "world":
                        scene.world.collision_objects.append(self._collision_remove(name))
                    scene.robot_state.attached_collision_objects.append(attached_object)
                    signature_parts.append(
                        "{}:attached:{:.3f}:{:.3f}:{:.3f}".format(
                            name,
                            sample.x,
                            sample.y,
                            sample.z,
                        )
                    )
                    next_modes[name] = "attached"
                    continue
                self._warn_attach_fallback(name)
            if previous_mode == "attached":
                scene.robot_state.attached_collision_objects.append(self._attached_remove(name))
            scene.world.collision_objects.append(collision_world)
            signature_parts.append(
                "{}:world:{:.3f}:{:.3f}:{:.3f}".format(
                    name,
                    sample.x,
                    sample.y,
                    sample.z,
                )
            )
            next_modes[name] = "world"

        request = ApplyPlanningScene.Request()
        request.scene = scene
        self._last_modes = next_modes
        return request, "|".join(signature_parts)

    def _timer_cb(self) -> None:
        if self._apply_future is not None and not self._apply_future.done():
            return
        if not self._apply_client.wait_for_service(timeout_sec=0.0):
            now = time.monotonic()
            if (now - self._last_service_log) >= 2.0:
                self._last_service_log = now
                self.get_logger().warning(
                    f"[SCENE_SYNC] servicio no disponible: {self._apply_service_name}"
                )
            return

        request, signature = self._build_scene_request()
        if signature == self._last_signature:
            return
        self._last_signature = signature
        self._apply_future = self._apply_client.call_async(request)
        self._apply_future.add_done_callback(self._on_apply_done)

    def _on_apply_done(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"[SCENE_SYNC] apply_planning_scene failed: {exc}")
            return
        if response is None or not bool(getattr(response, "success", False)):
            self.get_logger().warning("[SCENE_SYNC] apply_planning_scene devolvio success=false")


def main() -> None:
    rclpy.init()
    node = PlanningSceneSync()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()