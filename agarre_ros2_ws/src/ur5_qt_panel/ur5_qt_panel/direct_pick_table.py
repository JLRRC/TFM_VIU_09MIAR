#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/direct_pick_table.py
# Contenido: Prueba directa de pick sobre mesa sin CNN.
# Uso breve: Publica objetivos PoseStamped en base_link para ur5_moveit_bridge.
"""Direct table pick test for UR5 + RG2 without perception."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time
import uuid
from typing import Optional, Sequence

from builtin_interfaces.msg import Time as BuiltinTime
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Empty, Float64MultiArray, String
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener


def ros_now_msg(node: Node) -> BuiltinTime:
    """Return the current ROS time from *node* as a builtin_interfaces/Time."""
    return node.get_clock().now().to_msg()


def ros_now_ns(node: Node) -> int:
    """Return current ROS time in nanoseconds. With use_sim_time it is 0 until /clock is valid."""
    return int(node.get_clock().now().nanoseconds or 0)


def wait_for_valid_clock(node: Node, timeout_sec: float = 15.0) -> bool:
    """Block until node.get_clock().now().nanoseconds > 0."""
    deadline = time.monotonic() + max(0.1, float(timeout_sec))
    while rclpy.ok() and time.monotonic() < deadline:
        if ros_now_ns(node) > 0:
            return True
        rclpy.spin_once(node, timeout_sec=0.05)
    return ros_now_ns(node) > 0


def make_pose_stamped(
    node: Node,
    *,
    frame_id: str,
    xyz: Sequence[float],
    quat_xyzw: Sequence[float] = (0.0, 1.0, 0.0, 0.0),
) -> PoseStamped:
    """Build a PoseStamped using node.get_clock().now().to_msg()."""
    if len(xyz) != 3:
        raise ValueError(f"xyz must have 3 values, got {len(xyz)}")
    if len(quat_xyzw) != 4:
        raise ValueError(f"quat_xyzw must have 4 values, got {len(quat_xyzw)}")
    pose = PoseStamped()
    pose.header.frame_id = str(frame_id or "base_link").strip() or "base_link"
    pose.header.stamp = ros_now_msg(node)
    pose.pose.position.x = float(xyz[0])
    pose.pose.position.y = float(xyz[1])
    pose.pose.position.z = float(xyz[2])
    pose.pose.orientation.x = float(quat_xyzw[0])
    pose.pose.orientation.y = float(quat_xyzw[1])
    pose.pose.orientation.z = float(quat_xyzw[2])
    pose.pose.orientation.w = float(quat_xyzw[3])
    return pose


def wait_for_latest_tf(
    node: Node,
    tf_buffer: Buffer,
    *,
    target_frame: str,
    source_frame: str,
    timeout_sec: float = 10.0,
) -> bool:
    """Wait for the latest available TF using Time(), avoiding timestamp extrapolation."""
    deadline = time.monotonic() + max(0.1, float(timeout_sec))
    latest = Time()
    while rclpy.ok() and time.monotonic() < deadline:
        try:
            if tf_buffer.can_transform(
                str(target_frame),
                str(source_frame),
                latest,
                timeout=Duration(seconds=0.05),
            ):
                return True
        except Exception:
            pass
        rclpy.spin_once(node, timeout_sec=0.05)
    return False


def transform_pose_latest(
    node: Node,
    tf_buffer: Buffer,
    pose: PoseStamped,
    *,
    target_frame: str = "base_link",
    timeout_sec: float = 5.0,
) -> PoseStamped:
    """Transform a pose using latest TF (Time()) and stamp the output with ROS now."""
    source_frame = str(pose.header.frame_id or "").strip()
    if not source_frame:
        raise ValueError("pose.header.frame_id is empty")
    if source_frame == target_frame:
        out = PoseStamped()
        out.header.frame_id = target_frame
        out.header.stamp = ros_now_msg(node)
        out.pose = pose.pose
        return out
    if not wait_for_latest_tf(
        node,
        tf_buffer,
        target_frame=target_frame,
        source_frame=source_frame,
        timeout_sec=timeout_sec,
    ):
        raise RuntimeError(f"TF unavailable: {target_frame} <- {source_frame}")
    transform = tf_buffer.lookup_transform(target_frame, source_frame, Time())
    converted = do_transform_pose_stamped(pose, transform)
    converted.header.frame_id = target_frame
    converted.header.stamp = ros_now_msg(node)
    return converted


def build_direct_pick_pose_stamped(
    node: Node,
    tf_buffer: Buffer,
    *,
    object_x: float,
    object_y: float,
    object_z: float,
    object_frame: str = "world",
    target_frame: str = "base_link",
    object_to_tcp_z_offset_m: float = 0.0,
    approach_z_offset_m: float = 0.12,
    retreat_z_offset_m: float = 0.16,
    quat_xyzw: Sequence[float] = (0.0, 1.0, 0.0, 0.0),
) -> tuple[PoseStamped, PoseStamped, PoseStamped]:
    """Build pregrasp, grasp and retreat poses in base_link for direct table pick."""
    object_pose = make_pose_stamped(
        node,
        frame_id=object_frame,
        xyz=(object_x, object_y, object_z),
        quat_xyzw=quat_xyzw,
    )
    object_base = transform_pose_latest(
        node,
        tf_buffer,
        object_pose,
        target_frame=target_frame,
    )
    x = float(object_base.pose.position.x)
    y = float(object_base.pose.position.y)
    grasp_z = float(object_base.pose.position.z) + float(object_to_tcp_z_offset_m)
    pregrasp = make_pose_stamped(
        node,
        frame_id=target_frame,
        xyz=(x, y, grasp_z + float(approach_z_offset_m)),
        quat_xyzw=quat_xyzw,
    )
    grasp = make_pose_stamped(
        node,
        frame_id=target_frame,
        xyz=(x, y, grasp_z),
        quat_xyzw=quat_xyzw,
    )
    retreat = make_pose_stamped(
        node,
        frame_id=target_frame,
        xyz=(x, y, grasp_z + float(retreat_z_offset_m)),
        quat_xyzw=quat_xyzw,
    )
    return pregrasp, grasp, retreat


@dataclass
class MoveResult:
    ok: bool
    message: str
    request_id: int
    request_uuid: str


class DirectPickTableNode(Node):
    """Small ROS node that drives the existing ur5_moveit_bridge contract."""

    def __init__(self) -> None:
        super().__init__(
            "test_direct_pick_table",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True),
            ],
            automatically_declare_parameters_from_overrides=True,
        )
        self._declare_if_absent("object_x", 0.45)
        self._declare_if_absent("object_y", 0.00)
        self._declare_if_absent("object_z", 0.78)
        self._declare_if_absent("object_frame", "world")
        self._declare_if_absent("target_frame", "base_link")
        self._declare_if_absent("world_frame", "world")
        self._declare_if_absent("base_frame", "base_link")
        self._declare_if_absent("tool_frame", "tool0")
        self._declare_if_absent("tcp_frame", "rg2_pinch_center")
        self._declare_if_absent("object_name", "pick_demo")
        self._declare_if_absent("object_to_tcp_z_offset_m", 0.0)
        self._declare_if_absent("approach_z_offset_m", 0.12)
        self._declare_if_absent("retreat_z_offset_m", 0.16)
        self._declare_if_absent("move_timeout_sec", 120.0)
        self._declare_if_absent("clock_timeout_sec", 15.0)
        self._declare_if_absent("tf_timeout_sec", 15.0)
        self._declare_if_absent("settle_sec", 0.8)
        self._declare_if_absent("post_move_settle_sec", 4.0)
        self._declare_if_absent("gripper_open_rad", 0.0425)
        self._declare_if_absent("gripper_closed_rad", 0.0)
        self._declare_if_absent("gripper_joint2_sign", 1.0)
        self._declare_if_absent("attach_after_close", True)
        self._declare_if_absent("cartesian_descent", True)
        self._declare_if_absent("cartesian_retreat", False)
        self._declare_if_absent("quat_xyzw", [0.0, 1.0, 0.0, 0.0])

        self._pose_pub = self.create_publisher(PoseStamped, "/desired_grasp", 10)
        self._cart_pub = self.create_publisher(PoseStamped, "/desired_grasp_cartesian", 10)
        self._gripper_pub = self.create_publisher(
            Float64MultiArray,
            "/gripper_controller/commands",
            10,
        )
        self._attach_pub = None
        self._clock_seen_ns = 0
        self._last_result: Optional[dict] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(String, "/desired_grasp/result", self._on_result, 10)

    def _declare_if_absent(self, name: str, default_value) -> None:
        if self.has_parameter(name):
            return
        self.declare_parameter(name, default_value)

    def _on_clock(self, msg: Clock) -> None:
        self._clock_seen_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)

    def _on_result(self, msg: String) -> None:
        try:
            self._last_result = json.loads(msg.data)
        except Exception:
            self._last_result = {"success": False, "message": msg.data}

    def _param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _param_str(self, name: str) -> str:
        return str(self.get_parameter(name).value or "").strip()

    def _quat(self) -> tuple[float, float, float, float]:
        raw = list(self.get_parameter("quat_xyzw").value or [])
        if len(raw) != 4:
            raise RuntimeError("quat_xyzw must have exactly 4 values")
        q = tuple(float(v) for v in raw)
        norm = math.sqrt(sum(v * v for v in q))
        if norm <= 1e-9:
            raise RuntimeError("quat_xyzw has zero norm")
        return tuple(v / norm for v in q)  # type: ignore[return-value]

    def _publish_gripper(self, value: float) -> None:
        sign = self._param_float("gripper_joint2_sign")
        msg = Float64MultiArray()
        msg.data = [float(value), float(value) * sign]
        self._gripper_pub.publish(msg)

    def _attach_object(self) -> None:
        object_name = self._param_str("object_name") or "pick_demo"
        topic = f"/gripper/{object_name}/attach".replace("//", "/")
        if self._attach_pub is None:
            self._attach_pub = self.create_publisher(Empty, topic, 10)
        self._attach_pub.publish(Empty())
        self.get_logger().info(f"attach request published on {topic}")

    def _wait_bridge_subscriber(self, timeout_sec: float = 10.0) -> bool:
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            if self.count_subscribers("/desired_grasp") > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.count_subscribers("/desired_grasp") > 0

    def _publish_pose_and_wait(
        self,
        label: str,
        pose: PoseStamped,
        *,
        cartesian: bool = False,
        timeout_sec: float = 120.0,
    ) -> MoveResult:
        request_id = int(time.monotonic_ns() % 2_000_000_000)
        request_uuid = uuid.uuid4().hex
        frame_clean = str(pose.header.frame_id or "base_link").split("|", 1)[0]
        pose.header.frame_id = (
            f"{frame_clean}|rid={request_id}|uid={request_uuid}|phase_label={label}"
        )
        pose.header.stamp = ros_now_msg(self)
        self._last_result = None
        pub = self._cart_pub if cartesian else self._pose_pub
        pub.publish(pose)
        self.get_logger().info(
            f"published {label} frame={frame_clean} cartesian={str(cartesian).lower()} "
            "pos="
            f"({pose.pose.position.x:.3f},"
            f"{pose.pose.position.y:.3f},"
            f"{pose.pose.position.z:.3f}) "
            f"request_id={request_id}"
        )
        deadline = time.monotonic() + max(1.0, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            result = self._last_result
            if not result:
                continue
            if int(result.get("request_id", -1)) != int(request_id):
                continue
            if str(result.get("request_uuid", "")) != request_uuid:
                continue
            ok = bool(result.get("success")) and bool(result.get("exec_ok", True))
            msg = str(result.get("message", ""))
            return MoveResult(ok=ok, message=msg, request_id=request_id, request_uuid=request_uuid)
        return MoveResult(
            ok=False,
            message="timeout_waiting_desired_grasp_result",
            request_id=request_id,
            request_uuid=request_uuid,
        )

    def test_direct_pick_table(self) -> bool:
        """Execute: wait /clock, wait TF, pregrasp, grasp, close RG2, retreat."""
        clock_timeout = self._param_float("clock_timeout_sec")
        if not wait_for_valid_clock(self, timeout_sec=clock_timeout):
            self.get_logger().error(
                f"/clock invalid after {clock_timeout:.1f}s; node clock ns={ros_now_ns(self)}"
            )
            return False
        self.get_logger().info(f"/clock valid: now_ns={ros_now_ns(self)}")

        tf_timeout = self._param_float("tf_timeout_sec")
        world_frame = self._param_str("world_frame") or "world"
        base_frame = self._param_str("base_frame") or "base_link"
        tool_frame = self._param_str("tool_frame") or "tool0"
        tcp_frame = self._param_str("tcp_frame") or "rg2_pinch_center"
        frames = [
            (world_frame, base_frame),
            (base_frame, tool_frame),
            (base_frame, tcp_frame),
        ]
        for target, source in frames:
            if not wait_for_latest_tf(
                self,
                self._tf_buffer,
                target_frame=target,
                source_frame=source,
                timeout_sec=tf_timeout,
            ):
                self.get_logger().error(f"TF missing: {target} <- {source}")
                return False
            self.get_logger().info(f"TF ok: {target} <- {source}")

        if not self._wait_bridge_subscriber(timeout_sec=10.0):
            self.get_logger().error(
                "No subscriber on /desired_grasp; ur5_moveit_bridge is not ready"
            )
            return False

        pregrasp, grasp, retreat = build_direct_pick_pose_stamped(
            self,
            self._tf_buffer,
            object_x=self._param_float("object_x"),
            object_y=self._param_float("object_y"),
            object_z=self._param_float("object_z"),
            object_frame=self._param_str("object_frame") or "world",
            target_frame=self._param_str("target_frame") or "base_link",
            object_to_tcp_z_offset_m=self._param_float("object_to_tcp_z_offset_m"),
            approach_z_offset_m=self._param_float("approach_z_offset_m"),
            retreat_z_offset_m=self._param_float("retreat_z_offset_m"),
            quat_xyzw=self._quat(),
        )
        self.get_logger().info(
            "direct pick targets in base_link: "
            f"pre=({pregrasp.pose.position.x:.3f},"
            f"{pregrasp.pose.position.y:.3f},"
            f"{pregrasp.pose.position.z:.3f}) "
            f"grasp=({grasp.pose.position.x:.3f},"
            f"{grasp.pose.position.y:.3f},"
            f"{grasp.pose.position.z:.3f}) "
            f"retreat=({retreat.pose.position.x:.3f},"
            f"{retreat.pose.position.y:.3f},"
            f"{retreat.pose.position.z:.3f})"
        )

        self._publish_gripper(self._param_float("gripper_open_rad"))
        time.sleep(self._param_float("settle_sec"))

        move_timeout = self._param_float("move_timeout_sec")
        post_move_settle_sec = max(0.0, self._param_float("post_move_settle_sec"))
        for label, pose, cart in (
            ("DIRECT_TABLE_PREGRASP", pregrasp, False),
            ("DIRECT_TABLE_GRASP", grasp, bool(self.get_parameter("cartesian_descent").value)),
        ):
            result = self._publish_pose_and_wait(
                label,
                pose,
                cartesian=cart,
                timeout_sec=move_timeout,
            )
            if not result.ok:
                self.get_logger().error(f"{label} failed: {result.message}")
                return False
            if post_move_settle_sec > 0.0:
                self.get_logger().info(
                    f"{label} accepted; settling {post_move_settle_sec:.1f}s"
                )
                time.sleep(post_move_settle_sec)

        self._publish_gripper(self._param_float("gripper_closed_rad"))
        time.sleep(self._param_float("settle_sec"))
        if bool(self.get_parameter("attach_after_close").value):
            self._attach_object()
            time.sleep(0.3)

        result = self._publish_pose_and_wait(
            "DIRECT_TABLE_RETREAT",
            retreat,
            cartesian=bool(self.get_parameter("cartesian_retreat").value),
            timeout_sec=move_timeout,
        )
        if not result.ok:
            self.get_logger().error(f"DIRECT_TABLE_RETREAT failed: {result.message}")
            return False
        if post_move_settle_sec > 0.0:
            self.get_logger().info(
                f"DIRECT_TABLE_RETREAT accepted; settling {post_move_settle_sec:.1f}s"
            )
            time.sleep(post_move_settle_sec)
        self.get_logger().info("test_direct_pick_table completed")
        return True


def test_direct_pick_table() -> bool:
    """Standalone entry point callable from Python after rclpy.init()."""
    node = DirectPickTableNode()
    try:
        return node.test_direct_pick_table()
    finally:
        node.destroy_node()


def main() -> None:
    rclpy.init()
    ok = False
    node = DirectPickTableNode()
    try:
        ok = node.test_direct_pick_table()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
