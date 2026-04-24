#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/inspect_rg2_visual_pose.py
# Contenido: Herramienta de diagnóstico de pose visual del gripper RG2 en runtime.
# Uso breve: ros2 run ur5_tools inspect_rg2_visual_pose [--ros-args -p once:=true]
"""Runtime visual pose inspector for the RG2 gripper.

Compares TF-based semantic TCP (rg2_pinch_center) against the physical/visual
position of RG2 links obtained from Gazebo /world/pose/info and from
robot_state_publisher (URDF links). Prints a structured diagnostic table.

Sources:
  TF  → tool0, rg2_pinch_center, rg2_tcp, rg2_base_link,
         rg2_finger_link1, rg2_finger_link2  (via robot_state_publisher)
  Gz  → rg2_hand, rg2_leftfinger, rg2_rightfinger, pick_demo
         (via gz_pose_bridge → /world/<world>/pose/info as TFMessage)
  JS  → rg2_finger_joint1, rg2_finger_joint2  (via /joint_states)
"""
from __future__ import annotations

import math
import threading
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # (x, y, z, w)
PoseEntry = Tuple[Optional[Vec3], Optional[Quat]]


# ── helpers ──────────────────────────────────────────────────────────────────

def _dist3(a: Optional[Vec3], b: Optional[Vec3]) -> Optional[float]:
    if a is None or b is None:
        return None
    return math.sqrt(
        (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2
    )


def _midpoint(a: Optional[Vec3], b: Optional[Vec3]) -> Optional[Vec3]:
    if a is None or b is None:
        return None
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0)


def _fmt_vec(v: Optional[Vec3]) -> str:
    if v is None:
        return "N/A"
    return f"({v[0]:.4f}, {v[1]:.4f}, {v[2]:.4f})"


def _fmt_quat(q: Optional[Quat]) -> str:
    if q is None:
        return "N/A"
    return f"({q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f})"


def _fmt_dist(d: Optional[float]) -> str:
    if d is None:
        return "N/A"
    return f"{d:.4f}m"


def _rot_err_deg(q1: Optional[Quat], q2: Optional[Quat]) -> Optional[float]:
    """Angular error between two unit quaternions in degrees."""
    if q1 is None or q2 is None:
        return None
    dot = abs(q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3])
    dot = min(1.0, max(0.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def _rotate_vec_by_quat(v: Vec3, q: Quat) -> Vec3:
    """Rotate vector v by quaternion q (active rotation)."""
    qx, qy, qz, qw = q
    dx, dy, dz = v
    # t = 2 * cross(q_xyz, v)
    tx = 2.0 * (qy * dz - qz * dy)
    ty = 2.0 * (qz * dx - qx * dz)
    tz = 2.0 * (qx * dy - qy * dx)
    return (
        dx + qw * tx + qy * tz - qz * ty,
        dy + qw * ty + qz * tx - qx * tz,
        dz + qw * tz + qx * ty - qy * tx,
    )


def _obj_in_gripper_local(
    obj_world: Optional[Vec3],
    pinch_pos: Optional[Vec3],
    pinch_q: Optional[Quat],
) -> Optional[Vec3]:
    """Transform object world pos into rg2_pinch_center local frame."""
    if obj_world is None or pinch_pos is None or pinch_q is None:
        return None
    delta = (
        obj_world[0] - pinch_pos[0],
        obj_world[1] - pinch_pos[1],
        obj_world[2] - pinch_pos[2],
    )
    # inverse quaternion for unit quat: (−x,−y,−z,w)
    q_inv: Quat = (-pinch_q[0], -pinch_q[1], -pinch_q[2], pinch_q[3])
    return _rotate_vec_by_quat(delta, q_inv)


# ── node ─────────────────────────────────────────────────────────────────────

class Rg2VisualPoseInspector(Node):
    """Inspects and compares TF vs Gazebo visual poses of the RG2 gripper."""

    def __init__(self) -> None:
        super().__init__("rg2_visual_pose_inspector")

        self.declare_parameter("once", True)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tcp_frame", "rg2_pinch_center")
        self.declare_parameter("object_name", "pick_demo")
        self.declare_parameter("max_pinch_visual_mid_dist_m", 0.010)
        self.declare_parameter("max_visual_mid_obj_dist_m", 0.020)

        self._once = bool(self.get_parameter("once").value)
        self._rate_hz = float(self.get_parameter("rate_hz").value)
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tcp_frame = str(self.get_parameter("tcp_frame").value)
        self._object_name = str(self.get_parameter("object_name").value)
        self._max_pinch_visual_mid = float(
            self.get_parameter("max_pinch_visual_mid_dist_m").value
        )
        self._max_visual_mid_obj = float(
            self.get_parameter("max_visual_mid_obj_dist_m").value
        )

        # detect gz world topic from world_frame name (fallback to default)
        world_name = self._world_frame if self._world_frame != "world" else "ur5_mesa_objetos"
        self._gz_topic = f"/world/{world_name}/pose/info"

        self._lock = threading.Lock()
        # Gazebo entity poses: name → (Vec3, Quat)
        # Keys stored: full scoped name, last component, first component (via setdefault)
        self._gz_pose: Dict[str, PoseEntry] = {}
        # All raw names received from pose/info (for audit)
        self._gz_raw_names: list = []
        # joint_states
        self._joint_positions: Dict[str, float] = {}
        self._joint_stamp: float = 0.0
        self._gz_msg_count: int = 0
        self._js_msg_count: int = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(
            TFMessage,
            self._gz_topic,
            self._on_pose_info,
            10,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
        )

        if self._once:
            # wait 2 s for TF and pose/info to populate before firing
            self._fired = False
            self._once_timer = self.create_timer(2.0, self._run_once)
        else:
            period = 1.0 / max(0.1, self._rate_hz)
            self.create_timer(period, self._inspect)

        self.get_logger().info(
            f"[RG2_VISUAL_POSE] node ready "
            f"world={self._world_frame} tcp={self._tcp_frame} "
            f"object={self._object_name} "
            f"gz_topic={self._gz_topic} "
            f"once={self._once} rate={self._rate_hz:.1f}Hz"
        )

    # ── subscribers ──────────────────────────────────────────────────────────

    def _on_pose_info(self, msg: TFMessage) -> None:
        data: Dict[str, PoseEntry] = {}
        raw_names: list = []
        for tf in getattr(msg, "transforms", []):
            name = str(getattr(tf, "child_frame_id", "") or "").strip()
            if not name:
                continue
            t = getattr(tf, "transform", None)
            if not t:
                continue
            tr = getattr(t, "translation", None)
            ro = getattr(t, "rotation", None)
            if not tr:
                continue
            pos: Vec3 = (float(tr.x), float(tr.y), float(tr.z))
            quat: Optional[Quat] = (
                (float(ro.x), float(ro.y), float(ro.z), float(ro.w))
                if ro is not None
                else None
            )
            # 1. full scoped name (always stored, overwrites with latest)
            data[name] = (pos, quat)
            raw_names.append(name)
            if "::" in name:
                parts = name.split("::")
                # 2. last component = link name, e.g. "rg2_hand" from "ur5_rg2::rg2_hand"
                #    setdefault: exact bare name takes priority over scoped fallback
                data.setdefault(parts[-1], (pos, quat))
                # 3. first component = model name, e.g. "ur5_rg2"
                data.setdefault(parts[0], (pos, quat))
                # 4. two last components joined, e.g. "rg2_hand" (same as [-1] for depth-2)
                #    useful for depth-3+ scoped names like "model::link::sublink"
                if len(parts) >= 3:
                    data.setdefault("::".join(parts[-2:]), (pos, quat))
        with self._lock:
            self._gz_pose.update(data)
            self._gz_msg_count += 1
            # accumulate unique raw names for audit
            existing = set(self._gz_raw_names)
            for n in raw_names:
                if n not in existing:
                    self._gz_raw_names.append(n)
                    existing.add(n)

    def _on_joint_state(self, msg: JointState) -> None:
        names = list(getattr(msg, "name", []) or [])
        pos_list = list(getattr(msg, "position", []) or [])
        positions: Dict[str, float] = {}
        for i, name in enumerate(names):
            if i < len(pos_list):
                positions[str(name)] = float(pos_list[i])
        with self._lock:
            self._joint_positions = positions
            self._joint_stamp = time.monotonic()
            self._js_msg_count += 1

    # ── TF helpers ───────────────────────────────────────────────────────────

    def _tf_lookup(self, frame: str) -> PoseEntry:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame,
                frame,
                rclpy.time.Time(),
            )
            t = tf.transform.translation
            r = tf.transform.rotation
            pos: Vec3 = (float(t.x), float(t.y), float(t.z))
            quat: Quat = (float(r.x), float(r.y), float(r.z), float(r.w))
            return pos, quat
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None, None
        except Exception:
            return None, None

    def _gz_lookup(self, name: str) -> PoseEntry:
        """Look up a Gazebo entity pose by name.

        Priority:
          1. Exact key match (covers bare name stored directly or via setdefault).
          2. Scan all full scoped names for a last-component match, skipping
             any entry whose last component contains 'anchor' (avoids
             pick_demo_anchor being returned when querying pick_demo).
        """
        with self._lock:
            # 1. exact (catches both bare names and any setdefault alias)
            if name in self._gz_pose:
                return self._gz_pose[name]
            # 2. fallback: scan scoped names by last component
            for full_name, entry in self._gz_pose.items():
                if "::" not in full_name:
                    continue
                last = full_name.split("::")[-1]
                if last == name and "anchor" not in last:
                    return entry
        return (None, None)

    def _gz_all_keys_snapshot(self) -> list:
        """Return sorted list of all keys currently in the pose cache."""
        with self._lock:
            return sorted(self._gz_pose.keys())

    def _gz_raw_names_snapshot(self) -> list:
        """Return the raw scoped entity names seen in pose/info so far."""
        with self._lock:
            return list(self._gz_raw_names)

    # ── one-shot timer ────────────────────────────────────────────────────────

    def _run_once(self) -> None:
        if self._fired:
            return
        self._fired = True
        self._once_timer.cancel()
        self._inspect()
        rclpy.try_shutdown()

    # ── main inspection ──────────────────────────────────────────────────────

    def _inspect(self) -> None:
        # ── TF frames (URDF / robot_state_publisher) ──────────────────────
        tool0_pos, tool0_q = self._tf_lookup("tool0")
        pinch_pos, pinch_q = self._tf_lookup("rg2_pinch_center")
        rg2_tcp_pos, rg2_tcp_q = self._tf_lookup("rg2_tcp")
        rg2_base_pos, _ = self._tf_lookup("rg2_base_link")
        finger1_pos, _ = self._tf_lookup("rg2_finger_link1")
        finger2_pos, _ = self._tf_lookup("rg2_finger_link2")

        # ── Gazebo entity poses (SDF / gz_pose_bridge) ────────────────────
        rg2_hand_gz_pos, rg2_hand_gz_q = self._gz_lookup("rg2_hand")
        left_gz_pos, _ = self._gz_lookup("rg2_leftfinger")
        right_gz_pos, _ = self._gz_lookup("rg2_rightfinger")
        obj_gz_pos, _ = self._gz_lookup(self._object_name)

        # ── derived midpoints ─────────────────────────────────────────────
        urdf_finger_mid = _midpoint(finger1_pos, finger2_pos)
        gz_visual_mid = _midpoint(left_gz_pos, right_gz_pos)

        # ── joint state ────────────────────────────────────────────────────
        with self._lock:
            joints = dict(self._joint_positions)
            joint_age = (
                time.monotonic() - self._joint_stamp if self._joint_stamp else None
            )
            gz_count = int(self._gz_msg_count)
            js_count = int(self._js_msg_count)
        gz_raw_names = self._gz_raw_names_snapshot()
        gz_all_keys = self._gz_all_keys_snapshot()

        fj1 = joints.get("rg2_finger_joint1")
        fj2 = joints.get("rg2_finger_joint2")
        opening_sum: Optional[float] = None
        if fj1 is not None and fj2 is not None:
            opening_sum = abs(float(fj1)) + abs(float(fj2))

        # ── distances ──────────────────────────────────────────────────────
        dist_pinch_obj = _dist3(pinch_pos, obj_gz_pos)
        dist_gz_mid_obj = _dist3(gz_visual_mid, obj_gz_pos)
        dist_urdf_mid_obj = _dist3(urdf_finger_mid, obj_gz_pos)
        dist_pinch_gz_mid = _dist3(pinch_pos, gz_visual_mid)
        dist_pinch_urdf_mid = _dist3(pinch_pos, urdf_finger_mid)
        dist_tool0_pinch = _dist3(tool0_pos, pinch_pos)
        dist_hand_pinch = _dist3(rg2_hand_gz_pos, pinch_pos)

        # ── orientation ────────────────────────────────────────────────────
        rot_err_pinch_vs_hand_deg = _rot_err_deg(pinch_q, rg2_hand_gz_q)

        # closing axis: Y-axis of rg2_pinch_center in world (perpendicular to approach)
        closing_axis: Optional[Vec3] = None
        if pinch_q is not None:
            closing_axis = _rotate_vec_by_quat((0.0, 1.0, 0.0), pinch_q)

        # object in gripper-local frame
        obj_local = _obj_in_gripper_local(obj_gz_pos, pinch_pos, pinch_q)

        # ── verdict ────────────────────────────────────────────────────────
        has_gz_fingers = left_gz_pos is not None and right_gz_pos is not None

        if not has_gz_fingers:
            verdict = "UNKNOWN"
            reason = "missing_visual_finger_pose"
        elif dist_pinch_gz_mid is None:
            verdict = "UNKNOWN"
            reason = "dist_pinch_gz_mid_unavailable"
        elif dist_pinch_gz_mid > self._max_pinch_visual_mid:
            verdict = "FAIL"
            if dist_gz_mid_obj is not None and dist_gz_mid_obj <= self._max_visual_mid_obj:
                reason = (
                    f"pinch_vs_gz_mid={dist_pinch_gz_mid:.4f}m "
                    f"> tol={self._max_pinch_visual_mid:.3f}m  "
                    "gripper_visual_NOT_following_tcp_semantic"
                )
            else:
                reason = (
                    f"pinch_vs_gz_mid={dist_pinch_gz_mid:.4f}m "
                    f"> tol={self._max_pinch_visual_mid:.3f}m  "
                    "gripper_visual_desalineado_del_tcp"
                )
        elif dist_gz_mid_obj is None:
            verdict = "UNKNOWN"
            reason = "dist_gz_mid_obj_unavailable"
        elif dist_gz_mid_obj > self._max_visual_mid_obj:
            verdict = "FAIL"
            reason = (
                f"gz_visual_mid_to_obj={dist_gz_mid_obj:.4f}m "
                f"> tol={self._max_visual_mid_obj:.3f}m  "
                "finger_visual_mid_lejos_del_objeto"
            )
        else:
            verdict = "OK"
            reason = (
                f"pinch_vs_gz_mid={dist_pinch_gz_mid:.4f}m "
                f"<= {self._max_pinch_visual_mid:.3f}m  "
                f"gz_mid_to_obj={dist_gz_mid_obj:.4f}m "
                f"<= {self._max_visual_mid_obj:.3f}m"
            )

        self._print_report(
            tool0_pos=tool0_pos,
            tool0_q=tool0_q,
            pinch_pos=pinch_pos,
            pinch_q=pinch_q,
            rg2_tcp_pos=rg2_tcp_pos,
            rg2_base_pos=rg2_base_pos,
            finger1_pos=finger1_pos,
            finger2_pos=finger2_pos,
            urdf_finger_mid=urdf_finger_mid,
            rg2_hand_gz_pos=rg2_hand_gz_pos,
            rg2_hand_gz_q=rg2_hand_gz_q,
            left_gz_pos=left_gz_pos,
            right_gz_pos=right_gz_pos,
            gz_visual_mid=gz_visual_mid,
            obj_gz_pos=obj_gz_pos,
            fj1=fj1,
            fj2=fj2,
            opening_sum=opening_sum,
            joint_age=joint_age,
            gz_msg_count=gz_count,
            js_msg_count=js_count,
            gz_raw_names=gz_raw_names,
            gz_all_keys=gz_all_keys,
            dist_pinch_obj=dist_pinch_obj,
            dist_gz_mid_obj=dist_gz_mid_obj,
            dist_urdf_mid_obj=dist_urdf_mid_obj,
            dist_pinch_gz_mid=dist_pinch_gz_mid,
            dist_pinch_urdf_mid=dist_pinch_urdf_mid,
            dist_tool0_pinch=dist_tool0_pinch,
            dist_hand_pinch=dist_hand_pinch,
            rot_err_pinch_vs_hand_deg=rot_err_pinch_vs_hand_deg,
            closing_axis=closing_axis,
            obj_local=obj_local,
            verdict=verdict,
            reason=reason,
        )

    # ── report printer ────────────────────────────────────────────────────────

    def _print_report(self, **kw) -> None:
        log = self.get_logger()
        tag = "[RG2_VISUAL_POSE]"
        sep = "─" * 60

        log.info(f"{tag} {sep}")
        log.info(f"{tag} ── TF FRAMES (robot_state_publisher / URDF) ──────────────")
        log.info(
            f"{tag} tool0_tf_world={_fmt_vec(kw['tool0_pos'])} "
            f"q={_fmt_quat(kw['tool0_q'])}"
        )
        log.info(
            f"{tag} pinch_tf_world={_fmt_vec(kw['pinch_pos'])} "
            f"q={_fmt_quat(kw['pinch_q'])}"
        )
        log.info(f"{tag} rg2_tcp_tf_world={_fmt_vec(kw['rg2_tcp_pos'])}")
        log.info(f"{tag} rg2_base_link_tf_world={_fmt_vec(kw['rg2_base_pos'])}")
        log.info(f"{tag} rg2_finger_link1_tf_world={_fmt_vec(kw['finger1_pos'])}")
        log.info(f"{tag} rg2_finger_link2_tf_world={_fmt_vec(kw['finger2_pos'])}")
        log.info(f"{tag} urdf_finger_midpoint_world={_fmt_vec(kw['urdf_finger_mid'])}")

        log.info(f"{tag} ── GAZEBO VISUAL (SDF / gz_pose_bridge) ─────────────────")
        log.info(
            f"{tag} rg2_hand_gz_world={_fmt_vec(kw['rg2_hand_gz_pos'])} "
            f"q={_fmt_quat(kw['rg2_hand_gz_q'])}"
        )
        log.info(f"{tag} leftfinger_gz_world={_fmt_vec(kw['left_gz_pos'])}")
        log.info(f"{tag} rightfinger_gz_world={_fmt_vec(kw['right_gz_pos'])}")
        log.info(f"{tag} visual_finger_midpoint_world={_fmt_vec(kw['gz_visual_mid'])}")

        log.info(f"{tag} ── OBJECT ────────────────────────────────────────────────")
        log.info(
            f"{tag} object_world={_fmt_vec(kw['obj_gz_pos'])} "
            f"object_name={self._object_name}"
        )

        log.info(f"{tag} ── DISTANCES ─────────────────────────────────────────────")
        log.info(f"{tag} dist_pinch_to_obj={_fmt_dist(kw['dist_pinch_obj'])}")
        log.info(f"{tag} dist_gz_visual_mid_to_obj={_fmt_dist(kw['dist_gz_mid_obj'])}")
        log.info(
            f"{tag} dist_urdf_finger_mid_to_obj={_fmt_dist(kw['dist_urdf_mid_obj'])}"
        )
        log.info(
            f"{tag} dist_pinch_to_gz_visual_mid={_fmt_dist(kw['dist_pinch_gz_mid'])}"
        )
        log.info(
            f"{tag} dist_pinch_to_urdf_finger_mid={_fmt_dist(kw['dist_pinch_urdf_mid'])}"
        )
        log.info(f"{tag} dist_tool0_to_pinch={_fmt_dist(kw['dist_tool0_pinch'])}")
        log.info(f"{tag} dist_rg2_hand_to_pinch={_fmt_dist(kw['dist_hand_pinch'])}")

        log.info(f"{tag} ── JOINT STATE ───────────────────────────────────────────")
        fj1 = kw["fj1"]
        fj2 = kw["fj2"]
        os_val = kw["opening_sum"]
        ja = kw["joint_age"]
        log.info(
            f"{tag} rg2_finger_joint1="
            + (f"{fj1:.4f}" if fj1 is not None else "N/A")
        )
        log.info(
            f"{tag} rg2_finger_joint2="
            + (f"{fj2:.4f}" if fj2 is not None else "N/A")
        )
        log.info(
            f"{tag} finger_opening_sum="
            + (f"{os_val:.4f}" if os_val is not None else "N/A")
        )
        log.info(
            f"{tag} joint_state_age="
            + (f"{ja:.2f}s" if ja is not None else "N/A")
        )

        log.info(f"{tag} ── ORIENTATION (FASE 5) ──────────────────────────────────")
        rot_err = kw["rot_err_pinch_vs_hand_deg"]
        if rot_err is not None:
            log.info(f"{tag} rot_err_pinch_vs_hand_deg={rot_err:.2f}")
        else:
            _pq = kw.get("pinch_q")
            _hq = kw.get("rg2_hand_gz_q")
            if _pq is None and _hq is None:
                _or = "pinch_tf_and_hand_gz_quat_missing"
            elif _pq is None:
                _or = "pinch_tf_quat_missing"
            else:
                _or = "rg2_hand_gz_quat_missing"
            log.info(
                f"{tag} rot_err_pinch_vs_hand_deg=N/A "
                f"orientation_check=UNAVAILABLE reason={_or}"
            )
        log.info(f"{tag} closing_axis_world={_fmt_vec(kw['closing_axis'])}")
        log.info(f"{tag} obj_local_in_gripper={_fmt_vec(kw['obj_local'])}")

        log.info(f"{tag} ── DATA SOURCES ──────────────────────────────────────────")
        log.info(f"{tag} gz_pose_info_msgs={kw['gz_msg_count']}")
        log.info(f"{tag} joint_state_msgs={kw['js_msg_count']}")
        log.info(f"{tag} gz_topic={self._gz_topic}")

        log.info(f"{tag} ── GAZEBO RAW ENTITY NAMES (pose/info) ──────────────────")
        raw_names = kw.get("gz_raw_names") or []
        if raw_names:
            for rn in sorted(raw_names):
                log.info(f"{tag}   raw: {rn}")
        else:
            log.info(f"{tag}   (none received yet)")

        gz_keys = kw.get("gz_all_keys") or []
        target_keys = {"rg2_hand", "rg2_leftfinger", "rg2_rightfinger", self._object_name}
        for tk in sorted(target_keys):
            found = tk in gz_keys
            log.info(f"{tag} key_check  {tk}: {'PRESENT' if found else 'MISSING'}")

        log.info(f"{tag} ── VERDICT ───────────────────────────────────────────────")
        log.info(f"{tag} verdict={kw['verdict']} reason={kw['reason']}")
        log.info(f"{tag} {sep}")


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = Rg2VisualPoseInspector()
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
