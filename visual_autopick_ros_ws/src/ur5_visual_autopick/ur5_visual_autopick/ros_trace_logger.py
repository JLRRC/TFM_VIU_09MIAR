"""ros_trace_logger.py — Comprehensive phase/pose/joint/TF/DH/SDF trace logger.

Captura en tiempo real:
  - joint_states (6 arm + 2 gripper): posición, velocidad, esfuerzo
  - TF: world→base_link, base_link→rg2_pinch_center, base_link→tool0,
         world→rg2_pinch_center, world→tool0, world→rg2_tcp, base_link→rg2_tcp
  - SDF/Gazebo poses: todos los objetos en /world/ur5_mesa_objetos/pose/info
  - FK DH UR5: calculada desde joint_states
  - /desired_grasp (MOVEIT/AGARRE start)
  - /desired_grasp/result (MOVEIT/AGARRE end)
  - /desired_grasp_cartesian (Cartesian descent)
  - /gripper_controller/commands (gripper cmd)
  - Snapshot periódico cada 0.5 s
  - Detección automática de modo: DIRECTO / MOVEIT / AGARRE

Escribe:
  LOG_DIR/trace_full.log     — todo en texto plano (legible)
  LOG_DIR/trace_phases.log   — solo eventos de fase (inicio/fin)
  LOG_DIR/trace_joints.csv   — joint states en CSV para análisis
  LOG_DIR/trace_sdf.csv      — poses SDF/Gazebo en CSV
  LOG_DIR/trace_tf.csv       — TF snapshots en CSV
"""
from __future__ import annotations

import csv
import json
import math
import os
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

# ---------------------------------------------------------------------------
# UR5 DH parameters (standard)
# ---------------------------------------------------------------------------
_DH_A     = (0.0,      -0.425,   -0.39225, 0.0,      0.0,      0.0)
_DH_D     = (0.089159,  0.0,      0.0,     0.10915,  0.09465,  0.0823)
_DH_ALPHA = (math.pi/2, 0.0,      0.0,     math.pi/2, -math.pi/2, 0.0)

_ARM_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
_FINGER_JOINTS = ("rg2_finger_joint1", "rg2_finger_joint2")

_TF_PAIRS = [
    ("world",      "base_link"),
    ("base_link",  "rg2_pinch_center"),
    ("base_link",  "tool0"),
    ("base_link",  "rg2_tcp"),
    ("world",      "rg2_pinch_center"),
    ("world",      "tool0"),
    ("world",      "rg2_tcp"),
    ("tool0",      "rg2_pinch_center"),
]


# ---------------------------------------------------------------------------
# DH forward kinematics
# ---------------------------------------------------------------------------
def _dh_transform(a, d, alpha, theta):
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return [
        [ct,       -st,       0,        a],
        [st*ca,    ct*ca,    -sa,      -sa*d],
        [st*sa,    ct*sa,     ca,       ca*d],
        [0,         0,         0,        1],
    ]


def _mat_mul(A, B):
    n = len(A)
    C = [[0.0]*n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            for k in range(n):
                C[i][j] += A[i][k] * B[k][j]
    return C


def compute_dh_fk(joint_positions: Dict[str, float]) -> Optional[Tuple[float, float, float, list]]:
    """Return (x, y, z, rpy_deg) of tool0 via DH FK, or None if joints missing."""
    try:
        angles = [float(joint_positions.get(j, 0.0)) for j in _ARM_JOINTS]
    except Exception:
        return None

    T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
    for i in range(6):
        Ti = _dh_transform(_DH_A[i], _DH_D[i], _DH_ALPHA[i], angles[i])
        T = _mat_mul(T, Ti)

    x, y, z = T[0][3], T[1][3], T[2][3]

    # Extract RPY from rotation matrix
    r11, r21 = T[0][0], T[1][0]
    r31 = T[2][0]
    r32, r33 = T[2][1], T[2][2]
    roll  = math.degrees(math.atan2(r32, r33))
    pitch = math.degrees(math.atan2(-r31, math.sqrt(r32**2 + r33**2)))
    yaw   = math.degrees(math.atan2(r21, r11))

    return x, y, z, [roll, pitch, yaw]


# ---------------------------------------------------------------------------
# Logger Node
# ---------------------------------------------------------------------------
class RosTraceLogger(Node):

    def __init__(self, log_dir: str) -> None:
        super().__init__("ros_trace_logger")

        self._log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)

        # ---- Files ----
        self._f_full   = open(os.path.join(log_dir, "trace_full.log"),   "a", buffering=1)
        self._f_phases = open(os.path.join(log_dir, "trace_phases.log"), "a", buffering=1)

        self._csv_joints = csv.writer(
            open(os.path.join(log_dir, "trace_joints.csv"), "a", newline="", buffering=1)
        )
        self._csv_sdf = csv.writer(
            open(os.path.join(log_dir, "trace_sdf.csv"), "a", newline="", buffering=1)
        )
        self._csv_tf = csv.writer(
            open(os.path.join(log_dir, "trace_tf.csv"), "a", newline="", buffering=1)
        )

        # CSV headers (only if new files)
        self._csv_joints.writerow([
            "wall_time", "ros_time",
            "shoulder_pan", "shoulder_lift", "elbow",
            "wrist_1", "wrist_2", "wrist_3",
            "finger1", "finger2",
            "vel_pan", "vel_lift", "vel_elbow",
            "vel_w1", "vel_w2", "vel_w3",
            "eff_pan", "eff_lift", "eff_elbow",
            "eff_w1", "eff_w2", "eff_w3",
            "dh_x", "dh_y", "dh_z", "dh_roll", "dh_pitch", "dh_yaw",
        ])
        self._csv_sdf.writerow([
            "wall_time", "entity_name",
            "tx", "ty", "tz", "qx", "qy", "qz", "qw",
        ])
        self._csv_tf.writerow([
            "wall_time", "parent_frame", "child_frame",
            "tx", "ty", "tz", "qx", "qy", "qz", "qw",
            "tf_age_sec",
        ])

        # ---- State ----
        self._lock = threading.Lock()
        self._joint_pos: Dict[str, float] = {}
        self._joint_vel: Dict[str, float] = {}
        self._joint_eff: Dict[str, float] = {}
        self._joint_wall = 0.0
        self._sdf_poses: Dict[str, tuple] = {}   # name→(x,y,z,qx,qy,qz,qw,wall)
        self._moveit_req_count = 0
        self._moveit_mode = "IDLE"   # IDLE / MOVEIT_ACTIVE / GRASP_ACTIVE

        # ---- TF ----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---- QoS ----
        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_rel = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---- Subscriptions ----
        self.create_subscription(JointState,          "/joint_states",                   self._cb_joints,          20)
        self.create_subscription(TFMessage,           "/world/ur5_mesa_objetos/pose/info",self._cb_sdf_poses,       qos_be)
        self.create_subscription(PoseStamped,         "/desired_grasp",                  self._cb_moveit_req,      qos_rel)
        self.create_subscription(PoseStamped,         "/desired_grasp_cartesian",        self._cb_moveit_cart,     qos_rel)
        self.create_subscription(String,              "/desired_grasp/result",           self._cb_moveit_result,   qos_rel)
        self.create_subscription(Bool,                "/ur5_moveit_bridge/heartbeat",    self._cb_bridge_hb,       qos_rel)
        self.create_subscription(Float64MultiArray,   "/gripper_controller/commands",    self._cb_gripper_cmd,     10)

        # ---- Timers ----
        self.create_timer(0.5,  self._snapshot_timer)    # full snapshot 2 Hz
        self.create_timer(30.0, self._tf_summary_timer)  # TF summary every 30 s

        self._write_full("=" * 72)
        self._write_full(f"[TRACE_LOGGER] START wall={time.strftime('%Y-%m-%d %H:%M:%S')}")
        self._write_full(f"[TRACE_LOGGER] log_dir={log_dir}")
        self._write_full("=" * 72)
        self.get_logger().info(f"[TRACE_LOGGER] logging to {log_dir}")

    # ---------------------------------------------------------------- writers
    def _write_full(self, msg: str) -> None:
        ts = f"{time.time():.3f}"
        line = f"[{ts}] {msg}"
        self._f_full.write(line + "\n")

    def _write_phase(self, msg: str) -> None:
        ts = f"{time.time():.3f}"
        line = f"[{ts}] {msg}"
        self._f_phases.write(line + "\n")
        self._f_full.write(line + "\n")
        self.get_logger().info(f"[TRACE_LOGGER][PHASE] {msg}")

    # ---------------------------------------------------------------- helpers
    def _fmt_pos(self, x, y, z) -> str:
        return f"({x:.4f},{y:.4f},{z:.4f})"

    def _fmt_quat(self, qx, qy, qz, qw) -> str:
        return f"({qx:.4f},{qy:.4f},{qz:.4f},{qw:.4f})"

    def _quat_to_rpy(self, qx, qy, qz, qw) -> Tuple[float, float, float]:
        """Quaternion → RPY in degrees."""
        sinr = 2*(qw*qx + qy*qz);   cosr = 1 - 2*(qx*qx + qy*qy)
        roll  = math.degrees(math.atan2(sinr, cosr))
        sinp  = 2*(qw*qy - qz*qx)
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))
        siny  = 2*(qw*qz + qx*qy);   cosy = 1 - 2*(qy*qy + qz*qz)
        yaw   = math.degrees(math.atan2(siny, cosy))
        return roll, pitch, yaw

    def _get_tf(self, parent: str, child: str) -> Optional[dict]:
        try:
            t = self._tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except Exception:
            return None
        now_ns = time.time_ns()
        stamp = t.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age_sec = (now_ns - stamp_ns) / 1e9
        tr = t.transform.translation
        rot = t.transform.rotation
        roll, pitch, yaw = self._quat_to_rpy(rot.x, rot.y, rot.z, rot.w)
        return {
            "parent": parent, "child": child,
            "tx": tr.x, "ty": tr.y, "tz": tr.z,
            "qx": rot.x, "qy": rot.y, "qz": rot.z, "qw": rot.w,
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "age_sec": age_sec,
        }

    # ---------------------------------------------------------------- callbacks
    def _cb_joints(self, msg: JointState) -> None:
        with self._lock:
            for i, name in enumerate(msg.name):
                pos = msg.position[i] if i < len(msg.position) else 0.0
                vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                eff = msg.effort[i]   if i < len(msg.effort)   else 0.0
                self._joint_pos[name] = float(pos)
                self._joint_vel[name] = float(vel)
                self._joint_eff[name] = float(eff)
            self._joint_wall = time.time()

        # Log immediately to CSV
        now = time.time()
        jp = self._joint_pos
        jv = self._joint_vel
        je = self._joint_eff
        dh = compute_dh_fk(jp)
        dh_vals = list(dh[:3]) + dh[3] if dh else [0]*6

        self._csv_joints.writerow([
            f"{now:.3f}",
            f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}",
            *[f"{jp.get(j, 0.0):.6f}" for j in _ARM_JOINTS],
            f"{jp.get('rg2_finger_joint1', 0.0):.6f}",
            f"{jp.get('rg2_finger_joint2', 0.0):.6f}",
            *[f"{jv.get(j, 0.0):.6f}" for j in _ARM_JOINTS],
            *[f"{je.get(j, 0.0):.6f}" for j in _ARM_JOINTS],
            *[f"{v:.6f}" for v in dh_vals],
        ])

        # Arm joint directions (sign of velocity)
        dirs = " ".join(
            f"{j.split('_')[0]}:{'+' if jv.get(j,0)>=0 else '-'}{abs(jv.get(j,0)):.3f}"
            for j in _ARM_JOINTS
        )
        self._write_full(
            f"[JOINTS] "
            f"pan={jp.get('shoulder_pan_joint',0):.4f} "
            f"lift={jp.get('shoulder_lift_joint',0):.4f} "
            f"elbow={jp.get('elbow_joint',0):.4f} "
            f"w1={jp.get('wrist_1_joint',0):.4f} "
            f"w2={jp.get('wrist_2_joint',0):.4f} "
            f"w3={jp.get('wrist_3_joint',0):.4f} "
            f"f1={jp.get('rg2_finger_joint1',0):.4f} "
            f"f2={jp.get('rg2_finger_joint2',0):.4f} "
            f"dirs=[{dirs}] "
            f"dh_tcp={self._fmt_pos(*dh[:3]) if dh else 'N/A'} "
            f"dh_rpy=({dh[3][0]:.2f},{dh[3][1]:.2f},{dh[3][2]:.2f}) "
            if dh else f"dh=N/A"
        )

    def _cb_sdf_poses(self, msg: TFMessage) -> None:
        now = time.time()
        with self._lock:
            for tf in msg.transforms:
                name = tf.child_frame_id
                t = tf.transform.translation
                r = tf.transform.rotation
                self._sdf_poses[name] = (t.x, t.y, t.z, r.x, r.y, r.z, r.w, now)

        for tf in msg.transforms:
            t = tf.transform.translation
            r = tf.transform.rotation
            roll, pitch, yaw = self._quat_to_rpy(r.x, r.y, r.z, r.w)
            self._write_full(
                f"[SDF] name={tf.child_frame_id} "
                f"pos={self._fmt_pos(t.x,t.y,t.z)} "
                f"quat={self._fmt_quat(r.x,r.y,r.z,r.w)} "
                f"rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})"
            )
            self._csv_sdf.writerow([
                f"{now:.3f}", tf.child_frame_id,
                f"{t.x:.6f}", f"{t.y:.6f}", f"{t.z:.6f}",
                f"{r.x:.6f}", f"{r.y:.6f}", f"{r.z:.6f}", f"{r.w:.6f}",
            ])

    def _cb_moveit_req(self, msg: PoseStamped) -> None:
        self._moveit_req_count += 1
        p = msg.pose.position
        o = msg.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(o.x, o.y, o.z, o.w)
        self._write_phase(
            f"[MODE=MOVEIT][PHASE_START] req_#{self._moveit_req_count} "
            f"frame={msg.header.frame_id} "
            f"pos={self._fmt_pos(p.x,p.y,p.z)} "
            f"quat={self._fmt_quat(o.x,o.y,o.z,o.w)} "
            f"rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})"
        )
        self._dump_full_state(tag="MOVEIT_REQ_START")

    def _cb_moveit_cart(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        o = msg.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(o.x, o.y, o.z, o.w)
        self._write_phase(
            f"[MODE=MOVEIT][CARTESIAN_START] "
            f"frame={msg.header.frame_id} "
            f"pos={self._fmt_pos(p.x,p.y,p.z)} "
            f"rpy=({roll:.2f},{pitch:.2f},{yaw:.2f})"
        )
        self._dump_full_state(tag="MOVEIT_CARTESIAN_START")

    def _cb_moveit_result(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            data = {"raw": msg.data}
        success = data.get("success", False)
        message = data.get("message", "")
        plan_ok = data.get("plan_ok", "?")
        exec_ok = data.get("exec_ok", "?")
        backend = data.get("backend", "?")
        self._write_phase(
            f"[MODE=MOVEIT][PHASE_END] "
            f"success={success} plan_ok={plan_ok} exec_ok={exec_ok} "
            f"backend={backend} msg={message}"
        )
        self._dump_full_state(tag="MOVEIT_RESULT_END")

    def _cb_bridge_hb(self, msg: Bool) -> None:
        self._write_full(f"[BRIDGE_HB] alive={msg.data}")

    def _cb_gripper_cmd(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        target = data[0] if data else None
        mode = "OPEN" if (target is not None and target > 0.5) else "CLOSE"
        with self._lock:
            f1 = self._joint_pos.get("rg2_finger_joint1", None)
            f2 = self._joint_pos.get("rg2_finger_joint2", None)
        pre_sum = (abs(f1) + abs(f2)) if (f1 is not None and f2 is not None) else None
        f1_str   = f"{f1:.4f}"   if f1   is not None else "N/A"
        f2_str   = f"{f2:.4f}"   if f2   is not None else "N/A"
        sum_str  = f"{pre_sum:.4f}" if pre_sum is not None else "N/A"
        self._write_phase(
            f"[GRIPPER_CMD] cmd={mode} target={target} data={data} "
            f"pre_finger1={f1_str} "
            f"pre_finger2={f2_str} "
            f"pre_opening_sum={sum_str}"
        )

    # ---------------------------------------------------------------- snapshot
    def _snapshot_timer(self) -> None:
        self._dump_full_state(tag="PERIODIC_0.5s")

    def _tf_summary_timer(self) -> None:
        self._dump_tf_all(tag="TF_SUMMARY_30s")

    def _dump_full_state(self, tag: str = "SNAPSHOT") -> None:
        now = time.time()
        with self._lock:
            jp = dict(self._joint_pos)
            jv = dict(self._joint_vel)
            je = dict(self._joint_eff)
            sdf = dict(self._sdf_poses)

        # ---- Joints ----
        arm_pos = {j: jp.get(j, 0.0) for j in _ARM_JOINTS}
        arm_vel = {j: jv.get(j, 0.0) for j in _ARM_JOINTS}
        arm_eff = {j: je.get(j, 0.0) for j in _ARM_JOINTS}
        f1 = jp.get("rg2_finger_joint1", 0.0)
        f2 = jp.get("rg2_finger_joint2", 0.0)

        # ---- DH FK ----
        dh = compute_dh_fk(jp)

        # ---- TF snapshot ----
        tf_results = {}
        for parent, child in _TF_PAIRS:
            tf_results[f"{parent}->{child}"] = self._get_tf(parent, child)

        # ---- SDF pick_demo ----
        pick = sdf.get("pick_demo")

        # ---- Write ----
        self._write_full(f"{'='*60}")
        self._write_full(f"[SNAPSHOT:{tag}] wall={now:.3f} time={time.strftime('%H:%M:%S')}")

        # Arm joints
        joint_str = " ".join(f"{j.split('_')[0]}:{v:.4f}" for j, v in arm_pos.items())
        vel_str   = " ".join(f"{j.split('_')[0]}:{v:+.4f}" for j, v in arm_vel.items())
        eff_str   = " ".join(f"{j.split('_')[0]}:{v:+.4f}" for j, v in arm_eff.items())
        self._write_full(f"[JOINTS_POS] {joint_str}")
        self._write_full(f"[JOINTS_VEL] {vel_str}")
        self._write_full(f"[JOINTS_EFF] {eff_str}")
        self._write_full(
            f"[GRIPPER]   f1={f1:.4f} f2={f2:.4f} "
            f"opening_sum={abs(f1)+abs(f2):.4f} "
            f"state={'OPEN' if abs(f1)+abs(f2) > 0.5 else 'CLOSED/PARTIAL'}"
        )

        # DH FK
        if dh:
            self._write_full(
                f"[DH_FK] tool0_pos={self._fmt_pos(dh[0],dh[1],dh[2])} "
                f"rpy=({dh[3][0]:.3f},{dh[3][1]:.3f},{dh[3][2]:.3f}) deg"
            )
        else:
            self._write_full("[DH_FK] N/A")

        # TF
        for key, tf in tf_results.items():
            if tf:
                self._write_full(
                    f"[TF] {tf['parent']}->{tf['child']} "
                    f"pos={self._fmt_pos(tf['tx'],tf['ty'],tf['tz'])} "
                    f"quat={self._fmt_quat(tf['qx'],tf['qy'],tf['qz'],tf['qw'])} "
                    f"rpy=({tf['roll']:.2f},{tf['pitch']:.2f},{tf['yaw']:.2f}) "
                    f"age={tf['age_sec']:.3f}s"
                )
                self._csv_tf.writerow([
                    f"{now:.3f}", tf['parent'], tf['child'],
                    f"{tf['tx']:.6f}", f"{tf['ty']:.6f}", f"{tf['tz']:.6f}",
                    f"{tf['qx']:.6f}", f"{tf['qy']:.6f}", f"{tf['qz']:.6f}", f"{tf['qw']:.6f}",
                    f"{tf['age_sec']:.3f}",
                ])
            else:
                self._write_full(f"[TF] {key} N/A")

        # SDF pick_demo
        if pick:
            tx, ty, tz, qx, qy, qz, qw, stamp = pick
            age = now - stamp
            roll, pitch, yaw = self._quat_to_rpy(qx, qy, qz, qw)
            self._write_full(
                f"[SDF_SNAPSHOT] pick_demo "
                f"pos={self._fmt_pos(tx,ty,tz)} "
                f"quat={self._fmt_quat(qx,qy,qz,qw)} "
                f"rpy=({roll:.2f},{pitch:.2f},{yaw:.2f}) "
                f"age={age:.3f}s"
            )

        # DH vs TF divergence (tool0)
        tf_t0 = tf_results.get("base_link->tool0")
        if dh and tf_t0:
            dist = math.sqrt(
                (dh[0]-tf_t0['tx'])**2 +
                (dh[1]-tf_t0['ty'])**2 +
                (dh[2]-tf_t0['tz'])**2
            )
            self._write_full(
                f"[DH_VS_TF] dh_tool0={self._fmt_pos(dh[0],dh[1],dh[2])} "
                f"tf_tool0={self._fmt_pos(tf_t0['tx'],tf_t0['ty'],tf_t0['tz'])} "
                f"dist_m={dist:.4f} "
                f"verdict={'OK' if dist < 0.015 else 'DIVERGE'}"
            )

        self._write_full(f"{'='*60}")

    def _dump_tf_all(self, tag: str = "TF_ALL") -> None:
        now = time.time()
        self._write_full(f"[TF_ALL:{tag}] wall={now:.3f}")
        for parent, child in _TF_PAIRS:
            tf = self._get_tf(parent, child)
            if tf:
                self._write_full(
                    f"  {parent}->{child}: "
                    f"pos={self._fmt_pos(tf['tx'],tf['ty'],tf['tz'])} "
                    f"rpy=({tf['roll']:.2f},{tf['pitch']:.2f},{tf['yaw']:.2f}) "
                    f"age={tf['age_sec']:.3f}s"
                )
            else:
                self._write_full(f"  {parent}->{child}: N/A")


def main() -> None:
    rclpy.init()

    # Log dir: read from env or default to /tmp/ros_trace_TIMESTAMP
    log_dir = os.environ.get(
        "ROS_TRACE_LOG_DIR",
        f"/tmp/ros_trace_{time.strftime('%Y%m%d_%H%M%S')}"
    )
    node = RosTraceLogger(log_dir)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node._write_full("[TRACE_LOGGER] STOP")
        node.get_logger().info("[TRACE_LOGGER] stopped")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
