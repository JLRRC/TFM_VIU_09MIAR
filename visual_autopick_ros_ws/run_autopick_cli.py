#!/usr/bin/env python3
"""run_autopick_cli.py — Ejecuta secuencia autopick completa desde CLI sin GUI."""
import json, math, os, sys, threading, time, itertools

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

LOG_DIR = os.environ.get("ROS_TRACE_LOG_DIR", "/tmp/autopick_cli")
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = open(os.path.join(LOG_DIR, "panel.log"), "w", buffering=1)

def log(msg):
    ts = time.strftime("%H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line)
    LOG_FILE.write(line + "\n")

# ---- targets (base_link) precomputados ----
APPROACH = (0.436, 0.000, 0.060)
GRASP    = (0.436, 0.000, 0.030)
LIFT     = (0.436, 0.000, 0.105)

class AutopickRunner(Node):
    def __init__(self):
        super().__init__("autopick_cli_runner")
        self._rid_counter = itertools.count(1)
        qos_rel = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        self._pub   = self.create_publisher(PoseStamped, "/desired_grasp", qos_rel)
        self._gpub  = self.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)
        self._result: dict = {}
        self._result_event = threading.Event()
        self._current_rid: int = -1  # RID de la peticion en curso
        self._joint_pos = {}
        self._sdf_poses = {}
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)
        self.create_subscription(String,     "/desired_grasp/result",              self._cb_result,  qos_rel)
        self.create_subscription(JointState, "/joint_states",                      self._cb_js,      20)
        self.create_subscription(TFMessage,  "/world/ur5_mesa_objetos/pose/info",  self._cb_sdf,     10)

    def _cb_result(self, msg):
        try: data = json.loads(msg.data)
        except: data = {}
        rid = data.get("request_id", -1)
        # Solo aceptar resultado si coincide con el RID que publicamos
        if rid == self._current_rid and rid >= 0:
            self._result = data
            self._result_event.set()

    def _cb_js(self, msg):
        for i,n in enumerate(msg.name):
            self._joint_pos[n] = float(msg.position[i]) if i < len(msg.position) else 0.0

    def _cb_sdf(self, msg):
        for tf in msg.transforms:
            t = tf.transform.translation
            self._sdf_poses[tf.child_frame_id] = (t.x, t.y, t.z)

    def _get_tcp(self):
        try:
            t = self._tf_buf.lookup_transform("base_link","rg2_pinch_center", rclpy.time.Time())
            tr = t.transform.translation
            return (tr.x, tr.y, tr.z)
        except: return None

    def _dist(self, a, b):
        return math.sqrt(sum((ai-bi)**2 for ai,bi in zip(a,b))) if a and b else float("inf")

    def move(self, target, phase, timeout=35.0):
        # Drenar resultados cacheados del topic antes de publicar
        self._result = {}
        self._result_event.clear()
        self._current_rid = -1  # bloquear cualquier resultado entrante
        time.sleep(0.3)  # dar tiempo a callbacks pendientes
        self._result = {}
        self._result_event.clear()

        rid = next(self._rid_counter)
        self._current_rid = rid  # habilitar filtro antes de publicar

        msg = PoseStamped()
        msg.header.frame_id = f"base_link|rid={rid}"
        msg.header.stamp = self.get_clock().now().to_msg()
        req_sec = msg.header.stamp.sec
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
        msg.pose.orientation.w = 1.0

        log(f"[VISUAL_AUTOPICK][MOVE_REQUEST] phase={phase} rid={rid} tcp_frame=rg2_pinch_center target_base=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})")

        # Dump joints before move
        jp = self._joint_pos
        log(f"[JOINTS_PRE_{phase}] " + " ".join(f"{k.split('_')[0]}:{jp.get(k,0):.4f}"
            for k in ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                       "wrist_1_joint","wrist_2_joint","wrist_3_joint"]))
        f1 = jp.get("rg2_finger_joint1",0); f2 = jp.get("rg2_finger_joint2",0)
        log(f"[GRIPPER_PRE_{phase}] f1={f1:.4f} f2={f2:.4f} sum={abs(f1)+abs(f2):.4f}")
        tcp_pre = self._get_tcp()
        if tcp_pre:
            log(f"[TCP_PRE_{phase}] base=({tcp_pre[0]:.3f},{tcp_pre[1]:.3f},{tcp_pre[2]:.3f})")
        pd = self._sdf_poses.get("pick_demo")
        if pd:
            log(f"[SDF_PRE_{phase}] pick_demo world=({pd[0]:.3f},{pd[1]:.3f},{pd[2]:.3f})")

        # Resetear ANTES de publicar (el filtro ya esta activo via _current_rid)
        self._result = {}
        self._result_event.clear()

        self._pub.publish(msg)
        t_pub = time.monotonic()

        # Esperar resultado que coincida con nuestro RID
        fired = False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            fired = self._result_event.wait(timeout=0.5)
            if fired:
                break

        tcp_post = self._get_tcp()
        dist = self._dist(tcp_post, target) if tcp_post else None
        jp2 = self._joint_pos
        log(f"[JOINTS_POST_{phase}] " + " ".join(f"{k.split('_')[0]}:{jp2.get(k,0):.4f}"
            for k in ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                       "wrist_1_joint","wrist_2_joint","wrist_3_joint"]))
        success = bool(self._result.get("success", False)) if fired else False
        plan_ok = self._result.get("plan_ok", "?")
        exec_ok = self._result.get("exec_ok", "?")
        msg_str = self._result.get("message", "timeout") if fired else "timeout"
        tcp_str = f"({tcp_post[0]:.3f},{tcp_post[1]:.3f},{tcp_post[2]:.3f})" if tcp_post else "N/A"
        dist_str = f"{dist:.4f}" if dist is not None else "N/A"
        log(f"[VISUAL_AUTOPICK][MOVE_RESULT] phase={phase} success={success} "
            f"plan_ok={plan_ok} exec_ok={exec_ok} error={msg_str} "
            f"tcp_after={tcp_str} dist_to_target={dist_str}")
        return success

    def gripper(self, close=True):
        f1_pre = self._joint_pos.get("rg2_finger_joint1",0)
        f2_pre = self._joint_pos.get("rg2_finger_joint2",0)
        pre_sum = abs(f1_pre) + abs(f2_pre)
        target  = 0.0 if close else 1.0
        cmd = "CLOSE" if close else "OPEN"
        log(f"[VISUAL_AUTOPICK][GRIPPER] command={cmd.lower()} target={target} "
            f"pre_f1={f1_pre:.4f} pre_f2={f2_pre:.4f} pre_sum={pre_sum:.4f}")
        msg = Float64MultiArray()
        msg.data = [float(target), float(target)]
        self._gpub.publish(msg)
        time.sleep(3.5)
        f1_post = self._joint_pos.get("rg2_finger_joint1",0)
        f2_post = self._joint_pos.get("rg2_finger_joint2",0)
        post_sum = abs(f1_post) + abs(f2_post)
        delta = pre_sum - post_sum
        verdict = "CLOSED" if (close and delta >= 0.01) else ("OPEN" if not close else "FAILED")
        log(f"[VISUAL_AUTOPICK][GRIPPER] post_f1={f1_post:.4f} post_f2={f2_post:.4f} "
            f"post_sum={post_sum:.4f} delta={delta:.4f} verdict={verdict}")
        return verdict not in ("FAILED",)


def main():
    rclpy.init()
    node = AutopickRunner()
    exec_ = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    exec_.add_node(node)
    spin_t = threading.Thread(target=exec_.spin, daemon=True)
    spin_t.start()
    time.sleep(2)  # allow DDS discovery

    log("=" * 60)
    log(f"[VISUAL_AUTOPICK] INICIO SECUENCIA MOVEIT CLI")
    log(f"[VISUAL_AUTOPICK] log_dir={LOG_DIR}")
    log(f"[VISUAL_AUTOPICK][TARGETS] approach={APPROACH} grasp={GRASP} lift={LIFT}")
    log("=" * 60)

    # --- FASE 0: estado inicial ---
    jp = node._joint_pos
    log(f"[FASE_0_ESTADO_INICIAL] joints=" + str({k: round(v,4) for k,v in jp.items() if k in
        ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]}))
    pd = node._sdf_poses.get("pick_demo")
    if pd: log(f"[FASE_0_ESTADO_INICIAL] pick_demo_world=({pd[0]:.3f},{pd[1]:.3f},{pd[2]:.3f})")
    tcp0 = node._get_tcp()
    if tcp0: log(f"[FASE_0_ESTADO_INICIAL] tcp_base=({tcp0[0]:.3f},{tcp0[1]:.3f},{tcp0[2]:.3f})")

    result = "SUCCESS"

    # --- FASE 1: GRIPPER OPEN ---
    log("[VISUAL_AUTOPICK][PHASE] -> OPEN_GRIPPER")
    node.gripper(close=False)

    # --- FASE 2: APPROACH ---
    log("[VISUAL_AUTOPICK][PHASE] -> MOVE_APPROACH")
    if not node.move(APPROACH, "APPROACH"):
        log("[VISUAL_AUTOPICK][DONE] result=FAIL reason=APPROACH_FAILED object=pick_demo")
        result = "FAIL"

    if result == "SUCCESS":
        # --- FASE 3: GRASP ---
        log("[VISUAL_AUTOPICK][PHASE] -> MOVE_GRASP")
        if not node.move(GRASP, "GRASP"):
            log("[VISUAL_AUTOPICK][DONE] result=FAIL reason=GRASP_FAILED object=pick_demo")
            result = "FAIL"

    if result == "SUCCESS":
        # --- FASE 4: CLOSE GRIPPER ---
        log("[VISUAL_AUTOPICK][PHASE] -> CLOSE_GRIPPER")
        node.gripper(close=True)

        # --- FASE 5: LIFT ---
        log("[VISUAL_AUTOPICK][PHASE] -> LIFT")
        if not node.move(LIFT, "LIFT"):
            log("[VISUAL_AUTOPICK][DONE] result=FAIL reason=LIFT_FAILED object=pick_demo")
            result = "FAIL"

    # --- FIN ---
    log("=" * 60)
    log(f"[VISUAL_AUTOPICK][DONE] result={result} object=pick_demo")
    log("=" * 60)

    time.sleep(1)
    exec_.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    LOG_FILE.flush()
    LOG_FILE.close()

if __name__ == "__main__":
    main()
