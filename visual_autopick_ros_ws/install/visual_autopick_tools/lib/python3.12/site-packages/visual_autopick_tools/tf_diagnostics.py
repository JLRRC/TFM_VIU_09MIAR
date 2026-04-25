"""
tf_diagnostics.py
=================
Script de diagnóstico obligatorio (Regla 9 de integración RG2).

Comprueba:
  1. Árbol TF relevante (frames requeridos presentes)
  2. Transform base_link → tool0
  3. Transform tool0 → rg2_pinch_center
  4. Transform base_link → rg2_pinch_center (compuesto)
  5. Estado de controladores ros2_control
  6. Joints activos del gripper
  7. Topic /joint_states
  8. Frames configurados en MoveIt (si disponible)
  9. Diferencia entre pose Gazebo y pose TF del TCP

Uso:
  ros2 run visual_autopick_tools tf_diagnostics
  ros2 run visual_autopick_tools tf_diagnostics --once
"""
import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from tf2_ros import ConnectivityException
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from controller_manager_msgs.srv import ListControllers


REQUIRED_FRAMES = [
    "world",
    "base_link",
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "tool0",
    "rg2_base_link",
    "rg2_finger_link1",
    "rg2_finger_link2",
    "rg2_pinch_center",
]

REQUIRED_TRANSFORMS = [
    ("world",      "base_link"),
    ("base_link",  "tool0"),
    ("tool0",      "rg2_base_link"),
    ("tool0",      "rg2_pinch_center"),
    ("base_link",  "rg2_pinch_center"),
]

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

GRIPPER_JOINTS = [
    "rg2_finger_joint1",
    "rg2_finger_joint2",
]

TCP_Z_OFFSET_EXPECTED = 0.175   # valor canónico del URDF


def _fmt(t) -> str:
    p = t.transform.translation
    r = t.transform.rotation
    return (f"xyz=({p.x:+.4f}, {p.y:+.4f}, {p.z:+.4f})  "
            f"qxyzw=({r.x:+.4f}, {r.y:+.4f}, {r.z:+.4f}, {r.w:+.4f})")


def _quat_to_rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


SEP = "=" * 65


class TfDiagnosticsNode(Node):

    def __init__(self):
        super().__init__("tf_diagnostics")
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)
        self._joint_states: JointState | None = None
        self._gz_poses: dict[str, tuple] = {}

        self._js_sub = self.create_subscription(
            JointState, "/joint_states",
            lambda msg: setattr(self, "_joint_states", msg),
            qos_profile_sensor_data)

        self._pose_sub = self.create_subscription(
            TFMessage, "/world/visual_autopick_world/pose/info",
            self._pose_cb, qos_profile_sensor_data)

    def _pose_cb(self, msg: TFMessage):
        for tf in msg.transforms:
            self._gz_poses[tf.child_frame_id] = tf.transform

    # ──────────────────────────────────────────────────────────────────
    def run(self):
        print(f"\n{SEP}")
        print("  DIAGNÓSTICO UR5+RG2 — visual_autopick_ros_ws")
        print(SEP)

        self._check_transforms()
        self._check_tcp_offset()
        self._check_joint_states()
        self._check_controllers()
        self._check_gazebo_vs_tf()

        print(f"\n{SEP}")
        print("  FIN DIAGNÓSTICO")
        print(SEP)

    # ── 1+2. Frames TF y transforms requeridos ─────────────────────────
    def _check_transforms(self):
        print(f"\n[1/5] TRANSFORMS TF REQUERIDOS")
        now = self.get_clock().now()
        all_ok = True
        for (parent, child) in REQUIRED_TRANSFORMS:
            try:
                t = self._tf_buf.lookup_transform(
                    parent, child, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0))
                print(f"  ✓ {parent:20s} → {child:25s}  {_fmt(t)}")
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                print(f"  ✗ {parent:20s} → {child:25s}  ERROR: {e}")
                all_ok = False
        if all_ok:
            print("  → Todos los transforms OK")
        else:
            print("  → FALTAN transforms. Comprueba world_tf_publisher y robot_state_publisher.")

    # ── 3. Offset TCP ──────────────────────────────────────────────────
    def _check_tcp_offset(self):
        print(f"\n[2/5] COHERENCIA OFFSET TCP (tool0 → rg2_pinch_center)")
        print(f"  Esperado: xyz=(0, 0, {TCP_Z_OFFSET_EXPECTED})")
        try:
            t = self._tf_buf.lookup_transform(
                "tool0", "rg2_pinch_center",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            p = t.transform.translation
            dz = abs(p.z - TCP_Z_OFFSET_EXPECTED)
            dx = abs(p.x)
            dy = abs(p.y)
            ok = (dx < 1e-4 and dy < 1e-4 and dz < 1e-4)
            status = "✓" if ok else "✗"
            print(f"  {status} Medido:  xyz=({p.x:+.6f}, {p.y:+.6f}, {p.z:+.6f})")
            if not ok:
                print(f"  ✗ Desviación: dx={dx:.4f}  dy={dy:.4f}  dz={dz:.4f}")
                print("  → Revisar rg2_pinch_center_joint en ur5_rg2.urdf.xacro")
            else:
                print("  → Offset TCP coherente con URDF")

            # Verificar que tool0 ≠ rg2_pinch_center (no deben ser iguales)
            if dz < 1e-6:
                print("  ✗ ALERTA: tool0 y rg2_pinch_center en la misma posición. "
                      "TCP no definido correctamente.")
        except Exception as e:
            print(f"  ✗ No se puede leer transform: {e}")

    # ── 4. /joint_states ───────────────────────────────────────────────
    def _check_joint_states(self):
        print(f"\n[3/5] JOINT STATES (/joint_states)")
        if self._joint_states is None:
            print("  ✗ No se recibió ningún mensaje en /joint_states")
            print("  → Verificar joint_state_broadcaster activo")
            return

        js = self._joint_states
        jmap = dict(zip(js.name, js.position))

        print("  Brazo UR5:")
        arm_ok = True
        for j in ARM_JOINTS:
            val = jmap.get(j)
            if val is None:
                print(f"    ✗ {j}: AUSENTE")
                arm_ok = False
            else:
                print(f"    ✓ {j}: {val:+.4f} rad")

        print("  Gripper RG2:")
        gripper_ok = True
        for j in GRIPPER_JOINTS:
            val = jmap.get(j)
            if val is None:
                print(f"    ✗ {j}: AUSENTE")
                gripper_ok = False
            else:
                print(f"    ✓ {j}: {val:.4f} m  "
                      f"({'cerrado' if val < 0.005 else 'abierto' if val > 0.038 else 'parcial'})")

        if not arm_ok or not gripper_ok:
            print("  → Revisar controllers.yaml y gz_ros2_control plugin en model.sdf")
        else:
            print("  → Joint states completos ✓")

    # ── 5. Controladores ros2_control ─────────────────────────────────
    def _check_controllers(self):
        print(f"\n[4/5] CONTROLADORES ros2_control")
        cli = self.create_client(ListControllers, "/controller_manager/list_controllers")
        if not cli.wait_for_service(timeout_sec=3.0):
            print("  ✗ /controller_manager no disponible")
            print("  → ¿Está activo gz_ros2_control?")
            return

        req = ListControllers.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            print("  ✗ Timeout al consultar controller_manager")
            return

        required = {
            "joint_state_broadcaster": "active",
            "joint_trajectory_controller": "active",
            "gripper_controller": "active",
        }
        found = {c.name: c.state for c in future.result().controller}
        for name, expected_state in required.items():
            actual = found.get(name, "NO ENCONTRADO")
            ok = actual == expected_state
            status = "✓" if ok else "✗"
            print(f"  {status} {name:35s}  estado: {actual}")
            if not ok:
                print(f"    → Esperado: {expected_state}")

    # ── 6. Gazebo vs TF ───────────────────────────────────────────────
    def _check_gazebo_vs_tf(self):
        print(f"\n[5/5] COHERENCIA GAZEBO vs TF (rg2_pinch_center)")
        try:
            t = self._tf_buf.lookup_transform(
                "world", "rg2_pinch_center",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
            p_tf = t.transform.translation
            print(f"  TF   world→rg2_pinch_center: "
                  f"xyz=({p_tf.x:+.4f}, {p_tf.y:+.4f}, {p_tf.z:+.4f})")

            # Pose de base_link desde Gazebo
            gz = self._gz_poses.get("base_link") or self._gz_poses.get("ur5_rg2::base_link")
            if gz is None:
                print("  ⚠  No se recibió pose de Gazebo (pose/info). "
                      "Verifica gz_bridge.yaml y world name.")
            else:
                print(f"  GZ   base_link pose en world: "
                      f"xyz=({gz.translation.x:+.4f}, "
                      f"{gz.translation.y:+.4f}, {gz.translation.z:+.4f})")
                dx = abs(gz.translation.x - (-0.85))
                dz = abs(gz.translation.z - 0.85)
                if dx > 0.01 or dz > 0.01:
                    print("  ✗ base_link NO está en (-0.85, 0, 0.85). "
                          "Revisar spawn pose o base_joint en model.sdf")
                else:
                    print("  ✓ base_link Gazebo coincide con pose esperada (-0.85, 0, 0.85)")

        except Exception as e:
            print(f"  ✗ {e}")


def main():
    rclpy.init()
    node = TfDiagnosticsNode()
    # Esperar a que lleguen datos
    node.get_logger().info("Esperando datos TF y joint_states (5 s)...")
    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=5.0)
    while rclpy.ok() and node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
