#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/scripts/debug_grasp_geometry.py
"""Diagnóstico geométrico runtime UR5+RG2 — sin mover el robot.

Imprime JSON con:
- timestamp_ros / clock
- TF críticos: world→base_link, base_link→tool0, tool0→rg2_pinch_center,
  tool0→rg2_tcp, tool0→rg2_finger_link1/2, tool0→pick_demo_anchor
- Pose del objeto pick_demo en world (de /tf si gz_pose_bridge lo bridge-a)
- Pose del objeto en base_link (transformada)
- Pose tool0/rg2_pinch_center en base_link
- Distancias TCP-objeto: xy_error, z_error, dist3d
- Target grasp = objeto + (0, 0, 0.05) en base_link
- Warnings sobre frescura/discrepancia

Uso:
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 scripts/debug_grasp_geometry.py [--object pick_demo] [--out /path/to/out.json]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import tf2_ros
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_msgs.msg import TFMessage


def tf_to_dict(tf: TransformStamped) -> Dict:
    return {
        "frame_id": tf.header.frame_id,
        "child_frame_id": tf.child_frame_id,
        "stamp_sec": tf.header.stamp.sec,
        "stamp_nanosec": tf.header.stamp.nanosec,
        "translation": {
            "x": tf.transform.translation.x,
            "y": tf.transform.translation.y,
            "z": tf.transform.translation.z,
        },
        "rotation_quat": {
            "x": tf.transform.rotation.x,
            "y": tf.transform.rotation.y,
            "z": tf.transform.rotation.z,
            "w": tf.transform.rotation.w,
        },
    }


class GraspDiag(Node):
    def __init__(self, object_name: str = "pick_demo"):
        super().__init__("grasp_geometry_diag")
        # use_sim_time
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        self.object_name = object_name
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # gz_pose_bridge publica TFMessage en /world/<world>/pose/info — suscribimos
        # con QoS RELIABLE para registrar la pose live del objeto.
        self.world_object_pose: Optional[Dict] = None
        self._object_warnings = []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            TFMessage, "/world/ur5_mesa_objetos/pose/info",
            self._on_world_pose_info, qos
        )
        self.create_subscription(
            TFMessage, "/tf", self._on_tf, 100
        )
        # Cache local de pose pick_demo desde /tf si está bridgeada
        self._tf_object_pose: Optional[Dict] = None

    def _on_world_pose_info(self, msg: TFMessage):
        for tr in msg.transforms:
            if tr.child_frame_id == self.object_name:
                self.world_object_pose = tf_to_dict(tr)

    def _on_tf(self, msg: TFMessage):
        for tr in msg.transforms:
            if tr.child_frame_id == self.object_name:
                self._tf_object_pose = tf_to_dict(tr)

    def get_tf(self, source: str, target: str) -> Optional[Dict]:
        try:
            tr = self.tf_buffer.lookup_transform(source, target, rclpy.time.Time(), Duration(seconds=2.0))
            return tf_to_dict(tr)
        except Exception as exc:
            return {"error": f"tf_lookup_failed: {source}→{target}: {exc}"}

    def get_object_pose_world(self) -> Optional[Dict]:
        # Prefer live world_pose_info, fallback to /tf
        if self.world_object_pose is not None:
            return self.world_object_pose
        return self._tf_object_pose

    def transform_point_to_base(self, world_xyz: Dict[str, float]) -> Optional[Dict]:
        # world → base_link
        try:
            tr = self.tf_buffer.lookup_transform("base_link", "world", rclpy.time.Time(), Duration(seconds=2.0))
            tx = tr.transform.translation.x
            ty = tr.transform.translation.y
            tz = tr.transform.translation.z
            qx = tr.transform.rotation.x
            qy = tr.transform.rotation.y
            qz = tr.transform.rotation.z
            qw = tr.transform.rotation.w
            # rotación cuat * vec
            x, y, z = world_xyz["x"], world_xyz["y"], world_xyz["z"]
            # (q * (x,y,z,0) * q^-1).vec — fórmula sin librería
            ix = qw * x + qy * z - qz * y
            iy = qw * y + qz * x - qx * z
            iz = qw * z + qx * y - qy * x
            iw = -qx * x - qy * y - qz * z
            rx = ix * qw + iw * (-qx) + iy * (-qz) - iz * (-qy)
            ry = iy * qw + iw * (-qy) + iz * (-qx) - ix * (-qz)
            rz = iz * qw + iw * (-qz) + ix * (-qy) - iy * (-qx)
            return {
                "x": rx + tx,
                "y": ry + ty,
                "z": rz + tz,
            }
        except Exception as exc:
            return {"error": f"transform_world_to_base failed: {exc}"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--object", default="pick_demo")
    parser.add_argument("--out", default=None)
    parser.add_argument("--wait", type=float, default=4.0,
                        help="segundos a esperar antes de capturar")
    args = parser.parse_args()

    rclpy.init()
    node = GraspDiag(object_name=args.object)
    try:
        # spin breve para llenar TF buffer y suscripciones
        end_t = time.monotonic() + args.wait
        while time.monotonic() < end_t and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

        warnings = []
        out = {
            "iso_wall_ts": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "monotonic_ts": time.monotonic(),
            "object_name": args.object,
            "tfs": {},
            "object_world": None,
            "object_base_link": None,
            "tcp_object_xy_error_m": None,
            "tcp_object_z_error_m": None,
            "tcp_object_dist3d_m": None,
            "target_grasp_base_link": None,
            "warnings": warnings,
        }
        # /clock
        try:
            now_ros = node.get_clock().now()
            out["clock_ros_sec"] = now_ros.nanoseconds / 1e9
        except Exception as exc:
            warnings.append(f"clock_unavailable: {exc}")

        # TF críticos
        for src, tgt in [
            ("world", "base_link"),
            ("base_link", "tool0"),
            ("tool0", "rg2_pinch_center"),
            ("tool0", "rg2_tcp"),
            ("tool0", "rg2_finger_link1"),
            ("tool0", "rg2_finger_link2"),
            ("tool0", "pick_demo_anchor"),
            ("base_link", "rg2_pinch_center"),
            ("base_link", f"{args.object}"),
        ]:
            key = f"{src}__{tgt}"
            res = node.get_tf(src, tgt)
            out["tfs"][key] = res
            if res and "error" in res:
                warnings.append(res["error"])

        # Pose objeto en world
        obj_world = node.get_object_pose_world()
        if obj_world is None:
            warnings.append("object_pose_not_available_via_world_tf_or_pose_info")
        else:
            out["object_world"] = obj_world
            # Transform to base_link
            obj_base = node.transform_point_to_base(obj_world["translation"])
            out["object_base_link"] = obj_base

        # tool0 en base_link (TCP teórico arm)
        tool0_in_base = out["tfs"].get("base_link__tool0")
        pinch_in_base = out["tfs"].get("base_link__rg2_pinch_center")

        # Calcular error TCP-objeto si tenemos pinch_center y objeto en base_link
        if obj_world and isinstance(out["object_base_link"], dict) and "x" in out["object_base_link"] \
                and pinch_in_base and isinstance(pinch_in_base, dict) and "translation" in pinch_in_base:
            ox = out["object_base_link"]["x"]
            oy = out["object_base_link"]["y"]
            oz = out["object_base_link"]["z"]
            px = pinch_in_base["translation"]["x"]
            py = pinch_in_base["translation"]["y"]
            pz = pinch_in_base["translation"]["z"]
            xy = math.hypot(px - ox, py - oy)
            z_err = pz - oz
            d3 = math.sqrt((px - ox) ** 2 + (py - oy) ** 2 + (pz - oz) ** 2)
            out["tcp_object_xy_error_m"] = xy
            out["tcp_object_z_error_m"] = z_err
            out["tcp_object_dist3d_m"] = d3
            out["target_grasp_base_link"] = {
                "x": ox,
                "y": oy,
                "z": oz + 0.0,  # contact target = object center
                "comment": "z=object_center; ajustar +0.025 para top, -0.025 para bottom según convención TCP-down",
            }

        # Resultado
        text = json.dumps(out, indent=2)
        print(text)
        if args.out:
            with open(args.out, "w") as f:
                f.write(text)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
