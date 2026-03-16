#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_object_z_sanity.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Sanity check for object Z heights from /world/*/pose/info."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


TARGET_NAMES = ("box_yellow", "box_red", "cyl_purple")


def _resolve_name(raw_name: str, known: set[str]) -> Optional[str]:
    if not raw_name:
        return None
    if raw_name in known:
        return raw_name
    parts = [p for p in raw_name.split("::") if p]
    for token in reversed(parts):
        token = token.strip()
        if token in known:
            return token
    tail = raw_name.split("/")[-1].strip()
    if tail in known:
        return tail
    return None


class PoseInfoProbe(Node):
    def __init__(self) -> None:
        super().__init__("test_object_z_sanity_probe")
        self._snapshot: Dict[str, Tuple[float, float, float]] = {}
        self._raw_source: Dict[str, str] = {}
        self._sub = None

    def discover_pose_topic(self, timeout_sec: float = 3.0) -> Optional[str]:
        deadline = time.time() + max(0.5, timeout_sec)
        while time.time() < deadline:
            topics = self.get_topic_names_and_types()
            candidates = sorted(
                name
                for name, _types in topics
                if name.startswith("/world/") and name.endswith("/pose/info")
            )
            if candidates:
                return candidates[0]
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def start(self, topic: str, known: set[str]) -> None:
        def _on_msg(msg: TFMessage) -> None:
            for tf in getattr(msg, "transforms", []):
                raw_name = str(getattr(tf, "child_frame_id", "") or "")
                key = _resolve_name(raw_name, known)
                if not key:
                    continue
                tr = tf.transform.translation
                sample = (float(tr.x), float(tr.y), float(tr.z))
                prev = self._snapshot.get(key)
                # Prefer lower-Z representative when multiple links exist.
                if prev is None or sample[2] < prev[2]:
                    self._snapshot[key] = sample
                    self._raw_source[key] = raw_name

        self._sub = self.create_subscription(
            TFMessage, topic, _on_msg, qos_profile_sensor_data
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--targets",
        nargs="*",
        default=list(TARGET_NAMES),
        help="Objects to validate (default: box_yellow box_red cyl_purple)",
    )
    args = parser.parse_args()

    known = set(args.targets) | {"pick_demo"}
    rclpy.init()
    node = PoseInfoProbe()
    try:
        topic = node.discover_pose_topic(timeout_sec=min(3.0, float(args.timeout)))
        if not topic:
            print("[FAIL] No topic /world/*/pose/info visible")
            return 1
        node.start(topic, known)
        deadline = time.time() + max(0.5, float(args.timeout))
        while time.time() < deadline and len(node._snapshot) < 2:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node._snapshot:
            print(f"[FAIL] Sin datos en {topic} tras {args.timeout:.1f}s")
            return 1

        print(f"[INFO] pose_topic={topic}")
        for name in sorted(node._snapshot.keys()):
            x, y, z = node._snapshot[name]
            src = node._raw_source.get(name, name)
            print(f"[POSE] {name} z={z:.3f} xyz=({x:.3f},{y:.3f},{z:.3f}) source={src}")

        pick_z = None
        if "pick_demo" in node._snapshot:
            pick_z = node._snapshot["pick_demo"][2]

        offenders = []
        checked = []
        for name in args.targets:
            pose = node._snapshot.get(name)
            if pose is None:
                continue
            z = pose[2]
            checked.append((name, z))
            if z >= 1.2:
                offenders.append((name, z))

        high_drop = []
        if pick_z is not None and math.isfinite(pick_z) and 0.70 <= pick_z <= 0.90:
            for name, (x, y, z) in node._snapshot.items():
                if (name.startswith("box_") or name.startswith("cyl_")) and z >= 2.5:
                    high_drop.append((name, x, y, z))

        if len(checked) < 3:
            print(
                f"[FAIL] Solo se pudieron validar {len(checked)} target(s) de {len(args.targets)}"
            )
            return 1
        if high_drop:
            print("[FAIL] Detectado patrón anómalo: box_/cyl_ a z~2.775 con pick_demo en mesa")
            for name, x, y, z in sorted(high_drop):
                print(f"[OFFENDER] {name} xyz=({x:.3f},{y:.3f},{z:.3f})")
            return 1
        if offenders:
            print("[FAIL] Objetos target fuera de rango esperado (z >= 1.2)")
            for name, z in offenders:
                print(f"[OFFENDER] {name} z={z:.3f}")
            return 1

        print("[PASS] z_sanity OK: objetos target en rango de mesa")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
