#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_object_pose_sanity_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Sanity-check object world poses from /world/*/pose/info using rclpy only."""

from __future__ import annotations

import argparse
import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


DEFAULT_TARGETS = ("box_yellow", "box_red", "cyl_purple")


def _resolve_object_name(raw_name: str, known: set[str]) -> Optional[str]:
    name = (raw_name or "").strip()
    if not name:
        return None
    if name in known:
        return name
    parts = [p.strip() for p in name.split("::") if p.strip()]
    for token in reversed(parts):
        if token in known:
            return token
    tail = name.split("/")[-1].strip()
    if tail in known:
        return tail
    return None


class PoseInfoProbe(Node):
    def __init__(self) -> None:
        super().__init__("test_object_pose_sanity_rclpy")
        self.snapshot: Dict[str, Tuple[float, float, float]] = {}
        self.raw_entity: Dict[str, str] = {}
        self._sub = None

    def find_pose_topic(self, timeout_sec: float) -> Optional[str]:
        deadline = time.time() + max(0.5, timeout_sec)
        while time.time() < deadline:
            candidates = [
                name
                for name, _types in self.get_topic_names_and_types()
                if name.startswith("/world/") and name.endswith("/pose/info")
            ]
            if candidates:
                return sorted(candidates)[0]
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def start_subscription(self, topic: str, known: set[str]) -> None:
        def _on_pose(msg: TFMessage) -> None:
            for tf in getattr(msg, "transforms", []):
                raw = str(getattr(tf, "child_frame_id", "") or "")
                key = _resolve_object_name(raw, known)
                if not key:
                    continue
                tr = tf.transform.translation
                sample = (float(tr.x), float(tr.y), float(tr.z))
                prev = self.snapshot.get(key)
                # Keep the lowest-z sample to avoid grabbing helper links above object body.
                if prev is None or sample[2] < prev[2]:
                    self.snapshot[key] = sample
                    self.raw_entity[key] = raw

        self._sub = self.create_subscription(
            TFMessage,
            topic,
            _on_pose,
            qos_profile_sensor_data,
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--targets",
        nargs="*",
        default=list(DEFAULT_TARGETS),
        help="Objects to verify (default: box_yellow box_red cyl_purple)",
    )
    args = parser.parse_args()

    targets = [str(t).strip() for t in args.targets if str(t).strip()]
    known = set(targets) | {"pick_demo"}

    rclpy.init()
    node = PoseInfoProbe()
    try:
        pose_topic = node.find_pose_topic(timeout_sec=min(3.0, float(args.timeout)))
        if not pose_topic:
            print("[FAIL] no /world/*/pose/info topic detected")
            return 1

        node.start_subscription(pose_topic, known)
        deadline = time.time() + max(0.5, float(args.timeout))
        while time.time() < deadline and len(node.snapshot) < 2:
            rclpy.spin_once(node, timeout_sec=0.2)

        if not node.snapshot:
            print(f"[FAIL] no pose samples received from {pose_topic}")
            return 1

        print(f"[INFO] pose_topic={pose_topic}")
        for name in sorted(node.snapshot.keys()):
            x, y, z = node.snapshot[name]
            source = node.raw_entity.get(name, name)
            print(f"[POSE] {name} xyz=({x:.3f},{y:.3f},{z:.3f}) source={source}")

        pick_z = node.snapshot.get("pick_demo", (math.nan, math.nan, math.nan))[2]
        offenders: list[Tuple[str, float]] = []
        checked: list[Tuple[str, float]] = []
        for name in targets:
            pose = node.snapshot.get(name)
            if pose is None:
                continue
            z = float(pose[2])
            checked.append((name, z))
            if z >= 1.2:
                offenders.append((name, z))

        spike_2775: list[Tuple[str, float, float, float]] = []
        if math.isfinite(pick_z) and 0.70 <= pick_z <= 0.90:
            for name, (x, y, z) in node.snapshot.items():
                if (name.startswith("box_") or name.startswith("cyl_")) and z >= 2.5:
                    spike_2775.append((name, x, y, z))

        if len(checked) < 1:
            print(f"[FAIL] none of requested targets found: {targets}")
            return 1
        if spike_2775:
            print("[FAIL] suspicious pattern detected: box_/cyl_ near z~2.775 with pick_demo on table")
            for name, x, y, z in sorted(spike_2775):
                print(f"[OFFENDER] {name} xyz=({x:.3f},{y:.3f},{z:.3f})")
            return 1
        if offenders:
            print("[FAIL] target objects out of table range (z >= 1.2)")
            for name, z in offenders:
                print(f"[OFFENDER] {name} z={z:.3f}")
            return 1

        print("[PASS] object pose sanity OK")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

