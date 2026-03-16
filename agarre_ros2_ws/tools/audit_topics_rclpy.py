#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/audit_topics_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Audit pubs/subs + QoS for PICK object topics without ros2 CLI."""

from __future__ import annotations

import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy


def _qos_txt(q) -> str:
    rel = "RELIABLE" if q.reliability == ReliabilityPolicy.RELIABLE else "BEST_EFFORT"
    dur = "VOLATILE" if q.durability == DurabilityPolicy.VOLATILE else "TRANSIENT_LOCAL"
    hist = "KEEP_LAST" if q.history == HistoryPolicy.KEEP_LAST else "KEEP_ALL"
    return f"{rel}/{dur}/{hist}@depth={q.depth}"


def _dump(node: Node, topic: str) -> tuple[int, int]:
    pubs = node.get_publishers_info_by_topic(topic)
    subs = node.get_subscriptions_info_by_topic(topic)
    print(f"[AUDIT][GRAPH] topic={topic} pubs={len(pubs)} subs={len(subs)}")
    for idx, ep in enumerate(pubs, start=1):
        print(
            f"[AUDIT][QOS] topic={topic} role=pub[{idx}] "
            f"node={ep.node_namespace}/{ep.node_name} type={ep.topic_type} qos={_qos_txt(ep.qos_profile)}"
        )
    for idx, ep in enumerate(subs, start=1):
        print(
            f"[AUDIT][QOS] topic={topic} role=sub[{idx}] "
            f"node={ep.node_namespace}/{ep.node_name} type={ep.topic_type} qos={_qos_txt(ep.qos_profile)}"
        )
    return len(pubs), len(subs)


def main() -> int:
    rclpy.init()
    node = Node("audit_topics_rclpy")
    rc = 1
    try:
        deadline = time.monotonic() + 5.0
        pose_pubs = pose_subs = result_pubs = result_subs = 0
        while time.monotonic() < deadline:
            pose_pubs = len(node.get_publishers_info_by_topic("/desired_grasp"))
            pose_subs = len(node.get_subscriptions_info_by_topic("/desired_grasp"))
            result_pubs = len(node.get_publishers_info_by_topic("/desired_grasp/result"))
            result_subs = len(node.get_subscriptions_info_by_topic("/desired_grasp/result"))
            if pose_subs > 0 or result_pubs > 0:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        pose_pubs, pose_subs = _dump(node, "/desired_grasp")
        result_pubs, result_subs = _dump(node, "/desired_grasp/result")
        ok = True
        if pose_subs <= 0:
            print("[FAIL] /desired_grasp sin subscriptores (bridge no conectado)")
            ok = False
        if result_pubs != 1:
            print(f"[FAIL] /desired_grasp/result publishers esperados=1 got={result_pubs}")
            ok = False
        if ok:
            print(
                "[PASS] audit topics OK "
                f"(pose_pubs={pose_pubs}, pose_subs={pose_subs}, result_pubs={result_pubs}, result_subs={result_subs})"
            )
            rc = 0
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:
        pass
    os._exit(rc)


if __name__ == "__main__":
    raise SystemExit(main())
