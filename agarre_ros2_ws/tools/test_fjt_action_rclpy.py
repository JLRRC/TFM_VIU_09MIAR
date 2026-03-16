#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/test_fjt_action_rclpy.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""Check FollowJointTrajectory action server availability without ros2 CLI."""

from __future__ import annotations

import argparse
import os
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node


def _normalize(name: str) -> str:
    name = (name or "").strip()
    if not name:
        return ""
    if not name.startswith("/"):
        name = f"/{name}"
    return name.replace("//", "/")


def _candidates(action_name: str) -> list[str]:
    expected = _normalize(action_name)
    out: list[str] = []
    seen: set[str] = set()
    for item in (
        expected,
        "/joint_trajectory_controller/follow_joint_trajectory",
        "/controller_manager/joint_trajectory_controller/follow_joint_trajectory",
    ):
        n = _normalize(item)
        if n and n not in seen:
            seen.add(n)
            out.append(n)
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--action",
        default="/joint_trajectory_controller/follow_joint_trajectory",
        help="Expected FollowJointTrajectory action name",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Max seconds to wait per candidate",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("test_fjt_action_probe")
    rc = 1
    try:
        available_actions = []
        try:
            available_actions = [str(name) for name, _types in node.get_action_names_and_types()]
        except Exception:
            available_actions = []

        candidates = _candidates(args.action)
        deadline = time.monotonic() + max(0.2, args.timeout)
        found = ""
        attempts = 0
        while time.monotonic() < deadline and not found:
            for action_name in candidates:
                attempts += 1
                client = ActionClient(node, FollowJointTrajectory, action_name)
                try:
                    if client.wait_for_server(timeout_sec=0.2):
                        found = action_name
                        break
                except Exception:
                    pass
            if not found:
                time.sleep(0.05)

        if found:
            print(f"[PASS] FollowJointTrajectory server encontrado: {found}")
            print(f"[INFO] checked={','.join(candidates)} attempts={attempts}")
            rc = 0
        else:
            avail = ",".join(sorted(available_actions)) if available_actions else "none"
            print("[FAIL] FollowJointTrajectory server no disponible")
            print(f"[INFO] expected={_normalize(args.action)} checked={','.join(candidates)}")
            print(f"[INFO] available_actions={avail}")
            rc = 1
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
    # Avoid rare rclpy shutdown hangs under FastDDS churn.
    try:
        sys.stdout.flush()
        sys.stderr.flush()
    except Exception:
        pass
    os._exit(rc)


if __name__ == "__main__":
    raise SystemExit(main())
