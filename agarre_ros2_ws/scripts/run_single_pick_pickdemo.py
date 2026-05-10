#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/scripts/run_single_pick_pickdemo.py
"""Driver minimal: 1 ciclo /pick_place con object_name=pick_demo.

Captura best_obj_move/best_lift_delta/best_tcp_dist y métricas físicas
en el feedback intermedio del action.

Uso:
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 scripts/run_single_pick_pickdemo.py [--object pick_demo] [--out /path/to/log]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from typing import Any, Dict, List

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from ur5_panel_interfaces.action import PickPlace
from ur5_panel_interfaces.srv import ResolveObjectPoseWorld


class PickRunner(Node):
    def __init__(self):
        super().__init__("pick_demo_runner")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.client = ActionClient(self, PickPlace, "/pick_place")
        self.resolve_cli = self.create_client(ResolveObjectPoseWorld, "/orchestrator/resolve_object_pose_world")
        self.feedback_log: List[Dict[str, Any]] = []
        self.last_phase = ""

    def resolve_pose(self, name: str):
        if not self.resolve_cli.wait_for_service(timeout_sec=8.0):
            return None
        req = ResolveObjectPoseWorld.Request()
        req.object_name = name
        fut = self.resolve_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.result()

    def _on_feedback(self, msg):
        fb = msg.feedback
        # PickPlace.Feedback actual: current_phase, progress, phase_index, detail.
        phase = getattr(fb, "current_phase", getattr(fb, "phase", ""))
        progress = getattr(fb, "progress", getattr(fb, "sub_progress", 0.0))
        detail = getattr(fb, "detail", getattr(fb, "message", ""))
        rec = {
            "ts": time.monotonic(),
            "phase": phase,
            "progress": progress,
            "phase_index": getattr(fb, "phase_index", -1),
            "detail": detail,
        }
        # capturar métricas si existen como atributos
        for attr in ("best_obj_move", "best_lift_delta", "best_tcp_dist",
                     "obj_move", "lift_delta", "tcp_dist", "gripper_closed",
                     "logical_state", "physical_attached"):
            if hasattr(fb, attr):
                rec[attr] = getattr(fb, attr)
        self.feedback_log.append(rec)
        cur = rec["phase"]
        if cur and cur != self.last_phase:
            print(
                f"[FB] phase={cur} progress={rec['progress']:.2f} "
                f"idx={rec['phase_index']} detail={rec['detail'][:80]}",
                flush=True,
            )
            self.last_phase = cur


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--object", default="pick_demo")
    parser.add_argument("--out", default="/tmp/single_pick_result.json")
    parser.add_argument("--timeout", type=float, default=300.0,
                        help="seconds total para el cycle")
    parser.add_argument("--drop-x", type=float, default=-1.30,
                        help="drop target X en world")
    parser.add_argument("--drop-y", type=float, default=0.0,
                        help="drop target Y en world")
    parser.add_argument("--drop-z", type=float, default=0.85,
                        help="drop target Z en world")
    args = parser.parse_args()

    rclpy.init()
    runner = PickRunner()
    print(f"[INIT] runner up, object={args.object}", flush=True)

    # 1) Resolver pose actual
    pose_resp = runner.resolve_pose(args.object)
    if pose_resp is None or not pose_resp.success:
        print(f"[ERROR] resolve_object_pose_world failed: {pose_resp}", flush=True)
        rclpy.shutdown()
        sys.exit(2)
    pos = pose_resp.pose_world.position
    print(f"[POSE] {args.object} world=({pos.x:.4f}, {pos.y:.4f}, {pos.z:.4f}) detail={pose_resp.detail}", flush=True)

    # 2) Esperar action server
    if not runner.client.wait_for_server(timeout_sec=10.0):
        print("[ERROR] /pick_place server unavailable", flush=True)
        rclpy.shutdown()
        sys.exit(3)

    # 3) Construir goal
    goal = PickPlace.Goal()
    goal.object_name = args.object
    goal.drop_xyz_world.x = float(args.drop_x)
    goal.drop_xyz_world.y = float(args.drop_y)
    goal.drop_xyz_world.z = float(args.drop_z)
    goal.object_pose_world_hint.position = pose_resp.pose_world.position
    goal.object_pose_world_hint.orientation = pose_resp.pose_world.orientation

    print(
        f"[SEND_GOAL] goal.object_name={goal.object_name} "
        f"drop_world=({goal.drop_xyz_world.x:.3f},"
        f"{goal.drop_xyz_world.y:.3f},{goal.drop_xyz_world.z:.3f})",
        flush=True,
    )
    send_fut = runner.client.send_goal_async(goal, feedback_callback=runner._on_feedback)
    rclpy.spin_until_future_complete(runner, send_fut, timeout_sec=10.0)
    if send_fut.result() is None:
        print("[ERROR] send_goal timeout", flush=True)
        rclpy.shutdown()
        sys.exit(4)
    gh = send_fut.result()
    if not gh.accepted:
        print("[ERROR] goal rejected", flush=True)
        rclpy.shutdown()
        sys.exit(5)
    print("[ACCEPTED] esperando resultado...", flush=True)

    # 4) Esperar resultado
    t_start = time.monotonic()
    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(runner, res_fut, timeout_sec=args.timeout)
    elapsed = time.monotonic() - t_start
    if res_fut.result() is None:
        print(f"[TIMEOUT] {elapsed:.1f}s sin resultado", flush=True)
        verdict = "TIMEOUT"
        result_obj = None
    else:
        result_obj = res_fut.result().result
        status = res_fut.result().status
        success = getattr(result_obj, "success", False)
        reason = getattr(result_obj, "reason", getattr(result_obj, "message", ""))
        print(f"[RESULT] status={status} success={success} reason={reason} elapsed={elapsed:.1f}s", flush=True)
        verdict = "SUCCESS" if success else "FAIL"

    # 5) Métricas finales
    out = {
        "object": args.object,
        "verdict": verdict,
        "elapsed_sec": elapsed,
        "feedback_count": len(runner.feedback_log),
        "phases_seen": sorted(set(r["phase"] for r in runner.feedback_log if r.get("phase"))),
        "feedback_log": runner.feedback_log[-50:],  # últimos 50
    }
    if result_obj is not None:
        for attr in ("success", "reason", "duration_sec", "cycles_completed", "message", "best_obj_move", "best_lift_delta",
                     "best_tcp_dist", "obj_move", "lift_delta", "tcp_dist",
                     "logical_state", "physical_attached"):
            if hasattr(result_obj, attr):
                v = getattr(result_obj, attr)
                # convertir a tipos serializables
                try:
                    json.dumps(v)
                    out[f"result_{attr}"] = v
                except Exception:
                    out[f"result_{attr}"] = str(v)

    text = json.dumps(out, indent=2, default=str)
    print("==== RESULT JSON ====", flush=True)
    print(text)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text)

    rclpy.shutdown()
    return 0 if verdict == "SUCCESS" else 1


if __name__ == "__main__":
    sys.exit(main())
