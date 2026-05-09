#!/usr/bin/env python3
"""F1.7 v6: 3 cycles via /pick_place con pose resolved dinámicamente cada ciclo.

Estrategia:
  - Usar objetos distintos por ciclo (box_red, box_blue, box_green) para
    evitar contaminar el state entre ciclos.
  - Resolver pose REAL via /orchestrator/resolve_object_pose_world antes
    de cada send_goal — el hint hardcoded del v5 era stale para cycles 2-3.
  - Reset (/release_objects) entre ciclos para reposicionar los objects.
  - Sleep 6s tras reset para que physics settle (cycle 3 falló por
    MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE).
"""

from __future__ import annotations

import math
import sys
import time
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur5_panel_interfaces.action import PickPlace
from ur5_panel_interfaces.srv import ResolveObjectPoseWorld


OBJECTS = ["box_red", "box_blue", "box_green"]
DROP_XYZ = (-1.30, 0.0, 0.85)
LOG_PATH = "/tmp/three_cycles_v6_result.log"

# F1.13 audit-v4 (2026-05-08): hard reset a HOME entre ciclos.
# Evita el cascade state contamination donde un cycle fail deja al
# robot en pose extraña y el próximo HOME_INITIAL no recupera.
HOME_POSITIONS = (0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0)
HOME_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
HOME_DURATION_SEC = 8.0
HOME_TOL_RAD = 1.0  # lenient para sim physics
HOME_GOAL_TIME_TOL_SEC = 30.0


def log(msg: str) -> None:
    line = f"[{datetime.now().strftime('%H:%M:%S')}] {msg}"
    print(line, flush=True)
    with open(LOG_PATH, "a", encoding="utf-8") as fp:
        fp.write(line + "\n")


def hard_reset_to_home(node: Node, fjt_client: ActionClient) -> bool:
    """F1.13: envía FJT direct a HOME antes de cada cycle.

    Bypassa el orchestrator HOME_INITIAL — usa el controller directamente.
    Robot va a HOME canónico independientemente del estado previo.
    """
    log("--- hard reset to HOME (F1.13) ---")
    if not fjt_client.wait_for_server(timeout_sec=5.0):
        log("ERROR: /joint_trajectory_controller/follow_joint_trajectory unavailable")
        return False

    jt = JointTrajectory()
    jt.joint_names = list(HOME_JOINT_NAMES)
    point = JointTrajectoryPoint()
    point.positions = list(HOME_POSITIONS)
    point.velocities = [0.0] * len(HOME_JOINT_NAMES)
    sec = int(HOME_DURATION_SEC)
    nsec = int(round((HOME_DURATION_SEC - sec) * 1_000_000_000.0))
    point.time_from_start.sec = sec
    point.time_from_start.nanosec = nsec
    jt.points = [point]

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = jt
    goal.path_tolerance = [
        JointTolerance(name=jn, position=10.0)  # path tolerance ~ disabled
        for jn in HOME_JOINT_NAMES
    ]
    goal.goal_tolerance = [
        JointTolerance(name=jn, position=HOME_TOL_RAD)
        for jn in HOME_JOINT_NAMES
    ]
    sec = int(HOME_GOAL_TIME_TOL_SEC)
    nsec = int(round((HOME_GOAL_TIME_TOL_SEC - sec) * 1_000_000_000.0))
    goal.goal_time_tolerance.sec = sec
    goal.goal_time_tolerance.nanosec = nsec

    send_future = fjt_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
    if send_future.result() is None:
        log("ERROR: home reset send_goal timeout")
        return False
    gh = send_future.result()
    if not gh.accepted:
        log("ERROR: home reset goal rejected")
        return False

    result_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=60.0)
    if result_future.result() is None:
        log("ERROR: home reset result timeout 60s")
        return False
    err = result_future.result().result.error_code
    if err != 0:
        log(f"WARN: home reset error_code={err}, continuando igual")
        return False
    log("home reset ✅")
    return True


def main() -> int:
    rclpy.init()
    node = Node("f1_7_v6_driver")

    log("=== F1.7 LIVE 3 CICLOS v6 (pose resolved + objs distintos) ===")

    release_client = node.create_client(Trigger, "/release_objects")
    resolve_client = node.create_client(
        ResolveObjectPoseWorld, "/orchestrator/resolve_object_pose_world"
    )
    action_client = ActionClient(node, PickPlace, "/pick_place")
    # F1.13: cliente FJT direct para hard reset a HOME entre cycles.
    fjt_client = ActionClient(
        node,
        FollowJointTrajectory,
        "/joint_trajectory_controller/follow_joint_trajectory",
    )

    if not release_client.wait_for_service(timeout_sec=10.0):
        log("ERROR: /release_objects no disponible")
        return 1
    if not resolve_client.wait_for_service(timeout_sec=10.0):
        log("ERROR: /orchestrator/resolve_object_pose_world no disponible")
        return 1
    if not action_client.wait_for_server(timeout_sec=10.0):
        log("ERROR: /pick_place action server no disponible")
        return 1

    # F1.9 audit-v4: warmup release_objects con timeout largo (subprocess gz
    # primera llamada es lenta — 11 objetos × ~3s cada uno via gz service).
    log("--- warmup release_objects (timeout 60s) ---")
    warmup_future = release_client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, warmup_future, timeout_sec=60.0)
    if warmup_future.result() is None:
        log("ERROR: warmup release_objects timeout (60s) — continuando igual")
    else:
        log(f"warmup OK: {warmup_future.result().message}")
    time.sleep(4.0)

    successes = 0
    for i, obj_name in enumerate(OBJECTS, start=1):
        log("")
        log(f"=== CICLO {i}/3 — {obj_name} ===")

        # F1.13: hard reset a HOME ANTES de cada ciclo (incluso el primero).
        # Garantiza estado robot conocido independiente de cycle previo.
        hard_reset_to_home(node, fjt_client)

        # 1) Reset: release suspended objects.
        # F1.9 audit-v4: timeout subido 10 → 60s para acomodar gz subprocess
        # warmup (primera llamada lenta).
        # F1.17 audit-v4 (2026-05-08): subido 60 → 120s. Cycle 2 falló en run
        # 12:54 con timeout=60s — subprocess gz cli (~3.5s × 20 calls delete+
        # spawn de 10 objetos) puede acumular latencia tras cycle 1. 120s
        # da margen sin penalizar runs sanos (cycle 1 tardó solo 11s).
        log(f"--- reset ({obj_name}) ---")
        release_future = release_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, release_future, timeout_sec=120.0)
        if release_future.result() is None:
            log("ERROR: /release_objects timeout (120s)")
            continue
        log(f"reset OK: {release_future.result().message}")

        # 2) Wait for physics to settle.
        time.sleep(6.0)

        # 3) Resolve actual pose of the object.
        resolve_req = ResolveObjectPoseWorld.Request()
        resolve_req.object_name = obj_name
        resolve_future = resolve_client.call_async(resolve_req)
        rclpy.spin_until_future_complete(node, resolve_future, timeout_sec=10.0)
        if resolve_future.result() is None:
            log(f"ERROR: resolve_object_pose_world timeout for {obj_name}")
            continue
        resp = resolve_future.result()
        if not resp.success:
            log(f"ERROR: resolve failed: {resp.detail}")
            continue
        p = resp.pose_world.position
        q = resp.pose_world.orientation
        log(
            f"pose {obj_name}: pos=({p.x:.4f},{p.y:.4f},{p.z:.4f}) "
            f"quat=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})"
        )

        # 4) Send /pick_place goal.
        goal = PickPlace.Goal()
        goal.object_name = obj_name
        goal.drop_xyz_world.x = DROP_XYZ[0]
        goal.drop_xyz_world.y = DROP_XYZ[1]
        goal.drop_xyz_world.z = DROP_XYZ[2]
        goal.object_pose_world_hint.position = p
        goal.object_pose_world_hint.orientation = q

        log(f"--- send_goal {obj_name} ---")
        t0 = time.monotonic()
        send_future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
        if send_future.result() is None:
            log("ERROR: send_goal timeout")
            continue
        gh = send_future.result()
        if not gh.accepted:
            log("ERROR: goal rejected")
            continue
        log("goal accepted")

        result_future = gh.get_result_async()
        # F1.13: driver timeout > orchestrator outer (700s) para dar margen.
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=900.0)
        if result_future.result() is None:
            log(f"ERROR: result timeout 900s for {obj_name}")
            continue
        result = result_future.result().result
        status = result_future.result().status  # 4=SUCCEEDED, 6=ABORTED
        dt = time.monotonic() - t0
        ok = bool(getattr(result, "success", False))
        reason = getattr(result, "reason", "")
        if ok and status == 4:
            successes += 1
            log(f"CICLO {i}/3 ✅ SUCCESS ({dt:.1f}s) reason={reason}")
        else:
            log(f"CICLO {i}/3 ❌ ABORTED status={status} reason={reason} ({dt:.1f}s)")

        time.sleep(5.0)

    log("")
    log(f"=== F1.7 v6 COMPLETO — {successes}/3 SUCCESS ===")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if successes == 3 else 2


if __name__ == "__main__":
    sys.exit(main())
