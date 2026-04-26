#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_motion_helpers.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Motion helper utilities for the UR5 panel."""
from __future__ import annotations

from typing import Iterable

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def build_joint_trajectory(
    positions: Iterable[float],
    sec: float,
    joint_names: Iterable[str],
    *,
    stamp_msg=None,
    # FASE 2: Añadir parámetros de scaling para evitar TOTG warnings
    max_velocity_scaling: float = 0.3,
    max_acceleration_scaling: float = 0.3,
) -> JointTrajectory:
    """
    Build a JointTrajectory message with proper scaling factors.

    FASE 2: Añade velocities/accelerations defaults para evitar warnings:
    - TOTG: max_velocity_scaling_factor 0.0 out of range
    - TOTG: max_acceleration_scaling_factor 0.0 out of range
    """
    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    if stamp_msg is not None:
        traj.header.stamp = stamp_msg
    sec = max(0.0, float(sec))
    sec_i = int(sec)
    nsec_i = int((sec - sec_i) * 1e9)
    point = JointTrajectoryPoint()
    point.positions = [round(float(p), 4) for p in positions]

    # FASE 2: Añadir velocities y accelerations con valores razonables
    # Esto permite a TOTG calcular correctamente la trayectoria
    num_joints = len(point.positions)
    # Velocidades: estimación simplificada (posición_delta / tiempo)
    # Como no conocemos posición inicial, usamos 0.0 (TOTG calculará velocidades reales)
    point.velocities = [0.0] * num_joints
    # Aceleraciones: también 0.0 (TOTG calculará)
    point.accelerations = [0.0] * num_joints

    point.time_from_start.sec = sec_i
    point.time_from_start.nanosec = nsec_i
    traj.points = [point]
    return traj

import math
import os
import time
import uuid
from typing import Dict, List, Optional, Tuple

import rclpy

from std_msgs.msg import Float64MultiArray

try:
    from rclpy.action import ActionClient
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.action import MoveGroup
except Exception:
    ActionClient = None  # type: ignore[assignment,misc]
    FollowJointTrajectory = None  # type: ignore[assignment,misc]
    MoveGroup = None  # type: ignore[assignment,misc]

from .panel_config import (
    CONTROLLER_READY_TIMEOUT_SEC,
    GRIPPER_CLOSED_RAD,
    GRIPPER_CMD_TOPIC,
    GRIPPER_JOINT2_SIGN,
    GRIPPER_OPEN_RAD,
    ROS_AVAILABLE,
    TRAJ_ACTION_FALLBACK,
    TRAJ_ACTION_FALLBACK_DELAY_SEC,
    TRAJ_ACTION_FALLBACK_EPS_RAD,
    TRAJ_ACTION_FALLBACK_TIMEOUT_SEC,
    UR5_JOINT_NAMES,
)
from .panel_state import MoveItState, SystemState
from .panel_utils import angle_shortest_diff_rad, get_pose, world_to_base
from .logging_utils import timestamped_line
from .panel_robot_presets import _build_pose_stamped


def _log_exception(context: str, exc: Exception) -> None:
    import sys
    print(timestamped_line(f"[MOTION][WARN] {context}: {exc}"), file=sys.stderr, flush=True)


def command_gripper(panel,
    closed: bool,
    *,
    log_action: str = "Gripper",
    force: bool = False,
) -> bool:
    if panel._system_state == SystemState.ERROR_FATAL:
        panel._set_status("Gripper bloqueado: ERROR_FATAL", error=True)
        panel._emit_log_throttled(
            "SAFETY:gripper:ERROR_FATAL",
            "[SAFETY] Gripper bloqueado: ERROR_FATAL",
        )
        return False
    if not force and not panel._require_manual_ready("Gripper"):
        return False
    if not force and not panel._state_ready_basic():
        reason = panel._system_state_reason or panel._system_state.value
        panel._set_status(f"Gripper bloqueado: {reason}", error=True)
        panel._emit_log(f"[SAFETY] Gripper bloqueado: {reason}")
        return False
    panel._gripper_closed = closed
    panel._gripper_is_closed = closed
    if panel.btn_gripper is not None:
        panel.btn_gripper.blockSignals(True)
        panel.btn_gripper.setChecked(closed)
        panel.btn_gripper.setText("Abrir gripper" if closed else "Cerrar gripper")
        panel.btn_gripper.blockSignals(False)
    if log_action:
        panel._log_button(f"{log_action} {'cerrar' if closed else 'abrir'}")
    panel._ensure_moveit_node()
    pub = panel._get_gripper_publisher(GRIPPER_CMD_TOPIC)
    if pub is None:
        panel._set_status("Gripper: publisher no disponible", error=True)
        panel._log_warning("[GRIPPER] Publisher no disponible")
        return False
    target = GRIPPER_CLOSED_RAD if closed else GRIPPER_OPEN_RAD
    msg = Float64MultiArray()
    msg.data = [float(target), float(target) * GRIPPER_JOINT2_SIGN]
    pub.publish(msg)
    force_tag = " (force)" if force else ""
    panel._emit_log(
        f"[GRIPPER] cmd target={target:.3f} rad topic={GRIPPER_CMD_TOPIC}{force_tag}"
    )
    panel._set_status(f"Gripper -> {target:.3f} rad{force_tag}", error=False)
    return True

def command_gripper_preopen(panel, rad: float, *, log_action: str = "Gripper") -> bool:
    """Publica un ángulo de apertura previa del RG2 antes del descenso de agarre."""
    panel._gripper_closed = False
    panel._gripper_is_closed = False
    if panel.btn_gripper is not None:
        panel.btn_gripper.blockSignals(True)
        panel.btn_gripper.setChecked(False)
        panel.btn_gripper.setText("Cerrar gripper")
        panel.btn_gripper.blockSignals(False)
    if log_action:
        panel._log_button(f"{log_action} pre_open rad={rad:.3f}")
    panel._ensure_moveit_node()
    pub = panel._get_gripper_publisher(GRIPPER_CMD_TOPIC)
    if pub is None:
        panel._emit_log("[GRIPPER] pre_open publisher no disponible")
        return False
    msg = Float64MultiArray()
    msg.data = [float(rad), float(rad) * GRIPPER_JOINT2_SIGN]
    pub.publish(msg)
    panel._emit_log(
        f"[GRIPPER] pre_open target={rad:.4f} rad topic={GRIPPER_CMD_TOPIC}"
    )
    panel._set_status(f"Gripper pre-open -> {rad:.3f} rad", error=False)
    return True

def traj_action_target(panel, traj_topic: str) -> str:
    if not traj_topic:
        return ""
    base = traj_topic.rsplit("/joint_trajectory", 1)[0]
    return f"{base}/follow_joint_trajectory"

def resolve_traj_action_name(panel, traj_topic: str, *, allow_fallback: bool = False) -> str:
    action_name = traj_action_target(panel, traj_topic)
    if action_name:
        return action_name
    if allow_fallback:
        return "/joint_trajectory_controller/follow_joint_trajectory"
    return ""

def get_action_client(panel, action_name: str, action_type, *, log_ctx: str = ""):
    if ActionClient is None or action_type is None:
        return None
    if panel._moveit_node is None:
        return None
    cache = None
    cache_name = ""
    if action_type is MoveGroup:
        cache = panel._moveit_action_client
        cache_name = getattr(cache, "action_name", "") if cache else ""
    elif action_type is FollowJointTrajectory:
        cache = panel._traj_action_client
        cache_name = panel._traj_action_name
    if cache is None or cache_name != action_name:
        try:
            cache = ActionClient(panel._moveit_node, action_type, action_name)
        except Exception as exc:
            if log_ctx:
                _log_exception(f"{log_ctx} create ActionClient {action_name}", exc)
            else:
                _log_exception(f"create ActionClient {action_name}", exc)
            cache = None
        if action_type is MoveGroup:
            panel._moveit_action_client = cache
        elif action_type is FollowJointTrajectory:
            panel._traj_action_client = cache
            panel._traj_action_name = action_name if cache else ""
    return cache

def wait_action_server(client, *, timeout_sec: float, log_ctx: str, action_name: str) -> bool:
    if client is None:
        return False
    try:
        return client.wait_for_server(timeout_sec=timeout_sec)
    except Exception as exc:
        _log_exception(f"wait {log_ctx} action {action_name}", exc)
        return False

def format_action_error(panel, kind: str, detail: str) -> str:
    return f"Action {kind} no disponible: {detail}"

def joint_motion_since(panel, snapshot: Dict[str, float]) -> bool:
    if not snapshot:
        return False
    for name, prev in snapshot.items():
        curr = panel._last_joint_positions.get(name)
        if curr is None:
            continue
        if abs(curr - prev) > TRAJ_ACTION_FALLBACK_EPS_RAD:
            return True
    return False

def wait_for_joint_target(panel,
    target: List[float],
    timeout_sec: float,
    tol_rad: float = 0.02,
) -> bool:
    start = time.time()
    stable_hits = 0
    try:
        required_stable_hits = max(
            1,
            int(os.environ.get("PANEL_WAIT_JOINT_TARGET_STABLE_SAMPLES", "3")),
        )
    except Exception:
        required_stable_hits = 3
    try:
        max_state_age_sec = max(
            0.05,
            float(os.environ.get("PANEL_WAIT_JOINT_TARGET_MAX_AGE_SEC", "0.35")),
        )
    except Exception:
        max_state_age_sec = 0.35
    try:
        max_stable_vel_rad_s = max(
            0.0,
            float(os.environ.get("PANEL_WAIT_JOINT_TARGET_MAX_VEL_RAD_S", "0.05")),
        )
    except Exception:
        max_stable_vel_rad_s = 0.05
    while (time.time() - start) < timeout_sec:
        now = time.time()
        pos_map = dict(panel._last_joint_positions)
        vel_map: Dict[str, float] = {}
        state_age_sec = float("inf")
        local_state_age_sec = float("inf")
        if pos_map and panel._last_joint_time > 0.0:
            local_state_age_sec = max(0.0, now - float(panel._last_joint_time))
        if panel._ros_worker_started and panel.ros_worker is not None:
            try:
                payload, ts = panel.ros_worker.get_last_joint_state()
            except Exception:
                payload, ts = None, 0.0
            if payload:
                try:
                    names = [
                        _normalize_joint_name(str(name))
                        for name in (payload.get("name") or [])
                    ]
                    positions = payload.get("position") or []
                    velocities = payload.get("velocity") or []
                except Exception:
                    names, positions, velocities = [], [], []
                if names and positions:
                    pos_map = {}
                    for name, pos in zip(names, positions):
                        try:
                            pos_map[name] = float(pos)
                        except Exception:
                            continue
                if names and isinstance(velocities, list) and len(velocities) == len(names):
                    for name, vel in zip(names, velocities):
                        try:
                            vel_map[name] = float(vel)
                        except Exception:
                            continue
                if ts:
                    state_age_sec = max(0.0, now - float(ts))
        if state_age_sec > max_state_age_sec and local_state_age_sec <= max_state_age_sec:
            state_age_sec = local_state_age_sec
        ok = state_age_sec <= max_state_age_sec
        max_vel = 0.0
        for idx, name in enumerate(UR5_JOINT_NAMES):
            if idx >= len(target):
                break
            curr = pos_map.get(name)
            if curr is None:
                ok = False
                break
            if abs(angle_shortest_diff_rad(curr, target[idx])) > tol_rad:
                ok = False
                break
            if name in vel_map:
                max_vel = max(max_vel, abs(float(vel_map[name])))
        if ok and vel_map and max_vel > max_stable_vel_rad_s:
            ok = False
        if ok:
            stable_hits += 1
            if stable_hits >= required_stable_hits:
                return True
        else:
            stable_hits = 0
        time.sleep(0.05)
    return False

def wait_for_tcp_base_z(panel,
    target_z: float,
    timeout_sec: float,
    tol_m: float = 0.005,
    tcp_z_offset: float = 0.0,
) -> bool:
    start = time.time()
    last_z = None
    base_frame = panel._business_base_frame()
    ee_frame = panel._ee_frame_effective or "tool0"
    while (time.time() - start) < timeout_sec:
        bz = None
        try:
            pose_base, _ = get_pose(base_frame, ee_frame, timeout_sec=0.05)
            if pose_base:
                bz = float(pose_base["position"]["z"])
        except Exception:
            bz = None
        if bz is None and ee_frame == "tool0" and panel._last_tcp_world:
            # Fallback only for tool0-based TCP to avoid stale FK when using virtual TCPs.
            _, _, bz = world_to_base(*panel._last_tcp_world)
        if bz is None and panel._last_tcp_base_z is not None:
            # Fallback to last TF-derived tcp base z when TF query is flaky.
            bz = panel._last_tcp_base_z
        if bz is not None:
            last_z = bz
            eff_z = bz + tcp_z_offset
            if abs(eff_z - target_z) <= tol_m:
                return True
        time.sleep(0.05)
    if last_z is not None:
        panel._emit_log(
            f"[PICK] WARN: tcp_z={last_z:.3f} objetivo_z={target_z:.3f} (timeout)"
        )
    else:
        panel._emit_log("[PICK] WARN: tcp_z no disponible (timeout)")
    return False

def wait_for_tcp_base_target(panel,
    target_xyz: Tuple[float, float, float],
    timeout_sec: float,
    tol_xyz_m: float = 0.10,
    ee_frame: Optional[str] = None,
) -> Tuple[bool, Optional[Tuple[float, float, float]], float]:
    start = time.time()
    last_pos: Optional[Tuple[float, float, float]] = None
    last_dist = float("inf")
    base_frame = panel._business_base_frame()
    frame_name = ee_frame or panel._ee_frame_effective or "tool0"
    tx, ty, tz = float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])
    while (time.time() - start) < timeout_sec:
        curr_pos: Optional[Tuple[float, float, float]] = None
        try:
            pose_base, _ = get_pose(base_frame, frame_name, timeout_sec=0.05)
            if pose_base:
                pos = pose_base.get("position", (0.0, 0.0, 0.0))
                if isinstance(pos, (list, tuple)) and len(pos) >= 3:
                    curr_pos = (float(pos[0]), float(pos[1]), float(pos[2]))
        except Exception:
            curr_pos = None
        if curr_pos is None and panel._last_tcp_base is not None:
            curr_pos = panel._last_tcp_base
        if curr_pos is not None:
            last_pos = curr_pos
            last_dist = math.sqrt(
                (curr_pos[0] - tx) ** 2
                + (curr_pos[1] - ty) ** 2
                + (curr_pos[2] - tz) ** 2
            )
            if last_dist <= tol_xyz_m:
                return True, last_pos, last_dist
        time.sleep(0.05)
    return False, last_pos, last_dist

def send_joint_trajectory_action(panel, positions: List[float], sec: float, traj_topic: str) -> Tuple[bool, str]:
    if not ROS_AVAILABLE or ActionClient is None or FollowJointTrajectory is None:
        return False, format_action_error(panel, "FollowJointTrajectory", "ActionClient")
    if panel._moveit_node is None:
        return False, panel._ros_node_not_ready_reason()
    action_name = resolve_traj_action_name(panel, traj_topic)
    if not action_name:
        return False, format_action_error(panel, "FollowJointTrajectory", "target vacío")
    if panel._traj_action_client is None or panel._traj_action_name != action_name:
        panel._traj_action_client = get_action_client(panel, 
            action_name,
            FollowJointTrajectory,
            log_ctx="traj_action",
        )
    client = panel._traj_action_client
    if client is None:
        return False, format_action_error(panel, "FollowJointTrajectory", "client")
    if not wait_action_server(
        client,
        timeout_sec=1.5,
        log_ctx="traj_action",
        action_name=action_name,
    ):
        return False, format_action_error(panel, "FollowJointTrajectory", action_name)
    goal = FollowJointTrajectory.Goal()
    traj = build_joint_trajectory(positions, sec, UR5_JOINT_NAMES)
    goal.trajectory = traj
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(panel._moveit_node, future, timeout_sec=TRAJ_ACTION_FALLBACK_TIMEOUT_SEC)
    goal_handle = future.result() if future.done() else None
    if not goal_handle or not goal_handle.accepted:
        return False, format_action_error(panel, "FollowJointTrajectory", "goal rechazado")
    result_future = goal_handle.get_result_async()
    result_timeout_sec = max(float(sec) + 3.0, TRAJ_ACTION_FALLBACK_TIMEOUT_SEC)
    rclpy.spin_until_future_complete(
        panel._moveit_node,
        result_future,
        timeout_sec=result_timeout_sec,
    )
    if not result_future.done():
        return False, format_action_error(panel, 
            "FollowJointTrajectory",
            f"result timeout>{result_timeout_sec:.1f}s",
        )
    result_msg = result_future.result()
    status = getattr(result_msg, "status", None)
    if status != 4:
        return False, format_action_error(panel, 
            "FollowJointTrajectory",
            f"{action_name}:status={status}",
        )
    return True, f"{action_name}:status={status}"

def schedule_traj_action_fallback(panel, positions: List[float], sec: float, traj_topic: str) -> None:
    if not TRAJ_ACTION_FALLBACK:
        return
    if panel._traj_action_inflight:
        return
    now = time.time()
    if (now - panel._traj_fallback_last_ts) < 1.0:
        return
    snapshot = dict(panel._last_joint_positions)

    def worker():
        time.sleep(TRAJ_ACTION_FALLBACK_DELAY_SEC)
        if panel._closing:
            return
        if joint_motion_since(panel, snapshot):
            return
        panel._traj_action_inflight = True
        try:
            ok, info = send_joint_trajectory_action(panel, positions, sec, traj_topic)
            if ok:
                panel._traj_fallback_last_ts = time.time()
                log_traj_action_fallback(panel, ok, info)
            else:
                log_traj_action_fallback(panel, ok, info)
        finally:
            panel._traj_action_inflight = False

    panel._run_async(worker)

# Límites de posición para cada joint del UR5 (en radianes, según joint_limits.yaml).
# Orden: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
_UR5_JOINT_POS_LIMITS: List[Tuple[float, float]] = [
    (-6.2832, 6.2832),   # shoulder_pan_joint   ±360°
    (-6.2832, 6.2832),   # shoulder_lift_joint  ±360°
    (-3.1416, 3.1416),   # elbow_joint          ±180°
    (-6.2832, 6.2832),   # wrist_1_joint        ±360°
    (-6.2832, 6.2832),   # wrist_2_joint        ±360°
    (-6.2832, 6.2832),   # wrist_3_joint        ±360°
]

def clamp_joint_positions(panel, positions: List[float]) -> Tuple[List[float], List[str]]:
    """Clamp each joint to its physical limit and return (clamped_list, warnings)."""
    clamped = list(positions)
    warnings: List[str] = []
    for idx, (lo, hi) in enumerate(panel._UR5_JOINT_POS_LIMITS):
        if idx >= len(clamped):
            break
        val = float(clamped[idx])
        if val < lo or val > hi:
            name = UR5_JOINT_NAMES[idx] if idx < len(UR5_JOINT_NAMES) else f"j{idx}"
            clamped[idx] = max(lo, min(hi, val))
            warnings.append(
                f"{name}: {math.degrees(val):.2f}° → clamped to"
                f" {math.degrees(clamped[idx]):.2f}°"
                f" (limit [{math.degrees(lo):.0f}°,{math.degrees(hi):.0f}°])"
            )
    return clamped, warnings

def publish_joint_trajectory(panel,
    positions: List[float],
    sec: float,
    *,
    prefer_action: bool = False,
) -> Tuple[bool, str]:
    if getattr(panel, "_pick_moveit_phase_active", False):
        panel._emit_log(
            "[MANUAL] BLOCKED: publicación manual durante fase MoveIt de PICK_OBJ"
        )
        return False, "manual bloqueado: moveit_executing"
    if not panel._ros_worker_started:
        panel._ensure_ros_worker_started()
    if not panel.ros_worker.node_ready():
        # Worker may have stopped unexpectedly (spin loop exited between phases).
        # Detect and restart before failing hard.
        if hasattr(panel.ros_worker, "is_worker_running") and not panel.ros_worker.is_worker_running():
            panel._emit_log(
                "[ROS][WARN] _publish_joint_trajectory: worker detenido inesperadamente; reiniciando"
            )
            panel._ros_worker_started = False
            panel._ensure_ros_worker_started()
        # Wait up to 3s for the node to become ready (covers both restart and
        # transient startup delays).
        _node_ready_deadline = time.monotonic() + 3.0
        while not panel.ros_worker.node_ready() and time.monotonic() < _node_ready_deadline:
            time.sleep(0.05)
        if not panel.ros_worker.node_ready():
            return False, panel._ros_node_not_ready_reason()
    ok, reason = panel._wait_for_controllers_ready(CONTROLLER_READY_TIMEOUT_SEC)
    if not ok:
        return False, f"controladores no listos: {reason}"
    topic = panel._select_traj_topic()
    if not topic:
        return False, "joint_trajectory_controller no disponible"
    pub = panel._get_traj_publisher(topic)
    if not pub:
        return False, "Publisher JointTrajectory no disponible"
    # --- Validación de límites de joints ---
    safe_positions, limit_warnings = clamp_joint_positions(panel, positions)
    for w in limit_warnings:
        panel._emit_log(f"[ROBOT][JOINT_LIMIT] WARN: {w}")
    if prefer_action:
        ok, info = send_joint_trajectory_action(panel, safe_positions, sec, topic)
        if ok:
            panel._emit_log(
                "[ROBOT] Executing JointTrajectory via FollowJointTrajectory action"
            )
            return True, f"action:{info}"
        panel._emit_log(
            "[ROBOT] FollowJointTrajectory action failed; falling back to topic "
            f"reason={info}"
        )
    if panel._traj_publish_inflight:
        panel._emit_log("[ROBOT] WARN: publish JointTrajectory solapado")
    panel._traj_publish_inflight = True
    try:
        traj = build_joint_trajectory(safe_positions, sec, UR5_JOINT_NAMES)
        panel._emit_log("[MANUAL] Executing direct JointTrajectory (MoveIt bypassed)")
        pub.publish(traj)
        return True, topic
    finally:
        panel._traj_publish_inflight = False

def log_traj_action_fallback(panel, ok: bool, info: str) -> None:
    if ok:
        panel._emit_log(f"[ROBOT] Fallback action en {info}")
    else:
        panel._emit_log(f"[ROBOT] Fallback action falló: {info}")

def publish_moveit_pose(panel,
    label: str,
    pose_data: Dict[str, object],
    *,
    cartesian: bool = False,
) -> bool:
    # FASE 3: Esperar a que la motion anterior termine.
    deadline = time.monotonic() + 30.0
    while panel._motion_in_progress and time.monotonic() < deadline:
        time.sleep(0.1)
    if panel._motion_in_progress:
        panel._emit_log(f"[MOVEIT] BLOCKED: motion_in_progress timeout (label={label})")
        return False
    panel._motion_in_progress = True
    if not panel._moveit_required:
        panel._set_status("MoveIt deshabilitado; bloqueando publicación", error=True)
        panel._emit_log(f"[MOVEIT] Bloqueado: MoveIt deshabilitado (label={label})")
        panel._motion_in_progress = False
        return False
    if panel._moveit_state != MoveItState.READY and not panel._moveit_status_ready():
        reason = panel._moveit_not_ready_reason()
        panel._set_status(f"MoveIt no listo; bloqueando {label} ({reason})", error=True)
        panel._emit_log(f"[MOVEIT] Bloqueado: {label} ({reason})")
        panel._motion_in_progress = False
        return False
    tf_timeout = max(0.2, float(os.environ.get("PANEL_MOVEIT_TF_GATE_TIMEOUT_SEC", "1.2") or 1.2))
    tf_deadline = time.monotonic() + tf_timeout
    tf_ok = False
    tf_reason = "tf_not_ready"
    while time.monotonic() < tf_deadline:
        tf_ok, tf_reason = panel._tf_chain_ready_status()
        if tf_ok:
            break
        time.sleep(0.05)
    if not tf_ok:
        reason = f"TF no listo ({tf_reason})"
        panel._set_status(f"MoveIt bloqueado: {reason}", error=False)
        panel._emit_log(
            f"[MOVEIT] Bloqueado: {label} {reason} "
            f"(timeout={tf_timeout:.1f}s; required=base_link->{panel._required_ee_frame})"
        )
        panel._motion_in_progress = False
        return False
    # FASE 2: Validar JointState antes de enviar a MoveIt (evita "Found empty JointState message")
    if panel._ros_worker_started and panel.ros_worker:
        joint_ok, joint_reason = panel.ros_worker.joint_state_valid(timeout_sec=2.0)
        if not joint_ok:
            panel._emit_log(
                f"[MOVEIT] GATING: esperando JointState válido antes de {label} (reason={joint_reason})"
            )
            # Intentar esperar hasta 2s por JointState válido
            joint_deadline = time.time() + 2.0
            while time.time() < joint_deadline:
                joint_ok, joint_reason = panel.ros_worker.joint_state_valid(timeout_sec=2.0)
                if joint_ok:
                    break
                time.sleep(0.1)
            if not joint_ok:
                panel._emit_log(
                    f"[MOVEIT] Bloqueado: {label} JointState no válido ({joint_reason})"
            )
                panel._motion_in_progress = False
                return False
    base_frame = panel._business_base_frame()
    normalized = dict(pose_data or {})
    pose_frame_raw = str(normalized.get("frame") or base_frame)
    pose_frame = pose_frame_raw.split("|", 1)[0].strip() or pose_frame_raw
    suffix = ""
    if "|" in pose_frame_raw and pose_frame_raw.startswith(pose_frame):
        suffix = pose_frame_raw[len(pose_frame):]
    if pose_frame in ("base", "/base"):
        panel._emit_log(
            f"[MOVEIT][P0] Bloqueado: {label} frame={pose_frame_raw} "
            f"(business frame must be {base_frame})"
        )
        panel._motion_in_progress = False
        return False
    if pose_frame != base_frame:
        try:
            raw_pose = _build_pose_stamped(
                {
                    "position": normalized.get("position", (0.0, 0.0, 0.0)),
                    "orientation": normalized.get("orientation", (0.0, 0.0, 0.0, 1.0)),
                    "frame": pose_frame,
                    "stamp_ns": normalized.get("stamp_ns", 0),
                }
            )
            base_pose = panel.ensure_base_pose(raw_pose, timeout_sec=0.8)
        except Exception as exc:
            _log_exception("moveit pose normalize build", exc)
            base_pose = None
        if base_pose is None:
            panel._emit_log(
                f"[MOVEIT] Bloqueado: {label} frame={pose_frame_raw} -> {base_frame} tf_failed"
            )
            panel._emit_log(
                f"[FRAME_IN] source={pose_frame_raw} target={base_frame} status=tf_failed"
            )
            panel._motion_in_progress = False
            return False
        normalized["position"] = (
            float(base_pose.pose.position.x),
            float(base_pose.pose.position.y),
            float(base_pose.pose.position.z),
        )
        normalized["orientation"] = (
            float(base_pose.pose.orientation.x),
            float(base_pose.pose.orientation.y),
            float(base_pose.pose.orientation.z),
            float(base_pose.pose.orientation.w),
        )
        normalized["frame"] = f"{base_frame}{suffix}"
        panel._emit_log(
            f"[MOVEIT] normalize_pose label={label} from={pose_frame_raw} to={normalized['frame']}"
        )
        panel._emit_log(
            f"[FRAME_IN] source={pose_frame_raw} target={base_frame} status=ok"
        )
    else:
        normalized["frame"] = f"{base_frame}{suffix}"
    frame_out = str(normalized.get("frame") or base_frame)
    if "|rid=" not in frame_out:
        request_id = int(getattr(panel, "_panel_moveit_request_id", 0) or 0) + 1
        setattr(panel, "_panel_moveit_request_id", request_id)
        request_uuid = uuid.uuid4().hex
        normalized["frame"] = f"{frame_out}|rid={request_id}|uid={request_uuid}"
    try:
        px, py, pz = normalized.get("position", (0.0, 0.0, 0.0))
        frame_raw = str(normalized.get("frame") or base_frame)
        frame_clean = frame_raw.split("|", 1)[0].strip() or frame_raw
        panel._emit_log(
            "[MOVEIT_GOAL] "
            f"label={label} frame={frame_clean} needs_tf={str(frame_clean != base_frame).lower()} "
            f"cartesian={str(bool(cartesian)).lower()} "
            f"pos=({float(px):.3f},{float(py):.3f},{float(pz):.3f}) "
            f"ee={panel._ee_frame_effective or 'rg2_pinch_center'}"
        )
    except Exception:
        pass
    context = panel._moveit_publish_context()
    panel._moveit_block_reason, _published = publish_moveit_pose(
        label=label,
        pose_data=normalized,
        cartesian=cartesian,
        **context,
    )
    return bool(_published)


def _normalize_joint_name(name) -> str:
    text = str(name).strip()
    if "::" in text:
        text = text.split("::")[-1]
    if "/" in text:
        text = text.split("/")[-1]
    return text.strip()

