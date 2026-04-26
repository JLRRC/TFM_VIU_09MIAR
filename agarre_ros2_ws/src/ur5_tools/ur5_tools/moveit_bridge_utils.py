#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/moveit_bridge_utils.py
# Contenido: Helpers puros extraídos de UR5MoveItBridge — sin estado de nodo.
# Uso breve: Importar desde ur5_moveit_bridge.py o directamente en tests.
"""Pure utility helpers for the UR5 MoveIt bridge — no ROS node state."""
from __future__ import annotations

import math
import os
import time
from copy import deepcopy
from typing import Any, Optional

import numpy as np

try:
    from action_msgs.msg import GoalStatus
except Exception:  # pragma: no cover — ROS not available in unit tests
    GoalStatus = None  # type: ignore

try:
    from trajectory_msgs.msg import JointTrajectory
except Exception:  # pragma: no cover
    JointTrajectory = None  # type: ignore


# ---------------------------------------------------------------------------
# Environment helpers
# ---------------------------------------------------------------------------

def bridge_env_float(name: str, default: float) -> float:
    """Read a float from an environment variable, returning *default* on failure."""
    raw = os.environ.get(name, "")
    if not raw:
        return float(default)
    try:
        return float(raw)
    except Exception:
        return float(default)


# ---------------------------------------------------------------------------
# String / name utilities
# ---------------------------------------------------------------------------

def normalize_action_name(name: str) -> str:
    """Normalise a ROS action name: ensure leading slash, collapse double slashes."""
    raw = str(name or "").strip()
    if not raw:
        return ""
    if not raw.startswith("/"):
        raw = f"/{raw}"
    return raw.replace("//", "/")


def parse_request_meta(frame_raw: str) -> tuple[str, Optional[int], str, dict[str, Any]]:
    """Parse the frame_id field that encodes request metadata (frame|rid=N|uid=…)."""
    raw = str(frame_raw or "").strip()
    if not raw:
        return "", None, "", {}
    parts = [p.strip() for p in raw.split("|")]
    frame = parts[0] if parts else raw
    req_id: Optional[int] = None
    req_uuid = ""
    meta: dict[str, Any] = {}
    for token in parts[1:]:
        key, sep, value = token.partition("=")
        if sep != "=":
            continue
        k = key.strip().lower()
        v = value.strip()
        if not v:
            continue
        if k in ("rid", "request_id"):
            try:
                req_id = int(v)
            except Exception:
                req_id = None
        elif k in ("uid", "uuid", "request_uuid"):
            req_uuid = v
        elif k in ("tol", "tol_m", "ee_tol", "ee_target_tol_m"):
            try:
                meta["tol_m"] = float(v)
            except Exception:
                pass
        elif k in ("phase", "label"):
            meta["phase_label"] = v
    return frame, req_id, req_uuid, meta


# ---------------------------------------------------------------------------
# MoveIt plan success helpers
# ---------------------------------------------------------------------------

def plan_success_code(success: Any) -> Optional[int]:
    """Extract an integer error code from a MoveIt plan *success* value."""
    if success is None:
        return None
    if isinstance(success, bool):
        return 1 if success else 0
    if isinstance(success, (int, float)):
        return int(success)
    for attr in ("val", "value", "code"):
        try:
            if hasattr(success, attr):
                return int(getattr(success, attr))
        except Exception:
            continue
    return None


def plan_success_ok(success: Any) -> bool:
    """Return True when *success* indicates MoveItErrorCodes.SUCCESS (== 1)."""
    # moveit_msgs/MoveItErrorCodes.SUCCESS == 1
    code = plan_success_code(success)
    if code is not None:
        return int(code) == 1
    if isinstance(success, str):
        token = success.strip().upper()
        if token in ("SUCCESS", "SUCCEEDED", "OK"):
            return True
        if token in ("FAIL", "FAILED", "ERROR", "ABORTED"):
            return False
    if success is None:
        # Some bindings omit explicit success when trajectory is valid.
        return True
    return bool(success)


def plan_error_code_val(plan: Any) -> Optional[int]:
    """Return integer MoveIt error code from *plan*.error_code, or None."""
    ec = getattr(plan, "error_code", None)
    if ec is None:
        return None
    # moveit_msgs/MoveItErrorCodes has .val; raw int also possible.
    val = getattr(ec, "val", None)
    if val is not None:
        return int(val)
    if isinstance(ec, (int, float)):
        return int(ec)
    return None


# ---------------------------------------------------------------------------
# Diagnostics / result formatting
# ---------------------------------------------------------------------------

def describe_execute_result(result: Any) -> dict[str, Any]:
    """Build a lightweight metadata dict describing a MoveIt execute *result*."""
    if result is None:
        return {"type": "NoneType"}
    meta: dict[str, Any] = {"type": type(result).__name__}
    for key in ("success", "status", "message", "error_code", "val"):
        try:
            value = getattr(result, key)
        except Exception:
            continue
        if value is not None:
            meta[key] = value
    return meta


def result_meta_to_message(meta: dict[str, Any]) -> str:
    """Flatten a result metadata dict into a comma-separated log line."""
    pieces: list[str] = []
    for key in (
        "type",
        "success",
        "status",
        "status_text",
        "message",
        "error_code",
        "error_string",
        "action",
        "accepted",
        "timeout_sec",
        "goal_check",
        "ee_goal_check",
        "joint_goal_check",
        "joint_motion",
        "feedback_goal_check",
        "retry_timeout_sec",
        "retry_goal_time_tol_sec",
        "diag_reason",
        "joint_state_age_sec",
        "controller_action_available",
        "active_controllers",
        "val",
    ):
        if key in meta:
            pieces.append(f"{key}={meta[key]}")
    return ",".join(pieces) if pieces else "none"


def diag_to_message(diag: dict[str, Any]) -> str:
    """Format a bridge diagnostics dict into a single-line string."""
    js_age = diag.get("joint_state_age_sec")
    js_age_txt = "n/a" if js_age is None else f"{float(js_age):.3f}"
    return (
        f"{diag.get('reason')};backend={diag.get('backend')};"
        f"use_sim_time={str(diag.get('use_sim_time')).lower()};"
        f"controller={diag.get('controller')};"
        f"action={diag.get('controller_action')};"
        f"action_matched={diag.get('controller_action_matched') or 'none'};"
        f"action_available={str(diag.get('controller_action_available')).lower()};"
        f"controller_manager={diag.get('controller_manager')};"
        f"active_controllers={','.join(diag.get('active_controllers') or []) or 'none'};"
        f"joint_state_age_sec={js_age_txt};"
        f"trajectory_topic_pubs={diag.get('trajectory_topic_pubs')};"
        f"trajectory_topic_subs={diag.get('trajectory_topic_subs')}"
    )


def goal_status_text(status: int) -> str:
    """Return a human-readable string for a ROS action GoalStatus integer."""
    if GoalStatus is None:
        _mapping: dict[int, str] = {
            0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING",
            3: "CANCELING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED",
        }
        return _mapping.get(int(status), f"STATUS_{int(status)}")
    mapping = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return mapping.get(int(status), f"STATUS_{int(status)}")


# ---------------------------------------------------------------------------
# Async helpers
# ---------------------------------------------------------------------------

def wait_future_done(future: Any, timeout_sec: float) -> bool:
    """Poll *future* until it is done or *timeout_sec* elapses. Returns done status."""
    deadline = time.monotonic() + max(0.1, timeout_sec)
    while time.monotonic() < deadline:
        if future.done():
            return True
        time.sleep(0.02)
    return bool(future.done())


# ---------------------------------------------------------------------------
# JointTrajectory helpers
# ---------------------------------------------------------------------------

def joint_trajectory_duration_sec(jt: Any) -> float:
    """Return the total duration (seconds) of a JointTrajectory, or 0.0."""
    if jt is None:
        return 0.0
    points = list(getattr(jt, "points", []) or [])
    if not points:
        return 0.0
    tfs = getattr(points[-1], "time_from_start", None)
    if tfs is None:
        return 0.0
    try:
        return max(0.0, float(tfs.sec) + float(tfs.nanosec) / 1_000_000_000.0)
    except Exception:
        return 0.0


def joint_trajectory_initial_segment_max_delta(jt: Any) -> float:
    """Return max joint delta between the first two points of *jt*, or -1 if unknown."""
    points = list(getattr(jt, "points", []) or [])
    if len(points) < 2:
        return -1.0
    try:
        p0 = list(getattr(points[0], "positions", []) or [])
        p1 = list(getattr(points[1], "positions", []) or [])
        if len(p0) != len(p1) or not p0:
            return -1.0
        return max(abs(float(b) - float(a)) for a, b in zip(p0, p1))
    except Exception:
        return -1.0


def scale_joint_trajectory_timing(jt: Any, scale: float = 2.0) -> Any:
    """Return a deep copy of *jt* with all time_from_start values scaled by *scale*."""
    scaled = deepcopy(jt)
    factor = max(1.0, float(scale))
    for point in list(getattr(scaled, "points", []) or []):
        tfs = getattr(point, "time_from_start", None)
        if tfs is None:
            continue
        total = (
            float(getattr(tfs, "sec", 0) or 0)
            + float(getattr(tfs, "nanosec", 0) or 0) / 1_000_000_000.0
        )
        total = max(0.0, total * factor)
        sec = int(total)
        nsec = int(round((total - sec) * 1_000_000_000.0))
        if nsec >= 1_000_000_000:  # pragma: no cover — round() boundary, unreachable in practice
            sec += 1
            nsec -= 1_000_000_000
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nsec
    return scaled


# ---------------------------------------------------------------------------
# Geometry / pose math
# ---------------------------------------------------------------------------

def pose_to_matrix(pose: Any) -> "np.ndarray":
    """Convert geometry_msgs.msg.Pose to a 4×4 numpy homogeneous transform."""
    q = pose.orientation
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ], dtype=float)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[0, 3] = float(pose.position.x)
    T[1, 3] = float(pose.position.y)
    T[2, 3] = float(pose.position.z)
    return T


def matrix_to_pose(T: "np.ndarray") -> Any:
    """Convert a 4×4 numpy homogeneous transform to geometry_msgs.msg.Pose."""
    from geometry_msgs.msg import Pose as _Pose
    R = T[:3, :3]
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2]))
        w = (R[2, 1] - R[1, 2]) / s if s > 1e-9 else 1.0
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s if s > 1e-9 else 0.0
        z = (R[0, 2] + R[2, 0]) / s if s > 1e-9 else 0.0
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(max(0.0, 1.0 + R[1, 1] - R[0, 0] - R[2, 2]))
        w = (R[0, 2] - R[2, 0]) / s if s > 1e-9 else 1.0
        x = (R[0, 1] + R[1, 0]) / s if s > 1e-9 else 0.0
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s if s > 1e-9 else 0.0
    else:
        s = 2.0 * math.sqrt(max(0.0, 1.0 + R[2, 2] - R[0, 0] - R[1, 1]))
        w = (R[1, 0] - R[0, 1]) / s if s > 1e-9 else 1.0
        x = (R[0, 2] + R[2, 0]) / s if s > 1e-9 else 0.0
        y = (R[1, 2] + R[2, 1]) / s if s > 1e-9 else 0.0
        z = 0.25 * s
    pose = _Pose()
    pose.position.x = float(T[0, 3])
    pose.position.y = float(T[1, 3])
    pose.position.z = float(T[2, 3])
    pose.orientation.x = float(x)
    pose.orientation.y = float(y)
    pose.orientation.z = float(z)
    pose.orientation.w = float(w)
    return pose


# ---------------------------------------------------------------------------
# TF stamp freshness helpers
# ---------------------------------------------------------------------------

def stamp_age_sec(stamp_sec: float, now_sec: float) -> float:
    """Return how many seconds old a TF stamp is (non-negative)."""
    return max(0.0, now_sec - stamp_sec)


def is_stamp_fresh(stamp_sec: float, now_sec: float, max_age_sec: float) -> bool:
    """Return True if the stamp is younger than *max_age_sec* seconds."""
    if max_age_sec < 0:
        return True  # sentinel: freshness checking disabled
    return stamp_age_sec(stamp_sec, now_sec) <= max_age_sec
