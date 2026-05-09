#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_utils.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_utils.py
# Summary: Helpers and runners shared by the SUPER PRO panel.
from __future__ import annotations

import json
import os
import re
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import math
try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None
try:
    import psutil  # type: ignore
except Exception:  # pragma: no cover
    psutil = None

from .logging_utils import emit_log_line, timestamped_line
from .panel_env import env_str
from .panel_ui_params import get_panel_ui_params as _get_panel_ui_params
from .panel_process import (  # noqa: F401
    GZ_LOG_FILTERS,
    bash_preamble,
    build_log_filter_cmd,
    build_gz_env,
    ensure_dir,
    log_to_file,
    now_tag,
    resolve_gz_partition,
    rotate_log,
    run_cmd,
    safe_topic_name,
    with_line_buffer,
)
from .panel_workers import CmdRunner  # noqa: F401
from .panel_ros import RosWorker  # noqa: F401
from .panel_tf import get_tf_helper, shutdown_tf_helper  # noqa: F401
from .panel_objects import (  # noqa: F401
    bulk_update_object_positions,
    get_object_position,
    get_object_positions,
    load_object_positions,
    save_object_positions,
)

try:
    # Import correcto cuando ur5_qt_panel es un paquete (colcon/ament).
    # Las constantes de tabla y robot se re-exportan deliberadamente para
    # mantener compatibilidad con callers legacy que las importaban via
    # ``from .panel_utils import (...)``.
    from .panel_config import (  # noqa: F401
        ARM_TRAJ_TOPIC_DEFAULT,
        GZ_WORLD,
        SCRIPTS_DIR,
        TABLE_CALIB_PATH,
        TABLE_CENTER_X,
        TABLE_CENTER_Y,
        TABLE_IMAGE_FLIP_X,
        TABLE_IMAGE_FLIP_Y,
        TABLE_IMAGE_SWAP_XY,
        TABLE_PIXEL_AFFINE,
        TABLE_PIXEL_HOMOGRAPHY,
        TABLE_PIXEL_RECT,
        TABLE_CAM_INFO,
        TABLE_SIZE_X,
        TABLE_SIZE_Y,
        TABLE_OBJECT_XY_MARGIN,
        TABLE_OBJECT_Z_MIN,
        TABLE_OBJECT_Z_MAX,
        TABLE_OBJECT_WHITELIST,
        UR5_BASE_X,
        UR5_BASE_Y,
        UR5_BASE_Z,
        UR5_CONTROLLERS_YAML,
        UR5_HOME_DEFAULT,
        UR5_HOME_ENV,
        UR5_MODEL_NAME,
        UR5_REACH_RADIUS,
        BASE_FRAME,
        WORLD_FRAME,
        ROS_AVAILABLE,
        qos_profile_sensor_data,
        rclpy,
    )
except Exception:
    # Fallback si alguien ejecuta módulos fuera del contexto de paquete.
    from .panel_config import (  # type: ignore  # noqa: F401
        ARM_TRAJ_TOPIC_DEFAULT,
        GZ_WORLD,
        SCRIPTS_DIR,
        TABLE_CALIB_PATH,
        TABLE_CENTER_X,
        TABLE_CENTER_Y,
        TABLE_IMAGE_FLIP_X,
        TABLE_IMAGE_FLIP_Y,
        TABLE_IMAGE_SWAP_XY,
        TABLE_PIXEL_AFFINE,
        TABLE_PIXEL_HOMOGRAPHY,
        TABLE_PIXEL_RECT,
        TABLE_CAM_INFO,
        TABLE_SIZE_X,
        TABLE_SIZE_Y,
        TABLE_OBJECT_XY_MARGIN,
        TABLE_OBJECT_Z_MIN,
        TABLE_OBJECT_Z_MAX,
        TABLE_OBJECT_WHITELIST,
        UR5_BASE_X,
        UR5_BASE_Y,
        UR5_BASE_Z,
        UR5_CONTROLLERS_YAML,
        UR5_HOME_DEFAULT,
        UR5_HOME_ENV,
        UR5_MODEL_NAME,
        UR5_REACH_RADIUS,
        BASE_FRAME,
        WORLD_FRAME,
        ROS_AVAILABLE,
        qos_profile_sensor_data,
        rclpy,
    )

# Geometria pixel<->world<->table extraida a panel_pixel_geometry (B.2 refactor).
# Reexportado aqui para mantener compatibilidad con todos los callers que
# importan estas helpers como ``from .panel_utils import (...)``.
from .panel_pixel_geometry import (  # noqa: F401
    _apply_homography,
    _invert_3x3,
    _norm_to_pixel,
    _pixel_to_norm,
    _pixel_to_world_at_z,
    _solve_linear_system,
    compute_homography,
    norm_to_pixel,
    pixel_to_norm,
    pixel_to_table_xy,
    table_xy_to_pixel,
    table_xy_to_pixel_float,
    world_xyz_to_pixel,
    world_xyz_to_pixel_float,
)

# Helpers de controller_manager extraidos a panel_controllers_query (B.3).
from .panel_controllers_query import (  # noqa: F401
    _controller_manager_path,
    _discover_controller_manager,
    gripper_controller_defined,
    list_active_controllers,
    list_controllers_state,
    resolve_controller_manager,
    ros2_control_running,
)

# Helpers de status del sistema extraidos a panel_system_status (B.4).
from .panel_system_status import (  # noqa: F401
    _create_graph_node,
    _run_cmd_rc,
    bridge_status,
    clock_status,
    detect_arm_trajectory_topic,
    gz_sim_running,
    gz_sim_status,
    parse_ros_topics,
    robot_control_available,
    ros_clock_available,
)

# Helpers de discovery TF extraidos a panel_tf_discovery (B.5).
from .panel_tf_discovery import (  # noqa: F401
    EE_FRAME_CANDIDATE_BASES,
    EE_FRAME_SUBSTRING_KEYWORDS,
    ROBOT_FRAME_KEYWORDS,
    _build_frame_graph,
    _can_transform_between,
    _collect_leaf_frames,
    _extract_frames_from_yaml,
    _frame_depth,
    _load_tf_frame_records,
    _log_ee_unavailable_once,
    _log_tf_frame_summary,
    _log_tf_frames_once,
    _log_tf_yaml_head_once,
    _parse_tf_yaml_records,
    _preferred_base_frame,
    debug_dump_tf,
    discover_base_and_ee_frames,
    discover_robot_base_frame,
    discover_world_frame,
)

# Helpers de objetos de mesa + home pose extraidos a panel_table_objects (B.6).
from .panel_table_objects import (  # noqa: F401
    _parse_pose_json,
    get_object_pose_gz,
    load_home_pose,
    nearest_table_object,
    object_out_of_reach,
    save_home_pose,
    visible_table_object,
)

# Helpers de seleccion de frames + get_pose extraidos a panel_pose_helpers (B.7).
from .panel_pose_helpers import (  # noqa: F401
    BASE_FRAME_CANDIDATES,
    EE_FRAME_PREFERENCE,
    WORLD_FRAME_CANDIDATES,
    _select_base_frame,
    _select_ee_frame,
    effective_base_frame,
    effective_world_frame,
    get_pose,
    lookup_pose,
    transform_pose,
)

try:
    from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, TransformStamped
    from builtin_interfaces.msg import Time as BuiltinTime
    from controller_manager_msgs.srv import ListControllers
    from std_msgs.msg import Empty, String
    from tf2_ros import (
        Buffer,
        TransformListener,
        StaticTransformBroadcaster,
        LookupException,
        ConnectivityException,
        ExtrapolationException,
        TransformRegistration,
        TypeException,
    )
    from tf2_msgs.msg import TFMessage
    from rclpy.time import Time
    from rclpy.parameter import Parameter
    try:
        import tf2_geometry_msgs  # noqa: F401
    except Exception:
        tf2_geometry_msgs = None  # type: ignore
except Exception:
    PointStamped = None  # type: ignore
    PoseStamped = None  # type: ignore
    Quaternion = None  # type: ignore
    TransformStamped = None  # type: ignore
    BuiltinTime = None  # type: ignore
    tf2_geometry_msgs = None  # type: ignore
    Buffer = None  # type: ignore
    TransformListener = None  # type: ignore
    StaticTransformBroadcaster = None  # type: ignore
    LookupException = Exception  # type: ignore
    ConnectivityException = Exception  # type: ignore
    ExtrapolationException = Exception  # type: ignore
    TransformRegistration = None  # type: ignore
    TypeException = Exception  # type: ignore
    Time = None  # type: ignore
    TFMessage = None  # type: ignore
    Empty = None  # type: ignore
    String = None  # type: ignore
    ListControllers = None  # type: ignore
    Parameter = None  # type: ignore


ROS_TOPIC_RE = re.compile(r"^/([A-Za-z0-9_]+/)*[A-Za-z0-9_]+$")

_DEBUG_EXCEPTIONS = _get_panel_ui_params().debug_exceptions
_GRIPPER_CTRL_CACHE: Tuple[Optional[float], Optional[bool]] = (None, None)
_CM_CACHE: Tuple[float, str] = (0.0, "/controller_manager")
_CM_CACHE_TTL_SEC = 2.0


def _log_exception(context: str, exc: Exception) -> None:
    if not _DEBUG_EXCEPTIONS:
        return
    emit_log_line(timestamped_line(f"[PANEL_UTILS][WARN] {context}: {exc}"))


_TF_TRANSFORM_WARN_LAST: Dict[str, float] = {}
_TF_TRANSFORM_WARN_COUNT: Dict[str, int] = {}
_TF_TRANSFORM_WARN_PERIOD = 5.0
_GZ_SIM_STATUS_CACHE: Tuple[float, bool, str] = (0.0, False, "none")
_GZ_SIM_STATUS_TTL_SEC = 0.5
_WORLD_NAME_CACHE: Dict[str, Tuple[float, str]] = {}


def _log_tf_transform_warning(context: str, exc: Exception) -> None:
    global _TF_TRANSFORM_WARN_LAST, _TF_TRANSFORM_WARN_COUNT
    now = time.monotonic()
    key = f"{context}:{exc.__class__.__name__}"
    _TF_TRANSFORM_WARN_COUNT[key] = _TF_TRANSFORM_WARN_COUNT.get(key, 0) + 1
    last = _TF_TRANSFORM_WARN_LAST.get(key, 0.0)
    if (now - last) < _TF_TRANSFORM_WARN_PERIOD:
        return
    _TF_TRANSFORM_WARN_LAST[key] = now
    count = _TF_TRANSFORM_WARN_COUNT.get(key, 0)
    _TF_TRANSFORM_WARN_COUNT[key] = 0
    msg = f"[TRACE][TF] {context} failed ({count}): {exc}"
    emit_log_line(timestamped_line(msg))


def _transform_xyz_via_tf(
    coords: Tuple[float, float, float],
    source_frame: str,
    target_frame: str,
    *,
    timeout_sec: float = 0.05,
) -> Optional[Tuple[float, float, float]]:
    """Transform XYZ coordinates via TF when a live transform is available."""
    helper = get_tf_helper()
    if helper is None or PointStamped is None or rclpy is None:
        return None
    try:
        point = PointStamped()
        point.header.frame_id = str(source_frame or "").strip() or "world"
        if BuiltinTime is not None:
            point.header.stamp = BuiltinTime(sec=0, nanosec=0)
        else:
            point.header.stamp = rclpy.time.Time().to_msg()
        point.point.x = float(coords[0])
        point.point.y = float(coords[1])
        point.point.z = float(coords[2])
        converted = helper.transform_point(
            point,
            str(target_frame or "").strip() or "world",
            timeout_sec=timeout_sec,
        )
        if not converted:
            return None
        return (
            float(converted.point.x),
            float(converted.point.y),
            float(converted.point.z),
        )
    except Exception as exc:
        _log_tf_transform_warning(
            f"transform_xyz {source_frame}->{target_frame}",
            exc,
        )
        return None


def base_to_world(x: float, y: float, z: float) -> Tuple[float, float, float]:
    world_frame = str(WORLD_FRAME or "world").strip() or "world"
    transformed = _transform_xyz_via_tf(
        (float(x), float(y), float(z)),
        "base_link",
        world_frame,
    )
    if transformed is not None:
        return transformed
    return (x + UR5_BASE_X, y + UR5_BASE_Y, z + UR5_BASE_Z)

def read_world_name(world_path: str) -> str:
    """Extract the Gazebo world name from the SDF/WORLD file."""
    if not world_path:
        return ""
    try:
        mtime = os.path.getmtime(world_path)
    except Exception:
        mtime = -1.0
    cached = _WORLD_NAME_CACHE.get(world_path)
    if cached and cached[0] == mtime:
        return cached[1]
    try:
        text = Path(world_path).read_text(encoding="utf-8", errors="ignore")
        match = re.search(r'<world\s+name="([^"]+)"', text)
        if match:
            name = match.group(1)
            _WORLD_NAME_CACHE[world_path] = (mtime, name)
            return name
    except Exception as exc:
        _log_exception("read_world_name", exc)
    name = Path(world_path).stem
    _WORLD_NAME_CACHE[world_path] = (mtime, name)
    return name


def set_led(label, state: str):
    """Update a QLabel-based LED indicator."""
    colors = {
        "off": "#6b7280",
        "on": "#22c55e",
        "warn": "#f59e0b",
        "error": "#ef4444",
    }
    color = colors.get(state, colors["off"])
    label.setFixedSize(8, 10)
    label.setStyleSheet(
        f"background:{color}; border-radius:2px; border:1px solid #374151;"
    )


def kb_to_gb(kb: int) -> float:
    return kb / 1024.0 / 1024.0


def load_table_calib() -> Optional[object]:
    if not os.path.isfile(TABLE_CALIB_PATH):
        return None
    try:
        with open(TABLE_CALIB_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        global TABLE_CAM_INFO
        global TABLE_PIXEL_AFFINE
        global TABLE_PIXEL_RECT
        global TABLE_PIXEL_HOMOGRAPHY
        TABLE_CAM_INFO = None
        TABLE_PIXEL_AFFINE = None
        TABLE_PIXEL_RECT = None
        TABLE_PIXEL_HOMOGRAPHY = None
        cam = data.get("camera") if isinstance(data, dict) else None
        if isinstance(cam, dict):
            pos = cam.get("position")
            rot = cam.get("rotation")
            fx = cam.get("fx")
            fy = cam.get("fy")
            cx = cam.get("cx")
            cy = cam.get("cy")
            width = cam.get("width")
            height = cam.get("height")
            if (
                isinstance(pos, list) and len(pos) == 3
                and isinstance(rot, list) and len(rot) == 3
                and all(isinstance(row, list) and len(row) == 3 for row in rot)
            ):
                try:
                    TABLE_CAM_INFO = {
                        "position": [float(pos[0]), float(pos[1]), float(pos[2])],
                        "rotation": [[float(v) for v in row] for row in rot],
                        "fx": float(fx),
                        "fy": float(fy),
                        "cx": float(cx),
                        "cy": float(cy),
                        "width": int(width),
                        "height": int(height),
                    }
                except (TypeError, ValueError):
                    TABLE_CAM_INFO = None
        mode = data.get("mode")
        if mode == "rect":
            p1 = data.get("p1")
            p2 = data.get("p2")
            w1 = data.get("w1")
            w2 = data.get("w2")
            if all(isinstance(v, list) and len(v) == 2 for v in (p1, p2, w1, w2)):
                dx = float(p2[0]) - float(p1[0])
                dy = float(p2[1]) - float(p1[1])
                if abs(dx) < 50 or abs(dy) < 50:
                    return None
                sx = (float(w2[0]) - float(w1[0])) / dx
                sy = (float(w2[1]) - float(w1[1])) / dy
                if abs(sx) < 0.0003 or abs(sy) < 0.0003 or abs(sx) > 0.02 or abs(sy) > 0.02:
                    return None
                TABLE_PIXEL_RECT = {"p1": tuple(p1), "p2": tuple(p2), "w1": tuple(w1), "w2": tuple(w2)}
                return TABLE_PIXEL_RECT
        if mode == "homography":
            mat = data.get("h")
            if (
                isinstance(mat, list)
                and len(mat) == 3
                and all(isinstance(row, list) and len(row) == 3 for row in mat)
            ):
                mat = [[float(v) for v in row] for row in mat]
                if _invert_3x3(mat):
                    TABLE_PIXEL_HOMOGRAPHY = mat
                    return TABLE_PIXEL_HOMOGRAPHY
        affine = data.get("affine")
        if (
            isinstance(affine, list)
            and len(affine) == 2
            and all(isinstance(row, list) and len(row) == 3 for row in affine)
        ):
            mat = [[float(v) for v in row] for row in affine]
            det = (mat[0][0] * mat[1][1]) - (mat[0][1] * mat[1][0])
            if abs(det) < 1e-8:
                return None
            TABLE_PIXEL_AFFINE = mat
            return TABLE_PIXEL_AFFINE
    except Exception:
        return None
    return None


def angle_shortest_diff_rad(current: float, target: float) -> float:
    """Return the shortest signed angular distance between two angles."""
    delta = float(current) - float(target)
    return math.atan2(math.sin(delta), math.cos(delta))


def angle_shortest_diff_rad(current: float, target: float) -> float:
    """Return the shortest signed angular distance between two angles."""
    delta = float(current) - float(target)
    return math.atan2(math.sin(delta), math.cos(delta))


def world_to_base(x: float, y: float, z: float) -> Tuple[float, float, float]:
    world_frame = str(WORLD_FRAME or "world").strip() or "world"
    transformed = _transform_xyz_via_tf(
        (float(x), float(y), float(z)),
        world_frame,
        "base_link",
    )
    if transformed is not None:
        return transformed
    return (x - UR5_BASE_X, y - UR5_BASE_Y, z - UR5_BASE_Z)


def base_frame_candidates(
    base_frame_effective: Optional[str],
    base_frame_setting: Optional[str],
    *,
    fallbacks: Tuple[str, ...] = ("base_link",),
) -> List[str]:
    """Return ordered base frame candidates without duplicates."""
    candidates: List[str] = []
    if base_frame_effective == "base_link":
        candidates.append(base_frame_effective)
    if base_frame_setting == "base_link" and base_frame_setting not in candidates:
        candidates.append(base_frame_setting)
    for fallback in fallbacks:
        if fallback not in candidates:
            candidates.append(fallback)
    return candidates







# ------------------------------------------------------------------
# cold boot helpers
# ------------------------------------------------------------------

def cold_boot_kill(log_fn):
    log_fn("[COLD] cold_boot_kill -> scripts/kill_all.sh")
    script = os.path.join(SCRIPTS_DIR, "kill_all.sh")
    if not os.path.isfile(script):
        log_fn(f"[COLD] No existe {script}")
        return
    res = run_cmd(f"'{script}' || true", timeout=10, capture_output=True)
    if res.returncode != 0:
        log_fn(f"[COLD] ERROR al ejecutar kill_all.sh: {res.stderr or res.stdout}")


# ------------------------------------------------------------------
# command helpers
# ------------------------------------------------------------------

def yaw_from_quaternion(quat: "Quaternion") -> float:
    """Return yaw angle (Z) from quaternion."""
    if quat is None:
        return 0.0
    siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
    return math.atan2(siny_cosp, cosy_cosp)






def tf_world_base_valid(panel, helper, base_frame: str, world_frame: str) -> bool:
    """Validate world->base transform and guard against identity in non-zero base offsets."""
    transform = helper.lookup_transform(base_frame, world_frame, timeout_sec=0.15)
    if not transform:
        return False
    t = transform.transform.translation
    if abs(t.x) < 1e-3 and abs(t.y) < 1e-3 and abs(t.z) < 1e-3:
        if abs(UR5_BASE_X) > 0.1 or abs(UR5_BASE_Y) > 0.1 or abs(UR5_BASE_Z) > 0.1:
            msg = "[TF][ERROR] world->base es identidad; TF inválido para este mundo"
            if not getattr(panel, "_tf_invalid", False):
                panel._emit_log(msg)
                panel._ui_set_status("TF inválido: world->base es identidad", error=True)
                panel._trigger_fatal(msg)
            panel._tf_invalid = True
            return False
    panel._tf_invalid = False
    if getattr(panel, "_system_error_reason", "") == "[TF][ERROR] world->base es identidad; TF inválido para este mundo":
        panel._system_error_reason = ""
    return True


def tf_world_base_label(panel, *, default_base: str = "base_link") -> str:
    base_frame = effective_base_frame(panel, default=default_base)
    return f"TF world->{base_frame}"


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Return quaternion (x,y,z,w) from euler angles."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def detect_base_frame(
    source_frame: str = "world",
    timeout_sec: float = 0.2,
    candidates: Optional[List[str]] = None,
) -> Tuple[Optional[str], Optional[object], Optional[str]]:
    helper = get_tf_helper()
    if not helper:
        return None, None, "TF helper unavailable"
    # The canonical _BASE_FRAME_CACHE lives in panel_tf_discovery (see
    # panel_pose_helpers, which does `_tfd._BASE_FRAME_CACHE = ...`). Writing
    # `global _BASE_FRAME_CACHE` here used to create a panel_utils-only
    # attribute that nobody read, so the cache update was silently lost.
    # Update the shared one instead.
    try:
        from . import panel_tf_discovery as _tfd
    except Exception:
        _tfd = None  # type: ignore
    scan = ["base_link"]
    for frame in scan:
        if not frame:
            continue
        transform = helper.lookup_transform(frame, source_frame, timeout_sec=timeout_sec)
        if transform:
            if _tfd is not None:
                _tfd._BASE_FRAME_CACHE = frame
            return frame, transform, None
    return None, None, f"lookup {source_frame}->base_link timed out"


def _rotation_matrix_from_quaternion(quat: "Quaternion") -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
    """Return rotation matrix built from quaternion."""
    if quat is None:
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    w = float(quat.w)
    x = float(quat.x)
    y = float(quat.y)
    z = float(quat.z)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _apply_rotation(matrix: Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]], vector: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Multiply 3x3 matrix by vector."""
    x, y, z = vector
    row0, row1, row2 = matrix
    return (
        row0[0] * x + row0[1] * y + row0[2] * z,
        row1[0] * x + row1[1] * y + row1[2] * z,
        row2[0] * x + row2[1] * y + row2[2] * z,
    )


def _do_transform_point(point: "PointStamped", transform: "TransformStamped") -> "PointStamped":
    """Apply transform stamped to a PointStamped."""
    rotation = _rotation_matrix_from_quaternion(transform.transform.rotation)
    px = point.point.x
    py = point.point.y
    pz = point.point.z
    rx, ry, rz = _apply_rotation(rotation, (px, py, pz))
    translation = transform.transform.translation
    res = PointStamped()
    res.header.frame_id = transform.header.frame_id
    res.header.stamp = transform.header.stamp
    res.point.x = rx + translation.x
    res.point.y = ry + translation.y
    res.point.z = rz + translation.z
    return res


def _apply_transform_to_tuple(point: Tuple[float, float, float], transform: "TransformStamped") -> Tuple[float, float, float]:
    rotation = _rotation_matrix_from_quaternion(transform.transform.rotation)
    rx, ry, rz = _apply_rotation(rotation, point)
    translation = transform.transform.translation
    return (rx + translation.x, ry + translation.y, rz + translation.z)


def _register_point_stamp_tf():
    """Register PointStamped transform if not already available."""
    if PointStamped is None or TransformStamped is None or TransformRegistration is None:
        return
    try:
        TransformRegistration().get(PointStamped)
        return
    except TypeException:
        pass
    try:
        TransformRegistration().add(PointStamped, _do_transform_point)
    except Exception:
        pass


_register_point_stamp_tf()


def transform_point_to_frame(
    world_pos: Tuple[float, float, float],
    target_frame: str,
    source_frame: str = "world",
    timeout_sec: float = 0.6,
) -> Tuple[Optional[Tuple[float, float, float]], Optional[object]]:
    helper = get_tf_helper()
    if helper is None or world_pos is None:
        return None, None
    if PoseStamped is None:
        return None, None
    target_frame_norm = str(target_frame or BASE_FRAME or "base_link").strip() or "base_link"
    source_frame_norm = str(source_frame or WORLD_FRAME or "world").strip() or "world"
    if tf2_geometry_msgs is None:
        try:
            transform = helper.lookup_transform(
                target_frame_norm,
                source_frame_norm,
                timeout_sec=timeout_sec,
            )
            if not transform:
                return None, None
            coords = _apply_transform_to_tuple(world_pos, transform)
            return coords, transform
        except Exception as exc:
            _log_tf_transform_warning("transform_point_to_frame", exc)
            return None, None
    try:
        pose = PoseStamped()
        pose.header.frame_id = source_frame_norm
        if BuiltinTime is not None:
            pose.header.stamp = BuiltinTime(sec=0, nanosec=0)
        else:
            pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = world_pos
        pose.pose.orientation.w = 1.0
        transformed = helper.transform_pose(pose, target_frame_norm, timeout_sec)
        if not transformed:
            transform = helper.lookup_transform(
                target_frame_norm,
                source_frame_norm,
                timeout_sec=timeout_sec,
            )
            if not transform:
                return None, None
            coords = _apply_transform_to_tuple(world_pos, transform)
            return coords, transform
        coords = (
            transformed.pose.position.x,
            transformed.pose.position.y,
            transformed.pose.position.z,
        )
        transform = helper.lookup_transform(
            target_frame_norm,
            source_frame_norm,
            timeout_sec=timeout_sec,
        )
        return coords, transform
    except Exception as exc:
        _log_tf_transform_warning("transform_point_to_frame", exc)
        return None, None


def world_to_base_coords(
    world_pos: Tuple[float, float, float], frame: str = "world", require_tf: bool = False
) -> Tuple[
    Optional[Tuple[float, float, float]], Optional[object], Optional[str]
]:
    """Transform *world_pos* from *frame* into BASE_FRAME using TF if available."""
    helper = get_tf_helper()
    target_frame = "base_link"
    if helper:
        transform = helper.lookup_transform(target_frame, frame, timeout_sec=1.0)
        if transform:
            point = PointStamped()
            point.header.frame_id = frame
            point.header.stamp = rclpy.time.Time()
            point.point.x, point.point.y, point.point.z = world_pos
            converted = helper.transform_point(point, target_frame, timeout_sec=0.6)
            if converted:
                coords = (
                    converted.point.x,
                    converted.point.y,
                    converted.point.z,
                )
                return coords, transform, None
        if require_tf:
            return None, None, f"lookup {frame}->{target_frame} timed out"
    if require_tf:
        reason = "TF helper unavailable" if helper else "no TF helper"
        return None, None, reason
    bx, by, bz = world_to_base(*world_pos)
    return (bx, by, bz), None, None


def _list_tf_topics() -> Tuple[List[str], List[str]]:
    """Return the available tf and tf_static topics."""
    node = _create_graph_node("panel_tf_topics")
    if node is None:
        return [], []
    try:
        topics = node.get_topic_names_and_types()
        tf = [name for name, _ in topics if name == "/tf"]
        tf_static = [name for name, _ in topics if name == "/tf_static"]
        return tf, tf_static
    except Exception:
        return [], []
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


def _parse_static_tf_env() -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
    env = env_str("PANEL_STATIC_TF", "").strip()
    if not env:
        return None
    parts = env.split()
    if len(parts) != 6:
        return None
    try:
        x, y, z, roll, pitch, yaw = [float(v) for v in parts]
    except ValueError:
        return None
    quat = _euler_to_quaternion(roll, pitch, yaw)
    return (x, y, z), quat


def _publish_static_tf(world_frame: str, base_frame: str) -> Tuple[bool, Optional[str]]:
    if not ROS_AVAILABLE or StaticTransformBroadcaster is None:
        return False, "ROS unavailable"
    translation = None
    rotation = None
    pose = get_object_pose_gz(UR5_MODEL_NAME)
    if pose:
        pos = pose.get("position") or {}
        orient = pose.get("orientation") or {}
        try:
            translation = (
                float(pos.get("x", 0.0)),
                float(pos.get("y", 0.0)),
                float(pos.get("z", 0.0)),
            )
            rotation = (
                float(orient.get("x", 0.0)),
                float(orient.get("y", 0.0)),
                float(orient.get("z", 0.0)),
                float(orient.get("w", 1.0)),
            )
        except Exception:
            translation = None
            rotation = None
    if translation is None:
        manual = _parse_static_tf_env()
        if manual:
            translation, rotation = manual
    if translation is None or rotation is None:
        return False, "no robot pose available"
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
    except Exception:
        pass
    node = None
    broadcaster = None
    try:
        node = rclpy.create_node("panel_static_tf")
        broadcaster = StaticTransformBroadcaster(node)
        tfs = TransformStamped()
        tfs.header.stamp = node.get_clock().now().to_msg()
        tfs.header.frame_id = world_frame
        tfs.child_frame_id = base_frame
        tfs.transform.translation.x = translation[0]
        tfs.transform.translation.y = translation[1]
        tfs.transform.translation.z = translation[2]
        tfs.transform.rotation.x = rotation[0]
        tfs.transform.rotation.y = rotation[1]
        tfs.transform.rotation.z = rotation[2]
        tfs.transform.rotation.w = rotation[3]
        broadcaster.sendTransform(tfs)
        time.sleep(0.05)
    except Exception as exc:
        if node:
            node.destroy_node()
        return False, f"static tf error: {exc}"
    if node:
        node.destroy_node()
    return True, f"Using STATIC TF {world_frame}->{base_frame}"


def diagnose_tf_tree(
    world_pose: Optional[Tuple[float, float, float]],
    selection_frame: Optional[str] = None,
) -> Dict[str, object]:
    tf_topics, tf_static = _list_tf_topics()
    world_candidates = []
    if selection_frame:
        world_candidates.append(selection_frame)
    world_candidates.extend([WORLD_FRAME, *WORLD_FRAME_CANDIDATES])
    # remove duplicates preserving order
    seen = []
    filtered_worlds = []
    for cand in world_candidates:
        if cand and cand not in seen:
            seen.append(cand)
            filtered_worlds.append(cand)
    world_candidates = filtered_worlds

    topics_summary = []
    if tf_topics:
        topics_summary.append("/tf")
    if tf_static:
        topics_summary.append("/tf_static")
    result = {
        "tf_topics": topics_summary,
        "world_frame": None,
        "base_frame": None,
        "selected_base": None,
        "transform": None,
        "ok": False,
        "error": None,
        "fallback": "none",
    }

    if not world_pose:
        result["error"] = "no world pose"
        return result

    helper = get_tf_helper()
    if helper is None:
        result["error"] = "TF helper unavailable"
        return result

    world_frame = selection_frame or WORLD_FRAME or "world"
    base_frame = discover_robot_base_frame(world_frame)
    if base_frame:
        world_frame = discover_world_frame(helper, base_frame, selection_frame)
    result["world_frame"] = world_frame
    result["base_frame"] = "base_link"

    def attempt_transform(target_base: str, target_world: str) -> Tuple[Optional[Tuple[float, float, float]], Optional["TransformStamped"], Optional[str]]:
        coords, transform = transform_point_to_frame(world_pose, target_base, source_frame=target_world, timeout_sec=0.6)
        if not coords or not transform:
            return None, None, f"transform {target_world}->{target_base} timed out"
        return coords, transform, None

    coords, transform, error = (None, None, None)
    if base_frame and world_frame:
        coords, transform, error = attempt_transform(base_frame, world_frame)
    if coords and transform:
        result.update(
            {
                "selected_base": coords,
                "transform": transform,
                "ok": True,
            }
        )
        return result

    if error:
        result["error"] = error
    else:
        result["error"] = f"lookup {world_frame}->{result['base_frame']} timed out"

    if env_str("ENABLE_STATIC_TF_FALLBACK", "0") == "1":
        fallback_world = world_frame or WORLD_FRAME or "world"
        fallback_base = "base_link"
        ok, msg = _publish_static_tf(fallback_world, fallback_base)
        result["fallback"] = msg
        if ok:
            time.sleep(0.05)
            helper = get_tf_helper()
            base_frame = fallback_base
            world_frame = fallback_world
            result["world_frame"] = world_frame
            result["base_frame"] = base_frame
            coords, transform, error = attempt_transform(base_frame, world_frame)
            if coords and transform:
                result.update(
                    {
                        "selected_base": coords,
                        "transform": transform,
                        "ok": True,
                        "error": None,
                    }
                )
                return result
        else:
            if not result["error"]:
                result["error"] = "static tf publish failed"

    return result


# ---------------------------------------------------------------------------
# Shared log-string formatters (consolidated from pick_demo / pick_object)
# ---------------------------------------------------------------------------

def fmt_vec3(vec, *, null: str = "(--,--,--)") -> str:
    """Format a 3-element sequence as (x.3f,y.3f,z.3f)."""
    if vec is None:
        return null
    try:
        return f"({float(vec[0]):.3f},{float(vec[1]):.3f},{float(vec[2]):.3f})"
    except Exception:
        return null


def fmt_pose_dict(pose_data, *, null: str = "(--,--,--)") -> str:
    """Format a pose dict {frame, position} as 'frame=X pos=(x,y,z)'."""
    if not isinstance(pose_data, dict):
        return "frame=n/a pos=(--,--,--)"
    frame = str(pose_data.get("frame", "") or "n/a")
    pos = pose_data.get("position", None)
    return f"frame={frame} pos={fmt_vec3(pos, null=null)}"


def fmt_age_sec(value) -> str:
    """Format a numeric value as a seconds string."""
    try:
        return f"{float(value):.3f}s"
    except Exception:
        return "n/a"

def _runtime_time() -> float:
    """Steady local timestamp for runtime freshness and watchdog logic."""
    return time.monotonic()
