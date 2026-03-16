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
import shlex
import time
from pathlib import Path
from typing import TYPE_CHECKING, Dict, List, Optional, Set, Tuple
import math
try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None
try:
    import psutil  # type: ignore
except Exception:  # pragma: no cover
    psutil = None

from .logging_utils import timestamped_line
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

if TYPE_CHECKING:
    from .panel_tf import TfHelper

try:
    # Import correcto cuando ur5_qt_panel es un paquete (colcon/ament)
    from .panel_config import (
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
    # Fallback si alguien ejecuta módulos fuera del contexto de paquete
    from .panel_config import (  # type: ignore
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

_DEBUG_EXCEPTIONS = os.environ.get("PANEL_DEBUG_EXCEPTIONS", "").strip() in ("1", "true", "True")
_GRIPPER_CTRL_CACHE: Tuple[Optional[float], Optional[bool]] = (None, None)
_CM_CACHE: Tuple[float, str] = (0.0, "/controller_manager")
_CM_CACHE_TTL_SEC = 2.0


def _log_exception(context: str, exc: Exception) -> None:
    if not _DEBUG_EXCEPTIONS:
        return
    print(timestamped_line(f"[PANEL_UTILS][WARN] {context}: {exc}"), flush=True)


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
    print(timestamped_line(msg), flush=True)


def base_to_world(x: float, y: float, z: float) -> Tuple[float, float, float]:
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


# ------------------------------------------------------------------
# system stats
# ------------------------------------------------------------------

def read_cpu_times() -> Optional[Tuple[int, int]]:
    try:
        with open("/proc/stat", "r", encoding="utf-8") as f:
            line = f.readline().strip()
        parts = line.split()
        if not parts or parts[0] != "cpu":
            return None
        nums = [int(x) for x in parts[1:]]
        total = sum(nums)
        idle = nums[3] + (nums[4] if len(nums) > 4 else 0)
        return total, idle
    except Exception as exc:
        _log_exception("load_table_calibration", exc)
        return None


def read_meminfo_kb() -> Optional[Tuple[int, int]]:
    try:
        total = None
        avail = None
        with open("/proc/meminfo", "r", encoding="utf-8") as f:
            for line in f:
                key, rest = line.split(":", 1)
                value = int(rest.strip().split()[0])
                if key == "MemTotal":
                    total = value
                elif key == "MemAvailable":
                    avail = value
                if total is not None and avail is not None:
                    break
        if total is None or avail is None:
            return None
        return total, avail
    except Exception as exc:
        _log_exception("load_table_calibration", exc)
        return None


def read_loadavg() -> Optional[Tuple[float, float, float]]:
    try:
        with open("/proc/loadavg", "r", encoding="utf-8") as f:
            parts = f.read().split()
        return float(parts[0]), float(parts[1]), float(parts[2])
    except Exception:
        return None


def kb_to_gb(kb: int) -> float:
    return kb / 1024.0 / 1024.0


# ------------------------------------------------------------------
# table calibration
# ------------------------------------------------------------------

def _pixel_to_norm(px: float, py: float, w: int, h: int) -> Tuple[float, float]:
    nx = (px / float(w)) - 0.5
    ny = (py / float(h)) - 0.5
    if TABLE_IMAGE_SWAP_XY:
        nx, ny = ny, nx
    if TABLE_IMAGE_FLIP_X:
        nx = -nx
    if TABLE_IMAGE_FLIP_Y:
        ny = -ny
    return nx, ny


def _norm_to_pixel(nx: float, ny: float, w: int, h: int) -> Tuple[int, int]:
    if TABLE_IMAGE_FLIP_Y:
        ny = -ny
    if TABLE_IMAGE_FLIP_X:
        nx = -nx
    if TABLE_IMAGE_SWAP_XY:
        nx, ny = ny, nx
    px = (nx + 0.5) * w
    py = (ny + 0.5) * h
    return int(max(0, min(w - 1, px))), int(max(0, min(h - 1, py)))


def pixel_to_norm(px: float, py: float, w: int, h: int) -> Tuple[float, float]:
    return _pixel_to_norm(px, py, w, h)


def norm_to_pixel(nx: float, ny: float, w: int, h: int) -> Tuple[int, int]:
    return _norm_to_pixel(nx, ny, w, h)


def _apply_homography(mat: List[List[float]], u: float, v: float) -> Optional[Tuple[float, float]]:
    denom = (mat[2][0] * u) + (mat[2][1] * v) + mat[2][2]
    if abs(denom) < 1e-8:
        return None
    x = ((mat[0][0] * u) + (mat[0][1] * v) + mat[0][2]) / denom
    y = ((mat[1][0] * u) + (mat[1][1] * v) + mat[1][2]) / denom
    return x, y


def _invert_3x3(mat: List[List[float]]) -> Optional[List[List[float]]]:
    a, b, c = mat[0]
    d, e, f = mat[1]
    g, h, i = mat[2]
    det = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )
    if abs(det) < 1e-10:
        return None
    inv_det = 1.0 / det
    return [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ]


def _solve_linear_system(a: List[List[float]], b: List[float]) -> Optional[List[float]]:
    n = len(b)
    mat = [row[:] + [b[i]] for i, row in enumerate(a)]
    for col in range(n):
        pivot = col
        max_val = abs(mat[col][col])
        for r in range(col + 1, n):
            val = abs(mat[r][col])
            if val > max_val:
                max_val = val
                pivot = r
        if max_val < 1e-10:
            return None
        if pivot != col:
            mat[col], mat[pivot] = mat[pivot], mat[col]
        pivot_val = mat[col][col]
        for c in range(col, n + 1):
            mat[col][c] /= pivot_val
        for r in range(n):
            if r == col:
                continue
            factor = mat[r][col]
            if abs(factor) < 1e-12:
                continue
            for c in range(col, n + 1):
                mat[r][c] -= factor * mat[col][c]
    return [mat[i][n] for i in range(n)]


def compute_homography(
    pixels: List[Tuple[float, float]],
    worlds: List[Tuple[float, float]],
) -> Optional[List[List[float]]]:
    if len(pixels) < 4 or len(worlds) < 4:
        return None
    pts = list(zip(pixels, worlds))[:4]
    a: List[List[float]] = []
    b: List[float] = []
    for (u, v), (x, y) in pts:
        a.append([u, v, 1.0, 0.0, 0.0, 0.0, -x * u, -x * v])
        b.append(x)
        a.append([0.0, 0.0, 0.0, u, v, 1.0, -y * u, -y * v])
        b.append(y)
    sol = _solve_linear_system(a, b)
    if not sol:
        return None
    return [
        [sol[0], sol[1], sol[2]],
        [sol[3], sol[4], sol[5]],
        [sol[6], sol[7], 1.0],
    ]


def _pixel_to_world_at_z(px: float, py: float, w: int, h: int, z_target: float) -> Optional[Tuple[float, float]]:
    cam = TABLE_CAM_INFO
    if not cam:
        return None
    pos = cam.get("position")
    rot = cam.get("rotation")
    fx = cam.get("fx")
    fy = cam.get("fy")
    cx = cam.get("cx")
    cy = cam.get("cy")
    cam_w = cam.get("width")
    cam_h = cam.get("height")
    if (
        not isinstance(pos, list) or len(pos) != 3
        or not isinstance(rot, list) or len(rot) != 3
        or any(not isinstance(row, list) or len(row) != 3 for row in rot)
    ):
        return None
    try:
        fx = float(fx)
        fy = float(fy)
        cx = float(cx)
        cy = float(cy)
    except (TypeError, ValueError):
        return None
    cam_w = int(cam_w) if isinstance(cam_w, (int, float)) else w
    cam_h = int(cam_h) if isinstance(cam_h, (int, float)) else h
    if cam_w > 0 and cam_h > 0 and (cam_w != w or cam_h != h):
        sx = w / float(cam_w)
        sy = h / float(cam_h)
        fx *= sx
        fy *= sy
        cx *= sx
        cy *= sy
    xcam = 1.0
    ycam = -(px - cx) / fx
    zcam = -(py - cy) / fy
    dir_world = (
        rot[0][0] * xcam + rot[0][1] * ycam + rot[0][2] * zcam,
        rot[1][0] * xcam + rot[1][1] * ycam + rot[1][2] * zcam,
        rot[2][0] * xcam + rot[2][1] * ycam + rot[2][2] * zcam,
    )
    try:
        ox, oy, oz = float(pos[0]), float(pos[1]), float(pos[2])
    except (TypeError, ValueError):
        return None
    dz = dir_world[2]
    if abs(dz) < 1e-8:
        return None
    t = (float(z_target) - oz) / dz
    if t <= 0.0:
        return None
    xw = ox + t * dir_world[0]
    yw = oy + t * dir_world[1]
    return xw, yw


def pixel_to_table_xy(px: int, py: int, w: int, h: int, z_target: Optional[float] = None) -> Tuple[float, float]:
    if w <= 0 or h <= 0:
        return 0.0, 0.0
    if z_target is not None:
        out = _pixel_to_world_at_z(float(px), float(py), w, h, float(z_target))
        if out:
            return out
    if TABLE_PIXEL_HOMOGRAPHY:
        nx, ny = _pixel_to_norm(float(px), float(py), w, h)
        out = _apply_homography(TABLE_PIXEL_HOMOGRAPHY, nx, ny)
        if out:
            return out
    if TABLE_PIXEL_AFFINE:
        a, b, c = TABLE_PIXEL_AFFINE[0]
        d, e, f = TABLE_PIXEL_AFFINE[1]
        x = (a * px) + (b * py) + c
        y = (d * px) + (e * py) + f
        return x, y
    if TABLE_PIXEL_RECT:
        px1, py1 = TABLE_PIXEL_RECT["p1"]
        px2, py2 = TABLE_PIXEL_RECT["p2"]
        x1, y1 = TABLE_PIXEL_RECT["w1"]
        x2, y2 = TABLE_PIXEL_RECT["w2"]
        nx, ny = _pixel_to_norm(px, py, w, h)
        npx1, npy1 = _pixel_to_norm(px1, py1, w, h)
        npx2, npy2 = _pixel_to_norm(px2, py2, w, h)
        sx = (x2 - x1) / max(1e-6, (npx2 - npx1))
        sy = (y2 - y1) / max(1e-6, (npy2 - npy1))
        x = x1 + (nx - npx1) * sx
        y = y1 + (ny - npy1) * sy
        return x, y
    nx = (px / float(w)) - 0.5
    ny = (py / float(h)) - 0.5
    if TABLE_IMAGE_SWAP_XY:
        nx, ny = ny, nx
    if TABLE_IMAGE_FLIP_X:
        nx = -nx
    if TABLE_IMAGE_FLIP_Y:
        ny = -ny
    x = TABLE_CENTER_X + nx * TABLE_SIZE_X
    y = TABLE_CENTER_Y + ny * TABLE_SIZE_Y
    return x, y


def world_xyz_to_pixel(x: float, y: float, z: float, w: int, h: int) -> Optional[Tuple[int, int]]:
    out = world_xyz_to_pixel_float(x, y, z, w, h)
    if not out:
        return None
    u, v = out
    return int(max(0, min(w - 1, u))), int(max(0, min(h - 1, v)))


def world_xyz_to_pixel_float(x: float, y: float, z: float, w: int, h: int) -> Optional[Tuple[float, float]]:
    if w <= 0 or h <= 0:
        return None
    cam = TABLE_CAM_INFO
    if not cam:
        return None
    pos = cam.get("position")
    rot = cam.get("rotation")
    fx = cam.get("fx")
    fy = cam.get("fy")
    cx = cam.get("cx")
    cy = cam.get("cy")
    cam_w = cam.get("width")
    cam_h = cam.get("height")
    if (
        not isinstance(pos, list) or len(pos) != 3
        or not isinstance(rot, list) or len(rot) != 3
        or any(not isinstance(row, list) or len(row) != 3 for row in rot)
    ):
        return None
    try:
        fx = float(fx)
        fy = float(fy)
        cx = float(cx)
        cy = float(cy)
    except (TypeError, ValueError):
        return None
    cam_w = int(cam_w) if isinstance(cam_w, (int, float)) else w
    cam_h = int(cam_h) if isinstance(cam_h, (int, float)) else h
    if cam_w > 0 and cam_h > 0 and (cam_w != w or cam_h != h):
        sx = w / float(cam_w)
        sy = h / float(cam_h)
        fx *= sx
        fy *= sy
        cx *= sx
        cy *= sy
    try:
        ox, oy, oz = float(pos[0]), float(pos[1]), float(pos[2])
    except (TypeError, ValueError):
        return None
    vx = x - ox
    vy = y - oy
    vz = z - oz
    xcam = rot[0][0] * vx + rot[1][0] * vy + rot[2][0] * vz
    ycam = rot[0][1] * vx + rot[1][1] * vy + rot[2][1] * vz
    zcam = rot[0][2] * vx + rot[1][2] * vy + rot[2][2] * vz
    if xcam <= 1e-6:
        return None
    u = cx - (ycam / xcam) * fx
    v = cy - (zcam / xcam) * fy
    return max(0.0, min(float(w - 1), float(u))), max(0.0, min(float(h - 1), float(v)))


def table_xy_to_pixel(x: float, y: float, w: int, h: int) -> Optional[Tuple[int, int]]:
    out = table_xy_to_pixel_float(x, y, w, h)
    if not out:
        return None
    px, py = out
    return int(max(0, min(w - 1, px))), int(max(0, min(h - 1, py)))


def table_xy_to_pixel_float(x: float, y: float, w: int, h: int) -> Optional[Tuple[float, float]]:
    if w <= 0 or h <= 0:
        return None
    if TABLE_PIXEL_HOMOGRAPHY:
        inv = _invert_3x3(TABLE_PIXEL_HOMOGRAPHY)
        if not inv:
            return None
        out = _apply_homography(inv, float(x), float(y))
        if out:
            nx, ny = out
            if TABLE_IMAGE_SWAP_XY:
                nx, ny = ny, nx
            px = (nx + 0.5) * w
            py = (ny + 0.5) * h
            return max(0.0, min(float(w - 1), float(px))), max(0.0, min(float(h - 1), float(py)))
        return None
    if TABLE_PIXEL_AFFINE:
        a, b, c = TABLE_PIXEL_AFFINE[0]
        d, e, f = TABLE_PIXEL_AFFINE[1]
        det = (a * e) - (b * d)
        if abs(det) < 1e-8:
            return None
        inv = [[e / det, -b / det], [-d / det, a / det]]
        tx = x - c
        ty = y - f
        px = (inv[0][0] * tx) + (inv[0][1] * ty)
        py = (inv[1][0] * tx) + (inv[1][1] * ty)
        return max(0.0, min(float(w - 1), float(px))), max(0.0, min(float(h - 1), float(py)))
    if TABLE_PIXEL_RECT:
        px1, py1 = TABLE_PIXEL_RECT["p1"]
        px2, py2 = TABLE_PIXEL_RECT["p2"]
        x1, y1 = TABLE_PIXEL_RECT["w1"]
        x2, y2 = TABLE_PIXEL_RECT["w2"]
        npx1, npy1 = _pixel_to_norm(px1, py1, w, h)
        npx2, npy2 = _pixel_to_norm(px2, py2, w, h)
        sx = (npx2 - npx1) / max(1e-6, (x2 - x1))
        sy = (npy2 - npy1) / max(1e-6, (y2 - y1))
        nx = npx1 + (x - x1) * sx
        ny = npy1 + (y - y1) * sy
        if TABLE_IMAGE_SWAP_XY:
            nx, ny = ny, nx
        px = (nx + 0.5) * w
        py = (ny + 0.5) * h
        return max(0.0, min(float(w - 1), float(px))), max(0.0, min(float(h - 1), float(py)))
    nx = (x - TABLE_CENTER_X) / max(1e-6, TABLE_SIZE_X)
    ny = (y - TABLE_CENTER_Y) / max(1e-6, TABLE_SIZE_Y)
    if TABLE_IMAGE_FLIP_Y:
        ny = -ny
    if TABLE_IMAGE_FLIP_X:
        nx = -nx
    if TABLE_IMAGE_SWAP_XY:
        nx, ny = ny, nx
    px = (nx + 0.5) * w
    py = (ny + 0.5) * h
    return max(0.0, min(float(w - 1), float(px))), max(0.0, min(float(h - 1), float(py)))


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


def nearest_table_object(x: float, y: float) -> str:
    best = None
    best_d = 1e9
    items = list(get_object_positions().items())
    for name, (ox, oy, oz) in items:
        if not visible_table_object(name, (ox, oy, oz)):
            continue
        d = (ox - x) ** 2 + (oy - y) ** 2
        if d < best_d:
            best = name
            best_d = d
    if best:
        return best
    # No implicit fallback: caller must treat "no candidate" explicitly.
    return ""


def _parse_pose_json(raw: str) -> List[dict]:
    if not raw:
        return []
    candidates = []
    buf = ""
    for line in raw.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("{") and line.endswith("}"):
            candidates.append(line)
        elif line.startswith("{"):
            buf = line
        elif buf:
            buf += line
            if line.endswith("}"):
                candidates.append(buf)
                buf = ""
    if not candidates and "{" in raw and "}" in raw:
        start = raw.find("{")
        end = raw.rfind("}")
        if start >= 0 and end > start:
            candidates.append(raw[start : end + 1])
    for payload in reversed(candidates):
        try:
            data = json.loads(payload)
        except Exception:
            continue
        poses = []
        if isinstance(data, dict):
            poses = data.get("pose") or data.get("poses") or []
            if not poses and isinstance(data.get("msg"), dict):
                msg = data.get("msg")
                poses = msg.get("pose") or msg.get("poses") or []
        if isinstance(poses, list) and poses:
            return poses
    return []


def get_object_pose_gz(obj_name: str, partition: str = "") -> Optional[Tuple[float, float, float]]:
    """Obtiene la pose (x,y,z) y orientación (quaternion) de un modelo en Gazebo usando /pose/info."""
    if not ROS_AVAILABLE or TFMessage is None:
        return None
    node = _create_graph_node("panel_pose_probe")
    if node is None:
        return None
    target: Dict[str, object] = {}

    def _on_pose(msg: "TFMessage"):
        nonlocal target
        for tf in getattr(msg, "transforms", []):
            name = getattr(tf, "child_frame_id", "") or ""
            if name == obj_name or name.startswith(f"{obj_name}::"):
                t = tf.transform.translation
                r = tf.transform.rotation
                target = {
                    "position": {"x": float(t.x), "y": float(t.y), "z": float(t.z)},
                    "orientation": {"x": float(r.x), "y": float(r.y), "z": float(r.z), "w": float(r.w)},
                }
                break

    topic = f"/world/{GZ_WORLD}/pose/info"
    try:
        sub = node.create_subscription(TFMessage, topic, _on_pose, qos_profile_sensor_data)
    except Exception as exc:
        _log_exception("create pose subscription", exc)
        try:
            node.destroy_node()
        except Exception as exc_destroy:
            _log_exception("destroy probe node", exc_destroy)
        return None
    end = time.time() + 0.6
    while time.time() < end and not target:
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        node.destroy_subscription(sub)
    except Exception as exc:
        _log_exception("destroy pose subscription", exc)
    try:
        node.destroy_node()
    except Exception as exc:
        _log_exception("destroy probe node", exc)
    return target or None




def load_home_pose() -> List[float]:
    vals = list(UR5_HOME_DEFAULT)
    if os.path.isfile(UR5_HOME_ENV):
        try:
            with open(UR5_HOME_ENV, "r", encoding="utf-8") as f:
                for line in f:
                    if not line.startswith("HOME_POS_"):
                        continue
                    key, _, raw = line.partition("=")
                    raw = raw.strip()
                    if not raw:
                        continue
                    try:
                        idx = int(key.split("_")[-1])
                    except ValueError:
                        continue
                    if 0 <= idx < len(vals):
                        try:
                            vals[idx] = float(raw)
                        except ValueError:
                            continue
        except Exception as exc:
            _log_exception("load_home_pose", exc)
            vals = list(UR5_HOME_DEFAULT)
    if all(abs(v) < 1e-3 for v in vals):
        return list(UR5_HOME_DEFAULT)
    return vals

def save_home_pose(joint_values: List[float]) -> None:
    """Guarda la pose HOME actual en el archivo ur5_home_pose.env."""
    from .panel_config import UR5_HOME_ENV
    try:
        with open(UR5_HOME_ENV, "w", encoding="utf-8") as f:
            f.write("# Pose HOME del UR5, generada desde el panel\n")
            for idx, val in enumerate(joint_values):
                f.write(f"HOME_POS_{idx}={val}\n")
    except Exception as e:
        print(timestamped_line(f"[ERROR] No se pudo guardar la pose HOME: {e}"), flush=True)


def object_out_of_reach(x: float, y: float) -> bool:
    dx = x - UR5_BASE_X
    dy = y - UR5_BASE_Y
    return (dx * dx + dy * dy) > (UR5_REACH_RADIUS * UR5_REACH_RADIUS)


def world_to_base(x: float, y: float, z: float) -> Tuple[float, float, float]:
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


def visible_table_object(name: str, position: Tuple[float, float, float]) -> bool:
    """Return True if the object is a table-relevant piece."""
    x, y, z = position
    if TABLE_OBJECT_WHITELIST and name not in TABLE_OBJECT_WHITELIST:
        return False
    half_x = TABLE_SIZE_X / 2.0 + TABLE_OBJECT_XY_MARGIN
    half_y = TABLE_SIZE_Y / 2.0 + TABLE_OBJECT_XY_MARGIN
    dx = x - TABLE_CENTER_X
    dy = y - TABLE_CENTER_Y
    if abs(dx) > half_x or abs(dy) > half_y:
        return False
    if z < TABLE_OBJECT_Z_MIN or z > TABLE_OBJECT_Z_MAX:
        return False
    blacklisted = (
        "upper_arm_link",
        "forearm_link",
        "wrist_",
        "rg2_",
        "tool0",
        "link_",
        "p1v",
        "p2v",
        "p3v",
        "p4v",
    )
    if any(name.startswith(token) for token in blacklisted):
        return False
    return True


def gripper_controller_defined() -> bool:
    global _GRIPPER_CTRL_CACHE
    try:
        if not UR5_CONTROLLERS_YAML:
            return False
        try:
            mtime = os.path.getmtime(UR5_CONTROLLERS_YAML)
        except FileNotFoundError:
            mtime = None
        cached_mtime, cached_value = _GRIPPER_CTRL_CACHE
        if cached_mtime == mtime and cached_value is not None:
            return cached_value
        if mtime is None:
            _GRIPPER_CTRL_CACHE = (mtime, False)
            return False
        with open(UR5_CONTROLLERS_YAML, "r", encoding="utf-8", errors="ignore") as f:
            value = "gripper_controller:" in f.read()
        _GRIPPER_CTRL_CACHE = (mtime, value)
        return value
    except Exception as exc:
        _log_exception("gripper_controller_defined", exc)
        return False


def _discover_controller_manager(node) -> str:
    try:
        services = node.get_service_names_and_types()
    except Exception as exc:
        _log_exception("discover controller_manager", exc)
        return ""
    for name, _types in services:
        if name.endswith("/controller_manager/list_controllers"):
            return name.rsplit("/list_controllers", 1)[0]
    return ""


def _controller_manager_path(preferred: str = "") -> str:
    if preferred:
        return preferred
    env_path = os.environ.get("PANEL_CONTROLLER_MANAGER", "").strip()
    if env_path:
        return env_path
    return "/controller_manager"


def resolve_controller_manager(node=None, preferred: str = "") -> str:
    """Return controller_manager namespace, discovering when possible."""
    global _CM_CACHE
    cm_path = _controller_manager_path(preferred)
    if cm_path != "/controller_manager":
        return cm_path
    now = time.monotonic()
    if (now - _CM_CACHE[0]) <= _CM_CACHE_TTL_SEC:
        return _CM_CACHE[1]
    created = False
    if node is None:
        node = _create_graph_node("panel_cm_discover")
        created = node is not None
    if node is None:
        return cm_path
    try:
        discovered = _discover_controller_manager(node)
        resolved = discovered or cm_path
        _CM_CACHE = (now, resolved)
        return resolved
    finally:
        if created:
            try:
                node.destroy_node()
            except Exception as exc:
                _log_exception("destroy cm discover node", exc)


def list_controllers_state(
    controller_manager: str = "",
) -> Tuple[Optional[Dict[str, str]], Optional[str]]:
    if not ROS_AVAILABLE or ListControllers is None:
        return None, "ROS no disponible"
    node = _create_graph_node("panel_ctrl_probe")
    if node is None:
        return None, "node unavailable"
    cm_path = resolve_controller_manager(node, preferred=controller_manager)
    client = node.create_client(ListControllers, f"{cm_path}/list_controllers")
    if not client.wait_for_service(timeout_sec=0.5):
        node.destroy_node()
        return None, "service unavailable"
    future = client.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=0.6)
    if not future.done():
        node.destroy_node()
        return None, "timeout"
    result = future.result()
    states: Dict[str, str] = {}
    if result and hasattr(result, "controller"):
        for ctrl in result.controller:
            states[str(ctrl.name)] = str(ctrl.state)
    node.destroy_node()
    return states, None


def list_active_controllers(
    controller_manager: str = "",
) -> Tuple[Optional[Set[str]], Optional[str]]:
    states, err = list_controllers_state(controller_manager=controller_manager)
    if err or states is None:
        return None, err
    active = {name for name, st in states.items() if st == "active"}
    return active, None


def ros2_control_running(controller_manager: str = "") -> bool:
    node = _create_graph_node("panel_ctrl_check")
    if node is None:
        return False
    try:
        services = node.get_service_names_and_types()
        if controller_manager:
            target = f"{controller_manager}/list_controllers"
            return any(name == target for name, _ in services)
        return any(name.endswith("/controller_manager/list_controllers") for name, _ in services)
    except Exception as exc:
        _log_exception("ros2_control_running", exc)
        return False
    finally:
        try:
            node.destroy_node()
        except Exception as exc:
            _log_exception("destroy ctrl check node", exc)


def _run_cmd_rc(cmd: str, timeout_sec: float = 2.0) -> Tuple[int, str]:
    res = run_cmd(cmd, timeout=timeout_sec, capture_output=True)
    return res.returncode, res.stdout or res.stderr or ""


def gz_sim_status() -> Tuple[bool, str]:
    global _GZ_SIM_STATUS_CACHE
    now = time.monotonic()
    cached_ts, cached_ok, cached_reason = _GZ_SIM_STATUS_CACHE
    if (now - cached_ts) < _GZ_SIM_STATUS_TTL_SEC:
        return cached_ok, cached_reason
    if psutil is not None:
        try:
            for proc in psutil.process_iter(attrs=["cmdline", "status"]):
                info = proc.info
                if info.get("status") == psutil.STATUS_ZOMBIE:
                    continue
                cmdline = info.get("cmdline") or []
                if not cmdline:
                    continue
                joined = " ".join(cmdline).lower()
                if "gz sim" in joined:
                    _GZ_SIM_STATUS_CACHE = (now, True, "proc")
                    return True, "proc"
        except Exception as exc:
            _log_exception("gz_sim_status psutil", exc)
    res = run_cmd("pgrep -af 'gz sim' || true", timeout=1.2, capture_output=True)
    lines = [line.strip() for line in (res.stdout or "").splitlines() if line.strip()]
    for line in lines:
        pid = line.split(maxsplit=1)[0]
        stat = run_cmd(f"ps -o stat= -p {shlex.quote(pid)}", timeout=0.8, capture_output=True)
        state = (stat.stdout or "").strip()
        if state and "Z" not in state:
            _GZ_SIM_STATUS_CACHE = (now, True, "proc")
            return True, "proc"
    _GZ_SIM_STATUS_CACHE = (now, False, "none")
    return False, "none"


def gz_sim_running() -> bool:
    ok, _reason = gz_sim_status()
    return ok


def bridge_status() -> Tuple[bool, str]:
    node = _create_graph_node("panel_bridge_check")
    if node is None:
        return False, "node"
    try:
        names = node.get_node_names_and_namespaces()
        for name, _ns in names:
            if name in ("parameter_bridge", "ros_gz_bridge"):
                return True, "rosnode"
        return False, "none"
    except Exception as exc:
        _log_exception("bridge_status", exc)
        return False, "err"
    finally:
        try:
            node.destroy_node()
        except Exception as exc:
            _log_exception("destroy bridge check node", exc)


def clock_status() -> Tuple[bool, str]:
    node = _create_graph_node("panel_clock_check")
    if node is None:
        return False, "node"
    try:
        topics = node.get_topic_names_and_types()
        has_clock = any(name == "/clock" for name, _ in topics)
        if not has_clock:
            return False, "none"
        pubs = node.get_publishers_info_by_topic("/clock")
        if pubs:
            return True, "publisher"
        return True, "topic"
    except Exception as exc:
        _log_exception("clock_status", exc)
        return False, "err"
    finally:
        try:
            node.destroy_node()
        except Exception as exc:
            _log_exception("destroy clock check node", exc)


def _create_graph_node(name: str):
    if not ROS_AVAILABLE:
        return None
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
    except Exception as exc:
        _log_exception("rclpy.init", exc)
        return None
    try:
        return rclpy.create_node(name)
    except Exception as exc:
        _log_exception("create node", exc)
        return None


def ros_clock_available() -> bool:
    ok, _reason = clock_status()
    return ok


def robot_control_available() -> bool:
    cm_path = resolve_controller_manager()
    return ros2_control_running(cm_path) or (gz_sim_running() and ros_clock_available())


def detect_arm_trajectory_topic() -> str:
    force_ros2 = os.environ.get("FORCE_ROS2_CONTROL", "0") == "1"
    if not force_ros2 and gz_sim_running():
        return ARM_TRAJ_TOPIC_DEFAULT
    cm_path = resolve_controller_manager()
    active, err = list_active_controllers(controller_manager=cm_path)
    if not err and active and "joint_trajectory_controller" in active:
        return "/joint_trajectory_controller/joint_trajectory"
    return ARM_TRAJ_TOPIC_DEFAULT


def parse_ros_topics(raw: str) -> Tuple[List[str], List[str]]:
    items = [t.strip() for t in raw.split() if t.strip()]
    if not items:
        return [], []
    invalid = [t for t in items if not ROS_TOPIC_RE.match(t)]
    valid = [t for t in items if t not in invalid]
    return valid, invalid


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


def debug_dump_tf(
    target_frame: str, source_frame: str = "world", timeout_sec: float = 1.0
) -> Tuple[Optional[Dict[str, object]], Optional[str]]:
    """Lookup and describe TF from *source_frame* to *target_frame*."""
    helper = get_tf_helper()
    if not helper:
        return None, "TF helper unavailable"
    transform = helper.lookup_transform(target_frame, source_frame, timeout_sec=timeout_sec)
    if not transform:
        return None, f"lookup {source_frame}->{target_frame} timed out"
    t = transform.transform.translation
    yaw = yaw_from_quaternion(transform.transform.rotation)
    return (
        {
            "translation": (t.x, t.y, t.z),
            "yaw": yaw,
            "frame_rel": source_frame,
        },
        None,
    )


_TF_FRAMES_LOGGED = False
_TF_FRAME_SUMMARY_LOGGED = False
_TF_YAML_HEAD_LOGGED = False
_EE_UNAVAILABLE_LOGGED = False
_TF_YAML_HEAD_LOGGED = False
EE_FRAME_CANDIDATE_BASES = (
    "tool0",
    "rg2_tcp",
    "tcp",
    "ee_link",
    "gripper_link",
    "wrist_3_link",
    "flange",
    "ee",
    "tool_link",
    "end_effector_link",
    "rg2_hand",
    "ft_frame",
)

ROBOT_FRAME_KEYWORDS = ("wrist", "shoulder", "elbow", "tool", "tcp", "ee", "flange", "rg2", "hand")
EE_FRAME_SUBSTRING_KEYWORDS = ("tool", "tcp", "ee", "gripper", "flange", "wrist", "rg2", "hand", "ft")

_BASE_FRAME_CACHE: Optional[str] = None
_LAST_TRACE_FRAME_LOG: Optional[str] = None


def _parse_tf_yaml_records(yaml_text: str) -> List[Dict[str, object]]:
    if not yaml_text:
        return []
    parsed = None
    if yaml is not None:
        try:
            parsed = yaml.safe_load(yaml_text)
        except Exception:
            parsed = None
    records: List[Dict[str, object]] = []
    if isinstance(parsed, dict):
        frames = parsed.get("frames")
        if isinstance(frames, list):
            for entry in frames:
                if isinstance(entry, dict):
                    records.append(entry)
    if not records:
        for match in _FRAME_ID_PATTERN.finditer(yaml_text):
            records.append({"frame_id": match.group(1)})
    return records


_FRAME_ID_PATTERN = re.compile(r'frame_id:\s*"([^"]+)"')
_CHILD_FRAME_ID_PATTERN = re.compile(r'child_frame_id:\s*"([^"]+)"')
_FRAME_LINE_PATTERN = re.compile(r'Frame\s+["\']?([A-Za-z0-9_/:\.\-]+)["\']?', re.IGNORECASE)
_FRAME_KEY_PATTERN = re.compile(r'^\s*([A-Za-z0-9_/:\.\-]+)\s*:\s*(?:\{|\[|$)', re.MULTILINE)
_FRAME_LIST_ITEM_PATTERN = re.compile(r'^\s*-\s*([A-Za-z0-9_/:\.\-]+)\s*$', re.MULTILINE)
_FRAME_LIST_FRAME_PATTERN = re.compile(r'^\s*-\s*frame\s*:\s*([A-Za-z0-9_/:\.\-]+)\s*$', re.IGNORECASE | re.MULTILINE)


def _extract_frames_from_yaml(yaml_text: str) -> Set[str]:
    """Return frame names extracted from the TF YAML dump."""
    if not yaml_text:
        return set()
    found: Set[str] = set()
    for pattern in (
        _FRAME_ID_PATTERN,
        _CHILD_FRAME_ID_PATTERN,
        _FRAME_LINE_PATTERN,
        _FRAME_KEY_PATTERN,
        _FRAME_LIST_ITEM_PATTERN,
        _FRAME_LIST_FRAME_PATTERN,
    ):
        for match in pattern.finditer(yaml_text):
            name = match.group(1)
            if name:
                found.add(name)
    cleaned: Set[str] = set()
    ignore = {"frames", "transforms", "child_frames", "header", "data", "frame"}
    for frame in found:
        if frame.lower() in ignore:
            continue
        cleaned.add(frame)
    return cleaned


def _load_tf_frame_records(source) -> List[Dict[str, object]]:
    if source is None:
        return []
    if isinstance(source, str):
        return _parse_tf_yaml_records(source)
    try:
        yaml_text = source.all_frames_as_yaml()
    except Exception:
        return []
    return _parse_tf_yaml_records(yaml_text)


def _build_frame_graph(records: List[Dict[str, object]]) -> Tuple[Set[str], Dict[str, List[str]], Dict[str, str]]:
    frames: Set[str] = set()
    children: Dict[str, List[str]] = {}
    parent_map: Dict[str, str] = {}
    for entry in records:
        fid = entry.get("frame_id")
        if not fid:
            continue
        frames.add(fid)
        parent = entry.get("parent_frame_id")
        if parent and parent != fid:
            parent_map[fid] = parent
            children.setdefault(parent, []).append(fid)
        for child in entry.get("child_frames") or []:
            if isinstance(child, dict):
                child_id = child.get("frame_id")
                if child_id and child_id != fid:
                    frames.add(child_id)
                    parent_map[child_id] = fid
                    children.setdefault(fid, []).append(child_id)
    return frames, children, parent_map


def _collect_leaf_frames(frames: Set[str], children: Dict[str, List[str]]) -> List[str]:
    if not frames:
        return []
    parents = set(children.keys())
    return [frame for frame in frames if frame not in parents]


def _frame_depth(
    frame: str,
    parent_map: Dict[str, str],
    cache: Dict[str, int],
    visiting: Optional[Set[str]] = None,
) -> int:
    if frame in cache:
        return cache[frame]
    if visiting is None:
        visiting = set()
    if frame in visiting:
        cache[frame] = 0
        return 0
    visiting.add(frame)
    parent = parent_map.get(frame)
    if not parent or parent == frame:
        depth = 0
    else:
        depth = 1 + _frame_depth(parent, parent_map, cache, visiting)
    visiting.remove(frame)
    cache[frame] = depth
    return depth


def _log_tf_frames_once(frames: Set[str]) -> None:
    global _TF_FRAMES_LOGGED
    if _TF_FRAMES_LOGGED or not frames:
        return
    sample = ", ".join(sorted(frames)[:80])
    print(
        timestamped_line(f"[TRACE] Available TF frames ({len(frames)}): {sample}"),
        flush=True,
    )
    _TF_FRAMES_LOGGED = True

def _log_tf_frame_summary(all_frames: Set[str], robot_keyword_frames: List[str]) -> None:
    global _TF_FRAME_SUMMARY_LOGGED
    if _TF_FRAME_SUMMARY_LOGGED or not all_frames:
        return
    sample = ", ".join(sorted(all_frames)[:80])
    print(
        timestamped_line(
            f"[TRACE] TF summary frames={len(all_frames)} robot_candidates={len(robot_keyword_frames)} sample={sample}"
        ),
        flush=True,
    )
    _TF_FRAME_SUMMARY_LOGGED = True


def _log_tf_yaml_head_once(yaml_text: str) -> None:
    global _TF_YAML_HEAD_LOGGED
    if _TF_YAML_HEAD_LOGGED or not yaml_text:
        return
    lines = yaml_text.strip().splitlines()
    head = "\n".join(lines[:20])
    print(
        timestamped_line(f"[TRACE][DIAG] TF YAML head:\n{head}"),
        flush=True,
    )
    _TF_YAML_HEAD_LOGGED = True


def _log_ee_unavailable_once() -> None:
    global _EE_UNAVAILABLE_LOGGED
    if _EE_UNAVAILABLE_LOGGED:
        return
    print(timestamped_line("[TRACE] EE unavailable (no valid EE frame)"), flush=True)
    _EE_UNAVAILABLE_LOGGED = True


def _can_transform_between(helper: TfHelper, frame_a: str, frame_b: str, timeout_sec: float) -> bool:
    if not frame_a or not frame_b:
        return False
    if helper.can_transform(frame_a, frame_b, timeout_sec=timeout_sec):
        return True
    if helper.can_transform(frame_b, frame_a, timeout_sec=timeout_sec):
        return True
    return False


def _preferred_base_frame(helper: Optional[TfHelper], world_frame: str, timeout_sec: float = 0.2) -> Optional[str]:
    """Return the first base candidate that transforms to the world frame."""
    if helper is None:
        return None
    candidate = "base_link"
    if _can_transform_between(helper, candidate, world_frame, timeout_sec=timeout_sec):
        return candidate
    return None


def discover_robot_base_frame(world_frame: Optional[str] = None, timeout_sec: float = 0.2) -> Optional[str]:
    """Detect the most likely robot base frame given the TF tree."""
    helper = get_tf_helper()
    if helper is None:
        return None
    base_frame, _ = discover_base_and_ee_frames(world_frame, timeout_sec)
    return base_frame


def discover_world_frame(helper: TfHelper, base_frame: str, selection_frame: Optional[str] = None, timeout_sec: float = 0.2) -> Optional[str]:
    """Return the first world frame candidate that transforms to *base_frame*."""
    if helper is None or not base_frame:
        return selection_frame or WORLD_FRAME or "world"
    frames = helper.list_frames()
    candidates: List[str] = []
    if selection_frame:
        candidates.append(selection_frame)
    for candidate in (WORLD_FRAME, *WORLD_FRAME_CANDIDATES, "gz_world", "map", "odom"):
        if not candidate:
            continue
        if candidate not in candidates:
            candidates.append(candidate)
    for candidate in candidates:
        if candidate not in frames:
            continue
        if _can_transform_between(helper, candidate, base_frame, timeout_sec):
            return candidate
    return selection_frame or WORLD_FRAME or "world"


def discover_base_and_ee_frames(world_frame: Optional[str] = None, timeout_sec: float = 0.1) -> Tuple[Optional[str], Optional[str]]:
    """Return effective base and EE frames based on live TF contents."""
    global _BASE_FRAME_CACHE, _LAST_TRACE_FRAME_LOG, _EE_UNAVAILABLE_LOGGED
    helper = get_tf_helper()
    if helper is None:
        return None, None
    frames = helper.list_frames()
    if not frames:
        return None, None
    records = _load_tf_frame_records(helper.frames_yaml())
    graph_frames, children, parent_map = _build_frame_graph(records)
    all_frames = frames.union(graph_frames)
    if not all_frames:
        return None, None
    target_world = world_frame or WORLD_FRAME or "world"
    sorted_frames = sorted(all_frames)
    robot_keyword_frames = [
        frame for frame in sorted_frames if any(keyword in frame.lower() for keyword in ROBOT_FRAME_KEYWORDS)
    ]
    _log_tf_frame_summary(all_frames, robot_keyword_frames)
    base_frame: Optional[str] = None
    if "base_link" in all_frames and _can_transform_between(helper, "base_link", target_world, timeout_sec):
        base_frame = "base_link"
    fallback_base = "base_link"
    ee_frame = _select_ee_frame(helper, all_frames, children, parent_map, fallback_base, timeout_sec)
    effective_base = base_frame or fallback_base
    disallowed_ee = {effective_base}
    if target_world:
        disallowed_ee.add(target_world)
    if ee_frame and ee_frame in disallowed_ee:
        ee_frame = None
        _log_ee_unavailable_once()
    elif ee_frame:
        _EE_UNAVAILABLE_LOGGED = False
    log_msg = f"[TRACE] Using BASE_FRAME_EFFECTIVE={effective_base or 'n/a'} EE_FRAME_EFFECTIVE={ee_frame or 'n/a'}"
    if log_msg != _LAST_TRACE_FRAME_LOG:
        print(timestamped_line(log_msg), flush=True)
        _LAST_TRACE_FRAME_LOG = log_msg
    return effective_base, ee_frame


def _select_base_frame(
    helper: TfHelper,
    frames: Set[str],
    children: Dict[str, List[str]],
    parent_map: Dict[str, str],
    world_frame: str,
    robot_keyword_frames: List[str],
    timeout_sec: float,
) -> Optional[str]:
    global _BASE_FRAME_CACHE
    candidate_order: List[str] = []
    seen: Set[str] = set()

    def add_candidate(name: Optional[str]) -> None:
        if not name or name in seen:
            return
        seen.add(name)
        candidate_order.append(name)

    add_candidate(BASE_FRAME)
    for candidate in BASE_FRAME_CANDIDATES:
        add_candidate(candidate)
    add_candidate(_BASE_FRAME_CACHE)

    robot_frames = list(robot_keyword_frames)
    if not robot_frames:
        leaves = _collect_leaf_frames(frames, children)
        fallback_frames = leaves or sorted(frames)
        depth_cache: Dict[str, int] = {}
        robot_frames = sorted(
            fallback_frames,
            key=lambda f: _frame_depth(f, parent_map, depth_cache),
            reverse=True,
        )

    base_world_only: Optional[str] = None
    base_link_candidate: Optional[str] = None
    for candidate in candidate_order:
        if candidate not in frames:
            continue
        if not _can_transform_between(helper, candidate, world_frame, timeout_sec):
            continue
        if candidate == "base_link":
            base_link_candidate = candidate
        robot_connected = any(
            _can_transform_between(helper, candidate, robot_frame, timeout_sec)
            for robot_frame in robot_frames
        )
        if robot_connected:
            _BASE_FRAME_CACHE = candidate
            return candidate
        if candidate == "base":
            base_world_only = base_world_only or candidate

    if base_link_candidate:
        _BASE_FRAME_CACHE = base_link_candidate
        return base_link_candidate
    if base_world_only:
        _BASE_FRAME_CACHE = base_world_only
        return base_world_only
    return None


def _select_ee_frame(
    helper: TfHelper,
    frames: Set[str],
    children: Dict[str, List[str]],
    parent_map: Dict[str, str],
    base_frame: str,
    timeout_sec: float,
) -> Optional[str]:
    if not base_frame:
        return None
    seen: Set[str] = set()
    candidates: List[str] = []

    def add_candidate(name: str) -> None:
        if not name or name in seen or name not in frames:
            return
        seen.add(name)
        candidates.append(name)

    for preferred in EE_FRAME_PREFERENCE:
        add_candidate(preferred)

    # Align with tf_probe keepers priority.
    for keeper in ("rg2_tcp", "tool0", "tcp", "ee_link", "flange", "wrist_3_link", "ft_frame", "rg2_hand"):
        add_candidate(keeper)

    prefixes = set()
    for frame in frames:
        if "/" in frame:
            prefixes.add(frame.split("/", 1)[0])
        if "_" in frame:
            prefixes.add(frame.split("_", 1)[0])
    prefixes.discard("")

    for prefix in sorted(prefixes):
        for name in EE_FRAME_CANDIDATE_BASES:
            add_candidate(f"{prefix}/{name}")
            add_candidate(f"{prefix}_{name}")

    for name in EE_FRAME_CANDIDATE_BASES:
        add_candidate(name)

    for frame in sorted(frames):
        if any(keyword in frame.lower() for keyword in EE_FRAME_SUBSTRING_KEYWORDS):
            add_candidate(frame)

    for candidate in candidates:
        if _can_transform_between(helper, base_frame, candidate, timeout_sec):
            return candidate

    leaves = _collect_leaf_frames(frames, children) or list(frames)
    preferred = [
        leaf
        for leaf in leaves
        if any(keyword in leaf.lower() for keyword in EE_FRAME_SUBSTRING_KEYWORDS)
    ]
    leaf_candidates = preferred or leaves
    depth_cache: Dict[str, int] = {}
    ordered_leaves = sorted(
        leaf_candidates,
        key=lambda f: _frame_depth(f, parent_map, depth_cache),
        reverse=True,
    )
    for leaf in ordered_leaves:
        if _can_transform_between(helper, base_frame, leaf, timeout_sec):
            return leaf
    return None


def get_pose(
    target_frame: str,
    source_frame: str,
    timeout_sec: float = 0.8,
) -> Tuple[Optional[Dict[str, object]], Optional[str]]:
    helper = get_tf_helper()
    if helper is None:
        return None, "TF helper unavailable"
    transform = helper.lookup_transform(target_frame, source_frame, timeout_sec=timeout_sec)
    if not transform:
        return None, f"lookup {source_frame}->{target_frame} timed out"
    stamp_ns = 0
    try:
        stamp = transform.header.stamp
        stamp_ns = (int(getattr(stamp, "sec", 0)) * 1_000_000_000) + int(
            getattr(stamp, "nanosec", 0)
        )
    except Exception:
        stamp_ns = 0
    pose = {
        "frame": target_frame,
        "position": (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ),
        "orientation": (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ),
        "stamp_ns": int(stamp_ns),
    }
    return pose, None


def _pose_from_transform(transform: "TransformStamped", frame_id: str) -> Optional["PoseStamped"]:
    if not transform or PoseStamped is None:
        return None
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = transform.header.stamp
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z
    pose.pose.orientation = transform.transform.rotation
    return pose


def lookup_pose(
    frame_from: str,
    frame_to: str,
    timeout_sec: float = 0.8,
) -> Tuple[Optional["PoseStamped"], Optional[str]]:
    """Return PoseStamped of *frame_from* expressed in *frame_to*."""
    helper = get_tf_helper()
    if helper is None:
        return None, "TF helper unavailable"
    transform = helper.lookup_transform(frame_to, frame_from, timeout_sec=timeout_sec)
    if not transform:
        return None, f"lookup {frame_from}->{frame_to} timed out"
    pose = _pose_from_transform(transform, frame_to)
    if not pose:
        return None, "pose helper unavailable"
    return pose, None


def transform_pose(
    pose: Optional["PoseStamped"],
    target_frame: str,
    timeout_sec: float = 0.8,
) -> Optional["PoseStamped"]:
    helper = get_tf_helper()
    if helper is None or pose is None:
        return None
    return helper.transform_pose(pose, target_frame, timeout_sec)


BASE_FRAME_CANDIDATES = [
    "base_link",
]
EE_FRAME_PREFERENCE = [
    "rg2_tcp",
    "tool0",
    "tcp",
    "ee_link",
    "ee",
    "tool_link",
    "end_effector_link",
    "flange",
    "wrist_3_link",
    "rg2_hand",
    "ft_frame",
]
WORLD_FRAME_CANDIDATES = ["world", "map", "odom"]


def effective_base_frame(panel, *, default: str = "base_link") -> str:
    """Resolve the effective base frame for a panel instance."""
    candidate = str(getattr(panel, "_base_frame_effective", "") or "").strip()
    if candidate and candidate != "base_link":
        return "base_link"
    setting = str(BASE_FRAME or "").strip()
    if setting and setting != "base_link":
        return "base_link"
    return "base_link"


def effective_world_frame(panel, *, default: str = "world") -> str:
    """Resolve the effective world frame for a panel instance."""
    last = getattr(panel, "_world_frame_last_first", None)
    if callable(last):
        candidate = last()
    else:
        candidate = None
    return candidate or WORLD_FRAME or default


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
    global _BASE_FRAME_CACHE
    scan = ["base_link"]
    for frame in scan:
        if not frame:
            continue
        transform = helper.lookup_transform(frame, source_frame, timeout_sec=timeout_sec)
        if transform:
            _BASE_FRAME_CACHE = frame
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
    if tf2_geometry_msgs is None:
        try:
            transform = helper.lookup_transform(target_frame, source_frame, timeout_sec=timeout_sec)
            if not transform:
                return None, None
            coords = _apply_transform_to_tuple(world_pos, transform)
            return coords, transform
        except Exception as exc:
            _log_tf_transform_warning("transform_point_to_frame", exc)
            return None, None
    try:
        pose = PoseStamped()
        pose.header.frame_id = source_frame or ""
        if BuiltinTime is not None:
            pose.header.stamp = BuiltinTime(sec=0, nanosec=0)
        else:
            pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = world_pos
        pose.pose.orientation.w = 1.0
        transformed = helper.transform_pose(pose, target_frame, timeout_sec)
        if not transformed:
            transform = helper.lookup_transform(target_frame, source_frame, timeout_sec=timeout_sec)
            if not transform:
                return None, None
            coords = _apply_transform_to_tuple(world_pos, transform)
            return coords, transform
        coords = (
            transformed.pose.position.x,
            transformed.pose.position.y,
            transformed.pose.position.z,
        )
        transform = helper.lookup_transform(target_frame, source_frame, timeout_sec=timeout_sec)
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
    env = os.environ.get("PANEL_STATIC_TF", "").strip()
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

    if os.environ.get("ENABLE_STATIC_TF_FALLBACK", "0") == "1":
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
