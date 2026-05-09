#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_table_objects.py
# Contenido: Helpers de objetos de mesa + home pose + Gazebo query extraidos de panel_utils.
"""Helpers de objetos en mesa, home pose y Gazebo pose query (refactor B.6).

Extraido de ``panel_utils.py`` (lineas 461-610 + 647-674 originales). Las
funciones siguientes operan sobre objetos visibles en la mesa, sus poses
en Gazebo y los presets de home pose del UR5.

``load_table_calib`` se queda en panel_utils porque muta el global
TABLE_CAM_INFO de panel_config y mover esa mutacion rompe la importacion
en cascada (panel_config se importa en muchos sitios).

Reexportado desde panel_utils para mantener compatibilidad.

Funciones publicas:
- nearest_table_object
- get_object_pose_gz
- load_home_pose, save_home_pose
- object_out_of_reach
- visible_table_object

Helper privado:
- _parse_pose_json
"""

from __future__ import annotations

import json
import os
import time
from typing import Dict, List, Optional, Tuple

try:
    from tf2_msgs.msg import TFMessage
except Exception:  # pragma: no cover
    TFMessage = None  # type: ignore

try:
    from .panel_config import (
        GZ_WORLD,
        ROS_AVAILABLE,
        TABLE_CENTER_X,
        TABLE_CENTER_Y,
        TABLE_OBJECT_WHITELIST,
        TABLE_OBJECT_XY_MARGIN,
        TABLE_OBJECT_Z_MAX,
        TABLE_OBJECT_Z_MIN,
        TABLE_SIZE_X,
        TABLE_SIZE_Y,
        UR5_BASE_X,
        UR5_BASE_Y,
        UR5_HOME_DEFAULT,
        UR5_HOME_ENV,
        UR5_REACH_RADIUS,
        qos_profile_sensor_data,
        rclpy,
    )
except Exception:  # pragma: no cover
    from .panel_config import (  # type: ignore
        GZ_WORLD,
        ROS_AVAILABLE,
        TABLE_CENTER_X,
        TABLE_CENTER_Y,
        TABLE_OBJECT_WHITELIST,
        TABLE_OBJECT_XY_MARGIN,
        TABLE_OBJECT_Z_MAX,
        TABLE_OBJECT_Z_MIN,
        TABLE_SIZE_X,
        TABLE_SIZE_Y,
        UR5_BASE_X,
        UR5_BASE_Y,
        UR5_HOME_DEFAULT,
        UR5_HOME_ENV,
        UR5_REACH_RADIUS,
        qos_profile_sensor_data,
        rclpy,
    )

from .panel_ui_params import get_panel_ui_params as _get_panel_ui_params

from .logging_utils import emit_log_line, timestamped_line
from .panel_objects import get_object_positions
from .panel_system_status import _create_graph_node


def _log_exception(context: str, exc: Exception) -> None:
    """Inline simplificado para evitar circular import."""

    debug_enabled = _get_panel_ui_params().debug_exceptions
    if debug_enabled:
        import traceback
        traceback.print_exc()


# Las funciones extraidas se apenden aqui via sed.
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
        emit_log_line(timestamped_line(f"[ERROR] No se pudo guardar la pose HOME: {e}"))


def object_out_of_reach(x: float, y: float) -> bool:
    dx = x - UR5_BASE_X
    dy = y - UR5_BASE_Y
    return (dx * dx + dy * dy) > (UR5_REACH_RADIUS * UR5_REACH_RADIUS)


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
