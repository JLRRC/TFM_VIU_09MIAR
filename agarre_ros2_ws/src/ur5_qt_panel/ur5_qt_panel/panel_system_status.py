#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_system_status.py
# Contenido: Helpers de status del sistema (gz/bridge/clock/topics) extraidos de panel_utils.
"""Helpers de status del sistema extraidos de panel_utils.py (refactor B.4).

Extraido de ``panel_utils.py`` (lineas 641-771 originales). Funciones que
consultan estado en ejecucion del stack: Gazebo Sim corriendo, ros_gz_bridge
activo, /clock disponible, listado de topics del grafo, etc.

Reexportado desde panel_utils para mantener compatibilidad con
``panel_object_mgmt``, ``panel_helpers``, ``panel_status_mgmt``.

Funciones publicas:
- gz_sim_status, gz_sim_running
- bridge_status, clock_status
- ros_clock_available, robot_control_available
- detect_arm_trajectory_topic
- parse_ros_topics

Helpers privados:
- _run_cmd_rc
- _create_graph_node
"""

from __future__ import annotations

import os
import re
import shlex
import time
from typing import List, Tuple

try:
    import psutil  # type: ignore
except Exception:  # pragma: no cover
    psutil = None

try:
    from .panel_config import (
        ARM_TRAJ_TOPIC_DEFAULT,
        ROS_AVAILABLE,
        rclpy,
    )
except Exception:  # pragma: no cover
    from .panel_config import (  # type: ignore
        ARM_TRAJ_TOPIC_DEFAULT,
        ROS_AVAILABLE,
        rclpy,
    )

from .panel_ui_params import get_panel_ui_params as _get_panel_ui_params
from .panel_process import run_cmd
from .panel_controllers_query import (
    list_active_controllers,
    resolve_controller_manager,
    ros2_control_running,
)

ROS_TOPIC_RE = re.compile(r"^/([A-Za-z0-9_]+/)*[A-Za-z0-9_]+$")

# Cache de status de Gazebo Sim (TTL bajo para responder rapido al UI).
_GZ_SIM_STATUS_CACHE: Tuple[float, bool, str] = (0.0, False, "none")
_GZ_SIM_STATUS_TTL_SEC = 0.5


def _log_exception(context: str, exc: Exception) -> None:
    """Inline simplificado para evitar circular import con panel_utils."""

    debug_enabled = _get_panel_ui_params().debug_exceptions
    if debug_enabled:
        import traceback
        traceback.print_exc()


# Las funciones extraidas se apenden aqui via sed.
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
