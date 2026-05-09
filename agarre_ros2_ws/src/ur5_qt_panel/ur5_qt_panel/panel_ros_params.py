#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_ros_params.py
# Contenido: F2 — parámetros runtime de servicios/topics y timeouts ROS del panel.
"""Parámetros runtime de la interfaz ROS del panel (F2 bucket B).

Hermano de ``panel_pick_demo_params`` y ``panel_pick_object_params``.
Cubre los **servicios y topics** que conectan el panel con el resto del
stack ROS 2 (triggers de cámara, pick demo, pick object, TFM, recover,
select object, etc.) y los timeouts asociados.

Prioridad de resolución (idéntica a las otras dataclasses F2):

    env var (PANEL_*)  >  YAML override  >  default del dataclass

YAML: ``config/panel_ros_runtime.yaml``.

NO incluye:
- ``PANEL_DEBUG_EXCEPTIONS``: flag UI cross-cutting (irá en bucket C
  ``PanelUiParams``).
- ``PANEL_MAX_FPS``: tunable de cámara/UI (irá en bucket C).
- ``USE_SIM_TIME``: env var estructural del entorno ROS, no un tunable.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, Dict, Optional

try:
    import yaml  # type: ignore
except ImportError:
    yaml = None  # type: ignore


def _resolve_default_yaml_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = Path(get_package_share_directory("ur5_qt_panel"))
        candidate = share_dir / "config" / "panel_ros_runtime.yaml"
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    return (
        Path(__file__).resolve().parent.parent
        / "config"
        / "panel_ros_runtime.yaml"
    )


_DEFAULT_YAML_PATH = _resolve_default_yaml_path()


@dataclass(frozen=True)
class PanelRosParams:
    """Tunables ROS del panel: nombres de servicios/topics + timeouts."""

    # -- Camera triggers --
    camera_connect_trigger_service: str = "/panel/camera_connect"     # PANEL_CAMERA_CONNECT_TRIGGER_SERVICE
    camera_disconnect_trigger_service: str = "/panel/camera_disconnect"  # PANEL_CAMERA_DISCONNECT_TRIGGER_SERVICE

    # -- Recover --
    recover_trigger_topic: str = "/panel/recover"                     # PANEL_RECOVER_TRIGGER_TOPIC
    recover_trigger_service: str = "/panel/recover"                   # PANEL_RECOVER_TRIGGER_SERVICE

    # -- TFM (infer / execute) --
    tfm_infer_trigger_topic: str = "/panel/tfm_infer"                 # PANEL_TFM_INFER_TRIGGER_TOPIC
    tfm_infer_trigger_service: str = "/panel/tfm_infer"               # PANEL_TFM_INFER_TRIGGER_SERVICE
    tfm_execute_trigger_topic: str = "/panel/tfm_execute"             # PANEL_TFM_EXECUTE_TRIGGER_TOPIC
    tfm_execute_trigger_service: str = "/panel/tfm_execute"           # PANEL_TFM_EXECUTE_TRIGGER_SERVICE
    tfm_infer_service_timeout_sec: float = 30.0                       # PANEL_TFM_INFER_SERVICE_TIMEOUT_SEC
    tfm_execute_service_timeout_sec: float = 480.0                    # PANEL_TFM_EXECUTE_SERVICE_TIMEOUT_SEC

    # -- Pick demo --
    pick_demo_trigger_service: str = "/panel/pick_demo"               # PANEL_PICK_DEMO_TRIGGER_SERVICE
    pick_demo_service_timeout_sec: float = 120.0                      # PANEL_PICK_DEMO_SERVICE_TIMEOUT_SEC

    # -- Pick object --
    pick_object_trigger_topic: str = "/panel/pick_object"             # PANEL_PICK_OBJECT_TRIGGER_TOPIC
    pick_object_trigger_service: str = "/panel/pick_object"           # PANEL_PICK_OBJECT_TRIGGER_SERVICE

    # -- Select object --
    select_object_topic: str = "/panel/select_object"                 # PANEL_SELECT_OBJECT_TOPIC
    select_object_service: str = "/panel/select_object"               # PANEL_SELECT_OBJECT_SERVICE
    select_object_service_timeout_sec: float = 10.0                   # PANEL_SELECT_OBJECT_SERVICE_TIMEOUT_SEC

    # -- Remote select gating --
    remote_select_on_table_wait_sec: float = 8.0                      # PANEL_REMOTE_SELECT_ON_TABLE_WAIT_SEC

    # -- ROS executor --
    ros_executor_threads: int = 3                                     # PANEL_ROS_EXECUTOR_THREADS

    # -- Trajectory action / controller manager --
    expected_traj_action: str = "/joint_trajectory_controller/follow_joint_trajectory"  # PANEL_EXPECTED_TRAJ_ACTION
    strict_traj_action: bool = True                                   # PANEL_STRICT_TRAJ_ACTION
    controller_manager: str = ""                                      # PANEL_CONTROLLER_MANAGER

    # -- ROS misc --
    remote_select_on_table_poll_sec: float = 0.25                     # PANEL_REMOTE_SELECT_ON_TABLE_POLL_SEC

    # -- Attach z-ref mode (panel_gz_objects) --
    attach_z_ref_mode: str = "top"                                    # PANEL_ATTACH_Z_REF_MODE


ENV_VAR_BY_FIELD: Dict[str, str] = {
    "camera_connect_trigger_service":      "PANEL_CAMERA_CONNECT_TRIGGER_SERVICE",
    "camera_disconnect_trigger_service":   "PANEL_CAMERA_DISCONNECT_TRIGGER_SERVICE",
    "recover_trigger_topic":               "PANEL_RECOVER_TRIGGER_TOPIC",
    "recover_trigger_service":             "PANEL_RECOVER_TRIGGER_SERVICE",
    "tfm_infer_trigger_topic":             "PANEL_TFM_INFER_TRIGGER_TOPIC",
    "tfm_infer_trigger_service":           "PANEL_TFM_INFER_TRIGGER_SERVICE",
    "tfm_execute_trigger_topic":           "PANEL_TFM_EXECUTE_TRIGGER_TOPIC",
    "tfm_execute_trigger_service":         "PANEL_TFM_EXECUTE_TRIGGER_SERVICE",
    "tfm_infer_service_timeout_sec":       "PANEL_TFM_INFER_SERVICE_TIMEOUT_SEC",
    "tfm_execute_service_timeout_sec":     "PANEL_TFM_EXECUTE_SERVICE_TIMEOUT_SEC",
    "pick_demo_trigger_service":           "PANEL_PICK_DEMO_TRIGGER_SERVICE",
    "pick_demo_service_timeout_sec":       "PANEL_PICK_DEMO_SERVICE_TIMEOUT_SEC",
    "pick_object_trigger_topic":           "PANEL_PICK_OBJECT_TRIGGER_TOPIC",
    "pick_object_trigger_service":         "PANEL_PICK_OBJECT_TRIGGER_SERVICE",
    "select_object_topic":                 "PANEL_SELECT_OBJECT_TOPIC",
    "select_object_service":               "PANEL_SELECT_OBJECT_SERVICE",
    "select_object_service_timeout_sec":   "PANEL_SELECT_OBJECT_SERVICE_TIMEOUT_SEC",
    "remote_select_on_table_wait_sec":     "PANEL_REMOTE_SELECT_ON_TABLE_WAIT_SEC",
    "ros_executor_threads":                "PANEL_ROS_EXECUTOR_THREADS",
    "expected_traj_action":                "PANEL_EXPECTED_TRAJ_ACTION",
    "strict_traj_action":                  "PANEL_STRICT_TRAJ_ACTION",
    "controller_manager":                  "PANEL_CONTROLLER_MANAGER",
    "remote_select_on_table_poll_sec":     "PANEL_REMOTE_SELECT_ON_TABLE_POLL_SEC",
    "attach_z_ref_mode":                   "PANEL_ATTACH_Z_REF_MODE",
}


def _coerce(field_name: str, raw: Any, default: Any) -> Any:
    if raw is None:
        return default
    if isinstance(default, bool):
        s = str(raw).strip().lower()
        return s in ("1", "true", "yes", "on")
    if isinstance(default, int) and not isinstance(default, bool):
        try:
            return int(raw)
        except (TypeError, ValueError):
            return default
    if isinstance(default, float):
        try:
            return float(raw)
        except (TypeError, ValueError):
            return default
    return str(raw)


def _read_yaml(path: Path) -> Dict[str, Any]:
    if yaml is None or not path.is_file():
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return {}
    if not isinstance(data, dict):
        return {}
    return data


def load_panel_ros_params(yaml_path: Optional[Path] = None) -> PanelRosParams:
    target = yaml_path if yaml_path is not None else _DEFAULT_YAML_PATH
    yaml_data = _read_yaml(target)
    base = PanelRosParams()
    overrides: Dict[str, Any] = {}
    for f in fields(base):
        default = getattr(base, f.name)
        env_name = ENV_VAR_BY_FIELD.get(f.name, "")
        env_raw = os.environ.get(env_name) if env_name else None
        yaml_raw = yaml_data.get(f.name)
        if env_raw is not None and env_raw != "":
            overrides[f.name] = _coerce(f.name, env_raw, default)
        elif yaml_raw is not None:
            overrides[f.name] = _coerce(f.name, yaml_raw, default)
    if not overrides:
        return base
    return PanelRosParams(**{**{f.name: getattr(base, f.name) for f in fields(base)}, **overrides})


_PANEL_ROS_PARAMS_CACHE: Optional[PanelRosParams] = None


def get_panel_ros_params() -> PanelRosParams:
    """Lazy singleton (env > YAML > default)."""
    global _PANEL_ROS_PARAMS_CACHE
    if _PANEL_ROS_PARAMS_CACHE is None:
        _PANEL_ROS_PARAMS_CACHE = load_panel_ros_params()
    return _PANEL_ROS_PARAMS_CACHE


def reset_panel_ros_params_cache() -> None:
    global _PANEL_ROS_PARAMS_CACHE
    _PANEL_ROS_PARAMS_CACHE = None
