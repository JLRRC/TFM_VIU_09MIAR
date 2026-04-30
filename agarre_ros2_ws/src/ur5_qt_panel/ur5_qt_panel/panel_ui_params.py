#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_ui_params.py
# Contenido: F2 — flags UI/debug del panel (cross-cutting).
"""Tunables UI/debug del panel (F2 bucket C).

Cuarta dataclass F2. Cubre los flags que controlan:

- Manejo de excepciones de debug y modos diagnóstico (cross-cutting:
  ``PANEL_DEBUG_EXCEPTIONS`` se lee en 15+ módulos).
- Modo headless (``PANEL_FORCE_OFFSCREEN``).
- Auto-arranque del pick demo.
- Pausas de debug.
- Gating de cámaras/gripper durante motion.
- Overlays en la imagen del panel.
- Frame canónico del EE.

Prioridad de resolución: env > YAML > default.

NO incluye:
- ``PANEL_MAX_FPS``: divergencia (60 en panel_ros, 12 en cameras_tab).
- ``PANEL_SINGLE_CAM``: default runtime-dependent (``"1" if ros2_only
  else "0"``).
- ``PANEL_ROS2_ONLY``, ``PANEL_KEEP_CAMERAS``, ``PANEL_CAMERA_REQUIRED``:
  flags ad-hoc del launcher con uno o dos sites cada uno; quedan en
  panel_launchers.py.
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
        candidate = share_dir / "config" / "panel_ui_runtime.yaml"
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    return (
        Path(__file__).resolve().parent.parent
        / "config"
        / "panel_ui_runtime.yaml"
    )


_DEFAULT_YAML_PATH = _resolve_default_yaml_path()


@dataclass(frozen=True)
class PanelUiParams:
    """Flags UI/debug del panel."""

    # -- Debug --
    debug_exceptions: bool = False                  # PANEL_DEBUG_EXCEPTIONS
    diag_mode: bool = False                         # PANEL_DIAG_MODE
    fatal_stops_all: bool = False                   # PANEL_FATAL_STOPS_ALL
    metrics: bool = False                           # PANEL_METRICS

    # -- Display / headless --
    force_offscreen: bool = False                   # PANEL_FORCE_OFFSCREEN

    # -- Auto-run --
    auto_run_pick_demo: bool = False                # PANEL_AUTO_RUN_PICK_DEMO
    auto_run_pick_demo_attempts: int = 1            # PANEL_AUTO_RUN_PICK_DEMO_ATTEMPTS

    # -- Debug pause --
    debug_pause_alcance: bool = False               # PANEL_DEBUG_PAUSE_ALCANCE
    debug_pause_timeout_sec: float = 0.0            # PANEL_DEBUG_PAUSE_TIMEOUT_SEC

    # -- While-motion gating --
    allow_camera_while_motion: bool = True          # PANEL_ALLOW_CAMERA_WHILE_MOTION
    allow_gripper_while_motion: bool = True         # PANEL_ALLOW_GRIPPER_WHILE_MOTION
    allow_close_while_motion: bool = False          # PANEL_ALLOW_CLOSE_WHILE_MOTION
    manual_controls_always_enabled: bool = True     # PANEL_MANUAL_CONTROLS_ALWAYS_ENABLED

    # -- Camera config --
    camera_require_depth: bool = False              # PANEL_CAMERA_REQUIRE_DEPTH

    # -- Required EE frame --
    required_ee_frame: str = "rg2_pinch_center"     # PANEL_REQUIRED_EE_FRAME

    # -- Overlays --
    grasp_rect_topic: str = "/grasp_rect"           # PANEL_GRASP_RECT_TOPIC
    test_corner_overlay: bool = False               # PANEL_TEST_CORNER_OVERLAY
    test_corner_inset_m: float = 0.06               # PANEL_TEST_CORNER_INSET_M
    tcp_pose_overlay: bool = True                   # PANEL_TCP_POSE_OVERLAY
    tcp_pose_text_overlay: bool = False             # PANEL_TCP_POSE_TEXT_OVERLAY

    # -- TF gating --
    tf_drop_grace_sec: float = 4.0                  # PANEL_TF_DROP_GRACE_SEC

    # -- Debug extra --
    debug_pick_obj: bool = False                    # DEBUG_PICK_OBJ
    direct_debug_root: str = ""                     # PANEL_DIRECT_DEBUG_ROOT (empty = use historical fallback)
    strict_joint_identity: bool = True              # PANEL_STRICT_JOINT_IDENTITY


ENV_VAR_BY_FIELD: Dict[str, str] = {
    "debug_exceptions":                "PANEL_DEBUG_EXCEPTIONS",
    "diag_mode":                       "PANEL_DIAG_MODE",
    "fatal_stops_all":                 "PANEL_FATAL_STOPS_ALL",
    "metrics":                         "PANEL_METRICS",
    "force_offscreen":                 "PANEL_FORCE_OFFSCREEN",
    "auto_run_pick_demo":              "PANEL_AUTO_RUN_PICK_DEMO",
    "auto_run_pick_demo_attempts":     "PANEL_AUTO_RUN_PICK_DEMO_ATTEMPTS",
    "debug_pause_alcance":             "PANEL_DEBUG_PAUSE_ALCANCE",
    "debug_pause_timeout_sec":         "PANEL_DEBUG_PAUSE_TIMEOUT_SEC",
    "allow_camera_while_motion":       "PANEL_ALLOW_CAMERA_WHILE_MOTION",
    "allow_gripper_while_motion":      "PANEL_ALLOW_GRIPPER_WHILE_MOTION",
    "allow_close_while_motion":        "PANEL_ALLOW_CLOSE_WHILE_MOTION",
    "manual_controls_always_enabled":  "PANEL_MANUAL_CONTROLS_ALWAYS_ENABLED",
    "camera_require_depth":            "PANEL_CAMERA_REQUIRE_DEPTH",
    "required_ee_frame":               "PANEL_REQUIRED_EE_FRAME",
    "grasp_rect_topic":                "PANEL_GRASP_RECT_TOPIC",
    "test_corner_overlay":             "PANEL_TEST_CORNER_OVERLAY",
    "test_corner_inset_m":             "PANEL_TEST_CORNER_INSET_M",
    "tcp_pose_overlay":                "PANEL_TCP_POSE_OVERLAY",
    "tcp_pose_text_overlay":           "PANEL_TCP_POSE_TEXT_OVERLAY",
    "tf_drop_grace_sec":               "PANEL_TF_DROP_GRACE_SEC",
    "debug_pick_obj":                  "DEBUG_PICK_OBJ",
    "direct_debug_root":               "PANEL_DIRECT_DEBUG_ROOT",
    "strict_joint_identity":           "PANEL_STRICT_JOINT_IDENTITY",
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


def load_panel_ui_params(yaml_path: Optional[Path] = None) -> PanelUiParams:
    target = yaml_path if yaml_path is not None else _DEFAULT_YAML_PATH
    yaml_data = _read_yaml(target)
    base = PanelUiParams()
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
    return PanelUiParams(**{**{f.name: getattr(base, f.name) for f in fields(base)}, **overrides})


_PANEL_UI_PARAMS_CACHE: Optional[PanelUiParams] = None


def get_panel_ui_params() -> PanelUiParams:
    global _PANEL_UI_PARAMS_CACHE
    if _PANEL_UI_PARAMS_CACHE is None:
        _PANEL_UI_PARAMS_CACHE = load_panel_ui_params()
    return _PANEL_UI_PARAMS_CACHE


def reset_panel_ui_params_cache() -> None:
    global _PANEL_UI_PARAMS_CACHE
    _PANEL_UI_PARAMS_CACHE = None
