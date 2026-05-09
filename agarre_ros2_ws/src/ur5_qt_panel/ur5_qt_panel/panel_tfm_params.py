#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tfm_params.py
# Contenido: F2 — parámetros runtime del flujo TFM (inferencia + remote + canonical).
"""Tunables runtime del flujo TFM (F2 bucket E).

Sexta dataclass F2. Cubre:

- TFM science / output mode (``PANEL_TFM_REPRO_MODE``,
  ``PANEL_TFM_RAW_OUTPUT``).
- Edad máxima de frames de inferencia y de grasp resultante.
- Gating MoveIt active-request para infer/execute.
- Modo preprocesado ROI.
- Modo execute pretable y grasp cartesiano.
- TFM canonical: gating de pick_object, grasp yaw, snap selected XY,
  delta XY máx.
- TFM remote ready waits/polls (infer / execute).
- Pick demo remote ready waits/polls.

Prioridad: env > YAML > default.
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
        candidate = share_dir / "config" / "panel_tfm_runtime.yaml"
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    return (
        Path(__file__).resolve().parent.parent
        / "config"
        / "panel_tfm_runtime.yaml"
    )


_DEFAULT_YAML_PATH = _resolve_default_yaml_path()


@dataclass(frozen=True)
class PanelTfmParams:
    """Tunables runtime del flujo TFM."""

    # -- Science / output --
    repro_mode: str = ""                          # PANEL_TFM_REPRO_MODE
    raw_output: str = ""                          # PANEL_TFM_RAW_OUTPUT

    # -- Edad máxima de frames --
    infer_frame_max_age_sec: float = 4.0          # PANEL_TFM_INFER_FRAME_MAX_AGE_SEC
    grasp_max_age_sec: float = 60.0               # PANEL_TFM_GRASP_MAX_AGE_SEC

    # -- MoveIt active-request gating --
    moveit_active_request_grace_sec: float = 90.0 # PANEL_TFM_MOVEIT_ACTIVE_REQUEST_GRACE_SEC
    moveit_active_request_hb_sec: float = 2.5     # PANEL_TFM_MOVEIT_ACTIVE_REQUEST_HB_SEC

    # -- Preprocess --
    infer_use_roi: str = "auto"                   # PANEL_TFM_INFER_USE_ROI

    # -- Execute / grasp mode --
    execute_pretable: bool = True                 # PANEL_TFM_EXECUTE_PRETABLE
    grasp_cartesian: bool = False                 # PANEL_TFM_GRASP_CARTESIAN

    # -- Canonical --
    canonical_use_pick_object: bool = True        # PANEL_TFM_CANONICAL_USE_PICK_OBJECT
    canonical_use_grasp_yaw: bool = True          # PANEL_TFM_CANONICAL_USE_GRASP_YAW
    canonical_grasp_xy_max_delta_m: float = 0.080 # PANEL_TFM_CANONICAL_GRASP_XY_MAX_DELTA_M
    canonical_snap_selected_xy: bool = True       # PANEL_TFM_CANONICAL_SNAP_SELECTED_XY

    # -- Remote ready (TFM infer / execute) --
    remote_infer_ready_wait_sec: float = 20.0     # PANEL_TFM_REMOTE_INFER_READY_WAIT_SEC
    remote_infer_ready_poll_sec: float = 0.25     # PANEL_TFM_REMOTE_INFER_READY_POLL_SEC
    remote_execute_ready_wait_sec: float = 90.0   # PANEL_TFM_REMOTE_EXECUTE_READY_WAIT_SEC
    remote_execute_ready_poll_sec: float = 1.0    # PANEL_TFM_REMOTE_EXECUTE_READY_POLL_SEC

    # -- Pick demo remote ready --
    pick_demo_remote_ready_wait_sec: float = 90.0 # PANEL_PICK_DEMO_REMOTE_READY_WAIT_SEC
    pick_demo_remote_ready_poll_sec: float = 0.5  # PANEL_PICK_DEMO_REMOTE_READY_POLL_SEC


ENV_VAR_BY_FIELD: Dict[str, str] = {
    "repro_mode":                          "PANEL_TFM_REPRO_MODE",
    "raw_output":                          "PANEL_TFM_RAW_OUTPUT",
    "infer_frame_max_age_sec":             "PANEL_TFM_INFER_FRAME_MAX_AGE_SEC",
    "grasp_max_age_sec":                   "PANEL_TFM_GRASP_MAX_AGE_SEC",
    "moveit_active_request_grace_sec":     "PANEL_TFM_MOVEIT_ACTIVE_REQUEST_GRACE_SEC",
    "moveit_active_request_hb_sec":        "PANEL_TFM_MOVEIT_ACTIVE_REQUEST_HB_SEC",
    "infer_use_roi":                       "PANEL_TFM_INFER_USE_ROI",
    "execute_pretable":                    "PANEL_TFM_EXECUTE_PRETABLE",
    "grasp_cartesian":                     "PANEL_TFM_GRASP_CARTESIAN",
    "canonical_use_pick_object":           "PANEL_TFM_CANONICAL_USE_PICK_OBJECT",
    "canonical_use_grasp_yaw":             "PANEL_TFM_CANONICAL_USE_GRASP_YAW",
    "canonical_grasp_xy_max_delta_m":      "PANEL_TFM_CANONICAL_GRASP_XY_MAX_DELTA_M",
    "canonical_snap_selected_xy":          "PANEL_TFM_CANONICAL_SNAP_SELECTED_XY",
    "remote_infer_ready_wait_sec":         "PANEL_TFM_REMOTE_INFER_READY_WAIT_SEC",
    "remote_infer_ready_poll_sec":         "PANEL_TFM_REMOTE_INFER_READY_POLL_SEC",
    "remote_execute_ready_wait_sec":       "PANEL_TFM_REMOTE_EXECUTE_READY_WAIT_SEC",
    "remote_execute_ready_poll_sec":       "PANEL_TFM_REMOTE_EXECUTE_READY_POLL_SEC",
    "pick_demo_remote_ready_wait_sec":     "PANEL_PICK_DEMO_REMOTE_READY_WAIT_SEC",
    "pick_demo_remote_ready_poll_sec":     "PANEL_PICK_DEMO_REMOTE_READY_POLL_SEC",
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


def load_panel_tfm_params(yaml_path: Optional[Path] = None) -> PanelTfmParams:
    target = yaml_path if yaml_path is not None else _DEFAULT_YAML_PATH
    yaml_data = _read_yaml(target)
    base = PanelTfmParams()
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
    return PanelTfmParams(**{**{f.name: getattr(base, f.name) for f in fields(base)}, **overrides})


_PANEL_TFM_PARAMS_CACHE: Optional[PanelTfmParams] = None


def get_panel_tfm_params() -> PanelTfmParams:
    global _PANEL_TFM_PARAMS_CACHE
    if _PANEL_TFM_PARAMS_CACHE is None:
        _PANEL_TFM_PARAMS_CACHE = load_panel_tfm_params()
    return _PANEL_TFM_PARAMS_CACHE


def reset_panel_tfm_params_cache() -> None:
    global _PANEL_TFM_PARAMS_CACHE
    _PANEL_TFM_PARAMS_CACHE = None
