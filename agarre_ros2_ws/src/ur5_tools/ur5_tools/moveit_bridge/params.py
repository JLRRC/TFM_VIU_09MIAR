#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/moveit_bridge/params.py
# Contenido: F2 — parámetros runtime del bridge MoveIt (early-success / approach).
"""Tunables runtime del bridge MoveIt (F2 bucket D).

Quinta dataclass F2. Cubre los flags y timeouts que controlan el
comportamiento del bridge MoveIt en `executor.py`, `joint_state_helpers.py`
y `moveit_py_planner.py`. Estos eran 7 env vars distribuidas en 4 sitios
distintos.

Prioridad: env > YAML > default.
YAML: ``ur5_tools/config/moveit_bridge_runtime.yaml`` (instalable vía CMake
si se decide; por ahora el YAML default vive en source-tree).

NO incluye:
- env vars leídas via ``_env_float(name, default)`` con nombre dinámico:
  PANEL_MOVEIT_BRIDGE_APPROACH_PATH_CONSTRAINT_TOL_RAD,
  PANEL_MOVEIT_BRIDGE_APPROACH_RELAXED_PATH_CONSTRAINT_TOL_RAD,
  PANEL_MOVEIT_BRIDGE_PLAN_EE_TARGET_TOL_M y similares — requieren
  refactor del helper antes de migrar.
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
        share_dir = Path(get_package_share_directory("ur5_tools"))
        candidate = share_dir / "config" / "moveit_bridge_runtime.yaml"
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    return (
        Path(__file__).resolve().parent.parent.parent
        / "config"
        / "moveit_bridge_runtime.yaml"
    )


_DEFAULT_YAML_PATH = _resolve_default_yaml_path()


@dataclass(frozen=True)
class MoveItBridgeParams:
    """Tunables runtime del bridge MoveIt."""

    # -- Early-success switches (allow trayectoria a terminar antes) --
    allow_feedback_early_success: bool = False  # PANEL_MOVEIT_BRIDGE_ALLOW_FEEDBACK_EARLY_SUCCESS
    allow_joint_early_success: bool = False     # PANEL_MOVEIT_BRIDGE_ALLOW_JOINT_EARLY_SUCCESS
    allow_ee_early_success: bool = False        # PANEL_MOVEIT_BRIDGE_ALLOW_EE_EARLY_SUCCESS

    # -- APPROACH constraint behaviour --
    approach_internal_replan: bool = True       # PANEL_MOVEIT_BRIDGE_APPROACH_INTERNAL_REPLAN
    approach_skip_constraints: bool = False     # PANEL_MOVEIT_BRIDGE_APPROACH_SKIP_CONSTRAINTS
    approach_relaxed_constraint_retry: bool = True  # PANEL_MOVEIT_BRIDGE_APPROACH_RELAXED_CONSTRAINT_RETRY

    # -- Request timeout --
    request_timeout_sec: float = 60.0           # PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC

    # -- Motion gating (lecturas en panel_motion_helpers / panel_startup) --
    # Conceptualmente parte del bridge MoveIt aunque vivan en el panel.
    tf_gate_timeout_sec: float = 1.2                # PANEL_MOVEIT_TF_GATE_TIMEOUT_SEC
    startup_timeout_sec: float = 40.0               # PANEL_MOVEIT_STARTUP_TIMEOUT_SEC
    wait_joint_target_max_age_sec: float = 0.35     # PANEL_WAIT_JOINT_TARGET_MAX_AGE_SEC
    wait_joint_target_max_vel_rad_s: float = 0.05   # PANEL_WAIT_JOINT_TARGET_MAX_VEL_RAD_S
    wait_joint_target_stable_samples: int = 3       # PANEL_WAIT_JOINT_TARGET_STABLE_SAMPLES
    disable_joint_wrap_align: bool = False          # PANEL_DISABLE_JOINT_WRAP_ALIGN


ENV_VAR_BY_FIELD: Dict[str, str] = {
    "allow_feedback_early_success":         "PANEL_MOVEIT_BRIDGE_ALLOW_FEEDBACK_EARLY_SUCCESS",
    "allow_joint_early_success":            "PANEL_MOVEIT_BRIDGE_ALLOW_JOINT_EARLY_SUCCESS",
    "allow_ee_early_success":               "PANEL_MOVEIT_BRIDGE_ALLOW_EE_EARLY_SUCCESS",
    "approach_internal_replan":             "PANEL_MOVEIT_BRIDGE_APPROACH_INTERNAL_REPLAN",
    "approach_skip_constraints":            "PANEL_MOVEIT_BRIDGE_APPROACH_SKIP_CONSTRAINTS",
    "approach_relaxed_constraint_retry":    "PANEL_MOVEIT_BRIDGE_APPROACH_RELAXED_CONSTRAINT_RETRY",
    "request_timeout_sec":                  "PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC",
    "tf_gate_timeout_sec":                  "PANEL_MOVEIT_TF_GATE_TIMEOUT_SEC",
    "startup_timeout_sec":                  "PANEL_MOVEIT_STARTUP_TIMEOUT_SEC",
    "wait_joint_target_max_age_sec":        "PANEL_WAIT_JOINT_TARGET_MAX_AGE_SEC",
    "wait_joint_target_max_vel_rad_s":      "PANEL_WAIT_JOINT_TARGET_MAX_VEL_RAD_S",
    "wait_joint_target_stable_samples":     "PANEL_WAIT_JOINT_TARGET_STABLE_SAMPLES",
    "disable_joint_wrap_align":             "PANEL_DISABLE_JOINT_WRAP_ALIGN",
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


def load_moveit_bridge_params(yaml_path: Optional[Path] = None) -> MoveItBridgeParams:
    target = yaml_path if yaml_path is not None else _DEFAULT_YAML_PATH
    yaml_data = _read_yaml(target)
    base = MoveItBridgeParams()
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
    return MoveItBridgeParams(**{**{f.name: getattr(base, f.name) for f in fields(base)}, **overrides})


_MOVEIT_BRIDGE_PARAMS_CACHE: Optional[MoveItBridgeParams] = None


def get_moveit_bridge_params() -> MoveItBridgeParams:
    global _MOVEIT_BRIDGE_PARAMS_CACHE
    if _MOVEIT_BRIDGE_PARAMS_CACHE is None:
        _MOVEIT_BRIDGE_PARAMS_CACHE = load_moveit_bridge_params()
    return _MOVEIT_BRIDGE_PARAMS_CACHE


def reset_moveit_bridge_params_cache() -> None:
    global _MOVEIT_BRIDGE_PARAMS_CACHE
    _MOVEIT_BRIDGE_PARAMS_CACHE = None
