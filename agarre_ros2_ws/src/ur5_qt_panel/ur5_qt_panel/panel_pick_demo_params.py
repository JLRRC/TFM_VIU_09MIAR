#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo_params.py
# Contenido: F2 parcial — infraestructura de parámetros runtime para pick_demo.
# Uso breve:
#   from .panel_pick_demo_params import load_pick_demo_params
#   params = load_pick_demo_params()
#   z_offset = params.grasp_tcp_z_offset_m
"""Parámetros runtime del flujo pick_demo (F2 parcial).

Hoy ``panel_pick_demo.py`` lee 151 ``os.environ`` distintas. Este módulo
introduce una capa única (dataclass) con los **11 parámetros más críticos**
para que el tuneo se haga vía YAML en lugar de exportar variables. Es
trabajo incremental: el código antiguo que sigue leyendo ``os.environ``
NO se toca; este módulo simplemente provee otra puerta de entrada con
estas reglas de prioridad:

    env var > YAML override > default del dataclass

Para tunear sin perder reproducibilidad:
1. Editar ``agarre_ros2_ws/src/ur5_qt_panel/config/pick_demo_runtime.yaml``
2. (Alternativa rápida) ``export PANEL_PICK_DEMO_<NAME>=value`` y relanzar

La tabla de equivalencias está en el YAML (cada clave del YAML lleva el
nombre de la env como comentario).

Migración futura: cuando se quiera quitar un ``os.environ.get`` del
flujo grande, cambiarlo por ``params.<campo>`` aquí y validar que el
default del dataclass coincide con el default histórico de la env.
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


# Ruta absoluta del YAML por defecto. Si no existe, se usan los defaults
# del dataclass (idénticos a los que tenía cada ``os.environ.get`` en
# panel_pick_demo / pick_demo/internal_helpers / directo_geometry).
def _resolve_default_yaml_path() -> Path:
    """Resolver la ruta del YAML tanto en install como en source-tree."""
    # 1) Install (colcon): share/ur5_qt_panel/config/pick_demo_runtime.yaml
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = Path(get_package_share_directory("ur5_qt_panel"))
        candidate = share_dir / "config" / "pick_demo_runtime.yaml"
        if candidate.is_file():
            return candidate
    except Exception:
        pass
    # 2) Source-tree (dev): src/ur5_qt_panel/config/pick_demo_runtime.yaml
    src_candidate = (
        Path(__file__).resolve().parent.parent
        / "config"
        / "pick_demo_runtime.yaml"
    )
    return src_candidate


_DEFAULT_YAML_PATH = _resolve_default_yaml_path()


@dataclass(frozen=True)
class PickDemoParams:
    """11 parámetros críticos del flujo pick_demo.

    Defaults extraídos de las llamadas ``os.environ.get(NAME, default)`` del
    código histórico. Cualquier cambio aquí debe mantener paridad con esos
    defaults para no alterar comportamiento sin querer.
    """

    # -- IK seed override --------------------------------------------------
    # Empty string ≡ "auto" (el flujo elige seed dinámicamente).
    ik_seed_joints: str = ""  # PANEL_PICK_DEMO_IK_SEED_JOINTS

    # -- Tolerancias de gates ----------------------------------------------
    grasp_down_util_z_err_tol_m: float = 0.025  # PANEL_PICK_DEMO_GRASP_DOWN_UTIL_Z_ERR_TOL_M
    close_z_err_tol_m: float = 0.012            # PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M
    close_xy_tol_m: float = 0.012               # PANEL_PICK_DEMO_CLOSE_XY_TOL_M
    approach_coarse_keep_xy_tol_m: float = 0.020  # PANEL_PICK_DEMO_APPROACH_COARSE_KEEP_XY_TOL_M

    # -- Geometría TCP -----------------------------------------------------
    grasp_tcp_z_offset_m: float = 0.0  # PANEL_PICK_DEMO_GRASP_TCP_Z_OFFSET_M

    # -- Pinza -------------------------------------------------------------
    gripper_closed_opening_thr_m: float = 0.020  # PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M

    # -- Tiempos -----------------------------------------------------------
    # None ≡ usar el spinbox del panel (joint_time).
    move_sec: Optional[float] = None  # PANEL_PICK_DEMO_MOVE_SEC

    # -- Pesos / tolerancias IK GRASP_DOWN --------------------------------
    grasp_down_ik_err_tol: float = 0.080      # PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL
    grasp_down_ik_seed_weight: float = 0.65   # PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT
    grasp_down_rot_weight: float = 0.10       # PANEL_PICK_DEMO_GRASP_DOWN_ROT_WEIGHT

    # -- Tolerancias TCP por fase del pick (DIRECTO) ----------------------
    # Consumidas en directo_geometry._direct_runtime_target_tol_m. Cada fase
    # del flujo tiene su propia tolerancia objetivo del TCP en metros.
    approach_coarse_tcp_tol_m: float = 0.015        # PANEL_PICK_DEMO_APPROACH_COARSE_TCP_TOL_M
    approach_coarse_refine_tcp_tol_m: float = 0.006 # PANEL_PICK_DEMO_APPROACH_COARSE_REFINE_TCP_TOL_M
    grasp_down_tcp_tol_m: float = 0.020             # PANEL_PICK_DEMO_GRASP_DOWN_TCP_TOL_M
    grasp_align_tcp_tol_m: float = 0.015            # PANEL_PICK_DEMO_GRASP_ALIGN_TCP_TOL_M
    basket_transport_tcp_tol_m: float = 0.060       # PANEL_PICK_DEMO_BASKET_TRANSPORT_TCP_TOL_M
    direct_ik_tcp_tol_m: float = 0.040              # PANEL_PICK_DEMO_DIRECT_IK_TCP_TOL_M


# Mapeo campo dataclass → nombre de env var.
# Útil para implementar la prioridad env > YAML > default y para auditar
# qué parámetros existen.
ENV_VAR_BY_FIELD: Dict[str, str] = {
    "ik_seed_joints":                  "PANEL_PICK_DEMO_IK_SEED_JOINTS",
    "grasp_down_util_z_err_tol_m":     "PANEL_PICK_DEMO_GRASP_DOWN_UTIL_Z_ERR_TOL_M",
    "close_z_err_tol_m":               "PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M",
    "close_xy_tol_m":                  "PANEL_PICK_DEMO_CLOSE_XY_TOL_M",
    "approach_coarse_keep_xy_tol_m":   "PANEL_PICK_DEMO_APPROACH_COARSE_KEEP_XY_TOL_M",
    "grasp_tcp_z_offset_m":            "PANEL_PICK_DEMO_GRASP_TCP_Z_OFFSET_M",
    "gripper_closed_opening_thr_m":    "PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M",
    "move_sec":                        "PANEL_PICK_DEMO_MOVE_SEC",
    "grasp_down_ik_err_tol":           "PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL",
    "grasp_down_ik_seed_weight":       "PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT",
    "grasp_down_rot_weight":           "PANEL_PICK_DEMO_GRASP_DOWN_ROT_WEIGHT",
    "approach_coarse_tcp_tol_m":        "PANEL_PICK_DEMO_APPROACH_COARSE_TCP_TOL_M",
    "approach_coarse_refine_tcp_tol_m": "PANEL_PICK_DEMO_APPROACH_COARSE_REFINE_TCP_TOL_M",
    "grasp_down_tcp_tol_m":             "PANEL_PICK_DEMO_GRASP_DOWN_TCP_TOL_M",
    "grasp_align_tcp_tol_m":            "PANEL_PICK_DEMO_GRASP_ALIGN_TCP_TOL_M",
    "basket_transport_tcp_tol_m":       "PANEL_PICK_DEMO_BASKET_TRANSPORT_TCP_TOL_M",
    "direct_ik_tcp_tol_m":              "PANEL_PICK_DEMO_DIRECT_IK_TCP_TOL_M",
}


def _coerce(field_name: str, raw: Any, default: Any) -> Any:
    """Convertir ``raw`` al tipo del default; devolver ``default`` si falla."""
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
    if default is None:
        # Optional[float] como move_sec: aceptar float o None
        s = str(raw).strip()
        if not s:
            return None
        try:
            return float(s)
        except (TypeError, ValueError):
            return None
    # str u otros
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


def load_pick_demo_params(yaml_path: Optional[Path] = None) -> PickDemoParams:
    """Cargar los parámetros con prioridad env > YAML > default.

    El env var (``ENV_VAR_BY_FIELD[campo]``), si existe en ``os.environ``,
    gana. Si no, se busca la clave en el YAML (nombre del campo, p.ej.
    ``grasp_tcp_z_offset_m``). Si tampoco está, se usa el default del
    dataclass. Cualquier valor que no se pueda coercionar al tipo
    correspondiente cae al default sin levantar excepción.
    """
    target = yaml_path if yaml_path is not None else _DEFAULT_YAML_PATH
    yaml_data = _read_yaml(target)

    base = PickDemoParams()
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
        # else: keep default

    if not overrides:
        return base
    return PickDemoParams(**{**{f.name: getattr(base, f.name) for f in fields(base)}, **overrides})


# ---------------------------------------------------------------------------
# Lazy singleton — usado por panel_pick_demo y directo_geometry para evitar
# re-leer YAML/env en cada gate del pick. Vive aquí para que ambos módulos
# importen el mismo helper sin riesgo de dependencia circular.
# ---------------------------------------------------------------------------
_PICK_DEMO_PARAMS_CACHE: Optional[PickDemoParams] = None


def get_pick_demo_params() -> PickDemoParams:
    """Lazy singleton de PickDemoParams (env > YAML > default).

    F2: las lecturas migradas usan este helper en lugar de os.environ.get.
    Para invalidar (tests), usa reset_pick_demo_params_cache().
    """
    global _PICK_DEMO_PARAMS_CACHE
    if _PICK_DEMO_PARAMS_CACHE is None:
        _PICK_DEMO_PARAMS_CACHE = load_pick_demo_params()
    return _PICK_DEMO_PARAMS_CACHE


def reset_pick_demo_params_cache() -> None:
    """Invalida el singleton. Útil en tests que mutan env vars."""
    global _PICK_DEMO_PARAMS_CACHE
    _PICK_DEMO_PARAMS_CACHE = None
