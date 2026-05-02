#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_launchers_params.py
# Contenido: F2-step2 — parámetros runtime de los launchers del panel.
# Uso breve:
#   from .panel_launchers_params import get_launchers_params
#   p = get_launchers_params()
#   if p.use_sim_time: ...
"""Parámetros runtime de ``panel_launchers.py`` (F2-step2).

Hermano de ``panel_pick_demo_params``/``panel_pick_object_params``.
Cubre las env vars que ``panel_launchers.py`` leía directamente sin
canalización tipada:

* ``USE_SIM_TIME`` (4 lecturas idénticas reducidas a una).
* ``ATTACH_BACKEND_MAX_DIST_M`` / ``ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS``.
* ``GZ_RENDER_ENGINE``.
* ``PANEL_KEEP_CAMERAS`` / ``PANEL_CAMERA_REQUIRED`` (semántica "boolean
  laxo"; cualquier string no vacío que no sea explícitamente "0/false"
  cuenta como True — equivalente al bloque histórico de ``in (..)``).
* ``PANEL_MOVEIT_BRIDGE_SIM_TIME``.

Reglas de prioridad:

    env var > YAML override (si se añade en el futuro) > default del dataclass

NO migrados (deliberadamente):
* Helpers internos ``_env_flag`` / ``_env_float_opt`` que canalizan otras
  vars de manera tipada — son la implementación, no configuración.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, fields
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class LaunchersParams:
    """Parámetros estáticos consumidos por ``panel_launchers.py``."""

    # F2-step2: USE_SIM_TIME aparecía repetido 4 veces con la misma
    # comparación (raw == "1"). Centralizada como bool tipado.
    use_sim_time: bool = True  # USE_SIM_TIME

    # F2-step2: backend de attach.
    attach_backend_max_dist_m: float = 0.06           # ATTACH_BACKEND_MAX_DIST_M
    attach_backend_demo_transport_objects: str = "pick_demo"  # ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS

    # F2-step2: configuración Gazebo.
    gz_render_engine: str = "ogre2"  # GZ_RENDER_ENGINE

    # F2-step2: cámaras (semántica laxa: cualquier truthy NO vacío + None=False).
    keep_cameras: bool = False        # PANEL_KEEP_CAMERAS
    camera_required: bool = False     # PANEL_CAMERA_REQUIRED

    # F2-step2: bridge de MoveIt. Default False: MoveItPy en sim-time puede
    # abortar por qos_overrides./clock en ROS 2 Jazzy.
    moveit_bridge_sim_time: bool = False  # PANEL_MOVEIT_BRIDGE_SIM_TIME


ENV_VAR_BY_FIELD: Dict[str, str] = {
    "use_sim_time": "USE_SIM_TIME",
    "attach_backend_max_dist_m": "ATTACH_BACKEND_MAX_DIST_M",
    "attach_backend_demo_transport_objects": "ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS",
    "gz_render_engine": "GZ_RENDER_ENGINE",
    "keep_cameras": "PANEL_KEEP_CAMERAS",
    "camera_required": "PANEL_CAMERA_REQUIRED",
    "moveit_bridge_sim_time": "PANEL_MOVEIT_BRIDGE_SIM_TIME",
}


def _coerce_bool_lax(raw: str) -> bool:
    """Boolean laxo: cualquier string no vacío y no-falsy cuenta como True.

    Replica la semántica histórica de ``raw.strip() in ("1","true","True")``.
    """
    val = str(raw).strip().lower()
    return val in ("1", "true", "yes", "on")


def _coerce(field_name: str, raw: Any, default: Any) -> Any:
    if raw is None:
        return default
    if isinstance(default, bool):
        return _coerce_bool_lax(raw)
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
    s = str(raw).strip()
    if not s:
        return default
    return s


def load_launchers_params() -> LaunchersParams:
    """Cargar los parámetros con prioridad env > default."""
    base = LaunchersParams()
    overrides: Dict[str, Any] = {}
    for f in fields(base):
        env_name = ENV_VAR_BY_FIELD.get(f.name, "")
        env_raw = os.environ.get(env_name) if env_name else None
        # USE_SIM_TIME histórico era ``raw == "1"`` (estricto); para
        # mantener compat la _coerce_bool_lax acepta ("1","true","yes","on").
        if env_raw is not None:
            overrides[f.name] = _coerce(f.name, env_raw, getattr(base, f.name))
    if not overrides:
        return base
    return LaunchersParams(
        **{**{f.name: getattr(base, f.name) for f in fields(base)}, **overrides}
    )


_LAUNCHERS_PARAMS_CACHE: Optional[LaunchersParams] = None


def get_launchers_params() -> LaunchersParams:
    """Lazy singleton (env > default).

    Para invalidar (tests): :func:`reset_launchers_params_cache`.
    """
    global _LAUNCHERS_PARAMS_CACHE
    if _LAUNCHERS_PARAMS_CACHE is None:
        _LAUNCHERS_PARAMS_CACHE = load_launchers_params()
    return _LAUNCHERS_PARAMS_CACHE


def reset_launchers_params_cache() -> None:
    """Invalida el singleton."""
    global _LAUNCHERS_PARAMS_CACHE
    _LAUNCHERS_PARAMS_CACHE = None
