#!/usr/bin/env python3
"""Constantes geométricas únicas del stack UR5+RG2.

F2 (auditoría 2026-05-10): centraliza los valores numéricos de geometría
que estaban duplicados en múltiples archivos. Cualquier consumidor debe
importar desde aquí en lugar de hardcodear el valor.

F2.1 audit (2026-05-10): los valores hardcoded permanecen como fallback
rápido (sin I/O en import time) pero pueden validarse contra
``ur5_description/config/geometry.yaml`` mediante
:func:`load_geometry_yaml` / :func:`assert_consistent_with_yaml`.

Fuente autoritativa: el YAML mencionado y el URDF.
"""
from __future__ import annotations

import os
from typing import Any, Dict, Optional, Tuple

#: Origen del frame ``base_link`` del UR5 expresado en el frame ``world``.
#:
#: Fuente autoritativa: ``src/ur5_description/urdf/ur5.urdf.xacro``
#: línea 47 — ``<origin xyz="-0.85 0 0.850" rpy="0 0 0"/>`` del joint
#: ``world_to_base_link`` (sin rotación). Cualquier conversión
#: world↔base_link en código Python debe usar esta constante.
#:
#: Fórmulas:
#:   pos_base = pos_world - BASE_LINK_IN_WORLD
#:   pos_world = pos_base + BASE_LINK_IN_WORLD
BASE_LINK_IN_WORLD: Tuple[float, float, float] = (-0.85, 0.0, 0.850)

#: Offset desde tool0 hasta el TCP semántico del gripper RG2 (m).
#: Coincide con ``rg2_contact_tcp_xyz`` (URDF línea 20).
RG2_TCP_OFFSET_FROM_TOOL0: Tuple[float, float, float] = (0.0, 0.0, 0.175)

#: Workspace nominal UR5+RG2 (radio máximo en metros desde base_link).
UR5_REACH_RADIUS_MAX_M: float = 0.85

#: Workspace nominal UR5+RG2 (radio mínimo viable).
UR5_REACH_RADIUS_MIN_M: float = 0.20

#: HOME positions (rad) — coinciden con URDF initial_value y con
#: ``tfm_orchestrator.home_initial.UR5_HOME_POSITIONS_RAD``.
import math as _math

UR5_HOME_POSITIONS_RAD: Tuple[float, float, float, float, float, float] = (
    0.0,
    -_math.pi / 2.0,
    0.0,
    -_math.pi / 2.0,
    0.0,
    0.0,
)

#: Limits del finger prismático RG2 (m).
RG2_FINGER_LOWER_M: float = 0.0
RG2_FINGER_UPPER_M: float = 0.0425


# ---------------------------------------------------------------------
# Lectura opcional del YAML declarativo (validación / regression tests)
# ---------------------------------------------------------------------


def _resolve_geometry_yaml_path(override: Optional[str] = None) -> str:
    """Devuelve la ruta al ``geometry.yaml`` instalado o en source tree."""
    if override:
        return override
    # ament_index_python may not be available in pure-pytest sin source
    # install — intentamos primero pero caemos a una resolución relativa
    # por filesystem.
    try:  # pragma: no cover - dependiente del ROS install
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(
            get_package_share_directory("ur5_description"),
            "config",
            "geometry.yaml",
        )
    except Exception:
        pass
    # Fallback: paths relativos al source tree.
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.normpath(os.path.join(here, "..", "..", "..", "ur5_description", "config", "geometry.yaml")),
        os.path.normpath(os.path.join(here, "..", "..", "..", "..", "ur5_description", "config", "geometry.yaml")),
    ]
    for cand in candidates:
        if os.path.isfile(cand):
            return cand
    return candidates[0]


def load_geometry_yaml(path: Optional[str] = None) -> Dict[str, Any]:
    """Carga ``geometry.yaml`` como ``dict`` (vacío si no se encuentra)."""
    yaml_path = _resolve_geometry_yaml_path(path)
    try:
        import yaml  # type: ignore
    except Exception:
        return {}
    try:
        with open(yaml_path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh) or {}
    except FileNotFoundError:
        return {}
    if not isinstance(data, dict):
        return {}
    return data.get("geometry", {}) if isinstance(data.get("geometry"), dict) else data


def assert_consistent_with_yaml(path: Optional[str] = None) -> None:
    """Lanza ``AssertionError`` si las constantes no coinciden con el YAML.

    Pensado para tests de regresión: usado por
    ``test_geometry_yaml_consistency.py`` para garantizar que ningún
    refactor mute las constantes Python sin actualizar el YAML (o
    viceversa).
    """
    geom = load_geometry_yaml(path)
    if not geom:
        raise AssertionError(f"geometry.yaml not found or empty (path={path!r})")

    base = geom["ur5_base_in_world"]
    yaml_base = (float(base["x"]), float(base["y"]), float(base["z"]))
    assert yaml_base == BASE_LINK_IN_WORLD, (
        f"BASE_LINK_IN_WORLD mismatch: yaml={yaml_base} const={BASE_LINK_IN_WORLD}"
    )

    tcp = geom["rg2_tcp_offset"]
    yaml_tcp = (float(tcp["x"]), float(tcp["y"]), float(tcp["z"]))
    assert yaml_tcp == RG2_TCP_OFFSET_FROM_TOOL0, (
        f"RG2_TCP_OFFSET_FROM_TOOL0 mismatch: yaml={yaml_tcp} const={RG2_TCP_OFFSET_FROM_TOOL0}"
    )

    reach = geom["ur5_reach"]
    assert float(reach["radius_max_m"]) == UR5_REACH_RADIUS_MAX_M
    assert float(reach["radius_min_m"]) == UR5_REACH_RADIUS_MIN_M

    home = geom["ur5_home_positions_rad"]
    yaml_home = (
        float(home["shoulder_pan"]),
        float(home["shoulder_lift"]),
        float(home["elbow"]),
        float(home["wrist_1"]),
        float(home["wrist_2"]),
        float(home["wrist_3"]),
    )
    assert yaml_home == UR5_HOME_POSITIONS_RAD, (
        f"UR5_HOME_POSITIONS_RAD mismatch: yaml={yaml_home} const={UR5_HOME_POSITIONS_RAD}"
    )

    basket = geom["basket_drop_world"]
    yaml_basket = (float(basket["x"]), float(basket["y"]), float(basket["z"]))
    assert yaml_basket == BASKET_DROP_WORLD, (
        f"BASKET_DROP_WORLD mismatch: yaml={yaml_basket} const={BASKET_DROP_WORLD}"
    )

    rg2 = geom["rg2_gripper"]
    assert float(rg2["finger_lower_m"]) == RG2_FINGER_LOWER_M
    assert float(rg2["finger_upper_m"]) == RG2_FINGER_UPPER_M


def world_to_base(
    pos_world: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Convierte una posición ``(x, y, z)`` de ``world`` a ``base_link``.

    Asume que el robot está fijo en el mundo (sin pose dinámica del
    base_link). Equivalente a un ``tf2`` lookup estático pero sin
    necesidad de tener un buffer activo — útil en goal builders puros.
    """
    return (
        pos_world[0] - BASE_LINK_IN_WORLD[0],
        pos_world[1] - BASE_LINK_IN_WORLD[1],
        pos_world[2] - BASE_LINK_IN_WORLD[2],
    )


def base_to_world(
    pos_base: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Convierte una posición ``(x, y, z)`` de ``base_link`` a ``world``."""
    return (
        pos_base[0] + BASE_LINK_IN_WORLD[0],
        pos_base[1] + BASE_LINK_IN_WORLD[1],
        pos_base[2] + BASE_LINK_IN_WORLD[2],
    )


#: Pose default de la cesta de drop en frame ``world`` (m).
#:
#: Punto de soltado por defecto cuando un goal ``/pick_place`` no
#: especifica ``drop_xyz_world``. El valor histórico vivía en
#: ``ur5_qt_panel/panel_config.py`` como ``BASKET_DROP``; F2-followup
#: (auditoría 2026-05-10) lo mueve a este módulo neutral para que el
#: orchestrator pueda referenciarlo sin depender del paquete del panel.
#: ``panel_config.BASKET_DROP`` se mantiene como re-export.
BASKET_DROP_WORLD: Tuple[float, float, float] = (-1.30, 0.00, 0.82)


__all__ = [
    "BASE_LINK_IN_WORLD",
    "BASKET_DROP_WORLD",
    "RG2_FINGER_LOWER_M",
    "RG2_FINGER_UPPER_M",
    "RG2_TCP_OFFSET_FROM_TOOL0",
    "UR5_HOME_POSITIONS_RAD",
    "UR5_REACH_RADIUS_MAX_M",
    "UR5_REACH_RADIUS_MIN_M",
    "assert_consistent_with_yaml",
    "base_to_world",
    "load_geometry_yaml",
    "world_to_base",
]
