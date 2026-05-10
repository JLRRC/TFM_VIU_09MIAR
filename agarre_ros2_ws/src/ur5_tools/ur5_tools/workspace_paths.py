#!/usr/bin/env python3
"""F2 audit (2026-05-10): resolución centralizada de paths/env del workspace.

Punto único de verdad para variables de entorno relacionadas con
infraestructura (WS_DIR, GZ_PARTITION, GZ_IP, VISION_DIR). Todos los
nodos de ur5_tools deben usar estas funciones en lugar de leer
``os.environ`` directamente. Esto elimina drift entre defaults y
permite escribir tests deterministas.

Equivalente a ``panel_env.py`` para el lado del panel.
"""
from __future__ import annotations

import os
from pathlib import Path
from typing import Optional


# Default histórico (memoria de proyecto). Si la env var WS_DIR no está
# definida, se usa este path absoluto. Mantenido aquí para que sea la
# única fuente de verdad.
_DEFAULT_WS_DIR = "/home/laboratorio/TFM/agarre_ros2_ws"
_DEFAULT_VISION_DIR = "~/TFM/agarre_inteligente"


def get_ws_dir(default: Optional[str] = None) -> str:
    """Devuelve la ruta al workspace TFM (env var WS_DIR > default).

    Si ``WS_DIR`` está definido en el entorno, se devuelve esa ruta
    expandida. En caso contrario, se devuelve ``default`` o, si éste es
    None, el default histórico ``/home/laboratorio/TFM/agarre_ros2_ws``.
    """
    raw = os.environ.get("WS_DIR")
    if raw:
        return os.path.expanduser(raw)
    if default is None:
        default = _DEFAULT_WS_DIR
    return os.path.expanduser(default)


def get_ws_dir_path(default: Optional[str] = None) -> Path:
    """Versión ``Path`` de :func:`get_ws_dir`."""
    return Path(get_ws_dir(default=default))


def get_gz_partition(default: str = "") -> str:
    """Devuelve la partición Gazebo activa (variable GZ_PARTITION)."""
    return os.environ.get("GZ_PARTITION", default).strip()


def get_gz_ip(default: str = "") -> str:
    """Devuelve la IP de transporte Gazebo (GZ_IP)."""
    return os.environ.get("GZ_IP", default).strip()


def get_vision_dir(default: Optional[str] = None) -> str:
    """Devuelve la ruta al repo TFM de visión (VISION_DIR)."""
    raw = os.environ.get("VISION_DIR")
    if raw:
        return os.path.expanduser(raw)
    if default is None:
        default = _DEFAULT_VISION_DIR
    return os.path.expanduser(default)


def get_strict_physics_mode(default: bool = False) -> bool:
    """STRICT_PHYSICS_MODE como bool permisivo."""
    raw = os.environ.get("STRICT_PHYSICS_MODE")
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


def get_strict_self_collision(default: bool = False) -> bool:
    """STRICT_SELF_COLLISION como bool permisivo."""
    raw = os.environ.get("STRICT_SELF_COLLISION")
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")
