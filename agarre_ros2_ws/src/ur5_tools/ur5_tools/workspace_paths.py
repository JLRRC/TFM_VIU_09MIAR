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


# F1.3 audit (2026-05-10): default portable usando $HOME en lugar del
# path absoluto histórico /home/laboratorio/. Si la env var WS_DIR no
# está definida, se usa este path basado en el home del usuario.
_DEFAULT_WS_DIR = "~/TFM/agarre_ros2_ws"
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


# ---------------------------------------------------------------------
# F5 audit (2026-05-10): resolución de assets Gazebo Harmonic empaquetados.
# ---------------------------------------------------------------------


def resolve_ur5_gazebo_share() -> Optional[str]:
    """Devuelve la ruta a ``share/ur5_gazebo/`` instalado (None si no hay).

    Intenta ament_index primero (post colcon install) y cae a un path
    relativo en el source tree (dev sin install).
    """
    try:  # pragma: no cover - depende del install ROS
        from ament_index_python.packages import get_package_share_directory

        return get_package_share_directory("ur5_gazebo")
    except Exception:
        pass
    src_path = os.path.join(get_ws_dir(), "src", "ur5_gazebo")
    return src_path if os.path.isdir(src_path) else None


def resolve_world_file(world_name: str = "ur5_mesa_objetos") -> Optional[str]:
    """Devuelve la ruta al ``<world_name>.sdf`` empaquetado en ur5_gazebo."""
    share = resolve_ur5_gazebo_share()
    if not share:
        return None
    candidate = os.path.join(share, "worlds", f"{world_name}.sdf")
    return candidate if os.path.isfile(candidate) else None


def resolve_gazebo_models_root() -> Optional[str]:
    """Devuelve la ruta a ``share/ur5_gazebo/models/`` (None si no se instala)."""
    share = resolve_ur5_gazebo_share()
    return os.path.join(share, "models") if share else None
