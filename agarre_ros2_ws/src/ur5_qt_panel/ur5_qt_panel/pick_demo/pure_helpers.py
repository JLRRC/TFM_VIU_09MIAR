#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/pure_helpers.py
# Contenido: F3 step 1 — extracción de closures puras de panel_pick_demo.run_pick_demo.
"""Funciones puras extraídas del closure ``run_pick_demo``.

Primer paso del split F3 de ``panel_pick_demo.py`` (10.7 kLOC, 1 def
top-level con 107 closures anidadas). Esta primera extracción cubre
**5 funciones verdaderamente puras** (sin ``nonlocal``, sin acceso a
estado del scope, sin side-effects) que aparecen en ``run_pick_demo``
y son trivialmente reutilizables sin pasar contexto.

Las funciones aquí:

* No tocan el panel ni el estado del ciclo.
* No leen env vars ni tocan I/O.
* No conocen ROS; testean en pytest puro.

Lo que NO se mueve aquí (intencional):

* ``_tuple3``, ``_fmt_vec``, ``_fmt_scalar`` — son alias triviales de
  funciones ya importadas (``_pick_demo_tuple3``, ``fmt_vec3``,
  ``_pick_demo_fmt_scalar``). Cambiarlos requiere reescribir muchos
  sitios; queda para F3 step 2.
* ``_iso_now`` y ``_json_safe`` se mueven aquí porque su firma es
  estable y los demás sites del módulo pueden importarlos.
"""

from __future__ import annotations

import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional, Tuple


def iso_now() -> str:
    """Timestamp ISO 8601 con ms en zona local del sistema.

    Usado para etiquetar líneas de trazas. Determinista en tipo
    (str), pero el valor depende del reloj del sistema.
    """
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def json_safe(value: Any) -> Any:
    """Convertir cualquier valor a un objeto JSON-serializable.

    Mapeo:
      - None / str / int / float / bool → tal cual
      - Path → str(Path)
      - list / tuple → list de valores convertidos
      - dict → dict con claves str y valores convertidos
      - resto: intenta float(); si falla, str()
    """
    if value is None:
        return None
    if isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    try:
        return float(value)
    except Exception:
        return str(value)


def vector_minus(
    a: Optional[Tuple[float, float, float]],
    b: Optional[Tuple[float, float, float]],
) -> Optional[Tuple[float, float, float]]:
    """Resta vectorial componente a componente. Devuelve None si algún input es None.

    Las entradas se asumen ya como tuples3 (o convertibles). Si alguna no se
    puede convertir, devuelve None.
    """
    if a is None or b is None:
        return None
    try:
        return (
            float(a[0]) - float(b[0]),
            float(a[1]) - float(b[1]),
            float(a[2]) - float(b[2]),
        )
    except (TypeError, ValueError, IndexError):
        return None


def vec_norm(vec: Optional[Tuple[float, float, float]]) -> Optional[float]:
    """Norma euclídea de un vector 3D. None si vec es None o inválido."""
    if vec is None:
        return None
    try:
        x, y, z = float(vec[0]), float(vec[1]), float(vec[2])
    except (TypeError, ValueError, IndexError):
        return None
    return math.sqrt(x * x + y * y + z * z)


def z_delta(
    a: Optional[Tuple[float, float, float]],
    b: Optional[Tuple[float, float, float]],
) -> Optional[float]:
    """Diferencia en Z entre dos vectores 3D. None si alguno es inválido."""
    if a is None or b is None:
        return None
    try:
        return float(a[2]) - float(b[2])
    except (TypeError, ValueError, IndexError):
        return None
