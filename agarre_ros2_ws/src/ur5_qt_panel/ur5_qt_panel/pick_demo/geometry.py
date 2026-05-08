#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/geometry.py
# Contenido: F3 — helpers geométricos puros extraídos de panel_pick_demo.run_pick_demo.
"""Funciones geométricas puras del flujo pick_demo.

Extraídas de las closures internas de ``run_pick_demo``:

* ``object_top_pose(pose, object_height_m)``: top center de un objeto
  asumido cuboide/cilíndrico simétrico — eleva Z por la mitad de la
  altura.
* ``dynamic_pre_close_reference(obj_base, grasp_z, extra_z_m)``: pose
  pre-CLOSE manual referenciada al objeto + offset.
* ``apply_local_offset_to_fk(fk_pos, fk_rot, local_offset)``:
  combina FK base + rotación + offset local del TCP, con flip xy
  (convención del frame del modelo). Pura matricial.

Cero dependencia ROS / TF / panel — todas las entradas son tuples3
o matrices/listas de floats.
"""

from __future__ import annotations

import math
from typing import Any, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]


def _as_tuple3(v: Any) -> Optional[Vec3]:
    if v is None:
        return None
    try:
        return (float(v[0]), float(v[1]), float(v[2]))
    except (TypeError, ValueError, IndexError):
        return None


def vec_dist3(a: Vec3, b: Vec3) -> float:
    """Distancia euclídea entre dos puntos 3D."""
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
        + (float(a[2]) - float(b[2])) ** 2
    )


def object_top_pose(
    pose: Optional[Vec3],
    object_height_m: float,
) -> Optional[Vec3]:
    """Devuelve el top center del objeto asumiendo geometría simétrica.

    Top center = pose centro + (0, 0, height/2). Útil para ATTACH/LIFT
    cuando el grasp se referencia al top en lugar del centro.

    Si ``pose`` es None, devuelve None.
    """
    p = _as_tuple3(pose)
    if p is None:
        return None
    return (p[0], p[1], p[2] + 0.5 * float(object_height_m))


def dynamic_pre_close_reference(
    obj_base: Optional[Vec3],
    grasp_z: float,
    extra_z_m: float,
) -> Optional[Vec3]:
    """Pose pre-CLOSE referenciada al objeto + ``grasp_z`` + ``extra_z_m``.

    Usado por la rama ``manual_reference`` de APPROACH para subir un
    margen extra sobre el grasp (típicamente 0.10m).

    Si ``obj_base`` es None, devuelve None.
    ``extra_z_m`` se clampa a >= 0.
    """
    o = _as_tuple3(obj_base)
    if o is None:
        return None
    return (
        o[0],
        o[1],
        o[2] + float(grasp_z) + max(0.0, float(extra_z_m)),
    )


def apply_local_offset_to_fk(
    fk_pos: Sequence[float],
    fk_rot: Any,  # 3x3 array-like (numpy array or list of lists)
    local_offset: Sequence[float],
    *,
    flip_xy: bool = True,
) -> Vec3:
    """Aplica un offset local al frame del FK del UR5 y devuelve pose base.

    Calcula source_model = fk_pos + fk_rot @ local_offset, opcionalmente
    aplicando flip de signo en xy (convención del modelo DH usado en el
    panel donde la base está rotada 180º respecto al base_link ROS).

    Inputs:
      * ``fk_pos``: posición tool0 en frame del modelo (tuple/list de 3).
      * ``fk_rot``: rotación 3x3 del tool0 en el modelo (indexado [i][j]).
      * ``local_offset``: offset tool0 → source_frame en frame local.
      * ``flip_xy``: si True (default) niega xy del resultado para
        convertir frame del modelo a base_link ROS.

    Devuelve tuple3 con la pose en base_link.
    """
    sm0 = (
        float(fk_pos[0])
        + float(fk_rot[0][0]) * float(local_offset[0])
        + float(fk_rot[0][1]) * float(local_offset[1])
        + float(fk_rot[0][2]) * float(local_offset[2])
    )
    sm1 = (
        float(fk_pos[1])
        + float(fk_rot[1][0]) * float(local_offset[0])
        + float(fk_rot[1][1]) * float(local_offset[1])
        + float(fk_rot[1][2]) * float(local_offset[2])
    )
    sm2 = (
        float(fk_pos[2])
        + float(fk_rot[2][0]) * float(local_offset[0])
        + float(fk_rot[2][1]) * float(local_offset[1])
        + float(fk_rot[2][2]) * float(local_offset[2])
    )
    if flip_xy:
        return (-sm0, -sm1, sm2)
    return (sm0, sm1, sm2)
