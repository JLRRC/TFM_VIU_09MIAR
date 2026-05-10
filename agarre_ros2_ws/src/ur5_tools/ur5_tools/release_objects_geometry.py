#!/usr/bin/env python3
"""F8 (auditoría 2026-05-10): helpers puros de geometría para release_objects.

Lógica matemática y de parseo SDF extraída del LifecycleNode
``ReleaseObjectsService``. Cada función es 100% pura — no usa rclpy,
no toca filesystem (salvo ``parse_table_geometry_from_sdf`` que abre
un path ya validado por el caller). Testeable sin ROS.

Convenciones:
  * Las poses se devuelven como tuples ``(x, y, z, qx, qy, qz, qw)``.
    El nodo se encarga de convertirlas a ``geometry_msgs/Pose`` cuando
    necesita.
  * ``table_geometry`` es la tupla
    ``(center_x, center_y, size_x, size_y, table_z)`` donde ``table_z``
    es la cota Z de la cara superior.
"""
from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from typing import Optional, Tuple

PoseTuple7 = Tuple[float, float, float, float, float, float, float]
TableGeometry = Tuple[float, float, float, float, float]


def quat_from_rpy(
    roll: float,
    pitch: float,
    yaw: float,
) -> Tuple[float, float, float, float]:
    """Convierte ángulos RPY (radianes, ZYX intrínseco) a cuaternión XYZW."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def pose_tuple_from_text(text: str) -> PoseTuple7:
    """Parsea un texto SDF ``"x y z roll pitch yaw"`` a tuple7.

    Si faltan componentes se rellena con 0.0. Si sobran se ignoran.
    Devuelve ``(x, y, z, qx, qy, qz, qw)``.
    """
    parts = [p for p in text.strip().split() if p]
    values = [float(p) for p in parts[:6]]
    while len(values) < 6:
        values.append(0.0)
    x, y, z, roll, pitch, yaw = values
    qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
    return (x, y, z, qx, qy, qz, qw)


def is_pose_on_table(
    pose_xyz: Tuple[float, float, float],
    table_geometry: TableGeometry,
    *,
    margin_xy: float = 0.09,
    max_dz_above: float = 0.08,
) -> bool:
    """Verifica si una pose ``(x, y, z)`` cae dentro del rectángulo de la mesa.

    Reglas:
      * |x - center_x| <= size_x/2 + margin_xy
      * |y - center_y| <= size_y/2 + margin_xy
      * 0.0 <= (z - table_z) <= max_dz_above

    Los defaults reproducen el comportamiento histórico del nodo
    (margin 9 cm, holgura vertical 8 cm sobre la cara superior).
    """
    center_x, center_y, size_x, size_y, table_z = table_geometry
    x, y, z = pose_xyz
    half_x = (size_x / 2.0) + margin_xy
    half_y = (size_y / 2.0) + margin_xy
    dz = z - table_z
    return (
        abs(x - center_x) <= half_x
        and abs(y - center_y) <= half_y
        and 0.0 <= dz <= max_dz_above
    )


def parse_table_geometry_from_sdf(
    world_sdf_path: str,
    *,
    table_model_name: str = "mesa_pro",
    collision_name: str = "tablero_collision",
    default_size: Tuple[float, float, float] = (0.768, 0.80, 0.05),
) -> Optional[TableGeometry]:
    """Parsea ``mesa_pro/tablero_collision`` de un world SDF.

    Devuelve ``(center_x, center_y, size_x, size_y, table_z)`` o ``None``
    si el archivo no existe / no contiene la mesa esperada / parser falla.

    El nodo añade caching + logging encima — esta función NO los hace
    para mantenerse pura.
    """
    if not world_sdf_path or not os.path.exists(world_sdf_path):
        return None
    try:
        tree = ET.parse(world_sdf_path)
        root = tree.getroot()
        world_elem = root.find("world")
        if world_elem is None:
            return None
        table_model = None
        for model in world_elem.findall("model"):
            if model.get("name", "") == table_model_name:
                table_model = model
                break
        if table_model is None:
            return None
        model_pose = pose_tuple_from_text(
            table_model.findtext("pose", default="0 0 0 0 0 0")
        )
        link = table_model.find("link")
        if link is None:
            return None
        coll_xpath = f"collision[@name='{collision_name}']"
        collision = link.find(coll_xpath)
        if collision is None:
            collision = link.find("collision")
        if collision is None:
            return None
        coll_pose = pose_tuple_from_text(
            collision.findtext("pose", default="0 0 0 0 0 0")
        )
        size_text = collision.findtext(
            "geometry/box/size",
            default=" ".join(str(v) for v in default_size),
        )
        vals = [float(v) for v in size_text.split()]
        if len(vals) < 3:
            return None
        size_x, size_y, size_z = vals[:3]
        # model_pose / coll_pose son tuples (x,y,z,qx,qy,qz,qw)
        center_x = float(model_pose[0] + coll_pose[0])
        center_y = float(model_pose[1] + coll_pose[1])
        table_z = float(model_pose[2] + coll_pose[2] + (size_z / 2.0))
        return (center_x, center_y, float(size_x), float(size_y), table_z)
    except Exception:
        return None


__all__ = [
    "PoseTuple7",
    "TableGeometry",
    "quat_from_rpy",
    "pose_tuple_from_text",
    "is_pose_on_table",
    "parse_table_geometry_from_sdf",
]
