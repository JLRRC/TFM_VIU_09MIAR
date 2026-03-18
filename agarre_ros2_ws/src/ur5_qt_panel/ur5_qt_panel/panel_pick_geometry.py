#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_geometry.py
# Contenido: Helpers puros para la geometria vertical del pick del panel Qt.
# Uso breve: Se importa desde panel_pick_object y desde tests unitarios sin ROS.
"""Pure helpers for PICK Objeto vertical geometry."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PickHeightProfile:
    object_top_z: float
    approach_z: float
    pre_grasp_z: float
    legacy_grasp_z: float
    safe_tcp_floor_z: float
    contact_target_z: float
    grasp_z: float
    lift_z: float
    transport_z: float


def compute_pick_height_profile(
    *,
    object_center_z: float,
    object_height: float,
    table_top_z: float,
    basket_z: float,
    tcp_z_offset: float,
    pick_demo_grasp_z_offset: float,
    pick_demo_drop_z_offset: float,
    approach_clearance: float,
    min_approach_clearance: float,
    pre_margin: float,
    min_pre_clearance: float,
    insert_z: float,
    contact_down_z: float,
    table_margin: float,
    lift_clearance: float,
    transport_clearance: float,
) -> PickHeightProfile:
    """Return the vertical profile for PICK Objeto in base frame."""
    height = max(0.0, float(object_height))
    obj_top_z = float(object_center_z) + (height * 0.5 if height > 0.0 else 0.0)

    approach_clearance = max(float(approach_clearance), float(min_approach_clearance), 0.0)
    pre_clearance = max(float(pre_margin), float(min_pre_clearance), 0.0)
    contact_down_z = max(float(contact_down_z), 0.0)
    table_margin = max(float(table_margin), 0.0)

    approach_z = obj_top_z + approach_clearance
    pre_grasp_z = obj_top_z + pre_clearance
    legacy_grasp_z = max(float(table_top_z) + table_margin, obj_top_z - max(float(insert_z), 0.0))

    # The TCP target can go below the top surface, but never below the table safety plane.
    safe_tcp_floor_z = float(table_top_z) + table_margin
    contact_target_z = obj_top_z + float(tcp_z_offset) - contact_down_z
    grasp_z = max(safe_tcp_floor_z, contact_target_z)
    lift_z = max(grasp_z + max(float(lift_clearance), 0.0), approach_z)
    drop_z = float(basket_z) + float(pick_demo_drop_z_offset)
    transport_z = max(drop_z + max(float(transport_clearance), 0.0), lift_z)

    return PickHeightProfile(
        object_top_z=obj_top_z,
        approach_z=approach_z,
        pre_grasp_z=pre_grasp_z,
        legacy_grasp_z=legacy_grasp_z,
        safe_tcp_floor_z=safe_tcp_floor_z,
        contact_target_z=contact_target_z,
        grasp_z=grasp_z,
        lift_z=lift_z,
        transport_z=transport_z,
    )
