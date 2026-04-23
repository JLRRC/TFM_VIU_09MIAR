#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_pick_geometry.py
# Contenido: Tests unitarios de la geometria vertical del pick.
# Uso breve: Valida que el TCP no quede artificialmente alto sobre el objeto.
"""Unit tests for PICK Objeto geometry helpers."""
from __future__ import annotations

import pytest

from ur5_qt_panel.panel_pick_geometry import compute_pick_height_profile


def test_pick_height_profile_matches_reference_contact_height() -> None:
    profile = compute_pick_height_profile(
        object_center_z=0.100,
        object_height=0.050,
        table_top_z=0.075,
        basket_z=0.300,
        tcp_z_offset=0.100,
        pick_demo_grasp_z_offset=0.020,
        pick_demo_drop_z_offset=0.050,
        approach_clearance=0.20,
        min_approach_clearance=0.30,
        pre_margin=0.04,
        min_pre_clearance=0.15,
        insert_z=0.015,
        contact_down_z=0.070,
        table_margin=0.015,
        lift_clearance=0.20,
        transport_clearance=0.20,
    )

    assert profile.object_top_z == pytest.approx(0.125)
    assert profile.safe_tcp_floor_z == pytest.approx(0.090)
    assert profile.contact_target_z == pytest.approx(0.155)
    assert profile.grasp_z == pytest.approx(0.155)


def test_pick_height_profile_never_goes_below_table_floor() -> None:
    profile = compute_pick_height_profile(
        object_center_z=0.090,
        object_height=0.020,
        table_top_z=0.075,
        basket_z=0.300,
        tcp_z_offset=0.000,
        pick_demo_grasp_z_offset=0.020,
        pick_demo_drop_z_offset=0.050,
        approach_clearance=0.10,
        min_approach_clearance=0.10,
        pre_margin=0.04,
        min_pre_clearance=0.04,
        insert_z=0.015,
        contact_down_z=0.050,
        table_margin=0.015,
        lift_clearance=0.20,
        transport_clearance=0.20,
    )

    assert profile.contact_target_z == pytest.approx(0.050)
    assert profile.safe_tcp_floor_z == pytest.approx(0.090)
    assert profile.grasp_z == pytest.approx(0.090)
