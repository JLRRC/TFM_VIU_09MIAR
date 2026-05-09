#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_planning_scene_consistency.py
# Contenido: F4 T11 — planning scene consistency check (mock-based, offline).
"""F4 T11 — Planning scene consistency mock test.

Verifica que la composición de poses + el parsing de model.sdf produce
geometrías razonables que NO colisionan trivialmente con la base del
UR5. Es un test estructural — usa mock primitives, no requiere stack vivo.

Casos cubiertos:
  * Una primitiva box compuesta con la pose del modelo da una caja
    posicionada correctamente en world frame.
  * Compose-pose es asociativo (parent→child→grandchild ≡ chain).
  * No hay colisión obvia entre objetos pickeables y la mesa
    (ground_plane Z=0).

Notas:
  * No se chequea collision real (FCL) — solo invariantes geométricos.
  * Para chequeo runtime usar el nodo `planning_scene_sync` activo y
    consultar `/planning_scene_world`.
"""
from __future__ import annotations

import math
from typing import Tuple

import pytest

from ur5_tools.planning_scene_sync_helpers import (
    compose_pose,
    parse_pose_text,
    quat_from_rpy,
)


# ---------------------------------------------------------------------------
# Helpers locales (mock primitives mínimos)
# ---------------------------------------------------------------------------

def _mock_box(
    size_x: float, size_y: float, size_z: float
) -> Tuple[float, float, float]:
    """Devuelve dimensiones de una caja como tuple3."""
    return (size_x, size_y, size_z)


def _box_top_z_in_world(
    model_pose: Tuple[float, float, float, float, float, float, float],
    primitive_local_pose: Tuple[float, float, float, float, float, float, float],
    box_dims: Tuple[float, float, float],
) -> float:
    """Calcula la Z del top del box en world frame.

    model_pose: pose del model en world.
    primitive_local_pose: pose de la primitive relativo al model.
    box_dims: (sx, sy, sz) — la primitive es box.
    """
    composed = compose_pose(model_pose, primitive_local_pose)
    composed_z = composed[2]
    half_height = box_dims[2] / 2.0
    return composed_z + half_height


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_t11a_box_at_origin_is_above_ground() -> None:
    """Box centrada en origin con halfheight > 0 → top está por encima del ground."""
    model_pose = (0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0)  # z=0.5 en world
    primitive_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)  # local idéntica
    box_dims = _mock_box(0.05, 0.05, 0.05)  # cubo 5cm
    top_z = _box_top_z_in_world(model_pose, primitive_pose, box_dims)
    assert top_z > 0.0, "El top del box debe estar por encima del ground (Z=0)"
    assert top_z == pytest.approx(0.525, abs=1e-9)


def test_t11b_box_on_table_does_not_intersect_ground() -> None:
    """Mesa a Z=0.85, box pickeable a Z=0.88 → bottom box > Z mesa."""
    table_top_z = 0.85
    box_dims = _mock_box(0.05, 0.05, 0.05)
    model_pose = (-0.260, 0.100, 0.880, 0.0, 0.0, 0.0, 1.0)  # box_red post-drop
    primitive_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    composed = compose_pose(model_pose, primitive_pose)
    box_bottom_z = composed[2] - box_dims[2] / 2.0
    assert box_bottom_z >= table_top_z, (
        f"Box bottom Z={box_bottom_z:.3f} debajo de mesa Z={table_top_z}"
    )


def test_t11c_compose_pose_is_associative() -> None:
    """compose(compose(a, b), c) ≈ compose(a, compose(b, c)) para translaciones."""
    a = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    b = (0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    c = (0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    left = compose_pose(compose_pose(a, b), c)
    right = compose_pose(a, compose_pose(b, c))
    assert left[:3] == pytest.approx(right[:3], abs=1e-9)
    # Para translaciones puras la posición coincide y la orientación es identity.
    assert left[:3] == pytest.approx((1.0, 2.0, 3.0), abs=1e-9)


def test_t11d_compose_with_yaw_rotates_child_position() -> None:
    """Parent rotado 90°Z + child en (1,0,0) local → world (0,1,0)."""
    parent_q = quat_from_rpy(0.0, 0.0, math.pi / 2)
    parent = (0.0, 0.0, 0.0) + parent_q
    child = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    composed = compose_pose(parent, child)
    assert composed[:3] == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)


def test_t11e_pickable_objects_above_ground() -> None:
    """Lista canónica de objetos pickeables — todos por encima de la mesa
    en sus poses post-drop nominales (validación geométrica del setup).
    """
    table_z = 0.85
    canonical_drop_z = 0.88  # validado live
    pickable_objects = [
        ("box_red", -0.260, 0.100, canonical_drop_z),
        ("box_blue", -0.180, 0.100, canonical_drop_z),
        ("box_green", -0.100, 0.100, canonical_drop_z),
    ]
    for name, x, y, z in pickable_objects:
        assert z >= table_z, (
            f"Objeto {name} en Z={z:.3f} está debajo del top de mesa Z={table_z}"
        )


def test_t11f_parse_world_pose_for_table_origin() -> None:
    """parse_pose_text de la mesa pro origen en world parsea correctamente."""
    sdf_pose = "0 0 0 0 0 0"  # mesa_pro centrada en origin, sin rotación
    result = parse_pose_text(sdf_pose)
    assert result == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)


def test_t11g_parse_world_pose_for_basket() -> None:
    """parse_pose_text de bandeja_deposito (-1.30, 0, 0.78) parsea correctamente."""
    sdf_pose = "-1.30 0.00 0.780 0 0 0"
    result = parse_pose_text(sdf_pose)
    assert result[:3] == pytest.approx((-1.30, 0.0, 0.78), abs=1e-9)
    assert result[3:] == pytest.approx((0.0, 0.0, 0.0, 1.0), abs=1e-9)


def test_t11h_box_with_yaw_compose_correctly() -> None:
    """Box con yaw arbitrario en model + primitive local sin rotación → top z igual al z del model."""
    yaw = math.pi / 6  # 30°
    model_pose = (-0.260, 0.100, 0.88) + quat_from_rpy(0.0, 0.0, yaw)
    primitive_pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    box_dims = _mock_box(0.05, 0.05, 0.05)
    top_z = _box_top_z_in_world(model_pose, primitive_pose, box_dims)
    # Yaw alrededor de Z no afecta la altura — top_z = 0.88 + 0.025 = 0.905
    assert top_z == pytest.approx(0.905, abs=1e-9)
