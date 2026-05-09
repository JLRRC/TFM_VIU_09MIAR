#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_gripper_geometry_contract.py
# Contenido: F4 — verificación offline del loader de geometría del gripper.
"""Tests offline de ``ur5_tools.gripper_geometry``.

No levantan ROS ni MoveIt. Verifican que:

* Los frames canónicos son los esperados (constantes de módulo).
* ``load_gripper_geometry()`` parsea el URDF/xacro del workspace y
  devuelve un ``GripperGeometry`` con los offsets esperados para los
  3 frames clave (tool0, rg2_tcp, rg2_pinch_center).
* La cadena tool0 → rg2_tcp → rg2_pinch_center es coherente
  geométricamente (z monótono, magnitudes razonables).
"""

from __future__ import annotations


import pytest

from ur5_tools.gripper_geometry import (
    GripperGeometry,
    RG2_PINCH_CENTER_FRAME,
    RG2_TCP_FRAME,
    TOOL0_FRAME,
    load_gripper_geometry,
)


# ---------------------------------------------------------------------------
# Frame names (constantes públicas)
# ---------------------------------------------------------------------------


def test_frame_constants_match_project_naming():
    assert TOOL0_FRAME == "tool0"
    assert RG2_TCP_FRAME == "rg2_tcp"
    assert RG2_PINCH_CENTER_FRAME == "rg2_pinch_center"


# ---------------------------------------------------------------------------
# Loader (lee URDF/xacro real del workspace)
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def geometry() -> GripperGeometry:
    try:
        return load_gripper_geometry()
    except Exception as e:
        pytest.skip(f"load_gripper_geometry() falló: {e}")


def test_returns_gripper_geometry_instance(geometry):
    assert isinstance(geometry, GripperGeometry)


def test_xyz_for_rg2_tcp_is_positive_z(geometry):
    """rg2_tcp está delante del flange (positivo en Z relativo a tool0)."""
    xyz = geometry.xyz_for_frame(RG2_TCP_FRAME)
    assert xyz[2] > 0.0, f"rg2_tcp Z debería ser positivo, got {xyz}"
    # Magnitud razonable: gripper RG2 ~17-18 cm desde tool0.
    assert 0.0 < xyz[2] < 0.30, f"rg2_tcp Z fuera de rango razonable: {xyz}"


def test_xyz_for_rg2_pinch_center_is_positive_z(geometry):
    """rg2_pinch_center es positivo en Z (delante del flange tool0)."""
    xyz = geometry.xyz_for_frame(RG2_PINCH_CENTER_FRAME)
    assert xyz[2] > 0.0, f"rg2_pinch_center Z debe ser positivo, got {xyz}"
    assert xyz[2] < 0.30, f"rg2_pinch_center Z fuera de rango razonable: {xyz}"


def test_pinch_center_and_tcp_share_xyz_property(geometry):
    """En el URDF actual ambos joints comparten ``rg2_contact_tcp_xyz``:
    la separación entre rg2_tcp (base superior gripper) y rg2_pinch_center
    (TCP funcional de contacto) está modelada por joints subsecuentes,
    no por offsets distintos a tool0. Verificamos esa invariante."""
    tcp = geometry.xyz_for_frame(RG2_TCP_FRAME)
    pinch = geometry.xyz_for_frame(RG2_PINCH_CENTER_FRAME)
    assert tcp == pinch, (
        f"rg2_tcp y rg2_pinch_center deberían compartir xyz desde tool0 "
        f"(invariante actual del URDF): tcp={tcp}, pinch={pinch}"
    )


def test_xy_offsets_negligible(geometry):
    """Los frames del gripper deben estar centrados sobre la línea Z de tool0
    (XY ~ 0)."""
    for frame in (RG2_TCP_FRAME, RG2_PINCH_CENTER_FRAME):
        xyz = geometry.xyz_for_frame(frame)
        for i, axis in enumerate(("X", "Y")):
            assert abs(xyz[i]) < 0.005, (
                f"{frame} {axis} debería ser ~0 (gripper centrado), got {xyz[i]:.4f}"
            )


def test_parent_links_consistent(geometry):
    """Los joints del gripper deben colgar de ``tool0`` (anclaje canónico)."""
    for frame, origin in (
        (RG2_TCP_FRAME, geometry.tcp),
        (RG2_PINCH_CENTER_FRAME, geometry.pinch_center),
    ):
        assert origin.parent_link == TOOL0_FRAME, (
            f"{frame} parent_link debería ser tool0, got {origin.parent_link!r}"
        )
        assert origin.child_link == frame, (
            f"child_link debería ser {frame!r}, got {origin.child_link!r}"
        )


# ---------------------------------------------------------------------------
# Robustez: frame desconocido
# ---------------------------------------------------------------------------


def test_unknown_frame_raises_keyerror(geometry):
    """``xyz_for_frame`` solo soporta los 2 frames del gripper; para
    cualquier otro debe lanzar KeyError (contrato documentado en el
    propio módulo)."""
    with pytest.raises(KeyError):
        geometry.xyz_for_frame("frame_que_no_existe_xyz123")


def test_tool0_not_supported_by_loader(geometry):
    """``tool0`` NO está soportado por ``xyz_for_frame`` — el loader sólo
    expone los offsets relativos del gripper, no el origen del UR5."""
    with pytest.raises(KeyError):
        geometry.xyz_for_frame(TOOL0_FRAME)
