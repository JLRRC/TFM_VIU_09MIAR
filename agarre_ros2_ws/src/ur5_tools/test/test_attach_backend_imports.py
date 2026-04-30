#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_attach_backend_imports.py
# Contenido: F4 — smoke imports de los módulos attach_*.
"""Smoke tests de importación de los módulos ``attach_*.py``.

El backend de attach se compone de 7 módulos (mixins extraídos en el
refactor F3 inicial — ver memoria del 2026-04-27). Si cualquiera de
ellos rompe su API pública o introduce un import circular, el nodo
``gripper_attach_backend`` deja de cargar.

Este test se asegura de:

* Cada módulo se importa sin error.
* ``attach_math`` exporta sus operaciones cuaterniónicas y matriciales
  puras (sin dependencia ROS — testable directamente).
* La función entry-point ``gripper_attach_backend.main`` existe (no
  la invocamos, sólo verificamos la firma).
"""

from __future__ import annotations

import importlib

import pytest


ATTACH_MODULES = [
    "ur5_tools.attach_math",
    "ur5_tools.attach_anchor",
    "ur5_tools.attach_pose_lookup",
    "ur5_tools.attach_pose_sub",
    "ur5_tools.attach_set_pose",
    "ur5_tools.attach_gz_cli",
    "ur5_tools.attach_demo_transport",
]


_ROS_MARKERS = ("rclpy", "_msgs", "tf2_", "rcl_interfaces", "ament_index", "rosidl_runtime")


@pytest.mark.parametrize("modname", ATTACH_MODULES)
def test_attach_module_importable(modname):
    """Cada attach_*.py se importa sin error.

    Si depende de paquetes ROS 2 (msgs/rclpy/tf2_*) y el entorno no los
    provee, skipea con razón explícita.
    """
    try:
        importlib.import_module(modname)
    except ImportError as e:
        msg = str(e)
        if any(marker in msg for marker in _ROS_MARKERS):
            pytest.skip(f"{modname}: depende de paquetes ROS ausentes ({e})")
        raise


# ---------------------------------------------------------------------------
# attach_math: módulo puro sin ROS — verificar API
# ---------------------------------------------------------------------------


def test_attach_math_exports_pure_helpers():
    """attach_math.py debe exponer las primitivas de quaternion + matriz."""
    import ur5_tools.attach_math as m
    expected = (
        "_quat_normalize",
        "_quat_inverse",
        "_quat_multiply",
        "_rotate_vector",
        "_matmul3",
        "_matvec3",
        "_dh_transform",
    )
    for sym in expected:
        assert hasattr(m, sym), f"attach_math debe exportar {sym!r}"
        assert callable(getattr(m, sym)), f"{sym} debe ser callable"


def test_quat_normalize_identity_quaternion():
    """quat_normalize de la identidad ROS (0,0,0,1) la deja igual.

    Convención (x, y, z, w) — la usada por geometry_msgs.Quaternion y
    por las primitivas de attach_math (ver el unpacking en
    ``_quat_inverse``: ``x, y, z, w = q``).
    """
    from ur5_tools.attach_math import _quat_normalize
    q = (0.0, 0.0, 0.0, 1.0)
    qn = _quat_normalize(q)
    assert qn == pytest.approx(q), f"identity quat normalized != self: {qn}"


def test_quat_normalize_scales():
    """quat_normalize escala un cuaternion no unitario a norma 1."""
    from ur5_tools.attach_math import _quat_normalize
    q = (0.0, 0.0, 0.0, 2.0)  # convención (x,y,z,w): w=2 doble identidad
    qn = _quat_normalize(q)
    assert qn == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quat_inverse_of_identity_is_identity():
    """Inverso de la identidad ROS (0,0,0,1) es ella misma."""
    from ur5_tools.attach_math import _quat_inverse
    inv = _quat_inverse((0.0, 0.0, 0.0, 1.0))
    assert inv == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quat_inverse_negates_xyz():
    """Inverso de un cuaternion unitario cualquiera niega (x,y,z) y mantiene w."""
    from ur5_tools.attach_math import _quat_inverse
    # 90° around Z: (0, 0, sin45, cos45)
    s = 0.7071067811865476
    inv = _quat_inverse((0.0, 0.0, s, s))
    assert inv[0] == pytest.approx(0.0)
    assert inv[1] == pytest.approx(0.0)
    assert inv[2] == pytest.approx(-s)
    assert inv[3] == pytest.approx(s)


# ---------------------------------------------------------------------------
# gripper_attach_backend: entry-point principal
# ---------------------------------------------------------------------------


def test_gripper_attach_backend_has_main():
    """El entry-point del nodo expone ``main()`` callable."""
    try:
        import ur5_tools.gripper_attach_backend as backend
    except ImportError as e:
        if "rclpy" in str(e) or "rcl" in str(e):
            pytest.skip(f"gripper_attach_backend depende de rclpy ({e})")
        raise
    assert hasattr(backend, "main"), "gripper_attach_backend debe exponer main()"
    assert callable(backend.main)
