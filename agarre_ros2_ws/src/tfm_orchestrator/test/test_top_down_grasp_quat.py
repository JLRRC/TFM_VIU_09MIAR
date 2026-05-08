#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_top_down_grasp_quat.py
# Contenido: F1.8 — unit tests del helper compute_top_down_grasp_quat.
"""F1.8 — Unit tests para `compute_top_down_grasp_quat` en phase_dispatch.

El helper resuelve el bug TCP-orientation contract: el orchestrator estaba
pasando ``object.orientation`` como TCP-orientation directo, causando
OMPL FAILURE consistente. El nuevo helper extrae yaw del objeto y produce
una TCP-orientation top-down (Z apuntando hacia abajo) con yaw correcto.

Tests:
  * yaw=0 → quat canónico 180°-around-X (1,0,0,0)
  * yaw=π/2, π, -π/2 → quats esperados
  * objetos planos sobre mesa (qz pequeño, qw≈1) → yaw extraído correcto
  * quat normalizado output (|q|=1)
"""
from __future__ import annotations

import math

import pytest

from tfm_orchestrator.phase_dispatch import compute_top_down_grasp_quat


def _quat_norm(q: tuple[float, float, float, float]) -> float:
    return math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)


def test_zero_yaw_gives_180x_quat() -> None:
    """yaw=0 (object identity) → TCP rota 180° alrededor de X: (1,0,0,0)."""
    q = compute_top_down_grasp_quat((0.0, 0.0, 0.0, 1.0))
    assert q == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-9)


def test_pi_half_yaw_gives_diagonal_quat() -> None:
    """yaw=π/2 → (cos(π/4), sin(π/4), 0, 0) = (√2/2, √2/2, 0, 0)."""
    # Object at yaw=π/2 in world: quat = (0, 0, sin(π/4), cos(π/4))
    obj_q = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    q = compute_top_down_grasp_quat(obj_q)
    expected = (math.sqrt(2) / 2, math.sqrt(2) / 2, 0.0, 0.0)
    assert q == pytest.approx(expected, abs=1e-9)


def test_pi_yaw_gives_180y_quat() -> None:
    """yaw=π → (cos(π/2), sin(π/2), 0, 0) = (0, 1, 0, 0) = 180° around Y."""
    obj_q = (0.0, 0.0, 1.0, 0.0)  # yaw=π in world
    q = compute_top_down_grasp_quat(obj_q)
    assert q == pytest.approx((0.0, 1.0, 0.0, 0.0), abs=1e-9)


def test_negative_pi_half_yaw() -> None:
    """yaw=-π/2 → (cos(-π/4), sin(-π/4), 0, 0) = (√2/2, -√2/2, 0, 0)."""
    obj_q = (0.0, 0.0, math.sin(-math.pi / 4), math.cos(-math.pi / 4))
    q = compute_top_down_grasp_quat(obj_q)
    expected = (math.sqrt(2) / 2, -math.sqrt(2) / 2, 0.0, 0.0)
    assert q == pytest.approx(expected, abs=1e-9)


def test_output_is_unit_quaternion() -> None:
    """Para cualquier object orientation, |q_tcp| ≈ 1."""
    test_cases = [
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 0.0962, 0.9954),  # box_red post-drop (live data)
        (0.0, 0.0, -0.1715, 0.9852),  # box_blue post-drop
        (0.0, 0.0, 0.6330, 0.7742),  # box_blue alternativa
        (-0.1532, 0.6903, 0.1532, 0.6903),  # rotación arbitraria
    ]
    for obj_q in test_cases:
        q = compute_top_down_grasp_quat(obj_q)
        assert _quat_norm(q) == pytest.approx(1.0, abs=1e-9), f"obj={obj_q} → q={q}"


def test_z_component_is_always_zero() -> None:
    """El TCP downward quat siempre tiene qz=0 (no rota alrededor de world Z final).

    El yaw está embebido en qx/qy. Esta es la garantía estructural del
    fix: TCP queda con Z=down + body-yaw, sin componente Z extra.
    """
    test_cases = [
        (0.0, 0.0, 0.5, math.sqrt(0.75)),  # yaw arbitrario
        (0.0, 0.0, -0.3, math.sqrt(0.91)),  # yaw negativo
        (0.0, 0.0, 0.999, 0.0447),  # casi π
    ]
    for obj_q in test_cases:
        q = compute_top_down_grasp_quat(obj_q)
        assert q[2] == pytest.approx(0.0, abs=1e-9), f"obj={obj_q} → qz={q[2]} != 0"
        assert q[3] == pytest.approx(0.0, abs=1e-9), f"obj={obj_q} → qw={q[3]} != 0"


def test_z_axis_points_down_for_any_yaw() -> None:
    """Para cualquier yaw input, el Z axis del TCP frame apunta a -Z_world.

    Verificación matemática: rotar (0,0,1) (vector Z_world) por el quat
    debería dar (0,0,-1) (Z_TCP apuntando hacia abajo en world).
    """
    test_yaws = [0.0, math.pi / 4, math.pi / 2, math.pi, -math.pi / 3, 1.234]
    for yaw in test_yaws:
        obj_q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        q = compute_top_down_grasp_quat(obj_q)
        qx, qy, qz, qw = q
        # Rotate (0,0,1) by quaternion q: v' = q * v * q_conj
        # Para v=(0,0,1), simplifica a:
        # v'_x = 2 * (qx*qz + qw*qy)
        # v'_y = 2 * (qy*qz - qw*qx)
        # v'_z = 1 - 2*(qx² + qy²)
        vz_world_after_rot_z = 1.0 - 2.0 * (qx * qx + qy * qy)
        assert vz_world_after_rot_z == pytest.approx(-1.0, abs=1e-9), (
            f"yaw={yaw}, q={q}, vz_after_rot={vz_world_after_rot_z} (expected -1)"
        )


def test_real_live_box_red_pose() -> None:
    """Caso real de box_red post-drop (live data audit-v4 v6 ronda 1).

    pose box_red: pos=(-0.4321,0.2223,0.8800) quat=(-0.0000,0.0000,-0.0999,0.9950)
    Yaw esperado ≈ -0.20 rad ≈ -11.5°.
    """
    obj_q = (0.0, 0.0, -0.0999, 0.9950)
    q_tcp = compute_top_down_grasp_quat(obj_q)
    # Verifica que es unit quat con qz=0, qw=0
    assert _quat_norm(q_tcp) == pytest.approx(1.0, abs=1e-4)
    assert q_tcp[2] == pytest.approx(0.0, abs=1e-9)
    assert q_tcp[3] == pytest.approx(0.0, abs=1e-9)
    # qx=cos(yaw/2)≈cos(-0.10)≈0.995; qy=sin(yaw/2)≈sin(-0.10)≈-0.0998
    assert q_tcp[0] == pytest.approx(0.995, abs=1e-3)
    assert q_tcp[1] == pytest.approx(-0.0998, abs=1e-3)
