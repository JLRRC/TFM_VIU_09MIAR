#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_path_tolerance_contract.py
"""T31 (2026-05-08) — Contract test path_tolerance MoveIt↔controller.

Cubre dos capas:
1. Función pura ``compute_path_tolerance_value`` (lógica de decisión).
2. AST guardrail sobre executor.py para detectar regresiones del contrato.

Bug relacionado: ``auditoria/bugs_pendientes/BUG_BRIDGE_PATH_TOLERANCE.md``.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from ur5_tools.moveit_bridge.path_tolerance import (
    PathToleranceContract,
    build_joint_tolerance_pairs,
    compute_path_tolerance_value,
)


# ---------------------------------------------------------------------------
# compute_path_tolerance_value — función pura
# ---------------------------------------------------------------------------


def test_negative_param_means_do_not_send():
    """controller_path_tolerance_rad = -1.0 (default) ⇒ no enviar nada."""
    c = compute_path_tolerance_value(-1.0)
    assert c.send is False
    assert c.floor_applied is False


def test_zero_param_floors_to_default():
    """0.0 es válido pero por debajo del floor 0.05 ⇒ se eleva a 0.05."""
    c = compute_path_tolerance_value(0.0)
    assert c.send is True
    assert c.value_rad == pytest.approx(0.05)
    assert c.floor_applied is True


def test_positive_above_floor_passes_through():
    """0.10 > 0.05 (floor) ⇒ pasa sin cambio."""
    c = compute_path_tolerance_value(0.10)
    assert c.send is True
    assert c.value_rad == pytest.approx(0.10)
    assert c.floor_applied is False


def test_positive_below_floor_clamped():
    """0.02 < 0.05 (floor) ⇒ se clamps al floor."""
    c = compute_path_tolerance_value(0.02)
    assert c.send is True
    assert c.value_rad == pytest.approx(0.05)
    assert c.floor_applied is True


def test_override_takes_precedence_over_param():
    """override=0.20 con param=-1 ⇒ se envía 0.20 (override gana)."""
    c = compute_path_tolerance_value(-1.0, path_tol_override_rad=0.20)
    assert c.send is True
    assert c.value_rad == pytest.approx(0.20)
    assert c.floor_applied is False


def test_override_negative_disables_send():
    """override=-1 fuerza 'no enviar' aunque param sea positivo."""
    c = compute_path_tolerance_value(0.50, path_tol_override_rad=-1.0)
    assert c.send is False


def test_custom_floor_respected():
    """Caller puede subir el floor (e.g. 6.5 rad para FJT direct)."""
    c = compute_path_tolerance_value(0.10, path_tol_floor_rad=6.5)
    assert c.send is True
    assert c.value_rad == pytest.approx(6.5)
    assert c.floor_applied is True


# ---------------------------------------------------------------------------
# build_joint_tolerance_pairs — wrapper para comparar con goal.path_tolerance
# ---------------------------------------------------------------------------


def test_pairs_empty_when_contract_says_no_send():
    pairs = build_joint_tolerance_pairs(
        ["j1", "j2", "j3"], path_tol_param_rad=-1.0
    )
    assert pairs == []


def test_pairs_one_per_joint_with_floor():
    pairs = build_joint_tolerance_pairs(
        ["shoulder_pan", "shoulder_lift", "elbow"],
        path_tol_param_rad=0.10,
    )
    assert pairs == [
        ("shoulder_pan", pytest.approx(0.10)),
        ("shoulder_lift", pytest.approx(0.10)),
        ("elbow", pytest.approx(0.10)),
    ]


def test_pairs_empty_when_no_joints():
    pairs = build_joint_tolerance_pairs([], path_tol_param_rad=0.10)
    assert pairs == []


def test_pairs_use_override():
    pairs = build_joint_tolerance_pairs(
        ["j1"], path_tol_param_rad=0.10, path_tol_override_rad=0.30
    )
    assert pairs == [("j1", pytest.approx(0.30))]


# ---------------------------------------------------------------------------
# T31 AST guardrail — executor.py mantiene el contrato
# ---------------------------------------------------------------------------


WS_ROOT = Path(__file__).resolve().parents[3]
EXECUTOR_SRC = (
    WS_ROOT / "src" / "ur5_tools" / "ur5_tools" / "moveit_bridge" / "executor.py"
)


def test_executor_has_path_tolerance_floor_guard():
    """Guardrail: executor.py mantiene un floor positivo antes del send.

    Si alguien borra el ``path_tol_floor`` o lo pone a 0 sin comentario,
    este test cae. Detecta regresiones del bug bridge path_tolerance.
    """
    src = EXECUTOR_SRC.read_text(encoding="utf-8")
    assert "path_tol_floor" in src, (
        "executor.py debe declarar path_tol_floor — guardrail del bug bridge"
    )
    assert "path_tol_floor = 0.05" in src or "path_tol_floor=0.05" in src, (
        "El floor por defecto debe ser 0.05 rad — si lo cambias, "
        "actualiza este test y BUG_BRIDGE_PATH_TOLERANCE.md."
    )


def test_executor_skips_path_tolerance_when_negative():
    """Guardrail: executor.py respeta el contrato 'path_tol < 0 ⇒ no enviar'."""
    src = EXECUTOR_SRC.read_text(encoding="utf-8")
    # Buscamos el guard explícito o equivalente
    assert (
        "path_tol >= 0.0" in src
        or "path_tol_param_rad >= 0.0" in src
        or "if joint_names and path_tol >= 0" in src
    ), (
        "executor.py debe gatear el envío de path_tolerance con `path_tol >= 0.0`."
    )


def test_executor_uses_max_with_floor():
    """Guardrail: executor.py aplica ``max(path_tol_floor, path_tol)`` antes de send."""
    src = EXECUTOR_SRC.read_text(encoding="utf-8")
    assert "max(path_tol_floor" in src, (
        "executor.py debe aplicar `max(path_tol_floor, path_tol)` al construir "
        "JointTolerance — sin esto, valores < floor rompen seguimiento."
    )
