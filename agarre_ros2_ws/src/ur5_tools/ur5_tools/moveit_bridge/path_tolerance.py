#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/moveit_bridge/path_tolerance.py
"""T31 (2026-05-08) — Contract puro de path_tolerance MoveIt↔controller.

Lógica extraída de executor.py:415-444. Centraliza la regla:

- ``path_tol_param`` (controller_path_tolerance_rad) puede ser -1.0 ⇒ no
  enviar path_tolerance al controller (MoveIt aplica su default interno).
- ``path_tol_override_rad`` (cuando un caller pasa un valor explícito) toma
  precedencia sobre el param de Lifecycle.
- Si el valor efectivo es negativo, no se envía nada.
- Si es no-negativo, se aplica ``max(path_tol_floor, value)`` para evitar
  enviar tolerancias absurdamente pequeñas que rompan el seguimiento del
  controller en simulación.

Esta función es 100% pura y testeable offline (sin dependencias ROS). El
test T31 vive en ``test/test_path_tolerance_contract.py``.
"""

from __future__ import annotations

from typing import List, NamedTuple, Sequence


class PathToleranceContract(NamedTuple):
    """Resultado del contrato. Si ``send`` es False, no debe construirse goal.path_tolerance."""

    send: bool
    value_rad: float  # 0.0 si send=False (no significativo)
    floor_applied: bool  # True si el floor estaba por encima de value


def compute_path_tolerance_value(
    path_tol_param_rad: float,
    *,
    path_tol_override_rad: float | None = None,
    path_tol_floor_rad: float = 0.05,
) -> PathToleranceContract:
    """Calcula el path_tolerance efectivo a enviar al controller.

    Args:
        path_tol_param_rad: valor del parámetro Lifecycle
            ``controller_path_tolerance_rad``. -1.0 = "no enviar".
        path_tol_override_rad: si no es None, toma precedencia sobre el param.
        path_tol_floor_rad: piso aplicado cuando se envía (default 0.05 rad
            ≈ 2.86°). Evita tolerancias < 0.05 que rompen el seguimiento
            del controller en simulación con scaling agresivo.

    Returns:
        PathToleranceContract(send, value_rad, floor_applied).
    """
    effective = (
        float(path_tol_override_rad)
        if path_tol_override_rad is not None
        else float(path_tol_param_rad)
    )
    if effective < 0.0:
        return PathToleranceContract(send=False, value_rad=0.0, floor_applied=False)
    floor = float(path_tol_floor_rad)
    if effective < floor:
        return PathToleranceContract(send=True, value_rad=floor, floor_applied=True)
    return PathToleranceContract(send=True, value_rad=effective, floor_applied=False)


def build_joint_tolerance_pairs(
    joint_names: Sequence[str],
    *,
    path_tol_param_rad: float,
    path_tol_override_rad: float | None = None,
    path_tol_floor_rad: float = 0.05,
) -> List[tuple[str, float]]:
    """Construye lista de (joint_name, tolerance_rad) usando el contrato.

    Útil para tests: se puede comparar con la lista que envía el executor a
    ``goal.path_tolerance`` sin necesidad de instanciar JointTolerance reales.

    Returns lista vacía si el contrato dice no-enviar.
    """
    contract = compute_path_tolerance_value(
        path_tol_param_rad,
        path_tol_override_rad=path_tol_override_rad,
        path_tol_floor_rad=path_tol_floor_rad,
    )
    if not contract.send or not joint_names:
        return []
    return [(str(name), contract.value_rad) for name in joint_names]
