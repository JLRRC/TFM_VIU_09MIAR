#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/gripper_monitor.py
# Contenido: B-iter12 (2026-05-03) — monitor del estado del gripper RG2.
"""B-iter12 — Monitor del gripper RG2 desde joint_state.

El legacy ``run_pick_demo`` consulta ``opening_sum`` del gripper antes y
después de Close para evaluar si el cierre fue físicamente efectivo (gate
``close_delta`` de B-iter8). Esto requiere extraer del JointState global
los 2 joints del gripper RG2 y sumar sus posiciones.

Este módulo provee la lógica pura:

* ``extract_gripper_opening_sum(joint_state_msg, gripper_joint_names)`` →
  sum de las posiciones de los joints del gripper. None si alguno
  falta o el msg está vacío.

* ``GRIPPER_RG2_JOINT_NAMES``: tuple de los 2 joints canónicos del RG2
  (``"rg2_finger_joint_left"``, ``"rg2_finger_joint_right"``). Coincide
  con el SRDF/URDF del UR5+RG2 del proyecto.

100% offline. Sin imports ROS al cargar.
"""

from __future__ import annotations

from typing import Any, Optional, Sequence, Tuple


# Nombres canónicos de los joints del RG2 (SRDF UR5+RG2 del proyecto).
# El gripper es prismático con 2 dedos; opening_sum = pos_left + pos_right.
GRIPPER_RG2_JOINT_NAMES: Tuple[str, ...] = (
    "rg2_finger_joint_left",
    "rg2_finger_joint_right",
)


def extract_gripper_opening_sum(
    joint_state_msg: Any,
    *,
    gripper_joint_names: Sequence[str] = GRIPPER_RG2_JOINT_NAMES,
) -> Optional[float]:
    """Suma las posiciones de los joints del gripper en un JointState.

    Parameters:
        joint_state_msg: ``sensor_msgs/JointState`` con .name + .position.
        gripper_joint_names: nombres de los joints a sumar (default RG2).

    Returns:
        ``opening_sum: float`` (m, rad o equivalente del joint).
        None si alguno de los joints no está en el msg o la longitud
        no cuadra.
    """
    if joint_state_msg is None:
        return None
    names = list(getattr(joint_state_msg, "name", []) or [])
    positions = list(getattr(joint_state_msg, "position", []) or [])
    if len(names) != len(positions):
        return None
    name_to_pos = {str(n): float(p) for n, p in zip(names, positions)}
    try:
        return float(sum(name_to_pos[j] for j in gripper_joint_names))
    except KeyError:
        return None
    except (TypeError, ValueError):
        return None


def is_gripper_closed(
    joint_state_msg: Any,
    *,
    closed_threshold_sum: float = 0.020,
    gripper_joint_names: Sequence[str] = GRIPPER_RG2_JOINT_NAMES,
) -> Optional[bool]:
    """Estimación rápida: gripper cerrado si opening_sum <= threshold.

    Útil como sanity-check post-Close (e.g. verificar que el comando
    realmente movió los dedos antes de hacer un Attach lógico).

    Returns:
        True si opening_sum <= threshold (cerrado).
        False si > threshold (abierto).
        None si no se pudo medir (sin opening_sum).
    """
    opening = extract_gripper_opening_sum(
        joint_state_msg, gripper_joint_names=gripper_joint_names,
    )
    if opening is None:
        return None
    return opening <= float(max(0.0, closed_threshold_sum))
