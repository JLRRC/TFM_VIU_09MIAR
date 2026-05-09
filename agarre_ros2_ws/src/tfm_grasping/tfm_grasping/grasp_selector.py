#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/tfm_grasping/grasp_selector.py
"""F17 (2026-05-08) — Grasp selector puro (sin ROS).

Filtra y rankea poses de agarre candidatas según viabilidad geométrica
y kinemática. Lógica extraída del flujo `panel_pick_object` y del
orchestrator donde estaba dispersa.

API:
- ``select_best_grasp(candidates, ...)`` devuelve el mejor candidato
  según el ranker, o None si todos quedan filtrados.
- ``filter_by_workspace(candidates, ws_radius_m=...)`` filtra por
  alcance del UR5 (radio nominal 0.85m con el RG2).
- ``filter_by_tcp_down(candidates, tolerance_rad=...)`` mantiene poses
  con orientación TCP-down (Z apunta -world_Z).
- ``rank_by_distance_to_base(candidates, base_xyz)`` devuelve la lista
  ordenada por proximidad euclídea al base_link.

Sin estado, sin ROS al importar. Tests offline en
``test/test_grasp_selector.py``.

Diseño: el nodo ROS2 que envuelve esta lógica (grasp_selector_node) se
añadirá en una sesión separada cuando se valide live (queda pendiente
de F17-step2).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

# UR5 + RG2 workspace nominal: 0.85m de alcance desde base_link.
DEFAULT_WS_RADIUS_M = 0.85
# Mínimo razonable para alcance (objetos muy cercanos al base son inviables
# por singularidades del shoulder).
DEFAULT_WS_MIN_RADIUS_M = 0.20
# Tolerancia angular para considerar que un quaternion representa TCP-down.
# 0.20 rad ≈ 11.5° — alineado con `compute_top_down_grasp_quat` del proyecto.
DEFAULT_TCP_DOWN_TOLERANCE_RAD = 0.20


@dataclass(frozen=True)
class GraspCandidate:
    """Candidato de agarre con pose base_link y score del modelo CNN."""

    xyz: Tuple[float, float, float]  # posición en base_link
    quat_xyzw: Tuple[float, float, float, float]  # orientación normalizada
    score: float = 0.0  # score del modelo CNN (0..1, 1=mejor)
    label: str = ""  # opcional: id del candidato


@dataclass(frozen=True)
class SelectionResult:
    """Resultado de select_best_grasp con trazabilidad."""

    best: Optional[GraspCandidate]
    n_input: int
    n_after_workspace: int
    n_after_tcp_down: int
    reason: str  # "ok" | "no_input" | "filtered_workspace" | "filtered_tcp_down"


def _quat_z_axis_in_world(
    quat_xyzw: Tuple[float, float, float, float],
) -> Tuple[float, float, float]:
    """Devuelve el eje Z del frame del TCP expresado en base_link.

    R * (0,0,1)^T — tercera columna de la matriz de rotación.
    """
    qx, qy, qz, qw = (float(c) for c in quat_xyzw)
    # Tercera columna de la matriz rotación de quaternion.
    z_x = 2.0 * (qx * qz + qy * qw)
    z_y = 2.0 * (qy * qz - qx * qw)
    z_z = 1.0 - 2.0 * (qx * qx + qy * qy)
    return (z_x, z_y, z_z)


def is_tcp_down(
    quat_xyzw: Tuple[float, float, float, float],
    *,
    tolerance_rad: float = DEFAULT_TCP_DOWN_TOLERANCE_RAD,
) -> bool:
    """True si el quaternion apunta el eje Z del TCP hacia -world_Z (down).

    Tolerancia angular: la diferencia entre el eje Z del TCP y (0,0,-1)
    debe ser ≤ tolerance_rad. 0 rad = perfectamente vertical.
    """
    zx, zy, zz = _quat_z_axis_in_world(quat_xyzw)
    # cos(angle) = z_tcp · (0,0,-1) = -zz
    cos_angle = max(-1.0, min(1.0, -float(zz)))
    angle = math.acos(cos_angle)
    return angle <= float(tolerance_rad)


def filter_by_workspace(
    candidates: List[GraspCandidate],
    *,
    ws_radius_m: float = DEFAULT_WS_RADIUS_M,
    ws_min_radius_m: float = DEFAULT_WS_MIN_RADIUS_M,
    base_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> List[GraspCandidate]:
    """Conserva candidatos cuya distancia al base esté en [ws_min, ws_radius]."""
    out: List[GraspCandidate] = []
    bx, by, bz = (float(c) for c in base_xyz)
    for c in candidates:
        dx = float(c.xyz[0]) - bx
        dy = float(c.xyz[1]) - by
        dz = float(c.xyz[2]) - bz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if ws_min_radius_m <= dist <= ws_radius_m:
            out.append(c)
    return out


def filter_by_tcp_down(
    candidates: List[GraspCandidate],
    *,
    tolerance_rad: float = DEFAULT_TCP_DOWN_TOLERANCE_RAD,
) -> List[GraspCandidate]:
    """Conserva candidatos con orientación TCP-down dentro de la tolerancia."""
    return [c for c in candidates if is_tcp_down(c.quat_xyzw, tolerance_rad=tolerance_rad)]


def rank_by_score_and_distance(
    candidates: List[GraspCandidate],
    *,
    base_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    score_weight: float = 0.7,
    distance_weight: float = 0.3,
) -> List[GraspCandidate]:
    """Ordena candidatos por una combinación lineal de score (mayor=mejor) y
    distancia al base (menor=mejor, normalizado a [0,1]).

    Returns la lista ordenada de mejor a peor.
    """
    if not candidates:
        return []
    bx, by, bz = (float(c) for c in base_xyz)
    distances = []
    for c in candidates:
        dx = float(c.xyz[0]) - bx
        dy = float(c.xyz[1]) - by
        dz = float(c.xyz[2]) - bz
        distances.append(math.sqrt(dx * dx + dy * dy + dz * dz))
    max_dist = max(distances) or 1.0

    def _key(item: Tuple[float, GraspCandidate]) -> float:
        d, cand = item
        d_norm = float(d) / max_dist  # [0..1]
        # Score más alto + distancia más baja → ranking más alto.
        return -(float(score_weight) * float(cand.score) - float(distance_weight) * d_norm)

    pairs = list(zip(distances, candidates))
    pairs.sort(key=_key)
    return [c for _d, c in pairs]


def select_best_grasp(
    candidates: List[GraspCandidate],
    *,
    base_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ws_radius_m: float = DEFAULT_WS_RADIUS_M,
    ws_min_radius_m: float = DEFAULT_WS_MIN_RADIUS_M,
    tcp_down_tolerance_rad: float = DEFAULT_TCP_DOWN_TOLERANCE_RAD,
    require_tcp_down: bool = True,
) -> SelectionResult:
    """Pipeline completo: filtrar por workspace + TCP-down + rankear.

    Returns:
        SelectionResult(best, n_input, n_after_workspace, n_after_tcp_down,
                        reason).
    """
    n_input = len(candidates)
    if n_input == 0:
        return SelectionResult(
            best=None, n_input=0, n_after_workspace=0, n_after_tcp_down=0,
            reason="no_input",
        )
    after_ws = filter_by_workspace(
        candidates,
        ws_radius_m=ws_radius_m,
        ws_min_radius_m=ws_min_radius_m,
        base_xyz=base_xyz,
    )
    n_after_ws = len(after_ws)
    if n_after_ws == 0:
        return SelectionResult(
            best=None, n_input=n_input, n_after_workspace=0, n_after_tcp_down=0,
            reason="filtered_workspace",
        )
    after_tcp = (
        filter_by_tcp_down(after_ws, tolerance_rad=tcp_down_tolerance_rad)
        if require_tcp_down
        else list(after_ws)
    )
    n_after_tcp = len(after_tcp)
    if n_after_tcp == 0:
        return SelectionResult(
            best=None, n_input=n_input, n_after_workspace=n_after_ws,
            n_after_tcp_down=0, reason="filtered_tcp_down",
        )
    ranked = rank_by_score_and_distance(after_tcp, base_xyz=base_xyz)
    return SelectionResult(
        best=ranked[0],
        n_input=n_input,
        n_after_workspace=n_after_ws,
        n_after_tcp_down=n_after_tcp,
        reason="ok",
    )
