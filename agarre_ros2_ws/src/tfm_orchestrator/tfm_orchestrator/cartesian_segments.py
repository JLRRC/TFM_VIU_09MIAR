#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/cartesian_segments.py
# Contenido: B-iter11 (2026-05-03) — segmentación cartesiana de movimientos largos.
"""B-iter11 — Segmentación cartesiana para movimientos TCP grandes.

El legacy ``_move_tcp_direct`` (1.008 LOC del panel) descompone un
movimiento TCP largo en N waypoints intermedios para:

* Evitar que MoveIt tome shortcuts inseguros (joint flips).
* Detectar tempranamente colisiones / IK fail en el camino.
* Permitir cancellation a mitad de trayecto sin abortar el ciclo entero.

Este módulo provee la lógica pura de **segmentación**: dado un punto
inicial y un punto final TCP, devuelve N waypoints intermedios linealmente
interpolados (posición y orientación slerped).

API:

* ``segment_cartesian_path(start_xyz, end_xyz, n_segments, start_quat=None,
  end_quat=None)`` → ``List[(xyz, quat)]`` con N+1 puntos (incluyendo
  start y end).

* ``compute_segments_for_distance(start_xyz, end_xyz, max_segment_m)`` →
  número de segmentos sugerido para que cada uno mida <= max_segment_m.

* ``slerp(q0, q1, t)`` → cuaternión interpolado linealmente esférico
  entre q0 y q1, t en [0,1]. Cubre el caso de signo negativo (cuaterniones
  opuestos representan misma rotación, escogemos el más corto).

100% offline. Sin imports ROS.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple


Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]


def slerp(q0: Quat, q1: Quat, t: float) -> Quat:
    """Spherical linear interpolation entre dos cuaterniones (xyzw).

    Parameters:
        q0, q1: cuaterniones unitarios (x, y, z, w).
        t: parámetro de interpolación, 0.0 = q0, 1.0 = q1.

    Returns:
        Cuaternión interpolado, normalizado.

    Si dot(q0, q1) < 0, se niega q1 antes de interpolar para tomar el
    arco más corto (q y -q son la misma rotación).
    """
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    dot = x0 * x1 + y0 * y1 + z0 * z1 + w0 * w1
    if dot < 0.0:
        x1, y1, z1, w1 = -x1, -y1, -z1, -w1
        dot = -dot

    # Si los quats son casi paralelos, lerp simple.
    if dot > 0.9995:
        rx = x0 + t * (x1 - x0)
        ry = y0 + t * (y1 - y0)
        rz = z0 + t * (z1 - z0)
        rw = w0 + t * (w1 - w0)
    else:
        theta_0 = math.acos(max(-1.0, min(1.0, dot)))
        sin_theta_0 = math.sin(theta_0)
        theta = theta_0 * t
        sin_theta = math.sin(theta)
        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        rx = s0 * x0 + s1 * x1
        ry = s0 * y0 + s1 * y1
        rz = s0 * z0 + s1 * z1
        rw = s0 * w0 + s1 * w1

    norm = math.sqrt(rx * rx + ry * ry + rz * rz + rw * rw)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (rx / norm, ry / norm, rz / norm, rw / norm)


def segment_cartesian_path(
    start_xyz: Vec3,
    end_xyz: Vec3,
    n_segments: int,
    *,
    start_quat: Optional[Quat] = None,
    end_quat: Optional[Quat] = None,
) -> List[Tuple[Vec3, Quat]]:
    """Genera N+1 waypoints linealmente interpolados entre start y end.

    Parameters:
        start_xyz: punto inicial (x, y, z).
        end_xyz: punto final (x, y, z).
        n_segments: número de segmentos (>=1). El resultado tiene N+1 puntos.
        start_quat: orientación inicial. Si None, usa identity (0,0,0,1).
        end_quat: orientación final. Si None, usa start_quat (sin rotación).

    Returns:
        Lista de (xyz, quat) con N+1 elementos. El primero es start, el
        último es end.

    Raises:
        ValueError: si n_segments < 1.
    """
    n = int(n_segments)
    if n < 1:
        raise ValueError(f"n_segments debe ser >= 1, recibido {n}")

    sq = start_quat if start_quat is not None else (0.0, 0.0, 0.0, 1.0)
    eq = end_quat if end_quat is not None else sq

    sx, sy, sz = float(start_xyz[0]), float(start_xyz[1]), float(start_xyz[2])
    ex, ey, ez = float(end_xyz[0]), float(end_xyz[1]), float(end_xyz[2])

    waypoints: List[Tuple[Vec3, Quat]] = []
    for i in range(n + 1):
        t = float(i) / float(n)
        x = sx + (ex - sx) * t
        y = sy + (ey - sy) * t
        z = sz + (ez - sz) * t
        q = slerp(sq, eq, t)
        waypoints.append(((x, y, z), q))
    return waypoints


def compute_segments_for_distance(
    start_xyz: Vec3,
    end_xyz: Vec3,
    *,
    max_segment_m: float = 0.05,
    min_segments: int = 1,
    max_segments: int = 50,
) -> int:
    """Decide el número de segmentos para que cada uno mida <= max_segment_m.

    Parameters:
        start_xyz, end_xyz: extremos del path.
        max_segment_m: longitud máxima por segmento en metros (default 0.05m).
        min_segments: nunca devolver menos que esto (default 1).
        max_segments: tope superior por seguridad (default 50).

    Returns:
        Número de segmentos en [min_segments, max_segments].
    """
    sx, sy, sz = float(start_xyz[0]), float(start_xyz[1]), float(start_xyz[2])
    ex, ey, ez = float(end_xyz[0]), float(end_xyz[1]), float(end_xyz[2])
    dist = math.sqrt((ex - sx) ** 2 + (ey - sy) ** 2 + (ez - sz) ** 2)
    limit = float(max(1e-6, max_segment_m))
    n = int(math.ceil(dist / limit))
    n = max(int(min_segments), n)
    n = min(int(max_segments), n)
    return n
