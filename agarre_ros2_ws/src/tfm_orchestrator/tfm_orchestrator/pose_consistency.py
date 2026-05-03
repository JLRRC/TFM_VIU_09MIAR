#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/pose_consistency.py
# Contenido: B-iter10 (2026-05-03) — gates de consistencia de pose runtime.
"""B-iter10 — Gates de consistencia de pose para el orchestrator.

El legacy ``run_pick_demo`` calcula varias métricas de consistencia
durante el ciclo (TF live vs FK panel vs trace, source_age, source_sync,
phase_jump). Si las fuentes divergen o están stale, aborta antes de
mover el robot a una pose basada en datos obsoletos.

Este módulo migra los chequeos esenciales como **gates puros** al
orchestrator:

* ``evaluate_pose_freshness_gate(tcp_tf_age_sec, joint_state_age_sec,
  max_age_sec)`` — ambas fuentes <= max_age (default 0.20s, idéntico al
  legacy PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC).

* ``evaluate_pose_divergence_gate(pose_a, pose_b, max_norm_m,
  max_orient_rad)`` — divergencia posicional + orientacional entre 2
  poses tuple7 (típicamente TCP TF vs TCP FK del joint_state). Útil
  para detectar TF stale o controller jitter.

* ``compute_position_norm(pose_a, pose_b)`` — norma euclídea posicional.

* ``compute_orientation_angle_diff(quat_a, quat_b)`` — ángulo entre 2
  cuaterniones (cos⁻¹(2·dot²−1)). Independiente del signo del quat.

Sin imports ROS, sin dependencia del panel. 100% offline-testable.

Constantes default extraídas del legacy:
  - max_age_sec: 0.20 (PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC)
  - max_norm_m: 0.006 (PANEL_PICK_DEMO_POSE_SOURCE_TOL_M)
  - max_orient_rad: 0.05 (~2.9°)
"""

from __future__ import annotations

import math
from typing import Optional, Tuple


Tuple7 = Tuple[float, float, float, float, float, float, float]

DEFAULT_MAX_AGE_SEC: float = 0.20
DEFAULT_MAX_NORM_M: float = 0.006
DEFAULT_MAX_ORIENT_RAD: float = 0.05


def evaluate_pose_freshness_gate(
    tcp_tf_age_sec: Optional[float],
    joint_state_age_sec: Optional[float],
    *,
    max_age_sec: float = DEFAULT_MAX_AGE_SEC,
) -> Tuple[bool, str]:
    """Gate: ¿están frescas las fuentes de pose?

    Verifica que tanto el TCP TF como el JointState son recientes
    (age <= max_age_sec). Si alguna está stale, devuelve fail con
    detalle de cuál y cuánto.

    Parameters:
        tcp_tf_age_sec: edad del último lookup TF para rg2_tcp. None ⇒
            no medible (gate falla).
        joint_state_age_sec: edad del último JointState cacheado. None
            ⇒ no medible (gate falla).
        max_age_sec: edad máxima aceptada (default 0.20s).

    Returns:
        (ok, reason). ok=True si ambas <= max_age.
    """
    failures = []
    limit = float(max(0.0, max_age_sec))

    if tcp_tf_age_sec is None:
        failures.append("tcp_tf:unmeasured")
    else:
        try:
            a = float(tcp_tf_age_sec)
            if a > limit:
                failures.append(f"tcp_tf:stale:{a:.3f}s>{limit:.3f}s")
        except (TypeError, ValueError):
            failures.append(f"tcp_tf:invalid:{tcp_tf_age_sec!r}")

    if joint_state_age_sec is None:
        failures.append("joint_state:unmeasured")
    else:
        try:
            a = float(joint_state_age_sec)
            if a > limit:
                failures.append(f"joint_state:stale:{a:.3f}s>{limit:.3f}s")
        except (TypeError, ValueError):
            failures.append(f"joint_state:invalid:{joint_state_age_sec!r}")

    if failures:
        return False, "freshness:" + "|".join(failures)
    return True, (
        f"freshness:ok:tcp_tf={float(tcp_tf_age_sec):.3f}s "
        f"joint_state={float(joint_state_age_sec):.3f}s "
        f"<={limit:.3f}s"
    )


def compute_position_norm(
    pose_a: Optional[Tuple7],
    pose_b: Optional[Tuple7],
) -> Optional[float]:
    """Norma euclídea entre las componentes posicionales (x,y,z) de 2 poses.

    Devuelve None si alguna pose es None o no tiene los 3 primeros floats.
    """
    if pose_a is None or pose_b is None:
        return None
    try:
        ax, ay, az = float(pose_a[0]), float(pose_a[1]), float(pose_a[2])
        bx, by, bz = float(pose_b[0]), float(pose_b[1]), float(pose_b[2])
    except (TypeError, ValueError, IndexError):
        return None
    dx = ax - bx
    dy = ay - by
    dz = az - bz
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def compute_orientation_angle_diff(
    quat_a: Optional[Tuple[float, float, float, float]],
    quat_b: Optional[Tuple[float, float, float, float]],
) -> Optional[float]:
    """Ángulo (rad) entre 2 cuaterniones (xyzw).

    Usa cos⁻¹(2·dot²−1), invariante al signo del quat (q y -q son la
    misma rotación). Devuelve None si alguno no es parseable.
    Resultado en [0, π].
    """
    if quat_a is None or quat_b is None:
        return None
    try:
        ax, ay, az, aw = (float(quat_a[0]), float(quat_a[1]),
                          float(quat_a[2]), float(quat_a[3]))
        bx, by, bz, bw = (float(quat_b[0]), float(quat_b[1]),
                          float(quat_b[2]), float(quat_b[3]))
    except (TypeError, ValueError, IndexError):
        return None
    dot = ax * bx + ay * by + az * bz + aw * bw
    # Clamp para acos numérico estable.
    val = max(-1.0, min(1.0, 2.0 * dot * dot - 1.0))
    return math.acos(val)


def evaluate_pose_divergence_gate(
    pose_a: Optional[Tuple7],
    pose_b: Optional[Tuple7],
    *,
    max_norm_m: float = DEFAULT_MAX_NORM_M,
    max_orient_rad: float = DEFAULT_MAX_ORIENT_RAD,
) -> Tuple[bool, str]:
    """Gate: ¿divergen 2 poses (tuple7) más allá del umbral?

    Compara posición (norma euclídea) y orientación (ángulo entre quats).
    Útil para detectar TF stale o jitter del controller cuando se tienen
    2 fuentes independientes (e.g. TCP via TF live vs TCP via FK del
    joint_state).

    Parameters:
        pose_a, pose_b: poses tuple7 (x,y,z,qx,qy,qz,qw).
        max_norm_m: divergencia posicional máxima (default 0.006m).
        max_orient_rad: divergencia orientacional máxima (default 0.05 rad).

    Returns:
        (ok, reason). ok=True si pos<=max_norm y orient<=max_orient.
    """
    if pose_a is None or pose_b is None:
        return False, "divergence:one_or_both_poses_none"

    pos = compute_position_norm(pose_a, pose_b)
    if pos is None:
        return False, "divergence:position_unparseable"
    quat_a = (pose_a[3], pose_a[4], pose_a[5], pose_a[6])
    quat_b = (pose_b[3], pose_b[4], pose_b[5], pose_b[6])
    orient = compute_orientation_angle_diff(quat_a, quat_b)
    if orient is None:
        return False, "divergence:orientation_unparseable"

    pos_ok = pos <= float(max(0.0, max_norm_m))
    orient_ok = orient <= float(max(0.0, max_orient_rad))
    if pos_ok and orient_ok:
        return True, (
            f"divergence:ok:pos={pos:.4f}m<={float(max_norm_m):.4f}m "
            f"orient={orient:.4f}rad<={float(max_orient_rad):.4f}rad"
        )
    return False, (
        f"divergence:fail:pos={pos:.4f}m"
        f"{'<=' if pos_ok else '>'}"
        f"{float(max_norm_m):.4f}m "
        f"orient={orient:.4f}rad"
        f"{'<=' if orient_ok else '>'}"
        f"{float(max_orient_rad):.4f}rad"
    )
