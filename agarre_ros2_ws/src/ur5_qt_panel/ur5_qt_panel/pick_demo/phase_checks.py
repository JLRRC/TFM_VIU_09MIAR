#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/phase_checks.py
# Contenido: F3 — phase check pure logic for APPROACH_COARSE handoff.
"""Lógica pura del phase check de APPROACH_COARSE.

Extraído del closure ``_build_approach_coarse_phase_check`` (88 LOC) en
``run_pick_demo``. Decide si el handoff entre APPROACH_COARSE y la
siguiente fase está OK según dos modos de tolerancia:

* **strict**: ``gate_ok`` y ``tcp_obj_dist <= handoff_dist_tol`` y
  ``|dz_obj| <= handoff_dz_tol``.
* **relaxed corridor**: ``gate_xy_err <= relaxed_xy_cap_m``, pose_ok
  o ``relaxed_skip_pose_ok``, ``tcp_obj_dist <= relaxed_handoff_dist_tol``,
  ``0 <= dz_obj <= relaxed_handoff_dz_tol``.

Resultado OK si cualquiera de los dos modos pasa.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

Vec3 = Tuple[float, float, float]


def _as_tuple3(v: Any) -> Optional[Vec3]:
    if v is None:
        return None
    try:
        return (float(v[0]), float(v[1]), float(v[2]))
    except (TypeError, ValueError, IndexError):
        return None


def _vec_dist3(a: Vec3, b: Vec3) -> float:
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
        + (float(a[2]) - float(b[2])) ** 2
    )


def build_approach_coarse_phase_check(
    *,
    target_base: Optional[Vec3],
    gate_metrics: Dict[str, Any],
    fallback_tcp: Optional[Vec3],
    fallback_obj: Optional[Vec3],
    handoff_dist_tol: float,
    handoff_dz_tol: float,
    relaxed_handoff_dist_tol: float,
    relaxed_handoff_dz_tol: float,
    gate_xy_err: float,
    gate_pose_ok: bool,
    relaxed_xy_cap_m: float,
    relaxed_skip_pose_ok: bool,
) -> Dict[str, Any]:
    """Calcula el phase check de APPROACH_COARSE.

    Inputs:
      * ``target_base``: tcp target base (puede ser None).
      * ``gate_metrics``: dict del gate evaluator. Se usan keys
        ``tcp_base``, ``object_base``, ``ok``.
      * ``fallback_tcp`` / ``fallback_obj``: usados si gate_metrics no
        provee tcp_base / object_base.
      * ``handoff_*_tol``: tolerancias modo estricto.
      * ``relaxed_handoff_*_tol``: tolerancias modo relajado (corridor).
      * ``gate_xy_err``: error xy del gate (xy_dist).
      * ``gate_pose_ok``: si pose del gate es OK.
      * ``relaxed_xy_cap_m``: cap xy para modo relajado.
      * ``relaxed_skip_pose_ok``: permite ignorar pose_ok en relaxed mode.

    Devuelve dict con campos coherentes con el closure original:
      gate_ok, tcp_base, object_base, target_base, target_z, object_z,
      actual_z, tcp_obj_dist, dz_obj, handoff_dist_ok, handoff_dz_ok,
      relaxed_handoff_ok, result, gate_decision, block_reasons.
    """
    target_local = _as_tuple3(target_base)
    tcp_local = _as_tuple3(gate_metrics.get("tcp_base")) or _as_tuple3(fallback_tcp)
    obj_local = _as_tuple3(gate_metrics.get("object_base")) or _as_tuple3(fallback_obj)

    target_z = float(target_local[2]) if target_local is not None else None
    object_z = float(obj_local[2]) if obj_local is not None else None
    actual_z = float(tcp_local[2]) if tcp_local is not None else None

    if tcp_local is not None and obj_local is not None:
        tcp_obj_dist = _vec_dist3(tcp_local, obj_local)
    else:
        tcp_obj_dist = None

    if actual_z is not None and object_z is not None:
        dz_obj = actual_z - object_z
    else:
        dz_obj = None

    handoff_dist_ok = bool(
        tcp_obj_dist is not None and float(tcp_obj_dist) <= float(handoff_dist_tol)
    )
    handoff_dz_ok = bool(
        dz_obj is not None and abs(float(dz_obj)) <= float(handoff_dz_tol)
    )
    gate_ok = bool(gate_metrics.get("ok"))

    relaxed_handoff_xy_ok = bool(float(gate_xy_err) <= float(relaxed_xy_cap_m))
    relaxed_handoff_ok = bool(
        relaxed_handoff_xy_ok
        and (bool(gate_pose_ok) or bool(relaxed_skip_pose_ok))
        and tcp_obj_dist is not None
        and float(tcp_obj_dist) <= float(relaxed_handoff_dist_tol)
        and dz_obj is not None
        and float(dz_obj) >= 0.0
        and float(dz_obj) <= float(relaxed_handoff_dz_tol)
    )

    strict_handoff_ok = bool(gate_ok and handoff_dist_ok and handoff_dz_ok)
    result = "OK" if (strict_handoff_ok or relaxed_handoff_ok) else "NO"

    block_reasons: List[str] = []
    if not gate_ok and not relaxed_handoff_ok:
        block_reasons.append("phase_gate_not_ready")
    if not handoff_dist_ok and not relaxed_handoff_ok:
        block_reasons.append("tcp_obj_dist_exceeded")
    if not handoff_dz_ok and not relaxed_handoff_ok:
        block_reasons.append("dz_obj_exceeded")

    return {
        "gate_ok": gate_ok,
        "tcp_base": tcp_local,
        "object_base": obj_local,
        "target_base": target_local,
        "target_z": target_z,
        "object_z": object_z,
        "actual_z": actual_z,
        "tcp_obj_dist": tcp_obj_dist,
        "dz_obj": dz_obj,
        "handoff_dist_ok": handoff_dist_ok,
        "handoff_dz_ok": handoff_dz_ok,
        "relaxed_handoff_ok": relaxed_handoff_ok,
        "result": result,
        "gate_decision": (
            "handoff_ready"
            if strict_handoff_ok
            else "handoff_ready_relaxed_corridor"
            if relaxed_handoff_ok
            else "not_ready"
        ),
        "block_reasons": block_reasons,
    }
