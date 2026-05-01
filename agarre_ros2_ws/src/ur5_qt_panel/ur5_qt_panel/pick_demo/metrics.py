#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/metrics.py
# Contenido: F3 — métricas puras extraídas de panel_pick_demo.run_pick_demo.
"""Funciones puras de cálculo de métricas para el flujo pick_demo.

Extraídas de las closures internas de ``run_pick_demo``:

* ``joint_delta_metrics``: delta absoluto en cada joint UR5 entre dos vectores
  de configuración (referencia vs final), con métricas agregadas y subset
  crítico (shoulder_lift, elbow, wrist_1).
* ``joint_error_metrics``: error angular por joint vs valores actuales del
  panel (snapshot pasado como argumento), con detección de error máximo en
  shoulders.
* ``alignment_metrics_base``: cálculo común de xy_dist / z_gap / z_error
  TCP-objeto contra un Z target esperado, con tolerancias configurables.
* ``grasp_down_runtime_metrics``: error TCP vs target_base + TCP vs object_base
  (xy/z) usado para gating runtime de GRASP_DOWN.

Todas son puras: cero dependencia ROS, cero acceso a panel/scope, cero env
vars. Las tolerancias y posiciones de joint se reciben como argumentos.
"""

from __future__ import annotations

import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from ur5_qt_panel.directo_geometry import angle_shortest_diff_rad

Vec3 = Tuple[float, float, float]


# ---------------------------------------------------------------------------
# Joint metrics
# ---------------------------------------------------------------------------


def joint_delta_metrics(
    reference_joints: Optional[Sequence[float]],
    final_joints: Optional[Sequence[float]],
    joint_names: Sequence[str],
) -> dict:
    """Delta absoluto en radianes (shortest path) entre dos configuraciones.

    Devuelve dict con claves:
      * ``joint_delta_abs``: dict {joint_name -> abs_delta_rad}
      * ``max_joint_delta`` / ``sum_joint_delta``: agregados (None si vacío)
      * ``critical_joints_delta``: subset {shoulder_lift, elbow, wrist_1}
    """
    if not reference_joints or not final_joints:
        return {
            "joint_delta_abs": {},
            "max_joint_delta": None,
            "sum_joint_delta": None,
            "critical_joints_delta": {},
        }
    deltas: Dict[str, float] = {}
    for joint_name, ref_q, final_q in zip(joint_names, reference_joints, final_joints):
        deltas[str(joint_name)] = abs(
            float(angle_shortest_diff_rad(float(ref_q), float(final_q)))
        )
    critical_joint_names = (
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
    )
    delta_values = list(deltas.values())
    return {
        "joint_delta_abs": deltas,
        "max_joint_delta": max(delta_values) if delta_values else None,
        "sum_joint_delta": sum(delta_values) if delta_values else None,
        "critical_joints_delta": {
            joint_name: deltas.get(joint_name)
            for joint_name in critical_joint_names
            if joint_name in deltas
        },
    }


def joint_error_metrics(
    joints: Sequence[float],
    joint_names: Sequence[str],
    current_positions: Dict[str, float],
) -> dict:
    """Error angular absoluto por joint vs ``current_positions`` (snapshot).

    Devuelve ``{available, max_diff_rad, sum_diff_rad, shoulder_max_diff_rad}``.
    Si ningún joint tiene posición actual disponible, ``available=False``.

    ``shoulder_max_diff_rad`` se calcula sobre los dos primeros joints
    (shoulder_pan, shoulder_lift) por convención del flujo.
    """
    diffs: List[float] = []
    shoulder_diffs: List[float] = []
    for idx, name in enumerate(joint_names):
        if idx >= len(joints):
            break
        curr = current_positions.get(name)
        if curr is None:
            continue
        diff = abs(angle_shortest_diff_rad(float(curr), float(joints[idx])))
        diffs.append(float(diff))
        if idx in (0, 1):
            shoulder_diffs.append(float(diff))
    if not diffs:
        return {
            "available": False,
            "max_diff_rad": None,
            "sum_diff_rad": None,
            "shoulder_max_diff_rad": None,
        }
    return {
        "available": True,
        "max_diff_rad": float(max(diffs)),
        "sum_diff_rad": float(sum(diffs)),
        "shoulder_max_diff_rad": (
            float(max(shoulder_diffs)) if shoulder_diffs else 0.0
        ),
    }


# ---------------------------------------------------------------------------
# Alignment metrics (xy/z TCP-object vs target Z)
# ---------------------------------------------------------------------------


def _vec_dist3(a: Vec3, b: Vec3) -> float:
    return math.sqrt(
        (float(a[0]) - float(b[0])) ** 2
        + (float(a[1]) - float(b[1])) ** 2
        + (float(a[2]) - float(b[2])) ** 2
    )


def _as_tuple3(v) -> Optional[Vec3]:
    if v is None:
        return None
    try:
        return (float(v[0]), float(v[1]), float(v[2]))
    except (TypeError, ValueError, IndexError):
        return None


def alignment_metrics_base(
    *,
    obj_base: Optional[Vec3],
    tcp_base: Optional[Vec3],
    grasp_z_target: float,
    xy_tol: float,
    z_tol: float,
    pose_consistency: dict,
) -> dict:
    """Métricas xy/z de alignment TCP-objeto contra target_z esperado.

    Caso vacío (alguna pose None): devuelve ``ok=False, reason="pose_unavailable"``.

    Caso normal: calcula ``xy_dist``, ``z_gap`` (signed), ``z_error`` (abs vs
    target_z), ``tcp_obj_dist``. Geometry_ok si xy_dist≤xy_tol y z_error≤z_tol y
    ``pose_consistency.sources_ok`` es True.

    El llamador puede añadir campos extra (gripper_state, etc.) al dict
    devuelto.
    """
    obj_t = _as_tuple3(obj_base)
    tcp_t = _as_tuple3(tcp_base)
    if obj_t is None or tcp_t is None:
        return {
            "ok": False,
            "geometry_ok": False,
            "reason": "pose_unavailable",
            "xy_dist": None,
            "z_gap": None,
            "z_error": None,
            "tcp_obj_dist": None,
            "xy_tol": float(xy_tol),
            "z_tol": float(z_tol),
            "tcp_base": tcp_t,
            "object_base": obj_t,
            "pose_consistency": pose_consistency,
            "pose_source_ok": False,
        }
    xy_dist = math.hypot(tcp_t[0] - obj_t[0], tcp_t[1] - obj_t[1])
    z_gap = tcp_t[2] - obj_t[2]
    z_error = abs(z_gap - float(grasp_z_target))
    tcp_obj_dist = _vec_dist3(tcp_t, obj_t)
    pose_source_ok = bool(pose_consistency.get("sources_ok"))
    geometry_ok = bool(
        xy_dist <= float(xy_tol)
        and z_error <= float(z_tol)
        and pose_source_ok
    )
    return {
        "ok": geometry_ok,
        "geometry_ok": geometry_ok,
        "reason": (
            "ok"
            if geometry_ok
            else "pose_source_mismatch"
            if not pose_source_ok
            else "alignment_out_of_tolerance"
        ),
        "xy_dist": float(xy_dist),
        "z_gap": float(z_gap),
        "z_error": float(z_error),
        "tcp_obj_dist": float(tcp_obj_dist),
        "xy_tol": float(xy_tol),
        "z_tol": float(z_tol),
        "pose_consistency": pose_consistency,
        "pose_source_ok": pose_source_ok,
        "tcp_base": tcp_t,
        "object_base": obj_t,
    }


# ---------------------------------------------------------------------------
# Grasp-down runtime metrics
# ---------------------------------------------------------------------------


def grasp_down_runtime_metrics(
    *,
    target_base: Optional[Vec3],
    tcp_base: Optional[Vec3],
    obj_base: Optional[Vec3] = None,
) -> dict:
    """Errores xy/z TCP-target y TCP-object para gating de GRASP_DOWN.

    Inputs ya resueltos como tuples3 (None permitido). Si TCP+target están
    disponibles calcula xy_err_target/z_err_target/target_dist; si TCP+obj
    también, calcula xy_err_object/z_gap_object.
    """
    target_t = _as_tuple3(target_base)
    tcp_t = _as_tuple3(tcp_base)
    obj_t = _as_tuple3(obj_base)
    xy_err_target: Optional[float] = None
    z_err_target: Optional[float] = None
    target_dist: Optional[float] = None
    xy_err_object: Optional[float] = None
    z_gap_object: Optional[float] = None
    if tcp_t is not None and target_t is not None:
        xy_err_target = math.hypot(tcp_t[0] - target_t[0], tcp_t[1] - target_t[1])
        z_err_target = tcp_t[2] - target_t[2]
        target_dist = _vec_dist3(tcp_t, target_t)
    if tcp_t is not None and obj_t is not None:
        xy_err_object = math.hypot(tcp_t[0] - obj_t[0], tcp_t[1] - obj_t[1])
        z_gap_object = tcp_t[2] - obj_t[2]
    return {
        "target_base": target_t,
        "tcp_base": tcp_t,
        "object_base": obj_t,
        "xy_err_target": xy_err_target,
        "z_err_target": z_err_target,
        "target_dist": target_dist,
        "xy_err_object": xy_err_object,
        "z_gap_object": z_gap_object,
    }
