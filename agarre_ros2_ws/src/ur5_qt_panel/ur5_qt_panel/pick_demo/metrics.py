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
from typing import Dict, List, Optional, Sequence, Tuple

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


# ---------------------------------------------------------------------------
# Pose consistency metrics (F3-step35: extraído del closure run_pick_demo)
# ---------------------------------------------------------------------------


def compute_pose_consistency_metrics(
    panel,
    phase_pose_cache: dict,
    *,
    phase: Optional[str],
    tcp_base: Optional[Vec3],
    panel_fk_base: Optional[Vec3],
    panel_trace_base: Optional[Vec3],
    target_base: Optional[Vec3],
    tuple3,
    vec_norm,
    vector_minus,
    pick_demo_env_float,
    direct_pregrasp_gate_caps,
    time_monotonic,
) -> dict:
    """Calcula métricas de consistencia de pose para gating runtime.

    Función pura extraída de la closure ``run_pick_demo._pose_consistency_metrics``.
    Sin acceso a globals: todas las dependencias (helpers + panel/cache)
    se inyectan como argumentos. Retorna el dict con tolerancias derivadas
    de env (vía ``pick_demo_env_float``), edades de las tres fuentes
    (panel_fk / trace / joint_state) y banderas booleanas para el gate.

    El caller resuelve ``tcp_base`` antes de invocar (típicamente con
    ``_live_tcp_base()`` del scope) para evitar tener que importar el
    closure aquí.
    """
    tcp_base_3 = tuple3(tcp_base)
    panel_fk_3 = tuple3(panel_fk_base) or tuple3(getattr(panel, "_last_tcp_base", None))
    panel_trace_3 = tuple3(panel_trace_base) or tuple3(getattr(panel, "_last_trace_tcp_base", None))
    target_base_3 = tuple3(target_base)
    pose_caps = direct_pregrasp_gate_caps(phase)
    now_mono = float(time_monotonic())
    fk_live_delta = vec_norm(vector_minus(panel_fk_3, tcp_base_3))
    trace_live_delta = vec_norm(vector_minus(panel_trace_3, tcp_base_3))
    fk_trace_delta = vec_norm(vector_minus(panel_fk_3, panel_trace_3))
    target_live_delta = vec_norm(vector_minus(target_base_3, tcp_base_3))
    source_tol = pick_demo_env_float(
        "PANEL_PICK_DEMO_POSE_SOURCE_TOL_M",
        0.006,
        minimum=0.003,
        maximum=(pose_caps or {}).get("source_tol_m"),
    )
    source_age_tol = pick_demo_env_float(
        "PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC",
        0.20,
        minimum=0.05,
        maximum=(pose_caps or {}).get("source_age_tol_sec"),
    )
    source_sync_tol = pick_demo_env_float(
        "PANEL_PICK_DEMO_POSE_SOURCE_SYNC_TOL_SEC",
        0.20,
        minimum=0.02,
        maximum=(pose_caps or {}).get("source_sync_tol_sec"),
    )
    phase_jump_tol = pick_demo_env_float(
        "PANEL_PICK_DEMO_PHASE_JUMP_TOL_M",
        0.010,
        minimum=source_tol,
        maximum=(pose_caps or {}).get("phase_jump_tol_m"),
    )
    panel_fk_age_sec = None
    if float(getattr(panel, "_last_tcp_fk_ts", 0.0) or 0.0) > 0.0:
        panel_fk_age_sec = max(
            0.0,
            now_mono - float(getattr(panel, "_last_tcp_fk_ts", 0.0) or 0.0),
        )
    trace_age_sec = None
    if float(getattr(panel, "_last_trace_tcp_ts", 0.0) or 0.0) > 0.0:
        trace_age_sec = max(
            0.0,
            now_mono - float(getattr(panel, "_last_trace_tcp_ts", 0.0) or 0.0),
        )
    # TF bridge latency: age of the TF message stamp from Gazebo's perspective.
    tf_stamp_age_sec = None
    _tf_stamp_ns = int(getattr(panel, "_last_trace_tcp_tf_stamp_ns", 0) or 0)
    if _tf_stamp_ns > 0 and panel._ros_worker_started and panel.ros_worker is not None:
        try:
            _ros_now_ns = int(getattr(panel.ros_worker, "_last_clock_stamp_ns", 0) or 0)
            if _ros_now_ns > 0:
                tf_stamp_age_sec = max(0.0, (_ros_now_ns - _tf_stamp_ns) / 1_000_000_000.0)
        except Exception:
            pass
    joint_state_age_sec = None
    if panel._ros_worker_started and panel.ros_worker is not None:
        try:
            _payload, joint_ts = panel.ros_worker.get_last_joint_state()
        except Exception:
            joint_ts = 0.0
        if joint_ts:
            joint_state_age_sec = max(0.0, now_mono - float(joint_ts))
    source_sync_delta_sec = None
    if panel_fk_age_sec is not None and trace_age_sec is not None:
        source_sync_delta_sec = abs(
            float(getattr(panel, "_last_trace_tcp_ts", 0.0) or 0.0)
            - float(getattr(panel, "_last_tcp_fk_ts", 0.0) or 0.0)
        )
    phase_end_delta = None
    phase_end_age_sec = None
    if phase:
        cached = (phase_pose_cache.get("phase_end") or {}).get(str(phase))
        if cached and tcp_base_3 is not None:
            phase_end_delta = vec_norm(vector_minus(tcp_base_3, cached.get("tcp_base")))
            phase_end_age_sec = max(
                0.0,
                float(time_monotonic()) - float(cached.get("mono") or 0.0),
            )
    fk_live_ok = fk_live_delta is None or float(fk_live_delta) <= source_tol
    trace_live_ok = trace_live_delta is None or float(trace_live_delta) <= source_tol
    panel_fk_fresh_ok = panel_fk_age_sec is None or float(panel_fk_age_sec) <= source_age_tol
    trace_fresh_ok = trace_age_sec is None or float(trace_age_sec) <= source_age_tol
    joint_state_fresh_ok = (
        joint_state_age_sec is None
        or float(joint_state_age_sec) <= source_age_tol
    )
    source_sync_ok = (
        source_sync_delta_sec is None
        or float(source_sync_delta_sec) <= source_sync_tol
    )
    sources_fresh_ok = bool(
        panel_fk_fresh_ok
        and trace_fresh_ok
        and joint_state_fresh_ok
        and source_sync_ok
    )
    phase_jump_ok = phase_end_delta is None or float(phase_end_delta) <= phase_jump_tol
    sources_ok = bool(fk_live_ok and trace_live_ok and sources_fresh_ok)
    ok_for_gate = bool(sources_ok and phase_jump_ok)
    return {
        "phase": str(phase or "none"),
        "tcp_base": tcp_base_3,
        "panel_fk_base": panel_fk_3,
        "panel_trace_base": panel_trace_3,
        "target_base": target_base_3,
        "fk_live_delta_m": fk_live_delta,
        "trace_live_delta_m": trace_live_delta,
        "fk_trace_delta_m": fk_trace_delta,
        "target_live_delta_m": target_live_delta,
        "phase_end_delta_m": phase_end_delta,
        "phase_end_age_sec": phase_end_age_sec,
        "source_tol_m": float(source_tol),
        "source_age_tol_sec": float(source_age_tol),
        "source_sync_tol_sec": float(source_sync_tol),
        "phase_jump_tol_m": float(phase_jump_tol),
        "panel_fk_age_sec": panel_fk_age_sec,
        "trace_age_sec": trace_age_sec,
        "tf_stamp_age_sec": tf_stamp_age_sec,
        "joint_state_age_sec": joint_state_age_sec,
        "source_sync_delta_sec": source_sync_delta_sec,
        "fk_live_ok": bool(fk_live_ok),
        "trace_live_ok": bool(trace_live_ok),
        "panel_fk_fresh_ok": bool(panel_fk_fresh_ok),
        "trace_fresh_ok": bool(trace_fresh_ok),
        "joint_state_fresh_ok": bool(joint_state_fresh_ok),
        "source_sync_ok": bool(source_sync_ok),
        "sources_fresh_ok": bool(sources_fresh_ok),
        "phase_jump_ok": bool(phase_jump_ok),
        "sources_ok": bool(sources_ok),
        "ok_for_gate": bool(ok_for_gate),
        "pose_gate_profile": (
            "pregrasp_strict"
            if pose_caps is not None
            else "runtime_profile"
        ),
    }
