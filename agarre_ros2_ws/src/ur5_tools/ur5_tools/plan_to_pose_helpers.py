#!/usr/bin/env python3
"""F10 (auditoría 2026-05-10): helpers puros de plan_to_pose_server.

Lógica de parsing y selección de parámetros extraída del nodo
``PlanToPoseServer`` para que sea testeable sin ROS. El nodo mantiene
los wrappers que añaden TF lookup / logging / lectura de params, y
delega el cálculo a estas funciones puras.

Funciones públicas:
  * ``extract_ordered_joint_positions(joint_state_payload, ordered_names)``
    → ``(positions_list, missing_name)``. Dado un payload con keys
    ``"name"`` y ``"position"`` y una lista canónica de joints,
    devuelve las positions reordenadas o ``(None, joint_que_falta)``.
  * ``select_traj_duration_and_timeout(dist_m, *, default_duration_sec,
    default_timeout_sec, long_dist_threshold_m=0.4, long_min_duration_sec=25.0,
    long_min_timeout_sec=120.0, short_min_duration_sec=8.0,
    short_min_timeout_sec=30.0)`` → ``(duration_eff, timeout_eff, is_long)``.
    Política de selección de duración/timeout basada en distancia al
    target.
  * ``parse_plan_to_pose_request(request_obj, *, default_ee_frame='rg2_pinch_center')``
    → tuple ``(target_xyz, target_quat_xyzw, ee_frame, cartesian, timeout_sec)``.
    Extrae los campos relevantes de un objeto request duck-typed con
    ``.target_pose_base.{position,orientation}`` y demás.
"""
from __future__ import annotations

from typing import Any, List, Mapping, Optional, Sequence, Tuple


def extract_ordered_joint_positions(
    joint_state_payload: Optional[Mapping[str, Any]],
    ordered_names: Sequence[str],
) -> Tuple[Optional[List[float]], Optional[str]]:
    """Reordena positions según ``ordered_names``.

    Args:
        joint_state_payload: dict con keys ``"name"`` (sequence[str])
            y ``"position"`` (sequence[float]). ``None`` o sin keys
            devuelve ``(None, "no_payload")``.
        ordered_names: lista canónica de joints esperados.

    Returns:
        ``(positions, None)`` si todos los joints están presentes.
        ``(None, missing_name)`` si falta cualquier joint.
        ``(None, "no_payload")`` si el payload está vacío.
    """
    if joint_state_payload is None:
        return None, "no_payload"
    names = joint_state_payload.get("name") if hasattr(joint_state_payload, "get") else None
    positions = joint_state_payload.get("position") if hasattr(joint_state_payload, "get") else None
    if not names or not positions:
        return None, "no_payload"
    name_to_pos = {str(n): float(p) for n, p in zip(names, positions)}
    out: List[float] = []
    for jn in ordered_names:
        if jn not in name_to_pos:
            return None, jn
        out.append(name_to_pos[jn])
    return out, None


def select_traj_duration_and_timeout(
    dist_m: float,
    *,
    default_duration_sec: float,
    default_timeout_sec: float,
    long_dist_threshold_m: float = 0.4,
    long_min_duration_sec: float = 25.0,
    long_min_timeout_sec: float = 120.0,
    short_min_duration_sec: float = 8.0,
    short_min_timeout_sec: float = 30.0,
) -> Tuple[float, float, bool]:
    """Selecciona duration/timeout efectivos según distancia al target.

    Política replicada de ``PlanToPoseServer._fjt_compute_traj_params``:

    * dist > ``long_dist_threshold_m`` (default 0.4m → TRANSPORT, drop):
      ``duration = max(default_duration_sec, long_min_duration_sec)``,
      ``timeout = max(default_timeout_sec, long_min_timeout_sec)``,
      ``is_long = True``.
    * dist <= threshold (APPROACH/GRASP_DOWN/LIFT cortos):
      ``duration = max(default_duration_sec, short_min_duration_sec)``,
      ``timeout = max(default_timeout_sec, short_min_timeout_sec)``,
      ``is_long = False``.

    El caller pasa el ``dist_m`` ya calculado vía TF; esta función
    no toca TF — es policy pura. Si TF no está disponible, el caller
    puede pasar un valor conservador (e.g. ``0.5``) para forzar la
    rama "long".

    Args:
        dist_m: distancia euclidiana en metros al target.
        default_duration_sec: duración base configurada por param ROS.
        default_timeout_sec: timeout base configurado por param ROS.
        long_dist_threshold_m: límite que separa "corto" de "largo".
        long_min_duration_sec: mínimo absoluto para rutas largas.
        long_min_timeout_sec: idem para timeout largo.
        short_min_duration_sec: mínimo absoluto para rutas cortas.
        short_min_timeout_sec: idem para timeout corto.

    Returns:
        ``(duration_eff, timeout_eff, is_long)``.
    """
    is_long = dist_m > long_dist_threshold_m
    if is_long:
        duration_eff = max(float(default_duration_sec), long_min_duration_sec)
        timeout_eff = max(float(default_timeout_sec), long_min_timeout_sec)
    else:
        duration_eff = max(float(default_duration_sec), short_min_duration_sec)
        timeout_eff = max(float(default_timeout_sec), short_min_timeout_sec)
    return duration_eff, timeout_eff, is_long


def parse_plan_to_pose_request(
    request_obj: Any,
    *,
    default_ee_frame: str = "rg2_pinch_center",
) -> Tuple[
    Tuple[float, float, float],
    Tuple[float, float, float, float],
    str,
    bool,
    float,
]:
    """Extrae campos relevantes de un ``PlanToPose.Goal`` request.

    Acepta cualquier objeto duck-typed con:
      * ``request_obj.target_pose_base.position.{x, y, z}``
      * ``request_obj.target_pose_base.orientation.{x, y, z, w}``
      * ``request_obj.ee_frame`` (str, opcional)
      * ``request_obj.cartesian`` (bool, opcional)
      * ``request_obj.timeout_sec`` (float, opcional)

    Returns:
        ``(target_xyz, target_quat_xyzw, ee_frame, cartesian, timeout_sec)``
    """
    pose = request_obj.target_pose_base
    pos = pose.position
    ori = pose.orientation
    target_xyz = (float(pos.x), float(pos.y), float(pos.z))
    target_quat = (float(ori.x), float(ori.y), float(ori.z), float(ori.w))
    ee_frame = (
        str(getattr(request_obj, "ee_frame", "") or "").strip()
        or default_ee_frame
    )
    cartesian = bool(getattr(request_obj, "cartesian", False))
    timeout_sec = float(getattr(request_obj, "timeout_sec", 0.0) or 0.0)
    return target_xyz, target_quat, ee_frame, cartesian, timeout_sec


__all__ = [
    "extract_ordered_joint_positions",
    "select_traj_duration_and_timeout",
    "parse_plan_to_pose_request",
]
