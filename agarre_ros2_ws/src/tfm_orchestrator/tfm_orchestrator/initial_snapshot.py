#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/initial_snapshot.py
# Contenido: B-iter5 (2026-05-03) — captura real de INITIAL_SNAPSHOT en orchestrator.
"""B-iter5 — INITIAL_SNAPSHOT real del PickOrchestrator.

Reemplaza el scaffold (B-iter2) por una captura real del estado inicial
del robot + objeto target, llamando a fuentes ROS:

* TF (vía ``tf2_ros.Buffer``) → pose del TCP en base_link
* JointState (cache de subscriber) → 6 joint positions
* Service ``/orchestrator/resolve_object_pose_world`` → pose del objeto

Diseño puro/inyectable: el helper ``capture_initial_snapshot`` recibe
todas sus dependencias como callables/objetos, lo que permite tests
offline con mocks sin levantar ROS.

Devuelve un ``InitialSnapshotResult`` con:
  - tcp_pose_base: (x,y,z,qx,qy,qz,qw) en base_link, o None si TF stale.
  - joint_positions: tuple de 6 floats (rad), o None si joint cache vacío.
  - object_pose_world: (x,y,z,qx,qy,qz,qw) en world, o None si resolver fail.
  - success: bool. True si los 3 capturados están disponibles.
  - reason: detalle textual.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional, Tuple


Tuple7 = Tuple[float, float, float, float, float, float, float]
Tuple6 = Tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class InitialSnapshotResult:
    """Resultado de la captura inicial."""

    success: bool
    reason: str
    tcp_pose_base: Optional[Tuple7] = None
    joint_positions: Optional[Tuple6] = None
    object_pose_world: Optional[Tuple7] = None


def _transform_to_tuple7(transform_msg: Any) -> Optional[Tuple7]:
    """Convierte ``geometry_msgs/TransformStamped`` o ``Transform`` a tuple7.

    Devuelve None si el msg es None o no expone los atributos esperados.
    """
    if transform_msg is None:
        return None
    try:
        # TransformStamped tiene .transform; Transform es directo.
        tr = getattr(transform_msg, "transform", transform_msg)
        t = tr.translation
        r = tr.rotation
        return (
            float(t.x), float(t.y), float(t.z),
            float(r.x), float(r.y), float(r.z), float(r.w),
        )
    except (AttributeError, TypeError, ValueError):
        return None


def _pose_msg_to_tuple7(pose_msg: Any) -> Optional[Tuple7]:
    """Convierte ``geometry_msgs/Pose`` a tuple7. Equivalente al helper de
    phase_dispatch — duplicado aquí para no crear dependencia circular."""
    if pose_msg is None:
        return None
    try:
        pos = pose_msg.position
        ori = pose_msg.orientation
        return (
            float(pos.x), float(pos.y), float(pos.z),
            float(ori.x), float(ori.y), float(ori.z), float(ori.w),
        )
    except (AttributeError, TypeError, ValueError):
        return None


def capture_tcp_pose_base(
    tf_lookup: Callable[..., Any],
    *,
    base_frame: str = "base_link",
    tcp_frame: str = "rg2_tcp",
    timeout_sec: float = 0.5,
) -> Tuple[Optional[Tuple7], str]:
    """Lookup del TCP en base_link via TF.

    Parameters:
        tf_lookup: callable equivalente a ``tf2_ros.Buffer.lookup_transform``,
            firma ``(target_frame, source_frame, time, timeout) -> TransformStamped``.
        base_frame: frame destino (target).
        tcp_frame: frame origen (source). Default "rg2_tcp" (tip_link SRDF).
        timeout_sec: timeout del lookup.

    Returns:
        (tuple7_or_none, reason_text). tuple7 es la pose; reason vacío si OK
        o descripción del error si None.
    """
    try:
        from rclpy.duration import Duration  # lazy
    except ImportError:
        Duration = None  # type: ignore

    try:
        from rclpy.time import Time
        timeout = (
            Duration(seconds=float(timeout_sec))
            if Duration is not None
            else None
        )
        if timeout is not None:
            ts = tf_lookup(base_frame, tcp_frame, Time(), timeout)
        else:
            ts = tf_lookup(base_frame, tcp_frame)
    except Exception as exc:
        return None, f"tf_lookup_exception:{type(exc).__name__}:{exc}"

    tup = _transform_to_tuple7(ts)
    if tup is None:
        return None, "tf_transform_unparseable"
    return tup, "ok"


def extract_joint_positions(
    joint_state_msg: Any,
    *,
    joint_names_order: Tuple[str, ...] = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ),
) -> Tuple[Optional[Tuple6], str]:
    """Extrae los 6 joint positions del UR5 en el orden canónico SRDF.

    Acepta un ``sensor_msgs/JointState`` con campos ``name`` y ``position``.
    Si el msg es None, falta algún joint, o las longitudes no cuadran, devuelve
    None y un reason descriptivo.
    """
    if joint_state_msg is None:
        return None, "joint_state_msg_none"
    names = list(getattr(joint_state_msg, "name", []) or [])
    positions = list(getattr(joint_state_msg, "position", []) or [])
    if len(names) != len(positions):
        return None, f"joint_state_len_mismatch:{len(names)}!={len(positions)}"
    name_to_pos = {str(n): float(p) for n, p in zip(names, positions)}
    missing = [j for j in joint_names_order if j not in name_to_pos]
    if missing:
        return None, f"joint_state_missing:{','.join(missing)}"
    try:
        ordered = tuple(float(name_to_pos[j]) for j in joint_names_order)
    except (TypeError, ValueError) as exc:
        return None, f"joint_state_value_exception:{type(exc).__name__}:{exc}"
    return ordered, "ok"


def capture_initial_snapshot(
    *,
    object_name: str,
    tf_lookup: Optional[Callable[..., Any]],
    joint_state_msg: Any,
    resolve_object_pose: Optional[Callable[[str], Any]],
    base_frame: str = "base_link",
    tcp_frame: str = "rg2_tcp",
    tf_timeout_sec: float = 0.5,
    require_object_pose: bool = True,
) -> InitialSnapshotResult:
    """Captura los 3 estados iniciales del ciclo pick.

    Función pura: las 3 fuentes (TF lookup, joint cache, object resolver)
    se inyectan como callables/datos. Permite tests offline 100% mocks.

    Parameters:
        object_name: nombre del objeto target (vacío = no resolver pose).
        tf_lookup: ``tf2_ros.Buffer.lookup_transform`` o equivalente. None ⇒ skip TF.
        joint_state_msg: último ``sensor_msgs/JointState`` cacheado. None ⇒ skip joints.
        resolve_object_pose: callable ``(name) -> Response`` con `pose_world`,
            `success`, `detail`. None ⇒ skip resolver.
        base_frame: frame target del TF lookup.
        tcp_frame: frame source del TF lookup (default "rg2_tcp" = SRDF tip).
        tf_timeout_sec: timeout del lookup.
        require_object_pose: si False, success=True aunque no se resuelva pose objeto.

    Returns:
        InitialSnapshotResult con los 3 valores capturados (None los que fallaron).
    """
    reasons = []

    # 1. TCP pose en base_link
    tcp_pose: Optional[Tuple7] = None
    if tf_lookup is None:
        reasons.append("tcp:no_tf_lookup_provided")
    else:
        tcp_pose, tcp_reason = capture_tcp_pose_base(
            tf_lookup,
            base_frame=base_frame,
            tcp_frame=tcp_frame,
            timeout_sec=tf_timeout_sec,
        )
        if tcp_pose is None:
            reasons.append(f"tcp:{tcp_reason}")

    # 2. Joint positions
    joints: Optional[Tuple6] = None
    joints, joint_reason = extract_joint_positions(joint_state_msg)
    if joints is None:
        reasons.append(f"joints:{joint_reason}")

    # 3. Object pose en world
    object_pose: Optional[Tuple7] = None
    name = str(object_name or "").strip()
    if not name:
        if require_object_pose:
            reasons.append("object:empty_name")
    elif resolve_object_pose is None:
        if require_object_pose:
            reasons.append("object:no_resolver_provided")
    else:
        try:
            response = resolve_object_pose(name)
        except Exception as exc:
            response = None
            reasons.append(f"object:resolve_exception:{type(exc).__name__}:{exc}")
        if response is not None:
            success = bool(getattr(response, "success", False))
            if success:
                object_pose = _pose_msg_to_tuple7(getattr(response, "pose_world", None))
                if object_pose is None:
                    reasons.append("object:pose_unparseable")
            else:
                detail = str(getattr(response, "detail", "")) or "unknown"
                if require_object_pose:
                    reasons.append(f"object:resolver_failed:{detail}")

    # Decisión de success.
    snapshot_ok = (
        tcp_pose is not None
        and joints is not None
        and (object_pose is not None or not require_object_pose)
    )
    if snapshot_ok:
        reason = "snapshot_ok"
    else:
        reason = "snapshot_partial:" + "|".join(reasons) if reasons else "snapshot_failed"

    return InitialSnapshotResult(
        success=snapshot_ok,
        reason=reason,
        tcp_pose_base=tcp_pose,
        joint_positions=joints,
        object_pose_world=object_pose,
    )
