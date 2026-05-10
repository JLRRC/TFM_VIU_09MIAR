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
    """Resultado de la captura inicial.

    B-iter10: campos ``tcp_tf_age_sec`` y ``joint_state_age_sec`` opcionales
    se rellenan si el caller proveyó ``now_provider`` y los msgs tenían
    header.stamp. Permiten al caller invocar
    ``pose_consistency.evaluate_pose_freshness_gate`` post-captura.
    """

    success: bool
    reason: str
    tcp_pose_base: Optional[Tuple7] = None
    joint_positions: Optional[Tuple6] = None
    object_pose_world: Optional[Tuple7] = None
    tcp_tf_age_sec: Optional[float] = None
    joint_state_age_sec: Optional[float] = None


def _stamp_to_seconds(stamp_msg: Any) -> Optional[float]:
    """Convierte ``builtin_interfaces/Time`` (sec, nanosec) a float seconds.

    Devuelve None si stamp_msg no expone los atributos esperados.
    """
    if stamp_msg is None:
        return None
    try:
        sec = int(getattr(stamp_msg, "sec", 0))
        nsec = int(getattr(stamp_msg, "nanosec", 0))
        return float(sec) + float(nsec) / 1_000_000_000.0
    except (TypeError, ValueError):
        return None


def compute_msg_age_sec(
    msg_with_header: Any,
    now_sec: Optional[float],
) -> Optional[float]:
    """Edad (s) de un msg ROS con .header.stamp dada la hora actual now_sec.

    Devuelve None si:
      - msg es None, o
      - msg.header.stamp no tiene .sec/.nanosec, o
      - now_sec es None.

    Se permite age negativo (clock skew) — el caller decide qué hacer.
    """
    if msg_with_header is None or now_sec is None:
        return None
    header = getattr(msg_with_header, "header", None)
    if header is None:
        return None
    stamp_sec = _stamp_to_seconds(getattr(header, "stamp", None))
    if stamp_sec is None:
        return None
    return float(now_sec) - float(stamp_sec)


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


def _resolve_tf_time_and_timeout(timeout_sec: float) -> Tuple[Any, Any]:
    """Devuelve ``(time_arg, timeout_arg)`` para ``tf2_ros.Buffer.lookup_transform``.

    Si rclpy está disponible (entorno ROS): ``(rclpy.time.Time(), Duration(...))``.
    Si NO está disponible (entorno tests offline): ``(None, None)`` y el
    caller invocará el lookup sin esos kwargs. El mock de tests acepta
    ambos; el buffer real ignora None en time → comportamiento equivalente.

    Bug fix (2026-05-10): hasta esta fecha el import de rclpy estaba
    DENTRO del mismo try que envolvía la llamada a tf_lookup, así que
    una ImportError en entorno offline se reportaba como
    "tf_lookup_exception:ModuleNotFoundError". Ahora aislamos la
    resolución de los args para que la importabilidad de rclpy NO
    enmascare un fallo real del lookup.
    """
    try:
        from rclpy.duration import Duration
        from rclpy.time import Time
        return Time(), Duration(seconds=float(timeout_sec))
    except ImportError:
        return None, None


def _invoke_tf_lookup(
    tf_lookup: Callable[..., Any],
    base_frame: str,
    tcp_frame: str,
    timeout_sec: float,
) -> Any:
    """Llama a ``tf_lookup`` con los args correctos según disponibilidad rclpy."""
    time_arg, timeout_arg = _resolve_tf_time_and_timeout(timeout_sec)
    if time_arg is None and timeout_arg is None:
        return tf_lookup(base_frame, tcp_frame)
    return tf_lookup(base_frame, tcp_frame, time_arg, timeout_arg)


def _capture_tcp_with_msg(
    tf_lookup: Callable[..., Any],
    *,
    base_frame: str,
    tcp_frame: str,
    timeout_sec: float,
) -> Tuple[Optional[Tuple7], str, Any]:
    """Variante interna que también devuelve el TransformStamped raw para
    poder calcular age desde header.stamp."""
    try:
        ts = _invoke_tf_lookup(tf_lookup, base_frame, tcp_frame, timeout_sec)
    except Exception as exc:
        return None, f"tf_lookup_exception:{type(exc).__name__}:{exc}", None
    tup = _transform_to_tuple7(ts)
    if tup is None:
        return None, "tf_transform_unparseable", ts
    return tup, "ok", ts


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
        ts = _invoke_tf_lookup(tf_lookup, base_frame, tcp_frame, timeout_sec)
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
        vals = [float(name_to_pos[j]) for j in joint_names_order]
    except (TypeError, ValueError) as exc:
        return None, f"joint_state_value_exception:{type(exc).__name__}:{exc}"
    if len(vals) != 6:
        return None, f"joint_state_unexpected_count:{len(vals)}"
    # F7-step1.9: cast explícito a Tuple6 fijo (mypy strict no infiere
    # length-6 desde generator → tuple).
    ordered: Tuple6 = (vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])
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
    now_sec: Optional[float] = None,
    last_tf_lookup_msg: Any = None,
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
    tcp_lookup_msg: Any = None
    if tf_lookup is None:
        reasons.append("tcp:no_tf_lookup_provided")
    else:
        tcp_pose, tcp_reason, tcp_lookup_msg = _capture_tcp_with_msg(
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

    # B-iter10: calcular ages opcionales si el caller proveyó now_sec.
    tcp_age_sec: Optional[float] = None
    js_age_sec: Optional[float] = None
    if now_sec is not None:
        tcp_age_sec = compute_msg_age_sec(tcp_lookup_msg, now_sec)
        js_age_sec = compute_msg_age_sec(joint_state_msg, now_sec)

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
        tcp_tf_age_sec=tcp_age_sec,
        joint_state_age_sec=js_age_sec,
    )
