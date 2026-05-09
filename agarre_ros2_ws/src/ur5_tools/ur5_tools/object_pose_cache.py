#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/object_pose_cache.py
# Contenido: F5-step4 — cache pura de poses indexadas por nombre.
"""Cache de poses world por nombre de objeto (F5-step4).

Consume actualizaciones de gz_pose_bridge (publica TFMessage en
``/world/<name>/pose/info`` con un TransformStamped por objeto) y
permite lookup por nombre con validación de frescura.

Diseño puro Python sin ROS — los conversores (``transform_to_pose7``,
``tfmessage_to_dict``) aceptan dicts/objetos genéricos. Los wrappers
ROS viven en ``object_pose_resolver_service.py``.

Esto facilita:
  * Tests unitarios sin ROS.
  * Reusabilidad (otros nodos pueden consumir el cache).
  * Una sola responsabilidad por archivo.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


@dataclass(frozen=True)
class CachedPose:
    """Snapshot de una pose en un instante dado.

    pose7 = (x, y, z, qx, qy, qz, qw).
    stamp_sec = tiempo monotónico (no wall) del último update.
    frame_id = frame world declarado por el publisher (info, no usado para lookup).
    """

    pose7: Tuple[float, float, float, float, float, float, float]
    stamp_sec: float
    frame_id: str = "world"


class ObjectPoseCache:
    """Cache mutable indexado por nombre de objeto.

    No es thread-safe — el caller (típicamente un nodo ROS con
    callback_group reentrant) debe sincronizar si hace falta. En la
    práctica, las actualizaciones llegan desde el callback de un único
    sub y los lookups desde el callback del service, ambos serializados
    por el executor de rclpy si se usa SingleThreadedExecutor.
    """

    def __init__(self) -> None:
        self._poses: Dict[str, CachedPose] = {}

    def update(self, name: str, pose7: Tuple[float, ...], stamp_sec: float,
               frame_id: str = "world") -> None:
        """Inserta o actualiza la pose de un objeto."""
        if not name:
            return
        if len(pose7) != 7:
            return
        try:
            normalized = tuple(float(v) for v in pose7)
        except (TypeError, ValueError):
            return
        self._poses[str(name)] = CachedPose(
            pose7=normalized,  # type: ignore[arg-type]
            stamp_sec=float(stamp_sec),
            frame_id=str(frame_id or "world"),
        )

    def lookup(
        self,
        name: str,
        *,
        now_sec: float,
        max_age_sec: float = 1.0,
    ) -> Tuple[Optional[CachedPose], str]:
        """Devuelve (pose, "") si fresca, o (None, reason) si no.

        Reasons:
          * "object_not_in_cache": nunca se ha visto.
          * "pose_stale: age=...s > max=...s": último update demasiado viejo.
        """
        cached = self._poses.get(str(name))
        if cached is None:
            return None, "object_not_in_cache"
        age = float(now_sec) - cached.stamp_sec
        if max_age_sec > 0.0 and age > max_age_sec:
            return None, f"pose_stale:age={age:.3f}>max={max_age_sec:.3f}"
        return cached, ""

    def known_objects(self) -> List[str]:
        """Lista de nombres conocidos (para debug / introspection)."""
        return sorted(self._poses.keys())

    def clear(self) -> None:
        self._poses.clear()


def transform_to_pose7(transform: Any) -> Optional[Tuple[float, ...]]:
    """Convierte un ``geometry_msgs/Transform`` a tuple7.

    Acepta cualquier objeto con ``translation.{x,y,z}`` y
    ``rotation.{x,y,z,w}``. Devuelve None ante atributos faltantes.
    """
    if transform is None:
        return None
    try:
        t = transform.translation
        r = transform.rotation
        return (
            float(t.x), float(t.y), float(t.z),
            float(r.x), float(r.y), float(r.z), float(r.w),
        )
    except (AttributeError, TypeError, ValueError):
        return None


def tfmessage_to_updates(
    tfmessage: Any,
    now_sec: float,
) -> List[Tuple[str, Tuple[float, ...], float, str]]:
    """Itera un TFMessage y devuelve (name, pose7, stamp_sec, frame_id).

    El stamp se toma del callback (now_sec) en lugar del header del
    msg porque gz_pose_bridge a veces entrega timestamps inconsistentes
    si el clock de Gazebo no está alineado. Para un service de "pose
    actual", el stamp del callback es más representativo.

    Acepta cualquier objeto con ``transforms`` iterable de
    TransformStamped (atributos: child_frame_id, header.frame_id, transform).
    """
    out: List[Tuple[str, Tuple[float, ...], float, str]] = []
    if tfmessage is None:
        return out
    try:
        tfs = tfmessage.transforms
    except AttributeError:
        return out
    for tfs_msg in tfs:
        try:
            name = str(tfs_msg.child_frame_id or "").strip()
        except AttributeError:
            continue
        if not name:
            continue
        pose7 = transform_to_pose7(getattr(tfs_msg, "transform", None))
        if pose7 is None:
            continue
        try:
            frame_id = str(tfs_msg.header.frame_id or "world")
        except AttributeError:
            frame_id = "world"
        out.append((name, pose7, float(now_sec), frame_id))
    return out
