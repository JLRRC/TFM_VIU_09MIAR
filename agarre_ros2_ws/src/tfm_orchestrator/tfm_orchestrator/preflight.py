#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/preflight.py
# Contenido: F12-step2c.1 — Lógica pura de las fases INITIAL_SNAPSHOT y HOME_INITIAL.
"""Preflight: snapshot inicial + validación de pose HOME.

Módulo puro (sin imports ROS) que extrae al orchestrator la lógica
testeable de las dos primeras fases granulares del pipeline pick &
place:

* ``INITIAL_SNAPSHOT`` — captura del estado inicial (TCP base, joints,
  pose del objeto target). Equivalente al ``_emit_initial_snapshot_trace``
  del legacy ``panel_pick_demo.run_pick_demo`` (línea 802).

* ``HOME_INITIAL`` — comprobación de que los joints alcanzaron la pose
  HOME canónica con tolerancia angular. Equivalente al
  ``_emit_home_initial_trace`` (línea 809) tras un movimiento joint.

Diseño:

* ``InitialSnapshot`` — dataclass frozen con los campos del snapshot.
* ``compose_initial_snapshot(...)`` — constructor que valida tipos,
  redondea, e infiere `dist3d` si no se proporciona.
* ``HomeTargetCheck`` — resultado de validar la pose HOME.
* ``validate_home_target(target, current, tol_rad)`` — comparación
  joint a joint con tolerancia angular en radianes.

El orchestrator (LifecycleNode F9) llama estas funciones desde su
``_execute_callback``. Los movimientos físicos (publicar trayectoria,
esperar a controller) se delegan a service/action calls separados;
este módulo no toca ROS ni tiempos reales.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple


Vec3 = Tuple[float, float, float]


# ---------------------------------------------------------------------------
# INITIAL_SNAPSHOT
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class InitialSnapshot:
    """Snapshot del estado del sistema al inicio del pipeline pick.

    Campos:
        request_id: ID único de la sesión (correlación logs).
        tcp_base: posición del TCP en frame ``base_link`` (x, y, z) o None.
        object_base: posición del objeto target en ``base_link`` o None.
        dx, dy, dz: vector ``object_base - tcp_base`` o None si alguno falta.
        dist3d: norma euclídea ``|object_base - tcp_base|`` o None.
        joints: dict joint_name → posición rad. Vacío si no hay datos.
        pose_source: identificador del origen de la pose (ej. ``tf_live``).
        flow_name: ``DIRECT`` / ``MOVEIT`` / ``REMOTE``.
    """

    request_id: str = ""
    tcp_base: Optional[Vec3] = None
    object_base: Optional[Vec3] = None
    dx: Optional[float] = None
    dy: Optional[float] = None
    dz: Optional[float] = None
    dist3d: Optional[float] = None
    joints: Dict[str, float] = field(default_factory=dict)
    pose_source: str = ""
    flow_name: str = "DIRECT"

    def is_complete(self) -> bool:
        """True si TCP y objeto están presentes y la distancia es finita."""
        if self.tcp_base is None or self.object_base is None:
            return False
        if self.dist3d is None:
            return False
        return math.isfinite(self.dist3d)

    def to_log_dict(self) -> Dict[str, object]:
        """Serializa el snapshot a un dict listo para log estructurado."""
        return {
            "request_id": self.request_id,
            "tcp_base": list(self.tcp_base) if self.tcp_base is not None else None,
            "object_base": list(self.object_base) if self.object_base is not None else None,
            "dx": self.dx,
            "dy": self.dy,
            "dz": self.dz,
            "dist3d": self.dist3d,
            "joints": dict(self.joints),
            "pose_source": self.pose_source,
            "flow_name": self.flow_name,
        }


def _coerce_vec3(value: Any) -> Optional[Vec3]:
    """Convierte entradas (list/tuple de 3 numbers) en ``Vec3`` o None."""
    if value is None:
        return None
    if not isinstance(value, (list, tuple)):
        return None
    if len(value) < 3:
        return None
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError):
        return None


def _coerce_joints(value: Any) -> Dict[str, float]:
    """Convierte un dict-like de joints en dict[str, float] saneado."""
    if value is None:
        return {}
    if not isinstance(value, dict):
        return {}
    out: Dict[str, float] = {}
    for k, v in value.items():
        try:
            out[str(k)] = float(v)
        except (TypeError, ValueError):
            continue
    return out


def _compute_dist3d(tcp: Optional[Vec3], obj: Optional[Vec3]) -> Optional[float]:
    """Calcula ``|obj - tcp|`` cuando ambos están presentes."""
    if tcp is None or obj is None:
        return None
    return math.sqrt(
        (obj[0] - tcp[0]) ** 2
        + (obj[1] - tcp[1]) ** 2
        + (obj[2] - tcp[2]) ** 2
    )


def compose_initial_snapshot(
    *,
    request_id: str = "",
    tcp_base: Any = None,
    object_base: Any = None,
    joints: Any = None,
    pose_source: str = "",
    flow_name: str = "DIRECT",
) -> InitialSnapshot:
    """Construye un ``InitialSnapshot`` validando y normalizando entradas.

    El cálculo de ``dx``, ``dy``, ``dz`` y ``dist3d`` se hace aquí; el
    caller no necesita preocuparse de ellos. Si alguno de ``tcp_base`` o
    ``object_base`` es None o malformado, los deltas serán None.
    """
    tcp = _coerce_vec3(tcp_base)
    obj = _coerce_vec3(object_base)
    js = _coerce_joints(joints)

    dx: Optional[float]
    dy: Optional[float]
    dz: Optional[float]
    dist: Optional[float]
    if tcp is not None and obj is not None:
        dx = obj[0] - tcp[0]
        dy = obj[1] - tcp[1]
        dz = obj[2] - tcp[2]
        dist = _compute_dist3d(tcp, obj)
    else:
        dx = None
        dy = None
        dz = None
        dist = None

    return InitialSnapshot(
        request_id=str(request_id or ""),
        tcp_base=tcp,
        object_base=obj,
        dx=dx,
        dy=dy,
        dz=dz,
        dist3d=dist,
        joints=js,
        pose_source=str(pose_source or ""),
        flow_name=str(flow_name or "DIRECT"),
    )


# ---------------------------------------------------------------------------
# HOME_INITIAL
# ---------------------------------------------------------------------------


# Pose HOME canónica del UR5 (joints en radianes). Coincide con la usada
# por el legacy ``run_pick_demo`` en panel_pick_demo.py para el
# movimiento HOME_INITIAL previo a APPROACH_COARSE.
UR5_HOME_JOINTS_RAD: Tuple[float, ...] = (
    0.0,
    -math.pi / 2.0,
    0.0,
    -math.pi / 2.0,
    0.0,
    0.0,
)

# Joints UR5 estándar (orden FK).
UR5_JOINT_NAMES: Tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def _angle_shortest_diff_rad(a: float, b: float) -> float:
    """Diferencia angular más corta entre ``a`` y ``b`` en [-π, π]."""
    diff = (a - b) % (2.0 * math.pi)
    if diff > math.pi:
        diff -= 2.0 * math.pi
    return diff


@dataclass(frozen=True)
class HomeTargetCheck:
    """Resultado de comparar joints actuales contra el target HOME.

    Campos:
        target_reached: True si todos los joints están dentro de tol_rad.
        max_diff_rad: máxima diferencia angular detectada.
        per_joint_diffs: dict joint → |diff_rad| ordenado por nombre.
        missing_joints: lista de nombres ausentes en ``current``.
        reason: descripción (``ok``, ``missing:<joint>``, ``out_of_tol:<j>``).
    """

    target_reached: bool
    max_diff_rad: float
    per_joint_diffs: Dict[str, float] = field(default_factory=dict)
    missing_joints: List[str] = field(default_factory=list)
    reason: str = "ok"


def validate_home_target(
    current_joints: Optional[Dict[str, float]],
    *,
    target: Sequence[float] = UR5_HOME_JOINTS_RAD,
    joint_names: Sequence[str] = UR5_JOINT_NAMES,
    tol_rad: float = 0.02,
) -> HomeTargetCheck:
    """Valida si ``current_joints`` está dentro de tolerancia del HOME.

    Parameters
    ----------
    current_joints
        Snapshot de los joints actuales (``{nombre: rad}``). Si es None
        o vacío, devuelve ``target_reached=False`` y reason=``no_joints``.
    target
        Pose target en radianes (default: ``UR5_HOME_JOINTS_RAD``).
    joint_names
        Nombres de los joints en el orden de ``target``.
    tol_rad
        Tolerancia angular por joint (default 0.02 rad ≈ 1.15°).

    Returns
    -------
    HomeTargetCheck
        Estructura inmutable con el resultado.
    """
    if not current_joints:
        return HomeTargetCheck(
            target_reached=False,
            max_diff_rad=float("inf"),
            per_joint_diffs={},
            missing_joints=list(joint_names),
            reason="no_joints",
        )

    if len(target) != len(joint_names):
        return HomeTargetCheck(
            target_reached=False,
            max_diff_rad=float("inf"),
            reason=f"target_arity_mismatch:{len(target)}!={len(joint_names)}",
        )

    diffs: Dict[str, float] = {}
    missing: List[str] = []
    max_diff = 0.0
    first_oot: Optional[str] = None

    for name, tgt in zip(joint_names, target):
        if name not in current_joints:
            missing.append(name)
            continue
        diff = abs(_angle_shortest_diff_rad(float(current_joints[name]), float(tgt)))
        diffs[name] = diff
        if diff > max_diff:
            max_diff = diff
        if diff > float(tol_rad) and first_oot is None:
            first_oot = name

    if missing:
        return HomeTargetCheck(
            target_reached=False,
            max_diff_rad=max_diff,
            per_joint_diffs=diffs,
            missing_joints=missing,
            reason=f"missing:{','.join(missing)}",
        )

    if first_oot is not None:
        return HomeTargetCheck(
            target_reached=False,
            max_diff_rad=max_diff,
            per_joint_diffs=diffs,
            reason=f"out_of_tol:{first_oot}={diffs[first_oot]:.4f}>{tol_rad:.4f}",
        )

    return HomeTargetCheck(
        target_reached=True,
        max_diff_rad=max_diff,
        per_joint_diffs=diffs,
        reason="ok",
    )
