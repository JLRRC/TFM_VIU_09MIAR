#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_models.py
"""F#13-step (2026-05-08) — Modelos de datos puros del gripper attach backend.

Extraídas de ``gripper_attach_backend.py:119-157`` para:
- Aislar las dataclasses del LifecycleNode (testing offline).
- Permitir reuso desde tests / scripts de análisis sin importar ROS.
- Reducir LOC del backend (1234 → ~1190 LOC).

Sin estado mutable global. Sin ROS. Tests en ``test/test_gripper_attach_models.py``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PoseSample:
    """Snapshot de una pose 6DOF con timestamp.

    Attributes:
        x, y, z: posición en metros.
        qx, qy, qz, qw: cuaternión orientación.
        stamp_ns: timestamp en nanosegundos (típicamente ROS clock).
    """

    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    stamp_ns: int


@dataclass
class AttachedTarget:
    """Objeto attached con offset relativo + cuaternión y timestamp.

    Attributes:
        name: nombre del objeto en Gazebo (e.g. "pick_demo").
        offset_x/y/z: offset relativo del objeto al TCP en el momento del attach.
        qx/y/z/w: orientación relativa cuaternión.
        attach_stamp_ns: timestamp del attach (para detectar staleness).
        coherence_breach_count: contador de breaches detectados (drift > tol).
            Si supera un umbral configurado, dispara detach automático.
    """

    name: str
    offset_x: float
    offset_y: float
    offset_z: float
    qx: float
    qy: float
    qz: float
    qw: float
    attach_stamp_ns: int
    coherence_breach_count: int = 0


@dataclass
class DemoTransportState:
    """Estado del modo demo transport (objeto sigue al TCP físicamente).

    Usado para coordinar la simulación de "agarre" en cycles E2E donde
    el objeto se "pega" al TCP via SetEntityPose en Gazebo (sin physics
    real de fricción).

    Attributes:
        name: nombre del objeto demo (típicamente "pick_demo").
        last_pose: última PoseSample observada del objeto.
        last_spawn_ts: timestamp del último spawn (segundos wall).
        world_offset_*: offset world-locked aplicado al objeto.
        world_q*: orientación world-locked.
        use_world_locked_pose: si True, usa pose world-locked en lugar
            de TCP-relative. Sirve para comparar drift en tests.
    """

    name: str
    last_pose: Optional[PoseSample] = None
    last_spawn_ts: float = 0.0
    world_offset_x: float = 0.0
    world_offset_y: float = 0.0
    world_offset_z: float = -0.1
    world_qx: float = 0.0
    world_qy: float = 0.0
    world_qz: float = 0.0
    world_qw: float = 1.0
    use_world_locked_pose: bool = True


def coherence_breach_exceeded(
    target: AttachedTarget,
    *,
    breach_threshold: int,
) -> bool:
    """True si el counter de breaches del attached target supera el umbral.

    Función pura: encapsula la regla de "cuándo disparar detach automático
    por drift sostenido".

    Args:
        target: AttachedTarget con counter actual.
        breach_threshold: umbral (>= 1). Counter == threshold dispara.

    Returns:
        True si ``target.coherence_breach_count >= breach_threshold``.
    """
    return int(target.coherence_breach_count) >= max(1, int(breach_threshold))


def increment_breach(target: AttachedTarget) -> AttachedTarget:
    """Devuelve un nuevo AttachedTarget con coherence_breach_count + 1.

    Función pura: no muta el target original. Útil para tests que quieren
    simular cycles de breach.
    """
    return AttachedTarget(
        name=target.name,
        offset_x=target.offset_x,
        offset_y=target.offset_y,
        offset_z=target.offset_z,
        qx=target.qx,
        qy=target.qy,
        qz=target.qz,
        qw=target.qw,
        attach_stamp_ns=target.attach_stamp_ns,
        coherence_breach_count=int(target.coherence_breach_count) + 1,
    )


def reset_breach(target: AttachedTarget) -> AttachedTarget:
    """Devuelve un nuevo AttachedTarget con coherence_breach_count = 0."""
    return AttachedTarget(
        name=target.name,
        offset_x=target.offset_x,
        offset_y=target.offset_y,
        offset_z=target.offset_z,
        qx=target.qx,
        qy=target.qy,
        qz=target.qz,
        qw=target.qw,
        attach_stamp_ns=target.attach_stamp_ns,
        coherence_breach_count=0,
    )
