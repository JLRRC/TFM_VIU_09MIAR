#!/usr/bin/env python3
"""Audit-v4 (2026-05-08) — typed exceptions for the TFM pick pipeline.

Hierarchy:

    TFMError (base, all)
    ├── PickError (orchestrator + pick_demo runtime)
    │   ├── PhaseTransitionError      — invalid FSM transition.
    │   ├── PoseConsistencyError      — gate de freshness/divergence.
    │   ├── ApproachError             — APPROACH/APPROACH_COARSE failure.
    │   ├── GraspDownError            — Cartesian descent fallido.
    │   ├── GraspError                — close gripper fallido.
    │   ├── LiftError                 — LIFT failed (object slipped).
    │   ├── TransportError            — TRANSPORT failed.
    │   ├── ReleaseError              — RELEASE failed.
    │   └── HomeError                 — HOME_INITIAL/HOME_FINAL failed.
    ├── MotionError (plan_to_pose / executor / FJT)
    │   ├── PlanningError             — MoveIt PLANNING_FAILED / NO_IK.
    │   ├── ExecutionError            — FJT abort / tolerance violated.
    │   └── TimeoutError              — wait future / action timeout.
    ├── HardwareError (gripper / attach / Gazebo plumbing)
    │   ├── GripperError              — open/close service fallido.
    │   ├── AttachError               — attach/detach service fallido.
    │   └── ControllerNotReadyError   — controllers no en estado activo.
    └── ConfigError
        ├── EnvVarError               — env var inválida.
        └── YAMLError                 — YAML schema mismatch.

Usage:
    raise PickError("phase=APPROACH", reason="ompl_failure", phase="APPROACH")

Cada error tiene un ``reason`` corto (machine-readable) para evidence_logger
y un mensaje completo (human-readable).

Sin dependencias ROS — pueden usarse en helpers puros y propagarse al
LifecycleNode wrapper.
"""
from __future__ import annotations

from typing import Any, Dict, Optional


class TFMError(Exception):
    """Base class for typed errors of the TFM pick pipeline."""

    def __init__(
        self,
        message: str = "",
        *,
        reason: str = "",
        context: Optional[Dict[str, Any]] = None,
    ) -> None:
        super().__init__(message)
        self.reason: str = str(reason or self.__class__.__name__)
        self.context: Dict[str, Any] = dict(context or {})

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.__class__.__name__,
            "reason": self.reason,
            "message": str(self),
            "context": self.context,
        }


# ---- Pick orchestrator errors ----------------------------------------------


class PickError(TFMError):
    """Orchestrator/pick_demo runtime failure (phase-bound)."""

    def __init__(
        self,
        message: str = "",
        *,
        reason: str = "",
        phase: str = "",
        context: Optional[Dict[str, Any]] = None,
    ) -> None:
        ctx = dict(context or {})
        if phase:
            ctx["phase"] = str(phase)
        super().__init__(message, reason=reason or "pick_error", context=ctx)
        self.phase: str = str(phase)


class PhaseTransitionError(PickError):
    """Invalid FSM transition (e.g. APPROACH→GRASP without GRASP_DOWN)."""


class PoseConsistencyError(PickError):
    """Gate de freshness o divergence en pose-consistency."""


class ApproachError(PickError):
    """APPROACH or APPROACH_COARSE failure (timeout, OMPL, dist_tol)."""


class GraspDownError(PickError):
    """Cartesian descent fallido (Z-truncation, IK fail)."""


class GraspError(PickError):
    """Cierre del gripper fallido / no se confirmó cerrado."""


class LiftError(PickError):
    """LIFT failed — objeto resbaló o trayectoria abortada."""


class TransportError(PickError):
    """TRANSPORT failed (MoveIt timeout, wrong direction, stall)."""


class ReleaseError(PickError):
    """RELEASE failed (open service no respondió, drop incompleto)."""


class HomeError(PickError):
    """HOME_INITIAL or HOME_FINAL fallido."""


# ---- Motion / planning / executor errors -----------------------------------


class MotionError(TFMError):
    """plan_to_pose / executor / FJT failure."""


class PlanningError(MotionError):
    """MoveIt PLANNING_FAILED / compute_ik NO_SOLUTION."""


class ExecutionError(MotionError):
    """FJT aborted (PATH_TOLERANCE_VIOLATED, GOAL_TOLERANCE_VIOLATED)."""


class TimeoutError(MotionError):  # noqa: A001 — sombrea builtin a propósito.
    """Wait future / action call timeout."""


# ---- Hardware / Gazebo plumbing errors -------------------------------------


class HardwareError(TFMError):
    """Gripper / attach / controller plumbing failure."""


class GripperError(HardwareError):
    """open/close service fallido."""


class AttachError(HardwareError):
    """attach/detach service fallido."""


class ControllerNotReadyError(HardwareError):
    """Controllers no en estado activo o controller_manager unreachable."""


# ---- Config errors ---------------------------------------------------------


class ConfigError(TFMError):
    """Configuración inválida o inconsistente."""


class EnvVarError(ConfigError):
    """Env var inválida (fuera de rango, formato erróneo)."""


class YAMLError(ConfigError):
    """YAML schema mismatch o YAML mal formado."""
