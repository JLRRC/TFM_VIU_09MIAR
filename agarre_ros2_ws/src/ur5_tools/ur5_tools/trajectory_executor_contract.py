#!/usr/bin/env python3
"""F6 audit-v4 (2026-05-08) — contrato del futuro trajectory_executor_node.

Documenta y verifica el contrato que tendrá el nodo standalone
``trajectory_executor_node`` cuando se extraiga del monolito
``moveit_bridge/executor.py`` (1.546 LOC).

Este módulo NO ejecuta nada — son DTOs + factory checks que aseguran
que los helpers puros ya extraídos cubren la superficie pública del
contrato. Cualquier nuevo helper que entre al executor debe poderse
mapear contra este contrato.

Helpers puros ya extraídos (ver mypy_strict_baseline):
    - moveit_bridge/log_formatters.py        (formato de phase logs)
    - moveit_bridge/no_server_meta.py        (meta de "no server" failure)
    - moveit_bridge/path_tolerance.py        (cálculo path_tolerance)
    - moveit_bridge/queue_helpers.py         (request queue ops)
    - moveit_bridge/time_conversion.py       (sec ↔ Time msg)
    - fjt_direct_helpers.py                  (IK normalization + tolerances)

Pendientes de extraer (deferred v1.1):
    - send_goal_async wrapper                (F5-iter2)
    - poll result with gates                 (F5-iter3)
    - result handler + retry decision        (F5-iter4)
    - report builder                         (F5-iter5)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple


class TrajectoryExecutorPhase(str, Enum):
    """Fase de ejecución que el executor reporta hacia el orchestrator."""

    PREPARE = "prepare"     # validate + scale + jt prep
    DISPATCH = "dispatch"   # send_goal_async + accept
    EXECUTE = "execute"     # poll feedback + early success gates
    SETTLE = "settle"       # post-execute verification (joint/ee tol)
    REPORT = "report"       # build (success, reason, meta) tuple


@dataclass(frozen=True)
class ExecutorRequest:
    """Input contract de un trajectory execution call.

    Inmutable + sin tipos ROS (PoseStamped, JointTrajectory) para que
    pueda usarse en tests offline y wrappers de testing.
    """

    joint_names: Tuple[str, ...]
    waypoint_positions: Tuple[Tuple[float, ...], ...]
    waypoint_times_sec: Tuple[float, ...]
    timeout_sec: float = 8.0
    retry_on_tolerance_violation: bool = True
    path_tol_override_rad: Optional[float] = None
    goal_time_override_sec: Optional[float] = None
    ee_target_tol_m: Optional[float] = None
    phase_label: str = ""
    approach_replan_attempt: int = 0


@dataclass(frozen=True)
class ExecutorReport:
    """Output contract — qué reporta el executor."""

    success: bool
    reason: str
    duration_sec: float
    phase: TrajectoryExecutorPhase
    meta: Dict[str, object] = field(default_factory=dict)


CANONICAL_REASON_CODES: Tuple[str, ...] = (
    # Success
    "fjt:SUCCESSFUL",
    "early_success_via_feedback",
    "early_success_via_ee",
    "early_success_via_joint",
    # Common failures
    "fjt_action_client_unavailable",
    "fjt_goal_send_timeout",
    "fjt_goal_rejected",
    "fjt_goal_timeout",
    "fjt_err:PATH_TOLERANCE_VIOLATED",
    "fjt_err:GOAL_TOLERANCE_VIOLATED",
    "fjt_err:OLD_HEADER_TIMESTAMP",
    "fjt_err:INVALID_GOAL",
    "fjt_err:INVALID_JOINTS",
    # Validation
    "no_action_server_after_timeout",
    "ik_no_response",
    "ik:NO_IK_SOLUTION",
    "ik:PLANNING_FAILED",
    "ik:GOAL_IN_COLLISION",
)


def is_canonical_reason(reason: str) -> bool:
    """True si ``reason`` empieza por uno de los códigos canónicos."""
    if not reason:
        return False
    for prefix in CANONICAL_REASON_CODES:
        if reason == prefix or reason.startswith(f"{prefix}|"):
            return True
    return False


def categorize_reason(reason: str) -> str:
    """Devuelve "success" | "failure" | "unknown" para un reason."""
    if not reason:
        return "unknown"
    success_prefixes = ("fjt:SUCCESSFUL", "early_success_")
    if any(reason.startswith(p) for p in success_prefixes):
        return "success"
    if is_canonical_reason(reason):
        return "failure"
    return "unknown"


# ---- Helper registry (lo que ya está extraído del executor) ----------------


CANONICAL_PURE_HELPER_MODULES: Tuple[str, ...] = (
    "ur5_tools.moveit_bridge.log_formatters",
    "ur5_tools.moveit_bridge.no_server_meta",
    "ur5_tools.moveit_bridge.path_tolerance",
    "ur5_tools.moveit_bridge.queue_helpers",
    "ur5_tools.moveit_bridge.time_conversion",
    "ur5_tools.fjt_direct_helpers",
    # F5-iter1 (audit-v4): preparation extraction
    # (vive como método de _BridgeBase aún, no como módulo puro).
)


def list_extracted_pure_helpers() -> Tuple[str, ...]:
    """Devuelve tupla de módulos puros ya extraídos (snapshot)."""
    return CANONICAL_PURE_HELPER_MODULES
