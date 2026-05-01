#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/lifecycle_helpers.py
# Contenido: F9 — helpers puros para el ciclo de vida del orchestrator.
"""Helpers puros del ciclo de vida del orchestrator.

ROS 2 Lifecycle Nodes definen estados managed:

    UNCONFIGURED → (configure) → INACTIVE → (activate) → ACTIVE
                                       ↑                     │
                                       └─── (deactivate) ────┘
    INACTIVE → (cleanup) → UNCONFIGURED
    Cualquier estado → (shutdown) → FINALIZED

Las transiciones permitidas y la lógica de validación deben ser
testeables sin instanciar un LifecycleNode (que requiere rclpy).
Este módulo extrae:

* ``LifecycleState`` — enum de estados managed.
* ``LifecycleTransition`` — enum de transiciones.
* ``allowed_transitions(state)`` — qué transiciones son válidas desde
  cada estado.
* ``next_state(current, transition)`` — estado destino tras una
  transición; lanza ValueError si no está permitida.
* ``OrchestratorLifecycleResources`` — dataclass que encapsula los
  recursos (action_server, service_map, etc.) creados en configure y
  liberados en cleanup. Validaciones puras de invariantes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Set


class LifecycleState(str, Enum):
    UNCONFIGURED = "unconfigured"
    INACTIVE = "inactive"
    ACTIVE = "active"
    FINALIZED = "finalized"


class LifecycleTransition(str, Enum):
    CONFIGURE = "configure"
    ACTIVATE = "activate"
    DEACTIVATE = "deactivate"
    CLEANUP = "cleanup"
    SHUTDOWN = "shutdown"


# Tabla de transiciones permitidas: estado_origen -> {transición: estado_destino}.
_TRANSITIONS: Dict[LifecycleState, Dict[LifecycleTransition, LifecycleState]] = {
    LifecycleState.UNCONFIGURED: {
        LifecycleTransition.CONFIGURE: LifecycleState.INACTIVE,
        LifecycleTransition.SHUTDOWN: LifecycleState.FINALIZED,
    },
    LifecycleState.INACTIVE: {
        LifecycleTransition.ACTIVATE: LifecycleState.ACTIVE,
        LifecycleTransition.CLEANUP: LifecycleState.UNCONFIGURED,
        LifecycleTransition.SHUTDOWN: LifecycleState.FINALIZED,
    },
    LifecycleState.ACTIVE: {
        LifecycleTransition.DEACTIVATE: LifecycleState.INACTIVE,
        LifecycleTransition.SHUTDOWN: LifecycleState.FINALIZED,
    },
    LifecycleState.FINALIZED: {},
}


def allowed_transitions(state: LifecycleState) -> Set[LifecycleTransition]:
    """Devuelve las transiciones válidas desde ``state``."""
    return set(_TRANSITIONS.get(state, {}).keys())


def can_transition(state: LifecycleState, transition: LifecycleTransition) -> bool:
    """True si ``transition`` es válida desde ``state``."""
    return transition in _TRANSITIONS.get(state, {})


def next_state(
    state: LifecycleState,
    transition: LifecycleTransition,
) -> LifecycleState:
    """Devuelve el estado destino. Lanza ValueError si no está permitida."""
    table = _TRANSITIONS.get(state, {})
    if transition not in table:
        raise ValueError(
            f"transition_not_allowed: from={state.value} via={transition.value} "
            f"allowed={[t.value for t in table.keys()]}"
        )
    return table[transition]


@dataclass
class OrchestratorLifecycleResources:
    """Encapsula recursos del orchestrator gestionados por lifecycle.

    En ``configure`` se crean (action_server, clients, params).
    En ``activate`` se enable accept_goals.
    En ``deactivate`` se reject incoming goals (pero no destruir).
    En ``cleanup`` se destruyen.

    Validaciones puras de invariantes — útil en tests sin instanciar
    rclpy.
    """

    has_action_server: bool = False
    has_service_map: bool = False
    accepts_goals: bool = False
    params_loaded: bool = False
    config_summary: Dict[str, object] = field(default_factory=dict)

    def is_configured(self) -> bool:
        return self.has_action_server and self.has_service_map and self.params_loaded

    def is_activated(self) -> bool:
        return self.is_configured() and self.accepts_goals

    def configure_invariant(self) -> Optional[str]:
        """Devuelve None si invariantes de configure cumplen, sino mensaje."""
        if not self.has_action_server:
            return "action_server_not_created"
        if not self.has_service_map:
            return "service_map_not_initialized"
        if not self.params_loaded:
            return "params_not_loaded"
        if self.accepts_goals:
            return "should_not_accept_goals_yet"
        return None

    def activate_invariant(self) -> Optional[str]:
        """Devuelve None si invariantes de activate cumplen, sino mensaje."""
        if not self.is_configured():
            return "not_configured"
        if not self.accepts_goals:
            return "accepts_goals_not_enabled"
        return None

    def deactivate_invariant(self) -> Optional[str]:
        if not self.is_configured():
            return "not_configured"
        if self.accepts_goals:
            return "still_accepting_goals"
        return None

    def cleanup_invariant(self) -> Optional[str]:
        if self.has_action_server:
            return "action_server_still_present"
        if self.has_service_map:
            return "service_map_still_present"
        if self.accepts_goals:
            return "still_accepting_goals"
        if self.params_loaded:
            return "params_still_loaded"
        return None


def reject_reason_for_state(state: LifecycleState) -> Optional[str]:
    """Devuelve la razón por la cual rechazar un goal según el estado actual.

    None si el estado es ACTIVE (acepta goals).
    """
    if state == LifecycleState.ACTIVE:
        return None
    return f"node_not_active:{state.value}"
