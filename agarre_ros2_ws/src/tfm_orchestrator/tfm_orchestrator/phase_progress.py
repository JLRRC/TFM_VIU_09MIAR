#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/phase_progress.py
# Contenido: B-iter7 (2026-05-03) — interpolación de progress intra-fase.
"""B-iter7 — Helper puro para feedback intermedio del PickPlace.action.

Hasta B-iter6, el feedback emitido por el orchestrator publica
``progress`` con un valor fijo por fase (idx/N_PHASES). Esto deja al
cliente sin información de sub-progreso dentro de fases que duran
varios segundos (HOME_INITIAL ~5s, APPROACH/LIFT/TRANSPORT con MoveIt
~5-15s cada una).

Este módulo provee funciones puras para:

* ``interpolate_phase_progress(phase, sub_progress_0_1)`` — devuelve un
  ``progress`` global en ``[0.0, 1.0]`` interpolando entre el progress
  de la fase actual y el de la siguiente fase del happy path.

* ``fjt_progress_from_feedback(feedback_msg, joint_names_order)`` —
  estima ``sub_progress`` (0..1) de un ``FollowJointTrajectory.Feedback``
  comparando ``actual.positions`` vs ``desired.positions``.

* ``moveit_progress_from_state(state)`` — estima ``sub_progress`` (0..1)
  para el state textual de MoveGroup feedback (``"PLANNING"`` ~0.3,
  ``"MONITOR"`` ~0.7, ``"IDLE"`` ~1.0).

Sin estado, sin imports ROS al cargar el módulo. 100% offline-testable.
"""

from __future__ import annotations

from typing import Any, Optional, Sequence, Tuple


def interpolate_phase_progress(phase: Any, sub_progress_0_1: float) -> float:
    """Interpola progress global entre la fase actual y la siguiente.

    Parameters:
        phase: ``PickPhase`` (str enum) — la fase actualmente en ejecución.
        sub_progress_0_1: progreso dentro de la fase, en [0.0, 1.0].

    Returns:
        progress global en [0.0, 1.0]. Por ejemplo, si phase=APPROACH
        (idx 4) en happy path de 9 fases y sub_progress=0.5, el global
        es ((4 + 0.5) / 8) ≈ 0.5625.

    Si phase no está en el happy path o sub_progress es inválido, devuelve
    el progress fijo de phase (fallback seguro).
    """
    # Lazy import para no levantar deps de pick_fsm al cargar.
    from .pick_fsm import PickPhase, happy_path

    try:
        sub = max(0.0, min(1.0, float(sub_progress_0_1)))
    except (TypeError, ValueError):
        sub = 0.0

    path = happy_path()
    n = len(path) - 1  # IDLE..DONE → 9 transiciones
    if n <= 0:
        return 0.0

    try:
        idx = path.index(phase)
    except ValueError:
        return 0.0

    if phase in (PickPhase.IDLE,):
        return 0.0
    if phase in (PickPhase.DONE,):
        return 1.0

    # Posición actual: idx; siguiente: min(idx+1, n).
    next_idx = min(idx + 1, n)
    base = float(idx) / float(n)
    nxt = float(next_idx) / float(n)
    return float(max(0.0, min(1.0, base + (nxt - base) * sub)))


def _arr(msg_field: Any) -> Tuple[float, ...]:
    """Extrae un array numérico de un atributo de msg ROS, fail-soft."""
    if msg_field is None:
        return ()
    try:
        return tuple(float(v) for v in msg_field)
    except (TypeError, ValueError):
        return ()


def fjt_progress_from_feedback(
    feedback_msg: Any,
    joint_names_order: Optional[Sequence[str]] = None,
) -> float:
    """Estima sub_progress (0..1) de un FollowJointTrajectory.Feedback.

    Compara ``actual.positions`` vs ``desired.positions`` por joint y
    estima cuánto le queda al controller (basado en el max error
    angular). Devuelve 1.0 si max_error <= ``goal_tol_proxy_rad`` (0.01).

    Si el feedback no expone los campos esperados, devuelve 0.0
    (mejor "no progress info" que crash).
    """
    if feedback_msg is None:
        return 0.0
    fb = getattr(feedback_msg, "feedback", feedback_msg)
    if fb is None:
        return 0.0

    desired = getattr(fb, "desired", None)
    actual = getattr(fb, "actual", None)
    if desired is None or actual is None:
        return 0.0

    desired_pos = _arr(getattr(desired, "positions", None))
    actual_pos = _arr(getattr(actual, "positions", None))
    if not desired_pos or not actual_pos:
        return 0.0

    n = min(len(desired_pos), len(actual_pos))
    if n == 0:
        return 0.0

    max_err = 0.0
    for i in range(n):
        e = abs(desired_pos[i] - actual_pos[i])
        if e > max_err:
            max_err = e

    # Map [0, π/2] → progress [1.0, 0.0] linearly. π/2 ~ 1.57 rad es
    # un movimiento "grande"; close to 0 → 1.0 (nearly done).
    full_swing = 1.5708
    if max_err <= 0.01:
        return 1.0
    raw = 1.0 - min(1.0, max_err / full_swing)
    return float(max(0.0, min(0.99, raw)))


# MoveGroup.Feedback.state values (from action definition).
_MOVEIT_STATE_PROGRESS = {
    "PLANNING": 0.3,
    "MONITOR": 0.7,
    "IDLE": 1.0,
}


def moveit_progress_from_state(state: Any) -> float:
    """Estima sub_progress (0..1) del state textual de MoveGroup feedback.

    Mapping:
      - "PLANNING" → 0.30 (planner buscando trayectoria)
      - "MONITOR"  → 0.70 (controller ejecutando trayectoria)
      - "IDLE"     → 1.00 (terminado)
      - cualquier otro → 0.10 (placeholder mínimo)

    Acepta el feedback msg directamente (extrae .state) o el string solo.
    """
    if state is None:
        return 0.0
    if hasattr(state, "feedback"):
        state = getattr(state.feedback, "state", state)
    elif hasattr(state, "state"):
        state = state.state
    s = str(state or "").strip().upper()
    return float(_MOVEIT_STATE_PROGRESS.get(s, 0.10))
