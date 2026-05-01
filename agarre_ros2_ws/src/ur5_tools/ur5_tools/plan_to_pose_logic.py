#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/plan_to_pose_logic.py
# Contenido: F6.5 — lógica pura del action server PlanToPose (sin ROS).
"""Lógica pura del PlanToPose action server.

Diseño:

* La parte ROS (action server, callbacks) vive en
  ``plan_to_pose_server.py``.
* Esta lógica pura recibe el goal "ya parseado" (un dataclass
  ``PlanToPoseGoal``) y produce:
  - una **secuencia de Feedback intermedios** que el server publicará
    al cliente (PLANNING → EXECUTING).
  - un **Result** final (success + reason + final_pose + duration +
    attempts).
* No toca ROS, no levanta excepciones; todo el control flow se
  expresa en datos.

Estrategia stub: para esta primera implementación F6.5, el server
acepta cualquier goal válido (ee_frame en el set permitido,
target_pose con valores finitos) y devuelve éxito tras simular un
breve planning. **NO** invoca al bridge MoveIt real — esa integración
es F6.6 (futura sesión, requiere wiring con /desired_grasp).

Esto es suficiente para validar la cadena end-to-end con el
orchestrator: el orchestrator hace ``call_action_with_timeout`` al
server, el server stub acepta y devuelve success → el orchestrator
avanza la fase del FSM.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Tuple


_VALID_EE_FRAMES = frozenset({"rg2_pinch_center", "rg2_tcp", "tool0"})


@dataclass(frozen=True)
class PlanToPoseGoal:
    """Goal parseado del action PlanToPose."""

    target_xyz: Tuple[float, float, float]
    target_quat_xyzw: Tuple[float, float, float, float]
    ee_frame: str
    cartesian: bool
    timeout_sec: float


@dataclass(frozen=True)
class PlanToPoseFeedback:
    """Feedback intermedio publicable por el server."""

    current_state: str   # PLANNING | EXECUTING | RETRYING
    progress: float       # [0.0, 1.0]
    attempts: int
    detail: str


@dataclass(frozen=True)
class PlanToPoseResult:
    """Result final del action."""

    success: bool
    reason: str
    final_xyz: Tuple[float, float, float]
    final_quat_xyzw: Tuple[float, float, float, float]
    duration_sec: float
    attempts: int


def validate_goal(goal: PlanToPoseGoal) -> Tuple[bool, str]:
    """Valida un goal. Devuelve ``(ok, reason)``.

    Reglas:
      - ee_frame en {rg2_pinch_center, rg2_tcp, tool0}.
      - target xyz componentes finitos.
      - target quat componentes finitos y norma > 0 (no requiere
        normalización exacta — lo haremos al consumir).
      - timeout_sec >= 0 (0 = usar default del server).
    """
    if goal.ee_frame not in _VALID_EE_FRAMES:
        return False, f"unknown_ee_frame:{goal.ee_frame}"
    for v in goal.target_xyz:
        if not _is_finite(v):
            return False, "target_xyz_non_finite"
    for v in goal.target_quat_xyzw:
        if not _is_finite(v):
            return False, "target_quat_non_finite"
    qn = math.sqrt(sum(c * c for c in goal.target_quat_xyzw))
    if qn <= 1e-9:
        return False, "target_quat_zero_norm"
    if goal.timeout_sec < 0.0:
        return False, "timeout_sec_negative"
    return True, "ok"


def _is_finite(v: float) -> bool:
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


def normalize_quat(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """Normaliza un cuaternión a norma 1. Si norma es ~0, devuelve identity."""
    n = math.sqrt(sum(c * c for c in q))
    if n <= 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(c / n for c in q)


def feedback_sequence(
    goal: PlanToPoseGoal,
    *,
    n_planning_steps: int = 3,
    n_executing_steps: int = 4,
) -> List[PlanToPoseFeedback]:
    """Genera la secuencia de feedbacks que el server publicará.

    El stub publica:
      * n_planning_steps PLANNING, progress de 0.0 → 0.5
      * n_executing_steps EXECUTING, progress de 0.5 → 0.95

    Útil para tests: la lista es determinística dado el goal.
    """
    seq: List[PlanToPoseFeedback] = []
    if n_planning_steps > 0:
        for i in range(n_planning_steps):
            p = 0.5 * (i + 1) / max(1, n_planning_steps)
            seq.append(PlanToPoseFeedback(
                current_state="PLANNING",
                progress=p,
                attempts=1,
                detail=f"planning step {i + 1}/{n_planning_steps}",
            ))
    if n_executing_steps > 0:
        for i in range(n_executing_steps):
            p = 0.5 + 0.45 * (i + 1) / max(1, n_executing_steps)
            seq.append(PlanToPoseFeedback(
                current_state="EXECUTING",
                progress=p,
                attempts=1,
                detail=f"executing step {i + 1}/{n_executing_steps}",
            ))
    return seq


def execute_stub(goal: PlanToPoseGoal, *, duration_sec: float = 0.0) -> PlanToPoseResult:
    """Stub de ejecución — siempre éxito si el goal es válido.

    F6.6: el server real puede saltar este stub via param
    ``use_real_bridge:=true`` y delegar al bridge MoveIt usando los
    helpers ``encode_request_frame`` + ``parse_bridge_result``.
    """
    ok, reason = validate_goal(goal)
    if not ok:
        return PlanToPoseResult(
            success=False,
            reason=f"invalid_goal:{reason}",
            final_xyz=goal.target_xyz,
            final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
            duration_sec=duration_sec,
            attempts=0,
        )
    return PlanToPoseResult(
        success=True,
        reason="stub_planning_completed",
        final_xyz=goal.target_xyz,
        final_quat_xyzw=normalize_quat(goal.target_quat_xyzw),
        duration_sec=duration_sec,
        attempts=1,
    )


# ---------------------------------------------------------------------------
# F6.6 — helpers para wiring real con el bridge MoveIt
# (publish a /desired_grasp con frame_id codificado + match de result UUID).
#
# Patrón heredado de panel_pick_object._encode_request_frame:
#   "base_link|rid=42|uid=abc123|tol=0.020|phase=PLAN_TO_POSE"
# El bridge parsea base_link como frame, rid/uid para correlación,
# tol/phase como hints. El result llega en /desired_grasp/result como
# String:
#   "success=true reason=exec_ok request_uuid=abc123 ..."
# ---------------------------------------------------------------------------


def encode_request_frame(
    base_frame: str,
    request_id: int,
    request_uuid: str,
    *,
    tol_m: Optional[float] = None,
    phase_label: Optional[str] = None,
) -> str:
    """Codifica el ``frame_id`` con metadata para el bridge MoveIt.

    El bridge deserializa ``rid`` (correlación numérica), ``uid``
    (correlación string), opcionalmente ``tol`` (target tol metros)
    y ``phase`` (label de la fase del orchestrator).

    El primer token siempre es el frame ROS válido; el bridge debe
    interpretar el primer ``|``-separated token como el frame real.
    """
    base = str(base_frame or "base_link").strip() or "base_link"
    rid = int(request_id)
    uid = str(request_uuid or "").strip()
    parts: List[str] = [base, f"rid={rid}"]
    if uid:
        parts.append(f"uid={uid}")
    if tol_m is not None:
        try:
            parts.append(f"tol={float(tol_m):.3f}")
        except Exception:
            pass
    phase = str(phase_label or "").strip()
    if phase:
        phase = phase.replace("|", "_")
        parts.append(f"phase={phase}")
    return "|".join(parts)


def parse_bridge_result(text: str) -> Tuple[Optional[bool], str, str]:
    """Parsea ``/desired_grasp/result`` String del bridge.

    Devuelve ``(success, reason, request_uuid)``.

    Formato esperado del bridge:
      ``success=true|false reason=<text> request_uuid=<UUID> ...``

    Si ``success`` no aparece, devuelve ``(None, text, "")``.

    >>> parse_bridge_result("success=true reason=exec_ok request_uuid=abc")
    (True, 'exec_ok', 'abc')
    >>> parse_bridge_result("success=false reason=plan_failed request_uuid=xyz")
    (False, 'plan_failed', 'xyz')
    >>> parse_bridge_result("garbage payload")
    (None, 'garbage payload', '')
    """
    if not text:
        return None, "", ""
    success: Optional[bool] = None
    reason = text
    uid = ""
    try:
        kv = {}
        for tok in text.split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                kv[k] = v
        if "success" in kv:
            s = kv["success"].strip().lower()
            if s in ("true", "1", "yes"):
                success = True
            elif s in ("false", "0", "no"):
                success = False
        if "reason" in kv:
            reason = kv["reason"]
        if "request_uuid" in kv:
            uid = kv["request_uuid"]
    except Exception:
        pass
    return success, reason, uid


def result_matches_request(
    result_text: str, expected_uuid: str
) -> bool:
    """True si el ``request_uuid=`` del result text coincide con el esperado.

    Si ``expected_uuid`` es vacío, acepta cualquier result (modo
    correlación-libre, no recomendado para concurrency).
    """
    if not expected_uuid:
        return True
    _, _, got = parse_bridge_result(result_text)
    return got == expected_uuid
