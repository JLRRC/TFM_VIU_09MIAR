#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/pick_fsm.py
# Contenido: F5 — state machine pura del flujo pick & place (sin ROS).
"""Pick & place state machine — pure Python, no ROS imports.

El FSM es la implementación canónica del flujo pick & place del TFM.
Vive **fuera del proceso del panel Qt** (a diferencia de
``panel_pick_demo.run_pick_demo`` que tiene 10.7 kLOC con FSM
embebido y closures que tocan el panel).

Diseño:

* Estados como ``Enum`` ordenado (no jerárquico — lineal con
  posibles tránsitos a estados terminales en cualquier punto).
* Transiciones declaradas en una tabla constante; el método
  ``can_transition(src, dst)`` devuelve bool.
* Una instancia mantiene ``current_phase`` y ``phase_index``; los
  callers (el nodo ROS) la mueven explícitamente con ``advance(dst)``.
* Sin side-effects: no hace I/O, no toca ROS, no espera. Es pura
  computación. Esto la hace 100% testeable en pytest sin entorno.

El nodo ROS ``PickOrchestratorNode`` instancia este FSM y, en cada
fase, llama a los servicios apropiados (Open, Close, Attach,
WorldToBase, etc.) — esos sí son I/O ROS pero el FSM no los conoce.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, FrozenSet, List, Optional, Tuple


class PickPhase(str, Enum):
    """Fases del pipeline pick & place.

    Orden lineal del flujo "happy path":

        IDLE → INITIAL_SNAPSHOT → HOME_INITIAL → SELECT_OBJECT → APPROACH
            → GRASP → LIFT → TRANSPORT → RELEASE → DONE

    INITIAL_SNAPSHOT y HOME_INITIAL (F12-step2c.1, 2026-05-02) son las
    primeras dos fases granulares del legacy ``run_pick_demo`` migradas
    al FSM canónico. Capturan respectivamente el snapshot inicial del
    estado (TCP/objeto/joints) y el movimiento joint a la pose HOME
    previa a la selección de objeto.

    Estados terminales:

        DONE: éxito.
        FAILED: fallo en cualquier fase.
        ABORTED: cancelado por el usuario / cliente de la action.
    """

    IDLE = "IDLE"
    INITIAL_SNAPSHOT = "INITIAL_SNAPSHOT"
    HOME_INITIAL = "HOME_INITIAL"
    SELECT_OBJECT = "SELECT_OBJECT"
    APPROACH = "APPROACH"
    # 2026-05-07: GRASP_DOWN entre APPROACH y GRASP. Cartesian descent
    # vertical desde APPROACH (objeto+10cm) hasta justo antes del contacto
    # (objeto+2cm). Sin esto, GRASP cierra el gripper a 12cm del objeto
    # — attach lógico funciona pero las pinzas NO tocan el objeto en
    # Gazebo. Cierre del proyecto requiere validación visual del agarre.
    GRASP_DOWN = "GRASP_DOWN"
    GRASP = "GRASP"
    LIFT = "LIFT"
    TRANSPORT = "TRANSPORT"
    RELEASE = "RELEASE"
    DONE = "DONE"
    FAILED = "FAILED"
    ABORTED = "ABORTED"


# Orden lineal happy-path.
_HAPPY_PATH: List[PickPhase] = [
    PickPhase.IDLE,
    PickPhase.INITIAL_SNAPSHOT,
    PickPhase.HOME_INITIAL,
    PickPhase.SELECT_OBJECT,
    PickPhase.APPROACH,
    PickPhase.GRASP_DOWN,  # 2026-05-07
    PickPhase.GRASP,
    PickPhase.LIFT,
    PickPhase.TRANSPORT,
    PickPhase.RELEASE,
    PickPhase.DONE,
]

# Estados terminales (no admiten transiciones salientes).
TERMINAL_PHASES: FrozenSet[PickPhase] = frozenset(
    {PickPhase.DONE, PickPhase.FAILED, PickPhase.ABORTED}
)

# Transiciones permitidas. Construidas a partir del happy path +
# permitir FAILED/ABORTED desde cualquier fase no-terminal.
_ALLOWED_TRANSITIONS: Dict[PickPhase, FrozenSet[PickPhase]] = {}
for i, phase in enumerate(_HAPPY_PATH[:-1]):
    next_phase = _HAPPY_PATH[i + 1]
    # Cada fase puede avanzar a la siguiente del happy path o a
    # FAILED/ABORTED.
    _ALLOWED_TRANSITIONS[phase] = frozenset(
        {next_phase, PickPhase.FAILED, PickPhase.ABORTED}
    )
# Estados terminales no transicionan a ninguno.
for phase in TERMINAL_PHASES:
    _ALLOWED_TRANSITIONS[phase] = frozenset()


def allowed_transitions(src: PickPhase) -> FrozenSet[PickPhase]:
    """Devuelve el set de fases destino permitidas desde ``src``."""
    return _ALLOWED_TRANSITIONS[src]


def can_transition(src: PickPhase, dst: PickPhase) -> bool:
    """True si la transición ``src → dst`` está permitida."""
    return dst in _ALLOWED_TRANSITIONS.get(src, frozenset())


def phase_index(phase: PickPhase) -> int:
    """Posición de la fase en el happy path (0-based).

    Para FAILED/ABORTED devuelve -1.
    """
    if phase in (PickPhase.FAILED, PickPhase.ABORTED):
        return -1
    return _HAPPY_PATH.index(phase)


def progress_fraction(phase: PickPhase) -> float:
    """Progreso normalizado [0.0, 1.0].

    IDLE → 0.0. DONE → 1.0. FAILED/ABORTED → 0.0 (ya no avanza).
    Resto interpolado proporcionalmente.
    """
    if phase in (PickPhase.FAILED, PickPhase.ABORTED):
        return 0.0
    idx = _HAPPY_PATH.index(phase)
    return idx / (len(_HAPPY_PATH) - 1)


@dataclass
class PickContext:
    """Contexto runtime del FSM pick & place.

    Inmutable salvo por ``current_phase`` y ``detail``. Los atributos
    de configuración (object_name, drop_xyz_world, object_pose_world_hint)
    se setean al inicio desde el goal de la action.

    F5-step2: ``object_pose_world_hint`` es la pose actual del objeto en
    world tal y como la conoce el cliente (panel). Si es None o cumple
    ``is_no_hint(pose)``, el orchestrator usará el placeholder previo.
    Formato canónico: tuple de 7 floats ``(x, y, z, qx, qy, qz, qw)``.
    Aceptamos también tuple de 3 (xyz solo, orientación implícita
    identity) por simplicidad para callers que no manejan orientación.
    """

    object_name: str = ""
    drop_xyz_world: Tuple[float, ...] = (0.0, 0.0, 0.0)
    object_pose_world_hint: Optional[Tuple[float, ...]] = None
    # B-iter5 (2026-05-03): snapshot inicial capturado durante INITIAL_SNAPSHOT.
    # Permanecen None hasta que la fase INITIAL_SNAPSHOT corra exitosamente.
    # Las fases siguientes (APPROACH/GRASP/etc) pueden consumir estos valores
    # en lugar de re-resolver TF/joint_state cada vez (fuente única de verdad
    # del estado al inicio del ciclo).
    initial_tcp_pose_base: Optional[Tuple[float, ...]] = None       # (x,y,z,qx,qy,qz,qw) en base_link
    initial_joint_positions: Optional[Tuple[float, ...]] = None      # tuple de 6 floats (rad)
    initial_object_pose_world: Optional[Tuple[float, ...]] = None    # (x,y,z,qx,qy,qz,qw) en world
    current_phase: PickPhase = PickPhase.IDLE
    detail: str = ""
    history: List[PickPhase] = field(default_factory=list)

    def advance(self, dst: PickPhase, *, detail: str = "") -> None:
        """Mueve el FSM a ``dst``. Lanza ValueError si no permitido."""
        if not can_transition(self.current_phase, dst):
            raise ValueError(
                f"transición no permitida: {self.current_phase.value} -> {dst.value}"
            )
        self.history.append(self.current_phase)
        self.current_phase = dst
        self.detail = str(detail or "")

    def fail(self, reason: str) -> None:
        """Atajo: transiciona al estado FAILED con ``reason`` en detail."""
        self.advance(PickPhase.FAILED, detail=reason)

    def abort(self, reason: str = "client_canceled") -> None:
        """Atajo: transiciona a ABORTED."""
        self.advance(PickPhase.ABORTED, detail=reason)

    def is_terminal(self) -> bool:
        """True si está en un estado terminal."""
        return self.current_phase in TERMINAL_PHASES

    def progress(self) -> float:
        """Progreso normalizado [0,1]."""
        return progress_fraction(self.current_phase)

    def index(self) -> int:
        """Índice 0-based en el happy path."""
        return phase_index(self.current_phase)

    def feedback_snapshot(self) -> Dict[str, object]:
        """Snapshot dict listo para mapear al feedback de PickPlace.action."""
        return {
            "current_phase": self.current_phase.value,
            "progress": float(self.progress()),
            "phase_index": int(self.index()),
            "detail": str(self.detail),
        }


def happy_path() -> List[PickPhase]:
    """Devuelve copia del happy path canónico."""
    return list(_HAPPY_PATH)


def normalize_pose_hint(
    pose_hint: Optional[Tuple[float, ...]],
) -> Optional[Tuple[float, ...]]:
    """Normaliza un pose hint a tuple de 7 floats (x,y,z,qx,qy,qz,qw).

    Acepta:
      * None → None.
      * Tuple/list de 3 (xyz) → completa con identity (0,0,0,1).
      * Tuple/list de 7 (xyz + quat) → cast a floats.

    Si el input es inválido (longitud 4-6, no iterable, no convertible),
    devuelve None — fail-soft, el caller debe verificar.
    """
    if pose_hint is None:
        return None
    try:
        seq = tuple(float(v) for v in pose_hint)
    except (TypeError, ValueError):
        return None
    if len(seq) == 3:
        return seq + (0.0, 0.0, 0.0, 1.0)
    if len(seq) == 7:
        return seq
    return None


def is_no_hint(pose_hint: Optional[Tuple[float, ...]]) -> bool:
    """True si el pose hint es ``None`` o cumple la convención "no hint".

    Convención: position (0,0,0) y orientation identity (qw=1, qx=qy=qz=0).
    Cualquier otra pose se considera hint válido.
    """
    if pose_hint is None:
        return True
    norm = normalize_pose_hint(pose_hint)
    if norm is None:
        return True
    if len(norm) != 7:
        return True
    x, y, z, qx, qy, qz, qw = norm
    # F7: explicit bool() para que mypy strict no infiera Any de la cadena
    # de comparaciones sobre los floats unpacked.
    return bool(
        x == 0.0 and y == 0.0 and z == 0.0
        and qx == 0.0 and qy == 0.0 and qz == 0.0 and qw == 1.0
    )
