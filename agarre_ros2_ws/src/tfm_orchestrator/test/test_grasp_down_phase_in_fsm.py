# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_grasp_down_phase_in_fsm.py
# Contenido: T29 — drift detector de la fase GRASP_DOWN en el FSM canónico.
"""T29 — drift detector de GRASP_DOWN en happy path del FSM.

GRASP_DOWN se añadió al FSM en 2026-05-07 (commit a984234) para que las
pinzas RG2 toquen físicamente el objeto antes del attach. Es la fase
más crítica del agarre visual.

Si alguien:
  - Cambia el orden de las fases del happy path.
  - Quita GRASP_DOWN.
  - Añade fases sin actualizar happy_path.
  - Cambia las transiciones permitidas.

…este test falla y obliga a actualizar la documentación / el plan FSM
intencionadamente.
"""
from __future__ import annotations

from tfm_orchestrator.pick_fsm import (
    PickPhase,
    allowed_transitions,
    can_transition,
    happy_path,
    phase_index,
    progress_fraction,
)


# ---------------------------------------------------------------------------
# Estructura del happy path
# ---------------------------------------------------------------------------


def test_happy_path_has_11_entries_post_grasp_down_20260507():
    """2026-05-07: el happy path tiene 11 entries (era 10 pre-GRASP_DOWN)."""
    path = happy_path()
    assert len(path) == 11, (
        f"happy_path() tiene {len(path)} entries (esperado 11). "
        "Si añades/quitas fases, actualiza este drift detector."
    )


def test_happy_path_canonical_order_includes_grasp_down():
    """El orden canónico del happy path incluye GRASP_DOWN entre APPROACH y GRASP."""
    expected = [
        PickPhase.IDLE,
        PickPhase.INITIAL_SNAPSHOT,
        PickPhase.HOME_INITIAL,
        PickPhase.SELECT_OBJECT,
        PickPhase.APPROACH,
        PickPhase.GRASP_DOWN,
        PickPhase.GRASP,
        PickPhase.LIFT,
        PickPhase.TRANSPORT,
        PickPhase.RELEASE,
        PickPhase.DONE,
    ]
    actual = happy_path()
    assert actual == expected, (
        f"Orden canónico cambió.\nEsperado: {[p.value for p in expected]}\n"
        f"Actual:   {[p.value for p in actual]}"
    )


# ---------------------------------------------------------------------------
# Transiciones permitidas con GRASP_DOWN
# ---------------------------------------------------------------------------


def test_approach_can_transition_to_grasp_down():
    """APPROACH ahora transiciona a GRASP_DOWN, no directo a GRASP."""
    assert can_transition(PickPhase.APPROACH, PickPhase.GRASP_DOWN) is True


def test_approach_cannot_skip_grasp_down_to_grasp():
    """Sin GRASP_DOWN no se puede pasar a GRASP — proteger el agarre físico."""
    assert can_transition(PickPhase.APPROACH, PickPhase.GRASP) is False


def test_grasp_down_can_transition_to_grasp():
    """GRASP_DOWN → GRASP es la transición canónica post-descenso."""
    assert can_transition(PickPhase.GRASP_DOWN, PickPhase.GRASP) is True


def test_grasp_down_can_transition_to_failed():
    """Toda fase no terminal puede ir a FAILED si hay error."""
    assert can_transition(PickPhase.GRASP_DOWN, PickPhase.FAILED) is True
    assert can_transition(PickPhase.GRASP_DOWN, PickPhase.ABORTED) is True


def test_grasp_down_cannot_skip_to_lift():
    """No se puede saltar GRASP — siempre hay que cerrar pinzas + attach."""
    assert can_transition(PickPhase.GRASP_DOWN, PickPhase.LIFT) is False


def test_grasp_down_allowed_transitions_set():
    """El set canónico de transiciones desde GRASP_DOWN."""
    expected = frozenset({PickPhase.GRASP, PickPhase.FAILED, PickPhase.ABORTED})
    assert allowed_transitions(PickPhase.GRASP_DOWN) == expected


# ---------------------------------------------------------------------------
# Índices y progress_fraction
# ---------------------------------------------------------------------------


def test_grasp_down_phase_index_is_5():
    """GRASP_DOWN es el 6º elemento (índice 5) del happy path."""
    assert phase_index(PickPhase.GRASP_DOWN) == 5


def test_grasp_phase_index_is_6_post_grasp_down():
    """GRASP ahora es el 7º (índice 6) — era 5 antes de GRASP_DOWN."""
    assert phase_index(PickPhase.GRASP) == 6


def test_lift_phase_index_is_7_post_grasp_down():
    """LIFT ahora es el 8º (índice 7) — era 6 antes de GRASP_DOWN."""
    assert phase_index(PickPhase.LIFT) == 7


def test_grasp_down_progress_fraction_is_half():
    """GRASP_DOWN está justo en la mitad del happy path (5/10 = 0.5)."""
    progress = progress_fraction(PickPhase.GRASP_DOWN)
    assert 0.49 <= progress <= 0.51


def test_done_progress_fraction_is_one():
    """DONE siempre es 1.0 (validación que el divisor sigue siendo correcto)."""
    assert progress_fraction(PickPhase.DONE) == 1.0
