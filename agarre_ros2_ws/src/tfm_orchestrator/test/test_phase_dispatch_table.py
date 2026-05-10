#!/usr/bin/env python3
# Ruta/archivo: src/tfm_orchestrator/test/test_phase_dispatch_table.py
# Contenido: F9.4 audit (2026-05-10) — tests de la tabla declarativa.
"""F9.4 audit (2026-05-10): tests de PHASE_DISPATCH_METADATA.

Valida que la tabla declarativa cubre todas las fases no terminales
del FSM. Si añadimos una fase nueva al FSM y no actualizamos la tabla,
este test falla (gating).
"""
from __future__ import annotations

import pytest

from tfm_orchestrator.phase_dispatch import (
    PHASE_DISPATCH_METADATA,
    get_phase_dispatch_metadata,
)
from tfm_orchestrator.pick_fsm import PickPhase


# Fases terminales del FSM no requieren handler.
TERMINAL_PHASES = {
    PickPhase.IDLE,
    PickPhase.DONE,
    PickPhase.FAILED,
    PickPhase.ABORTED,
}


def test_table_covers_all_non_terminal_phases() -> None:
    """Cada PickPhase no terminal debe tener entrada en la tabla."""
    expected = {p for p in PickPhase if p not in TERMINAL_PHASES}
    actual = set(PHASE_DISPATCH_METADATA)
    missing = expected - actual
    assert not missing, (
        f"Fases sin entrada en PHASE_DISPATCH_METADATA: {sorted(p.name for p in missing)}. "
        "Si añades una fase al FSM, actualiza la tabla en phase_dispatch.py."
    )


def test_table_does_not_have_terminal_phases() -> None:
    """Las fases terminales no necesitan handler — no deben aparecer."""
    forbidden = TERMINAL_PHASES & set(PHASE_DISPATCH_METADATA)
    assert not forbidden, (
        f"Fases terminales con entrada en la tabla (eliminar): "
        f"{sorted(p.name for p in forbidden)}"
    )


@pytest.mark.parametrize("phase, meta", sorted(
    PHASE_DISPATCH_METADATA.items(), key=lambda kv: kv[0].value
))
def test_each_metadata_entry_has_purpose_and_type(phase, meta) -> None:
    """Cada entrada debe tener al menos type + purpose."""
    assert "type" in meta, f"{phase.name}: falta 'type'"
    assert "purpose" in meta, f"{phase.name}: falta 'purpose'"
    assert isinstance(meta["purpose"], str) and meta["purpose"], (
        f"{phase.name}: 'purpose' vacío"
    )


@pytest.mark.parametrize("phase, meta", sorted(
    PHASE_DISPATCH_METADATA.items(), key=lambda kv: kv[0].value
))
def test_action_phases_declare_actions_used(phase, meta) -> None:
    """Si type contiene 'action', actions_used debe ser no vacío."""
    if "action" in str(meta.get("type", "")):
        actions = meta.get("actions_used", [])
        assert actions, (
            f"{phase.name}: type={meta['type']} pero actions_used vacío"
        )


def test_get_phase_dispatch_metadata_returns_copy() -> None:
    """``get_phase_dispatch_metadata`` debe devolver copia (no la fuente)."""
    a = get_phase_dispatch_metadata()
    a[PickPhase.SELECT_OBJECT]["__mutated"] = True
    fresh = get_phase_dispatch_metadata()
    assert "__mutated" not in fresh[PickPhase.SELECT_OBJECT], (
        "get_phase_dispatch_metadata debe devolver copia, no referencia"
    )
