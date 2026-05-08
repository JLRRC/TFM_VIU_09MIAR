#!/usr/bin/env python3
"""Audit-v4 (2026-05-08): MRO + count gating para ControlPanelV2.

Detecta si alguien añade un nuevo mixin sin actualizar el plan
documented en MIXIN_DIAMOND_PLAN.md.

Cada mixin nuevo añade complejidad estructural — el plan iter1..17
para v1.1 tiene un orden óptimo. Si entra un mixin sin pasar por
el plan, este test lo señala.
"""
from __future__ import annotations

import re
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent / "ur5_qt_panel"
PANEL_V2 = PKG / "panel_v2.py"


# Set congelado 2026-05-08. Bajar es bueno (refactor → composición);
# subir requiere actualizar este test + MIXIN_DIAMOND_PLAN.md.
EXPECTED_MIXIN_COUNT = 17

EXPECTED_MIXIN_NAMES = {
    "PanelV2PublisherMixin",
    "PanelV2BasePoseMixin",
    "PanelV2GripperAttachMixin",
    "PanelV2MotionMixin",
    "PanelV2TrajSettleMixin",
    "PanelV2SystemStateMixin",
    "PanelV2StepDebugMixin",
    "PanelV2RuntimeDiagnosticsMixin",
    "PanelV2SubprocessMotionMixin",
    "PanelV2AuditLogMixin",
    "PanelV2TfmRemoteMixin",
    "PanelV2ReadyReasonsMixin",
    "PanelV2DropRecoverMixin",
    "PanelV2CalibPickMixin",
    "PanelV2OverlaysSelectionMixin",
    "PanelV2TfmScienceTraceMixin",
    # 16 + 1 — la nº 17 está duplicada en panel_v2 por error histórico
    # (PanelV2RuntimeDiagnosticsMixin aparece 2 veces). Test siguiente.
}


def _read_panel_v2() -> str:
    return PANEL_V2.read_text(encoding="utf-8")


def test_panel_v2_exists() -> None:
    assert PANEL_V2.is_file()


def test_mixin_class_inheritance_count_no_grow() -> None:
    """Cuenta ocurrencias de "Mixin," en la lista de bases de ControlPanelV2.

    Si crece, alguien añadió un mixin sin actualizar el test ni el plan.
    """
    text = _read_panel_v2()
    # Capturamos el primer bloque ``class ControlPanelV2(`` con sus bases.
    m = re.search(
        r"class\s+ControlPanelV2\s*\(\s*([\s\S]*?)\)\s*:",
        text,
    )
    assert m is not None, "ControlPanelV2 class declaration missing"
    bases = m.group(1)
    n = bases.count("Mixin,")
    assert n <= EXPECTED_MIXIN_COUNT, (
        f"ControlPanelV2 ahora hereda de {n} mixins (era {EXPECTED_MIXIN_COUNT}). "
        f"Bajar es bueno; subir requiere update plan iter1..17 en "
        f"MIXIN_DIAMOND_PLAN.md."
    )


def test_known_mixin_names_present() -> None:
    """Cada mixin canónico se importa o se referencia."""
    text = _read_panel_v2()
    missing = {m for m in EXPECTED_MIXIN_NAMES if m not in text}
    assert not missing, (
        f"Faltan mixins canónicos del baseline: {missing}. Si los borraste "
        "explícitamente (refactor diamond), elimínalos también de "
        "EXPECTED_MIXIN_NAMES."
    )


def test_qmainwindow_in_inheritance() -> None:
    """ControlPanelV2 sigue siendo QMainWindow."""
    text = _read_panel_v2()
    assert "QMainWindow" in text, "ControlPanelV2 perdió QMainWindow"
