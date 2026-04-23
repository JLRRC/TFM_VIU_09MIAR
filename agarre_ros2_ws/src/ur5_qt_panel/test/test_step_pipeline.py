#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_step_pipeline.py
# Contenido: Tests del pipeline visual paso a paso del panel Qt.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Tests unitarios del pipeline visual STEP by STEP."""

from ur5_qt_panel.step_pipeline_helpers import (
    step_phase_action_text,
    step_phase_sequence,
)


def test_direct_sequence_starts_with_approach_coarse() -> None:
    sequence = step_phase_sequence("DIRECT")
    assert sequence[0] == "APPROACH_COARSE"
    assert "INICIO" not in sequence


def test_pick_object_sequence_keeps_home_with_object() -> None:
    sequence = step_phase_sequence("PICK_OBJECT")
    mesa_idx = sequence.index("MESA_WITH_OBJECT")
    assert sequence[mesa_idx + 1] == "HOME_WITH_OBJECT"
    assert sequence[-1] == "HOME_FINAL"


def test_step_action_text_translates_release_decision() -> None:
    text = step_phase_action_text(
        "PICK_OBJECT",
        "RELEASE",
        "release_object",
    )
    assert "Abrirá la pinza" in text


def test_step_action_text_rewrites_old_siguiente_copy() -> None:
    text = step_phase_action_text(
        "PICK_OBJECT",
        "INICIO",
        "Pulse Siguiente → robot irá a pose MESA",
    )
    assert "Iniciar" in text
    assert "Siguiente" not in text
