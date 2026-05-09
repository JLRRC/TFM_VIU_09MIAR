#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_step_pipeline.py
# Contenido: Tests del pipeline visual paso a paso del panel Qt.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Tests unitarios del pipeline visual STEP by STEP."""

from ur5_qt_panel.step_pipeline_helpers import (
    step_phase_action_text,
    step_phase_gripper_state,
    step_phase_intent,
    step_phase_sequence,
    step_predict_next_phase,
    step_present_flow_name,
)


def test_direct_sequence_structure() -> None:
    sequence = step_phase_sequence("DIRECT")
    assert sequence[0] == "INITIAL_SNAPSHOT"
    assert "APPROACH_COARSE" in sequence
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


# ---------------------------------------------------------------------------
# step_present_flow_name
# ---------------------------------------------------------------------------

def test_present_flow_name_direct() -> None:
    assert step_present_flow_name("DIRECT") == "DIRECTO"


def test_present_flow_name_pick_object() -> None:
    assert step_present_flow_name("PICK_OBJECT") == "MOVEIT"


def test_present_flow_name_unknown_passthrough() -> None:
    assert step_present_flow_name("FOO") == "FOO"


def test_present_flow_name_empty() -> None:
    assert step_present_flow_name("") == "--"


# ---------------------------------------------------------------------------
# step_predict_next_phase
# ---------------------------------------------------------------------------

def test_predict_next_phase_direct_initial_snapshot() -> None:
    assert step_predict_next_phase("DIRECT", "INITIAL_SNAPSHOT") == "HOME_INITIAL"


def test_predict_next_phase_last_phase_returns_dash() -> None:
    assert step_predict_next_phase("DIRECT", "HOME_FINAL") == "--"


def test_predict_next_phase_unknown_phase_returns_dash() -> None:
    assert step_predict_next_phase("DIRECT", "NONEXISTENT") == "--"


def test_predict_next_phase_pick_object_lift() -> None:
    assert step_predict_next_phase("PICK_OBJECT", "LIFT") == "MESA_WITH_OBJECT"


# ---------------------------------------------------------------------------
# step_phase_intent
# ---------------------------------------------------------------------------

def test_phase_intent_direct_approach_coarse_not_empty() -> None:
    text = step_phase_intent("DIRECT", "APPROACH_COARSE")
    assert text and text != "Sin descripción"


def test_phase_intent_unknown_phase_returns_fallback() -> None:
    text = step_phase_intent("DIRECT", "NONEXISTENT_PHASE")
    assert text == "Sin descripción"


def test_phase_intent_unknown_flow_returns_fallback() -> None:
    text = step_phase_intent("BADFLOW", "APPROACH_COARSE")
    assert text == "Sin descripción"


def test_phase_intent_direct_close_mentions_pinza() -> None:
    text = step_phase_intent("DIRECT", "CLOSE")
    assert "pinza" in text.lower()


# ---------------------------------------------------------------------------
# step_phase_gripper_state
# ---------------------------------------------------------------------------

def test_gripper_state_direct_approach_open() -> None:
    assert step_phase_gripper_state("DIRECT", "APPROACH_COARSE") == "Abierta"


def test_gripper_state_direct_close_cerrada() -> None:
    assert step_phase_gripper_state("DIRECT", "CLOSE") == "Cerrada"


def test_gripper_state_direct_lift_cerrada() -> None:
    assert step_phase_gripper_state("DIRECT", "LIFT") == "Cerrada"


def test_gripper_state_direct_release_open() -> None:
    assert step_phase_gripper_state("DIRECT", "RELEASE") == "Abierta"


def test_gripper_state_unknown_returns_dash() -> None:
    assert step_phase_gripper_state("DIRECT", "NONEXISTENT") == "--"


def test_gripper_state_pick_object_grasp_attach_cerrada() -> None:
    assert step_phase_gripper_state("PICK_OBJECT", "GRASP_ATTACH") == "Cerrada"


# ---------------------------------------------------------------------------
# step_phase_action_text — decision_map entries
# ---------------------------------------------------------------------------

def test_action_text_phase_enter_decision() -> None:
    text = step_phase_action_text("DIRECT", "APPROACH_COARSE", "PHASE_ENTER")
    assert "ejecución normal" in text.lower() or "normal" in text


def test_action_text_close_and_attach_decision() -> None:
    text = step_phase_action_text("DIRECT", "CLOSE", "CLOSE_AND_ATTACH")
    assert "pinza" in text.lower()


def test_action_text_empty_decision_uses_intent() -> None:
    text = step_phase_action_text("DIRECT", "APPROACH_COARSE", "")
    assert text and text != "Sin descripción"


def test_action_text_empty_decision_unknown_flow_phase_returns_fallback() -> None:
    # intent == "Sin descripción" + empty decision → line 165: "Iniciará esta fase."
    text = step_phase_action_text("UNKNOWNFLOW", "UNKNOWNPHASE", "")
    assert text == "Iniciará esta fase."
