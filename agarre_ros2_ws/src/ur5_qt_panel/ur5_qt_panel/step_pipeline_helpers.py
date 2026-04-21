#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/step_pipeline_helpers.py
# Contenido: Reglas puras del pipeline visual paso a paso del panel.
# Uso breve: Lo usa el panel para renderizar fases, textos y estados sin depender de ROS.
"""Helpers puros para el pipeline visual STEP by STEP."""

from __future__ import annotations

from typing import List


def step_present_flow_name(flow: str) -> str:
    flow_name = str(flow or "").strip().upper()
    mapping = {
        "DIRECT": "DIRECTO",
        "DIRECT2": "DIRECTO2",
        "PICK_OBJECT": "MOVEIT",
        "MOVEIT": "MOVEIT",
        "AGARRE": "AGARRE",
    }
    return mapping.get(flow_name, flow_name or "--")


def step_phase_sequence(flow: str) -> List[str]:
    flow_name = str(flow or "").strip().upper()
    sequences = {
        "DIRECT": [
            "APPROACH_COARSE",
            "GRASP_DOWN_JOINT",
            "GRASP_ALIGN_IK",
            "PRE_CLOSE",
            "CLOSE",
            "ATTACH_GATE",
            "LIFT",
            "CARRY",
            "RELEASE",
            "HOME_FINAL",
        ],
        "DIRECT2": ["MESA_1", "POSE_BUENA", "MESA_2", "CESTA"],
        "PICK_OBJECT": [
            "INICIO",
            "APPROACH",
            "PRE_GRASP",
            "GRASP_DOWN",
            "GRASP_ATTACH",
            "LIFT",
            "MESA_WITH_OBJECT",
            "HOME_WITH_OBJECT",
            "TRANSPORT",
            "DROP",
            "RELEASE",
            "HOME_FINAL",
        ],
        "MOVEIT": [
            "APPROACH",
            "PRE_GRASP",
            "GRASP_DOWN",
            "GRASP_ATTACH",
            "LIFT",
            "TRANSPORT",
            "DROP",
            "RELEASE",
            "HOME_FINAL",
        ],
        "AGARRE": [],
    }
    return list(sequences.get(flow_name, []))


def step_predict_next_phase(flow: str, phase: str) -> str:
    phase_name = str(phase or "").strip().upper()
    sequence = step_phase_sequence(flow)
    if phase_name in sequence:
        idx = sequence.index(phase_name)
        if idx + 1 < len(sequence):
            return sequence[idx + 1]
    return "--"


def step_phase_intent(flow: str, phase: str) -> str:
    flow_name = str(flow or "").strip().upper()
    phase_name = str(phase or "").strip().upper()
    intents = {
        "DIRECT": {
            "PICK_PRE_CLOSE_REF": "Ir a la referencia manual previa al cierre.",
            "APPROACH_COARSE": "Acercarse al objeto de forma gruesa.",
            "GRASP_DOWN_JOINT": "Bajar de forma conservadora preservando el XY útil.",
            "GRASP_ALIGN_IK": "Alinear la pinza fina sobre el objeto.",
            "PRE_CLOSE": "Colocar la pinza justo antes del cierre.",
            "CLOSE": "Cerrar la pinza sobre el objeto.",
            "ATTACH_GATE": "Comprobar si el objeto ha quedado agarrado.",
            "LIFT": "Elevar el objeto tras el agarre.",
            "CARRY": "Transportar el objeto hacia la cesta.",
            "RELEASE": "Soltar el objeto en la cesta.",
            "HOME_FINAL": "Volver a la pose final segura.",
        },
        "DIRECT2": {
            "MESA_1": "Ir al primer punto fijo sobre la mesa.",
            "POSE_BUENA": "Ir a la pose buena de referencia.",
            "MESA_2": "Salir de la mesa con el objeto.",
            "CESTA": "Ir a la cesta para dejar el objeto.",
        },
        "PICK_OBJECT": {
            "INICIO": "Confirmar inicio e ir a MESA antes del agarre.",
            "MESA": "Ir a pose MESA como punto de partida estándar.",
            "APPROACH": "Acercarse al objeto con MoveIt.",
            "PRE_GRASP": "Colocarse en pre-agarre con MoveIt.",
            "GRASP_DOWN": "Descender hasta la altura de agarre con MoveIt.",
            "GRASP_ATTACH": "Cerrar pinza y validar el agarre.",
            "LIFT": "Levantar el objeto con MoveIt.",
            "MESA_WITH_OBJECT": "Retirarse a MESA con el objeto.",
            "HOME_WITH_OBJECT": "Pasar por HOME manteniendo el objeto.",
            "TRANSPORT": "Transportar el objeto hacia la cesta con MoveIt.",
            "DROP": "Bajar para dejar el objeto con MoveIt.",
            "RELEASE": "Abrir la pinza y liberar el objeto.",
            "HOME_FINAL": "Volver a la pose final segura.",
        },
        "MOVEIT": {
            "APPROACH": "Acercarse al objeto con MoveIt.",
            "PRE_GRASP": "Colocarse en pre-agarre.",
            "GRASP_DOWN": "Descender hasta la altura de agarre.",
            "GRASP_ATTACH": "Cerrar y validar el agarre.",
            "LIFT": "Levantar el objeto.",
            "TRANSPORT": "Transportar el objeto hacia la cesta.",
            "DROP": "Bajar para dejar el objeto.",
            "RELEASE": "Abrir la pinza y liberar el objeto.",
            "HOME_FINAL": "Volver a la pose final segura.",
        },
    }
    return str(intents.get(flow_name, {}).get(phase_name) or "Sin descripción")


def step_phase_action_text(flow: str, phase: str, decision: str) -> str:
    raw_decision = str(decision or "").strip()
    if raw_decision:
        if "SIGUIENTE" in raw_decision.upper():
            return (
                raw_decision
                .replace("Pulse Siguiente", "Pulse Iniciar")
                .replace("pulsa Siguiente", "pulsa Iniciar")
                .replace("Siguiente", "Iniciar")
                .replace("Sigue", "Iniciar")
            )
        decision_map = {
            "PHASE_ENTER": "Iniciará la ejecución normal de esta fase.",
            "PHASE_ENTER_SKIP": "Dará esta fase por válida sin ejecutar un movimiento adicional.",
            "SKIP_ALIGN_PRECLOSE_OK": "Omitirá la alineación fina porque PRE_CLOSE ya es válido.",
            "ALIGN_RETRY": "Reintentará la alineación fina antes de continuar.",
            "PHASE_EXIT": "Cerrará esta fase y dejará preparada la siguiente.",
            "GATE_CHECK": "Verificará si se cumplen las condiciones para continuar.",
            "BLOCKED": "Mantendrá la fase bloqueada hasta que se cumpla la condición indicada.",
            "ENTER": "Iniciará la fase con los parámetros ya calculados.",
            "CLOSE_AND_ATTACH": "Cerrará la pinza y comprobará que el objeto ha quedado agarrado.",
            "LIFT_OBJECT": "Elevará el objeto después del agarre.",
            "JOINT_RETURN_TO_TABLE": "Llevará el robot a la pose Mesa con el objeto sujeto.",
            "JOINT_RETURN_HOME": "Llevará el robot a HOME manteniendo el objeto sujeto.",
            "TRANSPORT_TO_BASKET": "Transportará el objeto hacia la cesta.",
            "DROP_PHASE": "Bajará el objeto a la altura de suelta en la cesta.",
            "RELEASE_OBJECT": "Abrirá la pinza para soltar el objeto.",
            "RETURN_HOME": "Volverá a HOME como cierre de la secuencia.",
        }
        translated = decision_map.get(raw_decision.upper())
        if translated:
            return translated
    intent = step_phase_intent(flow, phase)
    if intent and intent != "Sin descripción":
        return intent
    return "Iniciará esta fase."


def step_phase_gripper_state(flow: str, phase: str) -> str:
    flow_name = str(flow or "").strip().upper()
    phase_name = str(phase or "").strip().upper()
    states = {
        "DIRECT": {
            "PICK_PRE_CLOSE_REF": "Abierta",
            "APPROACH_COARSE": "Abierta",
            "GRASP_DOWN_JOINT": "Abierta",
            "GRASP_ALIGN_IK": "Abierta",
            "PRE_CLOSE": "Abierta",
            "CLOSE": "Cerrada",
            "ATTACH_GATE": "Cerrada",
            "LIFT": "Cerrada",
            "CARRY": "Cerrada",
            "RELEASE": "Abierta",
            "HOME_FINAL": "Abierta",
        },
        "DIRECT2": {
            "MESA_1": "Abierta",
            "POSE_BUENA": "Cerrada",
            "MESA_2": "Cerrada",
            "CESTA": "Abierta",
        },
        "PICK_OBJECT": {
            "APPROACH": "Abierta",
            "PRE_GRASP": "Abierta",
            "GRASP_DOWN": "Abierta",
            "GRASP_ATTACH": "Cerrada",
            "LIFT": "Cerrada",
            "MESA_WITH_OBJECT": "Cerrada",
            "HOME_WITH_OBJECT": "Cerrada",
            "TRANSPORT": "Cerrada",
            "DROP": "Cerrada",
            "RELEASE": "Abierta",
            "HOME_FINAL": "Abierta",
        },
        "MOVEIT": {
            "APPROACH": "Abierta",
            "PRE_GRASP": "Abierta",
            "GRASP_DOWN": "Abierta",
            "GRASP_ATTACH": "Cerrada",
            "LIFT": "Cerrada",
            "TRANSPORT": "Cerrada",
            "DROP": "Cerrada",
            "RELEASE": "Abierta",
            "HOME_FINAL": "Abierta",
        },
    }
    return str(states.get(flow_name, {}).get(phase_name) or "--")
