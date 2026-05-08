#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo/decision_helpers.py
"""F3-step40 (2026-05-08) — Decision string classifiers (puros).

Extraído de panel_pick_demo.py:245.

API:
- ``execution_type_from_decision(decision)`` → "preset" | "geometrico" | "hibrido".
"""

from __future__ import annotations


def execution_type_from_decision(decision: str | None) -> str:
    """Clasifica el tipo de ejecución a partir del decision string.

    Reglas:
    - ``fallback_joint_preset`` o ``target_unavailable`` → "preset".
    - ``direct_ik_move`` o ``direct_ik_move_refresh`` → "geometrico".
    - Cualquier otro string no vacío → "hibrido".
    - None / vacío → "hibrido" (default conservador).
    """
    decision_txt = str(decision or "").strip().lower()
    if "fallback_joint_preset" in decision_txt or "target_unavailable" in decision_txt:
        return "preset"
    if decision_txt in {"direct_ik_move", "direct_ik_move_refresh"}:
        return "geometrico"
    if decision_txt:
        return "hibrido"
    return "hibrido"
