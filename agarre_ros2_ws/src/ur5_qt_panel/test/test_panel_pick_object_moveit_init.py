#!/usr/bin/env python3
"""Tests estáticos para validar los cambios de auditoría MOVEIT en panel_pick_object.

Comprobaciones sin necesidad de un entorno ROS activo: importaciones, prefijos
de log y fallback del frame EE en el worker de MOVEIT.
"""
from __future__ import annotations

import ast
import re
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[3]
_PICK_OBJECT_SRC = (
    _ROOT / "src" / "ur5_qt_panel" / "ur5_qt_panel" / "panel_pick_object.py"
)
_SOURCE_TEXT = _PICK_OBJECT_SRC.read_text(encoding="utf-8")


# ---------------------------------------------------------------------------
# Importaciones
# ---------------------------------------------------------------------------


def test_rg2_tcp_frame_is_imported() -> None:
    """RG2_TCP_FRAME debe estar importado desde ur5_tools.gripper_geometry."""
    tree = ast.parse(_SOURCE_TEXT)
    imported = False
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if node.module == "ur5_tools.gripper_geometry":
                names = [alias.name for alias in node.names]
                if "RG2_TCP_FRAME" in names:
                    imported = True
    assert imported, "RG2_TCP_FRAME no está importado desde ur5_tools.gripper_geometry"


def test_tf_ready_status_is_imported() -> None:
    """tf_ready_status debe estar importado desde .panel_readiness."""
    tree = ast.parse(_SOURCE_TEXT)
    imported = False
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if node.module in (".panel_readiness", "panel_readiness"):
                names = [alias.name for alias in node.names]
                if "tf_ready_status" in names:
                    imported = True
    assert imported, "tf_ready_status no está importado desde .panel_readiness"


# ---------------------------------------------------------------------------
# Prefijos de log en el bloque INIT
# ---------------------------------------------------------------------------


def test_moveit_init_log_prefix_present() -> None:
    """El bloque de inicialización debe emitir logs con prefijo [PICK][MOVEIT][INIT]."""
    assert "[PICK][MOVEIT][INIT]" in _SOURCE_TEXT, (
        "No se encontró ningún log [PICK][MOVEIT][INIT] en panel_pick_object.py"
    )


def test_moveit_init_tf_and_ee_ok_log_present() -> None:
    """Debe existir el log tf_and_ee=OK para indicar init correcto."""
    assert "tf_and_ee=OK" in _SOURCE_TEXT, (
        "No se encontró 'tf_and_ee=OK' en el bloque INIT de MOVEIT"
    )


def test_moveit_init_tf_and_ee_fail_log_present() -> None:
    """Debe existir el log tf_and_ee=FAIL para señalizar fallos de TF/EE."""
    assert "tf_and_ee=FAIL" in _SOURCE_TEXT, (
        "No se encontró 'tf_and_ee=FAIL' en el bloque INIT de MOVEIT"
    )


def test_moveit_init_calls_tf_ready_status() -> None:
    """El bloque INIT debe invocar tf_ready_status para validar TF en caliente."""
    assert "tf_ready_status(panel)" in _SOURCE_TEXT, (
        "No se encontró llamada a tf_ready_status(panel) en panel_pick_object.py"
    )


# ---------------------------------------------------------------------------
# Prefijo de log en el gate de pasos (STEP)
# ---------------------------------------------------------------------------


def test_moveit_step_log_prefix_present() -> None:
    """_step_phase_gate debe emitir logs con prefijo [PICK][MOVEIT][STEP]."""
    assert "[PICK][MOVEIT][STEP]" in _SOURCE_TEXT, (
        "No se encontró ningún log [PICK][MOVEIT][STEP] en panel_pick_object.py"
    )


def test_moveit_step_gate_reuse_log_present() -> None:
    """El gate debe registrar reutilización de fase con 'gate_reuse'."""
    assert "gate_reuse" in _SOURCE_TEXT, (
        "No se encontró 'gate_reuse' en el log [PICK][MOVEIT][STEP]"
    )


# ---------------------------------------------------------------------------
# Prefijo LIFECYCLE y ERROR en el worker
# ---------------------------------------------------------------------------


def test_moveit_lifecycle_worker_start_log_present() -> None:
    """El worker debe emitir [PICK][MOVEIT][LIFECYCLE] stage=worker_start."""
    assert "[PICK][MOVEIT][LIFECYCLE]" in _SOURCE_TEXT, (
        "No se encontró [PICK][MOVEIT][LIFECYCLE] en panel_pick_object.py"
    )
    assert "stage=worker_start" in _SOURCE_TEXT, (
        "No se encontró 'stage=worker_start' en el log LIFECYCLE de MOVEIT"
    )


def test_moveit_error_log_prefix_present() -> None:
    """El handler de excepciones debe emitir [PICK][MOVEIT][ERROR]."""
    assert "[PICK][MOVEIT][ERROR]" in _SOURCE_TEXT, (
        "No se encontró [PICK][MOVEIT][ERROR] en panel_pick_object.py"
    )


# ---------------------------------------------------------------------------
# Fallback del frame EE en el worker
# ---------------------------------------------------------------------------


def test_worker_ee_frame_fallback_uses_rg2_tcp_frame() -> None:
    """El fallback de measured_ee_frame en el worker debe usar RG2_TCP_FRAME."""
    lines = _SOURCE_TEXT.splitlines()
    pattern = re.compile(
        r"measured_ee_frame\s*=\s*.*panel\._ee_frame_effective.*or\s+RG2_TCP_FRAME"
    )
    matches = [ln.strip() for ln in lines if pattern.search(ln)]
    assert len(matches) >= 1, (
        "No se encontró ninguna asignación 'measured_ee_frame = panel._ee_frame_effective or RG2_TCP_FRAME' "
        "en panel_pick_object.py — el fallback incorrecto (RG2_PINCH_CENTER_FRAME) puede estar activo"
    )


def test_worker_ee_frame_fallback_never_uses_rg2_pinch_center_as_default() -> None:
    """No debe quedar ningún fallback medido con RG2_PINCH_CENTER_FRAME como valor por defecto."""
    lines = _SOURCE_TEXT.splitlines()
    bad_pattern = re.compile(
        r"measured_ee_frame\s*=\s*.*panel\._ee_frame_effective\s*or\s+RG2_PINCH_CENTER_FRAME"
    )
    bad_matches = [ln.strip() for ln in lines if bad_pattern.search(ln)]
    assert bad_matches == [], (
        "Se encontraron asignaciones de measured_ee_frame con fallback RG2_PINCH_CENTER_FRAME:\n"
        + "\n".join(bad_matches)
    )
