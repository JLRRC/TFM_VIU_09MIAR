#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/test_no_duplicate_node_names_in_launch.py
# Contenido: F8 — guardrail anti bridges/nodes duplicados en launch files.
"""F8 — Guardrail: no hay node executables duplicados en los launch files.

Histórico relevante: el audit detectó múltiples instancias zombi de
``parameter_bridge`` corriendo simultáneamente tras runs sucesivos del
stack (5+ procesos compitiendo por el mismo nombre de nodo
``ros_gz_bridge_main``). Esto causa flakiness en /clock, /joint_states,
y otros topics bridged.

Este test es ESTÁTICO (no levanta el stack): parsea los launch files de
ur5_bringup buscando declaraciones ``Node(...)`` o ``ExecuteProcess(...)``
con el mismo executable + name combinación. Si detecta duplicados sin
condición exclusiva (IfCondition / UnlessCondition), falla.

Pure stdlib + AST. No requiere ROS sourced.
"""
from __future__ import annotations

import ast
from pathlib import Path
from typing import Dict, List, Tuple

import pytest


WS_ROOT = Path(__file__).resolve().parents[3]
LAUNCH_DIRS = [
    WS_ROOT / "src" / "ur5_bringup" / "launch",
    WS_ROOT / "src" / "ur5_moveit_config" / "launch",
]


def _collect_node_declarations(py_path: Path) -> List[Tuple[str, str, str]]:
    """Devuelve [(executable, name, kwargs_repr)] de cada Node(...) o ExecuteProcess(...) en el archivo.

    Hace un pase AST por todas las llamadas. No interpreta condiciones —
    el test las usa para determinar si dos declaraciones idénticas son
    realmente duplicadas o exclusivas.
    """
    out: List[Tuple[str, str, str]] = []
    try:
        tree = ast.parse(py_path.read_text(encoding="utf-8"), filename=str(py_path))
    except SyntaxError:
        return out
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if isinstance(func, ast.Name):
            func_name = func.id
        elif isinstance(func, ast.Attribute):
            func_name = func.attr
        else:
            continue
        if func_name not in {"Node", "ExecuteProcess", "LifecycleNode"}:
            continue
        # Extraer kwargs `executable=` y `name=` (literal strings only).
        kwargs = {kw.arg: kw.value for kw in node.keywords if kw.arg is not None}
        executable = ""
        name = ""
        if "executable" in kwargs and isinstance(kwargs["executable"], ast.Constant):
            executable = str(kwargs["executable"].value)
        if "name" in kwargs and isinstance(kwargs["name"], ast.Constant):
            name = str(kwargs["name"].value)
        # Detect IfCondition / UnlessCondition presence (literal check).
        has_condition = "condition" in kwargs
        cond_repr = "<conditional>" if has_condition else "<unconditional>"
        if executable or name:
            out.append((executable, name, cond_repr))
    return out


def test_f8_no_unconditional_duplicate_node_executables() -> None:
    """F8 — no hay 2+ Node con el mismo (executable, name) sin condición.

    Si dos Node tienen mismo executable+name y AMBOS son
    unconditional → duplicado garantizado en runtime → bridges fighting.
    """
    duplicates: List[str] = []
    for launch_dir in LAUNCH_DIRS:
        if not launch_dir.exists():
            continue
        for py_path in launch_dir.rglob("*.py"):
            decls = _collect_node_declarations(py_path)
            seen: Dict[Tuple[str, str], List[str]] = {}
            for exe, name, cond in decls:
                if not exe and not name:
                    continue
                key = (exe, name)
                seen.setdefault(key, []).append(cond)
            for key, conds in seen.items():
                unconditional = [c for c in conds if c == "<unconditional>"]
                if len(unconditional) > 1:
                    exe, name = key
                    duplicates.append(
                        f"{py_path.relative_to(WS_ROOT)}: "
                        f"{exe}/{name} declarado {len(unconditional)}x sin condición"
                    )
    assert not duplicates, (
        "Nodos duplicados sin condición exclusiva (causa bridges fighting):\n  "
        + "\n  ".join(duplicates)
    )


def test_f8_launch_file_uses_unique_remappings() -> None:
    """F8 — cada Node tiene remappings consistentes (no remap a topic ya usado).

    No es exhaustivo (no resuelve LaunchConfiguration runtime), pero
    detecta el caso obvio: dos Node con el mismo `remappings=[('/topic',
    '/clock')]` literal. Si esto pasa, es bug 100%.
    """
    # Skipped por ahora: requiere análisis runtime de LaunchConfigurations.
    # Placeholder para F8-step2 si surge problema concreto.
    pytest.skip("F8-step2 placeholder — runtime LaunchConfiguration analysis pendiente")


def test_f8_launch_files_define_main_executables() -> None:
    """F8 — los launch files declaran al menos los executables core esperados.

    Si alguien borra accidentalmente `move_group` o `gz sim` del stack,
    este test detecta la regresión. Lista de executables esperados es
    permisiva — solo verifica que TODOS aparezcan SOMEWHERE en el dir.
    """
    expected_executables = {
        "move_group",        # MoveIt2
        "robot_state_publisher",
        "controller_manager",
    }
    found: set[str] = set()
    for launch_dir in LAUNCH_DIRS:
        if not launch_dir.exists():
            continue
        for py_path in launch_dir.rglob("*.py"):
            decls = _collect_node_declarations(py_path)
            for exe, _, _ in decls:
                if exe in expected_executables:
                    found.add(exe)
            # Algunos executables se invocan via ExecuteProcess con cmd
            # — detectarlos en string literals.
            text = py_path.read_text(encoding="utf-8")
            for exe in expected_executables:
                if f'"{exe}"' in text or f"'{exe}'" in text:
                    found.add(exe)
    missing = expected_executables - found
    assert not missing, (
        f"Executables core ausentes de todos los launch files: {missing}. "
        "Indica una regresión grave del stack base."
    )
