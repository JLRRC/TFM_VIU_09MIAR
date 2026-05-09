#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_lifecycle_contract.py
# Contenido: F13 (2026-05-01) — verifica contrato LifecycleNode en 4 nodos críticos.
"""Tests offline del contrato LifecycleNode (F13).

Los 4 nodos críticos del stack pick & place migraron a
``LifecycleNode`` en F13:

* ``world_tf_publisher``
* ``system_state_manager``
* ``gripper_attach_backend``
* ``ur5_moveit_bridge``

Cada uno debe:

1. Heredar (directa o indirectamente) de ``LifecycleNode`` y NO de
   ``rclpy.node.Node`` plano.
2. Declarar el parámetro ``auto_activate`` (default True) que preserva
   backward-compat con los launch existentes.
3. Definir los callbacks ``on_configure``, ``on_activate``,
   ``on_deactivate``, ``on_cleanup`` y ``on_shutdown`` (interfaz
   estándar lifecycle).
4. Disparar ``trigger_configure()`` + ``trigger_activate()`` desde
   ``main()`` cuando ``auto_activate`` es True.

Estos tests son **estructurales sobre el código fuente** (AST + grep),
para poder ejecutarse offline sin rclpy/ROS instalado.
"""

from __future__ import annotations

import ast
import re
from pathlib import Path

import pytest


_REPO_SRC = Path(__file__).resolve().parents[2] / "ur5_tools" / "ur5_tools"

LIFECYCLE_NODES = [
    "world_tf_publisher.py",
    "system_state_manager.py",
    "gripper_attach_backend.py",
    # 2026-05-09: ur5_moveit_bridge.py BORRADO (path MoveIt-classic eliminado).
    # F16: tf_geometry_service standalone
    "tf_geometry_service.py",
    # F13b: nodos restantes migrados
    "release_objects_service.py",
    "gz_pose_bridge.py",
]

REQUIRED_CALLBACKS = (
    "on_configure",
    "on_activate",
    "on_deactivate",
    "on_cleanup",
    "on_shutdown",
)


def _read_module(name: str) -> str:
    path = _REPO_SRC / name
    assert path.is_file(), f"módulo {name} no encontrado en {_REPO_SRC}"
    return path.read_text(encoding="utf-8")


@pytest.mark.parametrize("module_name", LIFECYCLE_NODES)
def test_imports_lifecycle_node(module_name):
    """Cada nodo F13 debe importar LifecycleNode de rclpy.lifecycle."""
    src = _read_module(module_name)
    pat = r"from\s+rclpy\.lifecycle\s+import\s+[^\n]*LifecycleNode"
    assert re.search(pat, src), (
        f"{module_name}: falta import de LifecycleNode desde rclpy.lifecycle"
    )


@pytest.mark.parametrize("module_name", LIFECYCLE_NODES)
def test_class_inherits_lifecycle_node(module_name):
    """La clase principal debe heredar de LifecycleNode (no Node plano)."""
    src = _read_module(module_name)
    tree = ast.parse(src)
    classes = [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]
    # Filtramos solo clases que tengan __init__ y al menos una base — clase principal
    candidates = [c for c in classes if c.bases]
    assert candidates, f"{module_name}: no se encontró clase con bases"
    main_class = None
    for c in candidates:
        base_names = []
        for b in c.bases:
            if isinstance(b, ast.Name):
                base_names.append(b.id)
            elif isinstance(b, ast.Attribute):
                base_names.append(b.attr)
        if "LifecycleNode" in base_names:
            main_class = c
            break
    assert main_class is not None, (
        f"{module_name}: ninguna clase hereda explícitamente de LifecycleNode"
    )


@pytest.mark.parametrize("module_name", LIFECYCLE_NODES)
def test_declares_auto_activate_parameter(module_name):
    """Cada nodo declara el parámetro 'auto_activate' (default True)."""
    src = _read_module(module_name)
    pat = r'declare_parameter\(\s*["\']auto_activate["\']\s*,\s*True\s*\)'
    assert re.search(pat, src), (
        f"{module_name}: falta declare_parameter('auto_activate', True)"
    )


@pytest.mark.parametrize("module_name", LIFECYCLE_NODES)
@pytest.mark.parametrize("callback", REQUIRED_CALLBACKS)
def test_defines_lifecycle_callback(module_name, callback):
    """Cada nodo debe definir los 5 callbacks lifecycle estándar."""
    src = _read_module(module_name)
    pat = rf"def\s+{re.escape(callback)}\s*\("
    assert re.search(pat, src), (
        f"{module_name}: falta callback lifecycle '{callback}'"
    )


@pytest.mark.parametrize("module_name", LIFECYCLE_NODES)
def test_main_triggers_auto_activate(module_name):
    """main() debe disparar trigger_configure + trigger_activate cuando auto_activate=True."""
    src = _read_module(module_name)
    assert "trigger_configure" in src, (
        f"{module_name}: main() no llama a trigger_configure()"
    )
    assert "trigger_activate" in src, (
        f"{module_name}: main() no llama a trigger_activate()"
    )
    assert "auto_activate" in src, (
        f"{module_name}: main() no chequea el parámetro auto_activate"
    )


def test_lifecycle_helpers_module_available():
    """Los enums/transiciones puras de tfm_orchestrator.lifecycle_helpers
    siguen disponibles para uso compartido por nodos F13."""
    helpers_path = (
        _REPO_SRC.parents[1]
        / "tfm_orchestrator"
        / "tfm_orchestrator"
        / "lifecycle_helpers.py"
    )
    assert helpers_path.is_file(), (
        "tfm_orchestrator/lifecycle_helpers.py debe existir como referencia común"
    )
    src = helpers_path.read_text(encoding="utf-8")
    for sym in (
        "LifecycleState",
        "LifecycleTransition",
        "allowed_transitions",
        "next_state",
        "reject_reason_for_state",
    ):
        assert sym in src, f"lifecycle_helpers debe exportar {sym!r}"
