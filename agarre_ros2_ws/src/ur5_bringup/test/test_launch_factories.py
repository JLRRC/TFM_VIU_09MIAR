#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/test_launch_factories.py
# Contenido: F17 (2026-05-01) — verifica el contrato de las factories del launch modular.
"""Tests offline del refactor F17 — launch modular.

F17 extrae los grupos de actions del launch principal a factories
laterales para reducir el tamaño de ``ur5_stack.launch.py`` y mejorar
la separación de responsabilidades:

* ``gz_factory.build_gz_actions`` — Gazebo + bridges + guards.
* ``runtime_nodes_factory.build_runtime_node_actions`` — TF, state,
  release, attach, scene_sync, MoveIt bridge.

Estos tests son **estructurales** (AST + grep) para correr offline
sin ROS sourceado. Verifican:

1. Cada factory expone una función callable con su nombre canónico.
2. ``ur5_stack.launch.py`` importa las dos factories.
3. ``ur5_stack.launch.py`` queda por debajo del umbral F17 (<= 800
   LOC) tras el refactor — guardrail contra regresión.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest


_LAUNCH_DIR = Path(__file__).resolve().parent.parent / "launch"


def _read(name: str) -> str:
    path = _LAUNCH_DIR / name
    assert path.is_file(), f"módulo {name} no encontrado en {_LAUNCH_DIR}"
    return path.read_text(encoding="utf-8")


def test_gz_factory_exists():
    src = _read("gz_factory.py")
    assert "def build_gz_actions" in src, (
        "gz_factory.py debe exportar build_gz_actions(...)"
    )


def test_runtime_nodes_factory_exists():
    src = _read("runtime_nodes_factory.py")
    assert "def build_runtime_node_actions" in src, (
        "runtime_nodes_factory.py debe exportar build_runtime_node_actions(...)"
    )


def test_ur5_stack_imports_factories():
    src = _read("ur5_stack.launch.py")
    assert re.search(
        r"from\s+gz_factory\s+import\s+build_gz_actions", src
    ), "ur5_stack.launch.py debe importar build_gz_actions de gz_factory"
    assert re.search(
        r"from\s+runtime_nodes_factory\s+import\s+build_runtime_node_actions",
        src,
    ), (
        "ur5_stack.launch.py debe importar build_runtime_node_actions de "
        "runtime_nodes_factory"
    )


def test_ur5_stack_invokes_factories():
    """El launch principal usa las factories, no inline las actions."""
    src = _read("ur5_stack.launch.py")
    assert "build_gz_actions(" in src, (
        "ur5_stack.launch.py debe invocar build_gz_actions(...)"
    )
    assert "build_runtime_node_actions(" in src, (
        "ur5_stack.launch.py debe invocar build_runtime_node_actions(...)"
    )


def test_ur5_stack_under_800_lines():
    """Guardrail F17: ur5_stack.launch.py <= 800 LOC tras la extracción."""
    src = _read("ur5_stack.launch.py")
    line_count = src.count("\n") + (0 if src.endswith("\n") else 1)
    assert line_count <= 800, (
        f"ur5_stack.launch.py creció a {line_count} LOC; el umbral F17 es "
        "<=800. Si el aumento es legítimo, considera extraer otro grupo de "
        "actions a un factory adicional."
    )


@pytest.mark.parametrize(
    "factory_name,expected_kwargs",
    [
        (
            "gz_factory.py",
            ("world_file", "render_engine", "gui_config_file", "headless",
             "launch_gazebo", "launch_bridge", "runtime_yaml", "world_name",
             "use_sim_time"),
        ),
        (
            "runtime_nodes_factory.py",
            ("use_sim_time", "world_name", "world_file", "camera_required",
             "controller_manager", "launch_moveit", "launch_world_tf",
             "launch_system_state", "launch_release_service",
             "launch_attach_backend", "launch_scene_sync",
             "launch_tf_geometry_service",
             "demo_transport_objects", "bridge_params", "system_state_yaml",
             "system_state_extras", "attach_extras"),
        ),
    ],
    ids=["gz", "runtime_nodes"],
)
def test_factory_signature_keywords(factory_name, expected_kwargs):
    """Cada factory debe exponer los keyword arguments esperados.

    Esto previene que un refactor accidental rompa el contrato sin un
    fallo de import (los factories aceptan ``*`` keyword-only).
    """
    src = _read(factory_name)
    for kw in expected_kwargs:
        # Buscamos una línea con el keyword seguido de ':' o '=' o ',' (Lo cual
        # cubre tanto annotations como defaults).
        pat = rf"\b{re.escape(kw)}\s*[:=,)]"
        assert re.search(pat, src), (
            f"{factory_name}: el keyword '{kw}' debe estar en la firma de "
            "la factory"
        )
