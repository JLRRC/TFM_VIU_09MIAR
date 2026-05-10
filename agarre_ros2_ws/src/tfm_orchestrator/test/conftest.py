#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/conftest.py
"""Pytest config: añade el package root a sys.path para tests sin install.

Tras F2 (auditoría 2026-05-10) phase_dispatch.py importa
``ur5_tools.geometry_constants``. Para tests offline (colcon test sin
source install) añadimos también el package root de ur5_tools al
sys.path; si aún no se puede importar, inyectamos un stub mínimo.
"""

import os
import sys
import types

PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

# F2: añadir ur5_tools/ (paquete hermano) para resolver
# `from ur5_tools.geometry_constants import ...` sin depender de install.
WS_SRC_ROOT = os.path.abspath(os.path.join(PKG_ROOT, ".."))
UR5_TOOLS_ROOT = os.path.join(WS_SRC_ROOT, "ur5_tools")
if os.path.isdir(UR5_TOOLS_ROOT) and UR5_TOOLS_ROOT not in sys.path:
    sys.path.insert(0, UR5_TOOLS_ROOT)

# Si tras lo anterior ur5_tools no se puede importar (ejecuciones más
# aisladas), inyectar un stub mínimo de geometry_constants.
if "ur5_tools.geometry_constants" not in sys.modules:
    try:
        import ur5_tools.geometry_constants  # noqa: F401
    except ModuleNotFoundError:
        stub_pkg = sys.modules.setdefault(
            "ur5_tools", types.ModuleType("ur5_tools")
        )
        gc_mod = types.ModuleType("ur5_tools.geometry_constants")
        gc_mod.BASE_LINK_IN_WORLD = (-0.85, 0.0, 0.850)
        gc_mod.world_to_base = lambda p: (
            p[0] - gc_mod.BASE_LINK_IN_WORLD[0],
            p[1] - gc_mod.BASE_LINK_IN_WORLD[1],
            p[2] - gc_mod.BASE_LINK_IN_WORLD[2],
        )
        gc_mod.base_to_world = lambda p: (
            p[0] + gc_mod.BASE_LINK_IN_WORLD[0],
            p[1] + gc_mod.BASE_LINK_IN_WORLD[1],
            p[2] + gc_mod.BASE_LINK_IN_WORLD[2],
        )
        stub_pkg.geometry_constants = gc_mod  # type: ignore[attr-defined]
        sys.modules["ur5_tools.geometry_constants"] = gc_mod
