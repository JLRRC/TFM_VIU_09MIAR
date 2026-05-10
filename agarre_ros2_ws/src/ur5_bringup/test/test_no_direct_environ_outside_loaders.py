#!/usr/bin/env python3
"""F2 audit (2026-05-10): bloquea regresiones de lectura directa de env.

Las lecturas de ``os.environ`` deben pasar por los helpers tipados de:

* ``ur5_qt_panel.panel_env`` (helpers ``env_str/env_int/env_bool/...``)
* ``ur5_tools.workspace_paths`` (``get_ws_dir/get_gz_partition/...``)
* ``ur5_qt_panel.panel_*_params`` (dataclasses de configuración)
* ``ur5_tools.moveit_bridge_utils`` (``bridge_env_*`` helpers)
* ``ur5_qt_panel.panel_motion_helpers`` (helpers internos del módulo)

Otros módulos productivos no deben llamar ``os.environ.get`` ni
``os.environ.setdefault`` directamente: deben delegar en uno de los
helpers anteriores. Los launches sí pueden leerlo (es idiomático en
ROS 2), igual que los archivos ``_helpers.py`` que constituyen la API
de carga de env.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

WS = Path(__file__).resolve().parents[3]

# Carpetas excluidas (cache, build, etc.).
EXCLUDED_DIRS = {
    "build",
    "install",
    "log",
    "historico",
    "auditoria",
    ".pytest_cache",
    ".mypy_cache",
    ".ruff_cache",
    "__pycache__",
    "reports",
}

# Allowlist explícita: archivos que SÍ pueden leer os.environ porque
# constituyen la capa de abstracción del env. Cualquier nuevo helper
# debe añadirse aquí explícitamente.
ALLOWED_FILES = {
    # Helpers tipados del panel.
    "src/ur5_qt_panel/ur5_qt_panel/panel_env.py",
    # Dataclasses de configuración del panel.
    "src/ur5_qt_panel/ur5_qt_panel/panel_settings.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo_params.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_ros_params.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_ui_params.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_tfm_params.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_launchers_params.py",
    # Helpers internos del panel.
    "src/ur5_qt_panel/ur5_qt_panel/panel_motion_helpers.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_config.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_startup.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_system_status.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_gz_startup.py",
    "src/ur5_qt_panel/ur5_qt_panel/panel_launchers.py",
    "src/ur5_qt_panel/ur5_qt_panel/directo_gate_evaluator.py",
    # Helpers tipados de ur5_tools.
    "src/ur5_tools/ur5_tools/workspace_paths.py",
    "src/ur5_tools/ur5_tools/moveit_bridge_utils.py",
    # Bridge a subprocess gz topic; necesita env.copy() literal.
    "src/ur5_tools/ur5_tools/gz_pose_bridge.py",
    # tfm_grasping: usos defaultados aceptables (pendiente migrar en F8).
    "src/tfm_grasping/tfm_grasping/model.py",
    # Launch files: idiomáticos en ROS 2, no aplica restricción.
    "src/ur5_bringup/launch/ur5_stack.launch.py",
    "src/ur5_bringup/launch/launch_helpers.py",
    "src/ur5_bringup/launch/stack_factories.py",
    "src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py",
}

ENVIRON_PATTERN = re.compile(r"\bos\.environ\b")


def _ws_relpath(p: Path) -> str:
    return str(p.relative_to(WS))


def _iter_production_files() -> list[Path]:
    files: list[Path] = []
    for root in (WS / "src",):
        for path in root.rglob("*.py"):
            if not path.is_file():
                continue
            parts = path.parts
            if any(part in EXCLUDED_DIRS for part in parts):
                continue
            # Tests están permitidos: usan os.environ para fixtures.
            if "test" in parts and any(part.startswith("test") for part in parts):
                continue
            # __init__.py → permitido.
            if path.name == "__init__.py":
                continue
            files.append(path)
    return files


def test_no_direct_environ_outside_loaders() -> None:
    offenders: list[str] = []
    for path in _iter_production_files():
        rel = _ws_relpath(path)
        if rel in ALLOWED_FILES:
            continue
        try:
            txt = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        for lineno, line in enumerate(txt.splitlines(), start=1):
            stripped = line.strip()
            if stripped.startswith("#"):
                continue
            if ENVIRON_PATTERN.search(line):
                offenders.append(f"  {rel}:{lineno}: {stripped[:100]}")
    assert not offenders, (
        "Lectura directa de os.environ fuera de helpers/loaders "
        "(añade el archivo a ALLOWED_FILES en este test si es un "
        "nuevo helper, o usa los helpers tipados existentes):\n"
        + "\n".join(offenders[:30])
    )


def test_allowed_files_exist() -> None:
    """ALLOWED_FILES no debe acumular paths obsoletos."""
    missing = [rel for rel in ALLOWED_FILES if not (WS / rel).is_file()]
    assert not missing, (
        f"ALLOWED_FILES referencia paths inexistentes: {missing}"
    )


@pytest.mark.parametrize("helper_file,sym", [
    ("src/ur5_qt_panel/ur5_qt_panel/panel_env.py", "def env_str"),
    ("src/ur5_qt_panel/ur5_qt_panel/panel_env.py", "def env_int"),
    ("src/ur5_qt_panel/ur5_qt_panel/panel_env.py", "def env_bool"),
    ("src/ur5_tools/ur5_tools/workspace_paths.py", "def get_ws_dir"),
    ("src/ur5_tools/ur5_tools/workspace_paths.py", "def get_gz_partition"),
])
def test_canonical_helper_present(helper_file: str, sym: str) -> None:
    """Verifica que la API tipada existe y no se rompe accidentalmente."""
    p = WS / helper_file
    assert p.is_file()
    assert sym in p.read_text(encoding="utf-8"), (
        f"Helper canónico {sym!r} ausente de {helper_file}"
    )
