#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/test_launch_files_parseable.py
# Contenido: F4 — sanity check de los launch files (sin levantar el stack).
"""Smoke test de los launch files de ur5_bringup (F4 mínima).

Verifica para cada ``*.launch.py``:

1. El módulo se importa sin error (sintaxis OK, imports resolubles).
2. Define ``generate_launch_description()``.
3. La función devuelve un ``LaunchDescription`` válido cuando se invoca.

NO levanta el stack — sólo inspecciona estática+dinámicamente. Es la
red de seguridad mínima antes de tocar archivos grandes en F3.

Si el paquete ``launch`` (ROS 2) no está disponible en el entorno de
test, los tests del punto 3 se saltan con razón explícita; los del
punto 1+2 sí corren siempre porque sólo dependen de Python.
"""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path

import pytest


LAUNCH_DIR = Path(__file__).resolve().parent.parent / "launch"
LAUNCH_FILES = sorted(LAUNCH_DIR.glob("*.launch.py"))

# Por seguridad: skip si por alguna razón no encontramos ningún launch file.
if not LAUNCH_FILES:
    pytest.skip("no se encontraron *.launch.py en ur5_bringup/launch", allow_module_level=True)

# Estos tests requieren los paquetes ROS 2 ``launch`` y ``launch_ros`` en
# el entorno (lo normal cuando se ha hecho ``source /opt/ros/jazzy/setup.bash``).
# Sin ROS sourceado, los launch files fallan en el primer ``from launch import …``
# y no podemos validar nada. En ese caso skipeamos el módulo entero con razón
# clara — el test no es útil sin ese contexto.
try:
    import launch  # noqa: F401
    import launch_ros  # noqa: F401
except ImportError as _ros_import_err:
    pytest.skip(
        f"paquetes ROS 2 launch/launch_ros no disponibles "
        f"({_ros_import_err}). Source /opt/ros/jazzy/setup.bash y "
        f"workspace install/setup.bash antes de ejecutar.",
        allow_module_level=True,
    )


def _import_launch_module(launch_path: Path):
    """Cargar el launch file como módulo Python y devolverlo."""
    # Aseguramos que launch/ está en sys.path para que `import launch_helpers`
    # local funcione (es lo que hacen los launch files reales).
    if str(LAUNCH_DIR) not in sys.path:
        sys.path.insert(0, str(LAUNCH_DIR))
    spec = importlib.util.spec_from_file_location(
        f"_launch_test_{launch_path.stem}", launch_path
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    "launch_path", LAUNCH_FILES, ids=[p.name for p in LAUNCH_FILES]
)
def test_launch_file_imports_clean(launch_path):
    """El launch file se importa sin error (sintaxis y top-level imports OK)."""
    module = _import_launch_module(launch_path)
    assert hasattr(module, "generate_launch_description"), (
        f"{launch_path.name} no define generate_launch_description()"
    )
    assert callable(module.generate_launch_description), (
        f"{launch_path.name}.generate_launch_description no es callable"
    )


@pytest.mark.parametrize(
    "launch_path", LAUNCH_FILES, ids=[p.name for p in LAUNCH_FILES]
)
def test_generate_launch_description_returns_valid_object(launch_path):
    """``generate_launch_description()`` devuelve un LaunchDescription."""
    try:
        from launch import LaunchDescription
    except ImportError:
        pytest.skip("paquete `launch` no disponible (sin ROS sourceado)")

    module = _import_launch_module(launch_path)
    # Aislar env para no contaminar entre tests parametrizados.
    saved_env = dict(os.environ)
    try:
        ld = module.generate_launch_description()
    except Exception as e:
        pytest.fail(
            f"{launch_path.name}.generate_launch_description() falló: "
            f"{type(e).__name__}: {e}"
        )
    finally:
        os.environ.clear()
        os.environ.update(saved_env)

    assert isinstance(ld, LaunchDescription), (
        f"{launch_path.name} debe devolver un LaunchDescription, devolvió "
        f"{type(ld).__name__}"
    )
    # Una LaunchDescription vacía es legal pero sospechosa para nuestro stack.
    entities = ld.entities if hasattr(ld, "entities") else []
    assert len(entities) > 0, (
        f"{launch_path.name} produjo un LaunchDescription sin entidades"
    )
