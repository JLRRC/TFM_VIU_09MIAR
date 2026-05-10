#!/usr/bin/env python3
"""F13 (auditoría 2026-05-10): regression guard sobre publishers de TF.

Detecta riesgos de DUPLICACIÓN de TF estático antes de que lleguen
a runtime. Cubre dos clases de bug:

  1. ``static_transform_publisher`` invocado en launch files mientras
     ``world_tf_publisher`` también está activo. Dos publishers para
     el mismo frame (``world → base_link``) generan TF flickering en
     RViz/MoveIt.

  2. ``world_tf_publisher`` lanzado más de una vez en un mismo launch.

Este test es 100% AST/grep estático — no levanta ROS ni Gazebo. Cubre
los launch files de ``ur5_bringup`` y ``ur5_moveit_config``.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import List

WS_ROOT = Path(__file__).resolve().parents[3]


def _all_launch_files() -> List[Path]:
    out: List[Path] = []
    for pkg in ("ur5_bringup", "ur5_moveit_config"):
        launch_dir = WS_ROOT / "src" / pkg / "launch"
        if launch_dir.is_dir():
            out.extend(sorted(launch_dir.glob("*.launch.py")))
            out.extend(sorted(launch_dir.glob("*.py")))
    return [p for p in out if p.is_file() and "__pycache__" not in p.parts]


def test_no_static_transform_publisher_with_world_tf():
    """Si algún launch lanza ``static_transform_publisher``, NO debe
    coexistir con ``world_tf_publisher`` en el mismo launch."""
    static_re = re.compile(r"static_transform_publisher")
    world_tf_re = re.compile(r"world_tf_publisher")
    offenders = []
    for path in _all_launch_files():
        text = path.read_text(encoding="utf-8", errors="replace")
        # Excluir comentarios al hacer la check (línea por línea, strip
        # de '# ...').
        active_lines = []
        for line in text.splitlines():
            no_comment = line.split("#", 1)[0]
            active_lines.append(no_comment)
        active = "\n".join(active_lines)
        has_static = bool(static_re.search(active))
        has_world_tf = bool(world_tf_re.search(active))
        if has_static and has_world_tf:
            offenders.append(str(path.relative_to(WS_ROOT)))
    assert not offenders, (
        "Estos launch files lanzan static_transform_publisher Y "
        "world_tf_publisher simultáneamente — riesgo de duplicación TF:"
        + "\n  " + "\n  ".join(offenders)
    )


def test_world_tf_publisher_launched_at_most_once_per_file():
    """Cada launch file debe lanzar como máximo una instancia del
    ``world_tf_publisher`` (ejecutable). Detecta llamadas a Node con
    ``executable="world_tf_publisher"``."""
    pattern = re.compile(
        r'executable\s*=\s*["\']world_tf_publisher["\']'
    )
    offenders = []
    for path in _all_launch_files():
        text = path.read_text(encoding="utf-8", errors="replace")
        active_lines = []
        for line in text.splitlines():
            no_comment = line.split("#", 1)[0]
            active_lines.append(no_comment)
        active = "\n".join(active_lines)
        count = len(pattern.findall(active))
        if count > 1:
            offenders.append(f"{path.relative_to(WS_ROOT)} (count={count})")
    assert not offenders, (
        "Estos launch files instancian world_tf_publisher más de una vez:"
        + "\n  " + "\n  ".join(offenders)
    )


def test_at_least_one_world_tf_publisher_in_runtime_factories():
    """Sanity: el bringup debe lanzar ``world_tf_publisher`` al menos
    en un factory. Si esto falla, el stack arrancará sin TF
    ``world → base_link`` y el panel/MoveIt fallarán en runtime.
    """
    target_files = [
        WS_ROOT / "src" / "ur5_bringup" / "launch" / "runtime_nodes_factory.py",
    ]
    found = False
    for path in target_files:
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        if 'executable="world_tf_publisher"' in text or \
           "executable='world_tf_publisher'" in text:
            found = True
            break
    assert found, (
        "runtime_nodes_factory.py no parece lanzar world_tf_publisher. "
        "Sin él, el stack arranca sin TF world→base_link y el panel "
        "no podrá calcular geometría."
    )
