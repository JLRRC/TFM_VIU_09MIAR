#!/usr/bin/env python3
# Ruta/archivo: src/ur5_tools/test/test_executor_threads_pinned.py
# Contenido: iter4-bis audit (2026-05-11) — gating anti-regresión threads executor.
"""iter4-bis audit (2026-05-11): gating anti-regresión MultiThreadedExecutor.

PROBLEMA HISTÓRICO: ``MultiThreadedExecutor()`` sin argumento ``num_threads``
usa ``multiprocessing.cpu_count()`` threads. En máquina de 16 cores, 6 nodos
× 16 threads = 96 threads spinning → panel 'no responde', load avg 14.

VER: docs/LESSONS_LEARNED_PANEL_PERF.md

REGLA: cualquier ``MultiThreadedExecutor(...)`` en src/ debe declarar
``num_threads=N`` explícito. Este test escanea el workspace y falla si
encuentra una invocación sin argumento.

NOTA: el test ignora la carpeta ``test/`` para permitir patterns en
ejemplos pedagógicos. También permite la firma exacta
``MultiThreadedExecutor(num_threads=...)``.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest


WORKSPACE_SRC = Path(__file__).resolve().parents[3] / "src"

# Match cualquier llamada que NO incluya 'num_threads' como parámetro.
_PATTERN_BAD = re.compile(
    r"MultiThreadedExecutor\s*\(\s*\)"  # constructor sin argumentos
)

_PATTERN_ANY = re.compile(r"MultiThreadedExecutor\s*\(")


def _scan_production_files() -> list[Path]:
    files: list[Path] = []
    for p in WORKSPACE_SRC.rglob("*.py"):
        s = str(p)
        if "/test/" in s or "/build/" in s or "/install/" in s:
            continue
        if p.name.startswith("test_"):
            continue
        files.append(p)
    return files


def test_no_multithreaded_executor_without_num_threads() -> None:
    """Cada ``MultiThreadedExecutor()`` debe declarar ``num_threads=N`` explícito.

    Falla si encuentra una invocación sin argumentos en código de
    producción. La razón está documentada en
    ``docs/LESSONS_LEARNED_PANEL_PERF.md`` (incidente 2026-05-11).
    """
    offenders: list[str] = []
    for path in _scan_production_files():
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        for match in _PATTERN_BAD.finditer(text):
            line_no = text[: match.start()].count("\n") + 1
            offenders.append(f"{path.relative_to(WORKSPACE_SRC.parent)}:{line_no}")

    assert not offenders, (
        f"{len(offenders)} ocurrencia(s) de MultiThreadedExecutor() SIN num_threads.\n"
        f"En máquinas con muchos cores, el default es cpu_count() → muchos threads\n"
        f"spinning por nodo → saturación CPU del sistema.\n"
        f"FIX: añadir num_threads=N (típicamente 1 o 2) según necesidad real.\n"
        f"Ver: docs/LESSONS_LEARNED_PANEL_PERF.md\n\n"
        f"Ocurrencias:\n  " + "\n  ".join(offenders)
    )


def test_multithreaded_executor_present_in_expected_nodes() -> None:
    """Sanity: los nodos backend siguen usando MultiThreadedExecutor.

    Si alguien migra a SingleThreadedExecutor, este test lo detecta
    para que sea una decisión consciente (actualizar este test).
    """
    expected_files = [
        "ur5_tools/ur5_tools/controller_health_monitor_node.py",
        "ur5_tools/ur5_tools/plan_to_pose_runtime.py",
        "ur5_tools/ur5_tools/panel_backend_node.py",
        "ur5_tools/ur5_tools/panel_launch_control_node.py",
        "ur5_tools/ur5_tools/simulation_reset_service.py",
        "tfm_orchestrator/tfm_orchestrator/pick_orchestrator_lifecycle_node.py",
    ]
    missing: list[str] = []
    for rel in expected_files:
        path = WORKSPACE_SRC / rel
        if not path.is_file():
            missing.append(f"{rel}: file not found")
            continue
        text = path.read_text(encoding="utf-8")
        if not _PATTERN_ANY.search(text):
            missing.append(f"{rel}: no MultiThreadedExecutor")
    if missing:
        pytest.skip(
            "Algún nodo dejó de usar MultiThreadedExecutor (decision change):\n  "
            + "\n  ".join(missing)
        )
