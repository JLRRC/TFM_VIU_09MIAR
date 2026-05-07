#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_mypy_strict_baseline.py
# Contenido: F7 — guardrail: mypy --strict pasa en módulos puros baseline.
"""F7 — Mypy strict baseline guardrail.

Verifica que ``mypy --strict`` pasa sin errores en los módulos puros
listados en ``MYPY_STRICT_CLEAN_MODULES``. Cuando un módulo se "limpia"
(se le añaden las anotaciones faltantes y pasa mypy strict), se añade
a la lista. Una regresión (alguien añade un Any o un None) hace fallar
el test.

Skip si mypy no está instalado en el entorno (CI sólo, no offline-only).

Roadmap (audit-v4 F7-step2): ampliar la lista a:
  - phase_dispatch.py
  - initial_snapshot.py
  - home_initial.py
  - pose_consistency.py (ya casi)
  - phase_progress.py
  - pick_fsm.py
  - cartesian_segments.py
  - gripper_monitor.py
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

WS_ROOT = Path(__file__).resolve().parents[3]


# Módulos que han pasado mypy strict en F7 audit-v4.
MYPY_STRICT_CLEAN_MODULES: list[str] = [
    "src/tfm_orchestrator/tfm_orchestrator/pick_gates.py",
    "src/tfm_orchestrator/tfm_orchestrator/retry.py",
]


def _find_mypy() -> str | None:
    """Locate mypy: PATH first, then known venv paths."""
    candidate = shutil.which("mypy")
    if candidate:
        return candidate
    for path in ("/tmp/ruff-venv/bin/mypy", "/usr/local/bin/mypy"):
        if Path(path).exists():
            return path
    return None


def test_f7_mypy_strict_baseline() -> None:
    """F7 — los módulos baseline pasan ``mypy --strict``."""
    mypy = _find_mypy()
    if not mypy:
        pytest.skip(
            "mypy no instalado en el entorno (instala via pip o venv para "
            "ejecutar este guardrail; el test no es offline-only)"
        )
    config = WS_ROOT / "mypy.ini"
    if not config.exists():
        pytest.skip(f"mypy.ini no encontrado en {config}")
    cmd = [mypy, "--config-file", str(config), "--strict"] + [
        str(WS_ROOT / m) for m in MYPY_STRICT_CLEAN_MODULES
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, cwd=str(WS_ROOT))
    msg = (
        f"\n--- STDOUT ---\n{result.stdout}"
        f"\n--- STDERR ---\n{result.stderr}"
        f"\n--- COMMAND ---\n{' '.join(cmd)}"
    )
    assert result.returncode == 0, (
        f"mypy strict falló en módulos baseline:{msg}"
    )
