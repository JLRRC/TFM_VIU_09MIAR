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
    # F7-step1.5 (2026-05-08): añadidos asserts explícitos para narrow
    # tcp_tf_age_sec/joint_state_age_sec en success path. Pasa strict.
    "src/tfm_orchestrator/tfm_orchestrator/pose_consistency.py",
    # F7-step1.6 (2026-05-08): bare `tuple` → `Tuple[float, ...]` en 7 anotaciones
    # del PickContext + helpers. is_no_hint con bool() explícito + len-guard.
    "src/tfm_orchestrator/tfm_orchestrator/pick_fsm.py",
    # F7-step1.7 (2026-05-08): home_initial.py limpio tras path_tol_rad
    # opcional + Optional import.
    "src/tfm_orchestrator/tfm_orchestrator/home_initial.py",
    # F7-step1.8 (2026-05-08): 4 módulos puros adicionales pasan strict
    # sin necesidad de cambios.
    "src/tfm_orchestrator/tfm_orchestrator/phase_progress.py",
    "src/tfm_orchestrator/tfm_orchestrator/cartesian_segments.py",
    "src/tfm_orchestrator/tfm_orchestrator/gripper_monitor.py",
    "src/tfm_orchestrator/tfm_orchestrator/lifecycle_helpers.py",
    # F7-step1.9 (2026-05-08): initial_snapshot.py — explicit Tuple6 cast en
    # extract_joint_positions; type:ignore innecesario removido.
    "src/tfm_orchestrator/tfm_orchestrator/initial_snapshot.py",
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
