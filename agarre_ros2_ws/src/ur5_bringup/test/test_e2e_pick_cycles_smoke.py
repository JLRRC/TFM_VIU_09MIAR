#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/test_e2e_pick_cycles_smoke.py
"""T35 (2026-05-08) — Smoke offline para test_e2e_pick_cycles.py.

El test E2E live (test_e2e_pick_cycles.py) requiere stack vivo y se salta
en CI por default (skip si ``PICK_E2E_LIVE != 1``). Este smoke offline
garantiza:

1. El archivo de test live es parseable.
2. Los regex PASS_PATTERN / FAIL_PATTERN compilan.
3. PASS_PATTERN detecta los 3 markers canónicos (legacy + orchestrator).
4. FAIL_PATTERN detecta los markers de fallo (incluyendo bug controller
   feedback hang documentado en BUG_CONTROLLER_FEEDBACK_HANG.md).
5. El skipif está configurado correctamente.

Sin ejecutar nada del stack live. Permite detectar regresiones del
test E2E sin necesidad de simulador.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

WS_ROOT = Path(__file__).resolve().parents[3]
E2E_TEST_PATH = (
    WS_ROOT / "src" / "ur5_bringup" / "test" / "test_e2e_pick_cycles.py"
)


def _load_e2e_module():
    """Carga el módulo del E2E test sin ejecutar los tests."""
    spec = importlib.util.spec_from_file_location(
        "test_e2e_pick_cycles_module", E2E_TEST_PATH
    )
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------
# Estructura
# ---------------------------------------------------------------------------


def test_e2e_test_file_exists():
    assert E2E_TEST_PATH.exists(), f"E2E live test no encontrado en {E2E_TEST_PATH}"


def test_e2e_module_loads_offline():
    """El módulo debe cargarse sin importar ROS ni levantar stack."""
    mod = _load_e2e_module()
    # Sentinels de presencia
    assert hasattr(mod, "PASS_PATTERN")
    assert hasattr(mod, "FAIL_PATTERN")
    assert hasattr(mod, "MOVEIT_READY_PATTERN")
    assert hasattr(mod, "DEFAULT_PANEL_ENV")


# ---------------------------------------------------------------------------
# PASS_PATTERN — debe cubrir 3 paths
# ---------------------------------------------------------------------------


def test_pass_pattern_matches_legacy_panel():
    """Path 1: panel legacy emite SECUENCIA COMPLETADA EXITOSAMENTE."""
    mod = _load_e2e_module()
    assert mod.PASS_PATTERN.search(
        "[PICK][DIRECT] SECUENCIA COMPLETADA EXITOSAMENTE route=basket"
    )


def test_pass_pattern_matches_orchestrator_legacy_marker():
    """Path 2: orchestrator emite [PICK_DEMO][ORCH][DONE] success=true."""
    mod = _load_e2e_module()
    assert mod.PASS_PATTERN.search("[PICK_DEMO][ORCH][DONE] success=true reason=...")


def test_pass_pattern_matches_lifecycle_orchestrator():
    """Path 3: orchestrator lifecycle emite [ORCHESTRATOR_LC] result success=True."""
    mod = _load_e2e_module()
    assert mod.PASS_PATTERN.search("[ORCHESTRATOR_LC] result success=True duration=...")


# ---------------------------------------------------------------------------
# FAIL_PATTERN — debe cubrir todos los modos de fallo conocidos
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "fail_log",
    [
        "[PICK][DIRECT][ABORT] reason=APPROACH_COARSE_NOT_READY",
        "[PICK_OBJ][ABORT] reason=...",
        "Error en pick demo: timeout",
        "Error en pick objeto: collision",
        "[PICK_OBJ][FAIL_CLASS] class=visual_fail",
        "[PICK][DIRECT][...] APPROACH_COARSE_NOT_READY",
        "[PICK_DEMO][ORCH][DONE] success=false reason=timeout",
        "[ORCHESTRATOR_LC] result success=False reason=...",
        "carry_coherence_failed metric=...",
    ],
)
def test_fail_pattern_matches_known_failure_logs(fail_log):
    mod = _load_e2e_module()
    assert mod.FAIL_PATTERN.search(fail_log), (
        f"FAIL_PATTERN no detecta el log de fallo: {fail_log!r}. "
        "Si añadiste un nuevo modo de fallo, actualiza también el regex."
    )


def test_fail_pattern_does_not_match_pass_logs():
    mod = _load_e2e_module()
    pass_logs = [
        "[PICK][DIRECT] SECUENCIA COMPLETADA EXITOSAMENTE",
        "[PICK_DEMO][ORCH][DONE] success=true",
        "[ORCHESTRATOR_LC] result success=True",
        "moveit_state=READY",
    ]
    for log in pass_logs:
        assert not mod.FAIL_PATTERN.search(log), (
            f"FAIL_PATTERN matchea un log PASS: {log!r}"
        )


# ---------------------------------------------------------------------------
# Skip control
# ---------------------------------------------------------------------------


def test_skipif_disables_when_no_live_env(monkeypatch):
    """Sin PICK_E2E_LIVE=1, el test live se salta — guardrail."""
    # No invocamos pytest sobre el módulo (eso requeriría subprocess);
    # verificamos que la marca pytest.mark.skipif está presente y referencia
    # la variable correcta.
    src = E2E_TEST_PATH.read_text(encoding="utf-8")
    assert 'pytest.mark.skipif' in src
    assert 'PICK_E2E_LIVE' in src
    assert '"0"' in src or "'0'" in src, (
        "El skipif debe comparar contra '0' como default seguro."
    )


# ---------------------------------------------------------------------------
# DEFAULT_PANEL_ENV — invariantes
# ---------------------------------------------------------------------------


def test_default_panel_env_offscreen_for_ci():
    """En CI, el panel debe arrancar offscreen para no requerir display."""
    mod = _load_e2e_module()
    assert mod.DEFAULT_PANEL_ENV.get("PANEL_FORCE_OFFSCREEN") == "1"


def test_default_panel_env_auto_release_enabled():
    """Auto release del objeto post-cycle es necesario para 3 cycles consecutivos."""
    mod = _load_e2e_module()
    assert mod.DEFAULT_PANEL_ENV.get("PANEL_AUTO_RELEASE_DROP_OBJECTS") == "1"
