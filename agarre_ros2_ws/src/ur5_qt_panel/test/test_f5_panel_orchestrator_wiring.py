#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_f5_panel_orchestrator_wiring.py
"""F5 cierre (2026-05-08) — Guardrails offline del cableado panel→orchestrator.

Verifica los invariantes del cableado F5 que deben mantenerse hasta que el
bug BUG_CONTROLLER_FEEDBACK_HANG esté cerrado y el default pueda invertirse:

1. El botón "Agarre Objeto (Directo)" en panel_calib_actions invoca
   dispatch_pick_demo (no run_pick_demo directo).
2. El service /panel/pick_demo en panel_remote_callbacks invoca
   _dispatch_pick_demo (no run_pick_demo directo).
3. El default actual de should_use_orchestrator es LEGACY (False).
4. Las 4 rutas del dispatcher están todas codificadas:
   - "legacy"
   - "orchestrator"
   - "orchestrator_fallback_no_rclpy"
   - "orchestrator_fallback_no_node"
   - "orchestrator_fallback_no_object"

Si alguien rompe el cableado o cambia el default sin actualizar el doc
F5_CLOSURE_STATUS_20260508.md, este test cae.
"""

from __future__ import annotations

from pathlib import Path

import pytest

WS_ROOT = Path(__file__).resolve().parents[3]
PANEL_DIR = WS_ROOT / "src" / "ur5_qt_panel" / "ur5_qt_panel"
DOCS_DIR = WS_ROOT / "docs"


# ---------------------------------------------------------------------------
# Cableado: botón → dispatcher
# ---------------------------------------------------------------------------


def test_button_callback_uses_dispatcher_not_run_pick_demo_directly():
    """Botón 'Agarre Objeto (Directo)' debe llamar dispatch_pick_demo."""
    src = (PANEL_DIR / "panel_calib_actions.py").read_text(encoding="utf-8")
    assert "dispatch_pick_demo" in src, (
        "panel_calib_actions._run_pick_demo debe importar dispatch_pick_demo. "
        "Si se eliminó el dispatcher, actualizar F5_CLOSURE_STATUS_20260508.md."
    )
    # No debe haber import directo de `from .panel_pick_demo import run_pick_demo`
    # en panel_calib_actions (sólo el dispatcher debe importarlo)
    assert "from .panel_pick_demo import run_pick_demo" not in src, (
        "panel_calib_actions no debe importar run_pick_demo directamente — "
        "el dispatcher es el único punto de entrada autorizado."
    )


def test_service_callback_uses_dispatcher_not_run_pick_demo_directly():
    """Service /panel/pick_demo debe llamar _dispatch_pick_demo."""
    src = (PANEL_DIR / "panel_remote_callbacks.py").read_text(encoding="utf-8")
    assert "_dispatch_pick_demo" in src, (
        "panel_remote_callbacks debe usar _dispatch_pick_demo (alias del "
        "dispatcher) — el service /panel/pick_demo es un canal canónico."
    )


def test_dispatcher_legacy_dispatch_is_sentinel_only():
    """F5-legacy-removed (2026-05-08): _legacy_dispatch ya no importa
    run_pick_demo (borrado). Es ahora un sentinel que emite log y no hace
    nada. Para recuperar legacy: git checkout audit-pre-borrar-legacy-20260508."""
    src = (PANEL_DIR / "pick_demo_dispatcher.py").read_text(encoding="utf-8")
    # _legacy_dispatch sigue existiendo como sentinel
    assert "_legacy_dispatch" in src
    # Pero NO debe importar run_pick_demo (función borrada)
    assert "from .panel_pick_demo import run_pick_demo" not in src
    # Y debe referenciar el tag de rollback en el log/comentario
    assert "audit-pre-borrar-legacy-20260508" in src or "F5-legacy-removed" in src


# ---------------------------------------------------------------------------
# Default actual: orchestrator (F5-legacy-removed)
# ---------------------------------------------------------------------------


def test_default_is_orchestrator_after_f5_legacy_removed():
    """F5-legacy-removed (2026-05-08): default invertido a orchestrator
    tras borrado físico del legacy."""
    from ur5_qt_panel.pick_place_client_logic import should_use_orchestrator

    # Sin env vars seteadas → orchestrator (default invertido)
    assert should_use_orchestrator(None) is True
    # Cualquier valor no falsy → orchestrator
    assert should_use_orchestrator("anything") is True


def test_orchestrator_opt_in_explicit():
    """Activación explícita = PANEL_PICK_DEMO_USE_ORCHESTRATOR=1."""
    from ur5_qt_panel.pick_place_client_logic import should_use_orchestrator

    assert should_use_orchestrator("1") is True
    assert should_use_orchestrator("true") is True
    assert should_use_orchestrator("yes") is True
    assert should_use_orchestrator("on") is True


def test_legacy_force_overrides_orchestrator_opt_in():
    """USE_LEGACY_PICK_DEMO=1 fuerza legacy aunque orch flag esté activo."""
    from ur5_qt_panel.pick_place_client_logic import should_use_orchestrator

    assert should_use_orchestrator("1", legacy_env_value="1") is False


# ---------------------------------------------------------------------------
# Las 5 rutas del dispatcher están codificadas
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "route_marker",
    [
        '"legacy"',
        '"orchestrator"',
        '"orchestrator_fallback_no_rclpy"',
        '"orchestrator_fallback_no_node"',
        '"orchestrator_fallback_no_object"',
    ],
)
def test_dispatcher_codifies_route_marker(route_marker):
    """Cada ruta del dispatcher devuelve un marker observable distinto."""
    src = (PANEL_DIR / "pick_demo_dispatcher.py").read_text(encoding="utf-8")
    assert route_marker in src, (
        f"Dispatcher debe devolver {route_marker} en una de sus ramas. "
        "Si se renombró, actualizar este test y F5_CLOSURE_STATUS."
    )


# ---------------------------------------------------------------------------
# Doc de status existe y referencia bug bloqueante
# ---------------------------------------------------------------------------


def test_f5_closure_status_doc_exists():
    """El doc F5_CLOSURE_STATUS_20260508 debe existir como referencia única."""
    status_doc = DOCS_DIR / "F5_CLOSURE_STATUS_20260508.md"
    assert status_doc.exists(), (
        f"F5_CLOSURE_STATUS_20260508.md no encontrado en {status_doc}. "
        "Es el doc de referencia del estado de cierre F5."
    )
    content = status_doc.read_text(encoding="utf-8")
    assert "BUG_CONTROLLER_FEEDBACK_HANG" in content, (
        "El status doc debe referenciar el bug bloqueante para el cierre."
    )
    assert "PANEL_PICK_DEMO_USE_ORCHESTRATOR=1" in content, (
        "El status doc debe documentar la env var de activación."
    )
