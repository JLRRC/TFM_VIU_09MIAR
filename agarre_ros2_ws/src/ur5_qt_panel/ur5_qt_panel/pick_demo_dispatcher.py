#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_demo_dispatcher.py
# Contenido: F6.4 final — dispatcher que elige entre run_pick_demo legacy y orchestrator client.
"""Dispatcher del flujo Pick Demo del panel.

F12 (2026-05-01) — desde esta fase el **orchestrator es el camino
canónico** del botón "Agarre Objeto (Directo)". El legacy
``run_pick_demo`` (10.7k LOC closure embebida) queda en deprecación
y emite ``DeprecationWarning`` al ser llamado.

Reglas de selección (default = orchestrator):

* Si ``USE_LEGACY_PICK_DEMO=1|true|yes|on``: fuerza el legacy
  ``run_pick_demo(panel)`` (rollback rápido si hay regresión).
* Si ``PANEL_PICK_DEMO_USE_ORCHESTRATOR=0|false|no|off``: fuerza el
  legacy explícitamente.
* En cualquier otro caso (incluido sin env var): envía un goal al
  action server ``/pick_place`` (orchestrator) usando
  ``PickPlaceClient`` y propaga feedback al panel.

El dispatch se hace **best-effort**: si el orchestrator no responde
o rclpy no está disponible, hace fallback al legacy con un log claro.
Esto preserva la demo operativa al 100% mientras drena gradualmente
el legacy hacia el orchestrator (F12).

La lógica del flag está en ``pick_place_client_logic.should_use_orchestrator``
y la wrapping del ActionClient en ``pick_place_client.PickPlaceClient``,
ambos extensamente testeados offline.
"""

from __future__ import annotations

import os
from typing import Any, Callable, Optional


def _legacy_dispatch(panel: Any) -> None:
    """Importa y llama run_pick_demo perezosamente (evita ciclo)."""
    from .panel_pick_demo import run_pick_demo
    run_pick_demo(panel)


def _resolve_panel_node(panel: Any) -> Optional[Any]:
    """Localiza un rclpy.Node usable desde el panel.

    Sondea atributos comunes: ``ros_worker._node``, ``_moveit_node``,
    ``_node``. Devuelve None si no encuentra ninguno.
    """
    candidates = [
        getattr(getattr(panel, "ros_worker", None), "_node", None),
        getattr(panel, "_moveit_node", None),
        getattr(panel, "_node", None),
    ]
    for n in candidates:
        if n is not None and hasattr(n, "get_logger"):
            return n
    return None


def _drop_xyz_from_panel(panel: Any) -> tuple:
    """Resuelve el drop_xyz_world por defecto desde el panel.

    Usa ``panel._basket_drop_world`` si existe; si no, defaults
    razonables.
    """
    cand = getattr(panel, "_basket_drop_world", None)
    if cand is not None:
        try:
            return (float(cand[0]), float(cand[1]), float(cand[2]))
        except Exception:
            pass
    return (0.5, 0.0, 0.05)


def dispatch_pick_demo(
    panel: Any,
    *,
    env_value: Optional[str] = None,
    legacy_env_value: Optional[str] = None,
    legacy: Callable[[Any], None] = _legacy_dispatch,
) -> str:
    """Decide y ejecuta el modo Pick Demo apropiado.

    Devuelve un identificador del modo finalmente usado:
      - ``"legacy"``: se llamó al ``run_pick_demo`` legacy.
      - ``"orchestrator"``: se envió goal a ``/pick_place``.
      - ``"orchestrator_fallback_no_rclpy"``: flag activo pero sin
        rclpy disponible → fallback a legacy.
      - ``"orchestrator_fallback_no_node"``: flag activo pero panel
        sin Node ROS reconocible → fallback a legacy.
      - ``"orchestrator_fallback_no_object"``: flag activo pero no
        hay objeto seleccionado → fallback a legacy.

    Parameters:
        panel: instancia ControlPanelV2.
        env_value: override del valor del env var (testing). Si None,
            lee ``PANEL_PICK_DEMO_USE_ORCHESTRATOR`` del entorno real.
        legacy: callable opcional para inyección en tests.
    """
    from .pick_place_client_logic import should_use_orchestrator

    if env_value is None:
        env_value = os.environ.get("PANEL_PICK_DEMO_USE_ORCHESTRATOR")
    if legacy_env_value is None:
        legacy_env_value = os.environ.get("USE_LEGACY_PICK_DEMO")
    if not should_use_orchestrator(env_value, legacy_env_value=legacy_env_value):
        legacy(panel)
        return "legacy"

    # Flag activo — intentar la ruta orchestrator.
    from .pick_place_client import rclpy_available
    if not rclpy_available():
        try:
            panel._emit_log(
                "[PICK_DEMO][DISPATCH] flag PANEL_PICK_DEMO_USE_ORCHESTRATOR "
                "activo pero rclpy no disponible — fallback al legacy"
            )
        except Exception:
            pass
        legacy(panel)
        return "orchestrator_fallback_no_rclpy"

    node = _resolve_panel_node(panel)
    if node is None:
        try:
            panel._emit_log(
                "[PICK_DEMO][DISPATCH] flag activo pero el panel no expone "
                "Node ROS (ros_worker._node/_moveit_node/_node) — fallback"
            )
        except Exception:
            pass
        legacy(panel)
        return "orchestrator_fallback_no_node"

    object_name = str(getattr(panel, "_selected_object", "") or "").strip()
    if not object_name:
        try:
            panel._emit_log(
                "[PICK_DEMO][DISPATCH] flag activo pero no hay objeto "
                "seleccionado — fallback al legacy"
            )
        except Exception:
            pass
        legacy(panel)
        return "orchestrator_fallback_no_object"

    drop_xyz = _drop_xyz_from_panel(panel)

    from .pick_place_client import PickPlaceClient
    client = PickPlaceClient(node)

    def _on_feedback(ev: dict) -> None:
        try:
            panel._emit_log(
                f"[PICK_DEMO][ORCH][FB] phase={ev.get('current_phase','?')} "
                f"progress={ev.get('progress',0.0):.2f} "
                f"detail={ev.get('detail','')}"
            )
        except Exception:
            pass

    def _on_done(ev: dict) -> None:
        try:
            panel._emit_log(
                f"[PICK_DEMO][ORCH][DONE] success={ev.get('success', False)} "
                f"reason={ev.get('reason','')} "
                f"duration_sec={ev.get('duration_sec', 0.0):.2f}"
            )
        except Exception:
            pass

    def _on_rejected() -> None:
        try:
            panel._emit_log("[PICK_DEMO][ORCH] goal rejected by orchestrator")
        except Exception:
            pass

    if not client.wait_for_server(timeout_sec=2.0):
        try:
            panel._emit_log(
                "[PICK_DEMO][DISPATCH] /pick_place server no responde — "
                "fallback al legacy"
            )
        except Exception:
            pass
        legacy(panel)
        return "orchestrator_fallback_no_server"

    client.send_goal(
        object_name,
        drop_xyz,
        on_feedback=_on_feedback,
        on_done=_on_done,
        on_rejected=_on_rejected,
    )
    return "orchestrator"
