#!/usr/bin/env python3
# Ruta/archivo: src/ur5_qt_panel/test/test_panel_no_runtime_subprocess.py
# Contenido: F4.2 audit (2026-05-10) — guardia anti-subprocess en hot path.
"""F4.2 audit (2026-05-10): el panel UI no debe lanzar subprocess en runtime.

Política arquitectónica:
  * Lanzar procesos (gz sim, ros2 launch, etc.) es responsabilidad de
    ``panel_launchers.py`` (ya allowlisted) y, en F6, del nodo nuevo
    ``panel_launch_control_node``.
  * El resto del panel — UI, mixins, callbacks de pick — NO debe usar
    ``subprocess.*`` ni ``os.system``.
  * Esto previene el antipatrón de "panel lanza nodos directamente"
    durante un ciclo de pick (efectos secundarios no testables).

El test escanea los .py de producción del panel y permite subprocess
sólo en una ALLOWLIST congelada con justificación.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import FrozenSet, Set


WORKSPACE_SRC = Path(__file__).resolve().parents[2]
PANEL_SRC = WORKSPACE_SRC.parent / "src" / "ur5_qt_panel" / "ur5_qt_panel"


# Archivos donde subprocess es legítimo (lanzar el stack, herramientas
# de diagnóstico, etc.). Bajar es bueno; subir requiere justificación
# en el commit message.
SUBPROCESS_ALLOWLIST: FrozenSet[str] = frozenset({
    # F6 audit (2026-05-10): la migración a panel_launch_control_node
    # debería vaciar esta lista. Mientras tanto, baseline congelado.
    "panel_launchers.py",       # Lanzamiento del stack ROS.
    "panel_launch_control.py",  # Control de procesos del stack.
    "panel_process.py",         # Kill / status del stack.
    "panel_gz_startup.py",      # gz sim spawn.
    "panel_workers.py",         # Workers que delegan a subprocess.
    "panel_status_mgmt.py",     # ps / free para status.
    "panel_object_mgmt.py",     # ros2 service call para spawn objetos.
    "panel_controllers.py",     # ros2 control list_controllers.
    "panel_gz_objects.py",      # SetEntityPose vía ros2 service.
    "cameras_tab.py",           # Lanzamiento ros2 run camera_pub.
})


_SUBPROCESS_PATTERN = re.compile(
    r"\bsubprocess\.\w+\(|\bos\.system\(|\bos\.popen\("
)


def _scan_runtime_subprocess() -> Set[str]:
    """Devuelve los nombres de fichero (basename) que usan subprocess."""
    found: Set[str] = set()
    for path in PANEL_SRC.glob("*.py"):
        if path.name.startswith("test_"):
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        if _SUBPROCESS_PATTERN.search(text):
            found.add(path.name)
    return found


def test_no_new_subprocess_outside_allowlist() -> None:
    """Cualquier .py de panel con subprocess debe estar en SUBPROCESS_ALLOWLIST."""
    actual = _scan_runtime_subprocess()
    extras = actual - SUBPROCESS_ALLOWLIST
    assert not extras, (
        f"{len(extras)} archivo(s) NUEVO(s) con subprocess en panel runtime. "
        f"Mover a panel_launch_control_node (F6) o, si es legítimo, añadir "
        f"a SUBPROCESS_ALLOWLIST con justificación:\n  "
        + "\n  ".join(sorted(extras))
    )


def test_allowlist_entries_still_have_subprocess() -> None:
    """Si una entrada deja de usar subprocess, sale de la allowlist."""
    actual = _scan_runtime_subprocess()
    stale = SUBPROCESS_ALLOWLIST - actual
    assert not stale, (
        f"{len(stale)} entrada(s) en SUBPROCESS_ALLOWLIST ya no usan "
        f"subprocess. Migration exitoso — eliminarlas:\n  "
        + "\n  ".join(sorted(stale))
    )


def test_panel_v2_does_not_use_subprocess() -> None:
    """``panel_v2.py`` específicamente — la clase principal — debe estar limpia."""
    panel_v2 = PANEL_SRC / "panel_v2.py"
    if not panel_v2.is_file():
        return  # nothing to assert
    text = panel_v2.read_text(encoding="utf-8")
    matches = _SUBPROCESS_PATTERN.findall(text)
    assert not matches, (
        f"panel_v2.py usa subprocess directo ({len(matches)} match(es)) — "
        "delegar a panel_launchers / panel_process / panel_launch_control_node."
    )


def test_panel_state_machine_does_not_use_subprocess() -> None:
    """La FSM del panel (panel_state_machine.py) debe ser pura — sin I/O externo."""
    sm = PANEL_SRC / "panel_state_machine.py"
    if not sm.is_file():
        return
    text = sm.read_text(encoding="utf-8")
    matches = _SUBPROCESS_PATTERN.findall(text)
    assert not matches, (
        f"panel_state_machine.py usa subprocess ({len(matches)} match(es)) — "
        "la FSM del panel debe ser determinista; mover los efectos a un "
        "ejecutor o a un nodo aparte."
    )


def test_panel_pick_dispatcher_only_calls_action() -> None:
    """``pick_demo_dispatcher.py``: subprocess ok pero no os.system."""
    disp = PANEL_SRC / "pick_demo_dispatcher.py"
    if not disp.is_file():
        return
    text = disp.read_text(encoding="utf-8")
    forbidden = re.findall(r"\bos\.system\(|\bos\.popen\(", text)
    assert not forbidden, (
        "pick_demo_dispatcher: prohibido os.system / os.popen "
        "(usar subprocess.run con args list para shell-injection safety)."
    )
