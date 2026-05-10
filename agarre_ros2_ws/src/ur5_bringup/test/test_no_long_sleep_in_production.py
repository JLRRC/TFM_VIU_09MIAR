#!/usr/bin/env python3
"""F3 audit (2026-05-10): bloquea regresión de sleeps fijos largos.

``time.sleep(N)`` con N > 1.0 segundos en código productivo (no
tests, no helpers de polling) suele ser un mecanismo de
sincronización por tiempo en lugar de por evento. F3 introdujo
``ur5_tools.wait_helpers.wait_until/wait_for_value`` para reemplazar
este patrón.

Este test escanea el árbol productivo y falla si encuentra un sleep
literal > 1.0 s fuera de la allowlist documentada. Sleeps cortos
(<1 s) son aceptables para polling intervals.
"""
from __future__ import annotations

import re
from pathlib import Path

WS = Path(__file__).resolve().parents[3]

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

# Allowlist de archivos donde sleeps largos están justificados:
# - Tests (no aplica el lint).
# - Launch files (idiomáticos en lanzamiento de procesos externos).
# - Demos manuales o mantenimiento (documentar en el archivo).
ALLOWED_FILES = {
    # Helpers de polling internos.
    "src/ur5_tools/ur5_tools/wait_helpers.py",
    # Demo transport: cinemática en bucle, sleeps son intervalos de paso.
    "src/ur5_tools/ur5_tools/attach_demo_transport.py",
    # Reset de Gazebo: sleep estabiliza física post-respawn (documentado
    # en el módulo).
    "src/ur5_tools/ur5_tools/release_objects_service.py",
    # gz topic subprocess timeout — espera del CLI externo.
    "src/ur5_tools/ur5_tools/gz_pose_bridge.py",
    # Mitigación BUG_CONTROLLER_FEEDBACK_HANG: 20s wait deliberado entre
    # reset de JTC y retry MoveIt (race condition controller_manager,
    # documentada en docs/BUG_CONTROLLER_FEEDBACK_HANG.md).
    "src/ur5_tools/ur5_tools/plan_to_pose_server.py",
}

# Permitir cualquier sleep ≤ esta cota (segundos).
SLEEP_THRESHOLD_S = 1.0

SLEEP_RE = re.compile(r"\btime\.sleep\(\s*([0-9]+(?:\.[0-9]+)?)\s*\)")


def _ws_relpath(p: Path) -> str:
    return str(p.relative_to(WS))


def _iter_production_files():
    for path in (WS / "src").rglob("*.py"):
        if not path.is_file():
            continue
        parts = path.parts
        if any(part in EXCLUDED_DIRS for part in parts):
            continue
        # Ignorar tests.
        if "test" in parts:
            continue
        # Ignorar launch files (idiomáticos).
        if "launch" in parts:
            continue
        yield path


def test_no_fixed_sleep_above_threshold_in_production() -> None:
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
            m = SLEEP_RE.search(line)
            if not m:
                continue
            try:
                seconds = float(m.group(1))
            except ValueError:
                continue
            if seconds > SLEEP_THRESHOLD_S:
                offenders.append(
                    f"  {rel}:{lineno} sleep({seconds}) — usa "
                    f"ur5_tools.wait_helpers.wait_until/wait_for_value"
                )
    assert not offenders, (
        f"Sleeps fijos > {SLEEP_THRESHOLD_S}s en código productivo:\n"
        + "\n".join(offenders[:20])
    )
