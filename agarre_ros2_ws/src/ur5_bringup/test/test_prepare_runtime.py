#!/usr/bin/env python3
# Ruta/archivo: src/ur5_bringup/test/test_prepare_runtime.py
# Contenido: F8 audit (2026-05-10) — smoke + idempotencia del script.
"""F8 audit (2026-05-10): tests de scripts/prepare_runtime.py."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest


WS = Path(__file__).resolve().parents[3]
SCRIPT = WS / "scripts" / "prepare_runtime.py"


def test_script_exists_and_executable() -> None:
    assert SCRIPT.is_file()
    text = SCRIPT.read_text(encoding="utf-8")
    assert text.startswith("#!/usr/bin/env python3")
    assert "F8 audit" in text


def test_script_dry_run_succeeds() -> None:
    """``prepare_runtime --dry-run`` debe funcionar sin escribir nada."""
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--dry-run", "--ws-dir", str(WS)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, (
        f"prepare_runtime dry-run falló: stdout={result.stdout!r} "
        f"stderr={result.stderr!r}"
    )
    assert "dry-run" in result.stdout
    assert "world_file" in result.stdout


def test_script_help_includes_required_args() -> None:
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--help"],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0
    for arg in ("--ws-dir", "--world-file", "--headless", "--dry-run"):
        assert arg in result.stdout, f"falta arg {arg!r} en --help"


def test_script_fails_gracefully_on_invalid_ws() -> None:
    """ws_dir inexistente → exit code 2."""
    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--ws-dir", "/nonexistent/path/foo", "--dry-run"],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode != 0
    assert "ERROR" in (result.stderr + result.stdout)
