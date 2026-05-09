#!/usr/bin/env python3
"""Audit-v4 V1.1 (2026-05-08): puros helpers de audit/log para panel_helpers.

Extrae funciones puras del namespace ``panel_helpers.py`` (1.441 LOC) que
no dependen del objeto ``panel``. Primer paso del split iter2 documented
en ``V1_DEFERRED_TO_V1_1.md``.

Funciones:
- ``sha256_file(path)`` — hash SHA-256 de un fichero. Sin panel.
- ``audit_root_for_ws(ws_dir)`` — calcula la ruta canónica de auditoría
  desde un WS_DIR. Sin panel.
- ``audit_append_line(out_path, msg)`` — escribe línea timestamped en file.
- ``audit_write_json(out_path, payload)`` — escribe JSON pretty-printed.

panel_helpers.py mantiene los wrappers ``_audit_*(panel, ...)`` que ahora
delegan a estos helpers + leen ``panel._audit_root()`` para conseguir
la base path. Comportamiento idéntico.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any, Dict


def sha256_file(path: str) -> str:
    """SHA-256 de un fichero. Devuelve "" si falla (path no existe, etc.)."""
    h = hashlib.sha256()
    try:
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b""):
                h.update(chunk)
    except OSError:
        return ""
    return h.hexdigest()


def audit_root_for_ws(ws_dir: str) -> Path:
    """Devuelve la ruta canónica auditoria/panel_audit/ desde un WS_DIR.

    El layout del proyecto es:
        <repo_root>/agarre_ros2_ws/...           ← WS_DIR
        <repo_root>/auditoria/panel_audit/        ← target

    Por tanto subimos un nivel desde WS_DIR.
    """
    return Path(ws_dir).parent / "auditoria" / "panel_audit"


def _ensure_dir(path: str) -> None:
    """Crea directorio si no existe (mkdir -p semantic)."""
    try:
        os.makedirs(path, exist_ok=True)
    except OSError:
        pass


def audit_append_line(
    out_path: Path,
    msg: str,
    *,
    timestamper: Any = None,
) -> None:
    """Append timestamped ``msg`` a ``out_path``.

    Crea directorios padre si no existen. Silent on errors (best-effort).

    Args:
        out_path: ruta absoluta del fichero.
        msg: mensaje (será timestamped).
        timestamper: función opcional que recibe ``msg`` y devuelve la
            línea final (default: identidad).
    """
    try:
        _ensure_dir(str(Path(out_path).parent))
        line = timestamper(msg) if timestamper is not None else msg
        with Path(out_path).open("a", encoding="utf-8") as f:
            f.write(f"{line}\n")
    except OSError:
        pass


def audit_write_json(
    out_path: Path,
    payload: Dict[str, object],
) -> None:
    """Escribe ``payload`` como JSON pretty-printed.

    Crea directorios padre si no existen. Silent on errors.
    """
    try:
        _ensure_dir(str(Path(out_path).parent))
        Path(out_path).write_text(json.dumps(payload, indent=2), encoding="utf-8")
    except (OSError, TypeError):
        pass
