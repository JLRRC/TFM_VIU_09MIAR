#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_evidence_logger_offline.py
# Contenido: F4 — tests offline de helpers puros del evidence_logger.
"""Tests offline de los helpers puros de ``ur5_tools.evidence_logger``.

NO instancian la clase ``EvidenceLogger`` (que requiere rclpy.Node y
QoS de ROS 2). Sólo verifican:

* ``_now_iso()`` produce timestamps ISO 8601 razonables.
* ``_safe_unique_dir(root)`` crea directorios distintos en colisión.

Si rclpy no está disponible, se importan los helpers desde el módulo
con un try/except — si falla se skipea con razón clara.
"""

from __future__ import annotations

import re
from datetime import datetime
from pathlib import Path

import pytest


@pytest.fixture(scope="module")
def helpers():
    """Importa los helpers desde evidence_helpers (sin rclpy)."""
    from ur5_tools.evidence_helpers import now_iso, safe_unique_dir
    return now_iso, safe_unique_dir


# ---------------------------------------------------------------------------
# _now_iso
# ---------------------------------------------------------------------------


def test_now_iso_returns_string(helpers):
    _now_iso, _ = helpers
    s = _now_iso()
    assert isinstance(s, str)
    assert s.endswith("Z"), f"timestamp debe acabar en Z (UTC), got {s!r}"


def test_now_iso_parseable(helpers):
    _now_iso, _ = helpers
    s = _now_iso()
    # Formato esperado: 2026-04-30T22:01:02.345678Z
    assert re.match(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d+Z$", s), (
        f"timestamp no cumple formato esperado: {s!r}"
    )
    # Parseable como datetime ISO 8601 (sustituyendo Z por +00:00)
    dt = datetime.fromisoformat(s.replace("Z", "+00:00"))
    assert dt.year >= 2025  # sanity check del reloj


def test_now_iso_monotonic_or_equal(helpers):
    _now_iso, _ = helpers
    a = _now_iso()
    b = _now_iso()
    assert a <= b, f"timestamps consecutivos deben ser monotónicos: {a} > {b}"


# ---------------------------------------------------------------------------
# _safe_unique_dir
# ---------------------------------------------------------------------------


def test_safe_unique_dir_creates_directory(helpers, tmp_path):
    _, _safe_unique_dir = helpers
    out = _safe_unique_dir(tmp_path)
    assert out.exists() and out.is_dir()
    assert out.parent == tmp_path


def test_safe_unique_dir_handles_collision(helpers, tmp_path, monkeypatch):
    _, _safe_unique_dir = helpers
    # Forzar colisión: pre-crear un dir con el timestamp actual.
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    pre = tmp_path / stamp
    pre.mkdir()
    out = _safe_unique_dir(tmp_path)
    assert out != pre, "no debe sobrescribir un directorio existente"
    assert out.exists() and out.is_dir()
    assert out.name.startswith(stamp), (
        f"el sufijo debe basarse en el stamp original: got {out.name!r}"
    )


def test_safe_unique_dir_returns_path(helpers, tmp_path):
    _, _safe_unique_dir = helpers
    out = _safe_unique_dir(tmp_path)
    assert isinstance(out, Path)
