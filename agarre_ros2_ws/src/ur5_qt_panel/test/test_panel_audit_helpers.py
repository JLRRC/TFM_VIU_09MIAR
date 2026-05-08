#!/usr/bin/env python3
"""V1.1 audit-v4 (2026-05-08): tests offline para panel_audit_helpers."""
from __future__ import annotations

import json
import tempfile
from pathlib import Path

import pytest

from ur5_qt_panel.panel_audit_helpers import (
    audit_append_line,
    audit_root_for_ws,
    audit_write_json,
    sha256_file,
)


# ---- sha256_file -----------------------------------------------------------


def test_sha256_file_basic(tmp_path: Path) -> None:
    f = tmp_path / "x.bin"
    f.write_bytes(b"hello world")
    h = sha256_file(str(f))
    # Expected: SHA-256 de "hello world".
    assert h == "b94d27b9934d3e08a52e52d7da7dabfac484efe37a5380ee9088f7ace2efcde9"


def test_sha256_file_empty(tmp_path: Path) -> None:
    f = tmp_path / "empty.bin"
    f.write_bytes(b"")
    h = sha256_file(str(f))
    assert h == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"


def test_sha256_file_nonexistent_returns_empty() -> None:
    assert sha256_file("/no/such/path/abc.xyz") == ""


def test_sha256_file_directory_returns_empty(tmp_path: Path) -> None:
    """Pasar un directorio devuelve "" (best-effort silent)."""
    assert sha256_file(str(tmp_path)) == ""


def test_sha256_file_large(tmp_path: Path) -> None:
    """Test que el chunked read funciona con > 1 MB."""
    f = tmp_path / "big.bin"
    f.write_bytes(b"x" * (3 * 1024 * 1024 + 17))  # ~3 MB + tail
    h = sha256_file(str(f))
    assert len(h) == 64  # hex digest


# ---- audit_root_for_ws -----------------------------------------------------


def test_audit_root_for_ws_basic() -> None:
    ws = "/home/lab/TFM/agarre_ros2_ws"
    root = audit_root_for_ws(ws)
    assert str(root) == "/home/lab/TFM/auditoria/panel_audit"


def test_audit_root_for_ws_relative() -> None:
    ws = "./agarre_ros2_ws"
    root = audit_root_for_ws(ws)
    assert root.name == "panel_audit"
    assert root.parent.name == "auditoria"


# ---- audit_append_line -----------------------------------------------------


def test_audit_append_line_creates_dirs(tmp_path: Path) -> None:
    out = tmp_path / "deep" / "nested" / "log.txt"
    audit_append_line(out, "first line")
    audit_append_line(out, "second line")
    contents = out.read_text(encoding="utf-8")
    assert "first line" in contents
    assert "second line" in contents
    # Nueva línea separa.
    assert contents.count("\n") >= 2


def test_audit_append_line_timestamper(tmp_path: Path) -> None:
    out = tmp_path / "ts.log"
    audit_append_line(out, "msg", timestamper=lambda m: f"[T] {m}")
    assert out.read_text(encoding="utf-8") == "[T] msg\n"


def test_audit_append_line_silent_on_dir(tmp_path: Path) -> None:
    """Apuntar a un directorio existente NO debe lanzar (silent)."""
    audit_append_line(tmp_path, "msg")  # tmp_path is a dir → would fail open
    # Nada que asertar; el test es que no lanza.


# ---- audit_write_json ------------------------------------------------------


def test_audit_write_json_basic(tmp_path: Path) -> None:
    out = tmp_path / "data.json"
    payload = {"a": 1, "b": ["x", "y"], "c": {"nested": True}}
    audit_write_json(out, payload)
    parsed = json.loads(out.read_text(encoding="utf-8"))
    assert parsed == payload


def test_audit_write_json_creates_dirs(tmp_path: Path) -> None:
    out = tmp_path / "deep" / "j.json"
    audit_write_json(out, {"x": 1})
    assert out.is_file()


def test_audit_write_json_pretty_format(tmp_path: Path) -> None:
    """Indent=2 produce JSON multi-línea."""
    out = tmp_path / "p.json"
    audit_write_json(out, {"a": 1, "b": 2})
    txt = out.read_text(encoding="utf-8")
    assert "\n" in txt  # multiline


def test_audit_write_json_silent_on_unserializable(tmp_path: Path) -> None:
    """Payload no serializable → silent (no raise)."""
    out = tmp_path / "bad.json"
    audit_write_json(out, {"sock": object()})  # object() no es JSON
    # Test pasa si no lanzó.
