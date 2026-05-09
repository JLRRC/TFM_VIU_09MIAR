#!/usr/bin/env python3
"""Gating: production code must use logger, not print().

CLIs y docstrings están permitidos.
Excluye __pycache__/ y otros artefactos.
"""

from __future__ import annotations

import ast
from pathlib import Path
from typing import List, Tuple

PKG_ROOT = Path(__file__).resolve().parent.parent / "ur5_qt_panel"

CLI_ENTRY_POINTS: set[str] = set()

EXCLUDE_DIRS = {"__pycache__", "build", "install", "log"}


def _iter_prod_py_files() -> List[Path]:
    files: List[Path] = []
    for path in PKG_ROOT.rglob("*.py"):
        parts = set(path.parts)
        if parts & EXCLUDE_DIRS:
            continue
        if path.name in CLI_ENTRY_POINTS:
            continue
        files.append(path)
    return files


def _find_print_calls(tree: ast.AST) -> List[int]:
    lines: List[int] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "print":
            lines.append(node.lineno)
    return lines


def test_no_print_calls_in_production_code() -> None:
    offenders: List[Tuple[str, int]] = []
    for py in _iter_prod_py_files():
        try:
            tree = ast.parse(py.read_text(encoding="utf-8"))
        except SyntaxError:
            continue
        for line in _find_print_calls(tree):
            offenders.append((str(py.relative_to(PKG_ROOT.parent)), line))
    assert not offenders, (
        "print() detected in production code (use logger):\n"
        + "\n".join(f"  {f}:{l}" for f, l in offenders)
    )
