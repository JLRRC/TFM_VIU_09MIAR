#!/usr/bin/env python3
"""Gating: production code must use rclpy/logging, not print().

CLIs (entry_points sin nodo ROS) y docstrings están permitidos.
Excluye install/ y build/ (artefactos colcon no trackeados).
"""

from __future__ import annotations

import ast
import os
from pathlib import Path
from typing import List, Tuple

PKG_ROOT = Path(__file__).resolve().parent.parent / "ur5_tools"

CLI_ENTRY_POINTS = {
    "cli_cycle_timing.py",
    "generate_latency_table.py",
}

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
    """Return line numbers of real print() calls (skips strings/docstrings)."""
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
        "print() detected in production code (use rclpy logger or logging stdlib):\n"
        + "\n".join(f"  {f}:{l}" for f, l in offenders)
    )
