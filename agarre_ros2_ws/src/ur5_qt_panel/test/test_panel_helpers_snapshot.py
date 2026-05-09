#!/usr/bin/env python3
"""Audit-v4 (2026-05-08): snapshot del contrato de panel_helpers.

Congela LOC + key public functions para que el split iter2 (deferred
v1.1) no rompa los consumidores.
"""
from __future__ import annotations

from pathlib import Path
from typing import Set

PKG = Path(__file__).resolve().parent.parent / "ur5_qt_panel"
MODULE_FILE = PKG / "panel_helpers.py"


EXPECTED_PRIVATE_FUNCTIONS: Set[str] = {
    "_log_exception",
    "_emit_log",
    "_metric_mark",
    "_audit_root",
    "_audit_append",
    "_audit_write_json",
    "_sha256_file",
    "_should_emit_log",
    "_set_motion_lock",
    "_set_btn_state",
    "_set_launching_style",
    "_clear_launching_if_timeout",
    "_controller_drop_grace_active",
    "_require_ready_basic",
    "_basic_ready_status",
    "_pick_demo_remote_ready_status",
}


def test_file_exists() -> None:
    assert MODULE_FILE.is_file()


def test_loc_does_not_grow() -> None:
    BASELINE = 1441
    n = sum(1 for _ in MODULE_FILE.read_text(encoding="utf-8").splitlines())
    assert n <= BASELINE, f"panel_helpers.py grew {BASELINE}→{n}"


def test_expected_private_functions_present() -> None:
    text = MODULE_FILE.read_text(encoding="utf-8")
    import re
    names: Set[str] = set()
    for m in re.finditer(r"^def\s+(\w+)\s*\(", text, flags=re.MULTILINE):
        names.add(m.group(1))
    missing = EXPECTED_PRIVATE_FUNCTIONS - names
    assert not missing, f"missing helpers: {missing}"
