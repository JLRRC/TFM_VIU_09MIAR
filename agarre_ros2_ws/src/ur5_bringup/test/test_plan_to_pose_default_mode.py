#!/usr/bin/env python3
"""F7 audit (2026-05-10): default mode de plan_to_pose_server = MOVEIT_DIRECT.

STUB y REAL_BRIDGE están deprecados desde la auditoría 2026-05-10.
MOVEIT_DIRECT es el único modo soportado para producción. Este test
garantiza que ningún cambio futuro al launch reintroduce un default
legacy por accidente.
"""
from __future__ import annotations

import re
from pathlib import Path

WS = Path(__file__).resolve().parents[3]
RUNTIME_FACTORY = WS / "src" / "ur5_bringup" / "launch" / "runtime_nodes_factory.py"


def test_runtime_factory_defaults_to_moveit_direct() -> None:
    txt = RUNTIME_FACTORY.read_text(encoding="utf-8")
    # Buscar el parámetro `mode` del nodo plan_to_pose_server.
    pat = re.compile(r'\{\s*"mode"\s*:\s*"([^"]+)"\s*\}')
    matches = pat.findall(txt)
    assert matches, "No se encontró ningún parámetro 'mode' en runtime_nodes_factory.py"
    assert "MOVEIT_DIRECT" in matches, (
        f"plan_to_pose_server no tiene mode=MOVEIT_DIRECT por default. "
        f"Encontrado: {matches}"
    )
    # Si hay otros valores asignados en el mismo factory, deben ser
    # configurables (LaunchConfiguration), no literales.
    legacy_in_literals = [m for m in matches if m in ("STUB", "REAL_BRIDGE")]
    assert not legacy_in_literals, (
        f"plan_to_pose_server tiene literales legacy {legacy_in_literals} "
        "en runtime_nodes_factory.py — STUB/REAL_BRIDGE están deprecados (F7)."
    )


def test_plan_to_pose_server_emits_deprecation_warning() -> None:
    server = (
        WS / "src" / "ur5_tools" / "ur5_tools" / "plan_to_pose_server.py"
    ).read_text(encoding="utf-8")
    assert "[PLAN_TO_POSE][DEPRECATED]" in server, (
        "plan_to_pose_server.py debe emitir un warning DEPRECATED cuando "
        "se configura mode=STUB o mode=REAL_BRIDGE"
    )
