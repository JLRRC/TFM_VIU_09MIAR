#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_system_health_helpers.py
"""T33 (2026-05-08) — Tests offline para system_health_helpers.

Cubre:
1. count_processes_with_name (función pura).
2. zombie_move_group_check (veredicto estructurado).
3. Scripts canónicos del stack incluyen pkill de move_group (guardrail F1.16).
"""

from __future__ import annotations

from pathlib import Path

import pytest

from ur5_tools.system_health_helpers import (
    ZombieCheckResult,
    count_processes_with_name,
    list_known_move_group_killers,
    zombie_move_group_check,
)


# ---------------------------------------------------------------------------
# count_processes_with_name
# ---------------------------------------------------------------------------


def test_count_zero_when_no_match():
    assert count_processes_with_name("move_group", []) == 0
    assert count_processes_with_name("move_group", ["bash", "vim", "tmux"]) == 0


def test_count_one_when_single_match():
    cmdlines = ["bash", "/opt/ros/jazzy/lib/move_group/move_group --params"]
    assert count_processes_with_name("move_group", cmdlines) == 1


def test_count_multiple_zombies():
    """Caso real F1.16: 3 zombies de move_group post-cycles."""
    cmdlines = [
        "/opt/ros/jazzy/lib/move_group/move_group --params /tmp/move_group_1.yaml",
        "/opt/ros/jazzy/lib/move_group/move_group --params /tmp/move_group_2.yaml",
        "/opt/ros/jazzy/lib/move_group/move_group --params /tmp/move_group_3.yaml",
        "bash",
    ]
    assert count_processes_with_name("move_group", cmdlines) == 3


def test_count_substring_match():
    """Si el nombre aparece dentro de otro cmdline, cuenta — esto es por diseño."""
    cmdlines = ["my_test_move_group_helper", "/opt/move_group/bin"]
    assert count_processes_with_name("move_group", cmdlines) == 2


def test_count_empty_name_returns_zero():
    assert count_processes_with_name("", ["any", "cmdline"]) == 0
    assert count_processes_with_name("   ", ["any"]) == 0


# ---------------------------------------------------------------------------
# zombie_move_group_check
# ---------------------------------------------------------------------------


def test_zombie_check_healthy_when_no_processes():
    result = zombie_move_group_check([])
    assert result.process_name == "move_group"
    assert result.count == 0
    assert result.healthy is True


def test_zombie_check_unhealthy_with_zombies():
    cmdlines = [
        "/opt/ros/jazzy/lib/move_group/move_group --params 1",
        "/opt/ros/jazzy/lib/move_group/move_group --params 2",
    ]
    result = zombie_move_group_check(cmdlines)
    assert result.count == 2
    assert result.healthy is False


def test_zombie_check_custom_name():
    cmdlines = ["panel_v2_main"]
    result = zombie_move_group_check(cmdlines, name="panel_v2")
    assert result.process_name == "panel_v2"
    assert result.count == 1
    assert result.healthy is False


# ---------------------------------------------------------------------------
# Guardrail: scripts canónicos incluyen pkill move_group (F1.16)
# ---------------------------------------------------------------------------


WS_ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize("script_relpath", list_known_move_group_killers())
def test_canonical_scripts_kill_move_group(script_relpath):
    """Cada script canónico debe contener pkill (o equivalente) de move_group.

    Esto es la red de seguridad F1.16: si alguien refactoriza los cleanup
    scripts y olvida incluir `move_group`, este test cae. El bug original
    provocaba 3 zombies tras varios cycles → APPROACH timeout en cycle 2/3.
    """
    script = WS_ROOT / script_relpath
    if not script.exists():
        pytest.skip(f"Script opcional no presente: {script_relpath}")
    content = script.read_text(encoding="utf-8")
    assert "move_group" in content, (
        f"{script_relpath} debe matar move_group en su limpieza. "
        "Si lo borraste a propósito, actualiza list_known_move_group_killers()."
    )
    # Y debe tener pkill o equivalente (no sólo mencionarlo en un comentario)
    has_kill = (
        "pkill" in content
        or "kill -" in content
        or "kill_pids" in content
    )
    assert has_kill, (
        f"{script_relpath} menciona move_group pero no tiene pkill — "
        "probablemente sólo en un comentario. Asegúrate de que MATE el proceso."
    )


def test_at_least_5_scripts_kill_move_group():
    """Invariante: hay >=5 scripts en el set canónico de killers."""
    assert len(list_known_move_group_killers()) >= 5
