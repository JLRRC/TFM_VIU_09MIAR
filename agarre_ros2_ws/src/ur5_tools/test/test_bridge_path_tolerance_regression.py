#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_bridge_path_tolerance_regression.py
# Contenido: T28 — regression test del fix bug bridge race condition (commit f18b2c2).
"""T28 regression — fix bug bridge race condition (controller_manager).

Bug original (`auditoria/bugs_pendientes/BUG_BRIDGE_PATH_TOLERANCE.md`): el primer goal del
orchestrator a `/move_action` fallaba con `CONTROL_FAILED` porque MoveIt
`simple_controller_manager` aún no había detectado el
`joint_trajectory_controller` action server (race condition de arranque).

Fix aplicado en `plan_to_pose_server.py:_execute_moveit_direct` (commit
`f18b2c2`, 2026-05-07): tras `CONTROL_FAILED` o `TIMED_OUT`, espera 8s y
reintenta UNA vez. Si converge, marca reason con sufijo `retry_after_*`;
si vuelve a fallar, marca `retry_failed`.

Estos tests AST verifican que el fix sigue presente — no que funciona en
runtime (eso lo cubre la validación live de 3 ciclos pick+transport+release
documentada en F1.5 del audit v4). Su única función es evitar que el fix
se borre accidentalmente en un refactor futuro.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

WS_ROOT = Path(__file__).resolve().parents[3]
PLAN_TO_POSE_SRC = WS_ROOT / "src" / "ur5_tools" / "ur5_tools" / "plan_to_pose_server.py"


def _load_src() -> str:
    assert PLAN_TO_POSE_SRC.exists(), f"plan_to_pose_server.py no encontrado en {PLAN_TO_POSE_SRC}"
    return PLAN_TO_POSE_SRC.read_text(encoding="utf-8")


def _execute_moveit_direct_body() -> ast.Module:
    """Devuelve un AST.Module con `_execute_moveit_direct` + todos los helpers
    `_moveit_*` (refactor T15 2026-05-09 los extrajo a sub-métodos).

    Esto permite que los tests T28 sigan verificando que la BEHAVIOR del fix
    bug bridge está presente, independientemente de dónde viva exactamente
    cada bloque (orquestador vs helpers).
    """
    tree = ast.parse(_load_src(), filename=str(PLAN_TO_POSE_SRC))
    relevant: list = []
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and (
            node.name == "_execute_moveit_direct"
            or node.name.startswith("_moveit_")
        ):
            relevant.append(node)
    assert relevant, (
        "`_execute_moveit_direct` ni helpers `_moveit_*` encontrados en "
        "plan_to_pose_server.py — fix bug bridge ausente."
    )
    # Empaqueta en un Module sintético para `ast.unparse` y `ast.walk` consistentes.
    return ast.Module(body=relevant, type_ignores=[])


def test_t28_retry_block_present() -> None:
    """T28-a — el bloque de retry existe en `_execute_moveit_direct`.

    Busca un `if` cuyo test referencie literal "CONTROL_FAILED" o "TIMED_OUT".
    """
    fn = _execute_moveit_direct_body()
    src = ast.unparse(fn)
    assert "CONTROL_FAILED" in src, (
        "El fix bug bridge requiere detectar 'CONTROL_FAILED' como gatillo del retry. "
        "Si se renombró el código de error, actualiza también el bug doc y este test."
    )
    assert "TIMED_OUT" in src, (
        "El fix bug bridge requiere detectar 'TIMED_OUT' como gatillo del retry."
    )


def test_t28_retry_has_backoff_sleep() -> None:
    """T28-b — hay un `time.sleep(...)` con argumento ≥ 5s antes del retry.

    El fix usa 8s para dar tiempo a MoveIt a conectar al controller. Cualquier
    valor < 5s probablemente no resuelve la race condition.
    """
    fn = _execute_moveit_direct_body()
    sleep_durations: list[float] = []
    for node in ast.walk(fn):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "sleep"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[0].value, (int, float))
        ):
            sleep_durations.append(float(node.args[0].value))
    assert any(d >= 5.0 for d in sleep_durations), (
        f"El fix bug bridge requiere un sleep ≥ 5s antes del retry; "
        f"encontrados sleeps: {sleep_durations}. Si el sleep se externalizó a "
        "un parámetro, actualiza este test para leer el default."
    )


def test_t28_retry_reason_markers_present() -> None:
    """T28-c — el reason del retry incluye marcadores que el cliente parsea.

    `retry_after_<original_error>` indica retry exitoso; `retry_failed` indica
    fallo en el segundo intento. El orchestrator y el evidence_logger
    consumen estos marcadores para distinguir "OK al primer goal" vs
    "OK tras retry" vs "fallo definitivo".
    """
    fn = _execute_moveit_direct_body()
    src = ast.unparse(fn)
    assert "retry_after" in src, (
        "El fix bug bridge debe marcar el reason con 'retry_after_<error>' "
        "cuando el retry resuelve el fallo (downstream consumers parsean esto)."
    )
    assert "retry_failed" in src, (
        "El fix bug bridge debe marcar el reason con 'retry_failed' cuando el "
        "segundo intento también falla."
    )


def test_t28_debug_script_exists() -> None:
    """T28-d — el script de reproducción aislada sigue presente.

    `debug_bridge_isolated.sh` reproduce el bug sin orchestrator ni panel.
    Es la herramienta de diagnóstico canónica para futuras regresiones.
    """
    debug_script = WS_ROOT / "scripts" / "debug_bridge_isolated.sh"
    assert debug_script.exists(), (
        f"Script de reproducción aislada no encontrado en {debug_script}. "
        "Se necesita para investigar regresiones del bug bridge."
    )
    content = debug_script.read_text(encoding="utf-8")
    assert "/move_action" in content, "El script debe enviar goals a /move_action."
    assert "5 goals" in content.lower() or "for i in 1 2 3 4 5" in content, (
        "El script debe enviar al menos 5 goals consecutivos para detectar varianza."
    )


def test_t28_bug_doc_references_fix() -> None:
    """T28-e — el bug doc canónico existe y referencia el fix.

    El bug doc en `auditoria/bugs_pendientes/BUG_BRIDGE_PATH_TOLERANCE.md`
    (workspace root) debe seguir documentando el bug. Si se cierra y archiva,
    este test debe actualizarse.
    """
    repo_root = WS_ROOT.parent  # /home/laboratorio/TFM
    bug_doc = repo_root / "auditoria" / "bugs_pendientes" / "BUG_BRIDGE_PATH_TOLERANCE.md"
    assert bug_doc.exists(), (
        f"Bug doc no encontrado en {bug_doc}. Si se reorganizaron los docs, "
        "actualiza la ruta en este test."
    )
    content = bug_doc.read_text(encoding="utf-8")
    assert "CONTROL_FAILED" in content or "TIMED_OUT" in content, (
        "El bug doc debe describir los códigos de error del bug original."
    )
