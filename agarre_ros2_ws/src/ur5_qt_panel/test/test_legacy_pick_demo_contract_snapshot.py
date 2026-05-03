#!/usr/bin/env python3
"""F4 — Contract snapshot del legacy panel_pick_demo.

Bloquea regresiones en el contrato externo del legacy (env vars leídas,
signatures públicas, ROS topics/services) ANTES de la migración F8.

Si el orchestrator client (PickPlaceClient) reemplaza a run_pick_demo,
debe respetar este contrato o documentar explícitamente la divergencia.

No mide comportamiento dinámico — solo el ABI estático que F8 debe
preservar (o explicitamente romper con `# breaking_change_f8` comment).
"""

from __future__ import annotations

import ast
import inspect
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
LEGACY_PATH = REPO_ROOT / "src" / "ur5_qt_panel" / "ur5_qt_panel" / "panel_pick_demo.py"


def _module_ast() -> ast.Module:
    if not LEGACY_PATH.is_file():
        pytest.skip(f"legacy file not present: {LEGACY_PATH}")
    return ast.parse(LEGACY_PATH.read_text(encoding="utf-8"))


def _walk_calls(tree: ast.AST):
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            yield node


def _attr_chain(call: ast.Call) -> str:
    parts: list[str] = []
    func = call.func
    while isinstance(func, ast.Attribute):
        parts.append(func.attr)
        func = func.value
    if isinstance(func, ast.Name):
        parts.append(func.id)
    return ".".join(reversed(parts))


def _string_args(call: ast.Call) -> list[str]:
    out: list[str] = []
    for arg in call.args:
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
            out.append(arg.value)
    return out


def test_run_pick_demo_signature_unchanged():
    """run_pick_demo debe seguir siendo callable como ``run_pick_demo(panel)``."""
    tree = _module_ast()
    funcs = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "run_pick_demo"
    ]
    assert len(funcs) == 1, "Se esperaba exactamente un run_pick_demo top-level"
    func = funcs[0]
    arg_names = [a.arg for a in func.args.args]
    assert arg_names == ["panel"], (
        f"Signature de run_pick_demo cambió: args={arg_names}. "
        "Si F8 introduce un wrapper con kwargs, mantén alias panel-only."
    )


def test_dispatcher_imports_legacy_lazily():
    """pick_demo_dispatcher debe seguir capaz de cargar el legacy."""
    from ur5_qt_panel import pick_demo_dispatcher  # noqa: F401
    assert hasattr(pick_demo_dispatcher, "_legacy_dispatch"), (
        "pick_demo_dispatcher._legacy_dispatch missing — el fallback rollback se rompió"
    )
    src = inspect.getsource(pick_demo_dispatcher._legacy_dispatch)
    assert "from .panel_pick_demo import run_pick_demo" in src, (
        "_legacy_dispatch ya no importa run_pick_demo: rollback rápido perdido"
    )


def test_legacy_env_vars_minimum_set():
    """Bloque mínimo de env vars del legacy que el orchestrator debe respetar.

    Si la lista cambia, F8 debe (a) propagar los nuevos al orchestrator
    via parámetros del action goal o (b) documentar la migración.
    """
    tree = _module_ast()
    env_names: set[str] = set()
    for call in _walk_calls(tree):
        chain = _attr_chain(call)
        if chain in ("os.environ.get", "os.getenv"):
            env_names.update(_string_args(call))

    confirmed_present_2026_05_03 = {
        "PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M",
        "PANEL_PICK_DEMO_APPROACH_COARSE_KEEP_XY_TOL_M",
        "PANEL_PICK_DEMO_DIRECT_IK_ERR_TOL",
    }
    missing = confirmed_present_2026_05_03 - env_names
    assert not missing, (
        f"Env vars del contrato legacy 2026-05-03 desaparecieron: {missing}. "
        "Si F8 reemplazó el legacy, actualizar baseline o eliminar el test."
    )
    assert len(env_names) >= 15, (
        f"El legacy lee solo {len(env_names)} env vars (esperaba >=15). "
        "Posible regresión silenciosa del contrato."
    )


def test_dispatcher_orchestrator_path_present():
    """El path orchestrator del dispatcher debe seguir disponible (F8 base)."""
    from ur5_qt_panel import pick_demo_dispatcher
    src = inspect.getsource(pick_demo_dispatcher)
    assert "PickPlaceClient" in src, "PickPlaceClient ya no se usa en dispatcher"
    assert "should_use_orchestrator" in src, (
        "should_use_orchestrator missing — la lógica de selección se rompió"
    )


def test_pick_place_action_interface_unchanged():
    """La acción /pick_place debe seguir siendo la canónica (orchestrator)."""
    try:
        from ur5_panel_interfaces.action import PickPlace
    except Exception as exc:
        pytest.skip(f"ur5_panel_interfaces no disponible: {exc}")
    goal = PickPlace.Goal()
    feedback = PickPlace.Feedback()
    result = PickPlace.Result()
    assert hasattr(goal, "object_name"), "Goal.object_name desaparecido"
    assert hasattr(feedback, "phase") or hasattr(feedback, "current_phase"), (
        "Feedback sin campo phase — F7 perderá feedback intermedio"
    )
    assert hasattr(result, "success"), "Result.success desaparecido"


def test_legacy_loc_baseline_documented():
    """Snapshot del LOC del legacy en el momento de pre-F8.

    Si baja >50% asume que F8 ya está parcialmente hecho — actualizar
    el baseline. Si sube >5% es regresión.
    """
    if not LEGACY_PATH.is_file():
        pytest.skip(f"legacy file not present: {LEGACY_PATH}")
    loc = sum(1 for _ in LEGACY_PATH.read_text(encoding="utf-8").splitlines())
    baseline = 8623
    upper_regression = int(baseline * 1.05)
    lower_f8_floor = int(baseline * 0.50)
    assert loc <= upper_regression, (
        f"Regresión en panel_pick_demo.py: {loc} LOC > baseline*1.05={upper_regression}"
    )
    if loc < lower_f8_floor:
        pytest.skip(
            f"F8 ya parcialmente aplicado: {loc} LOC < {lower_f8_floor}. "
            "Actualizar baseline en este test."
        )
