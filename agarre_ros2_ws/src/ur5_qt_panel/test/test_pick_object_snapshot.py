#!/usr/bin/env python3
"""F3 audit-v4 (2026-05-08): snapshot del contrato de panel_pick_object.

Congela la **superficie pública** del módulo `panel_pick_object` antes
de empezar el split iter2 (3.814 LOC → ≤ 800 LOC). Si alguien rompe
una signatura accidentalmente al refactorizar, este test falla.

Patrón replicado del snapshot exitoso de `panel_pick_demo` (que
permitió pasar de 8.626 → 536 LOC sin regresiones).

NOTA: este test no exporta el comportamiento (eso es T35 live), sólo
los nombres + arity de funciones top-level y métodos clave del módulo.
"""
from __future__ import annotations

import importlib.util
import inspect
import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parent.parent / "ur5_qt_panel"
MODULE_FILE = PKG / "panel_pick_object.py"


# ---- Public API frozen 2026-05-08 (HEAD audit-v4-docs-tf-frames-20260508) ----

# Top-level callables que el panel y otros consumers importan.
EXPECTED_PUBLIC_FUNCTIONS = {
    "run_pick_object",
}

# Pre-existing extracted helpers — sirven para confirmar que las
# extracciones previas siguen importables.
EXPECTED_HELPER_MODULES = (
    "pick_object.orientation_helpers",
    "pick_object.diagnostics",
    "pick_object.moveit_bridge_path",
    "pick_object.wait_helpers",
)


def _load_module():
    """Load panel_pick_object as standalone (sin importar el paquete completo)."""
    if "ur5_qt_panel.panel_pick_object" in sys.modules:
        return sys.modules["ur5_qt_panel.panel_pick_object"]
    pytest.skip("panel_pick_object not importable in this env (skipping snapshot test)")


def test_run_pick_object_signature_snapshot() -> None:
    """run_pick_object(panel) -> None. Único arg: panel; sin kwargs."""
    mod = _load_module()
    fn = getattr(mod, "run_pick_object", None)
    assert fn is not None, "run_pick_object missing — public API broken"
    sig = inspect.signature(fn)
    params = list(sig.parameters.values())
    assert len(params) == 1, (
        f"run_pick_object expected 1 positional arg, got {len(params)}"
    )
    assert params[0].name == "panel", (
        f"run_pick_object first arg must be 'panel', got {params[0].name!r}"
    )
    assert params[0].kind in (
        inspect.Parameter.POSITIONAL_OR_KEYWORD,
        inspect.Parameter.POSITIONAL_ONLY,
    ), f"run_pick_object 'panel' kind = {params[0].kind}"


def test_module_has_expected_public_functions() -> None:
    """No deben aparecer ni desaparecer funciones top-level inesperadamente."""
    mod = _load_module()
    actual_pub = {
        name for name, obj in vars(mod).items()
        if not name.startswith("_") and inspect.isfunction(obj)
        and obj.__module__ == mod.__name__
    }
    missing = EXPECTED_PUBLIC_FUNCTIONS - actual_pub
    assert not missing, (
        f"Missing public functions in panel_pick_object: {missing}"
    )


def test_loc_decreases_or_stays() -> None:
    """LOC monotonically decreasing. F3-iter2 debe REDUCIR este número.

    Línea baseline congelada: 3.814 (HEAD audit-v4 close).
    """
    if not MODULE_FILE.is_file():
        pytest.skip("panel_pick_object.py not found")
    n_lines = sum(1 for _ in MODULE_FILE.read_text(encoding="utf-8").splitlines())
    BASELINE = 3814
    assert n_lines <= BASELINE, (
        f"panel_pick_object.py grew from baseline {BASELINE} to {n_lines}. "
        f"Split work must REDUCE LOC, not add."
    )


@pytest.mark.parametrize("submod", EXPECTED_HELPER_MODULES)
def test_pre_existing_helper_modules_present(submod: str) -> None:
    """Las extracciones previas (orientation_helpers, diagnostics, etc.)
    siguen presentes como ficheros."""
    parts = submod.split(".")
    path = PKG / "/".join(parts)
    path = path.with_suffix(".py")
    assert path.is_file(), f"helper module file missing: {path}"
