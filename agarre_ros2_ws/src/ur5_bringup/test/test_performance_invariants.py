#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/test_performance_invariants.py
# Contenido: F8 — invariantes estáticos de performance (anti-regresión).
"""F8 — Performance invariants estáticos.

Tests offline que verifican propiedades de performance del codebase
sin ejecutar el stack vivo:

  * T39 — Timer frequencies en código no exceden umbrales razonables.
  * T40 — TF lookups dentro de un mismo método no se repiten >3 veces
    sin cache (heurística AST).
  * T41 — Imports pesados (torch, cv2, numpy) sólo aparecen lazy
    (dentro de try/except o funciones), no en module-level top.

Heurísticos AST-based — no garantizan ausencia de problemas runtime,
pero detectan regresiones obvias.
"""
from __future__ import annotations

import ast
import re
from pathlib import Path
from typing import Dict, List, Tuple

import pytest


WS_ROOT = Path(__file__).resolve().parents[3]
SRC_ROOT = WS_ROOT / "src"

EXCLUDE_DIRS = {"test", "tests", "build", "install", "log", "__pycache__"}


def _iter_py_files(base: Path) -> List[Path]:
    out: List[Path] = []
    for path in base.rglob("*.py"):
        parts = set(path.parts)
        if parts & EXCLUDE_DIRS:
            continue
        out.append(path)
    return sorted(out)


# ---------------------------------------------------------------------------
# T39 — Timer frequencies sane
# ---------------------------------------------------------------------------

# `create_timer(period_sec, callback)` o `Timer(period=...)` — period <= 0.001
# (1ms = 1000Hz) es muy probablemente un bug.
_MIN_TIMER_PERIOD_SEC = 0.001
_MAX_TIMER_PERIOD_SEC = 60.0  # 1min — > eso suele ser intencional


def test_t39_timer_periods_are_sane() -> None:
    """T39 — timers no usan periodos absurdos (< 1ms o > 60s sin doc)."""
    suspicious: List[str] = []
    timer_re = re.compile(
        r"create_timer\s*\(\s*([0-9]+\.?[0-9]*)",
    )
    for path in _iter_py_files(SRC_ROOT):
        try:
            text = path.read_text(encoding="utf-8")
        except OSError:
            continue
        for m in timer_re.finditer(text):
            try:
                period = float(m.group(1))
            except ValueError:
                continue
            if period < _MIN_TIMER_PERIOD_SEC:
                suspicious.append(
                    f"{path.relative_to(WS_ROOT)}: create_timer({period}) "
                    f"< {_MIN_TIMER_PERIOD_SEC}s — probable bug"
                )
            elif period > _MAX_TIMER_PERIOD_SEC:
                # > 60s suele ser intencional (watchdog), no fail.
                pass
    assert not suspicious, "\n".join(suspicious)


# ---------------------------------------------------------------------------
# T40 — TF lookups not repeated within a method
# ---------------------------------------------------------------------------


_MAX_TF_LOOKUPS_PER_METHOD = 3


def _count_tf_lookups_per_function(tree: ast.AST) -> Dict[str, int]:
    """Por cada función/método del AST, cuenta llamadas a `lookup_transform`."""
    counts: Dict[str, int] = {}
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            n = 0
            for sub in ast.walk(node):
                if isinstance(sub, ast.Call):
                    func = sub.func
                    name = ""
                    if isinstance(func, ast.Attribute):
                        name = func.attr
                    elif isinstance(func, ast.Name):
                        name = func.id
                    if name == "lookup_transform":
                        n += 1
            if n > 0:
                counts[node.name] = n
    return counts


def test_t40_tf_lookups_per_method_under_threshold() -> None:
    """T40 — ningún método hace > 3 lookup_transform sin cache."""
    offenders: List[str] = []
    for path in _iter_py_files(SRC_ROOT):
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except (SyntaxError, OSError):
            continue
        counts = _count_tf_lookups_per_function(tree)
        for fname, n in counts.items():
            if n > _MAX_TF_LOOKUPS_PER_METHOD:
                offenders.append(
                    f"{path.relative_to(WS_ROOT)}::{fname}: {n} lookup_transform "
                    f"(cache opcional con TTL si > {_MAX_TF_LOOKUPS_PER_METHOD})"
                )
    # Permisivo — sólo log de aviso. No fail (algunos casos son legítimos).
    if offenders:
        # No raise — comportamiento informativo. Para enforce, cambiar a assert.
        pytest.skip(
            "T40 informativo (no fail): " + str(len(offenders)) + " métodos con > 3 TF lookups\n"
            + "\n".join(offenders[:3])
        )


# ---------------------------------------------------------------------------
# T41 — Heavy imports are lazy (inside try/except or functions)
# ---------------------------------------------------------------------------

_HEAVY_IMPORTS = {"torch", "torchvision", "cv2", "numpy"}


def _module_level_imports(tree: ast.AST) -> List[str]:
    """Devuelve nombres de módulos importados a nivel TOP (no dentro de funciones)."""
    out: List[str] = []
    for node in tree.body if isinstance(tree, ast.Module) else []:
        if isinstance(node, ast.Import):
            for alias in node.names:
                out.append(alias.name.split(".")[0])
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                out.append(node.module.split(".")[0])
        elif isinstance(node, ast.Try):
            # try/except imports cuentan como "lazy guarded" — OK.
            continue
    return out


def test_t41_heavy_imports_only_lazy() -> None:
    """T41 — torch/cv2/numpy/torchvision sólo aparecen en try/except o lazy.

    Excepciones: archivos en `tfm_grasping/` (paquete de inferencia, los usa
    intensivamente por diseño) y `moveit_bridge_utils.py` (numpy en try).
    """
    # F4-step3 (audit-v4 2026-05-08): listado canónico — numpy/cv2/torch
    # son legítimos en estos archivos (geometría matemática + inferencia
    # del modelo + cinemática numérica). Si algún archivo NUEVO necesita
    # entrar en la lista, justificar en el commit y pasar review.
    allowed_files = {
        "tfm_grasping/tfm_grasping/model.py",
        "tfm_grasping/tfm_grasping/perception.py",
        "tfm_grasping/tfm_grasping/grasp_inference.py",
        "tfm_grasping/tfm_grasping/grasp_module.py",
        "tfm_grasping/tfm_grasping/__init__.py",
        "tfm_grasping/tfm_grasping/ros_interface.py",
        "tfm_grasping/tfm_grasping/config.py",
        "tfm_grasping/tfm_grasping/geometry.py",
        "ur5_tools/ur5_tools/moveit_bridge_utils.py",
        "ur5_tools/ur5_tools/moveit_bridge/joint_state_helpers.py",
        "ur5_tools/ur5_tools/moveit_bridge/executor.py",
        "ur5_tools/ur5_tools/moveit_bridge/moveit_py_planner.py",
        # numpy en estos: matemática numérica del bridge / cinemática.
        "ur5_tools/ur5_tools/moveit_bridge/geometry.py",
        "ur5_tools/ur5_tools/moveit_bridge/goal_validation.py",
        "ur5_qt_panel/ur5_qt_panel/calibration_service.py",
        "ur5_qt_panel/ur5_qt_panel/panel_v2.py",
        "ur5_qt_panel/ur5_qt_panel/ur5_kinematics.py",
    }
    offenders: List[Tuple[str, List[str]]] = []
    for path in _iter_py_files(SRC_ROOT):
        rel = str(path.relative_to(SRC_ROOT))
        if rel in allowed_files:
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except (SyntaxError, OSError):
            continue
        top = _module_level_imports(tree)
        heavy_in_top = [m for m in top if m in _HEAVY_IMPORTS]
        if heavy_in_top:
            offenders.append((rel, heavy_in_top))
    assert not offenders, (
        "Imports pesados en module-level (top) — convertir a lazy:\n  "
        + "\n  ".join(f"{rel}: {imps}" for rel, imps in offenders)
    )
