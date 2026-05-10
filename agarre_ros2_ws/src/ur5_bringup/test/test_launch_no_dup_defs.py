"""Regresión arquitectónica: ningún `*.launch.py` contiene definiciones
duplicadas de la misma función a top-level.

Audit 2026-05-10 (Action 8) — protege contra el bug B1 donde
`_env_flag` y `_env_float` estaban definidas dos veces en
`ur5_stack.launch.py`. La 2ª definición sombreaba a la 1ª y
silenciaba `runtime_defaults.yaml` para ~30 tunables.

Bug cerrado en commit 2b32fb4 (2026-05-10).
"""
from __future__ import annotations

import ast
from pathlib import Path
from collections import Counter

import pytest

WS_SRC = Path(__file__).resolve().parents[2]
assert WS_SRC.name == "src", f"unexpected layout: {WS_SRC}"


def _discover_launch_files() -> list[Path]:
    files: list[Path] = []
    for pkg in WS_SRC.iterdir():
        launch_dir = pkg / "launch"
        if not launch_dir.is_dir():
            continue
        files.extend(sorted(launch_dir.glob("*.launch.py")))
        files.extend(sorted(launch_dir.glob("*.py")))
    # Dedup preservando orden.
    seen: set[Path] = set()
    out: list[Path] = []
    for path in files:
        if path in seen:
            continue
        seen.add(path)
        out.append(path)
    return out


@pytest.mark.parametrize(
    "launch_path",
    _discover_launch_files(),
    ids=lambda p: f"{p.parent.parent.name}/{p.name}",
)
def test_no_duplicate_top_level_defs(launch_path: Path) -> None:
    """A top-level `def foo(...)` no aparece más de una vez por fichero."""
    tree = ast.parse(launch_path.read_text(encoding="utf-8"))
    names: list[str] = []
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            names.append(node.name)
    counts = Counter(names)
    duplicates = {name: n for name, n in counts.items() if n > 1}
    assert not duplicates, (
        f"{launch_path.relative_to(WS_SRC.parent)}: definiciones top-level "
        f"duplicadas: {duplicates}"
    )
