"""Regresión arquitectónica: cada key de `runtime_defaults.yaml` se
referencia al menos una vez en el código del workspace.

Audit 2026-05-10 (Action 8). Detecta:
  - keys huérfanas en el YAML (defaults muertos que nadie consume).
  - regresión donde se borra la única lectura de una key sin retirar
    la entrada del YAML.

NOT a freshness check: solo busca el nombre como string en cualquier
fichero `*.py` o `*.yaml` del workspace (excluyendo build/install/
log/auditoria/historico/.tmp_*).
"""
from __future__ import annotations

from pathlib import Path

import pytest

WS_SRC = Path(__file__).resolve().parents[2]
assert WS_SRC.name == "src", f"unexpected layout: {WS_SRC}"
DEFAULTS_YAML = (
    WS_SRC / "ur5_bringup" / "config" / "runtime_defaults.yaml"
)
SEARCH_ROOT = WS_SRC.parent  # → agarre_ros2_ws/

EXCLUDE_DIRS = {
    "build",
    "install",
    "log",
    "auditoria",
    "historico",
    "__pycache__",
    ".pytest_cache",
    ".ruff_cache",
    ".mypy_cache",
}

# Permitir keys deliberadamente sólo-defaults (no consumidas hoy pero
# previstas para próximas fases). Mantener vacío hasta que sea
# imprescindible.
ALLOWLIST_UNUSED: set[str] = set()


def _load_keys() -> list[str]:
    import yaml

    with DEFAULTS_YAML.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    assert isinstance(data, dict), "runtime_defaults.yaml is not a YAML mapping"
    return sorted(data.keys())


def _gather_files() -> list[Path]:
    out: list[Path] = []
    for path in SEARCH_ROOT.rglob("*"):
        if not path.is_file():
            continue
        if any(part in EXCLUDE_DIRS or part.startswith(".tmp_") for part in path.parts):
            continue
        if path == DEFAULTS_YAML:
            continue
        if path.suffix not in {".py", ".yaml", ".yml", ".sh", ".md"}:
            continue
        out.append(path)
    return out


_FILES = _gather_files()


@pytest.mark.parametrize("key", _load_keys())
def test_runtime_default_key_consumed(key: str) -> None:
    if key in ALLOWLIST_UNUSED:
        pytest.skip(f"{key} en ALLOWLIST_UNUSED")
    for path in _FILES:
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        if key in text:
            return
    pytest.fail(
        f"key '{key}' de runtime_defaults.yaml no aparece en ningún "
        f"fichero del workspace. Considera borrarla del YAML o añadirla "
        f"a ALLOWLIST_UNUSED si es para una fase posterior."
    )
