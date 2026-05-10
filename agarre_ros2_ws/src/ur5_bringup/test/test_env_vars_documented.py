#!/usr/bin/env python3
# Ruta/archivo: src/ur5_bringup/test/test_env_vars_documented.py
# Contenido: F4.3 audit (2026-05-10) — guardia de documentación de env vars.
"""F4.3 audit (2026-05-10): cada PANEL_* env var debe estar documentada.

Política:
  * Cada env var managed (PANEL_*, SYSTEM_STATE_*, ATTACH_BACKEND_*)
    referenciada en código de producción debe tener al menos UNA de:
      a) entrada en `runtime_defaults.yaml` (default explícito + doc).
      b) entrada en una dataclass *_params.py con un comentario.
      c) entrada en residual_env_vars_baseline.txt (deuda explícita).
  * El test escanea `os.environ.get("VAR")` y `os.environ["VAR"]` y
    valida que cada VAR está documentada en alguna de las 3 fuentes.

Esto cierra P-08 (parcial): aunque hay 87 lecturas os.environ, al
menos cada variable está conscientemente declarada.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import Set


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SRC_ROOT = WORKSPACE_ROOT / "src"


_ENV_LITERAL_PATTERN = re.compile(
    r"os\.environ\.get\(\s*['\"]([A-Z_][A-Z0-9_]+)['\"]"
    r"|os\.environ\[\s*['\"]([A-Z_][A-Z0-9_]+)['\"]\s*\]"
)


_MANAGED_PREFIXES = ("PANEL_", "SYSTEM_STATE_", "ATTACH_BACKEND_", "TFM_GRASP_", "STRICT_PHYSICS_")


def _scan_env_literals_in_code() -> Set[str]:
    """Escanea env literals en src/ excluyendo tests, build y install."""
    found: Set[str] = set()
    for path in SRC_ROOT.rglob("*.py"):
        s = str(path)
        if "/test/" in s or "/build/" in s or "/install/" in s:
            continue
        if path.name.startswith("test_"):
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        for m in _ENV_LITERAL_PATTERN.finditer(text):
            name = m.group(1) or m.group(2)
            if name:
                found.add(name)
    return found


def _is_managed(name: str) -> bool:
    return any(name.startswith(p) for p in _MANAGED_PREFIXES)


def _yaml_keys() -> Set[str]:
    """Carga claves de runtime_defaults.yaml."""
    yaml_path = SRC_ROOT / "ur5_bringup" / "config" / "runtime_defaults.yaml"
    if not yaml_path.is_file():
        return set()
    try:
        import yaml  # type: ignore
    except Exception:
        return set()
    try:
        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    except Exception:
        return set()
    if not isinstance(data, dict):
        return set()
    return set(data.keys())


def _params_documented_keys() -> Set[str]:
    """Escanea *_params.py por env names referenciados en comentarios.

    Asumimos que cualquier dataclass *_params.py registra sus env vars
    como comentarios en línea. El patrón es: ``# PANEL_FOO_BAR``.
    """
    found: Set[str] = set()
    for path in SRC_ROOT.rglob("*_params.py"):
        if "/test/" in str(path):
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        for m in re.finditer(r"#\s*([A-Z_][A-Z0-9_]+)\b", text):
            name = m.group(1)
            if _is_managed(name):
                found.add(name)
    return found


def _residual_baseline() -> Set[str]:
    """Carga residual_env_vars_baseline.txt."""
    baseline_path = SRC_ROOT / "ur5_bringup" / "schemas" / "residual_env_vars_baseline.txt"
    if not baseline_path.is_file():
        return set()
    return {
        line.strip()
        for line in baseline_path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.strip().startswith("#")
    }


def test_each_managed_env_var_is_documented() -> None:
    """Cada PANEL_*/SYSTEM_STATE_*/etc referenciado tiene doc en YAML/params/baseline."""
    referenced = {n for n in _scan_env_literals_in_code() if _is_managed(n)}
    yaml_keys = _yaml_keys()
    params_keys = _params_documented_keys()
    baseline = _residual_baseline()
    documented = yaml_keys | params_keys | baseline
    undocumented = referenced - documented
    assert not undocumented, (
        f"{len(undocumented)} env var(s) managed sin documentación. Documentar en "
        f"runtime_defaults.yaml, en una dataclass *_params.py (comentario "
        f"# NOMBRE_VAR), o añadir a residual_env_vars_baseline.txt:\n  "
        + "\n  ".join(sorted(undocumented))
    )


def test_documentation_sources_exist() -> None:
    """Sanity: los 3 mecanismos de documentación están presentes."""
    assert _yaml_keys(), "runtime_defaults.yaml vacío o no encontrado"
    assert _params_documented_keys(), "ningún *_params.py tiene comentarios documentando env vars"
    assert _residual_baseline(), "residual_env_vars_baseline.txt vacío o no encontrado"


def test_no_env_var_in_baseline_and_yaml_simultaneously() -> None:
    """Una env var no debe estar simultáneamente en YAML y baseline.

    El baseline es deuda explícita; cuando la migras a YAML, la sacas
    del baseline. Si está en ambos, hay drift.
    """
    yaml_keys = _yaml_keys()
    baseline = _residual_baseline()
    overlap = yaml_keys & baseline
    assert not overlap, (
        "Env vars en YAML Y en residual_env_vars_baseline.txt — "
        "elegir una fuente:\n  " + "\n  ".join(sorted(overlap))
    )
