#!/usr/bin/env python3
"""F2 audit-v4 (2026-05-08): drift test env↔YAML.

Verifica consistencia bidireccional entre ``runtime_defaults.yaml`` y los
env vars referenciados en el código:

1. Cada env var managed (prefijo PANEL_/SYSTEM_STATE_/ATTACH_BACKEND_/...)
   referenciada en *.py debe aparecer en YAML (o en allowlist debug-only).
2. Cada YAML key debe ser referenciada en al menos un *.py o en una *.yaml.

Detecta env vars referenced en código como string literals — no requiere
pasar por os.environ.get explícito (cubre tuples/dicts/launch_helpers).

Why: ``runtime_defaults.yaml`` es la source of truth (CONFIG_HIERARCHY.md).
Env vars no documentadas son overrides silenciosos.
"""
from __future__ import annotations

import re
from pathlib import Path
from typing import Set

import pytest
import yaml

WS_DIR = Path(__file__).resolve().parents[3]
RUNTIME_DEFAULTS = WS_DIR / "src/ur5_bringup/config/runtime_defaults.yaml"

MANAGED_PREFIXES = (
    "PANEL_",
    "SYSTEM_STATE_",
    "ATTACH_BACKEND_",
    "STRICT_PHYSICS_",
    "GZ_RENDER_",
)

ALLOWED_DEBUG_ONLY: Set[str] = {
    "PICK_DEBUG",
    "PANEL_NO_GZ",
    "USE_LEGACY_PICK_DEMO",
    "STRICT_SELF_COLLISION",
    "GZ_PARTITION",
    "GZ_IP",
    "GZ_TRANSPORT_IP",
    "PANEL_ROS_TIMEOUT",
    "PANEL_MAX_FPS",
    "PANEL_ROS2_ONLY",
    "PANEL_SINGLE_CAM",
    "PANEL_NO_DISPLAY",
    "PANEL_BASE",
    "WORLD_FRAME",
    "DISPLAY",
    "ROS_DOMAIN_ID",
    "HOME",
    "USER",
    "PATH",
    "WS_DIR",
    "PANEL_PYTHON",
    "PANEL_QT_PLATFORM",  # F2 (2026-05-10): override Qt platform (xcb/wayland/offscreen)
    "GZ_RENDER_ENGINE",
    "PANEL_DIRECT_DEBUG_ROOT",
    "PANEL_KEEP_CAMERAS",
    "PANEL_CAMERA_REQUIRED",
    # Env vars que el panel/orchestrator usan transversalmente sin tunable.
    "PANEL_TF_HELPER_ROOT",
    "PANEL_PICK_DEMO_DRY_RUN",
    "PANEL_PICK_OBJECT_DRY_RUN",
}

EXCLUDE_DIRS = {"__pycache__", "build", "install", "log"}

# Env-var-shaped string literal: uppercase + underscore + minimum length 6.
# Heuristic: avoids common short ALL_CAPS like CSS, RGB, etc.
ENV_LITERAL_PATTERN = re.compile(r"[\"']([A-Z][A-Z0-9_]{5,})[\"']")


def _scan_env_literals_in_code() -> Set[str]:
    """Scan all .py and .yaml files in src/ for env-var-shaped string literals."""
    names: Set[str] = set()
    src = WS_DIR / "src"
    for ext in ("*.py", "*.yaml"):
        for f in src.rglob(ext):
            parts = set(f.parts)
            if parts & EXCLUDE_DIRS:
                continue
            if f.name.startswith("test_"):
                continue
            try:
                txt = f.read_text(encoding="utf-8")
            except (OSError, UnicodeDecodeError):
                continue
            for m in ENV_LITERAL_PATTERN.finditer(txt):
                names.add(m.group(1))
    return names


def _yaml_keys() -> Set[str]:
    if not RUNTIME_DEFAULTS.is_file():
        pytest.fail(f"runtime_defaults.yaml missing: {RUNTIME_DEFAULTS}")
    with RUNTIME_DEFAULTS.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        pytest.fail("runtime_defaults.yaml is not a mapping")
    return set(data.keys())


def _is_managed(name: str) -> bool:
    return any(name.startswith(prefix) for prefix in MANAGED_PREFIXES)


def _residual_baseline() -> Set[str]:
    """Load frozen baseline of env vars NOT yet in runtime_defaults.yaml.

    F2 audit-v4 (2026-05-08): the workspace has ~490 managed env vars
    referenced in code but not in runtime_defaults.yaml. Migrating them
    en bloc is risky (defaults could drift). Instead we freeze them as
    "known undocumented residual", and the gating test only fires when
    NEW env vars appear that aren't in YAML or in this baseline.

    The baseline file is monotonically decreasing: as env vars get
    centralized into runtime_defaults.yaml, remove them from the
    baseline. Adding a new entry should be a deliberate decision.
    """
    path = (
        WS_DIR
        / "src/ur5_bringup/schemas/residual_env_vars_baseline.txt"
    )
    if not path.is_file():
        return set()
    return {
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.startswith("#")
    }


def test_no_new_undocumented_managed_env_vars() -> None:
    """Each managed env var must appear in YAML, allowlist, or residual baseline.

    Gating: prevents NEW env vars from escaping documentation. The
    residual baseline is the frozen set of legacy env vars not yet
    migrated. Adding a new env var requires either documenting it in
    runtime_defaults.yaml or explicitly extending the baseline file.
    """
    names = _scan_env_literals_in_code()
    yaml_keys = _yaml_keys()
    baseline = _residual_baseline()
    managed = {n for n in names if _is_managed(n)}
    new_undocumented = managed - yaml_keys - ALLOWED_DEBUG_ONLY - baseline
    assert not new_undocumented, (
        "NEW env vars not in runtime_defaults.yaml, ALLOWED_DEBUG_ONLY, "
        "or residual_env_vars_baseline.txt. Document them properly:\n  "
        + "\n  ".join(sorted(new_undocumented))
    )


def test_residual_baseline_monotonically_decreasing() -> None:
    """The baseline must not contain entries that are now in YAML.

    Forces cleanup: when an env var moves into runtime_defaults.yaml,
    remove its line from residual_env_vars_baseline.txt.
    """
    yaml_keys = _yaml_keys()
    baseline = _residual_baseline()
    duplicated = baseline & yaml_keys
    assert not duplicated, (
        "Env vars present in BOTH residual_env_vars_baseline.txt and "
        "runtime_defaults.yaml. Remove them from the baseline:\n  "
        + "\n  ".join(sorted(duplicated))
    )


def test_residual_baseline_only_actually_referenced() -> None:
    """Baseline entries must actually appear in code (no zombie entries).

    Prevents the baseline from accumulating dead entries.
    """
    names = _scan_env_literals_in_code()
    baseline = _residual_baseline()
    zombies = baseline - names
    assert not zombies, (
        "Env vars in residual_env_vars_baseline.txt no longer referenced "
        "in code. Remove them:\n  " + "\n  ".join(sorted(zombies))
    )


def test_yaml_keys_referenced_somewhere() -> None:
    """Each YAML key must appear as a literal in some .py or .yaml file.

    Detects 'dead' YAML config: tunable defined but no consumer reads it.
    """
    names = _scan_env_literals_in_code()
    yaml_keys = _yaml_keys()
    unused = yaml_keys - names
    assert not unused, (
        "YAML keys defined in runtime_defaults.yaml but not referenced "
        "as a string literal anywhere in code:\n  "
        + "\n  ".join(sorted(unused))
    )
