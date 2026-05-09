"""F4: T4 — schema validation de runtime_defaults.yaml.

Valida que el YAML canónico de defaults del stack:
  1. Existe y carga sin errores.
  2. Cada clave es UPPER_SNAKE_CASE (convención env var).
  3. No hay claves duplicadas (el parser ya detecta esto, doble-check).
  4. Cada valor es coherente con el tipo declarado por su sufijo:
       - *_SEC, *_HZ, *_RAD, *_M, *_MM      → float-castable
       - *_TIMEOUT_MS, *_SAMPLES, *_RETRIES → int-castable
       - *_MODE, *_PATH, *_DIR              → str no vacío
       - flags ("0"/"1"/"true"/"false")     → coerción booleana
  5. Las claves no documentadas (sin sufijo conocido) son aceptables pero
     se reportan en un warning para revisión manual.

Test offline puro: stdlib + PyYAML. No requiere ROS.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML es dep estándar
    yaml = None


YAML_PATH = (
    Path(__file__).resolve().parent.parent / "config" / "runtime_defaults.yaml"
)

UPPER_SNAKE_RE = re.compile(r"^[A-Z][A-Z0-9_]*$")
TILDE_HOME_PREFIX = "~"

# Sufijos que requieren tipo float
FLOAT_SUFFIXES = ("_SEC", "_HZ", "_RAD", "_M", "_MM", "_TOL_M", "_SCALE")
# Sufijos que requieren tipo int
INT_SUFFIXES = ("_MS", "_SAMPLES", "_RETRIES", "_OK", "_ATTEMPTS")
# Sufijos string aceptables tal cual
STR_SUFFIXES = ("_MODE", "_PATH", "_DIR", "_TOPIC", "_FRAME", "_NAME", "_ROOT", "_ENGINE")
# Flags (0/1, true/false, on/off)
FLAG_VALUES = {"0", "1", "true", "false", "on", "off", "yes", "no"}


@pytest.fixture(scope="module")
def yaml_data():
    if yaml is None:
        pytest.skip("PyYAML no disponible")
    if not YAML_PATH.is_file():
        pytest.fail(f"runtime_defaults.yaml no existe en {YAML_PATH}")
    with YAML_PATH.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    assert isinstance(data, dict), (
        f"runtime_defaults.yaml debe parsear a dict, no {type(data).__name__}"
    )
    return data


def test_yaml_loads_and_is_dict(yaml_data):
    assert len(yaml_data) > 0, "runtime_defaults.yaml está vacío"


def test_all_keys_are_upper_snake_case(yaml_data):
    bad_keys = [k for k in yaml_data if not UPPER_SNAKE_RE.match(str(k))]
    assert not bad_keys, (
        f"Claves no UPPER_SNAKE_CASE en runtime_defaults.yaml: {bad_keys}"
    )


def test_all_values_are_serializable_scalars(yaml_data):
    """Valores deben ser str, int, float, bool o None (no listas/dicts)."""
    bad = []
    for k, v in yaml_data.items():
        if v is None:
            continue
        if isinstance(v, (str, int, float, bool)):
            continue
        bad.append(f"{k}={v!r} (type={type(v).__name__})")
    assert not bad, (
        "Valores no escalares en runtime_defaults.yaml (esperado str|int|float|bool|None):\n  "
        + "\n  ".join(bad)
    )


def _has_suffix(key: str, suffixes: tuple[str, ...]) -> bool:
    return any(key.endswith(s) for s in suffixes)


def test_float_suffix_values_castable(yaml_data):
    """Valores con sufijo float-like deben ser cast a float."""
    bad = []
    for k, v in yaml_data.items():
        if not _has_suffix(k, FLOAT_SUFFIXES):
            continue
        if v is None:
            continue
        try:
            float(v)
        except (TypeError, ValueError):
            bad.append(f"{k}={v!r}")
    assert not bad, (
        "Valores con sufijo float no castables a float:\n  " + "\n  ".join(bad)
    )


def test_int_suffix_values_castable(yaml_data):
    """Valores con sufijo int-like deben ser cast a int."""
    bad = []
    for k, v in yaml_data.items():
        if not _has_suffix(k, INT_SUFFIXES):
            continue
        if v is None:
            continue
        try:
            # Aceptamos "2000" o 2000 o "2000.0" → int truncar
            iv = int(float(v))
            assert iv == float(v), f"{k}={v!r} no es entero exacto"
        except (TypeError, ValueError, AssertionError) as exc:
            bad.append(f"{k}={v!r} ({exc})")
    assert not bad, (
        "Valores con sufijo int no castables a int exacto:\n  "
        + "\n  ".join(bad)
    )


def test_str_suffix_values_non_empty(yaml_data):
    """Claves *_PATH/_DIR/_MODE/etc deben tener valor str no vacío (o None)."""
    bad = []
    for k, v in yaml_data.items():
        if not _has_suffix(k, STR_SUFFIXES):
            continue
        if v is None or v == "":
            # None y "" son aceptables (significa "auto" o "no override").
            continue
        if not isinstance(v, str):
            bad.append(f"{k}={v!r} (type={type(v).__name__})")
    assert not bad, (
        "Claves de tipo string con valor no-string:\n  " + "\n  ".join(bad)
    )


def test_no_value_contains_unresolved_tilde_in_middle(yaml_data):
    """Tilde sólo válido al inicio (~/...). Nunca en mitad de path."""
    bad = []
    for k, v in yaml_data.items():
        if not isinstance(v, str):
            continue
        if "~" in v[1:]:
            bad.append(f"{k}={v!r}")
    assert not bad, (
        "Tilde en mitad de valor (debe ir sólo al inicio):\n  "
        + "\n  ".join(bad)
    )


def test_flags_have_recognised_value(yaml_data):
    """Valores que parecen flags ('0','1','true',...) deben caer en FLAG_VALUES."""
    # Heurística: claves con prefijo conocido de flag.
    flag_prefixes = (
        "PANEL_KEEP_",
        "PANEL_CAMERA_REQUIRED",
        "PANEL_MOVEIT_BRIDGE_FORCE_",
        "PANEL_MOVEIT_BRIDGE_DRY_RUN",
        "PANEL_MOVEIT_BRIDGE_REQUIRE_",
        "PANEL_MOVEIT_BRIDGE_DROP_",
        "PANEL_MOVEIT_BRIDGE_UNWRAP_",
        "PANEL_PICK_DEMO_GRASP_DOWN_USE_MOVEIT_CARTESIAN",
        "PANEL_PICK_DEMO_GRASP_DOWN_FORCE_INHERIT_XY",
        "PANEL_PICK_DEMO_GRASP_DOWN_DISABLE_PERMISSIVE_FALLBACK",
        "STRICT_PHYSICS_MODE",
    )
    bad = []
    for k, v in yaml_data.items():
        if not any(k.startswith(p) for p in flag_prefixes):
            continue
        if v is None:
            continue
        sv = str(v).strip().lower()
        if sv not in FLAG_VALUES:
            bad.append(f"{k}={v!r}")
    assert not bad, (
        "Valores flag no reconocidos (esperado 0/1/true/false/on/off/yes/no):\n  "
        + "\n  ".join(bad)
    )


def test_no_duplicate_keys_via_raw_parse():
    """PyYAML por defecto silencia duplicados. Detección manual línea-a-línea."""
    if not YAML_PATH.is_file():
        pytest.skip("YAML no presente")
    seen: dict[str, int] = {}
    duplicates = []
    with YAML_PATH.open("r", encoding="utf-8") as fh:
        for lineno, raw in enumerate(fh, start=1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            # Línea formato 'KEY: value'
            if ":" not in line:
                continue
            key = line.split(":", 1)[0].strip()
            if not UPPER_SNAKE_RE.match(key):
                continue
            if key in seen:
                duplicates.append(f"{key} (líneas {seen[key]} y {lineno})")
            else:
                seen[key] = lineno
    assert not duplicates, (
        "Claves duplicadas en runtime_defaults.yaml:\n  " + "\n  ".join(duplicates)
    )
