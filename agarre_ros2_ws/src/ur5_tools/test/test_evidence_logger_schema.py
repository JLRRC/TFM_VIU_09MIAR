#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/test/test_evidence_logger_schema.py
# Contenido: T12 — schema validation de events.jsonl + metrics.json del evidence_logger.
"""T12 — Schema validation del evidence_logger.

El evidence_logger escribe ``events.jsonl`` (JSON Lines, 1 evento/línea)
y, al cierre, ``metrics.json`` con agregados. Estos tests verifican que:

1. El esquema de eventos (campos obligatorios + tipos) está bien definido
   y se aplica consistentemente.
2. Eventos malformados son detectados.
3. Si existen ejemplos en ``historico/`` o ``report/runs/``, todos parsean
   contra el schema.
4. ``metrics.json`` (cuando existe) tiene los campos agregados esperados.

Pure stdlib (typing + json + pathlib). No requiere pydantic ni jsonschema.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

import pytest


WS_ROOT = Path(__file__).resolve().parents[3]
REPO_ROOT = WS_ROOT.parent  # /home/laboratorio/TFM


# ---------------------------------------------------------------------------
# Schema definition (events.jsonl)
# ---------------------------------------------------------------------------

# Campos obligatorios + tipos esperados por evento.
# El evidence_logger escribe estos campos con `json.dumps(entry, ensure_ascii=True)`.
_EVENT_REQUIRED_FIELDS: Dict[str, type] = {
    "ts_iso": str,         # ISO 8601 timestamp (now_iso())
    "ts_mono": (int, float),  # monotonic seconds (float)
    "kind": str,           # event kind: "session_start", "grasp_result",
                           # "system_state", "system_diag", "gripper_attach",
                           # "gripper_detach", "session_end", etc.
    "data": dict,          # payload (kind-specific structure)
}

# `ts_sim_ns` es opcional (sólo si use_sim_time=true).
_EVENT_OPTIONAL_FIELDS: Dict[str, type] = {
    "ts_sim_ns": (int, type(None)),
}


def _validate_event(entry: Any) -> List[str]:
    """Devuelve lista de errores; vacía si entry es válido."""
    errors: List[str] = []
    if not isinstance(entry, dict):
        return [f"entry not a dict: {type(entry).__name__}"]
    for field, expected_type in _EVENT_REQUIRED_FIELDS.items():
        if field not in entry:
            errors.append(f"missing required field: {field!r}")
            continue
        value = entry[field]
        if not isinstance(value, expected_type):  # type: ignore[arg-type]
            errors.append(
                f"field {field!r} type mismatch: "
                f"expected {expected_type}, got {type(value).__name__}"
            )
    for field, expected_type in _EVENT_OPTIONAL_FIELDS.items():
        if field in entry and not isinstance(entry[field], expected_type):  # type: ignore[arg-type]
            errors.append(
                f"optional field {field!r} type mismatch: "
                f"expected {expected_type}, got {type(entry[field]).__name__}"
            )
    return errors


def _validate_event_kind_data(entry: Dict[str, Any]) -> List[str]:
    """Validación adicional kind-specific de `data`. Devuelve errores."""
    errors: List[str] = []
    kind = entry.get("kind")
    data = entry.get("data", {})
    if not isinstance(data, dict):
        return errors  # _validate_event ya lo capturó
    if kind == "session_start":
        for f in ("session_dir", "events_path"):
            if f not in data:
                errors.append(f"session_start.data missing field {f!r}")
    elif kind == "grasp_result":
        for f in ("text",):
            if f not in data:
                errors.append(f"grasp_result.data missing field {f!r}")
    elif kind in ("gripper_attach", "gripper_detach"):
        for f in ("object", "attached"):
            if f not in data:
                errors.append(f"{kind}.data missing field {f!r}")
    return errors


# ---------------------------------------------------------------------------
# T12-a — Schema accepts well-formed event
# ---------------------------------------------------------------------------


def test_t12a_valid_session_start_event_passes() -> None:
    """T12-a — un session_start event con todos los campos pasa."""
    entry = {
        "ts_iso": "2026-05-08T00:00:00.000Z",
        "ts_mono": 1234.567,
        "ts_sim_ns": 1000000000,
        "kind": "session_start",
        "data": {
            "session_dir": "/tmp/runs/20260508",
            "events_path": "/tmp/runs/20260508/events.jsonl",
            "summary_path": "/tmp/runs/20260508/summary.csv",
        },
    }
    assert _validate_event(entry) == []
    assert _validate_event_kind_data(entry) == []


def test_t12b_valid_grasp_result_event_passes() -> None:
    """T12-b — un grasp_result event válido pasa."""
    entry = {
        "ts_iso": "2026-05-08T00:00:01.000Z",
        "ts_mono": 1235.0,
        "kind": "grasp_result",
        "data": {"text": "success reason=approach:moveit:SUCCESS"},
    }
    assert _validate_event(entry) == []
    assert _validate_event_kind_data(entry) == []


def test_t12c_valid_gripper_attach_event_passes() -> None:
    """T12-c — un gripper_attach event válido pasa."""
    entry = {
        "ts_iso": "2026-05-08T00:00:02.000Z",
        "ts_mono": 1236.0,
        "kind": "gripper_attach",
        "data": {"object": "box_red", "attached": True},
    }
    assert _validate_event(entry) == []
    assert _validate_event_kind_data(entry) == []


# ---------------------------------------------------------------------------
# T12-d — Schema rejects malformed events
# ---------------------------------------------------------------------------


def test_t12d_missing_required_field_rejected() -> None:
    """T12-d — falta un campo obligatorio → error."""
    entry = {
        "ts_iso": "2026-05-08T00:00:00.000Z",
        "ts_mono": 1234.0,
        # kind missing
        "data": {},
    }
    errors = _validate_event(entry)
    assert any("kind" in e for e in errors), errors


def test_t12e_wrong_type_rejected() -> None:
    """T12-e — tipo incorrecto en campo obligatorio → error."""
    entry = {
        "ts_iso": "2026-05-08T00:00:00.000Z",
        "ts_mono": "not-a-number",  # debería ser int|float
        "kind": "test",
        "data": {},
    }
    errors = _validate_event(entry)
    assert any("ts_mono" in e for e in errors), errors


def test_t12f_data_not_dict_rejected() -> None:
    """T12-f — data no es dict → error."""
    entry = {
        "ts_iso": "2026-05-08T00:00:00.000Z",
        "ts_mono": 1234.0,
        "kind": "test",
        "data": "should-be-dict",
    }
    errors = _validate_event(entry)
    assert any("data" in e for e in errors), errors


def test_t12g_session_start_missing_session_dir() -> None:
    """T12-g — session_start sin session_dir → error kind-specific."""
    entry = {
        "ts_iso": "2026-05-08T00:00:00.000Z",
        "ts_mono": 1234.0,
        "kind": "session_start",
        "data": {"events_path": "/tmp/x.jsonl"},  # falta session_dir
    }
    base_errors = _validate_event(entry)
    assert base_errors == []  # estructura general válida
    kind_errors = _validate_event_kind_data(entry)
    assert any("session_dir" in e for e in kind_errors), kind_errors


# ---------------------------------------------------------------------------
# T12-h — Si hay ejemplos en historico/, todos validan
# ---------------------------------------------------------------------------


def _collect_existing_event_files() -> List[Path]:
    """Devuelve archivos events.jsonl existentes en historico/ o report/runs/."""
    files: List[Path] = []
    for base in (REPO_ROOT / "historico", REPO_ROOT / "report" / "runs"):
        if not base.exists():
            continue
        files.extend(base.rglob("events.jsonl"))
    return files


def test_t12h_all_existing_events_files_validate() -> None:
    """T12-h — si hay events.jsonl previos, todos pasan el schema.

    No falla si no hay archivos (proyecto reciente). Falla si encuentra
    eventos malformados (contrato roto).
    """
    files = _collect_existing_event_files()
    if not files:
        pytest.skip("no events.jsonl files found yet (project pre-evidence)")
    bad: List[str] = []
    for path in files:
        with path.open(encoding="utf-8") as fp:
            for lineno, raw in enumerate(fp, start=1):
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    entry = json.loads(raw)
                except json.JSONDecodeError as exc:
                    bad.append(f"{path}:{lineno}: invalid JSON: {exc}")
                    continue
                errors = _validate_event(entry)
                if errors:
                    bad.append(f"{path}:{lineno}: {errors}")
    assert not bad, "\n".join(bad[:20])


# ---------------------------------------------------------------------------
# T12-i — metrics.json schema (si existe)
# ---------------------------------------------------------------------------


_METRICS_REQUIRED_FIELDS: Dict[str, type] = {
    "session_start_ts_iso": str,
    "session_end_ts_iso": (str, type(None)),
    "events_total": int,
    "kinds": dict,
}


def test_t12i_metrics_schema_when_present() -> None:
    """T12-i — si existe metrics.json en algún histórico, valida campos agregados."""
    files: List[Path] = []
    for base in (REPO_ROOT / "historico", REPO_ROOT / "report" / "runs"):
        if not base.exists():
            continue
        files.extend(base.rglob("metrics.json"))
    if not files:
        pytest.skip("no metrics.json files found yet")
    bad: List[str] = []
    for path in files:
        try:
            metrics = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            bad.append(f"{path}: invalid JSON: {exc}")
            continue
        if not isinstance(metrics, dict):
            bad.append(f"{path}: top-level not a dict")
            continue
        for field, expected in _METRICS_REQUIRED_FIELDS.items():
            if field not in metrics:
                bad.append(f"{path}: missing field {field!r}")
                continue
            if not isinstance(metrics[field], expected):  # type: ignore[arg-type]
                bad.append(
                    f"{path}: field {field!r} type mismatch: "
                    f"expected {expected}, got {type(metrics[field]).__name__}"
                )
    assert not bad, "\n".join(bad[:10])
