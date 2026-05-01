#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/evidence_helpers.py
# Contenido: F4/F10 — helpers puros del evidence_logger (sin rclpy).
"""Helpers puros del evidence_logger.

Extraído de ``evidence_logger.py`` para que sean importables sin
rclpy. El propio ``evidence_logger`` re-exporta las funciones para no
romper consumidores.

Funciones públicas:

* ``now_iso()`` — timestamp ISO 8601 UTC con microsegundos.
* ``safe_unique_dir(root)`` — crea ``root/<timestamp>`` único.
* ``parse_grasp_result(text)`` — parsea ``success=true reason=...`` del
  bridge MoveIt y devuelve ``(success, reason)``.
* ``compute_session_metrics(events)`` — agrega métricas de una sesión
  a partir de la lista de eventos JSONL.
"""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


def now_iso() -> str:
    """Timestamp ISO 8601 UTC con microsegundos, sufijo ``Z``."""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def safe_unique_dir(root: Path) -> Path:
    """Crea un directorio nuevo bajo ``root`` con timestamp + sufijo si colisiona."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    candidate = root / stamp
    suffix = 1
    while candidate.exists():
        candidate = root / f"{stamp}_{suffix}"
        suffix += 1
    candidate.mkdir(parents=True, exist_ok=False)
    return candidate


def parse_grasp_result(text: str) -> Tuple[Optional[bool], str]:
    """Extrae ``(success, reason)`` del texto del resultado del bridge MoveIt.

    El bridge emite cadenas tipo ``success=true reason=exec_ok ...`` o
    ``success=false reason=plan_failed:... request_uuid=...``. Si no se
    puede parsear devuelve ``(None, text)``.

    >>> parse_grasp_result("success=true reason=exec_ok foo=bar")
    (True, 'exec_ok')
    >>> parse_grasp_result("success=false reason=plan_failed")
    (False, 'plan_failed')
    >>> parse_grasp_result("")
    (None, '')
    """
    if not text:
        return None, ""
    success: Optional[bool] = None
    reason = text
    try:
        kv: Dict[str, str] = {}
        for tok in text.split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                kv[k] = v
        if "success" in kv:
            s = kv["success"].strip().lower()
            if s in ("true", "1", "yes"):
                success = True
            elif s in ("false", "0", "no"):
                success = False
        if "reason" in kv:
            reason = kv["reason"]
    except Exception:
        pass
    return success, reason


def compute_session_metrics(events: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    """Agrega métricas de una sesión a partir de los eventos JSONL.

    Recibe una iterable de dicts (eventos JSON Lines decodificados).
    Devuelve un dict con:

    * ``total_events``: número total de líneas procesadas.
    * ``by_kind``: conteo por tipo de evento.
    * ``grasp_success``, ``grasp_failure``, ``grasp_unknown``: conteo
      de resultados de grasp parseados.
    * ``grasp_success_rate``: ratio float [0,1] o None si no hubo
      resultados clasificados.
    * ``attach_count``, ``detach_count``: por evento gripper_state.
    * ``objects_attached``: lista de nombres únicos.
    * ``session_started_iso``, ``session_finished_iso``: timestamps si
      están presentes.
    * ``duration_sec``: duración monotónica (último ts_mono - primero)
      o None si no hay datos.
    """
    total = 0
    by_kind: Dict[str, int] = {}
    grasp_success = 0
    grasp_failure = 0
    grasp_unknown = 0
    attach_count = 0
    detach_count = 0
    objects_attached: List[str] = []
    seen_objects: set = set()
    session_started_iso: Optional[str] = None
    session_finished_iso: Optional[str] = None
    monos: List[float] = []

    for ev in events:
        if not isinstance(ev, dict):
            continue
        total += 1
        kind = str(ev.get("kind") or "")
        by_kind[kind] = by_kind.get(kind, 0) + 1
        try:
            mono = ev.get("ts_mono")
            if isinstance(mono, (int, float)) and float(mono) > 0:
                monos.append(float(mono))
        except Exception:
            pass

        if kind == "session_started":
            session_started_iso = str(ev.get("ts_iso") or "") or None
        elif kind == "session_finished":
            session_finished_iso = str(ev.get("ts_iso") or "") or None
        elif kind == "grasp_result":
            data = ev.get("data") or {}
            ok = data.get("success") if isinstance(data, dict) else None
            if ok is True:
                grasp_success += 1
            elif ok is False:
                grasp_failure += 1
            else:
                grasp_unknown += 1
        elif kind == "gripper_state":
            data = ev.get("data") or {}
            if isinstance(data, dict):
                obj = str(data.get("object") or "")
                attached = bool(data.get("attached"))
                if attached:
                    attach_count += 1
                    if obj and obj not in seen_objects:
                        seen_objects.add(obj)
                        objects_attached.append(obj)
                else:
                    detach_count += 1

    classified = grasp_success + grasp_failure
    success_rate: Optional[float] = (
        grasp_success / classified if classified > 0 else None
    )
    duration_sec: Optional[float] = (
        (max(monos) - min(monos)) if len(monos) >= 2 else None
    )

    return {
        "total_events": total,
        "by_kind": by_kind,
        "grasp_success": grasp_success,
        "grasp_failure": grasp_failure,
        "grasp_unknown": grasp_unknown,
        "grasp_success_rate": success_rate,
        "attach_count": attach_count,
        "detach_count": detach_count,
        "objects_attached": objects_attached,
        "session_started_iso": session_started_iso,
        "session_finished_iso": session_finished_iso,
        "duration_sec": duration_sec,
    }


# ---------------------------------------------------------------------------
# F8 — Phase timing aggregation
# ---------------------------------------------------------------------------


def aggregate_phase_timings(
    snapshots: Iterable[Dict[str, Any]],
) -> Dict[str, Any]:
    """Agrega múltiples snapshots de ``PhaseTimings`` para reporte.

    Recibe una iterable de dicts con la estructura producida por
    ``PhaseTimings.snapshot()`` (puede venir de un evento JSONL del
    evidence_logger con ``kind=phase_timings``).

    Devuelve un dict con métricas agregadas por fase:
      * ``per_phase``: dict {phase_name → {samples, mean_sec, min_sec,
        max_sec, success_rate}}
      * ``total_sessions``: número de snapshots procesados
      * ``successful_sessions``: snapshots sin fases fallidas
      * ``avg_total_duration_sec``: media de total_duration_sec en
        snapshots con valor.
    """
    per_phase: Dict[str, Dict[str, Any]] = {}
    total_sessions = 0
    successful_sessions = 0
    total_durations: List[float] = []

    for snap in snapshots:
        if not isinstance(snap, dict):
            continue
        total_sessions += 1
        if int(snap.get("phases_failed") or 0) == 0:
            successful_sessions += 1
        td = snap.get("total_duration_sec")
        if isinstance(td, (int, float)):
            total_durations.append(float(td))

        phases = snap.get("phases") or {}
        if not isinstance(phases, dict):
            continue
        for name, entry in phases.items():
            if not isinstance(entry, dict):
                continue
            duration = entry.get("duration_sec")
            success = entry.get("success")
            stats = per_phase.setdefault(
                str(name),
                {
                    "samples": 0,
                    "successes": 0,
                    "failures": 0,
                    "_durations": [],
                },
            )
            stats["samples"] += 1
            if success is True:
                stats["successes"] += 1
            elif success is False:
                stats["failures"] += 1
            if isinstance(duration, (int, float)):
                stats["_durations"].append(float(duration))

    # Finalizar agregaciones por fase
    finalized_per_phase: Dict[str, Dict[str, Any]] = {}
    for name, stats in per_phase.items():
        durations = stats.pop("_durations")
        classified = stats["successes"] + stats["failures"]
        success_rate = (
            stats["successes"] / classified if classified > 0 else None
        )
        finalized_per_phase[name] = {
            "samples": stats["samples"],
            "successes": stats["successes"],
            "failures": stats["failures"],
            "success_rate": success_rate,
            "mean_sec": (
                sum(durations) / len(durations) if durations else None
            ),
            "min_sec": min(durations) if durations else None,
            "max_sec": max(durations) if durations else None,
        }

    avg_total_duration_sec: Optional[float] = (
        sum(total_durations) / len(total_durations) if total_durations else None
    )

    return {
        "per_phase": finalized_per_phase,
        "total_sessions": total_sessions,
        "successful_sessions": successful_sessions,
        "avg_total_duration_sec": avg_total_duration_sec,
    }
