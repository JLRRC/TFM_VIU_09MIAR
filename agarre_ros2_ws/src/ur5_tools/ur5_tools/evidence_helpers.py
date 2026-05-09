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
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple


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
    seen_objects: Set[str] = set()
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


# ---------------------------------------------------------------------------
# F18 — Telemetría/observabilidad extendida
# ---------------------------------------------------------------------------


def compute_inter_event_latencies(
    events: Iterable[Dict[str, Any]],
    *,
    from_kind: str,
    to_kind: str,
) -> Dict[str, Any]:
    """Latencias entre eventos ``from_kind`` → siguiente ``to_kind``.

    Recorre los eventos en orden y, cada vez que aparece ``from_kind``,
    busca el siguiente ``to_kind`` y calcula la diferencia en
    ``ts_mono``. Devuelve un agregado con:

    * ``samples`` — número de pares matcheados.
    * ``mean_sec`` / ``min_sec`` / ``max_sec`` / ``p95_sec`` — sobre las
      diferencias en segundos. ``None`` si no hubo muestras.
    * ``unmatched_from`` — eventos ``from_kind`` que no encontraron
      pareja (cierre/abandono).

    Útil para medir, p.ej., latencia entre ``system_state=READY`` y el
    primer ``grasp_result`` del ciclo, o entre ``gripper_state`` y el
    siguiente snapshot ``phase_timings``.
    """
    diffs: List[float] = []
    pending_ts: Optional[float] = None
    unmatched_from = 0
    for ev in events:
        if not isinstance(ev, dict):
            continue
        kind = str(ev.get("kind") or "")
        ts = ev.get("ts_mono")
        if not isinstance(ts, (int, float)):
            continue
        ts_f = float(ts)
        if kind == from_kind:
            if pending_ts is not None:
                # un from previo nunca encontró su to → contar como unmatched
                unmatched_from += 1
            pending_ts = ts_f
        elif kind == to_kind and pending_ts is not None:
            delta = ts_f - pending_ts
            if delta >= 0.0:
                diffs.append(delta)
            pending_ts = None

    if pending_ts is not None:
        unmatched_from += 1

    if not diffs:
        return {
            "samples": 0,
            "mean_sec": None,
            "min_sec": None,
            "max_sec": None,
            "p95_sec": None,
            "unmatched_from": unmatched_from,
            "from_kind": from_kind,
            "to_kind": to_kind,
        }

    sorted_diffs = sorted(diffs)
    p95_idx = max(0, int(round(0.95 * (len(sorted_diffs) - 1))))
    return {
        "samples": len(diffs),
        "mean_sec": sum(diffs) / len(diffs),
        "min_sec": min(diffs),
        "max_sec": max(diffs),
        "p95_sec": sorted_diffs[p95_idx],
        "unmatched_from": unmatched_from,
        "from_kind": from_kind,
        "to_kind": to_kind,
    }


def compute_event_rates(
    events: Iterable[Dict[str, Any]],
) -> Dict[str, Dict[str, Any]]:
    """Tasa global (eventos/segundo) por ``kind`` en la sesión completa.

    Para cada kind devuelve un dict con:

    * ``count`` — total de eventos de ese kind.
    * ``rate_hz`` — count / span_sec, donde ``span_sec`` es la
      diferencia entre el primer y último ``ts_mono`` observado para
      ese kind. ``None`` si no hubo muestras o span ≤ 0.
    * ``span_sec`` — duración cubierta por los eventos del kind.

    Útil para ver, p.ej., a qué frecuencia real llega ``system_diag``
    o cuántos ``grasp_result`` por minuto produce el bridge.
    """
    per_kind: Dict[str, Dict[str, Any]] = {}
    monos_by_kind: Dict[str, List[float]] = {}
    for ev in events:
        if not isinstance(ev, dict):
            continue
        kind = str(ev.get("kind") or "")
        if not kind:
            continue
        monos_by_kind.setdefault(kind, [])
        ts = ev.get("ts_mono")
        # Aceptamos ts >= 0 (ts=0 es origen monotónico válido). Solo
        # rechazamos None / no-numérico.
        if isinstance(ts, (int, float)) and float(ts) >= 0.0:
            monos_by_kind[kind].append(float(ts))

    for kind, monos in monos_by_kind.items():
        count = len(monos)
        span: Optional[float]
        rate: Optional[float]
        if count >= 2:
            span = max(monos) - min(monos)
            rate = (count / span) if span > 0 else None
        else:
            span = 0.0 if count <= 1 else None
            rate = None
        per_kind[kind] = {
            "count": count,
            "rate_hz": rate,
            "span_sec": span,
        }
    return per_kind


def generate_latency_report_md(
    *,
    metrics: Dict[str, Any],
    title: str = "Reporte de telemetría F18",
) -> str:
    """Genera un reporte Markdown a partir del ``metrics.json`` extendido.

    El dict ``metrics`` debe contener al menos las claves producidas por
    ``compute_session_metrics`` y, opcionalmente, los nuevos campos
    F18: ``event_rates`` (de ``compute_event_rates``) e
    ``inter_event_latencies`` (lista de dicts de
    ``compute_inter_event_latencies``).

    Devuelve un string Markdown listo para incrustar en la memoria
    académica o en un README de evidencias.
    """
    lines: List[str] = [
        f"# {title}",
        "",
        "## Resumen de sesión",
        "",
        f"- Total de eventos: **{metrics.get('total_events', 0)}**",
        f"- Duración: **{_fmt_sec(metrics.get('duration_sec'))}**",
        f"- Grasp success: **{metrics.get('grasp_success', 0)}**",
        f"- Grasp failure: **{metrics.get('grasp_failure', 0)}**",
    ]
    success_rate = metrics.get("grasp_success_rate")
    if success_rate is not None:
        lines.append(f"- Tasa de éxito: **{success_rate * 100:.1f}%**")
    else:
        lines.append("- Tasa de éxito: **n/d**")
    lines.append("")

    by_kind = metrics.get("by_kind") or {}
    if by_kind:
        lines.append("## Conteo por tipo de evento")
        lines.append("")
        lines.append("| Kind | Count |")
        lines.append("|---|---:|")
        for kind in sorted(by_kind.keys()):
            lines.append(f"| `{kind}` | {by_kind[kind]} |")
        lines.append("")

    rates = metrics.get("event_rates") or {}
    if rates:
        lines.append("## Tasa por kind (eventos/segundo)")
        lines.append("")
        lines.append("| Kind | Count | Span (s) | Rate (Hz) |")
        lines.append("|---|---:|---:|---:|")
        for kind in sorted(rates.keys()):
            entry = rates[kind] or {}
            count = entry.get("count", 0)
            span = entry.get("span_sec")
            rate = entry.get("rate_hz")
            lines.append(
                f"| `{kind}` | {count} | {_fmt_sec(span)} | "
                f"{_fmt_rate(rate)} |"
            )
        lines.append("")

    latencies = metrics.get("inter_event_latencies") or []
    if latencies:
        lines.append("## Latencias entre eventos")
        lines.append("")
        lines.append(
            "| From → To | Samples | Mean (s) | Min (s) | Max (s) | "
            "P95 (s) |"
        )
        lines.append("|---|---:|---:|---:|---:|---:|")
        for entry in latencies:
            from_k = entry.get("from_kind", "?")
            to_k = entry.get("to_kind", "?")
            lines.append(
                f"| `{from_k}` → `{to_k}` | {entry.get('samples', 0)} | "
                f"{_fmt_sec(entry.get('mean_sec'))} | "
                f"{_fmt_sec(entry.get('min_sec'))} | "
                f"{_fmt_sec(entry.get('max_sec'))} | "
                f"{_fmt_sec(entry.get('p95_sec'))} |"
            )
        lines.append("")

    return "\n".join(lines)


def _fmt_sec(value: Any) -> str:
    if value is None:
        return "n/d"
    try:
        return f"{float(value):.3f}"
    except (TypeError, ValueError):
        return "n/d"


def _fmt_rate(value: Any) -> str:
    if value is None:
        return "n/d"
    try:
        return f"{float(value):.2f}"
    except (TypeError, ValueError):
        return "n/d"
