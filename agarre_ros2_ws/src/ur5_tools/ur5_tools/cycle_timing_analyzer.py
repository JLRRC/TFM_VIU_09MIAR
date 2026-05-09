#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/cycle_timing_analyzer.py
"""F8 / #20 (2026-05-08) — Analizador de timing de cycles E2E.

Parsea ``log/ros2_launch.log`` y reporta duración de cada fase del
pick demo. Utilizable para identificar bottlenecks (TRANSPORT lento,
APPROACH flaky, etc.) sin necesidad de stack vivo.

Markers reconocidos:
- ``[ORCHESTRATOR_LC][<PHASE>] start``  → inicio de fase.
- ``[ORCHESTRATOR_LC][<PHASE>] end success=...`` → fin de fase.
- ``SECUENCIA COMPLETADA EXITOSAMENTE`` → fin de cycle (legacy).
- ``[ORCHESTRATOR_LC] result success=True`` → fin de cycle (orchestrator).

Sin ROS. Sin live. Pure parser. Tests offline.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Tuple

# Timestamp de logs de ROS 2 launch: [INFO] [TIMESTAMP_S] [...] msg
# Soportamos dos formatos:
#   1. ``[INFO] [1715175123.456789] [node]: msg``  (ROS 2 logging)
#   2. ``2026-05-08 13:45:23.456 [INFO] msg``       (datetime stamped)
# El TS de ROS 2 es siempre `[NUM.NUM]`. Capturamos el primer `[<float>]`
# que aparezca en la línea (ignora `[INFO]`/`[WARN]` que no contienen ".").
_TS_PATTERN_NS = re.compile(r"\[(\d+\.\d+)\]")
_TS_PATTERN_DATETIME = re.compile(
    r"^(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}(?:\.\d+)?)"
)

_PHASE_START_PATTERN = re.compile(
    r"\[ORCHESTRATOR_LC\]\[([A-Z_]+)\]\s+start"
)
_PHASE_END_PATTERN = re.compile(
    r"\[ORCHESTRATOR_LC\]\[([A-Z_]+)\]\s+end\s+success=(\w+)"
)
_CYCLE_PASS_PATTERN = re.compile(
    r"SECUENCIA COMPLETADA EXITOSAMENTE"
    r"|\[ORCHESTRATOR_LC\] result success=True"
    r"|\[PICK_DEMO\]\[ORCH\]\[DONE\] success=true"
)


@dataclass(frozen=True)
class PhaseTiming:
    """Duración de una fase pick."""

    phase: str
    start_sec: float
    end_sec: Optional[float]
    duration_sec: Optional[float]
    success: Optional[bool]


@dataclass(frozen=True)
class CycleTiming:
    """Resumen de un cycle completo."""

    phases: List[PhaseTiming]
    total_duration_sec: Optional[float]
    success: bool
    n_phases: int


def _extract_timestamp_sec(line: str) -> Optional[float]:
    """Devuelve el timestamp de la línea en segundos, o None."""
    m = _TS_PATTERN_NS.search(line)
    if m:
        try:
            return float(m.group(1))
        except (TypeError, ValueError):
            return None
    m2 = _TS_PATTERN_DATETIME.search(line)
    if m2:
        try:
            ts_str = m2.group(1)
            for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S"):
                try:
                    dt = datetime.strptime(ts_str, fmt)
                    return dt.timestamp()
                except ValueError:
                    continue
        except (TypeError, ValueError):
            return None
    return None


def parse_phase_events(
    log_lines: List[str],
) -> Tuple[Dict[str, float], Dict[str, Tuple[float, Optional[bool]]]]:
    """Extrae eventos de inicio y fin de fases del log.

    Returns:
        (starts, ends) donde:
        - starts: dict[phase] → timestamp_sec del start.
        - ends: dict[phase] → (timestamp_sec, success).
    """
    starts: Dict[str, float] = {}
    ends: Dict[str, Tuple[float, Optional[bool]]] = {}
    for line in log_lines:
        ts = _extract_timestamp_sec(line)
        if ts is None:
            continue
        m_start = _PHASE_START_PATTERN.search(line)
        if m_start:
            phase = m_start.group(1)
            starts.setdefault(phase, ts)
            continue
        m_end = _PHASE_END_PATTERN.search(line)
        if m_end:
            phase = m_end.group(1)
            success_token = m_end.group(2).strip().lower()
            success: Optional[bool]
            if success_token in ("true", "yes", "ok"):
                success = True
            elif success_token in ("false", "no"):
                success = False
            else:
                success = None
            ends[phase] = (ts, success)
    return starts, ends


def analyze_cycle(log_lines: List[str]) -> CycleTiming:
    """Analiza el log y devuelve resumen del cycle.

    Si no se detectan markers de fase, devuelve CycleTiming vacío.
    """
    starts, ends = parse_phase_events(log_lines)

    # Detectar PASS global del cycle.
    cycle_passed = any(_CYCLE_PASS_PATTERN.search(line) for line in log_lines)

    phases: List[PhaseTiming] = []
    for phase, start_sec in starts.items():
        end_info = ends.get(phase)
        end_sec = end_info[0] if end_info else None
        success = end_info[1] if end_info else None
        duration = (
            float(end_sec) - float(start_sec)
            if end_sec is not None
            else None
        )
        phases.append(PhaseTiming(
            phase=phase,
            start_sec=float(start_sec),
            end_sec=end_sec,
            duration_sec=duration,
            success=success,
        ))

    # Total = max(end) - min(start) si hay datos.
    total: Optional[float] = None
    if phases:
        starts_collected = [p.start_sec for p in phases]
        ends_collected = [p.end_sec for p in phases if p.end_sec is not None]
        if starts_collected and ends_collected:
            total = max(ends_collected) - min(starts_collected)

    return CycleTiming(
        phases=sorted(phases, key=lambda p: p.start_sec),
        total_duration_sec=total,
        success=cycle_passed,
        n_phases=len(phases),
    )


def format_cycle_summary(cycle: CycleTiming) -> str:
    """Formatea un resumen legible del cycle."""
    if cycle.n_phases == 0:
        return "no phases detected in log"
    lines: List[str] = []
    lines.append(f"cycle: success={cycle.success} n_phases={cycle.n_phases}")
    if cycle.total_duration_sec is not None:
        lines.append(f"total_duration={cycle.total_duration_sec:.1f}s")
    for p in cycle.phases:
        if p.duration_sec is None:
            lines.append(f"  {p.phase:>20s}: incomplete (no end marker)")
        else:
            ok = "OK" if p.success else ("FAIL" if p.success is False else "?")
            lines.append(
                f"  {p.phase:>20s}: {p.duration_sec:6.1f}s [{ok}]"
            )
    return "\n".join(lines)
