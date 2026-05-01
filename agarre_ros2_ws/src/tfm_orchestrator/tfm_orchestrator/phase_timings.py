#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/phase_timings.py
# Contenido: F8 — instrumentación de latencias por fase del orchestrator.
"""Instrumentación pura de latencias por fase.

Clase ``PhaseTimings`` que mantiene timestamps monotónicos start/end
por cada fase del FSM (PickPhase). Cero dependencia ROS — se puede
testear en pytest puro y se integra en ``pick_orchestrator_node`` con
``time.monotonic`` como reloj inyectado.

Uso típico:

    timings = PhaseTimings()
    timings.mark_start("APPROACH", clock_now=time.monotonic())
    ... ejecutar fase ...
    timings.mark_end("APPROACH", clock_now=time.monotonic(), success=True)

    snapshot = timings.snapshot()
    # → {"phases": {"APPROACH": {"start_mono": ..., "end_mono": ...,
    #                            "duration_sec": ..., "success": True}},
    #    "total_duration_sec": ...}

Diseño:

* No usa ``time`` directamente — el reloj se pasa siempre como argumento
  para tests deterministas.
* Una fase puede registrarse varias veces si se reintenta — sólo el
  último start/end se mantiene (consistente con el flujo del orchestrator).
* ``snapshot()`` devuelve datos JSON-serializables para integrar en
  evidence_logger.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class _PhaseEntry:
    start_mono: Optional[float] = None
    end_mono: Optional[float] = None
    success: Optional[bool] = None
    detail: str = ""

    def duration_sec(self) -> Optional[float]:
        if self.start_mono is None or self.end_mono is None:
            return None
        return max(0.0, float(self.end_mono) - float(self.start_mono))

    def to_dict(self) -> Dict[str, object]:
        return {
            "start_mono": self.start_mono,
            "end_mono": self.end_mono,
            "duration_sec": self.duration_sec(),
            "success": self.success,
            "detail": self.detail,
        }


@dataclass
class PhaseTimings:
    """Almacena timestamps monotónicos start/end por fase."""

    _phases: Dict[str, _PhaseEntry] = field(default_factory=dict)
    _order: List[str] = field(default_factory=list)
    _session_start_mono: Optional[float] = None
    _session_end_mono: Optional[float] = None

    def mark_session_start(self, *, clock_now: float) -> None:
        """Registra el inicio de la sesión (antes de la primera fase)."""
        self._session_start_mono = float(clock_now)

    def mark_session_end(self, *, clock_now: float) -> None:
        """Registra el fin de la sesión (tras la última fase o abort)."""
        self._session_end_mono = float(clock_now)

    def mark_start(
        self,
        phase: str,
        *,
        clock_now: float,
        detail: str = "",
    ) -> None:
        """Registra el inicio de ``phase``. Si se llama dos veces para la
        misma fase, sobrescribe (caso re-intento).
        """
        name = str(phase)
        if name not in self._phases:
            self._order.append(name)
        entry = self._phases.setdefault(name, _PhaseEntry())
        entry.start_mono = float(clock_now)
        if detail:
            entry.detail = str(detail)

    def mark_end(
        self,
        phase: str,
        *,
        clock_now: float,
        success: Optional[bool] = None,
        detail: str = "",
    ) -> None:
        """Registra el fin de ``phase`` con success y detalle opcional.

        Si ``mark_start`` no se llamó previamente, se crea entry con
        sólo el end_mono (start_mono queda None y duration_sec=None).
        """
        name = str(phase)
        if name not in self._phases:
            self._order.append(name)
            self._phases[name] = _PhaseEntry()
        entry = self._phases[name]
        entry.end_mono = float(clock_now)
        if success is not None:
            entry.success = bool(success)
        if detail:
            entry.detail = str(detail)

    def total_duration_sec(self) -> Optional[float]:
        """Duración total de la sesión.

        Si ``mark_session_start`` y ``mark_session_end`` se llamaron,
        usa esos timestamps. Si no, usa el min(start_mono) y
        max(end_mono) de las fases registradas. None si no hay datos.
        """
        if self._session_start_mono is not None and self._session_end_mono is not None:
            return max(0.0, self._session_end_mono - self._session_start_mono)
        starts = [e.start_mono for e in self._phases.values() if e.start_mono is not None]
        ends = [e.end_mono for e in self._phases.values() if e.end_mono is not None]
        if not starts or not ends:
            return None
        return max(0.0, max(ends) - min(starts))

    def snapshot(self) -> Dict[str, object]:
        """Devuelve un dict JSON-serializable con todas las latencias.

        Estructura:
          {
            "phases": {<name>: {start_mono, end_mono, duration_sec,
                                success, detail}},
            "phase_order": [<name1>, <name2>, ...],
            "session_start_mono": <float|None>,
            "session_end_mono": <float|None>,
            "total_duration_sec": <float|None>,
            "phases_completed": <int>,
            "phases_failed": <int>,
          }
        """
        completed = sum(1 for e in self._phases.values() if e.success is True)
        failed = sum(1 for e in self._phases.values() if e.success is False)
        return {
            "phases": {name: entry.to_dict() for name, entry in self._phases.items()},
            "phase_order": list(self._order),
            "session_start_mono": self._session_start_mono,
            "session_end_mono": self._session_end_mono,
            "total_duration_sec": self.total_duration_sec(),
            "phases_completed": completed,
            "phases_failed": failed,
        }

    def critical_path(self) -> List[Dict[str, object]]:
        """Devuelve la lista ordenada de fases con sus duraciones para
        análisis de critical path / bottleneck.

        Cada elemento: {phase, duration_sec, success}. Útil para tabla
        LaTeX o markdown del informe.
        """
        return [
            {
                "phase": name,
                "duration_sec": self._phases[name].duration_sec(),
                "success": self._phases[name].success,
            }
            for name in self._order
        ]
