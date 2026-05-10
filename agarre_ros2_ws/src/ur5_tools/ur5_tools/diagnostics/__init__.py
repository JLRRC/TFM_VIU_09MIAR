"""F11 (auditoría 2026-05-10): namespace de diagnostics + cycle timing.

Subpaquete que re-exporta los módulos de "diagnóstico, perfilado y
cycle timing":
  * ``cycle_timing_analyzer`` / ``cycle_timing_aggregator`` — análisis
    de latencias por fase.
  * ``cli_cycle_timing`` / ``generate_latency_table`` — CLIs.
  * ``cycle_logger`` — recorder.
  * ``perf_helpers`` — helpers puros de perf.
  * ``clock_probe`` / ``tf_probe`` / ``jt_smoke_test`` — CLIs de debug.

Los archivos físicos siguen en top-level; F11 iter 2 los moverá.
"""
from __future__ import annotations

from ..perf_helpers import *  # noqa: F401,F403

__all__: list[str] = []
