"""F11 (auditoría 2026-05-10): namespace de system state + evidence.

Subpaquete que re-exporta los módulos del dominio "estado global del
sistema + grabación de evidencias":
  * ``system_state_manager`` — LifecycleNode publisher de SystemState.
  * ``system_health_helpers`` — helpers puros de health.
  * ``evidence_logger`` — Node grabador events.jsonl + summary.csv.
  * ``evidence_helpers`` — helpers puros (timestamps, parsing,
    métricas agregadas).
  * ``controller_bootstrap`` — Node spawneador de controllers ros2_control.

Los archivos físicos siguen en top-level; F11 iter 2 los moverá.
"""
from __future__ import annotations

from ..evidence_helpers import (  # noqa: F401
    compute_session_metrics,
    now_iso,
    parse_grasp_result,
    safe_unique_dir,
)

__all__ = [
    "compute_session_metrics",
    "now_iso",
    "parse_grasp_result",
    "safe_unique_dir",
]
