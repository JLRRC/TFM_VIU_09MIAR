#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/tf_batch_lookups.py
"""F8 / #20 (2026-05-08) — Batching helper puro para lookups TF.

Problema observado en el run E2E del 2026-05-08 (ciclo 174.9s):
panel_pick_demo dispara cientos de ``tf2_ros.Buffer.lookup_transform``
dispersos a lo largo del closure ``worker``. Muchos son redundantes:
piden el mismo (target, source) varias veces dentro de una misma fase.

Este módulo provee una función pura ``batch_lookup_requests`` que toma
una lista de ``LookupRequest(target_frame, source_frame, time_sec,
timeout_sec)`` y devuelve un plan optimizado:
- Deduplica entradas idénticas.
- Agrupa por (target, source) si time_sec ≈ Time(0) (lookup latest).
- Reporta cuántas llamadas se ahorrarían respecto a la lista naive.

NO ejecuta los lookups (eso requiere ``tf2_ros.Buffer`` real). Es la
parte testeable; el wrapper que lo llama vivirá en el ejecutor del
panel cuando se valide live.

Sin estado, sin ROS al importar. Tests offline.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, NamedTuple, Tuple


@dataclass(frozen=True)
class LookupRequest:
    """Petición canónica de lookup TF.

    Attributes:
        target_frame: frame destino (e.g. "rg2_pinch_center").
        source_frame: frame fuente (e.g. "base_link").
        time_sec: timestamp ROS en segundos. 0.0 = "latest" (Time(0)).
        timeout_sec: timeout de la lookup. No afecta el batching.
    """

    target_frame: str
    source_frame: str
    time_sec: float = 0.0
    timeout_sec: float = 0.0


class BatchPlan(NamedTuple):
    """Plan optimizado tras batching."""

    unique_requests: List[LookupRequest]  # peticiones a ejecutar
    n_input: int  # total de peticiones originales
    n_output: int  # tras batching
    saved_calls: int  # n_input - n_output
    duplicates_grouped: Dict[Tuple[str, str], int]
    # ^ cuántas peticiones del mismo (target, source) había
    # antes del batching (sólo para latest = Time(0)).


def _is_latest(time_sec: float, *, eps: float = 1e-6) -> bool:
    """True si time_sec representa Time(0) (latest)."""
    return abs(float(time_sec)) < float(eps)


def batch_lookup_requests(requests: List[LookupRequest]) -> BatchPlan:
    """Deduplica y agrupa peticiones equivalentes.

    Reglas:
    - Si dos peticiones tienen mismo (target, source) y ambas con
      time_sec ≈ 0 (latest), se conserva sólo una (la más permisiva en
      timeout — máximo).
    - Si dos peticiones tienen mismo (target, source, time_sec)
      con time_sec > 0, se conserva sólo una (mismo criterio).
    - Peticiones con (target, source) iguales pero time_sec distintos
      (>0) se mantienen separadas — son lookups en momentos distintos.

    Returns:
        BatchPlan con la lista deduplicada + estadísticas.
    """
    n_input = len(requests)
    if n_input == 0:
        return BatchPlan(
            unique_requests=[], n_input=0, n_output=0, saved_calls=0,
            duplicates_grouped={},
        )

    # Para latest (time_sec ≈ 0): clave (target, source).
    # Para timestamped: clave (target, source, time_sec).
    latest_buckets: Dict[Tuple[str, str], LookupRequest] = {}
    timestamped_buckets: Dict[Tuple[str, str, float], LookupRequest] = {}
    duplicates: Dict[Tuple[str, str], int] = {}

    for req in requests:
        target = str(req.target_frame).strip()
        source = str(req.source_frame).strip()
        if _is_latest(req.time_sec):
            key = (target, source)
            duplicates[key] = duplicates.get(key, 0) + 1
            existing = latest_buckets.get(key)
            if existing is None or float(req.timeout_sec) > float(existing.timeout_sec):
                latest_buckets[key] = LookupRequest(
                    target_frame=target,
                    source_frame=source,
                    time_sec=0.0,
                    timeout_sec=float(req.timeout_sec),
                )
        else:
            ts_key = (target, source, float(req.time_sec))
            existing_ts = timestamped_buckets.get(ts_key)
            if existing_ts is None or float(req.timeout_sec) > float(existing_ts.timeout_sec):
                timestamped_buckets[ts_key] = LookupRequest(
                    target_frame=target,
                    source_frame=source,
                    time_sec=float(req.time_sec),
                    timeout_sec=float(req.timeout_sec),
                )

    unique = list(latest_buckets.values()) + list(timestamped_buckets.values())
    return BatchPlan(
        unique_requests=unique,
        n_input=n_input,
        n_output=len(unique),
        saved_calls=n_input - len(unique),
        duplicates_grouped=duplicates,
    )


def reduction_ratio(plan: BatchPlan) -> float:
    """Fracción de llamadas ahorradas (0..1). 0 si nada ahorrable."""
    if plan.n_input <= 0:
        return 0.0
    return float(plan.saved_calls) / float(plan.n_input)
