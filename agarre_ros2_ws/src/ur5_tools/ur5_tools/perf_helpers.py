#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/perf_helpers.py
# Contenido: F19 (2026-05-01) — helpers puros de análisis de rendimiento.
"""Helpers puros de análisis de rendimiento (F19).

F19 (2026-05-01) cierra la fase de optimización proporcionando los
helpers analíticos que permiten medir antes/después de un cambio
sin necesidad de ROS vivo. Pensados para consumir los timestamps y
métricas que ``evidence_logger`` ya emite a ``events.jsonl``.

Funciones públicas:

* ``compute_percentiles(samples, *, percentiles=(0.50, 0.95, 0.99))``
  — devuelve un dict ``{p50: ..., p95: ..., p99: ...}`` o ``None``
  si no hay muestras.
* ``detect_periodic_oscillation(samples, *, expected_period_sec,
  tol_sec=0.02)`` — True si las diferencias consecutivas
  ``samples[i+1] - samples[i]`` se desvían sistemáticamente del
  período esperado más allá de la tolerancia.
* ``classify_topic_health(rate_hz, *, expected_hz, tol_pct=0.30)``
  — devuelve ``"ok"|"slow"|"fast"|"missing"`` comparando la tasa
  observada con la esperada.
* ``summarize_performance_run(samples_by_metric)`` — agrega
  percentiles + n + mean + min + max para un dict
  ``{metric_name: [samples]}``.
"""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


def compute_percentiles(
    samples: Sequence[float],
    *,
    percentiles: Tuple[float, ...] = (0.50, 0.95, 0.99),
) -> Optional[Dict[str, float]]:
    """Devuelve un dict con percentiles ``p50``, ``p95``, ``p99`` etc.

    Si ``samples`` está vacío, devuelve ``None`` (no hay nada que
    agregar). Los percentiles se calculan via ordenación + índice
    discreto (no interpolación), suficiente para reportes de
    latencia.

    Cada clave del dict tiene la forma ``"p<NN>"`` con ``NN`` en
    centésimas (p50, p95, p99, p999 si pasara 0.999).
    """
    nums: List[float] = []
    for s in samples:
        try:
            nums.append(float(s))
        except (TypeError, ValueError):
            continue
    if not nums:
        return None
    nums.sort()
    out: Dict[str, float] = {}
    n = len(nums)
    for p in percentiles:
        try:
            pf = float(p)
        except (TypeError, ValueError):
            continue
        pf = max(0.0, min(1.0, pf))
        idx = max(0, min(n - 1, int(round(pf * (n - 1)))))
        # Etiqueta canónica: p50, p95, p99, p999
        if pf == 1.0:
            label = "p100"
        else:
            scaled = pf * 100
            if abs(scaled - round(scaled)) < 1e-9:
                label = f"p{int(round(scaled))}"
            else:
                # ej. 0.999 → p999
                label = f"p{int(round(pf * 1000))}"
        out[label] = nums[idx]
    return out


def detect_periodic_oscillation(
    samples: Sequence[float],
    *,
    expected_period_sec: float,
    tol_sec: float = 0.02,
) -> bool:
    """True si las muestras se desvían sistemáticamente del período esperado.

    "Sistemáticamente" = más del 50% de las diferencias consecutivas
    están fuera del rango ``[expected - tol, expected + tol]``.

    Útil para detectar publishers desincronizados (deberían publicar a
    período fijo, pero llegan a doble/mitad de Hz por backpressure).

    Si hay menos de 2 muestras válidas, devuelve False (no hay
    suficiente información).
    """
    try:
        period = float(expected_period_sec)
        tol = max(0.0, float(tol_sec))
    except (TypeError, ValueError):
        return False
    if period <= 0.0:
        return False
    nums: List[float] = []
    for s in samples:
        try:
            nums.append(float(s))
        except (TypeError, ValueError):
            continue
    if len(nums) < 2:
        return False
    nums.sort()
    diffs = [b - a for a, b in zip(nums, nums[1:]) if (b - a) > 0]
    if not diffs:
        return False
    out_of_range = [
        d for d in diffs if abs(d - period) > tol
    ]
    return len(out_of_range) > (len(diffs) // 2)


def classify_topic_health(
    rate_hz: Optional[float],
    *,
    expected_hz: float,
    tol_pct: float = 0.30,
) -> str:
    """Clasifica la salud de un topic comparando rate observada con esperada.

    Devuelve uno de:

    * ``"missing"`` — ``rate_hz`` es None o <= 0.
    * ``"slow"``    — rate < expected * (1 - tol_pct).
    * ``"fast"``    — rate > expected * (1 + tol_pct).
    * ``"ok"``      — rate dentro del rango esperado.
    """
    if rate_hz is None:
        return "missing"
    try:
        rate = float(rate_hz)
        exp = float(expected_hz)
        tol = max(0.0, float(tol_pct))
    except (TypeError, ValueError):
        return "missing"
    if rate <= 0.0:
        return "missing"
    if exp <= 0.0:
        return "ok"
    lower = exp * (1.0 - tol)
    upper = exp * (1.0 + tol)
    if rate < lower:
        return "slow"
    if rate > upper:
        return "fast"
    return "ok"


def summarize_performance_run(
    samples_by_metric: Dict[str, Iterable[float]],
) -> Dict[str, Dict[str, Any]]:
    """Agrega percentiles + n + mean + min + max por métrica.

    Devuelve un dict ``{metric_name: {n, mean, min, max, p50, p95,
    p99}}``. Métricas sin muestras devuelven ``{n: 0}`` y nada más.

    Útil como fila de tabla en LaTeX/Markdown para comparar
    rendimiento antes/después de un cambio.
    """
    out: Dict[str, Dict[str, Any]] = {}
    for name, samples in samples_by_metric.items():
        nums: List[float] = []
        for s in samples or []:
            try:
                nums.append(float(s))
            except (TypeError, ValueError):
                continue
        if not nums:
            out[str(name)] = {"n": 0}
            continue
        pcts = compute_percentiles(nums) or {}
        out[str(name)] = {
            "n": len(nums),
            "mean": sum(nums) / len(nums),
            "min": min(nums),
            "max": max(nums),
            **pcts,
        }
    return out
