#!/usr/bin/env python3
"""F8 audit-v4 (2026-05-08): runtime ejecutor del plan de batch TF lookups.

Wrapper sobre ``tf2_ros.Buffer.lookup_transform`` que:
  1. Recibe una lista de ``LookupRequest`` (de ``tf_batch_lookups``).
  2. Llama ``batch_lookup_requests`` para deduplicar.
  3. Ejecuta una sola lookup por (target, source[, time]) único.
  4. Devuelve dict ``{(target,source): TransformStamped|None}``.

Esto evita el patrón observado en panel_pick_demo: el mismo
(world, base_link) lookup repetido 10+ veces dentro de una fase. Una
sola llamada al Buffer y se cachea el resultado en memoria local.

Sin tf2_ros al importar (lazy). Tests offline con mock Buffer.
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

from .tf_batch_lookups import LookupRequest, batch_lookup_requests


def execute_batch(
    buffer: Any,
    requests: Sequence[LookupRequest],
    *,
    log_fn: Optional[Any] = None,
) -> Dict[Tuple[str, str, float], Any]:
    """Ejecuta plan deduplicado contra ``buffer.lookup_transform``.

    El buffer debe exponer ``lookup_transform(target, source, time, timeout)``
    como ``tf2_ros.Buffer``. Time(0) se construye lazily desde rclpy.time
    si está disponible; si no, se pasa ``time_sec`` como float crudo
    (algunos buffers lo aceptan).

    Returns:
        dict {(target, source, time_sec) -> TransformStamped or None}.
        None si el lookup individual falló.
    """
    plan = batch_lookup_requests(list(requests))
    results: Dict[Tuple[str, str, float], Any] = {}
    if log_fn is not None:
        log_fn(
            f"[TF_BATCH] n_input={plan.n_input} n_output={plan.n_output} "
            f"saved={plan.saved_calls}"
        )
    Time = None
    Duration = None
    try:
        from rclpy.time import Time as _Time
        from rclpy.duration import Duration as _Duration
        Time = _Time
        Duration = _Duration
    except ImportError:
        pass

    for req in plan.unique_requests:
        time_arg: Any
        timeout_arg: Any
        if Time is not None and abs(req.time_sec) < 1e-6:
            time_arg = Time()
        else:
            time_arg = req.time_sec
        if Duration is not None and req.timeout_sec > 0.0:
            timeout_arg = Duration(seconds=float(req.timeout_sec))
        else:
            timeout_arg = req.timeout_sec
        try:
            ts = buffer.lookup_transform(
                req.target_frame,
                req.source_frame,
                time_arg,
                timeout_arg,
            )
        except Exception as exc:
            if log_fn is not None:
                log_fn(
                    f"[TF_BATCH][FAIL] {req.target_frame}<-{req.source_frame}"
                    f" {type(exc).__name__}:{exc}"
                )
            ts = None
        results[(req.target_frame, req.source_frame, req.time_sec)] = ts
    return results


def lookup_with_batch_cache(
    buffer: Any,
    requests: Sequence[LookupRequest],
    target_frame: str,
    source_frame: str,
    time_sec: float = 0.0,
    *,
    log_fn: Optional[Any] = None,
) -> Any:
    """Devuelve transform de un (target,source[,time]) usando un batch cache.

    Útil para consumers que hacen muchas lookups dentro de una fase:
    en lugar de llamar ``buffer.lookup_transform`` directo cada vez,
    construyen una lista de LookupRequest, llaman a esto N veces, y
    sólo se hacen las llamadas únicas al buffer.

    Si la combinación pedida no estaba en ``requests``, devuelve None
    (no realiza lookup ad-hoc — preserva el contrato del batch).
    """
    cache = execute_batch(buffer, requests, log_fn=log_fn)
    return cache.get((target_frame, source_frame, time_sec))
