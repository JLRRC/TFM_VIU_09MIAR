#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/retry.py
# Contenido: B-iter9 (2026-05-03) — retry con back-off para fases del orchestrator.
"""B-iter9 — Retry con back-off exponencial para fases inestables.

El legacy ``run_pick_demo`` reintenta TRANSPORT/APPROACH ante fallos
transitorios (planning failure, controller stale) con back-off antes de
abortar. Esto evita que un fallo aislado (ej. MoveIt en mal estado por
1s) tumbe el ciclo entero.

Este módulo provee una primitiva genérica:

    final_result, attempts_made = retry_with_backoff(
        call_fn,
        max_attempts=2,
        initial_backoff_sec=1.0,
        backoff_factor=2.0,
        sleep_fn=time.sleep,
    )

Donde:
  - ``call_fn()`` es un callable sin args que devuelve un objeto con
    atributo ``.success: bool``. Se invoca hasta que devuelva success
    o se agoten los intentos.
  - ``sleep_fn`` se inyecta para tests offline (mock que no duerme real).
  - retorna ``(final_result, attempts_made)`` donde attempts_made es el
    número total de intentos (1..max_attempts).

Constantes default extraídas del legacy (PANEL_PICK_DEMO_TRANSPORT_*):
  - max_attempts: 2 (1 try + 1 retry)
  - initial_backoff_sec: 1.0
  - backoff_factor: 2.0  → back-offs: 1s, 2s, 4s...
"""

from __future__ import annotations

import time
from typing import Any, Callable, Optional, Tuple


DEFAULT_MAX_ATTEMPTS: int = 2
DEFAULT_INITIAL_BACKOFF_SEC: float = 1.0
DEFAULT_BACKOFF_FACTOR: float = 2.0


def retry_with_backoff(
    call_fn: Callable[[], Any],
    *,
    max_attempts: int = DEFAULT_MAX_ATTEMPTS,
    initial_backoff_sec: float = DEFAULT_INITIAL_BACKOFF_SEC,
    backoff_factor: float = DEFAULT_BACKOFF_FACTOR,
    sleep_fn: Optional[Callable[[float], None]] = None,
) -> Tuple[Any, int]:
    """Invoca ``call_fn`` hasta success=True o agotar max_attempts.

    Parameters:
        call_fn: callable () → result. result debe exponer .success: bool.
            Si call_fn lanza excepción, se captura y se trata como
            fallo (intento gastado, retry si quedan).
        max_attempts: número total de intentos (>=1). Default 2.
        initial_backoff_sec: pausa antes del 2º intento (>=0). Default 1.0.
        backoff_factor: multiplicador del back-off entre intentos (>=1.0).
            Default 2.0 (back-offs: 1s, 2s, 4s, ...).
        sleep_fn: función para pausar (inyectable para tests).

    Returns:
        (final_result, attempts_made). final_result es el último result
        obtenido (success o no); attempts_made en [1, max_attempts].
        Si call_fn lanzó excepción en TODOS los intentos, final_result
        es un placeholder con success=False y reason="exception:...".
    """
    n = max(1, int(max_attempts))
    backoff = max(0.0, float(initial_backoff_sec))
    factor = max(1.0, float(backoff_factor))
    # Resolución tardía del default para que monkeypatch.setattr sobre
    # ``time.sleep`` funcione en tests sin pasar sleep_fn explícito.
    actual_sleep = sleep_fn if sleep_fn is not None else time.sleep

    last_result: Any = None
    last_exception: Exception | None = None

    for attempt in range(1, n + 1):
        try:
            last_result = call_fn()
            last_exception = None
        except Exception as exc:
            last_exception = exc
            last_result = _make_exception_placeholder(exc, attempt)

        if last_result is not None and bool(getattr(last_result, "success", False)):
            return last_result, attempt

        # Si no es el último intento, aplicar back-off.
        if attempt < n:
            try:
                actual_sleep(backoff)
            except Exception:
                pass  # nunca dejar que sleep rompa el retry
            backoff *= factor

    # Agotó intentos sin éxito.
    return last_result, n


def _make_exception_placeholder(exc: Exception, attempt: int) -> Any:
    """Crea un objeto-result placeholder con success=False y reason exc."""
    from types import SimpleNamespace
    return SimpleNamespace(
        success=False,
        reason=f"exception:attempt={attempt}:{type(exc).__name__}:{exc}",
    )
