#!/usr/bin/env python3
"""F3 audit (2026-05-10): primitivas para esperar eventos en lugar de sleeps fijos.

Sustituyen patrones ``time.sleep(N)`` (esperar N segundos por si acaso)
por bucles de polling con condición de salida. Cualquier nodo del stack
debería usar estas primitivas en lugar de sleeps fijos.

Diseño:

* :func:`wait_until` — polling tradicional con timeout. Devuelve
  ``True`` si la condición se cumplió, ``False`` si se agotó el
  timeout. ``poll_dt`` permite controlar el coste de la espera.
* :func:`wait_for_message` — espera a recibir un mensaje en un topic
  ROS 2 (one-shot). Usa una subscription temporal + executor.
* :func:`backoff_iter` — generador de tiempos de back-off
  exponencial limitado, útil para reintentos.

Estos helpers son síncronos (bloquean el hilo del caller) por
intención: están pensados para hilos worker / acciones secuenciales.
Para callbacks asíncronos, usar timers ROS o futures.
"""
from __future__ import annotations

import time
from typing import Callable, Iterator, Optional, TypeVar

T = TypeVar("T")


def wait_until(
    predicate: Callable[[], bool],
    timeout_sec: float,
    poll_dt: float = 0.05,
    sleep_fn: Optional[Callable[[float], None]] = None,
) -> bool:
    """Espera a que ``predicate()`` devuelva True o se agote el timeout.

    Args:
        predicate: callable que devuelve True cuando la condición se
            cumple. Debe ser barato (se invoca cada ``poll_dt``).
        timeout_sec: tiempo máximo de espera en segundos.
        poll_dt: intervalo entre evaluaciones de ``predicate``.
        sleep_fn: alternativa a ``time.sleep`` para tests deterministas.

    Returns:
        ``True`` si la condición se cumplió antes del timeout, ``False``
        si se agotó.
    """
    if poll_dt <= 0:
        raise ValueError("poll_dt debe ser positivo")
    sleep = sleep_fn or time.sleep
    deadline = time.monotonic() + max(0.0, timeout_sec)
    # Primera evaluación inmediata.
    if predicate():
        return True
    while time.monotonic() < deadline:
        sleep(poll_dt)
        if predicate():
            return True
    return False


def wait_for_value(
    fetch: Callable[[], Optional[T]],
    timeout_sec: float,
    poll_dt: float = 0.05,
    sleep_fn: Optional[Callable[[float], None]] = None,
) -> Optional[T]:
    """Variante de :func:`wait_until` que devuelve el valor obtenido.

    ``fetch()`` debe devolver ``None`` mientras el valor no está
    disponible y un valor concreto cuando lo está. Si se agota el
    timeout sin valor, devuelve ``None``.
    """
    sleep = sleep_fn or time.sleep
    deadline = time.monotonic() + max(0.0, timeout_sec)
    val = fetch()
    if val is not None:
        return val
    while time.monotonic() < deadline:
        sleep(poll_dt)
        val = fetch()
        if val is not None:
            return val
    return None


def backoff_iter(
    initial_sec: float,
    factor: float = 2.0,
    max_sec: Optional[float] = None,
    max_attempts: Optional[int] = None,
) -> Iterator[float]:
    """Itera tiempos de back-off exponencial.

    >>> list(backoff_iter(1.0, factor=2.0, max_attempts=4))
    [1.0, 2.0, 4.0, 8.0]
    >>> list(backoff_iter(1.0, factor=2.0, max_sec=3.5, max_attempts=5))
    [1.0, 2.0, 3.5, 3.5, 3.5]
    """
    if initial_sec < 0:
        raise ValueError("initial_sec no puede ser negativo")
    if factor < 1.0:
        raise ValueError("factor debe ser >= 1.0")
    cur = initial_sec
    n = 0
    while max_attempts is None or n < max_attempts:
        if max_sec is not None and cur > max_sec:
            cur = max_sec
        yield cur
        cur *= factor
        n += 1


def wait_for_message(
    node,
    topic: str,
    msg_type,
    timeout_sec: float = 5.0,
    qos=None,
):
    """Espera one-shot un mensaje en ``topic``.

    Crea una subscription temporal, hace spin hasta recibir el primer
    mensaje o agotar el timeout, y limpia la subscription. Devuelve el
    mensaje o ``None`` si timeout.

    Requiere un nodo ROS 2 ya iniciado. Implementación lazy del import
    de rclpy para mantener este módulo testable offline.
    """
    try:
        from rclpy.qos import QoSProfile, ReliabilityPolicy
    except ImportError:  # pragma: no cover - sólo si no hay ROS
        return None

    if qos is None:
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

    received: list = []

    def _cb(msg) -> None:
        received.append(msg)

    sub = node.create_subscription(msg_type, topic, _cb, qos)
    try:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline and not received:
            try:
                node.executor.spin_once(timeout_sec=0.05)  # type: ignore[union-attr]
            except Exception:
                # Sin executor configurado, fallback a rclpy.spin_once.
                import rclpy
                rclpy.spin_once(node, timeout_sec=0.05)
        return received[0] if received else None
    finally:
        node.destroy_subscription(sub)
