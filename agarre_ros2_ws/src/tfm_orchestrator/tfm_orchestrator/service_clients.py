#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/service_clients.py
# Contenido: F6.1 — helpers de service call con timeout para el orchestrator.
"""Service client helpers para el orchestrator.

Cada fase del FSM necesita llamar a 1-N services definidos en
``ur5_panel_interfaces`` (Open, Close, SetWidth, Attach, Detach,
SelectObject, WorldToBase, ComputeApproachPose). Estos helpers
encapsulan el patrón:

    client = node.create_client(SrvType, name)
    if not client.wait_for_service(timeout): return fail(unavailable)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout)
    if not future.done(): return fail(timeout)
    return ok(future.result())

con un único punto de gestión de errores y un resultado tipado.

Diseño:

* ``ServiceCallResult`` es un dataclass inmutable con success + reason
  + payload. No depende de ROS — se puede inspeccionar/aserter en
  pytest puro.
* ``call_service_with_timeout(node, srv_type, name, request, ...)``
  hace toda la dance y devuelve ``ServiceCallResult``. Devuelve
  fail-soft (no levanta excepción) — el caller decide si abortar el
  FSM o degradar.
* Cliente cacheado por nombre via ``_get_or_create_client`` (evita
  recrear cliente en cada llamada).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class ServiceCallResult:
    """Resultado de un intento de llamada a service ROS 2.

    Atributos:
        success: True si el service respondió y la respuesta tiene
            success=True (o no expone success). False si timeout o
            respuesta con success=False.
        reason: cadena descriptiva del estado (siempre presente).
        payload: respuesta cruda del service o None.
        elapsed_sec: tiempo total transcurrido en la llamada.
    """

    success: bool
    reason: str
    payload: Optional[Any] = None
    elapsed_sec: float = 0.0


def _response_success(payload: Any) -> bool:
    """Heurística: si la respuesta tiene atributo ``success``, lo usa.

    Caso contrario asume que la simple llegada de respuesta = ok.
    """
    if payload is None:
        return False
    val = getattr(payload, "success", None)
    if val is None:
        return True
    return bool(val)


def _response_message(payload: Any) -> str:
    """Extrae mensaje/reason/detail de la respuesta si está disponible."""
    if payload is None:
        return ""
    for attr in ("reason", "message", "detail"):
        val = getattr(payload, attr, None)
        if val is not None:
            return str(val)
    return ""


def call_service_with_timeout(
    node: Any,
    srv_type: Any,
    service_name: str,
    request: Any,
    *,
    discovery_timeout_sec: float = 2.0,
    call_timeout_sec: float = 10.0,
    client_cache: Optional[Dict[str, Any]] = None,
) -> ServiceCallResult:
    """Llama a ``service_name`` y devuelve un ``ServiceCallResult``.

    No levanta excepciones; toda condición de error vuelve como
    ``ServiceCallResult(success=False, reason=…)``.

    Parameters:
        node: nodo ROS 2 (rclpy.node.Node) que hace la llamada.
        srv_type: clase del service (ej. ``ur5_panel_interfaces.srv.Open``).
        service_name: nombre absoluto del service (ej. ``"/gripper/open"``).
        request: instancia del request del service.
        discovery_timeout_sec: cuánto esperar a que aparezca el server.
        call_timeout_sec: cuánto esperar a la respuesta tras enviar.
        client_cache: dict opcional para reusar clients entre calls.
    """
    import time as _time
    try:
        import rclpy as _rclpy
    except ImportError:
        return ServiceCallResult(
            success=False,
            reason="rclpy_unavailable",
        )

    start = _time.monotonic()
    try:
        if client_cache is not None and service_name in client_cache:
            client = client_cache[service_name]
        else:
            client = node.create_client(srv_type, service_name)
            if client_cache is not None:
                client_cache[service_name] = client

        if not client.wait_for_service(timeout_sec=discovery_timeout_sec):
            return ServiceCallResult(
                success=False,
                reason=f"service_unavailable:{service_name}",
                elapsed_sec=_time.monotonic() - start,
            )

        future = client.call_async(request)
        _rclpy.spin_until_future_complete(
            node, future, timeout_sec=call_timeout_sec
        )

        if not future.done():
            return ServiceCallResult(
                success=False,
                reason=f"call_timeout:{service_name}",
                elapsed_sec=_time.monotonic() - start,
            )

        payload = future.result()
        ok = _response_success(payload)
        msg = _response_message(payload) or ("ok" if ok else "service_returned_false")
        return ServiceCallResult(
            success=ok,
            reason=msg,
            payload=payload,
            elapsed_sec=_time.monotonic() - start,
        )

    except Exception as exc:
        return ServiceCallResult(
            success=False,
            reason=f"exception:{type(exc).__name__}:{exc}",
            elapsed_sec=_time.monotonic() - start,
        )


@dataclass
class PhaseServiceMap:
    """Mapeo declarativo fase → service name.

    Permite override por param ROS sin tocar el código del FSM.
    """

    select_object: str = "/panel/select_object"
    approach_compute: str = "/orchestrator/compute_approach_pose"
    gripper_open: str = "/gripper/open"
    gripper_close: str = "/gripper/close"
    attach: str = "/orchestrator/attach"
    detach: str = "/orchestrator/detach"
    world_to_base: str = "/orchestrator/world_to_base"

    @classmethod
    def from_dict(cls, overrides: Dict[str, str]) -> "PhaseServiceMap":
        """Construye desde un dict, ignorando claves desconocidas."""
        valid_keys = {f.name for f in cls.__dataclass_fields__.values()}
        kwargs = {k: v for k, v in overrides.items() if k in valid_keys}
        return cls(**kwargs)
