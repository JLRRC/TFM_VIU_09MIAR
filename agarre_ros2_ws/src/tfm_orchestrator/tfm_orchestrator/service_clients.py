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

from dataclasses import dataclass
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
    import threading as _threading
    try:
        import rclpy as _rclpy  # noqa: F401
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

        # F5-step7 (2026-05-03): Event-based wait en lugar de
        # spin_until_future_complete. Evita deadlock cuando se llama desde
        # dentro del callback de un action server con MultiThreadedExecutor +
        # ReentrantCallbackGroup: spin_until_future_complete crearía un
        # SingleThreadedExecutor nuevo que conflictúa con el executor
        # principal. add_done_callback delega al executor principal que
        # ya está spinning las callbacks del action client.
        future = client.call_async(request)
        done_event = _threading.Event()
        future.add_done_callback(lambda _f: done_event.set())
        done_event.wait(timeout=call_timeout_sec)

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

    # F5-step6c (2026-05-03): defaults alineados con services REALES
    # del stack vivo. Validado en runtime con Gazebo activo:
    #   * /panel/select_object        ← exposed by ControlPanelV2 (panel_ros)
    #   * /tf_geometry/world_to_base  ← tf_geometry_service (existe)
    #   * /tf_geometry/compute_approach_pose ← tf_geometry_service (existe)
    #   * /orchestrator/resolve_object_pose_world ← object_pose_resolver_service
    #     (cableado en F5-step5)
    #   * /orchestrator/plan_to_pose  ← plan_to_pose_server (existe en
    #     ur5_tools, falta auto-launch — F5-step6a pendiente)
    #   * /gripper/open|close, /orchestrator/attach|detach ← NO existen
    #     todavía. F5-step6b pendiente: implementar wrappers en
    #     gripper_attach_backend.
    select_object: str = "/panel/select_object"
    approach_compute: str = "/tf_geometry/compute_approach_pose"
    resolve_object_pose_world: str = "/orchestrator/resolve_object_pose_world"
    gripper_open: str = "/gripper/open"
    gripper_close: str = "/gripper/close"
    attach: str = "/orchestrator/attach"
    detach: str = "/orchestrator/detach"
    world_to_base: str = "/tf_geometry/world_to_base"
    plan_to_pose_action: str = "/orchestrator/plan_to_pose"

    @classmethod
    def from_dict(cls, overrides: Dict[str, str]) -> "PhaseServiceMap":
        """Construye desde un dict, ignorando claves desconocidas."""
        valid_keys = {f.name for f in cls.__dataclass_fields__.values()}
        kwargs = {k: v for k, v in overrides.items() if k in valid_keys}
        return cls(**kwargs)


# ---------------------------------------------------------------------------
# F6.3: Action client helper (similar pattern al de service_clients)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ActionCallResult:
    """Resultado de una action call.

    Atributos:
        success: True si el action server respondió y la respuesta tiene
            success=True (o no expone success). False si el server no
            apareció, el goal fue rechazado, hubo timeout o el result
            llegó con success=False.
        reason: cadena descriptiva del estado.
        result: payload .result del action o None.
        feedback_count: nº de feedback recibidos durante la ejecución.
        elapsed_sec: tiempo total transcurrido.
    """

    success: bool
    reason: str
    result: Optional[Any] = None
    feedback_count: int = 0
    elapsed_sec: float = 0.0


def call_action_with_timeout(
    node: Any,
    action_type: Any,
    action_name: str,
    goal: Any,
    *,
    discovery_timeout_sec: float = 2.0,
    accept_timeout_sec: float = 3.0,
    result_timeout_sec: float = 60.0,
    feedback_callback: Optional[Any] = None,
    client_cache: Optional[Dict[str, Any]] = None,
) -> ActionCallResult:
    """Llama a un action server y devuelve un ``ActionCallResult``.

    No levanta excepciones; toda condición de error vuelve como
    ``ActionCallResult(success=False, reason=…)``.

    Parameters:
        node: nodo ROS 2 (rclpy.node.Node) que hace la llamada.
        action_type: clase del action (ej. PlanToPose).
        action_name: nombre absoluto del action server.
        goal: instancia de la Goal del action.
        discovery_timeout_sec: cuánto esperar a que aparezca el server.
        accept_timeout_sec: cuánto esperar al accept del goal.
        result_timeout_sec: cuánto esperar al result tras accept.
        feedback_callback: opcional, llamado con cada Feedback que llega.
        client_cache: dict opcional para reusar clients entre calls.
    """
    import time as _time
    import threading as _threading
    try:
        import rclpy as _rclpy  # noqa: F401
        # F19 (2026-05-08): rclpy.action stubs no exportan ActionClient
        # consistentemente entre mypy versions; unused-ignore se silencia
        # para no requerir actualización por entorno.
        from rclpy.action import ActionClient as _ActionClient  # type: ignore[attr-defined,unused-ignore]
    except ImportError:
        return ActionCallResult(
            success=False,
            reason="rclpy_unavailable",
        )

    start = _time.monotonic()
    feedback_box = {"count": 0}

    def _feedback_wrapper(fb_msg: Any) -> None:
        feedback_box["count"] += 1
        if feedback_callback is not None:
            try:
                feedback_callback(fb_msg)
            except Exception:
                pass

    try:
        if client_cache is not None and action_name in client_cache:
            client = client_cache[action_name]
        else:
            client = _ActionClient(node, action_type, action_name)  # type: ignore[no-untyped-call,unused-ignore]
            if client_cache is not None:
                client_cache[action_name] = client

        if not client.wait_for_server(timeout_sec=discovery_timeout_sec):
            return ActionCallResult(
                success=False,
                reason=f"action_server_unavailable:{action_name}",
                elapsed_sec=_time.monotonic() - start,
            )

        # F5-step7 (2026-05-03): Event-based wait. Evita deadlock al llamar
        # desde dentro de un goal callback del action server propio.
        send_future = client.send_goal_async(goal, feedback_callback=_feedback_wrapper)
        send_event = _threading.Event()
        send_future.add_done_callback(lambda _f: send_event.set())
        send_event.wait(timeout=accept_timeout_sec)
        if not send_future.done():
            return ActionCallResult(
                success=False,
                reason=f"goal_send_timeout:{action_name}",
                feedback_count=feedback_box["count"],
                elapsed_sec=_time.monotonic() - start,
            )

        goal_handle = send_future.result()
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            return ActionCallResult(
                success=False,
                reason=f"goal_rejected:{action_name}",
                feedback_count=feedback_box["count"],
                elapsed_sec=_time.monotonic() - start,
            )

        result_future = goal_handle.get_result_async()
        result_event = _threading.Event()
        result_future.add_done_callback(lambda _f: result_event.set())
        result_event.wait(timeout=result_timeout_sec)
        if not result_future.done():
            return ActionCallResult(
                success=False,
                reason=f"result_timeout:{action_name}",
                feedback_count=feedback_box["count"],
                elapsed_sec=_time.monotonic() - start,
            )

        wrapper = result_future.result()
        payload = getattr(wrapper, "result", None)
        ok = _response_success(payload)
        msg = _response_message(payload) or ("ok" if ok else "action_returned_false")
        return ActionCallResult(
            success=ok,
            reason=msg,
            result=payload,
            feedback_count=feedback_box["count"],
            elapsed_sec=_time.monotonic() - start,
        )

    except Exception as exc:
        return ActionCallResult(
            success=False,
            reason=f"exception:{type(exc).__name__}:{exc}",
            feedback_count=feedback_box["count"],
            elapsed_sec=_time.monotonic() - start,
        )
