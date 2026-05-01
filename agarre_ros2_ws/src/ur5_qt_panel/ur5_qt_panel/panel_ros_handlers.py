#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_ros_handlers.py
# Contenido: F3 — handlers/parsers puros para callbacks ROS de RosWorker.
"""Handlers y parsers puros para callbacks ROS de RosWorker.

Extraídos de ``panel_ros.py`` para reducir el god-file y permitir
tests offline. Cero dependencia de ROS / threading / locks — toda la
lógica que aquí vive recibe los inputs como argumentos y devuelve
valores.

Funciones públicas:

* ``parse_moveit_result_payload(data)``: parsea el JSON ``data`` de
  un msg ``std_msgs/String`` que contiene el resultado de MoveIt
  (request_id, request_uuid, success). Devuelve ``(req_id, req_uuid,
  success_str, parse_status)``.
* ``format_rx_result_log(...)``: construye log
  ``[PICK_OBJ][RX_RESULT] ts=... req_id=... ...``.
* ``classify_subscription_action(...)``: helper de decisión sobre qué
  hacer al recibir un ``subscribe_*`` request — útil para reducir
  duplicación en los métodos subscribe_*.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Optional, Tuple


def parse_moveit_result_payload(
    data: object,
) -> Tuple[int, str, str, str]:
    """Parsea el campo ``data`` de un ``std_msgs/String`` con resultado MoveIt.

    Esperado ``data`` es un str con JSON:
      ``{"request_id": <int>, "request_uuid": <str>, "success": <bool>, ...}``

    Devuelve ``(req_id, req_uuid, success_str, parse_status)``:
      * req_id: int (-1 si no presente o parse error)
      * req_uuid: str ("" si no presente)
      * success_str: "true"|"false"|"n/a" (lowercase)
      * parse_status: "json" si parseó OK, "raw" si no
    """
    raw = ""
    try:
        raw = str(data or "")
    except Exception:
        raw = ""
    req_id = -1
    req_uuid = ""
    success = "n/a"
    parse_status = "raw"
    try:
        payload = json.loads(raw)
    except (json.JSONDecodeError, ValueError, TypeError):
        return req_id, req_uuid, success, parse_status
    if not isinstance(payload, dict):
        return req_id, req_uuid, success, parse_status
    try:
        req_id = int(payload.get("request_id", -1) or -1)
    except (TypeError, ValueError):
        req_id = -1
    try:
        req_uuid = str(payload.get("request_uuid", "") or "")
    except Exception:
        req_uuid = ""
    try:
        success = str(bool(payload.get("success", False))).lower()
    except Exception:
        success = "n/a"
    parse_status = "json"
    return req_id, req_uuid, success, parse_status


def format_rx_result_log(
    *,
    ts_wall: float,
    req_id: int,
    req_uuid: str,
    success_str: str,
    parse_status: str,
    topic: str,
    seq: int,
) -> str:
    """Construye log ``[PICK_OBJ][RX_RESULT]``."""
    return (
        "[PICK_OBJ][RX_RESULT] "
        f"ts={float(ts_wall):.6f} req_id={int(req_id)} "
        f"req_uuid={req_uuid or 'n/a'} "
        f"success={success_str} match=unknown accepted=queued "
        f"reason={parse_status} topic={topic} seq={int(seq)}"
    )


@dataclass(frozen=True)
class SubscriptionDecision:
    action: str  # "skip" | "noop_already_subscribed" | "destroy_then_create" | "create"
    error_msg: Optional[str] = None


def classify_subscription_action(
    *,
    ros_available: bool,
    msg_class_available: bool,
    topic: str,
    node_ready: bool,
    current_sub_present: bool,
    current_topic: str,
) -> SubscriptionDecision:
    """Decide qué acción tomar ante un ``subscribe_*`` request.

    Acciones:
      * ``skip``: condiciones previas no cumplidas (ROS no disponible,
        topic vacío, nodo no listo). Devuelve también ``error_msg``.
      * ``noop_already_subscribed``: ya hay subscription al mismo topic.
      * ``destroy_then_create``: hay subscription a otro topic; hay que
        destruir antes de crear.
      * ``create``: no hay subscription previa; crear directamente.
    """
    if not ros_available or not msg_class_available:
        return SubscriptionDecision(action="skip", error_msg="ros_unavailable")
    topic_clean = (topic or "").strip()
    if not topic_clean:
        return SubscriptionDecision(action="skip", error_msg="empty_topic")
    if not node_ready:
        return SubscriptionDecision(action="skip", error_msg="node_not_ready")
    if current_sub_present and current_topic == topic_clean:
        return SubscriptionDecision(action="noop_already_subscribed")
    if current_sub_present:
        return SubscriptionDecision(action="destroy_then_create")
    return SubscriptionDecision(action="create")
