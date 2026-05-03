#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/pick_object/moveit_bridge_path.py
# Contenido: F3-step5bis-c — extracción del CUERPO PRINCIPAL de _ensure_moveit_bridge_path.
"""MoveIt bridge path verifier extraído del closure run_pick_object.worker.

``ensure_moveit_bridge_path_main_body`` valida que el ``ur5_moveit_bridge``
está escuchando el ``topic`` esperado y, si no, dispara recovery.

Diseño F3-step5bis-c (variante específica): la fn original tenía 4
nested defs internas con kwargs HOMÓNIMOS a free vars del closure
padre (``timeout_sec``, ``cold_start``, ``log_tag``, ``require_heartbeat``).
La sustitución regex automática rompía la sintaxis. Solución:

* Las 4 nested defs PERMANECEN dentro del wrapper en el closure.
* Aquí extraemos sólo el **cuerpo principal** (~100 LOC tras las
  nested defs), que orquesta los checks de subscriptores + topic_pubs
  + heartbeat + dispara recovery via los callables del ctx.
* ``MoveItBridgePathContext`` recibe las 4 nested como Callables,
  garantizando un contrato explícito.

Reduce ~100 LOC del closure manteniendo la lógica completa.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Tuple


@dataclass
class MoveItBridgePathContext:
    """Captura de dependencias closure de ensure_moveit_bridge_path."""

    panel: Any
    moveit_hb_topic: str
    moveit_result_topic: str
    # Las 4 nested defs originales se inyectan como callables; permanecen
    # en el wrapper del closure por sus colisiones de kwargs homónimos.
    read_bridge_topics_fn: Callable[[], Tuple[int, int, int, float, bool]]
    wait_bridge_match_fn: Callable[..., Tuple[bool, int, int, int, float, bool]]
    restart_moveit_bridge_and_wait_fn: Callable[..., None]


def ensure_moveit_bridge_path_main_body(
    ctx: MoveItBridgePathContext,
    topic: str,
) -> bool:
    """Cuerpo principal de la verificación + recovery del moveit_bridge.

    Devuelve True si se disparó un recovery (cold start o restart) y
    bridge quedó funcional. False si ya estaba todo OK desde el
    arranque.

    Lanza RuntimeError si:
      * ROS node no está listo.
      * Tras recovery, sigue sin pose subscribers.
      * No se puede subscribe al result topic.
      * No hay publisher en el result topic tras settle.
    """
    panel = ctx.panel
    bridge_recovered = False

    if not panel._ros_worker_started:
        panel._ensure_ros_worker_started()
    if not panel.ros_worker.node_ready():
        raise RuntimeError("ROS node no listo para MoveIt bridge")
    panel.ros_worker.subscribe_moveit_bridge_heartbeat(ctx.moveit_hb_topic)
    subs, result_pubs, result_subs, hb_age, hb_recent = ctx.read_bridge_topics_fn()
    hb_age_txt = "inf" if math.isinf(hb_age) else f"{hb_age:.2f}s"
    panel._emit_log(
        f"[PICK_OBJ][MOVEIT][PLAN] topics pose_subs={subs} result_pubs={result_pubs} result_subs={result_subs} "
        f"pose_topic={topic} result_topic={ctx.moveit_result_topic} "
        f"hb_recent={str(bool(hb_recent)).lower()} hb_age={hb_age_txt}"
    )
    if subs <= 0 or result_pubs <= 0:
        bridge_alive = bool(panel._proc_alive(getattr(panel, "moveit_bridge_proc", None)))
        bridge_detected = False
        try:
            bridge_detected = bool(panel._moveit_bridge_detected())
        except Exception:
            bridge_detected = False
        if bridge_alive or bridge_detected or hb_recent:
            panel._emit_log(
                "[PICK_OBJ][MOVEIT][PLAN] bridge_match_warmup "
                f"pose_subs={subs} result_pubs={result_pubs} bridge_alive={str(bridge_alive).lower()} "
                f"bridge_detected={str(bool(bridge_detected)).lower()}"
            )
            matched, subs, result_pubs, result_subs, hb_age, hb_recent = ctx.wait_bridge_match_fn(
                timeout_sec=1.8,
                log_tag="bridge_match_warmup_check",
            )
            if matched:
                hb_age_txt = "inf" if math.isinf(hb_age) else f"{hb_age:.2f}s"
                panel._emit_log(
                    f"[PICK_OBJ][MOVEIT][PLAN] bridge_match_warmup_ok pose_subs={subs} result_pubs={result_pubs} "
                    f"result_subs={result_subs} hb_recent={str(bool(hb_recent)).lower()} hb_age={hb_age_txt}"
                )
        if subs <= 0 or result_pubs <= 0:
            if not bridge_alive and not bridge_detected and not hb_recent:
                ctx.restart_moveit_bridge_and_wait_fn(cold_start=True)
            else:
                panel._emit_log(
                    "[PICK_OBJ][MOVEIT][PLAN] bridge_path_missing "
                    f"pose_subs={subs} result_pubs={result_pubs}; attempting_recover=true"
                )
                ctx.restart_moveit_bridge_and_wait_fn(cold_start=False)
            bridge_recovered = True
            subs, result_pubs, result_subs, hb_age, hb_recent = ctx.read_bridge_topics_fn()
            hb_age_txt = "inf" if math.isinf(hb_age) else f"{hb_age:.2f}s"
            panel._emit_log(
                f"[PICK_OBJ][MOVEIT][PLAN] topics_after_recover pose_subs={subs} result_pubs={result_pubs} "
                f"result_subs={result_subs} pose_topic={topic} result_topic={ctx.moveit_result_topic} "
                f"hb_recent={str(bool(hb_recent)).lower()} hb_age={hb_age_txt}"
            )
            if subs <= 0:
                msg = f"MoveItBridge NO conectado: {topic} sin subscriptores"
                panel._emit_log(f"[PICK_OBJ][ABORT] {msg}")
                raise RuntimeError(msg)
    if not panel.ros_worker.subscribe_moveit_result(ctx.moveit_result_topic):
        msg = f"No se pudo suscribir a {ctx.moveit_result_topic}"
        panel._emit_log(f"[PICK_OBJ][ABORT] {msg}")
        raise RuntimeError(msg)
    result_pubs = panel.ros_worker.topic_publisher_count(ctx.moveit_result_topic)
    result_subs = panel.ros_worker.topic_subscriber_count(ctx.moveit_result_topic)
    if result_subs <= 0:
        result_wait_deadline = time.time() + 1.5
        while time.time() < result_wait_deadline:
            result_pubs = panel.ros_worker.topic_publisher_count(ctx.moveit_result_topic)
            result_subs = panel.ros_worker.topic_subscriber_count(ctx.moveit_result_topic)
            if result_pubs > 0 and result_subs > 0:
                break
            time.sleep(0.10)
    panel._emit_log(
        f"[PICK_OBJ][MOVEIT][PLAN] result_path result_pubs={result_pubs} result_subs={result_subs} "
        f"topic={ctx.moveit_result_topic}"
    )
    if result_pubs <= 0:
        msg = f"MoveIt bridge sin publisher en {ctx.moveit_result_topic}"
        panel._emit_log(f"[PICK_OBJ][ABORT] {msg}")
        raise RuntimeError(msg)
    if bridge_recovered:
        hb_deadline = time.time() + 2.5
        while time.time() < hb_deadline:
            hb_age = panel.ros_worker.moveit_bridge_heartbeat_age()
            hb_recent = panel.ros_worker.has_recent_moveit_bridge_heartbeat(1.2)
            if hb_recent:
                hb_age_txt = "inf" if math.isinf(hb_age) else f"{hb_age:.2f}s"
                panel._emit_log(
                    "[PICK_OBJ][MOVEIT][PLAN] bridge_recover_warmup "
                    f"heartbeat_ready=true hb_age={hb_age_txt}"
                )
                # After a stop/start cycle the first volatile result can be lost
                # if we publish immediately. Give DDS matching a short settle window.
                time.sleep(0.35)
                break
            time.sleep(0.10)
        else:
            hb_age = panel.ros_worker.moveit_bridge_heartbeat_age()
            hb_age_txt = "inf" if math.isinf(hb_age) else f"{hb_age:.2f}s"
            panel._emit_log(
                "[PICK_OBJ][MOVEIT][PLAN] bridge_recover_warmup "
                f"heartbeat_timeout hb_age={hb_age_txt}"
            )
    return bridge_recovered
