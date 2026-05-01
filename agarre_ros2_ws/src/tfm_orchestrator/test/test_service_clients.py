#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/test/test_service_clients.py
# Contenido: F6.1 — tests offline del helper de service call.
"""Tests del helper ``call_service_with_timeout`` y ``PhaseServiceMap``.

NO levantan ROS — usan mocks de Node + client + future. Cubren:

* Resultado fail-soft cuando rclpy no está disponible.
* Service unavailable (wait_for_service devuelve False).
* Call timeout (future never done).
* Respuesta con ``success=False``.
* Respuesta con ``success=True``.
* Respuesta sin atributo success (asume ok por contrato).
* Excepción dentro de la llamada → fail con reason exception:.
* Cache de clients por nombre.
* PhaseServiceMap defaults + from_dict ignora claves extras.
"""

from __future__ import annotations

import sys
import types
from unittest.mock import MagicMock

import pytest

from tfm_orchestrator.service_clients import (
    PhaseServiceMap,
    ServiceCallResult,
    call_service_with_timeout,
)


# ---------------------------------------------------------------------------
# Mock infra para simular rclpy + node + client + future
# ---------------------------------------------------------------------------


@pytest.fixture
def mock_rclpy(monkeypatch):
    """Inyecta rclpy mock con spin_until_future_complete no-op.

    El fixture es deliberadamente pasivo: el future ya viene con
    ``done()`` y ``result()`` configurados desde ``_make_mock_node``.
    El spin solo simula el bloqueo (no hace nada).
    """
    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.spin_until_future_complete = MagicMock()
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    return fake_rclpy


def _make_mock_node(client_response):
    """Devuelve un mock node cuyo create_client devuelve un client mock.

    El client mock:
      - wait_for_service: devuelve client_response['available'] (bool)
      - call_async: devuelve un mock future con done()/result()
        preconfigurados según ``ever_done`` y ``payload``.
    """
    node = MagicMock(name="Node")

    def _create_client(srv_type, name):  # noqa: ARG001
        client = MagicMock(name=f"Client({name})")
        client.wait_for_service = MagicMock(
            return_value=client_response.get("available", True)
        )

        def _call_async(req):  # noqa: ARG001
            future = MagicMock(name="Future")
            ever_done = client_response.get("ever_done", True)
            future.done = MagicMock(return_value=ever_done)
            if ever_done:
                future.result = MagicMock(
                    return_value=client_response.get("payload")
                )
            return future

        client.call_async = _call_async
        return client

    node.create_client = _create_client
    return node


# ---------------------------------------------------------------------------
# call_service_with_timeout
# ---------------------------------------------------------------------------


def test_returns_dataclass(mock_rclpy):
    payload = MagicMock()
    payload.success = True
    payload.reason = "ok"
    node = _make_mock_node({"available": True, "payload": payload})
    r = call_service_with_timeout(node, MagicMock, "/srv", MagicMock())
    assert isinstance(r, ServiceCallResult)
    assert r.success is True
    assert r.reason == "ok"


def test_unavailable_service_fails_soft(mock_rclpy):
    node = _make_mock_node({"available": False})
    r = call_service_with_timeout(
        node, MagicMock, "/missing", MagicMock(),
        discovery_timeout_sec=0.01,
    )
    assert r.success is False
    assert r.reason == "service_unavailable:/missing"
    assert r.payload is None


def test_call_timeout(mock_rclpy):
    node = _make_mock_node({"available": True, "ever_done": False})
    r = call_service_with_timeout(
        node, MagicMock, "/slow", MagicMock(),
        call_timeout_sec=0.01,
    )
    assert r.success is False
    assert r.reason == "call_timeout:/slow"


def test_response_success_false(mock_rclpy):
    payload = MagicMock()
    payload.success = False
    payload.reason = "object_not_visible"
    node = _make_mock_node({"available": True, "payload": payload})
    r = call_service_with_timeout(node, MagicMock, "/x", MagicMock())
    assert r.success is False
    assert r.reason == "object_not_visible"


def test_response_without_success_attr_treated_as_ok(mock_rclpy):
    """Si la respuesta no tiene 'success' (Trigger-like), se asume ok."""
    payload = types.SimpleNamespace()  # sin .success
    node = _make_mock_node({"available": True, "payload": payload})
    r = call_service_with_timeout(node, MagicMock, "/trigger", MagicMock())
    assert r.success is True


def test_response_message_extracted_from_reason(mock_rclpy):
    payload = types.SimpleNamespace(success=True, reason="canonical_ok")
    node = _make_mock_node({"available": True, "payload": payload})
    r = call_service_with_timeout(node, MagicMock, "/x", MagicMock())
    assert r.reason == "canonical_ok"


def test_response_message_falls_back_to_message_attr(mock_rclpy):
    payload = types.SimpleNamespace(success=True, message="legacy_msg")
    node = _make_mock_node({"available": True, "payload": payload})
    r = call_service_with_timeout(node, MagicMock, "/x", MagicMock())
    assert r.reason == "legacy_msg"


def test_response_message_falls_back_to_detail_attr(mock_rclpy):
    payload = types.SimpleNamespace(success=True, detail="from_detail")
    node = _make_mock_node({"available": True, "payload": payload})
    r = call_service_with_timeout(node, MagicMock, "/x", MagicMock())
    assert r.reason == "from_detail"


def test_response_message_default_ok(mock_rclpy):
    payload = types.SimpleNamespace(success=True)  # sin reason/message/detail
    node = _make_mock_node({"available": True, "payload": payload})
    r = call_service_with_timeout(node, MagicMock, "/x", MagicMock())
    assert r.reason == "ok"


def test_exception_handled(monkeypatch):
    """Excepción al crear client → fail-soft con reason exception:."""
    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.spin_until_future_complete = MagicMock()
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)

    node = MagicMock()
    node.create_client = MagicMock(side_effect=RuntimeError("boom"))
    r = call_service_with_timeout(node, MagicMock, "/x", MagicMock())
    assert r.success is False
    assert r.reason.startswith("exception:RuntimeError:boom")


def test_rclpy_not_installed(monkeypatch):
    """Si rclpy no se puede importar, fail-soft."""
    # Forzar ImportError eliminando rclpy si está y bloqueando import
    if "rclpy" in sys.modules:
        monkeypatch.delitem(sys.modules, "rclpy")

    real_import = __builtins__["__import__"] if isinstance(__builtins__, dict) else __builtins__.__import__

    def fake_import(name, *args, **kwargs):
        if name == "rclpy":
            raise ImportError("rclpy not installed")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr("builtins.__import__", fake_import)
    node = MagicMock()
    r = call_service_with_timeout(node, MagicMock, "/x", MagicMock())
    assert r.success is False
    assert r.reason == "rclpy_unavailable"


def test_client_cache_reused(mock_rclpy):
    """Misma key en cache devuelve mismo client (no re-creates)."""
    payload = types.SimpleNamespace(success=True, reason="ok")
    node = _make_mock_node({"available": True, "payload": payload})
    cache = {}
    call_service_with_timeout(node, MagicMock, "/cached", MagicMock(), client_cache=cache)
    assert "/cached" in cache
    cached_client = cache["/cached"]

    call_service_with_timeout(node, MagicMock, "/cached", MagicMock(), client_cache=cache)
    assert cache["/cached"] is cached_client


# ---------------------------------------------------------------------------
# PhaseServiceMap
# ---------------------------------------------------------------------------


def test_phase_service_map_defaults():
    m = PhaseServiceMap()
    assert m.gripper_open == "/gripper/open"
    assert m.gripper_close == "/gripper/close"
    assert m.attach == "/orchestrator/attach"
    assert m.detach == "/orchestrator/detach"
    assert m.world_to_base == "/orchestrator/world_to_base"
    assert m.select_object == "/panel/select_object"
    assert m.approach_compute == "/orchestrator/compute_approach_pose"


def test_phase_service_map_from_dict_overrides():
    m = PhaseServiceMap.from_dict({"gripper_open": "/custom/open"})
    assert m.gripper_open == "/custom/open"
    # otros defaults
    assert m.gripper_close == "/gripper/close"


def test_phase_service_map_from_dict_ignores_unknown():
    m = PhaseServiceMap.from_dict({"unknown_key": "x", "gripper_open": "/y"})
    assert m.gripper_open == "/y"
    # No raise
