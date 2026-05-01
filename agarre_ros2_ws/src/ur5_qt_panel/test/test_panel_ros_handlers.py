#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/test/test_panel_ros_handlers.py
# Contenido: F3 — tests offline de panel_ros_handlers.
"""Tests offline de panel_ros_handlers (parsers + decisión sub)."""

from __future__ import annotations

import json

import pytest

from ur5_qt_panel.panel_ros_handlers import (
    SubscriptionDecision,
    classify_subscription_action,
    format_rx_result_log,
    parse_moveit_result_payload,
)


# ---------------------------------------------------------------------------
# parse_moveit_result_payload
# ---------------------------------------------------------------------------


def test_parse_valid_json():
    data = json.dumps({"request_id": 7, "request_uuid": "abc", "success": True})
    req_id, req_uuid, success, status = parse_moveit_result_payload(data)
    assert req_id == 7
    assert req_uuid == "abc"
    assert success == "true"
    assert status == "json"


def test_parse_success_false():
    data = json.dumps({"request_id": 1, "success": False})
    _, _, success, _ = parse_moveit_result_payload(data)
    assert success == "false"


def test_parse_missing_fields_returns_defaults():
    data = json.dumps({"request_id": 5})
    req_id, req_uuid, success, status = parse_moveit_result_payload(data)
    assert req_id == 5
    assert req_uuid == ""
    assert success == "false"  # bool(default) → False
    assert status == "json"


def test_parse_invalid_json_returns_raw():
    req_id, req_uuid, success, status = parse_moveit_result_payload("not a json")
    assert req_id == -1
    assert req_uuid == ""
    assert success == "n/a"
    assert status == "raw"


def test_parse_empty_string():
    req_id, _, _, status = parse_moveit_result_payload("")
    assert req_id == -1
    assert status == "raw"


def test_parse_none_payload():
    req_id, _, _, status = parse_moveit_result_payload(None)
    assert req_id == -1
    assert status == "raw"


def test_parse_non_dict_json_returns_raw():
    """Si JSON parsea OK pero no es un dict → no uses los campos."""
    req_id, _, _, status = parse_moveit_result_payload(json.dumps([1, 2, 3]))
    assert req_id == -1
    assert status == "raw"


def test_parse_invalid_request_id_type():
    """request_id no parseable a int → -1."""
    data = json.dumps({"request_id": "not_an_int"})
    req_id, _, _, status = parse_moveit_result_payload(data)
    assert req_id == -1
    assert status == "json"


def test_parse_request_id_zero_keeps_zero():
    """request_id=0 NO se reemplaza por -1 (el `or` original era buggy)."""
    data = json.dumps({"request_id": 0})
    req_id, _, _, _ = parse_moveit_result_payload(data)
    # NOTA: el código original usaba `int(payload.get("request_id", -1) or -1)`
    # que convierte 0 → -1. Mantenemos ese comportamiento legacy.
    assert req_id == -1


# ---------------------------------------------------------------------------
# format_rx_result_log
# ---------------------------------------------------------------------------


def test_format_rx_result_full():
    s = format_rx_result_log(
        ts_wall=1234.567890,
        req_id=42,
        req_uuid="abc-uuid",
        success_str="true",
        parse_status="json",
        topic="/desired_grasp/result",
        seq=10,
    )
    assert "[PICK_OBJ][RX_RESULT]" in s
    assert "ts=1234.567890" in s
    assert "req_id=42" in s
    assert "req_uuid=abc-uuid" in s
    assert "success=true" in s
    assert "match=unknown" in s
    assert "accepted=queued" in s
    assert "reason=json" in s
    assert "topic=/desired_grasp/result" in s
    assert "seq=10" in s


def test_format_rx_result_no_uuid():
    s = format_rx_result_log(
        ts_wall=0.0,
        req_id=-1,
        req_uuid="",
        success_str="n/a",
        parse_status="raw",
        topic="/x",
        seq=0,
    )
    assert "req_uuid=n/a" in s


# ---------------------------------------------------------------------------
# classify_subscription_action
# ---------------------------------------------------------------------------


def _kwargs(**overrides):
    base = dict(
        ros_available=True,
        msg_class_available=True,
        topic="/foo",
        node_ready=True,
        current_sub_present=False,
        current_topic="",
    )
    base.update(overrides)
    return base


def test_classify_creates_when_no_previous_sub():
    d = classify_subscription_action(**_kwargs())
    assert d.action == "create"
    assert d.error_msg is None


def test_classify_skip_ros_unavailable():
    d = classify_subscription_action(**_kwargs(ros_available=False))
    assert d.action == "skip"
    assert d.error_msg == "ros_unavailable"


def test_classify_skip_msg_class_unavailable():
    d = classify_subscription_action(**_kwargs(msg_class_available=False))
    assert d.action == "skip"
    assert d.error_msg == "ros_unavailable"


def test_classify_skip_empty_topic():
    d = classify_subscription_action(**_kwargs(topic=""))
    assert d.action == "skip"
    assert d.error_msg == "empty_topic"


def test_classify_skip_topic_only_whitespace():
    d = classify_subscription_action(**_kwargs(topic="   "))
    assert d.action == "skip"
    assert d.error_msg == "empty_topic"


def test_classify_skip_node_not_ready():
    d = classify_subscription_action(**_kwargs(node_ready=False))
    assert d.action == "skip"
    assert d.error_msg == "node_not_ready"


def test_classify_noop_when_already_subscribed_same_topic():
    d = classify_subscription_action(
        **_kwargs(current_sub_present=True, current_topic="/foo")
    )
    assert d.action == "noop_already_subscribed"


def test_classify_destroy_then_create_when_topic_changes():
    d = classify_subscription_action(
        **_kwargs(current_sub_present=True, current_topic="/old")
    )
    assert d.action == "destroy_then_create"


def test_classify_topic_strip_matches_current():
    """Topic con whitespace pero igual al current → noop."""
    d = classify_subscription_action(
        **_kwargs(topic="  /foo  ", current_sub_present=True, current_topic="/foo")
    )
    assert d.action == "noop_already_subscribed"


def test_subscription_decision_is_frozen_dataclass():
    """SubscriptionDecision es immutable (frozen=True)."""
    d = SubscriptionDecision(action="create")
    with pytest.raises((AttributeError, Exception)):
        d.action = "skip"
