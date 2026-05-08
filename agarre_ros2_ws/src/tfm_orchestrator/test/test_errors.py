#!/usr/bin/env python3
"""Audit-v4 (2026-05-08): tests offline para errors typed hierarchy."""
from __future__ import annotations

import pytest

from tfm_orchestrator.errors import (
    ApproachError,
    AttachError,
    ConfigError,
    ControllerNotReadyError,
    EnvVarError,
    ExecutionError,
    GraspDownError,
    GraspError,
    GripperError,
    HardwareError,
    HomeError,
    LiftError,
    MotionError,
    PhaseTransitionError,
    PickError,
    PlanningError,
    PoseConsistencyError,
    ReleaseError,
    TFMError,
    TimeoutError,
    TransportError,
    YAMLError,
)


def test_tfm_error_base():
    err = TFMError("test message", reason="custom")
    assert str(err) == "test message"
    assert err.reason == "custom"
    assert err.context == {}


def test_tfm_error_default_reason_is_class_name():
    err = TFMError("test")
    assert err.reason == "TFMError"


def test_tfm_error_to_dict():
    err = TFMError("msg", reason="r1", context={"k": "v"})
    d = err.to_dict()
    assert d["type"] == "TFMError"
    assert d["reason"] == "r1"
    assert d["message"] == "msg"
    assert d["context"] == {"k": "v"}


def test_pick_error_carries_phase():
    err = PickError("approach failed", phase="APPROACH", reason="ompl_failure")
    assert err.phase == "APPROACH"
    assert err.reason == "ompl_failure"
    assert err.context["phase"] == "APPROACH"


def test_pick_error_default_reason():
    err = PickError("no args")
    assert err.reason == "pick_error"


@pytest.mark.parametrize("cls,parent", [
    (PhaseTransitionError, PickError),
    (PoseConsistencyError, PickError),
    (ApproachError, PickError),
    (GraspDownError, PickError),
    (GraspError, PickError),
    (LiftError, PickError),
    (TransportError, PickError),
    (ReleaseError, PickError),
    (HomeError, PickError),
    (PickError, TFMError),
    (PlanningError, MotionError),
    (ExecutionError, MotionError),
    (TimeoutError, MotionError),
    (MotionError, TFMError),
    (GripperError, HardwareError),
    (AttachError, HardwareError),
    (ControllerNotReadyError, HardwareError),
    (HardwareError, TFMError),
    (EnvVarError, ConfigError),
    (YAMLError, ConfigError),
    (ConfigError, TFMError),
])
def test_hierarchy(cls, parent):
    assert issubclass(cls, parent)
    assert issubclass(cls, TFMError)


def test_can_catch_specific_as_base():
    """PickError debería poderse atrapar como TFMError."""
    try:
        raise GraspError("close failed", reason="opening_too_wide")
    except TFMError as exc:
        assert exc.reason == "opening_too_wide"
        assert isinstance(exc, GraspError)
        assert isinstance(exc, PickError)
    else:
        pytest.fail("Should have raised")


def test_typed_error_serializable_for_evidence_logger():
    """to_dict() debe ser JSON-serializable."""
    import json
    err = TransportError(
        "stuck", reason="wrong_direction", phase="TRANSPORT",
        context={"distance_m": 0.5},
    )
    d = err.to_dict()
    s = json.dumps(d)
    parsed = json.loads(s)
    assert parsed["type"] == "TransportError"
    assert parsed["context"]["phase"] == "TRANSPORT"
    assert parsed["context"]["distance_m"] == 0.5
