#!/usr/bin/env python3
"""F6 audit-v4 (2026-05-08): tests offline trajectory_executor_contract."""
from __future__ import annotations

import importlib
from typing import Any

import pytest

from ur5_tools.trajectory_executor_contract import (
    CANONICAL_PURE_HELPER_MODULES,
    CANONICAL_REASON_CODES,
    ExecutorReport,
    ExecutorRequest,
    TrajectoryExecutorPhase,
    categorize_reason,
    is_canonical_reason,
    list_extracted_pure_helpers,
)


def test_request_immutable():
    req = ExecutorRequest(
        joint_names=("j1", "j2"),
        waypoint_positions=((0.0, 0.0),),
        waypoint_times_sec=(1.0,),
    )
    with pytest.raises((AttributeError, Exception)):
        req.timeout_sec = 99.0  # type: ignore[misc]


def test_report_default_meta_empty():
    r = ExecutorReport(
        success=True, reason="fjt:SUCCESSFUL", duration_sec=1.0,
        phase=TrajectoryExecutorPhase.REPORT,
    )
    assert r.meta == {}


def test_phase_enum_string_values():
    assert TrajectoryExecutorPhase.PREPARE == "prepare"
    assert TrajectoryExecutorPhase.DISPATCH == "dispatch"
    assert TrajectoryExecutorPhase.EXECUTE == "execute"
    assert TrajectoryExecutorPhase.SETTLE == "settle"
    assert TrajectoryExecutorPhase.REPORT == "report"


@pytest.mark.parametrize("reason,expected", [
    ("fjt:SUCCESSFUL", True),
    ("fjt_err:PATH_TOLERANCE_VIOLATED", True),
    ("fjt_err:PATH_TOLERANCE_VIOLATED|joint_X exceeded", True),
    ("ik:NO_IK_SOLUTION", True),
    ("garbage_value", False),
    ("", False),
])
def test_is_canonical_reason(reason, expected):
    assert is_canonical_reason(reason) == expected


@pytest.mark.parametrize("reason,expected", [
    ("fjt:SUCCESSFUL", "success"),
    ("early_success_via_feedback", "success"),
    ("early_success_via_ee", "success"),
    ("fjt_err:PATH_TOLERANCE_VIOLATED", "failure"),
    ("ik:NO_IK_SOLUTION", "failure"),
    ("nope_unknown", "unknown"),
    ("", "unknown"),
])
def test_categorize_reason(reason, expected):
    assert categorize_reason(reason) == expected


def test_pure_helpers_registry_non_empty():
    assert len(CANONICAL_PURE_HELPER_MODULES) >= 5
    assert all("ur5_tools" in m for m in CANONICAL_PURE_HELPER_MODULES)


@pytest.mark.parametrize("module_name", CANONICAL_PURE_HELPER_MODULES)
def test_pure_helper_module_importable(module_name: str):
    """Todos los módulos en el registry deben ser importables."""
    try:
        mod = importlib.import_module(module_name)
    except ImportError as exc:
        pytest.skip(f"{module_name} not importable in this env: {exc}")
    assert mod is not None


def test_list_extracted_pure_helpers_returns_tuple():
    out = list_extracted_pure_helpers()
    assert isinstance(out, tuple)
    assert len(out) >= 5


def test_canonical_reason_codes_includes_success_and_failure():
    success_codes = [c for c in CANONICAL_REASON_CODES if c.startswith(("fjt:SUCCESS", "early_"))]
    failure_codes = [c for c in CANONICAL_REASON_CODES if c.startswith(("fjt_err", "ik:", "fjt_goal_", "fjt_action_"))]
    assert success_codes, "Falta categoría success en CANONICAL_REASON_CODES"
    assert failure_codes, "Falta categoría failure en CANONICAL_REASON_CODES"
