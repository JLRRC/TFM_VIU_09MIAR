"""Smoke import test: every package module imports without error.

Prevents regressions where a refactor breaks `from tfm_orchestrator import X`
without surfacing immediately. Runs offline (no rclpy init).

Audit 2026-05-10 (Action 7).
"""
from __future__ import annotations

import importlib

import pytest

MODULES = [
    "tfm_orchestrator",
    "tfm_orchestrator.cartesian_segments",
    "tfm_orchestrator.errors",
    "tfm_orchestrator.gripper_monitor",
    "tfm_orchestrator.home_initial",
    "tfm_orchestrator.initial_snapshot",
    "tfm_orchestrator.lifecycle_helpers",
    "tfm_orchestrator.phase_dispatch",
    "tfm_orchestrator.phase_progress",
    "tfm_orchestrator.phase_timings",
    "tfm_orchestrator.pick_fsm",
    "tfm_orchestrator.pick_gates",
    "tfm_orchestrator.pose_consistency",
    "tfm_orchestrator.preflight",
    "tfm_orchestrator.retry",
    "tfm_orchestrator.service_clients",
]


@pytest.mark.parametrize("module_name", MODULES)
def test_module_imports(module_name: str) -> None:
    importlib.import_module(module_name)
