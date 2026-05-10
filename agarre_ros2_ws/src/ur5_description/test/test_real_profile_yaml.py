#!/usr/bin/env python3
"""F14 audit (2026-05-10): perfil hardware real ur5_controllers_real.yaml.

Verifica que el perfil de hardware real existe, es YAML válido y tiene
las tolerancias adecuadas (más estrictas que el perfil de sim).
"""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parent.parent
SIM_YAML = PKG / "config" / "ur5_controllers.yaml"
REAL_YAML = PKG / "config" / "ur5_controllers_real.yaml"


def _load(p: Path) -> dict:
    return yaml.safe_load(p.read_text(encoding="utf-8"))


def test_real_profile_exists() -> None:
    assert REAL_YAML.is_file(), (
        "ur5_controllers_real.yaml debe existir como perfil F14 para hardware real"
    )


def test_real_profile_loads_as_yaml() -> None:
    data = _load(REAL_YAML)
    assert isinstance(data, dict)
    assert "controller_manager" in data
    assert "joint_trajectory_controller" in data


def test_real_profile_uses_measured_state() -> None:
    """En hardware real, interpolate_from_desired_state debe ser False."""
    data = _load(REAL_YAML)
    jtc = data["joint_trajectory_controller"]["ros__parameters"]
    assert jtc["interpolate_from_desired_state"] is False, (
        "Real profile debe usar measured state (False); sim usa True por drift physics"
    )


def test_real_profile_has_strict_tolerances_vs_sim() -> None:
    sim = _load(SIM_YAML)["joint_trajectory_controller"]["ros__parameters"]
    real = _load(REAL_YAML)["joint_trajectory_controller"]["ros__parameters"]
    # goal_time real < goal_time sim (sim usa 300s por sim_per_wall ≈ 0.58)
    assert real["constraints"]["goal_time"] < sim["constraints"]["goal_time"], (
        f"goal_time real={real['constraints']['goal_time']} debería ser "
        f"< sim={sim['constraints']['goal_time']}"
    )
    # stopped_velocity_tolerance real < sim
    assert (
        real["constraints"]["stopped_velocity_tolerance"]
        < sim["constraints"]["stopped_velocity_tolerance"]
    ), "stopped_velocity_tolerance real debe ser más estricta que sim"


@pytest.mark.parametrize("joint", [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
])
def test_real_profile_per_joint_tolerances_sane(joint: str) -> None:
    """Por joint, tolerancias real más estrictas que sim."""
    sim = _load(SIM_YAML)["joint_trajectory_controller"]["ros__parameters"]
    real = _load(REAL_YAML)["joint_trajectory_controller"]["ros__parameters"]
    sim_goal = sim["constraints"][joint]["goal"]
    real_goal = real["constraints"][joint]["goal"]
    assert real_goal < sim_goal, (
        f"{joint}.goal real={real_goal} debería ser < sim={sim_goal}"
    )


def test_real_profile_keeps_update_rate_125() -> None:
    """UR5 CB-series: 125 Hz nominal. NO 500 (e-series)."""
    data = _load(REAL_YAML)
    rate = data["controller_manager"]["ros__parameters"]["update_rate"]
    assert rate == 125, (
        f"update_rate={rate}: UR5 CB-series usa 125 Hz. Si migras a "
        "e-series (500 Hz), actualiza también docs/SIM_VS_REAL.md."
    )
