#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/test/test_grasp_selector.py
"""F17 (2026-05-08) — Tests offline para grasp_selector.

Cubre todas las funciones puras del selector:
- is_tcp_down (cuaternión TCP-down detection)
- filter_by_workspace
- filter_by_tcp_down
- rank_by_score_and_distance
- select_best_grasp (pipeline completo)
"""

from __future__ import annotations

import math

import pytest

from tfm_grasping.grasp_selector import (
    DEFAULT_TCP_DOWN_TOLERANCE_RAD,
    DEFAULT_WS_RADIUS_M,
    GraspCandidate,
    SelectionResult,
    filter_by_tcp_down,
    filter_by_workspace,
    is_tcp_down,
    rank_by_score_and_distance,
    select_best_grasp,
)

# Quat TCP-down canónico (eje Z apunta -world_Z).
# Roll π sobre eje X: [sin(π/2), 0, 0, cos(π/2)] = [1, 0, 0, 0]
TCP_DOWN_QUAT = (1.0, 0.0, 0.0, 0.0)
# Identity quat: TCP apunta hacia +world_Z (no down).
IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)
# Quat tilteado 30° desde TCP-down (eje Z del TCP apunta a 30° de -world_Z).
# Roll(210°) sobre X: quat = (sin(105°), 0, 0, cos(105°)).
ALMOST_DOWN_30DEG = (
    math.sin(math.pi * 7.0 / 12.0),
    0.0,
    0.0,
    math.cos(math.pi * 7.0 / 12.0),
)


# ---------------------------------------------------------------------------
# is_tcp_down
# ---------------------------------------------------------------------------


def test_tcp_down_pure_roll_pi_is_down():
    """Roll π sobre X: TCP Z apunta -Z global → True."""
    assert is_tcp_down(TCP_DOWN_QUAT) is True


def test_tcp_down_identity_is_not_down():
    """Identity quat: TCP Z apunta +Z global → False."""
    assert is_tcp_down(IDENTITY_QUAT) is False


def test_tcp_down_with_custom_tolerance():
    """Quat con desviación 30° (~0.52 rad) cae fuera de tolerancia 0.20 default."""
    assert is_tcp_down(ALMOST_DOWN_30DEG, tolerance_rad=0.20) is False


def test_tcp_down_with_wide_tolerance():
    """Misma quat dentro de tolerancia 1.0 rad (~57°)."""
    assert is_tcp_down(ALMOST_DOWN_30DEG, tolerance_rad=1.0) is True


# ---------------------------------------------------------------------------
# filter_by_workspace
# ---------------------------------------------------------------------------


def test_filter_workspace_within_bounds():
    cands = [
        GraspCandidate(xyz=(0.5, 0.0, 0.3), quat_xyzw=TCP_DOWN_QUAT, score=0.9),
        GraspCandidate(xyz=(0.4, 0.2, 0.2), quat_xyzw=TCP_DOWN_QUAT, score=0.8),
    ]
    out = filter_by_workspace(cands)
    assert len(out) == 2


def test_filter_workspace_too_far():
    """Distance 1.5m > 0.85 default → filtrado."""
    cands = [
        GraspCandidate(xyz=(1.5, 0.0, 0.0), quat_xyzw=TCP_DOWN_QUAT, score=0.9),
    ]
    assert filter_by_workspace(cands) == []


def test_filter_workspace_too_close():
    """Distance 0.10 < 0.20 default → filtrado (singularity zone)."""
    cands = [
        GraspCandidate(xyz=(0.05, 0.0, 0.05), quat_xyzw=TCP_DOWN_QUAT, score=0.9),
    ]
    assert filter_by_workspace(cands) == []


def test_filter_workspace_custom_base():
    """Base no en origen → distancia se mide desde base custom."""
    cands = [
        GraspCandidate(xyz=(1.5, 0.0, 0.0), quat_xyzw=TCP_DOWN_QUAT, score=0.9),
    ]
    # Base en (1.0, 0, 0): distance al cand = 0.5 → dentro
    out = filter_by_workspace(cands, base_xyz=(1.0, 0.0, 0.0))
    assert len(out) == 1


# ---------------------------------------------------------------------------
# filter_by_tcp_down
# ---------------------------------------------------------------------------


def test_filter_tcp_down_keeps_only_down():
    cands = [
        GraspCandidate(xyz=(0.5, 0.0, 0.3), quat_xyzw=TCP_DOWN_QUAT, score=0.9),
        GraspCandidate(xyz=(0.5, 0.1, 0.3), quat_xyzw=IDENTITY_QUAT, score=0.8),
    ]
    out = filter_by_tcp_down(cands)
    assert len(out) == 1
    assert out[0].quat_xyzw == TCP_DOWN_QUAT


# ---------------------------------------------------------------------------
# rank_by_score_and_distance
# ---------------------------------------------------------------------------


def test_rank_higher_score_first_when_distances_equal():
    cands = [
        GraspCandidate(xyz=(0.5, 0.0, 0.3), quat_xyzw=TCP_DOWN_QUAT, score=0.5, label="a"),
        GraspCandidate(xyz=(0.5, 0.0, 0.3), quat_xyzw=TCP_DOWN_QUAT, score=0.9, label="b"),
    ]
    ranked = rank_by_score_and_distance(cands)
    assert ranked[0].label == "b"


def test_rank_closer_first_when_scores_equal():
    cands = [
        GraspCandidate(xyz=(0.7, 0.0, 0.3), quat_xyzw=TCP_DOWN_QUAT, score=0.5, label="far"),
        GraspCandidate(xyz=(0.4, 0.0, 0.3), quat_xyzw=TCP_DOWN_QUAT, score=0.5, label="close"),
    ]
    ranked = rank_by_score_and_distance(cands)
    assert ranked[0].label == "close"


def test_rank_empty_returns_empty():
    assert rank_by_score_and_distance([]) == []


# ---------------------------------------------------------------------------
# select_best_grasp pipeline
# ---------------------------------------------------------------------------


def test_select_no_input_returns_no_input_reason():
    result = select_best_grasp([])
    assert result.best is None
    assert result.reason == "no_input"
    assert result.n_input == 0


def test_select_filtered_by_workspace():
    cands = [
        GraspCandidate(xyz=(2.0, 0.0, 0.3), quat_xyzw=TCP_DOWN_QUAT, score=0.9),
    ]
    result = select_best_grasp(cands)
    assert result.best is None
    assert result.reason == "filtered_workspace"
    assert result.n_input == 1
    assert result.n_after_workspace == 0


def test_select_filtered_by_tcp_down():
    cands = [
        GraspCandidate(xyz=(0.5, 0.0, 0.3), quat_xyzw=IDENTITY_QUAT, score=0.9),
    ]
    result = select_best_grasp(cands)
    assert result.best is None
    assert result.reason == "filtered_tcp_down"
    assert result.n_after_workspace == 1
    assert result.n_after_tcp_down == 0


def test_select_returns_best_when_all_pass():
    cands = [
        GraspCandidate(xyz=(0.5, 0.0, 0.3), quat_xyzw=TCP_DOWN_QUAT, score=0.5, label="low"),
        GraspCandidate(xyz=(0.5, 0.1, 0.3), quat_xyzw=TCP_DOWN_QUAT, score=0.9, label="high"),
    ]
    result = select_best_grasp(cands)
    assert result.best is not None
    assert result.best.label == "high"
    assert result.reason == "ok"
    assert result.n_input == 2
    assert result.n_after_workspace == 2
    assert result.n_after_tcp_down == 2


def test_select_with_require_tcp_down_false_keeps_all():
    cands = [
        GraspCandidate(xyz=(0.5, 0.0, 0.3), quat_xyzw=IDENTITY_QUAT, score=0.9),
    ]
    result = select_best_grasp(cands, require_tcp_down=False)
    assert result.best is not None
    assert result.reason == "ok"


def test_select_with_custom_workspace_radius():
    cands = [
        GraspCandidate(xyz=(1.2, 0.0, 0.3), quat_xyzw=TCP_DOWN_QUAT, score=0.9),
    ]
    # ws_radius=1.5 ⇒ pasa workspace
    result = select_best_grasp(cands, ws_radius_m=1.5)
    assert result.best is not None
