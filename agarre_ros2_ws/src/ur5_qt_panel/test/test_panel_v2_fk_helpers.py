#!/usr/bin/env python3
"""F12 (auditoría 2026-05-10): tests offline de panel_v2_fk_helpers."""
from __future__ import annotations

import numpy as np
import pytest

from ur5_qt_panel.panel_v2_fk_helpers import (
    canonical_tool0_to_semantic_frame,
    fk_model_to_base_link,
    fk_tool0_to_ee_base_link,
)


# ---------------- canonical_tool0_to_semantic_frame ----------------


def _stub_offset(frame):
    return {
        "rg2_pinch_center": (0.0, 0.0, 0.175),
        "rg2_tcp": (0.0, 0.0, 0.175),
    }.get(frame)


def test_canonical_returns_offset_for_pinch_center():
    out = canonical_tool0_to_semantic_frame("rg2_pinch_center", _stub_offset)
    assert out == (0.0, 0.0, 0.175)


def test_canonical_returns_offset_for_rg2_tcp():
    out = canonical_tool0_to_semantic_frame("rg2_tcp", _stub_offset)
    assert out == (0.0, 0.0, 0.175)


def test_canonical_returns_none_for_unknown_frame():
    assert canonical_tool0_to_semantic_frame("tool0", _stub_offset) is None
    assert canonical_tool0_to_semantic_frame("base_link", _stub_offset) is None
    assert canonical_tool0_to_semantic_frame("", _stub_offset) is None


def test_canonical_strips_whitespace():
    out = canonical_tool0_to_semantic_frame("  rg2_tcp  ", _stub_offset)
    assert out == (0.0, 0.0, 0.175)


# ---------------- fk_model_to_base_link ----------------


def test_fk_model_to_base_link_inverts_xy_signs():
    pos_model = (1.0, 2.0, 3.0)
    rot_model = np.eye(3)
    base_pos, base_rot = fk_model_to_base_link(pos_model, rot_model)
    assert base_pos == (-1.0, -2.0, 3.0)
    # rz_pi @ I = rz_pi
    expected = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], dtype=float)
    np.testing.assert_array_almost_equal(base_rot, expected)


def test_fk_model_origin():
    """Origen del modelo → origen base (X/Y invertidos pero 0)."""
    base_pos, _ = fk_model_to_base_link((0.0, 0.0, 0.0), np.eye(3))
    assert base_pos == (0.0, 0.0, 0.0)


def test_fk_model_accepts_list():
    base_pos, _ = fk_model_to_base_link([0.5, -0.3, 0.05], np.eye(3))
    assert base_pos == (-0.5, 0.3, 0.05)


def test_fk_model_accepts_numpy():
    base_pos, _ = fk_model_to_base_link(
        np.array([0.5, -0.3, 0.05]), np.eye(3)
    )
    assert base_pos == (-0.5, 0.3, 0.05)


def test_fk_model_rotates_full_rot_correctly():
    """rz_pi composición de rotación del modelo."""
    # Rot del modelo: identidad → rot base = rz_pi
    _, base_rot = fk_model_to_base_link((0, 0, 0), np.eye(3))
    expected = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]], dtype=float)
    np.testing.assert_array_almost_equal(base_rot, expected)


# ---------------- fk_tool0_to_ee_base_link ----------------


def test_fk_tool0_no_ee_frame_returns_tool0():
    pos = (1.0, 2.0, 3.0)
    rot = np.eye(3)
    out_pos, _ = fk_tool0_to_ee_base_link(pos, rot, "")
    # mismo que fk_model_to_base_link
    assert out_pos == (-1.0, -2.0, 3.0)


def test_fk_tool0_explicit_tool0_returns_tool0():
    out_pos, _ = fk_tool0_to_ee_base_link((0, 0, 0), np.eye(3), "tool0")
    assert out_pos == (0.0, 0.0, 0.0)


def test_fk_tool0_uses_tf_lookup_when_available():
    """Si TF lookup devuelve offset, lo prefiere sobre canonical."""
    def tf_lookup(parent, child):
        assert parent == "tool0"
        assert child == "rg2_pinch_center"
        return (0.0, 0.0, 0.20)  # offset distinto al canonical

    out_pos, _ = fk_tool0_to_ee_base_link(
        (0.0, 0.0, 0.0), np.eye(3), "rg2_pinch_center",
        tf_lookup=tf_lookup,
        canonical_offset=_stub_offset,
    )
    # rz_pi @ (0, 0, 0.20) = (0, 0, 0.20). Aplicado al base_pos (0,0,0):
    assert out_pos[2] == pytest.approx(0.20)


def test_fk_tool0_falls_back_to_canonical_when_tf_returns_none():
    def tf_lookup(parent, child):
        return None

    out_pos, _ = fk_tool0_to_ee_base_link(
        (0.0, 0.0, 0.0), np.eye(3), "rg2_tcp",
        tf_lookup=tf_lookup,
        canonical_offset=_stub_offset,
    )
    # canonical = (0, 0, 0.175)
    assert out_pos[2] == pytest.approx(0.175)


def test_fk_tool0_falls_back_to_canonical_when_tf_raises():
    def tf_lookup(parent, child):
        raise RuntimeError("tf_unavailable")

    out_pos, _ = fk_tool0_to_ee_base_link(
        (0.0, 0.0, 0.0), np.eye(3), "rg2_tcp",
        tf_lookup=tf_lookup,
        canonical_offset=_stub_offset,
    )
    assert out_pos[2] == pytest.approx(0.175)


def test_fk_tool0_returns_tool0_pose_if_no_offset_resolvable():
    """Sin TF ni canonical → devuelve la pose tool0 sin modificar."""
    out_pos, _ = fk_tool0_to_ee_base_link(
        (1.0, 2.0, 3.0), np.eye(3), "unknown_frame",
        tf_lookup=lambda p, c: None,
        canonical_offset=lambda f: None,
    )
    assert out_pos == (-1.0, -2.0, 3.0)


def test_fk_tool0_no_callbacks_returns_tool0():
    """Sin tf_lookup ni canonical_offset → tool0 puro."""
    out_pos, _ = fk_tool0_to_ee_base_link(
        (0.5, 0.0, 0.0), np.eye(3), "rg2_tcp",
    )
    assert out_pos == (-0.5, 0.0, 0.0)
