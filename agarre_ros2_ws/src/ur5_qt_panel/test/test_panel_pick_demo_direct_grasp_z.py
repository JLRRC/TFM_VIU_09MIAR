#!/usr/bin/env python3
"""Pruebas unitarias para la normalización del offset Z de DIRECTO."""
from __future__ import annotations

import pytest

from ur5_qt_panel.panel_pick_demo import _effective_direct_grasp_z


def test_rg2_pinch_center_forces_zero_grasp_offset() -> None:
    assert _effective_direct_grasp_z("rg2_pinch_center", 0.05) == pytest.approx(0.0)


def test_legacy_frames_keep_requested_grasp_offset() -> None:
    assert _effective_direct_grasp_z("rg2_tcp", 0.05) == pytest.approx(0.05)
