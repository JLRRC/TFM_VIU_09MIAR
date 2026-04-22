#!/usr/bin/env python3
"""Pruebas de regresión para los valores por defecto del panel."""

from ur5_qt_panel.panel_settings import PanelSettings


def test_panel_settings_default_tcp_offset_is_zero(monkeypatch):
    monkeypatch.delenv("PANEL_GRIPPER_TCP_Z_OFFSET", raising=False)
    monkeypatch.delenv("PANEL_SETTINGS_YAML", raising=False)

    settings = PanelSettings.from_env()

    assert settings.gripper_tcp_z_offset == 0.0
