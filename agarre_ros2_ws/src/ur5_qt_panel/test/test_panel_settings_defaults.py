#!/usr/bin/env python3
"""Pruebas de regresión para eliminar offsets TCP legacy del panel."""

from pathlib import Path

from ur5_qt_panel.panel_settings import PanelSettings


def test_panel_settings_no_longer_exposes_legacy_tcp_offset(monkeypatch):
    monkeypatch.delenv("PANEL_GRIPPER_TCP_Z_OFFSET", raising=False)
    monkeypatch.delenv("PANEL_SETTINGS_YAML", raising=False)

    settings = PanelSettings.from_env()

    assert not hasattr(settings, "gripper_tcp_z_offset")


def test_panel_settings_ignores_nonzero_tcp_offset_env(monkeypatch):
    monkeypatch.setenv("PANEL_GRIPPER_TCP_Z_OFFSET", "0.175")
    monkeypatch.delenv("PANEL_SETTINGS_YAML", raising=False)

    settings = PanelSettings.from_env()

    assert not hasattr(settings, "gripper_tcp_z_offset")


def test_panel_settings_ignores_nonzero_tcp_offset_yaml(monkeypatch, tmp_path: Path):
    yaml_path = tmp_path / "panel_settings.yaml"
    yaml_path.write_text("panel_settings:\n  gripper_tcp_z_offset: 0.175\n", encoding="utf-8")
    monkeypatch.delenv("PANEL_GRIPPER_TCP_Z_OFFSET", raising=False)
    monkeypatch.setenv("PANEL_SETTINGS_YAML", str(yaml_path))

    settings = PanelSettings.from_env()

    assert not hasattr(settings, "gripper_tcp_z_offset")
