from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent
SRC_DIR = ROOT / "src" / "ur5_qt_panel"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from ur5_qt_panel.panel_settings import PanelSettings


def test_infer_roi_disabled_by_default(monkeypatch):
    monkeypatch.delenv("INFER_ROI_SIZE", raising=False)

    settings = PanelSettings.from_env()

    assert settings.infer_roi_size == 0


def test_infer_roi_env_override_still_supported(monkeypatch):
    monkeypatch.setenv("INFER_ROI_SIZE", "160")

    settings = PanelSettings.from_env()

    assert settings.infer_roi_size == 160


def test_infer_ckpt_env_override_still_supported(monkeypatch):
    custom_ckpt = "/tmp/custom/best.pth"
    monkeypatch.setenv("INFER_CKPT", custom_ckpt)

    settings = PanelSettings.from_env()

    assert settings.infer_ckpt == custom_ckpt