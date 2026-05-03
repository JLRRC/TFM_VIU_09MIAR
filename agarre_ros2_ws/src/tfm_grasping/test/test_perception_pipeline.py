"""F4: cobertura de tfm_grasping.perception.PerceptionPipeline.

Tests puros sobre las transformaciones HWC→CHW y normalización depth.
Sólo se ejecutan si numpy está disponible (es lo normal en este workspace).
cv2 puede no estar instalado en CI offline; en tal caso el método devuelve
None y nosotros validamos ese contrato.
"""

from __future__ import annotations

import pytest

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None  # type: ignore

from tfm_grasping.perception import InputFrame, PerceptionPipeline


pytestmark = pytest.mark.skipif(np is None, reason="numpy no disponible")


def _has_cv2() -> bool:
    try:
        import cv2  # noqa: F401
        return True
    except Exception:
        return False


def test_input_frame_construction_minimal():
    img = np.zeros((10, 10, 3), dtype=np.uint8)
    f = InputFrame(image=img, width=10, height=10, timestamp=1.5)
    assert f.image is img
    assert f.width == 10
    assert f.height == 10
    assert f.timestamp == 1.5
    assert f.roi is None
    assert f.preprocessed is False


def test_input_frame_with_roi():
    img = np.zeros((10, 10, 3), dtype=np.uint8)
    f = InputFrame(image=img, width=10, height=10, timestamp=0.0, roi=(2, 3, 4))
    assert f.roi == (2, 3, 4)


def test_input_frame_preprocessed_flag():
    img = np.zeros((10, 10, 3), dtype=np.uint8)
    f = InputFrame(
        image=img, width=10, height=10, timestamp=0.0, preprocessed=True
    )
    assert f.preprocessed is True


@pytest.mark.skipif(not _has_cv2(), reason="cv2 no disponible")
def test_to_preprocessed_returns_chw_float32():
    img = np.full((224, 224, 3), 128, dtype=np.uint8)
    out = PerceptionPipeline.to_preprocessed(img, img_size=224)
    assert out is not None
    assert out.dtype == np.float32
    assert out.shape == (3, 224, 224)
    assert out.flags["C_CONTIGUOUS"]
    # 128/255 ~= 0.502
    assert abs(float(out.mean()) - 128.0 / 255.0) < 1e-3


@pytest.mark.skipif(not _has_cv2(), reason="cv2 no disponible")
def test_to_preprocessed_resizes_to_target_size():
    img = np.zeros((100, 200, 3), dtype=np.uint8)
    out = PerceptionPipeline.to_preprocessed(img, img_size=224)
    assert out is not None
    assert out.shape == (3, 224, 224)


def test_to_preprocessed_returns_none_for_invalid_input():
    # No es np.ndarray
    assert PerceptionPipeline.to_preprocessed("not an array") is None  # type: ignore[arg-type]
    # Dimensión incorrecta
    assert PerceptionPipeline.to_preprocessed(np.zeros((10, 10), dtype=np.uint8)) is None


def test_to_preprocessed_returns_none_for_few_channels():
    img = np.zeros((10, 10, 2), dtype=np.uint8)
    assert PerceptionPipeline.to_preprocessed(img) is None


@pytest.mark.skipif(not _has_cv2(), reason="cv2 no disponible")
def test_normalize_depth_returns_float32_normalized():
    depth = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
    out = PerceptionPipeline.normalize_depth(depth, img_size=4)
    assert out is not None
    assert out.dtype == np.float32
    assert out.shape == (4, 4)
    # Max original = 4.0; normalizado < 1.0
    assert float(out.max()) <= 1.0


@pytest.mark.skipif(not _has_cv2(), reason="cv2 no disponible")
def test_normalize_depth_handles_3d_input():
    depth = np.ones((4, 4, 3), dtype=np.float32)
    out = PerceptionPipeline.normalize_depth(depth, img_size=4)
    assert out is not None
    assert out.shape == (4, 4)


@pytest.mark.skipif(not _has_cv2(), reason="cv2 no disponible")
def test_normalize_depth_handles_nan_inf():
    depth = np.array(
        [[1.0, float("nan")], [float("inf"), 2.0]], dtype=np.float32
    )
    out = PerceptionPipeline.normalize_depth(depth, img_size=4)
    assert out is not None
    # NaN / inf reemplazados por 0
    assert bool(np.isfinite(out).all())


def test_normalize_depth_returns_none_for_wrong_dim():
    depth = np.zeros((4,), dtype=np.float32)
    assert PerceptionPipeline.normalize_depth(depth) is None


def test_normalize_depth_returns_none_for_non_array():
    assert PerceptionPipeline.normalize_depth("not array") is None  # type: ignore[arg-type]


@pytest.mark.skipif(not _has_cv2(), reason="cv2 no disponible")
def test_to_preprocessed_rgbd_concatenates_4_channels():
    img = np.full((224, 224, 3), 100, dtype=np.uint8)
    depth = np.ones((224, 224), dtype=np.float32)
    out = PerceptionPipeline.to_preprocessed_rgbd(img, depth, img_size=224)
    assert out is not None
    assert out.shape == (4, 224, 224)
    assert out.dtype == np.float32


def test_to_preprocessed_rgbd_returns_none_when_rgb_invalid():
    out = PerceptionPipeline.to_preprocessed_rgbd(
        "not_array", np.ones((10, 10), dtype=np.float32)
    )
    assert out is None
