import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "src" / "tfm_grasping"))

from tfm_grasping.model import GraspModel
from tfm_grasping.perception import PerceptionPipeline


def test_prepare_preprocessed_preserves_frame_metadata():
    image = np.zeros((3, 224, 224), dtype=np.float32)
    frame = PerceptionPipeline().prepare(
        image,
        width=320,
        height=240,
        roi=(160, 120, 160),
        preprocessed=True,
    )

    assert frame is not None
    assert frame.preprocessed is True
    assert frame.width == 320
    assert frame.height == 240
    assert frame.roi == (160, 120, 160)


def test_decode_prediction_scales_preprocessed_full_frame_to_original_size():
    model = GraspModel(img_size=224)
    frame = PerceptionPipeline().prepare(
        np.zeros((3, 224, 224), dtype=np.float32),
        width=320,
        height=240,
        preprocessed=True,
    )

    grasp = model._decode_prediction(np.array([0.483896, 0.545065, 0.069351, 0.067076, -0.01087]), frame, None)

    assert grasp is not None
    assert abs(grasp.center_x - 154.84672) < 1e-4
    assert abs(grasp.center_y - 130.8156) < 1e-4
    assert abs(grasp.width_px - 22.19232) < 1e-4
    assert abs(grasp.height_px - 16.09824) < 1e-4


def test_roi_to_box_scales_preprocessed_roi_back_to_original_space():
    model = GraspModel(img_size=224)
    frame = PerceptionPipeline().prepare(
        np.zeros((3, 224, 224), dtype=np.float32),
        width=320,
        height=240,
        roi=(160, 120, 160),
        preprocessed=True,
    )

    roi_box = model._roi_to_box(frame)

    assert roi_box == (80, 40, 160, 160)