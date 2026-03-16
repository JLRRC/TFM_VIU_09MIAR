from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parent
SRC_DIR = ROOT / "src" / "tfm_grasping"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from tfm_grasping.model import GraspModel
from tfm_grasping.perception import InputFrame, PerceptionPipeline


def test_to_preprocessed_rgbd_builds_four_channels():
    rgb = np.zeros((32, 48, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    depth = np.linspace(0.0, 10.0, 32 * 48, dtype=np.float32).reshape(32, 48)

    pre = PerceptionPipeline.to_preprocessed_rgbd(rgb, depth, img_size=224)

    assert pre is not None
    assert pre.shape == (4, 224, 224)
    assert pre.dtype == np.float32
    assert np.isclose(float(pre[0].max()), 1.0)
    assert 0.99 <= float(pre[3].max()) <= 1.0
    assert float(pre[3].min()) >= 0.0


class _FakeTensor:
    def __init__(self, arr):
        self._arr = np.asarray(arr)

    def unsqueeze(self, axis):
        return _FakeTensor(np.expand_dims(self._arr, axis))

    def to(self, _device):
        return self

    def squeeze(self, axis=None):
        return _FakeTensor(np.squeeze(self._arr, axis=axis))

    def cpu(self):
        return self

    def numpy(self):
        return self._arr


class _FakeNoGrad:
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False


class _FakeTorch:
    class cuda:
        @staticmethod
        def is_available():
            return False

    @staticmethod
    def device(name):
        return name

    @staticmethod
    def from_numpy(arr):
        return _FakeTensor(arr)

    @staticmethod
    def no_grad():
        return _FakeNoGrad()


def test_model_accepts_preprocessed_rgbd(monkeypatch):
    import tfm_grasping.model as model_module

    monkeypatch.setattr(model_module, "torch", _FakeTorch)

    model = GraspModel(img_size=224)
    model.info.loaded = True
    model.info.in_channels = 4
    model._model = lambda _tensor: _FakeTensor(np.array([[0.5, 0.25, 0.2, 0.1, 0.0]], dtype=np.float32))

    frame = InputFrame(
        image=np.zeros((4, 224, 224), dtype=np.float32),
        width=640,
        height=480,
        timestamp=0.0,
        preprocessed=True,
    )

    grasp = model.infer(frame)

    assert grasp is not None
    assert model.last_error() == ""
    assert abs(grasp.center_x - 320.0) < 1e-6
    assert abs(grasp.center_y - 120.0) < 1e-6