#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_grasping/test/test_grasp_module_api.py
# Contenido: F4 — tests de API pública de TFMGraspModule (sin modelo real).
"""F4 — Tests de TFMGraspModule API.

Cubre el comportamiento de la fachada principal del paquete tfm_grasping
sin requerir un modelo PyTorch real cargado. Se ejercitan los caminos:
  * inicialización + estado defaults
  * load_model con path vacío / inválido
  * last_grasp / get_grasp_representation cuando no hay grasp
  * set_last_grasp + reset
  * infer_grasp sin frame previo
  * publish_grasp sin grasp
  * model_info estructura

Los tests validan el contrato API. La inferencia real con modelo cargado
está cubierta por `test_model_load_exp11.py`.
"""
from __future__ import annotations

import math

import pytest

from tfm_grasping.geometry import Grasp2D
from tfm_grasping.grasp_module import TFMGraspModule


@pytest.fixture
def captured_logs() -> list[str]:
    return []


@pytest.fixture
def module(captured_logs: list[str]) -> TFMGraspModule:
    return TFMGraspModule(logger=captured_logs.append, model_path="")


def test_module_init_default_state(module: TFMGraspModule) -> None:
    """T34-a — al crear el módulo, no hay frame ni grasp cacheados."""
    assert module.is_model_loaded() is False
    assert module.get_grasp_representation() is None


def test_load_model_with_empty_path_returns_false(
    module: TFMGraspModule, captured_logs: list[str]
) -> None:
    """T34-b — load_model('') retorna False y logga error informativo."""
    ok = module.load_model("")
    assert ok is False
    assert any("Modelo no cargado" in line for line in captured_logs)


def test_load_model_with_nonexistent_path_returns_false(
    module: TFMGraspModule,
) -> None:
    """T34-c — load_model con ruta inexistente retorna False sin crash."""
    ok = module.load_model("/tmp/this/path/does/not/exist.pth")
    assert ok is False
    assert module.is_model_loaded() is False


def test_infer_grasp_without_input_returns_none(
    module: TFMGraspModule, captured_logs: list[str]
) -> None:
    """T34-d — infer_grasp sin set_input_image retorna None y logga aviso."""
    result = module.infer_grasp()
    assert result is None
    assert any("sin imagen de entrada" in line for line in captured_logs)


def test_publish_grasp_without_grasp_returns_false(
    module: TFMGraspModule, captured_logs: list[str]
) -> None:
    """T34-e — publish_grasp sin grasp previo retorna False."""
    ok = module.publish_grasp()
    assert ok is False
    assert any("No hay grasp para publicar" in line for line in captured_logs)


def test_get_grasp_representation_without_grasp_logs_warning(
    module: TFMGraspModule, captured_logs: list[str]
) -> None:
    """T34-f — get_grasp_representation sin grasp logga aviso."""
    result = module.get_grasp_representation()
    assert result is None
    assert any("No hay grasp para visualizar" in line for line in captured_logs)


def test_set_last_grasp_caches_externally(module: TFMGraspModule) -> None:
    """T34-g — set_last_grasp permite inyectar grasp externo (camino legacy)."""
    g = Grasp2D(
        center_x=320.0,
        center_y=240.0,
        angle_rad=math.pi / 4,
        width_px=50.0,
        quality=0.85,
        height_px=20.0,
        depth_m=0.4,
        frame_id="camera_frame",
    )
    module.set_last_grasp(g)
    rep = module.get_grasp_representation()
    assert rep is not None
    assert rep["center_x"] == pytest.approx(320.0)
    assert rep["frame_id"] == "camera_frame"


def test_reset_clears_grasp_and_frame(module: TFMGraspModule) -> None:
    """T34-h — reset() limpia grasp y frame cacheados."""
    g = Grasp2D(
        center_x=10.0,
        center_y=20.0,
        angle_rad=0.0,
        width_px=30.0,
        quality=0.5,
    )
    module.set_last_grasp(g)
    assert module.get_grasp_representation() is not None
    module.reset()
    assert module.get_grasp_representation() is None


def test_model_info_structure_when_unloaded(module: TFMGraspModule) -> None:
    """T34-i — model_info() devuelve dict con campos esperados."""
    info = module.model_info()
    assert isinstance(info, dict)
    expected_keys = {"model_path", "loaded", "model_name", "in_channels", "img_size"}
    assert expected_keys <= set(info.keys())
    assert info["loaded"] is False


def test_last_error_is_string(module: TFMGraspModule) -> None:
    """T34-j — last_error() retorna str (vacío inicialmente o tras fallo)."""
    err = module.last_error()
    assert isinstance(err, str)


def test_infer_grasp_params_returns_none_without_input(
    module: TFMGraspModule,
) -> None:
    """T34-k — infer_grasp_params sin frame retorna None."""
    result = module.infer_grasp_params()
    assert result is None
