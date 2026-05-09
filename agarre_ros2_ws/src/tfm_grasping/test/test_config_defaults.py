"""F4: cobertura mínima de tfm_grasping.config.

Verifica que los defaults exportados existen y tienen tipos esperados.
Estos defaults los consume el resto del paquete.
"""

from __future__ import annotations

from tfm_grasping import config as cfg


def test_default_model_path_is_string():
    assert isinstance(cfg.DEFAULT_MODEL_PATH, str)


def test_default_ros_topic_is_non_empty_string():
    assert isinstance(cfg.DEFAULT_ROS_TOPIC, str)
    assert cfg.DEFAULT_ROS_TOPIC.startswith("/")


def test_default_frame_id_is_non_empty_string():
    assert isinstance(cfg.DEFAULT_FRAME_ID, str)
    assert len(cfg.DEFAULT_FRAME_ID) > 0


def test_default_min_confidence_in_unit_range():
    assert isinstance(cfg.DEFAULT_MIN_CONFIDENCE, float)
    assert 0.0 <= cfg.DEFAULT_MIN_CONFIDENCE <= 1.0
