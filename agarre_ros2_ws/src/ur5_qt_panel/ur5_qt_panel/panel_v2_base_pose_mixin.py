#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_base_pose_mixin.py
# Contenido: F14-step3 (2026-05-01) — mixin de helpers TCP/base pose extraído de ControlPanelV2.
"""Mixin de helpers de base pose / TCP de ``ControlPanelV2`` (F14-step3).

F14-step3 extrae los 11 wrappers thin relacionados con la
construcción del frame de negocio (base_link), la lectura de TCP
y la conversión de poses a la base. Todos delegan en
``panel_state_methods`` (alias ``_stm``):

* ``_expected_world_frame`` — frame world configurado.
* ``_business_base_frame`` — base_link efectivo del panel.
* ``_base_frame_candidates`` — candidatos a base_frame en orden de
  prioridad.
* ``ensure_base_pose`` / ``_ensure_base_coords`` — preparación de
  recursos para queries de pose.
* ``get_tcp_base`` / ``get_tcp_pose_base`` — TCP en base_link.
* ``transform_pose_to_base`` — convierte pose arbitraria a base_link.
* ``log_pose_base`` / ``log_pose`` — emisión estructurada de poses.
* ``get_pose_in_base`` — query genérica de pose por nombre de objeto.

``ControlPanelV2`` lo incluye como mixin junto a
``PanelV2PublisherMixin``.
"""

from __future__ import annotations

from . import panel_state_methods as _stm


class PanelV2BasePoseMixin:
    """Wrappers thin de helpers TCP / base pose.

    Asume que ``panel_state_methods`` (``_stm``) sigue siendo la
    implementación canónica. El mixin solo provee la interfaz unificada
    de la clase ControlPanelV2 sin reimplementar la lógica.
    """

    def _expected_world_frame(self, *args, **kwargs):
        return _stm._expected_world_frame(self, *args, **kwargs)

    def _business_base_frame(self, *args, **kwargs):
        return _stm._business_base_frame(self, *args, **kwargs)

    def _base_frame_candidates(self, *args, **kwargs):
        return _stm._base_frame_candidates(self, *args, **kwargs)

    def ensure_base_pose(self, *args, **kwargs):
        return _stm.ensure_base_pose(self, *args, **kwargs)

    def _ensure_base_coords(self, *args, **kwargs):
        return _stm._ensure_base_coords(self, *args, **kwargs)

    def get_tcp_base(self, *args, **kwargs):
        return _stm.get_tcp_base(self, *args, **kwargs)

    def get_tcp_pose_base(self, *args, **kwargs):
        return _stm.get_tcp_pose_base(self, *args, **kwargs)

    def transform_pose_to_base(self, *args, **kwargs):
        return _stm.transform_pose_to_base(self, *args, **kwargs)

    def log_pose_base(self, *args, **kwargs):
        return _stm.log_pose_base(self, *args, **kwargs)

    def log_pose(self, *args, **kwargs):
        return _stm.log_pose(self, *args, **kwargs)

    def get_pose_in_base(self, *args, **kwargs):
        return _stm.get_pose_in_base(self, *args, **kwargs)
