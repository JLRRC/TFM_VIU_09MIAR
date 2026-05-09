#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_gripper_attach_mixin.py
# Contenido: F14-step4 (2026-05-01) — mixin de comandos gripper + attach extraído de ControlPanelV2.
"""Mixin de comandos gripper + attach (F14-step4).

Cuarto paso del refactor F14: extrae los wrappers thin relacionados
con la lógica de attach físico (detachable_joint, follow_tcp) y los
comandos al gripper (close/open/preopen). Todos delegan al módulo
``panel_state_methods`` (alias ``_stm``).

Métodos agrupados:

* ``_normalize_attach_name`` — sanea el nombre del objeto para el
  topic /gripper/<obj>/attach (helper sin estado).
* ``_find_attach_candidate`` — busca el mejor candidato a adherirse
  según pose/distance.
* ``_attempt_attach`` / ``_schedule_attach_attempt`` — orquestación
  del intento de attach con retries y validación geométrica.
* ``_command_gripper`` — emite comando al controller del gripper.
* ``_command_gripper_preopen`` — pre-apertura RG2 antes de bajar al
  objeto (basada en minor_axis del grasp rect).

``ControlPanelV2`` lo incluye como mixin junto a los anteriores
(PanelV2PublisherMixin, PanelV2BasePoseMixin).
"""

from __future__ import annotations

from . import panel_state_methods as _stm


class PanelV2GripperAttachMixin:
    """Wrappers thin de comandos gripper + lógica de attach.

    El cuerpo real vive en ``panel_state_methods``. Este mixin solo
    provee la interfaz unificada de la clase ControlPanelV2.
    """

    def _normalize_attach_name(self, *args, **kwargs):
        # NOTA: helper estático en _stm — NO pasa self.
        return _stm._normalize_attach_name(*args, **kwargs)

    def _find_attach_candidate(self, *args, **kwargs):
        return _stm._find_attach_candidate(self, *args, **kwargs)

    def _attempt_attach(self, *args, **kwargs):
        return _stm._attempt_attach(self, *args, **kwargs)

    def _schedule_attach_attempt(self, *args, **kwargs):
        return _stm._schedule_attach_attempt(self, *args, **kwargs)

    def _command_gripper(self, *args, **kwargs):
        return _stm._command_gripper(self, *args, **kwargs)

    def _command_gripper_preopen(self, *args, **kwargs):
        return _stm._command_gripper_preopen(self, *args, **kwargs)
