#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_tfm_remote_mixin.py
# Contenido: F14-step12 (2026-05-01) — mixin TFM grasp + remote callbacks.
"""Mixin TFM grasp + remote callbacks (F14-step12).

Decimosegundo paso del refactor F14: 27 wrappers thin que cubren
el flujo TFM (inferencia, ejecución, visualización) y los callbacks
de control remoto del panel (panel_remote_callbacks _rc).

Cobertura:

* TFM grasp (19): _tfm_infer_grasp, _tfm_canonical_use_pick_object,
  _complete_pending_tfm_infer_request,
  _complete_pending_tfm_execute_request,
  _complete_pending_pick_demo_request,
  _build_tfm_pick_object_override, _tfm_canonical_state_reset,
  _tfm_canonical_phase_update, _tfm_canonical_finish,
  _restore_infer_selection_snapshot,
  _latest_camera_frame_snapshot, _ensure_selected_object_in_store,
  _handle_infer_result, _sync_tfm_module_grasp_state,
  _tfm_visualize_grasp, _wait_tfm_moveit_result,
  _execute_tfm_world_grasp, _on_tfm_grasp_object_clicked,
  _tfm_publish_grasp.

  Estas funciones se importan directamente desde ``panel_tfm`` (sin
  alias). Tres firmas legacy preservadas (NO propagan
  *args/**kwargs): ``_tfm_infer_grasp``, ``_tfm_visualize_grasp``,
  ``_on_tfm_grasp_object_clicked``.

* Remote callbacks (8): _on_remote_camera_connect_request,
  _on_remote_camera_disconnect_request, _on_remote_recover_request,
  _on_remote_tfm_infer_request, _on_remote_tfm_execute_request,
  _on_remote_pick_demo_request, _on_remote_pick_object_request,
  _on_remote_object_select_request. Tipos en la firma preservados
  (source: str / name: str). Delegan a ``panel_remote_callbacks``
  (alias ``_rc``).
"""

from __future__ import annotations

from . import panel_remote_callbacks as _rc
from .panel_tfm import (
    build_tfm_pick_object_override,
    complete_pending_pick_demo_request,
    complete_pending_tfm_execute_request,
    complete_pending_tfm_infer_request,
    ensure_selected_object_in_store,
    execute_tfm_world_grasp,
    handle_infer_result,
    latest_camera_frame_snapshot,
    on_tfm_grasp_object_clicked,
    restore_infer_selection_snapshot,
    sync_tfm_module_grasp_state,
    tfm_canonical_finish,
    tfm_canonical_phase_update,
    tfm_canonical_state_reset,
    tfm_canonical_use_pick_object,
    tfm_infer_grasp,
    tfm_publish_grasp,
    tfm_visualize_grasp,
    wait_tfm_moveit_result,
)


class PanelV2TfmRemoteMixin:
    """27 wrappers TFM grasp + remote callbacks."""

    # ------------------------------------------------------------------
    # TFM grasp wrappers (delegan a panel_tfm sin alias)
    # ------------------------------------------------------------------

    def _tfm_infer_grasp(self, *args, **kwargs):
        # Legacy: NO propaga *args/**kwargs.
        return tfm_infer_grasp(self)

    def _tfm_canonical_use_pick_object(self, *args, **kwargs):
        return tfm_canonical_use_pick_object(self, *args, **kwargs)

    def _complete_pending_tfm_infer_request(self, *args, **kwargs):
        return complete_pending_tfm_infer_request(self, *args, **kwargs)

    def _complete_pending_tfm_execute_request(self, *args, **kwargs):
        return complete_pending_tfm_execute_request(self, *args, **kwargs)

    def _complete_pending_pick_demo_request(self, *args, **kwargs):
        return complete_pending_pick_demo_request(self, *args, **kwargs)

    def _build_tfm_pick_object_override(self, *args, **kwargs):
        return build_tfm_pick_object_override(self, *args, **kwargs)

    def _tfm_canonical_state_reset(self, *args, **kwargs):
        return tfm_canonical_state_reset(self, *args, **kwargs)

    def _tfm_canonical_phase_update(self, *args, **kwargs):
        return tfm_canonical_phase_update(self, *args, **kwargs)

    def _tfm_canonical_finish(self, *args, **kwargs):
        return tfm_canonical_finish(self, *args, **kwargs)

    def _restore_infer_selection_snapshot(self, *args, **kwargs):
        return restore_infer_selection_snapshot(self, *args, **kwargs)

    def _latest_camera_frame_snapshot(self, *args, **kwargs):
        return latest_camera_frame_snapshot(self, *args, **kwargs)

    def _ensure_selected_object_in_store(self, *args, **kwargs):
        return ensure_selected_object_in_store(self, *args, **kwargs)

    def _handle_infer_result(self, *args, **kwargs):
        return handle_infer_result(self, *args, **kwargs)

    def _sync_tfm_module_grasp_state(self, *args, **kwargs):
        return sync_tfm_module_grasp_state(self, *args, **kwargs)

    def _tfm_visualize_grasp(self, *args, **kwargs):
        # Legacy: NO propaga *args/**kwargs.
        return tfm_visualize_grasp(self)

    def _wait_tfm_moveit_result(self, *args, **kwargs):
        return wait_tfm_moveit_result(self, *args, **kwargs)

    def _execute_tfm_world_grasp(self, *args, **kwargs):
        return execute_tfm_world_grasp(self, *args, **kwargs)

    def _on_tfm_grasp_object_clicked(self, *args, **kwargs):
        # Legacy: NO propaga *args/**kwargs.
        return on_tfm_grasp_object_clicked(self)

    def _tfm_publish_grasp(self, *args, **kwargs):
        return tfm_publish_grasp(self, *args, **kwargs)

    # ------------------------------------------------------------------
    # Remote callbacks (delegan a panel_remote_callbacks _rc)
    # ------------------------------------------------------------------

    def _on_remote_camera_connect_request(self, source: str) -> None:
        _rc._on_remote_camera_connect_request(self, source)

    def _on_remote_camera_disconnect_request(self, source: str) -> None:
        _rc._on_remote_camera_disconnect_request(self, source)

    def _on_remote_recover_request(self, source: str) -> None:
        _rc._on_remote_recover_request(self, source)

    def _on_remote_tfm_infer_request(self, source: str) -> None:
        _rc._on_remote_tfm_infer_request(self, source)

    def _on_remote_tfm_execute_request(self, source: str) -> None:
        _rc._on_remote_tfm_execute_request(self, source)

    def _on_remote_pick_demo_request(self, source: str) -> None:
        _rc._on_remote_pick_demo_request(self, source)

    def _on_remote_pick_object_request(self, source: str) -> None:
        _rc._on_remote_pick_object_request(self, source)

    def _on_remote_object_select_request(self, name: str, source: str) -> None:
        _rc._on_remote_object_select_request(self, name, source)
