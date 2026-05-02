#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2_tfmsciencetrace_mixin.py
# Contenido: F14-step (2026-05-02) — mixin TfmScienceTrace extraído de ControlPanelV2.
"""Mixin TfmScienceTrace extraído de ControlPanelV2 (F14).

F14-step17 — wrappers thin de TFM repro/science + trace UI + tf sanity. Aliases: _tc, _ts. Algunos métodos preservan firmas typed (signal_close_panel, world_frame: str, helper, base_frame: str, etc).
"""

from __future__ import annotations

from . import panel_trace_callbacks as _tc
from . import panel_tfm_science as _ts


class PanelV2TfmScienceTraceMixin:
    """Wrappers thin de tfmsciencetrace."""

    def _tfm_repro_profile_env(self, *args, **kwargs):
        return _ts._tfm_repro_profile_env(self, *args, **kwargs)

    def _tfm_repro_profile(self, *args, **kwargs):
        return _ts._tfm_repro_profile(self, *args, **kwargs)

    def _tfm_raw_output_env_enabled(self, *args, **kwargs):
        return _ts._tfm_raw_output_env_enabled(self, *args, **kwargs)

    def _tfm_postprocess_enabled(self, *args, **kwargs):
        return _ts._tfm_postprocess_enabled(self, *args, **kwargs)

    def _tfm_postprocess_policy_label(self, *args, **kwargs):
        return _ts._tfm_postprocess_policy_label(self, *args, **kwargs)

    def _on_tfm_repro_mode_changed(self, *args, **kwargs):
        return _ts._on_tfm_repro_mode_changed(self, *args, **kwargs)

    def _on_tfm_postprocess_mode_changed(self, *args, **kwargs):
        return _ts._on_tfm_postprocess_mode_changed(self, *args, **kwargs)

    def _on_tfm_checkpoint_selection_changed(self, *args, **kwargs):
        return _ts._on_tfm_checkpoint_selection_changed(self, *args, **kwargs)

    def _tfm_apply_memoria_case(self, *args, **kwargs):
        return _ts._tfm_apply_memoria_case(self)

    def _tfm_repro_checkpoint(self, *args, **kwargs):
        return _ts._tfm_repro_checkpoint(self, *args, **kwargs)

    def _tfm_repro_checkpoint_meta(self, *args, **kwargs):
        return _ts._tfm_repro_checkpoint_meta(self, *args, **kwargs)

    def _tfm_is_aux_experiment(self, *args, **kwargs):
        return _ts._tfm_is_aux_experiment(self, *args, **kwargs)

    def _tfm_select_seed_from_summary(self, *args, **kwargs):
        return _ts._tfm_select_seed_from_summary(self, *args, **kwargs)

    def _discover_tfm_checkpoints(self, *args, **kwargs):
        return _ts._discover_tfm_checkpoints(self, *args, **kwargs)

    def _pick_default_tfm_checkpoint(self, *args, **kwargs):
        return _ts._pick_default_tfm_checkpoint(self, *args, **kwargs)

    def _refresh_tfm_checkpoint_options(self, *args, **kwargs):
        return _ts._refresh_tfm_checkpoint_options(self, *args, **kwargs)

    def _format_ckpt_label(self, *args, **kwargs):
        return _ts._format_ckpt_label(self, *args, **kwargs)

    def _tfm_get_ckpt_path(self, *args, **kwargs):
        return _ts._tfm_get_ckpt_path(self, *args, **kwargs)

    def _tfm_apply_experiment(self, *args, **kwargs):
        return _ts._tfm_apply_experiment(self)

    def _tfm_reset_grasp(self, *args, **kwargs):
        return _ts._tfm_reset_grasp(self)

    def _load_experiment_info(self, *args, **kwargs):
        return _ts._load_experiment_info(self, *args, **kwargs)

    def _format_value(self, *args, **kwargs):
        return _ts._format_value(self, *args, **kwargs)

    def _refresh_science_ui(self, *args, **kwargs):
        return _ts._refresh_science_ui(self, *args, **kwargs)

    def _world_to_pixel(self, *args, **kwargs):
        return _ts._world_to_pixel(self, *args, **kwargs)

    def _world_to_pixel_diag(self, *args, **kwargs):
        return _ts._world_to_pixel_diag(self, *args, **kwargs)

    def _build_reference_grasp(self, *args, **kwargs):
        return _ts._build_reference_grasp(self, *args, **kwargs)

    def _grasp_projection_z_target(self, *args, **kwargs):
        return _ts._grasp_projection_z_target(self, *args, **kwargs)

    def _compute_world_grasp(self, *args, **kwargs):
        return _ts._compute_world_grasp(self, *args, **kwargs)

    def _world_grasp_to_base(self, *args, **kwargs):
        return _ts._world_grasp_to_base(self, *args, **kwargs)

    def _update_cornell_metrics(self, *args, **kwargs):
        return _ts._update_cornell_metrics(self, *args, **kwargs)

    def _refresh_cornell_metrics(self, *args, **kwargs):
        return _ts._refresh_cornell_metrics(self, *args, **kwargs)

    def _save_episode(self, *args, **kwargs):
        return _ts._save_episode(self, *args, **kwargs)

    def _build_science_group(self) -> QGroupBox:
        return build_science_group(self)

    def _build_trace_group(self) -> QGroupBox:
        return build_trace_group(self)

    def _start_trace_timer(self):
        return _tc._start_trace_timer(self)

    def _resolve_trace_frames(self, world_frame: str) -> Tuple[str, Optional[str]]:
        return _tc._resolve_trace_frames(self, world_frame)

    def _refresh_trace_data(self):
        return _tc._refresh_trace_data(self)

    def _log_trace_transform_warning(self, message: str) -> None:
        _tc._log_trace_transform_warning(self, message)

    def _maybe_log_tf_not_ready(self):
        return _tc._maybe_log_tf_not_ready(self)

    def _maybe_log_trace(self, now: float):
        _tc._maybe_log_trace(self, now)

    def _reset_trace_throttle(self, reason: str):
        _tc._reset_trace_throttle(self, reason)

    def _run_trace_diag_once(self):
        return _tc._run_trace_diag_once(self)

    def _tf_sanity_check(self) -> Tuple[bool, str]:
        return _tc._tf_sanity_check(self)

    def _run_self_check_once(self) -> None:
        _tc._run_self_check_once(self)

    def _self_check_worker(self) -> None:
        _tc._self_check_worker(self)

    def _trace_diag_worker(self) -> None:
        _tc._trace_diag_worker(self)

    def _try_mark_tf_ready(self):
        return _tc._try_mark_tf_ready(self)

    def _start_tf_ready_timer(self):
        return _tc._start_tf_ready_timer(self)

    def _wait_for_tf_ready(self, world_frame: str, helper) -> Optional[str]:
        return _tc._wait_for_tf_ready(self, world_frame, helper)

    def _tf_world_base_valid(self, helper, base_frame: str, world_frame: str) -> bool:
        return _tc._tf_world_base_valid(self, helper, base_frame, world_frame)

    def _stop_tf_ready_timer(self):
        return _tc._stop_tf_ready_timer(self)

    def _log_tf_chain_once(self, world_frame: str, base_frame: str, ee_frame) -> None:
        _tc._log_tf_chain_once(self, world_frame, base_frame, ee_frame)

    def _check_tcp_source_mismatch(self, now_mono: float) -> None:
        _tc._check_tcp_source_mismatch(self, now_mono)

    def _build_trace_text(self, world_frame, base_frame, ee_frame, object_world,
                          object_base, tcp_world, tcp_base, base_error, world_error,
                          tf_transform, *, object_source, tcp_source):
        return _tc._build_trace_text(self, world_frame, base_frame, ee_frame,
                                     object_world, object_base, tcp_world, tcp_base,
                                     base_error, world_error, tf_transform,
                                     object_source=object_source, tcp_source=tcp_source)

    def _set_trace_row(self, row, world_data, base_data, world_frame, base_frame):
        _tc._set_trace_row(self, row, world_data, base_data, world_frame, base_frame)

    def _value_from_pose(self, data, key: str):
        return _tc._value_from_pose(self, data, key)

    def _set_trace_item(self, row: int, col: int, text: str):
        _tc._set_trace_item(self, row, col, text)

    def _format_dual_value(self, first, second) -> str:
        return _tc._format_dual_value(self, first, second)

    def _pose_dict(self, position, orientation, frame: str):
        return _tc._pose_dict(self, position, orientation, frame)

    def _compute_error(self, source, target):
        return _tc._compute_error(self, source, target)

    def _format_error_text(self, error) -> str:
        return _tc._format_error_text(self, error)

    def _format_error_tuple(self, error) -> str:
        return _tc._format_error_tuple(self, error)

    def _format_pose_summary(self, label: str, data) -> str:
        return _tc._format_pose_summary(self, label, data)

    def _copy_trace_text(self):
        return _tc._copy_trace_text(self)

