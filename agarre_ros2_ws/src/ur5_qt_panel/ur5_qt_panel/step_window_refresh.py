#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/step_window_refresh.py
# Contenido: F3 — step window refresh (runtime + main refresh, 274 LOC).
"""Step window refresh helpers.

Extraído de ``panel_step_callbacks.py`` para reducir el god-file. 2
funciones grandes y cohesivas:

* ``_step_runtime_refresh`` — refresca los labels de runtime audit por
  bloque cuando hay snapshot fresco.
* ``_step_window_refresh`` — refresca por completo la ventana del step
  pipeline (table + audit blocks + status). Es el helper más grande
  del fichero original (~250 LOC).

Re-exportadas desde ``panel_step_callbacks``.
"""

from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QTableWidgetItem

from .panel_runtime_pose_auditor import (
    build_runtime_audit_snapshot,
    compute_step_history_metrics,
    runtime_status_style,
)



def _step_runtime_refresh(panel) -> None:
    if not getattr(panel, "_step_runtime_block_labels", None):
        return
    try:
        snapshot = build_runtime_audit_snapshot(panel)
    except Exception as exc:
        panel._log_warning(f"[STEP][RUNTIME] {exc}")
        for labels in panel._step_runtime_block_labels.values():
            labels["status"].setText("STALE")
            labels["status"].setStyleSheet(runtime_status_style("STALE"))
            labels["summary"].setText("sin dato")
            labels["planned"].setText("sin dato")
            labels["runtime"].setText("sin dato")
        return
    for block_key, block in snapshot.blocks.items():
        labels = panel._step_runtime_block_labels.get(block_key)
        if not labels:
            continue
        labels["status"].setText(str(block.status or "STALE"))
        labels["status"].setStyleSheet(runtime_status_style(block.status))
        labels["summary"].setText(str(block.summary or "sin dato"))
        labels["planned"].setText("\n".join(block.planned_lines) or "sin dato")
        labels["runtime"].setText("\n".join(block.runtime_lines) or "sin dato")

def _step_window_refresh(panel) -> None:
    panel._ensure_step_window()
    current_flow = panel._step_effective_flow()
    current_phase = str(panel._step_current_phase or "").strip()
    running_phase = str(panel._step_running_phase or "").strip()
    next_phase = str(panel._step_next_phase or "").strip() or "--"
    if panel._step_mode_label is not None:
        panel._step_mode_label.setText(
            f"Flujo cargado: {panel._step_present_flow_name(current_flow)}"
        )
    if panel._step_phase_label is not None:
        if panel._step_wait_active and current_phase:
            phase_text = current_phase
        elif current_flow:
            phase_text = "esperando siguiente punto de control"
        else:
            phase_text = "--"
        panel._step_phase_label.setText(f"Fase lista para iniciar: {phase_text}")
    if panel._step_current_label is not None:
        current = running_phase or "--"
        current_gripper = panel._step_phase_gripper_state(
            current_flow,
            current,
        )
        panel._step_current_label.setText(
            f"Fase en ejecución: {current} | pinza esperada: {current_gripper}"
        )
    if panel._step_next_label is not None:
        next_gripper = panel._step_phase_gripper_state(
            current_flow,
            next_phase,
        )
        panel._step_next_label.setText(
            f"Próxima fase bloqueada: {next_phase} | pinza esperada: {next_gripper}"
        )
    if panel._step_intent_label is not None:
        intent = panel._step_phase_intent(current_flow, current_phase)
        panel._step_intent_label.setText(f"Objetivo de la fase: {intent}")
    if panel._step_decision_label is not None:
        action_text = panel._step_phase_action_text(
            current_flow,
            current_phase,
            panel._step_decision,
        ) if current_phase else "--"
        panel._step_decision_label.setText(
            f"Acción exacta al pulsar Iniciar: {action_text}"
        )
    if panel._step_target_label is not None:
        world_frame = panel._world_frame_last_first()
        panel._step_target_label.setText(
            panel._step_live_pose_text(
                "XYZ objetivo de la fase (world)",
                world_frame,
                panel._step_display_position(panel._step_phase_position),
            )
        )
    operational_frame = panel._step_operational_frame_name()
    world_frame = panel._world_frame_last_first()
    operational_live = panel._step_fetch_live_pose(operational_frame)
    operational_live_display = panel._step_display_position(operational_live)
    # Sin fallback a caché stale: si TF no disponible, el label muestra "--"
    if operational_live is not None:
        panel._emit_log(
            f"[STEP][LIVE_POSE_HEADER] "
            f"frame={operational_frame} "
            f"xyz={panel._step_format_inline_xyz(operational_live)}"
        )
    if panel._step_live_operational_label is not None:
        panel._step_live_operational_label.setText(
            panel._step_live_pose_text("XYZ actual del TCP (world)", world_frame, operational_live_display)
        )
    # FIX: visual_frame usaba "tool0" (base del gripper) en vez del TCP real.
    # rg2_pinch_center está desplazado respecto a tool0 por la geometría canónica
    # del URDF, así que mezclar ambos frames introduce un desfase sistemático.
    # Ahora ambos labels usan el mismo frame operacional.
    visual_frame = operational_frame
    visual_live = panel._step_fetch_live_pose(visual_frame)
    visual_live_display = panel._step_display_position(visual_live)
    # Log [PINZA_ALIGN]: diferencia entre frame visual y frame actual para validación
    if operational_live is not None and visual_live is not None:
        _dz = float(visual_live[2]) - float(operational_live[2]) if (
            len(visual_live) > 2 and len(operational_live) > 2
        ) else None
        panel._emit_log(
            f"[PINZA_ALIGN] "
            f"actual_frame={operational_frame} "
            f"visual_frame={visual_frame} "
            f"actual_xyz={panel._step_format_inline_xyz(operational_live)} "
            f"visual_xyz={panel._step_format_inline_xyz(visual_live)} "
            f"dz={f'{_dz:.4f}' if _dz is not None else '--'}"
        )
    # [MESH_ALIGN] Log de diagnóstico comparando todos los frames relevantes
    # en el MISMO ciclo de refresco para detectar desfases entre UI y geometría.
    try:
        _ma_tool0_base = panel._step_fetch_live_pose("tool0")
        _ma_pinch_base = operational_live  # ya calculado arriba (base_link)
        _ma_obj_world = panel._step_fetch_object_world()  # world directo de pose/info
        # Convertir todo a world para comparación unificada
        _ma_tool0_world = panel._step_display_position(_ma_tool0_base)
        _ma_pinch_world = operational_live_display  # ya es world
        # Org de la tabla STEP (primera fila de la fase activa, en base_link)
        _ma_step_org_world = None
        _ma_step_org_raw = None
        if panel._step_history_rows:
            _active_row = next(
                (r for r in reversed(panel._step_history_rows)
                 if str(r.get("phase", "")).strip().upper()
                 == str(panel._step_current_phase or "").strip().upper()),
                None,
            )
            if _active_row is not None:
                _ma_step_org_raw = _active_row.get("origin_snapshot")
                _ma_step_org_world = panel._step_display_position(_ma_step_org_raw)
        def _maz(a, b):
            """Z diff (a - b), both world tuples."""
            if a is None or b is None:
                return "--"
            try:
                return f"{float(a[2]) - float(b[2]):.4f}"
            except Exception:
                return "--"
        panel._emit_log(
            f"[MESH_ALIGN] "
            f"phase={panel._step_current_phase or '--'} "
            f"tool0_world={panel._step_format_inline_xyz(_ma_tool0_world)} "
            f"rg2_pinch_center_world={panel._step_format_inline_xyz(_ma_pinch_world)} "
            f"obj_root_world={panel._step_format_inline_xyz(_ma_obj_world)} "
            f"step_org_world={panel._step_format_inline_xyz(_ma_step_org_world)} "
            f"header_label_world={panel._step_format_inline_xyz(_ma_pinch_world)} "
            f"dz_tool0_pinch={_maz(_ma_tool0_world, _ma_pinch_world)} "
            f"dz_pinch_obj={_maz(_ma_pinch_world, _ma_obj_world)} "
            f"dz_tool0_obj={_maz(_ma_tool0_world, _ma_obj_world)} "
            f"dz_org_header={_maz(_ma_step_org_world, _ma_pinch_world)} "
            f"dz_org_obj={_maz(_ma_step_org_world, _ma_obj_world)}"
        )
    except Exception as _ma_exc:
        panel._emit_log(f"[MESH_ALIGN] exception={_ma_exc}")
    if panel._step_live_visual_label is not None:
        panel._step_live_visual_label.setText(
            panel._step_live_pose_text("XYZ de referencia visual (world)", world_frame, visual_live_display)
        )
    if panel._step_gripper_expected_label is not None:
        expected_gripper = panel._step_phase_gripper_state(current_flow, current_phase)
        panel._step_gripper_expected_label.setText(
            f"Pinza esperada en la fase seleccionada: {expected_gripper}"
        )
    if panel._step_live_gripper_label is not None:
        panel._step_live_gripper_label.setText(
            f"Pinza live: {panel._step_live_gripper_state()}"
        )
    if panel._step_object_label is not None:
        panel._step_object_label.setText(
            f"Objeto XYZ (world): {panel._step_format_inline_xyz(panel._step_display_position(panel._step_object_position))}"
        )
    if panel._step_start_pose_label is not None:
        trigger = str(panel._step_start_trigger or "").strip() or "--"
        xyz_txt = panel._step_format_inline_xyz(panel._step_display_position(panel._step_start_pose_base))
        rpy_txt = panel._step_format_inline_rpy(panel._step_start_pose_rpy_deg)
        panel._step_start_pose_label.setText(
            f"Pose inicial del robot al lanzar la secuencia (world): {xyz_txt} | "
            f"frame: {world_frame} | RPY: {rpy_txt} | botón: {trigger}"
        )
    panel._step_runtime_refresh()
    panel._step_refresh_pipeline_table()
    if getattr(panel, "_step_history_frame_help_label", None) is not None:
        panel._step_history_frame_help_label.setText(
            f"Tabla STEP (frame operacional: {world_frame} | interno: {operational_frame}@{panel._business_base_frame()}). "
            "Org=pose robot al abrir la fase | TCP-TF=TCP real por TF al cerrar | Obj World=pose objeto en world | "
            "Target/Exec separados | D TCP-Obj / D Target-Obj / Err TCP-Exec en metros | "
            "Tipo Target=OBJETO_EXACTO / OBJETO_MAS_CLEARANCE / CONTACTO_GRASP / EXEC_REAL / CACHE."
        )
    if panel._step_history_table is not None:
        def _fmt_metric(value) -> str:
            try:
                return f"{float(value):.3f}"
            except Exception:
                return "--"

        panel._step_history_table.setRowCount(len(panel._step_history_rows))
        for row_idx, row_data in enumerate(panel._step_history_rows):
            phase_name = str(row_data.get("phase") or "").strip().upper()
            pos3 = row_data.get("target")
            reached = row_data.get("reached")
            # Org: origin_snapshot congelado al abrir el gate (donde estaba el robot).
            # Fallback a actual si no hay snapshot; TF en vivo solo para la fase activa PEND.
            _origin_snap = row_data.get("origin_snapshot")
            if _origin_snap is not None:
                org3 = _origin_snap
            else:
                org3 = row_data.get("actual")
                if org3 is None and phase_name == str(panel._step_current_phase or "").strip().upper():
                    org3 = operational_live  # fallback: TF en vivo si no hay snapshot aún
            # TCP-TF (columna Cierre renombrada): pose real del TCP por TF al cerrar la fase.
            # Es None (PEND) mientras la fase no haya concluido.
            cierre3 = row_data.get("actual")
            # Target: destino teórico (calculado al planificar).
            # Exec: target efectivo enviado al IK (fijado por _step_set_exec_target).
            exec3 = row_data.get("exec_target_snapshot")
            # Objeto world y distancia TCP↔objeto al cerrar la fase.
            obj_world3 = row_data.get("object_world_snapshot")
            history_metrics = compute_step_history_metrics(panel, row_data)
            dist_tcp_obj = history_metrics.get("dist_tcp_obj")
            dist_target_obj = history_metrics.get("dist_target_obj")
            err_tcp_exec = history_metrics.get("err_tcp_exec")
            target_kind = str(history_metrics.get("target_kind") or "CACHE")
            target_note = str(history_metrics.get("target_note") or "")
            check_reason = row_data.get("check_reason") or ""
            org3_display = panel._step_display_position(org3)
            cierre3_display = panel._step_display_position(cierre3)
            pos3_display = panel._step_display_position(pos3)
            exec3_display = panel._step_display_position(exec3)
            # obj_world3 ya es world (guardado por _step_fetch_object_world que devuelve
            # pose/info en world frame). No pasar por _step_display_position (base→world)
            # porque produciría una doble conversión y valores erróneos en la tabla.
            obj_world3_display = obj_world3
            display_phase = f"{phase_name} - {panel._step_phase_intent(panel._step_history_flow, phase_name)}"
            expected_gripper = panel._step_phase_gripper_state(panel._step_history_flow, phase_name)
            values = (
                (display_phase, expected_gripper)
                + panel._step_format_xyz(org3_display)
                + panel._step_format_xyz(cierre3_display)
                + panel._step_format_xyz(obj_world3_display)
                + panel._step_format_xyz(pos3_display)
                + panel._step_format_xyz(exec3_display)
                + (_fmt_metric(dist_tcp_obj),)
                + (_fmt_metric(dist_target_obj),)
                + (_fmt_metric(err_tcp_exec),)
                + (target_kind,)
                + (check_reason,)
            )
            for col_idx, value in enumerate(values):
                item = QTableWidgetItem(value)
                if col_idx not in (0, 20, len(values) - 1):
                    item.setTextAlignment(Qt.AlignCenter)
                if col_idx == 20 and target_note:
                    item.setToolTip(target_note)
                if col_idx == len(values) - 1 and check_reason:
                    item.setToolTip(str(check_reason))
                panel._step_history_table.setItem(row_idx, col_idx, item)
            # Estado (columna final: índice 22)
            panel._step_history_table.setItem(
                row_idx,
                len(values),
                panel._step_status_item(
                    reached,
                    row_state=str(row_data.get("row_state") or ""),
                    row_kind=str(row_data.get("row_kind") or ""),
                ),
            )

