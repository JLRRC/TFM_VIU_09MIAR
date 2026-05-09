#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/step_history_recorder.py
# Contenido: F3 — recorder de history del step pipeline (8 funciones).
"""Step pipeline history recorder.

Extraído de ``panel_step_callbacks.py`` (~640 LOC, ~32% del fichero
original) para reducir el god-file.

8 funciones cohesivas que graban filas de history en el step pipeline
del modo STEP_BY_STEP: capture_start_pose, record_history, pre-insert
de la fila INICIO, snapshots iniciales/HOME/MESA y event snapshots,
upsert ordenado, y record_direct_event_snapshot.

Cada función toma ``panel`` como primer arg y usa atributos
(``_step_history_rows``, ``_step_window``, etc.) y métodos
(``_business_base_frame``, ``_step_format_xyz``, etc.) del panel.

Re-exportadas desde ``panel_step_callbacks`` para preservar API
pública.
"""

from __future__ import annotations

import json
import time
from typing import Dict

from .tf_pose_utils import get_tcp_in_base as tf_get_tcp_in_base



def _step_capture_start_pose(panel, trigger: str) -> None:
    base_frame = panel._business_base_frame()
    ee_frame = panel._step_operational_frame_name()
    start_xyz = None
    start_rpy = None
    tcp_pose_base, tcp_rpy_deg, _tcp_reason = tf_get_tcp_in_base(
        base_frame=base_frame,
        ee_frame=ee_frame,
        timeout=0.12,
        logger=None,
    )
    if tcp_pose_base is not None:
        start_xyz = (
            float(tcp_pose_base.pose.position.x),
            float(tcp_pose_base.pose.position.y),
            float(tcp_pose_base.pose.position.z),
        )
        if tcp_rpy_deg is not None and len(tcp_rpy_deg) >= 3:
            start_rpy = (
                float(tcp_rpy_deg[0]),
                float(tcp_rpy_deg[1]),
                float(tcp_rpy_deg[2]),
            )
    if start_xyz is None:
        start_xyz = panel._last_trace_tcp_base or panel._last_tcp_base
    if start_rpy is None:
        start_rpy = panel._last_trace_tcp_rpy_deg or panel._last_tcp_rpy_deg
    panel._step_start_pose_base = start_xyz
    panel._step_start_pose_rpy_deg = start_rpy
    panel._step_start_trigger = str(trigger or "").strip()

def _step_record_history(panel, flow: str, phase: str, position=None) -> None:
    phase_name = str(phase or "").strip().upper()
    flow_name = str(flow or "").strip().upper()
    pos3 = None
    if isinstance(position, (list, tuple)) and len(position) >= 3:
        try:
            pos3 = (float(position[0]), float(position[1]), float(position[2]))
        except Exception:
            pos3 = None
    sequence = panel._step_phase_sequence(flow_name)
    first_phase = sequence[0] if sequence else ""
    # No resetear si la fila INICIO ya fue pre-insertada desde el hilo principal.
    # El guard se activa si existe una fila con la fase inicial (independientemente
    # de si actual está fijado o no — la fila es válida en cualquier caso).
    _inicio_pre_frozen = bool(
        first_phase
        and panel._step_history_rows
        and str(panel._step_history_rows[0].get("phase", "")).strip().upper() == first_phase
    )
    if flow_name != panel._step_history_flow:
        panel._step_history_flow = flow_name
        panel._step_history_rows = []
    elif first_phase and phase_name == first_phase and not _inicio_pre_frozen:
        panel._step_history_rows = []
    if panel._step_history_rows:
        last_row = panel._step_history_rows[-1]
        last_phase = str(last_row.get("phase") or "").strip().upper()
        if last_phase != phase_name and last_row.get("actual") is None:
            _op_frame = panel._step_operational_frame_name()
            actual_pose = panel._step_fetch_live_pose(_op_frame)
            _pose_src = "tf_live" if actual_pose is not None else "unavailable"
            _xyz_log = (
                f"({actual_pose[0]:.3f},{actual_pose[1]:.3f},{actual_pose[2]:.3f})"
                if actual_pose is not None else "none"
            )
            panel._emit_log(
                f"[STEP][TF_LIVE] available={str(actual_pose is not None).lower()} "
                f"frame={_op_frame} xyz={_xyz_log}"
            )
            panel._emit_log(
                f"[STEP][POSE_SOURCE] phase={last_phase} source={_pose_src} "
                f"frame={_op_frame} xyz={_xyz_log} "
                f"same_frame_as_backend=true same_source_as_backend=true"
            )
            # Sin fallback a caché stale: si TF no devuelve pose, actual queda None
            # y la tabla muestra PEND en vez de datos inventados.
            last_row["actual"] = actual_pose
            panel._step_update_row_object_metrics(last_row, actual_pose)
            last_row["reached"] = panel._step_assess_target_reached(last_row.get("target"), actual_pose)
            if str(last_row.get("row_kind") or "").strip().upper() == "PHASE":
                last_row["row_state"] = (
                    "PHASE_DONE"
                    if last_row.get("reached") is True
                    else "PHASE_BLOCKED"
                )
        elif last_phase != phase_name and last_row.get("actual") is not None:
            panel._emit_log(
                "[STEP][ROW_FROZEN] "
                f"phase={last_phase} "
                f"actual={panel._step_format_inline_xyz(last_row.get('actual'))} "
                f"target={panel._step_format_inline_xyz(last_row.get('target'))}"
            )
    _op_frame_snap = panel._step_operational_frame_name()
    if panel._step_history_rows and str(panel._step_history_rows[-1].get("phase") or "").strip().upper() == phase_name:
        # Mismo fase: actualizar target.
        # origin_snapshot se actualiza si actual=None (gate aún no cerrado): esto ocurre
        # cuando _step_pre_insert_inicio_row lo capturó antes de que el robot llegara
        # a MESA. El worker llama de nuevo con la pose real post-MESA para corregirlo.
        _existing_snap = panel._step_history_rows[-1].get("origin_snapshot")
        _row_actual = panel._step_history_rows[-1].get("actual")
        if _row_actual is None:
            # Gate no cerrado aún: actualizar origin_snapshot con TF live actual.
            _current_snap = panel._step_fetch_live_pose(_op_frame_snap)
            if _current_snap is not None:
                if _existing_snap is not None:
                    _mut_delta = max(abs(_current_snap[i] - _existing_snap[i]) for i in range(3))
                    panel._emit_log(
                        f"[STEP][ORIGIN_UPDATE] phase={phase_name} "
                        f"old={panel._step_format_inline_xyz(_existing_snap)} "
                        f"new={panel._step_format_inline_xyz(_current_snap)} "
                        f"delta_m={_mut_delta:.4f}"
                    )
                panel._step_history_rows[-1]["origin_snapshot"] = _current_snap
        elif _existing_snap is not None:
            _current_snap = panel._step_fetch_live_pose(_op_frame_snap)
            if _current_snap is not None:
                _mut_delta = max(abs(_current_snap[i] - _existing_snap[i]) for i in range(3))
                if _mut_delta > 0.02:
                    panel._emit_log(
                        f"[STEP][ORIGIN_MUTATION_ERROR] phase={phase_name} "
                        f"old={panel._step_format_inline_xyz(_existing_snap)} "
                        f"new={panel._step_format_inline_xyz(_current_snap)} "
                        f"delta_m={_mut_delta:.4f}"
                    )
        panel._step_history_rows[-1]["target"] = pos3
    else:
        # Nueva fase: capturar pose actual como origin_snapshot (congelado).
        # Retry hasta 3 veces con 80ms de espera si TF devuelve None.
        # Siempre se usa TF live para reflejar la posición real del robot
        # cuando abre el gate, independientemente de la fase anterior.
        # Override: si existe _step_origin_override_for_next_gate (capturado antes
        # de un HOME silencioso), usarlo como ORG para reflejar la pose real pre-HOME.
        _origin_snap = None
        _snap_source = "unavailable_after_3_retries"
        _override_snap = getattr(panel, "_step_origin_override_for_next_gate", None)
        if _override_snap is not None:
            _origin_snap = _override_snap
            _snap_source = "origin_override_pre_home"
            try:
                delattr(panel, "_step_origin_override_for_next_gate")
            except AttributeError:
                pass
            panel._emit_log(
                f"[STEP][ORG_OVERRIDE] phase={phase_name} "
                f"xyz={panel._step_format_inline_xyz(_origin_snap)} "
                f"source={_snap_source}"
            )
        else:
            for _snap_attempt in range(3):
                _origin_snap = panel._step_fetch_live_pose(_op_frame_snap)
                if _origin_snap is not None:
                    _snap_source = f"tf_live_attempt_{_snap_attempt + 1}"
                    break
                time.sleep(0.08)
        # Fallback si TF falló: usar el actual de la fila anterior.
        if _origin_snap is None and panel._step_history_rows:
            _prev_actual = panel._step_history_rows[-1].get("actual")
            if _prev_actual is not None:
                _origin_snap = _prev_actual
                _snap_source = "prev_row_actual_fallback"
        panel._emit_log(
            f"[STEP][ORG_CAPTURE] phase={phase_name} "
            f"frame={_op_frame_snap} "
            f"xyz={panel._step_format_inline_xyz(_origin_snap)} "
            f"source={_snap_source}"
        )
        panel._emit_log(
            f"[STEP][ORG_CAPTURE_FRAME] phase={phase_name} "
            f"op_frame={_op_frame_snap} "
            f"capture_time_mono={time.monotonic():.3f}"
        )
        panel._step_history_rows.append(
            {
                "phase": phase_name,
                "target": pos3,
                "actual": None,            # TCP TF live al cerrar la fase (columna TCP-TF)
                "origin_snapshot": _origin_snap,  # pose robot al abrir la gate (congelada)
                "exec_target_snapshot": None,     # fijado por _step_set_exec_target
                "reached": None,
                "row_kind": "PHASE",
                "row_state": "PHASE_READY",
                "object_world_snapshot": None,    # pose objeto world al cerrar la fase
                "dist_tcp_obj_snapshot": None,    # distancia TCP↔objeto al cerrar la fase
                "check_reason": None,             # texto corto del motivo de check
            }
        )
        panel._emit_log(f"[STEP][PREPARED_PHASE] phase={phase_name} started=false")
        panel._emit_log(
            f"[STEP][ORIGIN_SNAPSHOT] phase={phase_name} "
            f"pose={panel._step_format_inline_xyz(_origin_snap)} frame={_op_frame_snap}"
        )
        panel._emit_log(
            f"[STEP][TARGET_SNAPSHOT] phase={phase_name} "
            f"pose={panel._step_format_inline_xyz(pos3)}"
        )
        # actual queda None hasta que la fase SIGUIENTE empiece: en ese momento
        # la lógica de transición (líneas ~5224-5246) captura la pose real del
        # robot, que YA habrá llegado al destino de esta fase. Esto garantiza que
        # la columna "actual" muestre dónde llegó el robot (pose de llegada),
        # no dónde estaba antes de empezar a moverse (pose de salida).

def _step_pre_insert_inicio_row(panel,
    pos_actual,
    target_pos,
    obj_pos=None,
    flow_name: str = "DIRECT",
) -> None:
    """Pre-inserta la fila INICIO en la tabla STEP_BY_STEP desde el hilo principal,
    ANTES del diálogo de confirmación, para que sea visible antes de pulsar Iniciar.

    - No bloquea (no espera evento de continue).
    - 'actual' se deja None: _step_window_refresh muestra la pose TF2 en vivo mientras
      el usuario espera.  Cuando el worker pase a APPROACH_COARSE, la lógica de transición
      en _step_record_history capturará la pose real (robot aún en HOME) y marcará
      reached=True (INICIO es fase de verificación, no de movimiento).
    - 'target' = pos3_target (destino APPROACH_COARSE): la columna "X/Y/Z Obj" del
      panel muestra adónde irá el robot cuando el usuario pulse Sigue.
    - Deshabilita el botón "Sigue" hasta que el worker esté listo y llame a
      _step_wait_for_phase, que lo habilitará via _step_window_set_waiting.
    - Solo actúa en modo STEP_BY_STEP; no hace nada en AUTO.
    - obj_pos: pose del objeto en frame base_link (para el label "Objeto XYZ").
    """
    if panel._step_mode != "STEP_BY_STEP":
        return
    first_phase = "INICIO"

    # Reset del flow para este nuevo ciclo
    panel._step_history_flow = flow_name
    panel._step_history_rows = []

    pos3_actual = None
    if isinstance(pos_actual, (list, tuple)) and len(pos_actual) >= 3:
        try:
            pos3_actual = (float(pos_actual[0]), float(pos_actual[1]), float(pos_actual[2]))
        except Exception:
            pos3_actual = None

    pos3_target = None
    if isinstance(target_pos, (list, tuple)) and len(target_pos) >= 3:
        try:
            pos3_target = (float(target_pos[0]), float(target_pos[1]), float(target_pos[2]))
        except Exception:
            pos3_target = None

    # Insertar fila INICIO:
    #   origin_snapshot = pos3_actual → pose congelada del robot al abrir la gate (MESA)
    #   actual = None  → PEND hasta que el worker capture el Cierre tras el gate
    #   target = pos3_target → pose MESA (= pos3_actual, donde ya está el robot)
    #   reached = None → PEND hasta captura
    panel._step_history_rows.append({
        "phase": first_phase,
        "target": pos3_target,           # ← pose MESA (= pos3_actual)
        "actual": None,                  # ← capturado en worker tras pulsar Sigue
        "origin_snapshot": pos3_actual,  # ← congelado: pose robot al abrir la gate
        "exec_target_snapshot": pos3_target,
        "reached": None,                 # ← PEND hasta captura
        "object_world_snapshot": None,
        "dist_tcp_obj_snapshot": None,
        "check_reason": None,
    })
    panel._emit_log(f"[STEP][PREPARED_PHASE] phase={first_phase} started=false")
    panel._emit_log(
        f"[STEP][ORIGIN_SNAPSHOT] phase={first_phase} "
        f"pose={panel._step_format_inline_xyz(pos3_actual)} frame={panel._step_operational_frame_name()}"
    )
    panel._emit_log(
        f"[STEP][TARGET_SNAPSHOT] phase={first_phase} "
        f"pose={panel._step_format_inline_xyz(pos3_target)}"
    )

    # Pose del objeto físico en frame base_link (label "Objeto XYZ")
    pos3_obj = None
    if isinstance(obj_pos, (list, tuple)) and len(obj_pos) >= 3:
        try:
            pos3_obj = (float(obj_pos[0]), float(obj_pos[1]), float(obj_pos[2]))
        except Exception:
            pos3_obj = None
    panel._step_object_position = pos3_obj

    # Actualizar estado visible de la ventana
    panel._step_pending_flow = flow_name.lower()
    panel._step_pending_phase = f"{flow_name}.{first_phase}"
    panel._step_current_phase = first_phase
    panel._step_next_phase = panel._step_predict_next_phase(flow_name, first_phase)
    panel._step_decision = "INICIO - Ir a MESA y abrir la pinza antes de continuar"
    panel._step_phase_position = pos3_target

    # Abrir ventana y mostrar la secuencia; el botón de la fase aún queda bloqueado
    # hasta que el worker alcance el punto de control correspondiente.
    panel._ensure_step_window()
    panel._step_window_refresh()
    if panel._step_window is not None:
        panel._step_window.show()

    panel._emit_log(
        "[STEP][PRE_INSERT_INICIO] "
        f"actual={panel._step_format_inline_xyz(pos3_actual)} "
        f"target={panel._step_format_inline_xyz(pos3_target)} "
        "btn_iniciar=disabled pending_worker=true"
    )

def _step_record_direct_initial_snapshot(panel,
    *,
    request_id: str,
    tcp_base=None,
    object_base=None,
    dx=None,
    dy=None,
    dz=None,
    dist3d=None,
    joints=None,
    pose_source: str = "",
    flow_name: str = "DIRECT",
) -> None:
    panel._step_record_direct_event_snapshot(
        phase_name="INITIAL_SNAPSHOT",
        request_id=request_id,
        tcp_base=tcp_base,
        object_base=object_base,
        dx=dx,
        dy=dy,
        dz=dz,
        dist3d=dist3d,
        joints=joints,
        pose_source=pose_source,
        flow_name=flow_name,
        prepend=True,
        decision_text="INITIAL_SNAPSHOT - Snapshot inicial previo al movimiento hacia MESA",
    )

def _step_upsert_history_row_ordered(panel,
    *,
    flow_name: str,
    row: Dict[str, object],
    prepend: bool = False,
) -> None:
    flow_name = str(flow_name or "DIRECT").strip().upper() or "DIRECT"
    phase_name = str(row.get("phase") or "").strip().upper()
    if flow_name != panel._step_history_flow:
        panel._step_history_flow = flow_name
        panel._step_history_rows = []

    for idx, existing_row in enumerate(panel._step_history_rows):
        if str(existing_row.get("phase") or "").strip().upper() == phase_name:
            panel._step_history_rows[idx] = row
            return

    sequence = panel._step_phase_sequence(flow_name)
    phase_index = sequence.index(phase_name) if phase_name in sequence else None
    if phase_index is None:
        if prepend:
            panel._step_history_rows.insert(0, row)
        else:
            panel._step_history_rows.append(row)
        return

    insert_at = len(panel._step_history_rows)
    for idx, existing_row in enumerate(panel._step_history_rows):
        existing_phase = str(existing_row.get("phase") or "").strip().upper()
        if existing_phase not in sequence:
            continue
        if sequence.index(existing_phase) > phase_index:
            insert_at = idx
            break
    panel._step_history_rows.insert(insert_at, row)

def _step_record_direct_home_initial(panel,
    *,
    request_id: str,
    tcp_base=None,
    object_base=None,
    dx=None,
    dy=None,
    dz=None,
    dist3d=None,
    joints=None,
    pose_source: str = "",
    flow_name: str = "DIRECT",
) -> None:
    if panel._step_mode != "STEP_BY_STEP":
        return

    flow_name = str(flow_name or "DIRECT").strip().upper() or "DIRECT"
    phase_name = "HOME_INITIAL"

    tcp_pos3 = None
    if isinstance(tcp_base, (list, tuple)) and len(tcp_base) >= 3:
        try:
            tcp_pos3 = (float(tcp_base[0]), float(tcp_base[1]), float(tcp_base[2]))
        except Exception:
            tcp_pos3 = None

    obj_pos3 = None
    if isinstance(object_base, (list, tuple)) and len(object_base) >= 3:
        try:
            obj_pos3 = (float(object_base[0]), float(object_base[1]), float(object_base[2]))
        except Exception:
            obj_pos3 = None

    dist_val = None
    if dist3d is not None:
        try:
            dist_val = float(dist3d)
        except Exception:
            dist_val = None

    def _fmt_scalar_local(value) -> str:
        if value is None:
            return "--"
        try:
            return f"{float(value):.3f}"
        except Exception:
            return "--"

    joint_values = []
    if isinstance(joints, (list, tuple)):
        for value in joints:
            try:
                joint_values.append(float(value))
            except Exception:
                joint_values.append(None)
    joints_txt = json.dumps(joint_values, ensure_ascii=True)
    reason = (
        f"request_id={str(request_id or '').strip() or 'none'} "
        f"source={str(pose_source or '').strip() or 'none'} "
        f"dx={_fmt_scalar_local(dx)} dy={_fmt_scalar_local(dy)} "
        f"dz={_fmt_scalar_local(dz)} dist3d={_fmt_scalar_local(dist_val)} "
        f"joints={joints_txt}"
    )
    row = {
        "phase": phase_name,
        "target": tcp_pos3,
        "actual": tcp_pos3,
        "origin_snapshot": tcp_pos3,
        "exec_target_snapshot": tcp_pos3,
        "reached": True,
        "row_kind": "PHASE",
        "row_state": "PHASE_DONE",
        "object_world_snapshot": panel._step_display_position(obj_pos3),
        "dist_tcp_obj_snapshot": dist_val,
        "check_reason": reason,
        "request_id": str(request_id or "").strip(),
        "pose_source": str(pose_source or "").strip(),
        "joint_snapshot": joint_values,
        "object_base_snapshot": obj_pos3,
        "delta_snapshot": {
            "dx": dx,
            "dy": dy,
            "dz": dz,
            "dist3d": dist_val,
        },
    }

    panel._step_upsert_history_row_ordered(
        flow_name=flow_name,
        row=row,
        prepend=False,
    )

    preserve_active_gate = bool(panel._step_wait_active) or bool(
        str(panel._step_running_phase or "").strip()
    )
    panel._step_pipeline_flow = flow_name
    if not preserve_active_gate:
        panel._step_pending_flow = flow_name
        panel._step_pending_phase = f"{flow_name}.{phase_name}"
        panel._step_current_phase = phase_name
        panel._step_next_phase = panel._step_predict_next_phase(flow_name, phase_name)
        panel._step_running_phase = ""
        panel._step_decision = (
            "HOME_INITIAL - Robot movido a MESA y detenido; esperando confirmación para APPROACH_COARSE"
        )
        panel._step_phase_position = tcp_pos3
        panel._step_object_position = obj_pos3

    panel._emit_log(
        f"[STEP][{phase_name}] "
        f"request_id={str(request_id or '').strip() or 'none'} "
        f"tcp={panel._step_format_inline_xyz(tcp_pos3)} "
        f"object={panel._step_format_inline_xyz(obj_pos3)} "
        f"source={str(pose_source or '').strip() or 'none'} "
        f"dist3d={_fmt_scalar_local(dist_val)} "
        f"row_state=PHASE_DONE preserve_active_gate={str(preserve_active_gate).lower()}"
    )
    panel._step_window_refresh()
    if panel._step_window is not None:
        panel._step_window.show()

def _step_record_direct_mesa_ready(panel,
    *,
    request_id: str,
    tcp_base=None,
    object_base=None,
    dx=None,
    dy=None,
    dz=None,
    dist3d=None,
    joints=None,
    pose_source: str = "",
    flow_name: str = "DIRECT",
) -> None:
    panel._step_record_direct_home_initial(
        request_id=request_id,
        tcp_base=tcp_base,
        object_base=object_base,
        dx=dx,
        dy=dy,
        dz=dz,
        dist3d=dist3d,
        joints=joints,
        pose_source=pose_source,
        flow_name=flow_name,
    )

def _step_record_direct_event_snapshot(panel,
    *,
    phase_name: str,
    request_id: str,
    tcp_base=None,
    object_base=None,
    dx=None,
    dy=None,
    dz=None,
    dist3d=None,
    joints=None,
    pose_source: str = "",
    flow_name: str = "DIRECT",
    prepend: bool = False,
    decision_text: str = "",
) -> None:
    if panel._step_mode != "STEP_BY_STEP":
        return

    phase_name = str(phase_name or "").strip().upper() or "DIRECT_EVENT"
    flow_name = str(flow_name or "DIRECT").strip().upper() or "DIRECT"

    tcp_pos3 = None
    if isinstance(tcp_base, (list, tuple)) and len(tcp_base) >= 3:
        try:
            tcp_pos3 = (float(tcp_base[0]), float(tcp_base[1]), float(tcp_base[2]))
        except Exception:
            tcp_pos3 = None

    obj_pos3 = None
    if isinstance(object_base, (list, tuple)) and len(object_base) >= 3:
        try:
            obj_pos3 = (float(object_base[0]), float(object_base[1]), float(object_base[2]))
        except Exception:
            obj_pos3 = None

    dist_val = None
    if dist3d is not None:
        try:
            dist_val = float(dist3d)
        except Exception:
            dist_val = None

    def _fmt_scalar_local(value) -> str:
        if value is None:
            return "--"
        try:
            return f"{float(value):.3f}"
        except Exception:
            return "--"

    joint_values = []
    if isinstance(joints, (list, tuple)):
        for value in joints:
            try:
                joint_values.append(float(value))
            except Exception:
                joint_values.append(None)
    joints_txt = json.dumps(joint_values, ensure_ascii=True)
    reason = (
        f"request_id={str(request_id or '').strip() or 'none'} "
        f"source={str(pose_source or '').strip() or 'none'} "
        f"dx={_fmt_scalar_local(dx)} dy={_fmt_scalar_local(dy)} "
        f"dz={_fmt_scalar_local(dz)} dist3d={_fmt_scalar_local(dist_val)} "
        f"joints={joints_txt}"
    )
    row = {
        "phase": phase_name,
        "target": tcp_pos3,
        "actual": tcp_pos3,
        "origin_snapshot": tcp_pos3,
        "exec_target_snapshot": tcp_pos3,
        "reached": None,
        "row_kind": "EVENT",
        "row_state": "EVENT_SNAPSHOT",
        "object_world_snapshot": panel._step_display_position(obj_pos3),
        "dist_tcp_obj_snapshot": dist_val,
        "check_reason": reason,
        "request_id": str(request_id or "").strip(),
        "pose_source": str(pose_source or "").strip(),
        "joint_snapshot": joint_values,
        "object_base_snapshot": obj_pos3,
        "delta_snapshot": {
            "dx": dx,
            "dy": dy,
            "dz": dz,
            "dist3d": dist_val,
        },
    }

    panel._step_upsert_history_row_ordered(
        flow_name=flow_name,
        row=row,
        prepend=prepend,
    )

    preserve_active_gate = bool(panel._step_wait_active) or bool(
        str(panel._step_running_phase or "").strip()
    )
    panel._step_pipeline_flow = flow_name
    if not preserve_active_gate:
        panel._step_pending_flow = flow_name
        panel._step_pending_phase = f"{flow_name}.{phase_name}"
        panel._step_current_phase = phase_name
        panel._step_next_phase = panel._step_predict_next_phase(flow_name, phase_name)
        panel._step_running_phase = ""
        panel._step_decision = str(decision_text or "").strip()
        panel._step_phase_position = tcp_pos3
        panel._step_object_position = obj_pos3

    panel._emit_log(
        f"[STEP][{phase_name}] "
        f"request_id={str(request_id or '').strip() or 'none'} "
        f"tcp={panel._step_format_inline_xyz(tcp_pos3)} "
        f"object={panel._step_format_inline_xyz(obj_pos3)} "
        f"source={str(pose_source or '').strip() or 'none'} "
        f"dist3d={_fmt_scalar_local(dist_val)} "
        f"preserve_active_gate={str(preserve_active_gate).lower()}"
    )
    panel._step_window_refresh()
    if panel._step_window is not None:
        panel._step_window.show()

