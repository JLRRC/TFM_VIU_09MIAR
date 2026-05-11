#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tfm.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""TFM inference helpers for the panel."""
from __future__ import annotations

import os
import math
import subprocess
import time

from .panel_utils import pixel_to_table_xy
from .panel_robot_presets import JOINT_TABLE_POSE_RAD
from .panel_tfm_preprocess import (  # noqa: F401
    _clip_roi,
    _get_cached_preprocessed_input,
    _resolve_infer_roi,
    _store_preprocessed_cache,
    build_tfm_preprocessed_input,
    reconcile_inferred_grasp_angle,
    reconcile_inferred_grasp_center,
    reconcile_inferred_grasp_size,
)
# F3: TFM inference flow extraído a panel_tfm_inference.py.
# Re-exportado para preservar API pública.
from .panel_tfm_inference import (  # noqa: F401
    _prepare_module_artifacts,
    _selection_snapshot,
    ensure_selected_object_in_store,
    handle_infer_result,
    latest_camera_frame_snapshot,
    restore_infer_selection_snapshot,
    sync_tfm_module_grasp_state,
    tfm_infer,
    tfm_infer_grasp,
)

# 2026-05-09: panel_tfm_execute borrado (path MoveIt-classic eliminado).
# Las funciones execute_tfm_world_grasp / tfm_visualize_grasp /
# wait_tfm_moveit_result eran wrappers que terminaban llamando
# run_pick_object (ahora deprecated). El botón "TFM Grasp Object" del
# panel queda sin backend efectivo (su handler emitirá un log
# [DEPRECATED]). La inferencia TFM (panel_tfm_inference) sigue funcional
# — sólo se desactiva el "execute via MoveIt" downstream.

# F3: canonical state helpers extraídos a panel_tfm_canonical.py.
# Re-exportados aquí para preservar API pública (panel_v2 los importa
# desde panel_tfm).
from .panel_tfm_canonical import (  # noqa: F401
    build_tfm_pick_object_override,
    complete_pending_pick_demo_request,
    complete_pending_tfm_execute_request,
    complete_pending_tfm_infer_request,
    tfm_canonical_finish,
    tfm_canonical_phase_update,
    tfm_canonical_state_reset,
    tfm_canonical_use_pick_object,
)


def execute_tfm_world_grasp(panel, *args, **kwargs) -> bool:
    """DEPRECATED 2026-05-09: path MoveIt-classic borrado.
    El botón TFM Grasp Object ya no ejecuta — solo loguea la inferencia."""
    panel._emit_log(
        "[TFM_GRASP][DEPRECATED] execute_tfm_world_grasp llamado pero el "
        "path MoveIt-classic fue eliminado el 2026-05-09. La inferencia TFM "
        "sigue funcional (botón Inferir TFM); para ejecutar el grasp usa "
        "'Pick Demo' (orchestrator → FJT directo)."
    )
    return False


def tfm_visualize_grasp(panel, *args, **kwargs) -> None:
    """Alterna comparación visual P (predicción) ↔ R (referencia Cornell).

    Reescrito 2026-05-11 (sustituye stub deprecated 2026-05-09): el botón
    "Comparar grasp/ref" hace toggle de ``panel._tfm_visual_compare_enabled``;
    el siguiente frame de cámara overhead repintará añadiendo el rectángulo
    verde discontinuo de la referencia Cornell sobre el rojo de la predicción
    (la lógica de dibujo ya existía en panel_draw_overlays._draw_grasp_overlay).

    Si no hay grasp inferido (``_last_grasp_px``) el toggle se permite pero se
    avisa por log; si la referencia está disponible se loguea IoU + diff de
    centro/ángulo para feedback inmediato.
    """
    try:
        new_state = not bool(getattr(panel, "_tfm_visual_compare_enabled", False))
        panel._tfm_visual_compare_enabled = new_state
    except Exception as exc:  # pragma: no cover - defensivo
        panel._emit_log(f"[TFM_COMPARE] toggle_err={exc}")
        return None

    has_pred = bool(getattr(panel, "_last_grasp_px", None))
    state_tag = "ON" if new_state else "OFF"
    if not has_pred:
        panel._emit_log(
            f"[TFM_COMPARE] state={state_tag} sin predicción "
            "(_last_grasp_px=None); ejecuta 'Inferir agarre' antes."
        )
        return None

    if not new_state:
        panel._emit_log(f"[TFM_COMPARE] state={state_tag}")
        return None

    # ON + hay predicción → intentar loguear métricas vs referencia.
    try:
        frame = getattr(panel, "_last_camera_frame", None)
        fw = int(frame.get("w", 0)) if isinstance(frame, dict) else 0
        fh = int(frame.get("h", 0)) if isinstance(frame, dict) else 0
        ref = panel._build_reference_grasp(fw, fh) if fw > 0 and fh > 0 else None
    except Exception as exc:
        panel._emit_log(f"[TFM_COMPARE] state={state_tag} ref_err={exc}")
        return None

    if not ref:
        panel._emit_log(
            f"[TFM_COMPARE] state={state_tag} sin referencia Cornell "
            "para el objeto seleccionado."
        )
        return None

    pred = panel._last_grasp_px
    try:
        from .panel_v2 import grasp_iou  # type: ignore
    except Exception:
        grasp_iou = None  # type: ignore

    iou_str = "n/a"
    if grasp_iou is not None:
        try:
            iou_str = f"{float(grasp_iou(pred, ref)):.3f}"
        except Exception as exc:
            iou_str = f"err:{exc}"

    try:
        dcx = float(pred.get("cx", 0.0)) - float(ref.get("cx", 0.0))
        dcy = float(pred.get("cy", 0.0)) - float(ref.get("cy", 0.0))
        d_ang = float(pred.get("angle_deg", 0.0)) - float(ref.get("angle_deg", 0.0))
        # normalizar a [-90, 90] (rectángulo simétrico π)
        while d_ang > 90.0:
            d_ang -= 180.0
        while d_ang <= -90.0:
            d_ang += 180.0
        panel._emit_log(
            f"[TFM_COMPARE] state={state_tag} iou={iou_str} "
            f"d_center_px=({dcx:+.1f},{dcy:+.1f}) d_angle_deg={d_ang:+.1f}"
        )
    except Exception as exc:
        panel._emit_log(f"[TFM_COMPARE] state={state_tag} iou={iou_str} metrics_err={exc}")
    return None


def wait_tfm_moveit_result(*args, **kwargs):
    """DEPRECATED 2026-05-09: stub que devuelve None (no MoveIt result)."""
    return None

def _tfm_grasp_compute_yaw_from_minor_axis(
    panel,
    *,
    cx_px: float,
    cy_px: float,
    fw: int,
    fh: int,
    proj_z: float,
    opening_axis_theta_img: float,
    grasp_base_d: dict,
) -> tuple[float, bool]:
    """F3-step12a: cómputo del yaw_base a partir del eje fino del rectángulo.

    Proyecta el eje fino opening_axis_theta_img a base_link vía
    pixel_to_table_xy (z=proj_z) y devuelve (minor_axis_yaw_deg, conversion_ok).
    Si fw/fh inválidos o proyección falla, fallback al grasp_base_d.yaw_deg.
    Emite [TFM_GRASP][YAW_SOURCE] con ambos puntos px y base + yaw final.
    """
    minor_axis_yaw_deg = float(grasp_base_d.get("yaw_deg", 0.0))
    yaw_conversion_ok = False
    _yaw_method = "fallback_grasp_base"
    _p0_px = (cx_px, cy_px)
    _p1_px = (cx_px, cy_px)
    _p0_base = (0.0, 0.0)
    _p1_base = (0.0, 0.0)
    if fw > 0 and fh > 0:
        try:
            step = 10.0
            dx = math.cos(opening_axis_theta_img) * step
            dy = math.sin(opening_axis_theta_img) * step
            wx_c2, wy_c2 = pixel_to_table_xy(
                int(round(cx_px)), int(round(cy_px)), fw, fh, z_target=proj_z
            )
            wx2, wy2 = pixel_to_table_xy(
                int(round(cx_px + dx)), int(round(cy_px + dy)), fw, fh, z_target=proj_z
            )
            if wx_c2 is not None and wx2 is not None:
                _p1_px = (cx_px + dx, cy_px + dy)
                _p0_base = (float(wx_c2), float(wy_c2))
                _p1_base = (float(wx2), float(wy2))
                minor_axis_yaw_deg = math.degrees(
                    math.atan2(float(wy2) - float(wy_c2), float(wx2) - float(wx_c2))
                )
                yaw_conversion_ok = True
                _yaw_method = "minor_axis_projection"
        except Exception as _exc:
            panel._emit_log(f"[TFM_GRASP] yaw_conversion_err={_exc}")
    else:
        yaw_conversion_ok = grasp_base_d.get("yaw_deg") is not None

    panel._emit_log(
        f"[TFM_GRASP][YAW_SOURCE] method={_yaw_method} "
        f"p0_px=({_p0_px[0]:.1f},{_p0_px[1]:.1f}) "
        f"p1_px=({_p1_px[0]:.1f},{_p1_px[1]:.1f}) "
        f"p0_base=({_p0_base[0]:.3f},{_p0_base[1]:.3f}) "
        f"p1_base=({_p1_base[0]:.3f},{_p1_base[1]:.3f}) "
        f"yaw_base_deg={minor_axis_yaw_deg:.2f}"
    )
    return minor_axis_yaw_deg, yaw_conversion_ok


def _tfm_grasp_compute_width_and_preopen(
    panel,
    *,
    cx_px: float,
    cy_px: float,
    minor_px: float,
    fw: int,
    fh: int,
    proj_z: float,
) -> tuple[float, float, float, bool]:
    """F3-step12b: cómputo apertura RG2 a partir del ancho menor del rect.

    Convierte minor_px a metros vía pixel_to_table_xy (fallback 0.04m si fw/fh
    inválidos). Calcula pre_open_width_m + finger_cmd_rad vía
    _compute_rg2_preopen_from_minor_width. Emite [TFM_GRASP][WIDTH] con la
    fuente (calibrated|fallback). Devuelve (minor_width_m, pre_open_width_m,
    finger_cmd_rad, width_conversion_ok).
    """
    minor_width_m = 0.04
    width_conversion_ok = False
    if fw > 0 and fh > 0:
        try:
            wx_c, wy_c = pixel_to_table_xy(
                int(round(cx_px)), int(round(cy_px)), fw, fh, z_target=proj_z
            )
            wx_e, wy_e = pixel_to_table_xy(
                int(round(cx_px + minor_px)), int(round(cy_px)), fw, fh, z_target=proj_z
            )
            if wx_c is not None and wx_e is not None:
                minor_width_m = math.hypot(
                    float(wx_e) - float(wx_c), float(wy_e) - float(wy_c)
                )
                width_conversion_ok = True
        except Exception as _exc:
            panel._emit_log(f"[TFM_GRASP] width_conversion_err={_exc}")

    pre_open_width_m, finger_cmd_rad = _compute_rg2_preopen_from_minor_width(minor_width_m)
    _width_source = "calibrated" if width_conversion_ok else "fallback"
    panel._emit_log(
        f"[TFM_GRASP][WIDTH] source={_width_source} "
        f"minor_width_m={minor_width_m:.4f} "
        f"pre_open_width_m={pre_open_width_m:.4f}"
    )
    return minor_width_m, pre_open_width_m, finger_cmd_rad, width_conversion_ok


def _tfm_grasp_run_pre_checks(panel, *, source: str):
    """F3-step12c: ejecuta los 3 CHECKs + RECT validate de on_tfm_grasp_object_clicked.

    CHECK 1 experiment_applied / CHECK 2 inference_valid (with stale detection) /
    CHECK 3 base_link conversion / RECT validation. Emite [TFM_GRASP][CHECK]
    + [TFM_GRASP][RECT] logs y panel._set_status + audit_append en cada
    fallo.

    Devuelve dict {grasp_base, grasp_px, cx_px, cy_px, w_px, h_px,
    angle_deg, theta_img} si todo OK; None si cualquier check falla
    (caller debe return inmediatamente).
    """
    experiment_ready, experiment_reason = panel._tfm_experiment_ready_status()
    panel._emit_log(
        f"[TFM_GRASP][CHECK] experiment_applied={str(experiment_ready).lower()} "
        f"reason={experiment_reason or 'ok'}"
    )
    if not experiment_ready:
        if "aplica" in experiment_reason.lower() or "experiment" in experiment_reason.lower():
            msg_ui = "No hay experimento aplicado. Aplica primero un experimento."
        else:
            msg_ui = f"TFM bloqueado: {experiment_reason}"
        panel._set_status(f"TFM: {msg_ui}", error=True)
        panel._audit_append(
            "logs/execute.log",
            f"[TFM_GRASP] execute FAIL mode=moveit_sequence source=infer_model "
            f"reason={experiment_reason}",
        )
        return None

    grasp_ts = float(getattr(panel, "_last_grasp_update_ts", 0.0) or 0.0)
    age_sec = max(0.0, _runtime_time() - grasp_ts) if grasp_ts > 0.0 else -1.0
    grasp_ok, grasp_reason = panel._current_grasp_status()
    panel._emit_log(
        f"[TFM_GRASP][CHECK] inference_valid={str(grasp_ok).lower()} "
        f"source={source} age_sec={age_sec:.1f} reason={grasp_reason or 'ok'}"
    )
    if not grasp_ok:
        if grasp_reason == "sin grasp":
            msg_ui = "No hay inferencia válida. Ejecuta primero la inferencia."
        elif "expirado" in grasp_reason or "stale" in grasp_reason.lower():
            msg_ui = "La inferencia no es reciente. Ejecuta de nuevo la inferencia."
        else:
            msg_ui = f"No hay inferencia válida ({grasp_reason})."
        panel._set_status(f"TFM: {msg_ui}", error=True)
        panel._audit_append(
            "logs/execute.log",
            f"[TFM_GRASP] execute FAIL mode=moveit_sequence source={source} reason={grasp_reason}",
        )
        return None

    grasp_base = getattr(panel, "_last_grasp_base", None)
    if not grasp_base:
        panel._emit_log("[TFM_GRASP][CHECK] base_link=unavailable")
        panel._set_status(
            "TFM: No se pudo convertir la hipótesis de agarre a objetivo MoveIt.", error=True
        )
        panel._audit_append(
            "logs/execute.log",
            f"[TFM_GRASP] execute FAIL mode=moveit_sequence source={source} "
            "reason=base_link_unavailable",
        )
        return None

    grasp_px = dict(panel._last_grasp_px or {})
    cx_px = float(grasp_px.get("cx", 0.0))
    cy_px = float(grasp_px.get("cy", 0.0))
    w_px = float(grasp_px.get("w", 0.0))
    h_px = float(grasp_px.get("h", 0.0))
    angle_deg = float(grasp_px.get("angle_deg", 0.0))
    theta_img = math.radians(angle_deg)

    panel._emit_log(
        f"[TFM_GRASP][RECT] cx={cx_px:.1f} cy={cy_px:.1f} "
        f"w={w_px:.1f} h={h_px:.1f} theta={angle_deg:.2f}deg"
    )

    if not (
        math.isfinite(cx_px) and math.isfinite(cy_px)
        and w_px > 0 and h_px > 0 and math.isfinite(theta_img)
    ):
        panel._set_status("TFM: rectángulo de inferencia inválido.", error=True)
        panel._audit_append(
            "logs/execute.log",
            f"[TFM_GRASP] execute FAIL mode=moveit_sequence source={source} reason=rect_invalid",
        )
        return None

    return {
        "grasp_base": grasp_base,
        "grasp_px": grasp_px,
        "cx_px": cx_px,
        "cy_px": cy_px,
        "w_px": w_px,
        "h_px": h_px,
        "angle_deg": angle_deg,
        "theta_img": theta_img,
    }


def _agarre_moveit_simple_spawn_backend(panel) -> subprocess.Popen | None:
    """Arranca el backend en modo servicio si no existe ya.

    El proceso se termina al acabar la llamada para evitar zombies. Si el
    servicio ya existia, devuelve None y no toca el proceso externo.
    """
    env = os.environ.copy()
    cmd = [
        "ros2",
        "run",
        "ur5_tools",
        "physical_grasp_demo",
        "--ros-args",
        "-p",
        "autostart:=false",
        "-p",
        "keep_alive:=true",
    ]
    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
        )
    except Exception as exc:
        panel._emit_log(
            f"[AGARRE_MOVEIT_SIMPLE][FAIL] phase=BACKEND_START reason={exc}"
        )
        return None
    panel._emit_log(
        "[AGARRE_MOVEIT_SIMPLE][BACKEND] spawned "
        "cmd='ros2 run ur5_tools physical_grasp_demo --ros-args -p autostart:=false -p keep_alive:=true'"
    )
    return proc


def _agarre_moveit_simple_wait_service(panel, service: str, timeout_sec: float) -> bool:
    deadline = time.time() + max(0.1, float(timeout_sec))
    while time.time() < deadline:
        try:
            if panel.ros_worker.has_service(service):
                return True
        except Exception:
            pass
        time.sleep(0.2)
    return False


def _agarre_moveit_simple_stop_backend(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=3.0)
    except Exception:
        try:
            proc.kill()
            proc.wait(timeout=2.0)
        except Exception:
            pass


def _agarre_moveit_simple_run_worker(panel) -> None:
    service = "/physical_grasp_demo/run"
    spawned_proc: subprocess.Popen | None = None
    try:
        panel._set_motion_lock(True)
        panel._baseline_busy = True
        panel._emit_log("[AGARRE_MOVEIT_SIMPLE][MESA][START]")
        panel._ui_set_status("Agarre MoveIt simple: moviendo a Mesa")
        move_sec = float(panel.joint_time.value()) if panel.joint_time else 3.0
        ok, info = panel._publish_joint_trajectory(list(JOINT_TABLE_POSE_RAD), move_sec)
        if not ok:
            panel._emit_log(f"[AGARRE_MOVEIT_SIMPLE][MESA][FAIL] reason={info}")
            panel._ui_set_status(f"Agarre MoveIt: Mesa fallo: {info}", error=True)
            return
        converged, reason = panel._wait_for_joint_convergence(
            list(JOINT_TABLE_POSE_RAD),
            timeout_sec=move_sec + 2.0,
            tolerance_rad=0.03,
            label="AGARRE_MOVEIT_SIMPLE_MESA",
        )
        if not converged:
            panel._emit_log(f"[AGARRE_MOVEIT_SIMPLE][MESA][FAIL] reason={reason}")
            panel._ui_set_status(
                f"Agarre MoveIt: Mesa no confirmada: {reason}", error=True
            )
            return
        panel._emit_log("[AGARRE_MOVEIT_SIMPLE][MESA][OK]")

        if not getattr(panel, "_ros_worker_started", False):
            panel._ensure_ros_worker_started()
        if not panel.ros_worker.node_ready():
            panel._emit_log(
                "[AGARRE_MOVEIT_SIMPLE][FAIL] phase=SERVICE reason=ros_worker_not_ready"
            )
            panel._ui_set_status("Agarre MoveIt: nodo ROS del panel no listo", error=True)
            return

        if not panel.ros_worker.has_service(service):
            spawned_proc = _agarre_moveit_simple_spawn_backend(panel)
            if spawned_proc is None:
                panel._ui_set_status("Agarre MoveIt: no se pudo arrancar backend", error=True)
                return
            if not _agarre_moveit_simple_wait_service(panel, service, 15.0):
                panel._emit_log(
                    "[AGARRE_MOVEIT_SIMPLE][FAIL] phase=SERVICE "
                    f"reason=timeout_waiting_{service}"
                )
                panel._ui_set_status("Agarre MoveIt: servicio backend no aparece", error=True)
                return

        panel._emit_log(f"[AGARRE_MOVEIT_SIMPLE][SERVICE][START] name={service}")
        ok, message = panel.ros_worker.call_trigger_detail(service, timeout_sec=300.0)
        for line in str(message or "").splitlines():
            if line.strip():
                panel._emit_log(line.strip())
        if ok:
            panel._emit_log("[AGARRE_MOVEIT_SIMPLE][DONE]")
            panel._ui_set_status("Agarre MoveIt simple finalizado", error=False)
        else:
            panel._emit_log(
                f"[AGARRE_MOVEIT_SIMPLE][FAIL] phase=SERVICE reason={message}"
            )
            panel._ui_set_status(f"Agarre MoveIt simple fallo: {message}", error=True)
    except Exception as exc:
        panel._emit_log(f"[AGARRE_MOVEIT_SIMPLE][FAIL] phase=EXCEPTION reason={exc}")
        panel._ui_set_status(f"Agarre MoveIt simple excepcion: {exc}", error=True)
    finally:
        _agarre_moveit_simple_stop_backend(spawned_proc)
        panel._baseline_busy = False
        panel._set_motion_lock(False)


def on_tfm_grasp_object_clicked(panel) -> None:
    """Boton 'Agarre objeto': Mesa -> demo fisica MoveIt sobre pick_demo."""
    if getattr(panel, "_agarre_moveit_simple_busy", False):
        panel._emit_log("[AGARRE_MOVEIT_SIMPLE][BUTTON] ignored reason=busy")
        return
    panel._log_button("TFM Agarre objeto")
    panel._emit_log("[AGARRE_MOVEIT_SIMPLE][BUTTON] clicked")
    panel._agarre_moveit_simple_busy = True

    def _done() -> None:
        panel._agarre_moveit_simple_busy = False

    panel._run_async(
        lambda: _agarre_moveit_simple_run_worker(panel),
        name="agarre_moveit_simple",
        on_done=_done,
    )

def tfm_publish_grasp(panel):
    on_tfm_grasp_object_clicked(panel)
    if getattr(panel, "_tfm_execute_inflight", False):
        return True, "ejecucion iniciada"
    return False, "agarre no iniciado"

# Grasp geometry helpers (compartidos con panel_v2 via panel_tfm_geometry).
from .panel_tfm_geometry import (  # noqa: E402,F401
    _compute_minor_axis_from_grasp_rect,
    _compute_rg2_preopen_from_minor_width,
    _tfm_clamp,
    _tfm_normalize_angle,
)
