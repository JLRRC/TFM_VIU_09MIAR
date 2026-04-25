#!/usr/bin/env python3
"""Motion control, UI state, joint convergence, and panel flow methods for ControlPanelV2."""
from __future__ import annotations

import math
import time
from typing import List, Tuple

from PyQt5.QtCore import QTimer

from .panel_config import (
    CONTROLLER_READY_TIMEOUT_SEC,
    REACH_OVERLAY_POINTS,
    REACH_OVERLAY_Z,
    UR5_BASE_X,
    UR5_BASE_Y,
    UR5_JOINT_NAMES,
    UR5_REACH_RADIUS,
)
from .panel_env import effective_mode
from .panel_motion_helpers import build_joint_trajectory
from .panel_objects import recalc_object_states
from .panel_robot_presets import JOINT_HOME_POSE_RAD
from .panel_state import SystemState
from .panel_utils import load_home_pose, table_xy_to_pixel, world_xyz_to_pixel


def _log_exception(label: str, exc: Exception) -> None:
    try:
        import traceback
        print(f"[EXC][{label}] {exc}\n{traceback.format_exc()}")
    except Exception:
        pass


def _update_ui_state(panel):
    """Inicializar UI: arranque manual desde los controles del panel."""
    panel._gz_running = False
    panel._bridge_running = False
    panel._bag_running = False
    panel._moveit_running = False
    panel._moveit_bridge_running = False
    panel._auto_joint2_move_done = False
    
    # Gazebo controls
    panel.btn_gz_start.setEnabled(True)
    panel.btn_gz_stop.setEnabled(False)
    panel.btn_debug_joints.setEnabled(False)
    panel.btn_debug_joints.setChecked(False)
    panel.world_combo.setEnabled(True)
    panel.mode_combo.setEnabled(True)
    panel.btn_world_browse.setEnabled(True)
    
    # Bridge controls
    panel.btn_bridge_start.setEnabled(False)
    panel.btn_bridge_stop.setEnabled(False)
    panel.bridge_presets.setEnabled(False)
    panel.bridge_edit.setEnabled(False)
    panel.btn_bridge_browse.setEnabled(False)
    
    # Bag controls
    panel.btn_bag_start.setEnabled(False)
    panel.btn_bag_stop.setEnabled(False)
    panel.bag_name.setEnabled(False)
    panel.bag_topics.setEnabled(False)

    # MoveIt controls
    panel.btn_moveit_start.setEnabled(True)
    panel.btn_moveit_stop.setEnabled(False)
    panel.btn_moveit_bridge_start.setEnabled(False)
    panel.btn_moveit_bridge_stop.setEnabled(False)
    
    # Cámara (deshabilitada hasta que bridge esté activo)
    panel.camera_topic_combo.setEnabled(False)
    panel.btn_camera_refresh.setEnabled(False)
    panel.btn_camera_connect.setEnabled(False)
    if getattr(panel, "btn_camera_far_front", None) is not None:
        panel.btn_camera_far_front.setEnabled(False)
    if getattr(panel, "btn_camera_top", None) is not None:
        panel.btn_camera_top.setEnabled(False)
    if getattr(panel, "btn_camera_wrist", None) is not None:
        panel.btn_camera_wrist.setEnabled(False)
    if panel.btn_calibrate is not None:
        panel.btn_calibrate.setEnabled(False)
    if panel.btn_release_objects is not None:
        panel.btn_release_objects.setEnabled(False)
    if hasattr(panel, "chk_trace_freeze"):
        panel.chk_trace_freeze.setEnabled(False)
    if hasattr(panel, "btn_trace_diag"):
        panel.btn_trace_diag.setEnabled(False)
    if hasattr(panel, "btn_copy_trace"):
        panel.btn_copy_trace.setEnabled(False)
    if hasattr(panel, "btn_save_episode"):
        panel.btn_save_episode.setEnabled(False)
    
    # Control manual de joints: por defecto lo mantenemos disponible para depuración.
    manual_boot_enabled = bool(getattr(panel, "_manual_controls_always_enabled", False))
    panel.btn_send_joints.setEnabled(manual_boot_enabled)
    panel.joint_time.setEnabled(manual_boot_enabled)
    panel.chk_auto_joints.setEnabled(manual_boot_enabled)
    for slider in panel.joint_sliders:
        slider.setEnabled(manual_boot_enabled)

    # Botones de movimiento (bloqueados hasta bridge)
    panel.btn_debug_motion.setEnabled(True)
    panel._set_debug_motion_button_waiting(False)
    panel.btn_home.setEnabled(False)
    panel.btn_table.setEnabled(False)
    panel.btn_basket.setEnabled(False)
    panel.btn_gripper.setEnabled(False)
    panel.btn_pick_demo.setEnabled(False)
    panel.btn_pick_object.setEnabled(False)
    
    # Debug y otros
    panel.btn_debug_joints.setEnabled(True)
    panel.btn_debug_logs.setEnabled(True)
    panel.btn_kill_hard.setEnabled(False)
    panel.btn_close_terminal.setEnabled(True)

def _effective_mode(panel) -> str:
    m = panel.mode_combo.currentText().strip().lower()
    return effective_mode(m)

def _apply_home_joint2_offset(panel, retries: int = 2):
    """Mover a HOME ajustando joint2 un -20% (sentido negativo) al lanzar Gazebo."""
    if panel._auto_joint2_move_done:
        return
    panel._schedule_controller_check()
    if not panel._bridge_running:
        if retries > 0:
            panel._schedule_home_offset_retry(1500, retries - 1)
        return
    if not panel._gz_running:
        if retries > 0:
            panel._schedule_home_offset_retry(1500, retries - 1)
        return

    if not panel._controllers_ok:
        if retries > 0:
            panel._log("[AUTO] Controller manager no listo, reintentando en 2s")
            panel._schedule_home_offset_retry(2000, retries - 1)
        else:
            panel._log_warning("[AUTO] Controller manager no disponible (ajuste automático cancelado)")
        return

    home = load_home_pose()
    if len(home) < 6:
        panel._log_warning("[AUTO] HOME inválido, no se aplica offset joint2")
        return

    target = list(home[:6])
    base = abs(target[1])
    if base < 1e-3:
        base = 0.25  # fallback pequeño si HOME era ~0
    offset = base * 0.20
    target[1] = -offset

    def worker():
        panel._set_motion_lock(True)
        try:
            panel._log(f"[AUTO] Ajuste joint2=-20% desde HOME -> {target[1]:.3f} rad")
            ok, info = panel._publish_joint_trajectory(target, 3.0)
            if ok:
                time.sleep(3.15)
                panel._auto_joint2_move_done = True
                panel._ui_set_status("AUTO: joint2 ajustado (-20% HOME)")
            else:
                panel._log_warning(f"[AUTO] Falló mover joint2 (-20%): {info}")
                if retries > 0:
                    panel.signal_schedule_home_offset.emit(2000, retries - 1)
        finally:
            panel._set_motion_lock(False)
            recalc_object_states("manual_move")

    panel._run_async(worker)

def _schedule_home_offset_retry(panel, delay_ms: int, retries: int) -> None:
    QTimer.singleShot(delay_ms, lambda: panel._apply_home_joint2_offset(retries=retries))

def _get_home_joint_pose(panel) -> List[float]:
    home = load_home_pose()
    if len(home) >= 6:
        return list(home[:6])
    return list(JOINT_HOME_POSE_RAD)

def _get_gripper_force(panel) -> float:
    """
    Obtener la fuerza actual del gripper RG2.
    
    Retorna:
        float: Fuerza en Newton (0.0 si no disponible)
    
    Nota: Esta es una implementación simplificada que estima la fuerza
    basada en la corriente del motor del gripper. En un sistema real,
    se usaría la retroalimentación de fuerza del gripper.
    """
    try:
        # Intentar obtener fuerza desde el estado del gripper
        # En MoveIt/ROS 2, esto vendría de:
        # - Topic: /rg2/gripper_cmd/gripper_force (si está disponible)
        # - Topic: /rg2/gripper_mimic_joint_follower/state
        # - Servicio: /rg2_controller/get_gripper_force (si existe)
        
        # Para ahora, implementamos heurística simple:
        # Si el gripper está "cerrado" y hay resistencia, estimamos fuerza
        
        if not hasattr(panel, '_gripper_force_estimate'):
            panel._gripper_force_estimate = 0.0
        
        # Lógica: Si el gripper se está moviendo hacia cierre pero encuentra resistencia,
        # la estimamos como contacto (fuerza > 0.1 N)
        # Este es un placeholder - en producción, leer desde ROS topic
        
        # TEMPORAL: Retornar fuerza ficticia (0.5 N cuando está cerrado)
        # TODO: Integrar con RG2_GRIPPER topic real
        if hasattr(panel, '_gripper_is_closed'):
            if panel._gripper_is_closed:
                return 0.5  # 0.5 N cuando está cerrado (estimación)
        
        return 0.0  # Sin contacto
        
    except Exception as e:
        panel._log(f"[GRIPPER] Error leyendo fuerza: {e}")
        return 0.0

def _wait_for_joint_convergence(panel,
    target_pose: list,
    timeout_sec: float,
    tolerance_rad: float = 0.03,
    label: str = "BASELINE",
) -> tuple:
    """Espera (en hilo worker) a que los joints UR5 converjan a target_pose.

    - Lee panel._last_joint_positions (dict {nombre: rad}, actualizado por _on_joint_state).
    - Mapea por UR5_JOINT_NAMES, sin asumir el orden del mensaje /joint_states.
    - Sondea cada 50 ms; emite [JOINT_ERR] una vez por segundo para no saturar el log.

    Retorna:
        (True,  "converged")         todos los joints dentro de tolerance_rad
        (False, "timeout ...")       se agotó timeout_sec
        (False, "no_joint_state")    sin datos de joint_states durante todo el timeout
    """
    panel._log(
        f"[BASELINE][WAIT_JOINTS] target={label} "
        f"timeout={timeout_sec:.1f}s tol={tolerance_rad:.4f}rad"
    )
    deadline      = time.time() + timeout_sec
    poll_interval = 0.05   # 50 ms
    log_interval  = 1.0    # emitir JOINT_ERR cada 1 s
    next_log_t    = time.time() + log_interval

    while time.time() < deadline:
        pos_map = dict(getattr(panel, "_last_joint_positions", {}) or {})

        if not pos_map:
            time.sleep(poll_interval)
            continue

        errors: list = []
        missing: list = []
        for idx, name in enumerate(UR5_JOINT_NAMES):
            if idx >= len(target_pose):
                break
            curr = pos_map.get(name)
            if curr is None:
                missing.append(name)
            else:
                errors.append(abs(curr - target_pose[idx]))

        if missing:
            # Joints aún no recibidos; esperar siguiente ciclo
            time.sleep(poll_interval)
            continue

        max_err = max(errors) if errors else 0.0

        if time.time() >= next_log_t:
            panel._log(
                f"[BASELINE][JOINT_ERR] target={label} max_err={max_err:.4f}rad"
            )
            next_log_t = time.time() + log_interval

        if max_err <= tolerance_rad:
            panel._log(
                f"[BASELINE][CONVERGED] target={label} max_err={max_err:.4f}rad"
            )
            return True, "converged"

        time.sleep(poll_interval)

    # ── timeout ────────────────────────────────────────────────────────────
    pos_map = dict(getattr(panel, "_last_joint_positions", {}) or {})
    if not pos_map:
        panel._log(f"[BASELINE][TIMEOUT] target={label} no_joint_state")
        return False, "no_joint_state"

    errors = []
    for idx, name in enumerate(UR5_JOINT_NAMES):
        if idx >= len(target_pose):
            break
        curr = pos_map.get(name)
        if curr is not None:
            errors.append(abs(curr - target_pose[idx]))

    max_err = max(errors) if errors else float("nan")
    panel._log(f"[BASELINE][TIMEOUT] target={label} max_err={max_err:.4f}rad")
    return False, f"timeout after {timeout_sec:.1f}s max_err={max_err:.4f}rad"

def _run_baseline_motion(panel, target_name: str, pose_fn) -> None:
    """Ejecuta un movimiento Baseline (HOME/MESA/CESTA) con bloqueo de UI y logs estandarizados.

    Garantías:
    - Rechaza doble pulsación mediante _baseline_busy.
    - Bloquea joint sliders, gripper y todos los botones via _script_motion_active.
    - Valida llegada real con _wait_for_joint_convergence en vez de time.sleep fijo.
    - Restaura siempre la UI en el bloque finally, incluso ante excepción.
    """
    if getattr(panel, "_baseline_busy", False):
        panel._log(
            f"[BASELINE][BUSY] target={target_name} "
            "Robot ocupado: espere a que termine el movimiento actual"
        )
        return
    if not panel._require_manual_ready(target_name):
        return
    panel._log(f"[BASELINE][START] target={target_name}")
    panel._set_status(f"Moviendo a {target_name}…")
    move_sec = float(panel.joint_time.value()) if panel.joint_time else 3.0
    panel._baseline_busy = True
    panel._set_motion_lock(True)
    panel._log("[BASELINE][UI_LOCK] enabled=False")

    def worker():
        try:
            pose = pose_fn()
            ok, info = panel._publish_joint_trajectory(pose, move_sec)
            if not ok:
                panel._ui_set_status(f"{target_name} falló: {info}", error=True)
                panel._log(f"[BASELINE][ERROR] target={target_name} error={info}")
                panel._log(f"[BASELINE][DONE] target={target_name} success=False")
                return
            converged, reason = panel._wait_for_joint_convergence(
                pose,
                timeout_sec=move_sec + 2.0,
                tolerance_rad=0.03,
                label=target_name,
            )
            if converged:
                panel._ui_set_status(f"{target_name} ejecutado (JointTrajectory)")
                panel._log(f"[BASELINE][DONE] target={target_name} success=True")
            else:
                panel._ui_set_status(
                    f"{target_name} no confirmó convergencia: {reason}", error=True
                )
                panel._log(f"[BASELINE][ERROR] target={target_name} error={reason}")
                panel._log(f"[BASELINE][DONE] target={target_name} success=False")
        except Exception as exc:
            panel._log(f"[BASELINE][ERROR] target={target_name} error={exc}")
        finally:
            panel._baseline_busy = False
            panel._set_motion_lock(False)
            panel._log("[BASELINE][UI_LOCK] enabled=True")
            recalc_object_states("manual_move")

    panel._run_async(worker)

def _go_home(panel):
    panel._log_button("Go HOME")
    panel._run_baseline_motion("HOME", panel._get_home_joint_pose)
def _set_test_failed(panel, reason: str) -> None:
    reason_txt = (reason or "unknown").strip()
    panel._robot_test_substate = "FAILED"
    panel._robot_test_last_failure = reason_txt
    panel._ui_set_status(f"TEST ROBOT falló: {reason_txt}", error=True)
    panel._emit_log(f"[STATE] TEST_FAILED(reason={reason_txt})")
    panel._audit_append(
        "logs/test_robot.log",
        f"[TEST] RESULT=FAIL reason={reason_txt}",
    )
    # TEST failure must not latch global ERROR; keep system operational.
    if panel._system_error_reason.startswith("drop:"):
        panel._system_error_reason = ""
    if (
        not panel._managed_mode
        and panel._system_state != SystemState.ERROR_FATAL
        and panel._gazebo_state() == "GAZEBO_READY"
        and panel._controllers_ok
    ):
        panel._set_system_state(SystemState.READY_VISION, f"test_failed: {reason_txt}")

def _set_robot_test_done(panel, done: bool) -> None:
    panel._robot_test_done = done
    if done:
        panel._robot_test_disabled = True
        # FASE 5: Log claro del cambio de estado TEST -> PICK.
        panel._emit_log("[STATE] TEST_PASSED enabling PICK disabling TEST")
        # FASE 5: Habilitar PICK buttons explícitamente.
        pick_ok, _pick_reason = panel._moveit_control_status()
        pick_enabled = pick_ok and bool(panel._ee_frame_effective)
        if pick_enabled:
            demo_ready = (
                panel._moveit_ready()
                and panel._controllers_ok
                and panel._tf_ready_state
                and bool(panel._ee_frame_effective)
                and not panel._pick_demo_executed
            )
            if demo_ready:
                panel.btn_pick_demo.setEnabled(True)
                panel.btn_pick_demo.setToolTip("Demo (sin exigir estabilización de objetos)")
            pick_obj_ready = (
                pick_ok
                and panel._pose_info_ok
                and panel._tf_ready_state
                and panel._camera_stream_ok
                and bool(panel._selected_object)
            )
            if pick_obj_ready:
                panel.btn_pick_object.setEnabled(True)
                panel.btn_pick_object.setToolTip("PICK objeto seleccionado")
            panel._emit_log("[STATE] PICK_READY: PICK buttons enabled")
        # FASE 5: HOME/MESA/CESTA también se habilitan.
        panel.btn_home.setEnabled(True)
        panel.btn_table.setEnabled(True)
        panel.btn_basket.setEnabled(True)
    panel._refresh_controls()

def _set_panel_flow_state(panel, state: str, reason: str = "") -> None:
    state_txt = str(state or "BOOT").strip().upper() or "BOOT"
    reason_txt = str(reason or "").strip()
    prev_state = str(getattr(panel, "_panel_flow_state", "BOOT") or "BOOT")
    prev_reason = str(getattr(panel, "_panel_flow_reason", "") or "")
    panel._panel_flow_state = state_txt
    panel._panel_flow_reason = reason_txt
    if state_txt != prev_state or reason_txt != prev_reason:
        panel._emit_log(
            f"[PANEL_FLOW] {prev_state} -> {state_txt}"
            + (f" reason={reason_txt}" if reason_txt else "")
        )

def _update_panel_flow_state(panel,
    *,
    ready_basic: bool,
    camera_ready: bool,
    pick_enabled: bool,
    system_error: bool,
) -> None:
    if system_error:
        panel._set_panel_flow_state("ERROR", panel._system_state_reason or "error_fatal")
        return
    if panel._script_motion_active:
        panel._set_panel_flow_state("PICK_RUNNING", "script_motion_active")
        return
    if not ready_basic:
        panel._set_panel_flow_state("BOOT", panel._system_state_reason or "waiting_basic")
        return
    tf_ok, tf_reason = panel._tf_chain_ready_status()
    camera_ok = (not panel._camera_required) or camera_ready
    if not tf_ok or not panel._controllers_ok or not camera_ok:
        reason = tf_reason if not tf_ok else ("controllers_not_ready" if not panel._controllers_ok else "camera_not_ready")
        panel._set_panel_flow_state("READY_BASIC", reason)
        return
    if not pick_enabled:
        panel._set_panel_flow_state("TEST_PASSED", "awaiting_pick_ready")
        return
    panel._set_panel_flow_state("PICK_READY", "pick_enabled")

def _compute_reach_overlay_points(panel, w: int, h: int) -> List[Tuple[int, int]]:
    if w <= 0 or h <= 0:
        return []
    points: List[Tuple[int, int]] = []
    count = max(12, int(REACH_OVERLAY_POINTS))
    step = (2.0 * math.pi) / float(count)
    for idx in range(count):
        ang = step * idx
        x = UR5_BASE_X + UR5_REACH_RADIUS * math.cos(ang)
        y = UR5_BASE_Y + UR5_REACH_RADIUS * math.sin(ang)
        pix = world_xyz_to_pixel(x, y, REACH_OVERLAY_Z, w, h)
        if not pix:
            pix = table_xy_to_pixel(x, y, w, h)
        if pix:
            points.append(pix)
    return points

def _step_joint(panel, idx: int, direction: int):
    slider = panel.joint_sliders[idx]
    slider.setValue(slider.value() + direction)

def _maybe_send_auto(panel):
    if panel.chk_auto_joints.isChecked():
        panel._send_joints()

def _send_joints(panel):
    panel._log_button("Send joints")
    if getattr(panel, "_pick_moveit_phase_active", False):
        panel._set_status("Movimiento manual bloqueado: PICK_OBJ MoveIt en ejecución", error=True)
        panel._emit_log(
            "[MANUAL] BLOCKED: movimiento manual durante fase MoveIt de PICK_OBJ"
        )
        return
    if panel._system_state == SystemState.ERROR_FATAL:
        panel._set_status("Movimiento manual bloqueado: ERROR_FATAL", error=True)
        panel._emit_log_throttled(
            "SAFETY:manual:ERROR_FATAL",
            "[SAFETY] Movimiento manual bloqueado: ERROR_FATAL",
        )
        return
    if not panel._require_manual_ready("Movimiento manual"):
        return
    if panel._manual_inflight:
        panel._manual_pending = True
        panel._set_status("Movimiento manual en curso…", error=False)
        return

    def worker():
        panel._manual_inflight = True
        try:
            if not panel._ros_worker_started:
                panel._ensure_ros_worker_started()
            if not panel.ros_worker.node_ready():
                panel._ui_set_status("Nodo ROS no listo", error=True)
                return
            ok, reason = panel._wait_for_controllers_ready(CONTROLLER_READY_TIMEOUT_SEC)
            if not ok:
                panel._ui_set_status(f"Controladores no listos: {reason}", error=True)
                return

            topic = panel._select_traj_topic()
            if panel._debug_logs_enabled:
                panel._log(f"[MANUAL] Topic: {topic}")
            positions = [round(p, 4) for p in panel._current_joint_positions_rad()]
            tsec = float(panel.joint_time.value())
            sec = max(0.0, tsec)
            # Debug: mostrar valores enviados
            if panel._debug_logs_enabled:
                pos_str = ", ".join([f"{p:+.3f}" for p in positions])
                panel._log(f"[MANUAL] Enviando joints: [{pos_str}] (t={sec:.1f}s)")

            pub = panel._get_traj_publisher(topic)
            if not pub:
                panel._ui_set_status("Publisher JointTrajectory no disponible", error=True)
                return
            if getattr(panel, "_pick_moveit_phase_active", False):
                panel._ui_set_status(
                    "Movimiento manual bloqueado: PICK_OBJ MoveIt en ejecución",
                    error=True,
            )
                panel._emit_log(
                    "[MANUAL] BLOCKED: intento de publicar durante fase MoveIt de PICK_OBJ"
            )
                return
            panel._emit_log("[MANUAL] Executing direct JointTrajectory (MoveIt bypassed)")
            traj = build_joint_trajectory(
                positions,
                sec,
                UR5_JOINT_NAMES,
            )
            if panel._traj_publish_inflight:
                panel._emit_log("[MANUAL] WARN: publish JointTrajectory solapado")
            panel._traj_publish_inflight = True
            try:
                pub.publish(traj)
                panel._ui_set_status(f"Trayectoria enviada a {topic}")
            finally:
                panel._traj_publish_inflight = False
            if panel._debug_logs_enabled:
                panel._log("[MANUAL] ✓ Publicación JointTrajectory")
        finally:
            panel._manual_inflight = False
            if panel._manual_pending:
                panel._manual_pending = False
                # Emitir señal thread-safe en lugar de QTimer.singleShot()
                panel.retry_send_joints.emit()

    panel._run_async(worker)

def _send_joints_retry(panel):
    """Retry de _send_joints después de 200ms (thread-safe desde worker)."""
    if panel._system_state == SystemState.ERROR_FATAL:
        panel._set_status("Movimiento manual bloqueado: ERROR_FATAL", error=True)
        panel._emit_log_throttled(
            "SAFETY:manual:ERROR_FATAL",
            "[SAFETY] Movimiento manual bloqueado: ERROR_FATAL",
        )
        return
    QTimer.singleShot(200, panel._send_joints)
