#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Pick demo sequence helper for the panel."""
from __future__ import annotations

import time

from .panel_robot_presets import (
    JOINT_TABLE_POSE_RAD,
    JOINT_PICK_IMAGE_POSE_RAD,
    JOINT_GRASP_DOWN_POSE_RAD,
    JOINT_BASKET_POSE_RAD,
    JOINT_BASKET_DEMO_RELEASE_POSE_RAD,
)
from .panel_config import BASKET_DROP, WORLD_FRAME, BASE_FRAME
from .panel_robot_presets import PICK_DEMO_OBJECT_NAME
from .panel_objects import (
    mark_object_grasped,
    mark_object_attached,
    mark_object_released,
    get_object_state,
    is_on_table,
    update_object_state,
    ObjectOwner,
    ObjectLogicalState,
)
from .panel_readiness import tf_ready_status
from .panel_utils import transform_point_to_frame


def _demo_object_in_basket(panel, timeout_sec: float = 4.0) -> bool:
    """Confirma por posicion que el objeto demo esta en la cesta."""
    start = time.monotonic()
    basket_world = tuple(float(v) for v in BASKET_DROP)
    base_frame = str(getattr(panel, "_base_frame_effective", "") or BASE_FRAME)
    basket_base, _ = transform_point_to_frame(
        basket_world,
        base_frame,
        source_frame=WORLD_FRAME,
    )
    xy_tol_world = 0.35
    z_tol_world = 0.35
    xy_tol_base = 0.30
    z_tol_base = 0.25
    while (time.monotonic() - start) <= timeout_sec:
        st = get_object_state(PICK_DEMO_OBJECT_NAME)
        if st is not None:
            xw, yw, zw = (float(st.position[0]), float(st.position[1]), float(st.position[2]))

            # Criterio en mundo (pose state suele almacenarse en WORLD_FRAME).
            dxw = xw - basket_world[0]
            dyw = yw - basket_world[1]
            dxy_world = (dxw * dxw + dyw * dyw) ** 0.5
            dz_world = abs(zw - basket_world[2])
            world_ok = dxy_world <= xy_tol_world and dz_world <= z_tol_world

            # Criterio alternativo en base_link para tolerar drift de origen.
            base_ok = False
            dxy_base = float("inf")
            dz_base = float("inf")
            if basket_base:
                obj_base, _ = transform_point_to_frame(
                    (xw, yw, zw),
                    base_frame,
                    source_frame=WORLD_FRAME,
                )
                if obj_base:
                    dxb = float(obj_base[0]) - float(basket_base[0])
                    dyb = float(obj_base[1]) - float(basket_base[1])
                    dxy_base = (dxb * dxb + dyb * dyb) ** 0.5
                    dz_base = abs(float(obj_base[2]) - float(basket_base[2]))
                    base_ok = dxy_base <= xy_tol_base and dz_base <= z_tol_base

            detached_ok = (not bool(st.attached)) and (st.owner == ObjectOwner.NONE)
            if detached_ok and (world_ok or base_ok):
                panel._emit_log(
                    "[PICK][DEMO] confirmacion cesta OK "
                    f"world_obj=({xw:.3f},{yw:.3f},{zw:.3f}) "
                    f"world_basket=({basket_world[0]:.3f},{basket_world[1]:.3f},{basket_world[2]:.3f}) "
                    f"dxy_w={dxy_world:.3f} dz_w={dz_world:.3f} "
                    f"dxy_b={dxy_base:.3f} dz_b={dz_base:.3f}"
                )
                return True
        time.sleep(0.2)
    panel._emit_log("[PICK][DEMO] confirmacion cesta NO alcanzada (timeout)")
    return False


def run_pick_demo(panel) -> None:
    panel._log_button("PICK MESA → CESTA")
    panel._emit_log("[DEMO] Inicio pick & place (mesa -> cesta)")
    if not panel._require_ready_basic("PICK DEMO"):
        return
    # Si _require_ready_basic pasó, el sistema está en READY_BASIC o superior.
    # Solo verificar que TF y EE frame estén disponibles (necesarios para pick).
    tf_ok, tf_reason = tf_ready_status(panel)
    if not tf_ok or not bool(panel._ee_frame_effective):
        panel._set_status(f"TF no listo; esperando pick ({tf_reason})", error=False)
        panel._emit_log(f"[PICK] Bloqueado: {panel._tf_not_ready_reason()}")
        return
    panel._emit_log("[PICK] Secuencia manual iniciada (MoveIt no requerido)")
    
    # Normalizar estado previo del objeto demo antes del grasp manual.
    obj_state = get_object_state(PICK_DEMO_OBJECT_NAME)
    if obj_state:
        if obj_state.logical_state in (ObjectLogicalState.GRASPED, ObjectLogicalState.CARRIED):
            panel._emit_log(f"[PICK] Limpiando estado anterior: {PICK_DEMO_OBJECT_NAME} era {obj_state.logical_state.value}")
            update_object_state(
                PICK_DEMO_OBJECT_NAME,
                logical_state=ObjectLogicalState.ON_TABLE,
                owner=ObjectOwner.NONE,
                attached=False,
                reason="pick_demo_cleanup"
            )
            panel._emit_log(f"[PICK] Estado limpiado: {PICK_DEMO_OBJECT_NAME} → ON_TABLE")
        elif obj_state.logical_state in (ObjectLogicalState.SPAWNED, ObjectLogicalState.RELEASED):
            panel._emit_log(
                f"[PICK] Normalizando estado previo: {PICK_DEMO_OBJECT_NAME} era {obj_state.logical_state.value} → ON_TABLE"
            )
            update_object_state(
                PICK_DEMO_OBJECT_NAME,
                logical_state=ObjectLogicalState.ON_TABLE,
                owner=ObjectOwner.NONE,
                attached=False,
                reason="pick_demo_on_table_normalize"
            )
    
    ready, reason = panel._controllers_ready()
    if not ready:
        panel._emit_log(f"[PICK] controladores no listos ({reason})")
        panel._set_status("Controladores no listos; esperando", error=False)
        return
    panel._emit_log("[PICK] target_source=DEMO (manual)")
    panel._set_status("Pick demo: ejecutando secuencia manual…")
    panel._set_motion_lock(True)

    def worker():
        try:
            move_sec = float(panel.joint_time.value()) if panel.joint_time else 3.0
            home_pose = panel._get_home_joint_pose()

            def _joint_error_snapshot(joints):
                names = list(getattr(panel, "UR5_JOINT_NAMES", []) or [])
                if not names:
                    names = [
                        "shoulder_pan_joint",
                        "shoulder_lift_joint",
                        "elbow_joint",
                        "wrist_1_joint",
                        "wrist_2_joint",
                        "wrist_3_joint",
                    ]
                parts = []
                for idx, name in enumerate(names):
                    if idx >= len(joints):
                        break
                    curr = panel._last_joint_positions.get(name)
                    if curr is None:
                        parts.append(f"{name}=n/a")
                        continue
                    diff = abs(float(curr) - float(joints[idx]))
                    parts.append(f"{name}={diff:.3f}")
                return " ".join(parts)

            def _run_joint_step(label, joints, timeout_sec=None, tol_rad=0.02):
                panel._emit_log(f"[PICK] Paso joint: {label}")
                ok, info = panel._publish_joint_trajectory(joints, move_sec)
                if not ok:
                    raise RuntimeError(f"{label} fallo: {info}")
                wait_timeout = move_sec + 2.0 if timeout_sec is None else timeout_sec
                if panel._wait_for_joint_target(joints, wait_timeout, tol_rad=tol_rad):
                    return
                if label in {"HOME", "MESA", "PICK_IMAGE", "HOME_WITH_OBJECT", "CESTA", "CESTA_RELEASE"}:
                    panel._emit_log(
                        f"[PICK][RECOVERY] {label} no alcanzado; reintentando una vez diffs={_joint_error_snapshot(joints)}"
                    )
                    ok_retry, info_retry = panel._publish_joint_trajectory(joints, move_sec)
                    if not ok_retry:
                        raise RuntimeError(f"{label} retry fallo: {info_retry}")
                    retry_timeout = max(wait_timeout, move_sec + 4.0)
                    retry_tol = max(tol_rad, 0.06)
                    if panel._wait_for_joint_target(joints, retry_timeout, tol_rad=retry_tol):
                        panel._emit_log(f"[PICK][RECOVERY] {label} alcanzado tras reintento")
                        return
                raise RuntimeError(
                    f"{label} no alcanzado (timeout) diffs={_joint_error_snapshot(joints)}"
                )

            _run_joint_step("HOME", home_pose)
            _run_joint_step("MESA", JOINT_TABLE_POSE_RAD)

            panel._emit_log("[DEMO] Abriendo pinza en posición MESA")
            panel.signal_run_ui.emit(lambda: panel._command_gripper(False, log_action="PICK", force=True))
            time.sleep(0.6)

            _run_joint_step("PICK_IMAGE", JOINT_PICK_IMAGE_POSE_RAD)

            panel._emit_log("[DEMO] Bajando a pose de grasp (joints)")
            _run_joint_step(
                "GRASP_DOWN_JOINT",
                JOINT_GRASP_DOWN_POSE_RAD,
                timeout_sec=move_sec + 6.0,
                tol_rad=0.08,
            )
            time.sleep(3.0)
            panel._emit_log("[DEMO] Cerrando pinza")
            def _close_and_attach():
                panel._command_gripper(True, log_action="PICK", force=True)
                mark_object_grasped(PICK_DEMO_OBJECT_NAME, reason="demo_grasp")
                mark_object_attached(PICK_DEMO_OBJECT_NAME, reason="demo_attach")

            panel.signal_run_ui.emit(_close_and_attach)
            time.sleep(1.0)

            _run_joint_step("MESA_POST_GRASP", JOINT_TABLE_POSE_RAD, timeout_sec=move_sec + 8.0, tol_rad=0.10)
            _run_joint_step(
                "HOME_WITH_OBJECT",
                home_pose,
                timeout_sec=move_sec + 8.0,
                tol_rad=0.10,
            )
            _run_joint_step(
                "CESTA",
                JOINT_BASKET_POSE_RAD,
                timeout_sec=move_sec + 10.0,
                tol_rad=0.12,
            )
            _run_joint_step(
                "CESTA_RELEASE",
                JOINT_BASKET_DEMO_RELEASE_POSE_RAD,
                timeout_sec=move_sec + 8.0,
                tol_rad=0.08,
            )

            panel._emit_log("[DEMO] Abriendo pinza en cesta")
            def _open_and_release():
                panel._command_gripper(False, log_action="DROP", force=True)
                mark_object_released(PICK_DEMO_OBJECT_NAME, reason="demo_drop")

            panel.signal_run_ui.emit(_open_and_release)
            time.sleep(1.0)
            panel._emit_log("[DEMO] Cerrando pinza en cesta")
            panel.signal_run_ui.emit(lambda: panel._command_gripper(True, log_action="DROP", force=True))
            time.sleep(0.4)

            _run_joint_step("HOME_FINAL", home_pose)

            panel._ui_set_status("Pick demo completado")
            panel._emit_log("[PICK] Secuencia PICK completada.")
            
            # Marcar como exitoso y diferir confirmación de cesta para evitar contenciones del executor
            panel._pick_demo_executed = True
            panel._emit_log("[PICK][DEMO] Deferiendo confirmación de cesta...")
            
            def _deferred_basket_check():
                """Ejecuta verificación de cesta después de dar tiempo al executor"""
                time.sleep(1.0)  # Dar tiempo para que el executor se libere
                if _demo_object_in_basket(panel):
                    def _lock_pick_demo_button() -> None:
                        panel._pick_demo_executed = True
                        panel.btn_pick_demo.setEnabled(False)
                        panel.btn_pick_demo.setToolTip("Ya ejecutado: objeto demo confirmado en cesta")
                        panel._emit_log("[PICK][DEMO] boton deshabilitado (objeto confirmado en cesta)")

                    panel.signal_run_ui.emit(_lock_pick_demo_button)
                else:
                    panel._emit_log("[PICK][DEMO] Cesta no confirmada pero secuencia completada")
                    def _disable_button_anyway() -> None:
                        panel.btn_pick_demo.setEnabled(False)
                        panel.btn_pick_demo.setToolTip("Secuencia completada (objeto en cesta no confirmado visualmente)")
                    panel.signal_run_ui.emit(_disable_button_anyway)
            
            # Ejecutar verificación en thread separado para no bloquear
            panel._pick_demo_checker_thread = panel._run_async(_deferred_basket_check)
            
        except Exception as exc:
            try:
                panel._emit_log("[PICK][RECOVERY] Error detectado; intentando HOME_SAFE")
                _run_joint_step("HOME_SAFE", home_pose, timeout_sec=move_sec + 3.0, tol_rad=0.08)
            except Exception as home_exc:
                panel._emit_log(f"[PICK][RECOVERY] HOME_SAFE falló: {home_exc}")
            panel._ui_set_status(f"Error en pick demo: {exc}", error=True)
            panel._emit_log(f"[PICK] ✗ Error: {exc}")
            # Marcar como ejecutado sin confirmación si falló
            panel._pick_demo_executed = False
        finally:
            panel._set_motion_lock(False)

    panel._run_async(worker)
