#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_ui_state.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""UI gating logic for the UR5 panel."""
from __future__ import annotations

from typing import TYPE_CHECKING

from .panel_state import SystemState

if TYPE_CHECKING:
    from .panel_v2 import ControlPanelV2


def apply_ui_state(panel: "ControlPanelV2", effective_state: SystemState, effective_reason: str) -> None:
    ready_basic = effective_state in (SystemState.READY_BASIC, SystemState.READY_VISION, SystemState.READY_MOVEIT)
    system_error = effective_state == SystemState.ERROR_FATAL
    gz_state = panel._gazebo_state()
    gz_active = gz_state != "GAZEBO_OFF"
    bridge_active = panel._bridge_running or panel._pose_info_active()
    gz_proc_alive = panel._proc_alive(panel.gz_proc)
    bridge_proc_alive = panel._proc_alive(panel.bridge_proc)
    bag_proc_alive = panel._proc_alive(panel.bag_proc)
    moveit_proc_alive = panel._proc_alive(panel.moveit_proc)
    moveit_bridge_proc_alive = panel._proc_alive(panel.moveit_bridge_proc)
    if panel._managed_mode:
        panel.btn_gz_start.setEnabled(False)
        panel.btn_gz_stop.setEnabled(False)
    else:
        panel.btn_gz_start.setEnabled(not gz_active)
        panel.btn_gz_stop.setEnabled(gz_proc_alive)
    if panel._gz_launching:
        panel._set_launching_style(panel.btn_gz_start, True)
        panel.btn_gz_start.setEnabled(False)
    else:
        panel._set_launching_style(panel.btn_gz_start, False)
    panel.btn_debug_joints.setEnabled(gz_active)
    bridge_enabled = gz_active
    panel.bridge_presets.setEnabled(bridge_enabled and not bridge_active)
    panel.bridge_edit.setEnabled(bridge_enabled and not bridge_active)
    if panel._managed_mode:
        panel.btn_bridge_browse.setEnabled(False)
        panel.btn_bridge_start.setEnabled(False)
        panel.btn_bridge_stop.setEnabled(False)
    else:
        panel.btn_bridge_browse.setEnabled(bridge_enabled and not bridge_active)
        panel.btn_bridge_start.setEnabled(bridge_enabled and not bridge_active)
        panel.btn_bridge_stop.setEnabled(bridge_proc_alive)
    if panel._bridge_launching:
        panel._set_launching_style(panel.btn_bridge_start, True)
        panel.btn_bridge_start.setEnabled(False)
    else:
        panel._set_launching_style(panel.btn_bridge_start, False)
    bag_enabled = gz_active and bridge_active
    panel.bag_name.setEnabled(bag_enabled and not bag_proc_alive)
    panel.bag_topics.setEnabled(bag_enabled and not bag_proc_alive)
    panel.btn_bag_start.setEnabled(bag_enabled and not bag_proc_alive)
    panel.btn_bag_stop.setEnabled(bag_proc_alive)
    if panel._managed_mode:
        panel.btn_moveit_start.setEnabled(False)
        panel.btn_moveit_stop.setEnabled(False)
    else:
        panel.btn_moveit_start.setEnabled(not panel._moveit_ready())
        panel.btn_moveit_stop.setEnabled(moveit_proc_alive)
    if panel._moveit_launching:
        panel._set_launching_style(panel.btn_moveit_start, True)
        panel.btn_moveit_start.setEnabled(False)
    else:
        panel._set_launching_style(panel.btn_moveit_start, False)
    moveit_bridge_enabled = ready_basic and panel._tf_ready_state
    if panel._managed_mode:
        panel.btn_moveit_bridge_start.setEnabled(False)
        panel.btn_moveit_bridge_stop.setEnabled(False)
    else:
        panel.btn_moveit_bridge_start.setEnabled(moveit_bridge_enabled and not panel._moveit_bridge_detected())
        panel.btn_moveit_bridge_stop.setEnabled(moveit_bridge_proc_alive)
    if panel._moveit_bridge_launching:
        panel._set_launching_style(panel.btn_moveit_bridge_start, True)
        panel.btn_moveit_bridge_start.setEnabled(False)
    else:
        panel._set_launching_style(panel.btn_moveit_bridge_start, False)

    if panel._managed_mode:
        panel.btn_star.setEnabled(False)
        panel.btn_kill_hard.setEnabled(False)
    else:
        running = panel._system_running()
        panel.btn_star.setEnabled((not running) and (not panel._star_inflight))
        panel.btn_kill_hard.setEnabled(running or panel._star_inflight)

    camera_enabled = bridge_active and not panel._script_motion_active
    panel.camera_topic_combo.setEnabled(camera_enabled)
    panel.btn_camera_refresh.setEnabled(camera_enabled)
    panel.btn_camera_connect.setEnabled(camera_enabled)
    if getattr(panel, "btn_calibrate", None) is not None:
        panel.btn_calibrate.setEnabled(
            camera_enabled
            and not system_error
            and panel._camera_stream_ok
            and panel._calibration_topic_allowed()
        )
    if getattr(panel, "btn_release_objects", None) is not None:
        panel.btn_release_objects.setEnabled(
            camera_enabled
            and panel._camera_stream_ok
            and not panel._objects_release_done
            and not panel._detach_inflight
        )

    camera_ready = panel._camera_stream_ok
    camera_degraded_ok = bool(panel._managed_mode and panel._pose_info_ok and panel._controllers_ok)
    camera_gate_ok = camera_ready or camera_degraded_ok
    if (
        (not camera_ready or panel._gazebo_state() != "GAZEBO_READY")
        and panel._robot_test_done
        and not panel._robot_test_disabled
    ):
        panel._robot_test_done = False
    test_pending = camera_gate_ok and not panel._robot_test_done

    trace_enabled = camera_gate_ok and not test_pending
    if hasattr(panel, "chk_trace_freeze"):
        panel.chk_trace_freeze.setEnabled(trace_enabled)
    if hasattr(panel, "btn_trace_diag"):
        panel.btn_trace_diag.setEnabled(trace_enabled)
    if hasattr(panel, "btn_copy_trace"):
        panel.btn_copy_trace.setEnabled(trace_enabled)
    if hasattr(panel, "btn_save_episode"):
        panel.btn_save_episode.setEnabled(trace_enabled)

    manual_ok, manual_reason = panel._manual_control_status()
    manual_enabled = manual_ok and not panel._script_motion_active and camera_gate_ok and not test_pending
    basic_reason = manual_reason or (effective_reason or effective_state.value)
    if not camera_gate_ok:
        basic_reason = "Cámara no lista"
    elif not camera_ready and camera_degraded_ok:
        basic_reason = "Cámara sin frames (modo degradado)"
    elif test_pending:
        basic_reason = "Ejecuta TEST ROBOT para habilitar"
    panel._set_btn_state(
        panel.btn_send_joints,
        manual_enabled,
        f"Bloqueado: {basic_reason}" if not manual_enabled else "Mover articulaciones",
    )
    panel.joint_time.setEnabled(manual_enabled)
    panel.chk_auto_joints.setEnabled(manual_enabled)
    for slider in panel.joint_sliders:
        slider.setEnabled(manual_enabled)

    motion_enabled = manual_ok and not panel._script_motion_active
    motion_tip = "" if motion_enabled else f"Bloqueado: {basic_reason}"
    traj_topic = panel._select_traj_topic()
    externals = panel._external_publishers_for_topic(traj_topic) if traj_topic else []
    external_block = bool(externals)
    external_tip = ""
    moveit_only = False
    if externals:
        moveit_node = "panel_v2_moveit_publisher"
        bridge_node = "ur5_moveit_bridge"
        ns = panel.ros_worker.node_namespace()
        moveit_nodes = {
            moveit_node,
            f"/{moveit_node}",
            f"{ns}/{moveit_node}".replace("//", "/"),
            bridge_node,
            f"/{bridge_node}",
            f"{ns}/{bridge_node}".replace("//", "/"),
        }
        # Allow externally launched MoveIt bridge nodes too (not only panel-started ones).
        moveit_only = panel._moveit_bridge_detected() and all(pub in moveit_nodes for pub in externals)
    if external_block and not moveit_only:
        external_tip = f"Publishers externos en {traj_topic}: {', '.join(externals)}"
        motion_enabled = False
        motion_tip = f"Bloqueado: {external_tip}"
        if external_tip != panel._external_motion_block_reason:
            panel._external_motion_block_reason = external_tip
        panel._set_robot_test_blocked(external_tip)
    elif panel._external_motion_block_reason:
        panel._external_motion_block_reason = None
        panel._set_robot_test_blocked(None)
    test_locked = bool(panel._robot_test_disabled)
    test_locked_tip = "TEST ROBOT ya ejecutado"

    if external_block and not moveit_only:
        block_tip = f"Bloqueado: {external_tip}"
        panel._set_btn_state(panel.btn_test_robot, False, test_locked_tip if test_locked else block_tip)
        panel._set_btn_state(panel.btn_home, False, block_tip)
        panel._set_btn_state(panel.btn_table, False, block_tip)
        panel._set_btn_state(panel.btn_basket, False, block_tip)
        panel._set_btn_state(panel.btn_gripper, False, block_tip)
    elif moveit_only:
        panel._set_robot_test_blocked(None)
        moveit_tip = "MoveIt bridge activo; se pausará antes del test"
        panel._set_btn_state(panel.btn_test_robot, False if test_locked else motion_enabled, test_locked_tip if test_locked else moveit_tip)
        # HOME/MESA/CESTA habilitados si test ya completado
        if panel._robot_test_done:
            panel._set_btn_state(panel.btn_home, motion_enabled, motion_tip)
            panel._set_btn_state(panel.btn_table, motion_enabled, motion_tip)
            panel._set_btn_state(panel.btn_basket, motion_enabled, motion_tip)
        else:
            block_tip = "Bloqueado: MoveIt bridge activo"
            panel._set_btn_state(panel.btn_home, False, block_tip)
            panel._set_btn_state(panel.btn_table, False, block_tip)
            panel._set_btn_state(panel.btn_basket, False, block_tip)
        panel._set_btn_state(panel.btn_gripper, False, "Bloqueado: MoveIt bridge activo")
    elif not camera_gate_ok:
        block_tip = "Cámara no lista"
        # Deshabilitar si ya se ejecutó
        if test_locked:
            panel._set_btn_state(panel.btn_test_robot, False, test_locked_tip)
        else:
            panel._set_btn_state(panel.btn_test_robot, False, block_tip)
        panel._set_btn_state(panel.btn_home, False, block_tip)
        panel._set_btn_state(panel.btn_table, False, block_tip)
        panel._set_btn_state(panel.btn_basket, False, block_tip)
        panel._set_btn_state(panel.btn_gripper, False, block_tip)
    elif test_pending:
        block_tip = "Ejecuta TEST ROBOT para habilitar"
        # Deshabilitar si ya se ejecutó
        if test_locked:
            panel._set_btn_state(panel.btn_test_robot, False, test_locked_tip)
        else:
            panel._set_btn_state(panel.btn_test_robot, motion_enabled, motion_tip)
        panel._set_btn_state(panel.btn_home, False, block_tip)
        panel._set_btn_state(panel.btn_table, False, block_tip)
        panel._set_btn_state(panel.btn_basket, False, block_tip)
        panel._set_btn_state(panel.btn_gripper, False, block_tip)
    else:
        # Deshabilitar si ya se ejecutó
        if test_locked:
            panel._set_btn_state(panel.btn_test_robot, False, test_locked_tip)
        else:
            panel._set_btn_state(panel.btn_test_robot, motion_enabled, motion_tip)
        panel._set_btn_state(panel.btn_home, motion_enabled, motion_tip)
        panel._set_btn_state(panel.btn_table, motion_enabled, motion_tip)
        panel._set_btn_state(panel.btn_basket, motion_enabled, motion_tip)
        panel._set_btn_state(panel.btn_gripper, motion_enabled, motion_tip)
    panel._schedule_controller_check()
    pick_ok, pick_reason = panel._moveit_control_status()
    pick_enabled = pick_ok and bool(panel._ee_frame_effective)
    pick_tip = "Requiere MoveIt, cámara y objetos estables"
    # Demo pick: NO requiere MoveIt (usa trayectoria de joints directa).
    demo_ready = (
        panel._controllers_ok
        and panel._tf_ready_state
        and bool(panel._ee_frame_effective)
        and not panel._pick_demo_executed  # Deshabilitar si ya se ejecutó
    )
    demo_tip = "Demo (secuencia joints, sin MoveIt)"
    if not test_pending:
        if panel._pick_demo_executed:
            panel._set_btn_state(panel.btn_pick_demo, False, "PICK DEMO ya ejecutado (una sola vez)")
        else:
            panel._set_btn_state(panel.btn_pick_demo, demo_ready, demo_tip if demo_ready else pick_tip)
    else:
        block_tip = "Ejecuta TEST ROBOT para habilitar"
        panel._set_btn_state(panel.btn_pick_demo, False, block_tip)
    pick_object_ready = pick_ok and panel._pose_info_ok and panel._tf_ready_state and bool(panel._selected_object)
    if camera_gate_ok and not test_pending:
        panel._set_btn_state(
            panel.btn_pick_object,
            pick_object_ready,
            "PICK objeto seleccionado" if pick_object_ready else ("Selecciona un objeto" if not panel._selected_object else pick_tip),
        )
    else:
        panel._set_btn_state(panel.btn_pick_object, False, block_tip)
    tfm_ready = (
        camera_ready
        and bool(panel.tfm_module)
        and not test_pending
        and panel._joint_limits_ok
        and panel._objects_release_done
    )
    if not camera_ready:
        tfm_block_tip = "Cámara no lista"
    elif test_pending:
        tfm_block_tip = "Ejecuta TEST ROBOT para habilitar"
    elif not panel._joint_limits_ok:
        tfm_block_tip = panel._joint_limits_err or "Limites articulares no cargados"
    elif not panel.tfm_module:
        tfm_block_tip = "Modelo no disponible"
    else:
        tfm_block_tip = ""
    # Selector y aplicar experimento no deben depender de cámara/test;
    # solo los botones de ejecución (infer/visualize/publish) mantienen gating estricto.
    tfm_selector_ready = bool(panel.tfm_module)
    tfm_selector_tip = "" if tfm_selector_ready else "Modelo no disponible"
    tfm_has_grasp = bool(panel._last_grasp_px)
    tfm_grasp_tip = "" if tfm_has_grasp else "Primero infiere o recibe un grasp"
    # Botones de ejecución requieren _objects_release_done
    tfm_controls_ready = bool(panel.tfm_module) and panel._objects_release_done
    tfm_controls_tip = "" if tfm_controls_ready else ("Modelo no disponible" if not panel.tfm_module else tfm_block_tip)
    if hasattr(panel, "combo_tfm_experiment"):
        panel.combo_tfm_experiment.setEnabled(tfm_selector_ready)
    panel._set_btn_state(panel.btn_tfm_apply, tfm_selector_ready, tfm_selector_tip)
    panel._set_btn_state(panel.btn_tfm_infer, tfm_ready, "" if tfm_ready else tfm_block_tip)
    panel._set_btn_state(
        panel.btn_tfm_visualize,
        tfm_ready and tfm_has_grasp,
        "" if (tfm_ready and tfm_has_grasp) else (tfm_block_tip if not tfm_ready else tfm_grasp_tip),
    )
    panel._set_btn_state(
        panel.btn_tfm_publish,
        tfm_ready and tfm_has_grasp,
        "" if (tfm_ready and tfm_has_grasp) else (tfm_block_tip if not tfm_ready else tfm_grasp_tip),
    )
    panel._set_btn_state(panel.btn_tfm_reset, tfm_controls_ready, tfm_controls_tip)
    if panel._script_motion_active:
        busy_tip = "Robot en movimiento"
        panel._set_btn_state(panel.btn_test_robot, False, busy_tip)
        panel._set_btn_state(panel.btn_home, False, busy_tip)
        panel._set_btn_state(panel.btn_table, False, busy_tip)
        panel._set_btn_state(panel.btn_basket, False, busy_tip)
        panel._set_btn_state(panel.btn_gripper, False, busy_tip)
        panel._set_btn_state(panel.btn_pick_demo, False, busy_tip)
        panel._set_btn_state(panel.btn_pick_object, False, busy_tip)
        if panel.tfm_module:
            panel._set_btn_state(panel.btn_tfm_infer, False, busy_tip)
            panel._set_btn_state(panel.btn_tfm_visualize, False, busy_tip)
            panel._set_btn_state(panel.btn_tfm_publish, False, busy_tip)
    # La lista de objetos solo necesita pose_info; no debe quedar bloqueada si
    # la vista de camara tarda en marcarse como ready.
    pick_ui_enabled = panel._pose_info_ok
    if hasattr(panel, "obj_panel"):
        panel.obj_panel.setEnabled(pick_ui_enabled)
    if hasattr(panel, "camera_view"):
        panel.camera_view.setEnabled(panel._camera_stream_ok)
    if ready_basic and motion_enabled and panel._controllers_ok and not pick_enabled:
        reason = pick_reason or panel._tf_not_ready_reason()
        if not panel._ee_frame_effective:
            reason = "EE frame no disponible"
        if reason != panel._pick_block_reason:
            panel._set_status(f"PICK en espera: {reason}", error=False)
            panel._emit_log(f"[PICK] Bloqueado: {reason}")
            panel._pick_block_reason = reason
    else:
        panel._pick_block_reason = None

    panel.world_combo.setEnabled(not gz_active)
    panel.mode_combo.setEnabled(not gz_active)
    panel.btn_world_browse.setEnabled(not gz_active)
    panel._update_panel_flow_state(
        ready_basic=ready_basic,
        camera_ready=camera_ready,
        test_pending=test_pending,
        pick_enabled=pick_enabled,
        system_error=system_error,
    )
    panel._state_event.set()
