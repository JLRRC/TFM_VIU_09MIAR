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


def _apply_ui_state_launchers(
    panel: "ControlPanelV2",
    *,
    gz_active: bool,
    gz_proc_alive: bool,
    bridge_active: bool,
    bridge_proc_alive: bool,
    bag_proc_alive: bool,
    moveit_proc_alive: bool,
    moveit_bridge_proc_alive: bool,
    ready_basic: bool,
) -> None:
    """F3-step11a: gating de botones launcher (gz/bridge/bag/moveit/star).

    Aplica setEnabled + _set_launching_style a btn_gz_start/stop, btn_bridge_*,
    btn_moveit_*, btn_moveit_bridge_*, btn_star/kill_hard, btn_debug_joints,
    bridge_presets/edit, bag_name/topics. Respeta panel._managed_mode (todos
    deshabilitados) y los flags de _*_launching (estilo "lanzando…").
    """
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


def _apply_ui_state_tfm_buttons(
    panel: "ControlPanelV2",
    *,
    btn_tfm_apply,
    btn_tfm_memoria_case,
    btn_tfm_infer,
    btn_tfm_visualize,
    btn_tfm_publish,
    btn_tfm_reset,
) -> None:
    """F3-step11b: gating de los 6 botones TFM + 3 checks/combo.

    Calcula infer_ok / tfm_experiment_ready / tfm_action_ready / tfm_selector_
    ready / tfm_controls_tip / infer_btn_ready / infer_btn_tip y aplica
    setEnabled a combo_tfm_experiment + chk_tfm_repro_mode + chk_tfm_raw_output
    + 6 botones tfm vía panel._set_btn_state.
    """
    infer_ok, infer_reason = panel._tfm_infer_ready_status()
    tfm_experiment_ready, tfm_experiment_reason = panel._tfm_experiment_ready_status()
    tfm_action_ready = (
        tfm_experiment_ready
        and not panel._tfm_infer_inflight
        and not panel._tfm_execute_inflight
    )
    if panel._tfm_infer_inflight:
        tfm_block_tip = "Inferencia en curso"
    elif panel._tfm_execute_inflight:
        tfm_block_tip = "Ejecución en curso"
    elif not tfm_experiment_ready:
        tfm_block_tip = tfm_experiment_reason or "Aplica un experimento primero"
    elif not infer_ok:
        tfm_block_tip = infer_reason or "TFM no listo"
    else:
        tfm_block_tip = ""
    tfm_selector_ready = bool(panel.tfm_module)
    tfm_selector_tip = "" if tfm_selector_ready else "Modelo no disponible"
    tfm_controls_tip = "" if tfm_action_ready else tfm_block_tip
    infer_btn_ready = tfm_action_ready and infer_ok
    infer_btn_tip = (infer_reason or "Cámara no lista") if (tfm_action_ready and not infer_ok) else ("" if infer_btn_ready else tfm_block_tip)
    if hasattr(panel, "combo_tfm_experiment"):
        panel.combo_tfm_experiment.setEnabled(tfm_selector_ready)
    if hasattr(panel, "chk_tfm_repro_mode"):
        panel.chk_tfm_repro_mode.setEnabled(tfm_selector_ready)
    if hasattr(panel, "chk_tfm_raw_output"):
        panel.chk_tfm_raw_output.setEnabled(tfm_selector_ready)

    def _set(button, enabled, tooltip):
        if button is not None:
            panel._set_btn_state(button, enabled, tooltip)

    _set(btn_tfm_apply, tfm_selector_ready, tfm_selector_tip)
    _set(btn_tfm_memoria_case, tfm_selector_ready, tfm_selector_tip)
    _set(btn_tfm_infer, infer_btn_ready, infer_btn_tip)
    _set(btn_tfm_visualize, tfm_action_ready, "" if tfm_action_ready else tfm_controls_tip)
    _set(btn_tfm_publish, tfm_action_ready, "" if tfm_action_ready else tfm_controls_tip)
    _set(btn_tfm_reset, tfm_action_ready, tfm_controls_tip)


def _apply_ui_state_camera_and_release(
    panel: "ControlPanelV2",
    *,
    bridge_active: bool,
    gz_active: bool,
    system_error: bool,
) -> None:
    """F3-step11c: gating de cámara + botón Soltar objetos.

    Habilita camera_topic_combo + 5 botones cámara (refresh/connect/far_front/
    top/wrist) en función de bridge_active + script_motion_active +
    _allow_camera_while_script_motion. btn_calibrate siempre habilitado.
    btn_release_objects: lógica detach_inflight / objects_release_done /
    gz_active / system_error.
    """
    camera_enabled = bridge_active and (
        not panel._script_motion_active
        or bool(getattr(panel, "_allow_camera_while_script_motion", False))
    )
    panel.camera_topic_combo.setEnabled(camera_enabled)
    panel.btn_camera_refresh.setEnabled(camera_enabled)
    panel.btn_camera_connect.setEnabled(camera_enabled)
    if getattr(panel, "btn_camera_far_front", None) is not None:
        panel.btn_camera_far_front.setEnabled(camera_enabled)
    if getattr(panel, "btn_camera_top", None) is not None:
        panel.btn_camera_top.setEnabled(camera_enabled)
    if getattr(panel, "btn_camera_wrist", None) is not None:
        panel.btn_camera_wrist.setEnabled(camera_enabled)
    if getattr(panel, "btn_calibrate", None) is not None:
        panel._set_btn_state(panel.btn_calibrate, True, "")
    if getattr(panel, "btn_release_objects", None) is not None:
        if panel._detach_inflight:
            release_ok = False
            release_tip = "Soltando objetos…"
        elif panel._objects_release_done:
            release_ok = False
            release_tip = "Soltar objetos ya ejecutado en este arranque"
        else:
            release_ok = gz_active and not system_error
            release_tip = "" if release_ok else ("Gazebo no activo" if not gz_active else "Sistema en error")
        panel._set_btn_state(panel.btn_release_objects, release_ok, release_tip)


def _apply_ui_state_pick_buttons(
    panel: "ControlPanelV2",
    *,
    camera_gate_ok: bool,
    test_pending: bool,
):
    """F3-step11d: gating de btn_pick_demo + btn_pick_object.

    Calcula pick_ok/pick_reason via _moveit_control_status, demo_ready
    (controllers_ok + tf_ready_state + ee_frame_effective + not _pick_demo_
    executed + _selected_object), pick_object_ready (pick_ok + pose_info_ok +
    tf_ready_state + _selected_object). Aplica setEnabled+tooltip a los 2.
    Devuelve (pick_enabled, pick_reason) para que apply_ui_state los use
    en el cálculo de _pick_block_reason.
    """
    pick_ok, pick_reason = panel._moveit_control_status()
    pick_enabled = pick_ok and bool(panel._ee_frame_effective)
    pick_tip = "Requiere MoveIt, cámara y objetos estables"
    demo_ready = (
        panel._controllers_ok
        and panel._tf_ready_state
        and bool(panel._ee_frame_effective)
        and not panel._pick_demo_executed
        and bool(panel._selected_object)
    )
    demo_tip = "Demo (secuencia joints, sin MoveIt)"
    if panel._pick_demo_executed:
        panel._set_btn_state(panel.btn_pick_demo, False, "PICK DEMO ya ejecutado (una sola vez)")
    else:
        if not panel._selected_object:
            demo_block_tip = "Selecciona un objeto"
        else:
            demo_block_tip = "Demo directa: espera controladores/TF"
        panel._set_btn_state(
            panel.btn_pick_demo,
            demo_ready,
            demo_tip if demo_ready else demo_block_tip,
        )
    block_tip = "Ejecuta AUTO TUNE para habilitar"
    pick_object_ready = pick_ok and panel._pose_info_ok and panel._tf_ready_state and bool(panel._selected_object)
    if camera_gate_ok and not test_pending:
        panel._set_btn_state(
            panel.btn_pick_object,
            pick_object_ready,
            "PICK objeto seleccionado" if pick_object_ready else ("Selecciona un objeto" if not panel._selected_object else pick_tip),
        )
    else:
        panel._set_btn_state(panel.btn_pick_object, False, block_tip)
    return pick_enabled, pick_reason


def _apply_ui_state_nav_and_finalize(
    panel: "ControlPanelV2",
    *,
    btn_test_robot,
    btn_tfm_infer,
    btn_tfm_visualize,
    btn_tfm_publish,
    pick_enabled: bool,
    pick_reason: str,
    motion_enabled: bool,
    ready_basic: bool,
    camera_ready: bool,
    gz_active: bool,
    system_error: bool,
) -> None:
    """F3-step11e: nav buttons + busy gating + obj/camera_view + status final.

    Aplica gating a btn_home/table/basket; si _script_motion_active, fuerza
    OFF a btn_test_robot/gripper/pick_demo/pick_object/tfm_*.
    Habilita obj_panel + camera_view. Calcula _pick_block_reason en función
    de ready_basic/motion_enabled/controllers_ok/pick_enabled. Bloquea
    world_combo/mode_combo/btn_world_browse durante gz_active. Llama a
    _update_panel_flow_state y dispara _state_event.set().
    """
    def _set_optional(button, enabled, tooltip):
        if button is not None:
            panel._set_btn_state(button, enabled, tooltip)

    nav_enabled = not panel._script_motion_active
    nav_tip = "Robot en movimiento" if panel._script_motion_active else ""
    panel._set_btn_state(panel.btn_home, nav_enabled, nav_tip)
    panel._set_btn_state(panel.btn_table, nav_enabled, nav_tip)
    panel._set_btn_state(panel.btn_basket, nav_enabled, nav_tip)
    if panel._script_motion_active:
        busy_tip = "Robot en movimiento"
        _set_optional(btn_test_robot, False, busy_tip)
        if not bool(getattr(panel, "_allow_gripper_while_script_motion", False)):
            panel._set_btn_state(panel.btn_gripper, False, busy_tip)
        panel._set_btn_state(panel.btn_pick_demo, False, busy_tip)
        panel._set_btn_state(panel.btn_pick_object, False, busy_tip)
        if panel.tfm_module:
            _set_optional(btn_tfm_infer, False, busy_tip)
            _set_optional(btn_tfm_visualize, False, busy_tip)
            _set_optional(btn_tfm_publish, False, busy_tip)
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
        pick_enabled=pick_enabled,
        system_error=system_error,
    )
    panel._state_event.set()


def apply_ui_state(panel: "ControlPanelV2", effective_state: SystemState, effective_reason: str) -> None:
    btn_test_robot = getattr(panel, "btn_test_robot", None)
    btn_tfm_apply = getattr(panel, "btn_tfm_apply", None)
    btn_tfm_memoria_case = getattr(panel, "btn_tfm_memoria_case", None)
    btn_tfm_infer = getattr(panel, "btn_tfm_infer", None)
    btn_tfm_visualize = getattr(panel, "btn_tfm_visualize", None)
    btn_tfm_publish = getattr(panel, "btn_tfm_publish", None)
    btn_tfm_reset = getattr(panel, "btn_tfm_reset", None)

    def _set_test_robot_btn_state(enabled: bool, tooltip: str) -> None:
        if btn_test_robot is not None:
            panel._set_btn_state(btn_test_robot, enabled, tooltip)

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
    _apply_ui_state_launchers(
        panel,
        gz_active=gz_active,
        gz_proc_alive=gz_proc_alive,
        bridge_active=bridge_active,
        bridge_proc_alive=bridge_proc_alive,
        bag_proc_alive=bag_proc_alive,
        moveit_proc_alive=moveit_proc_alive,
        moveit_bridge_proc_alive=moveit_bridge_proc_alive,
        ready_basic=ready_basic,
    )

    _apply_ui_state_camera_and_release(
        panel,
        bridge_active=bridge_active,
        gz_active=gz_active,
        system_error=system_error,
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
    manual_force_enabled = bool(getattr(panel, "_manual_controls_always_enabled", False))
    if manual_force_enabled:
        # Mantener disponibles los mandos manuales de joints incluso durante gates
        # de cámara/test/script; solo respetamos la seguridad base de manual_ok.
        manual_enabled = manual_ok
        basic_reason = manual_reason or (effective_reason or effective_state.value)
    else:
        manual_enabled = manual_ok and not panel._script_motion_active and camera_gate_ok and not test_pending
        basic_reason = manual_reason or (effective_reason or effective_state.value)
        if not camera_gate_ok:
            basic_reason = "Cámara no lista"
        elif not camera_ready and camera_degraded_ok:
            basic_reason = "Cámara sin frames (modo degradado)"
        elif test_pending:
            basic_reason = "Ejecuta AUTO TUNE para habilitar"
    panel._set_btn_state(
        panel.btn_send_joints,
        manual_enabled,
        f"Bloqueado: {basic_reason}" if not manual_enabled else "Mover articulaciones",
    )
    panel.joint_time.setEnabled(manual_enabled)
    panel.chk_auto_joints.setEnabled(manual_enabled)
    for slider in panel.joint_sliders:
        slider.setEnabled(manual_enabled)
    for btn_minus, btn_plus in getattr(panel, "joint_step_buttons", []):
        btn_minus.setEnabled(manual_enabled)
        btn_plus.setEnabled(manual_enabled)

    motion_enabled = manual_ok and not panel._script_motion_active
    gripper_motion_enabled = manual_ok and (
        not panel._script_motion_active
        or bool(getattr(panel, "_allow_gripper_while_script_motion", False))
    )
    motion_tip = "" if motion_enabled else f"Bloqueado: {basic_reason}"
    gripper_motion_tip = "" if gripper_motion_enabled else f"Bloqueado: {basic_reason}"
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
    test_locked_tip = "AUTO TUNE ya ejecutado"

    if external_block and not moveit_only:
        block_tip = f"Bloqueado: {external_tip}"
        _set_test_robot_btn_state(False, test_locked_tip if test_locked else block_tip)
        panel._set_btn_state(panel.btn_gripper, False, block_tip)
    elif moveit_only:
        panel._set_robot_test_blocked(None)
        moveit_tip = "MoveIt bridge activo; se pausará antes de AUTO TUNE"
        _set_test_robot_btn_state(False if test_locked else motion_enabled, test_locked_tip if test_locked else moveit_tip)
        panel._set_btn_state(panel.btn_gripper, gripper_motion_enabled, gripper_motion_tip)
    elif not camera_gate_ok:
        block_tip = "Cámara no lista"
        if test_locked:
            _set_test_robot_btn_state(False, test_locked_tip)
        else:
            _set_test_robot_btn_state(False, block_tip)
        panel._set_btn_state(panel.btn_gripper, gripper_motion_enabled, gripper_motion_tip)
    elif test_pending:
        block_tip = "Ejecuta AUTO TUNE para habilitar"
        if test_locked:
            _set_test_robot_btn_state(False, test_locked_tip)
        else:
            _set_test_robot_btn_state(motion_enabled, motion_tip)
        panel._set_btn_state(panel.btn_gripper, gripper_motion_enabled, gripper_motion_tip)
    else:
        if test_locked:
            _set_test_robot_btn_state(False, test_locked_tip)
        else:
            _set_test_robot_btn_state(motion_enabled, motion_tip)
        panel._set_btn_state(panel.btn_gripper, gripper_motion_enabled, gripper_motion_tip)
    panel._schedule_controller_check()
    pick_enabled, pick_reason = _apply_ui_state_pick_buttons(
        panel,
        camera_gate_ok=camera_gate_ok,
        test_pending=test_pending,
    )
    _apply_ui_state_tfm_buttons(
        panel,
        btn_tfm_apply=btn_tfm_apply,
        btn_tfm_memoria_case=btn_tfm_memoria_case,
        btn_tfm_infer=btn_tfm_infer,
        btn_tfm_visualize=btn_tfm_visualize,
        btn_tfm_publish=btn_tfm_publish,
        btn_tfm_reset=btn_tfm_reset,
    )
    _apply_ui_state_nav_and_finalize(
        panel,
        btn_test_robot=btn_test_robot,
        btn_tfm_infer=btn_tfm_infer,
        btn_tfm_visualize=btn_tfm_visualize,
        btn_tfm_publish=btn_tfm_publish,
        pick_enabled=pick_enabled,
        pick_reason=pick_reason,
        motion_enabled=motion_enabled,
        ready_basic=ready_basic,
        camera_ready=camera_ready,
        gz_active=gz_active,
        system_error=system_error,
    )
