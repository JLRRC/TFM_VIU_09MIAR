#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_launchers.py
# Contenido: Codigo del panel Qt y de la logica ROS 2 asociada al UR5.
# Uso breve: Se usa en build con colcon y en ejecucion mediante el entry point panel_v2.
"""Launch/stop helpers for Gazebo/Bridge/MoveIt."""
from __future__ import annotations

import os
import re
import shlex
import shutil
import subprocess
import time

from PyQt5.QtCore import QTimer

from .panel_config import (
    GZ_PARTITION_FILE,
    GZ_WORLD,
    LOG_DIR,
    MODELS_DIR,
    GZ_LAUNCH_TIMEOUT_SEC,
    BRIDGE_LAUNCH_TIMEOUT_SEC,
    TF_INIT_GRACE_SEC,
    UR5_CONTROLLERS_YAML,
    UR5_MODEL_NAME,
    WORLDS_DIR,
)
from .panel_utils import (
    bash_preamble,
    build_gz_env,
    effective_base_frame,
    effective_world_frame,
    ensure_dir,
    log_to_file,
    read_world_name,
    resolve_gz_partition,
    rotate_log,
    set_led,
    with_line_buffer,
)
from .logging_utils import timestamped_line

_DEBUG_EXCEPTIONS = os.environ.get("PANEL_DEBUG_EXCEPTIONS", "").strip() in ("1", "true", "True")


def _log_exception(context: str, exc: Exception) -> None:
    if not _DEBUG_EXCEPTIONS:
        return
    print(timestamped_line(f"[LAUNCHERS][WARN] {context}: {exc}"), flush=True)


def _env_flag(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return bool(default)
    val = str(raw).strip().lower()
    if val in ("1", "true", "yes", "on"):
        return True
    if val in ("0", "false", "no", "off"):
        return False
    return bool(default)


def _env_float_opt(name: str) -> float | None:
    raw = os.environ.get(name)
    if raw is None or str(raw).strip() == "":
        return None
    try:
        return float(raw)
    except Exception:
        return None

def start_world_tf_publisher(panel, world_name: str) -> None:
    if panel.world_tf_proc is not None and panel.world_tf_proc.poll() is None:
        return
    try:
        ensure_dir(LOG_DIR)
        tf_log = os.path.join(LOG_DIR, "world_tf_publisher.log")
        rotate_log(tf_log)
        use_sim_time = os.environ.get("USE_SIM_TIME", "1") == "1"
        sim_arg = " -p use_sim_time:=true" if use_sim_time else ""
        world_file = os.path.join(WORLDS_DIR, f"{world_name}.sdf")
        world_file_arg = ""
        if os.path.isfile(world_file):
            world_file_arg = f" -p world_file:={shlex.quote(world_file)}"
        clock_timeout = max(10.0, float(TF_INIT_GRACE_SEC))
        pose_timeout = max(10.0, float(TF_INIT_GRACE_SEC))
        static_grace = min(5.0, float(TF_INIT_GRACE_SEC))
        base_frame = effective_base_frame(panel)
        # Keep world TF anchored to base_link. Some sessions resolve a transient
        # "base" frame here, which later causes base/base_link inconsistencies.
        if str(base_frame or "").strip() in ("", "base", "/base"):
            base_frame = "base_link"
        world_frame = effective_world_frame(panel)
        cmd_core = with_line_buffer(
            "ros2 run ur5_tools world_tf_publisher "
            f"--ros-args -p world_name:={shlex.quote(world_name)} "
            f"-p model_name:={shlex.quote(UR5_MODEL_NAME)} "
            f"-p base_frame:={shlex.quote(base_frame)} "
            f"-p world_frame:={shlex.quote(world_frame)}{sim_arg}"
            f"{world_file_arg}"
            f" -p clock_timeout_sec:={clock_timeout:.1f}"
            f" -p pose_timeout_sec:={pose_timeout:.1f}"
            f" -p static_grace_sec:={static_grace:.1f}"
        )
        cmd = bash_preamble(panel.ws_dir) + f"{cmd_core} > '{tf_log}' 2>&1"
        panel.world_tf_proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            preexec_fn=os.setsid,
        )
        panel._started_world_tf = True
        panel._emit_log("[TF] world_tf_publisher lanzado")
    except Exception as exc:
        panel._log_error(f"[TF] Error iniciando world_tf_publisher: {exc}")

def start_gz_pose_bridge(panel, world_name: str) -> None:
    if panel.gz_pose_proc is not None and panel.gz_pose_proc.poll() is None:
        return
    try:
        ensure_dir(LOG_DIR)
        pose_log = os.path.join(LOG_DIR, "gz_pose_bridge.log")
        rotate_log(pose_log)
        use_sim_time = os.environ.get("USE_SIM_TIME", "1") == "1"
        sim_arg = " -p use_sim_time:=true" if use_sim_time else ""
        cmd_core = with_line_buffer(
            "ros2 run ur5_tools gz_pose_bridge "
            f"--ros-args -p world_name:={shlex.quote(world_name)}{sim_arg}"
        )
        cmd = bash_preamble(panel.ws_dir) + f"{cmd_core} > '{pose_log}' 2>&1"
        panel.gz_pose_proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            preexec_fn=os.setsid,
        )
        panel._emit_log("[BRIDGE] gz_pose_bridge lanzado")
    except Exception as exc:
        panel._log_error(f"[BRIDGE] Error iniciando gz_pose_bridge: {exc}")
def stop_world_tf_publisher(panel) -> None:
    panel._kill_proc(panel.world_tf_proc, "world_tf_publisher")
    panel.world_tf_proc = None
    panel._started_world_tf = False

def start_gazebo(panel):
    if panel._block_if_managed("Start Gazebo"):
        return
    if panel._gz_launching:
        return
    if not panel._ros_worker_started:
        panel._ensure_ros_worker_started()
    gz_state = panel._gazebo_state()
    if gz_state != "GAZEBO_OFF":
        panel._log_error(f"Gazebo ya activo ({gz_state})")
        panel._set_status(f"Gazebo ya activo ({gz_state})", error=True)
        set_led(panel.led_gz, "on" if gz_state == "GAZEBO_READY" else "warn")
        panel.signal_refresh_controls.emit()
        return
    panel._log_button("Start Gazebo")
    world = panel.world_combo.currentText().strip()
    panel._log(f"[GZ] Mundo: {world}")
    if not world:
        panel._log_error("Mundo no seleccionado")
        panel._set_status("Selecciona mundo para Gazebo", error=True)
        return
    if not os.path.isfile(world):
        panel._log_error(f"Archivo mundo no existe: {world}")
        panel._set_status("Mundo no existe", error=True)
        return
    panel._set_status("Lanzando Gazebo…")
    panel._gz_launching = True
    panel._gz_launch_start = time.time()
    panel._critical_clock_deadline = time.monotonic() + max(0.1, GZ_LAUNCH_TIMEOUT_SEC)
    panel._set_launching_style(panel.btn_gz_start, True)
    panel.btn_gz_start.setEnabled(False)
    panel._start_robot_state_publisher()
    panel._drop_hold_enabled = False
    panel.gz_partition = f"ur5pro_{int(time.time())}"
    os.environ["GZ_PARTITION"] = panel.gz_partition
    try:
        with open(GZ_PARTITION_FILE, "w", encoding="utf-8") as f:
            f.write(panel.gz_partition)
    except Exception as exc:
        _log_exception("write GZ_PARTITION_FILE", exc)

    def worker():
        ensure_dir(LOG_DIR)
        gz_log = os.path.join(LOG_DIR, "gz_server.log")
        rotate_log(gz_log)
        runtime_models_root = os.path.join(LOG_DIR, "gz_models")
        runtime_ur5_model = os.path.join(runtime_models_root, "ur5_rg2")
        controllers_yaml = UR5_CONTROLLERS_YAML
        if not os.path.isfile(controllers_yaml):
            panel._ui_set_status("No se encontró ur5_controllers.yaml", error=True)
            panel.signal_set_led.emit(panel.led_gz, "error")
            panel._gz_launching = False
            panel._gz_launch_start = 0.0
            panel.signal_refresh_controls.emit()
            return
        try:
            ensure_dir(runtime_ur5_model)
            shutil.copytree(os.path.join(MODELS_DIR, "ur5_rg2"), runtime_ur5_model, dirs_exist_ok=True)
            model_sdf = os.path.join(runtime_ur5_model, "model.sdf")
            if os.path.isfile(model_sdf):
                with open(model_sdf, "r", encoding="utf-8") as f:
                    sdf_text = f.read()
                sdf_text = sdf_text.replace("$(env UR5_CONTROLLERS_YAML)", controllers_yaml)
                sdf_text = re.sub(
                    r"<parameters>\\s*--params-file\\s+[^<]+</parameters>",
                    f"<parameters>{controllers_yaml}</parameters>",
                    sdf_text,
                    flags=re.DOTALL,
                )
                with open(model_sdf, "w", encoding="utf-8") as f:
                    f.write(sdf_text)
        except Exception as exc:
            _log_exception("prepare runtime model", exc)
        render_engine = os.environ.get("GZ_RENDER_ENGINE", "").strip() or "ogre2"
        env = (
            build_gz_env(panel.gz_partition)
            + f"export GZ_SIM_RESOURCE_PATH='{runtime_models_root}:{MODELS_DIR}:{WORLDS_DIR}:${{GZ_SIM_RESOURCE_PATH:-}}' ; "
            "export GZ_LOG_LEVEL=error; export IGN_LOGGER_LEVEL=error; export QT_LOGGING_RULES='qt.qml.*=false'; "
        )
        mode = panel._effective_mode()
        runtime_world = world
        try:
            if os.path.isfile(world):
                with open(world, "r", encoding="utf-8") as f:
                    world_text = f.read()
                keep_cameras = os.environ.get("PANEL_KEEP_CAMERAS", "").strip() in ("1", "true", "True")
                if not keep_cameras:
                    keep_cameras = os.environ.get("PANEL_CAMERA_REQUIRED", "").strip() in ("1", "true", "True")
                if mode != "gui" and not keep_cameras:
                    world_text = re.sub(
                        r"<plugin\s+filename=['\"]gz-sim-sensors-system['\"][\s\S]*?</plugin>",
                        "",
                        world_text,
                        flags=re.DOTALL,
                    )
                    for cam_name in (
                        "camera_overhead",
                        "camera_north",
                        "camera_south",
                        "camera_east",
                        "camera_west",
                    ):
                        pattern = rf"<model\s+name=['\"]{cam_name}['\"]>.*?</model>"
                        world_text = re.sub(pattern, "", world_text, flags=re.DOTALL)
                world_text = world_text.replace(
                    "<uri>model://ur5_rg2</uri>",
                    f"<uri>file://{runtime_ur5_model}</uri>",
                )
                runtime_world = os.path.join(LOG_DIR, "world_runtime.sdf")
                with open(runtime_world, "w", encoding="utf-8") as f:
                    f.write(world_text)
        except Exception as exc:
            _log_exception("prepare runtime world", exc)
            runtime_world = world
        if mode == "gui":
            cmd_core = with_line_buffer(f"gz sim -r -v 1 {shlex.quote(runtime_world)}")
            cmd = bash_preamble(panel.ws_dir) + env + log_to_file(cmd_core, gz_log, None)
        else:
            cmd_core = with_line_buffer(
                "gz sim -s -r --headless-rendering --render-engine ogre2 -v 1 "
                + shlex.quote(runtime_world)
            )
            cmd = (
                bash_preamble(panel.ws_dir)
                + env
                + f"env -u DISPLAY GZ_RENDER_ENGINE={render_engine} "
                + log_to_file(cmd_core, gz_log, None)
            )
        try:
            panel.gz_proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                preexec_fn=os.setsid,
            )
            panel._gz_root_pid = int(getattr(panel.gz_proc, "pid", 0) or 0)
            panel._gz_real_pid = panel._gz_root_pid
            try:
                panel._gz_pgid = int(os.getpgid(panel._gz_root_pid))
            except Exception:
                panel._gz_pgid = 0
            def monitor():
                rc = panel.gz_proc.wait()
                panel._emit_log(f"[GZ] gz sim exited rc={rc} (log: {gz_log})")
                try:
                    tail = subprocess.run(
                        ["bash", "-lc", f"tail -n 80 {shlex.quote(gz_log)}"],
                        text=True,
                        capture_output=True,
                        timeout=2.0,
                    )
                    if tail.stdout:
                        for line in tail.stdout.splitlines():
                            panel._emit_log(f"[GZ][LOG] {line}")
                except Exception as exc:
                    _log_exception("tail gz log", exc)
            panel._run_async(monitor)
            panel._started_gazebo = True
            panel._gz_running = True
            panel._gz_world_name = read_world_name(world) or GZ_WORLD
            panel._log("[GZ] Gazebo lanzado")
            panel._invalidate_settle("gazebo start", restart=True)
            panel._pose_info_ok = False
            panel._ensure_pose_subscription()
            panel._ui_set_status("Gazebo lanzado")
            panel.signal_set_led.emit(panel.led_gz, "on")
            panel.signal_request_auto_bridge_start.emit()
            panel._gz_launching = False
            panel._gz_launch_start = 0.0
            panel.signal_refresh_controls.emit()
        except Exception as exc:
            panel._ui_set_status(f"Error lanzando Gazebo: {exc}", error=True)
            panel.signal_set_led.emit(panel.led_gz, "error")
            panel._gz_launching = False
            panel._gz_launch_start = 0.0
            panel.signal_refresh_controls.emit()

    panel._run_async(worker)
    # Programar ajuste automático de joint2 tras bridge (no en arranque de Gazebo)
    panel._auto_joint2_move_done = False



def stop_gazebo(panel):
    if panel._block_if_managed("Stop Gazebo"):
        return
    panel._log_button("Stop Gazebo")
    panel._set_status("Deteniendo Gazebo…")
    panel._stop_debug_poses()
    panel._kill_proc(panel.gz_proc, "gz sim")
    panel.gz_proc = None
    panel._kill_proc(panel.rsp_proc, "robot_state_publisher")
    panel.rsp_proc = None
    panel._stop_world_tf_publisher()
    panel._kill_proc(panel.release_service_proc, "release_objects_service")
    panel.release_service_proc = None
    panel._objects_settled = False
    panel._objects_seen_fall = False
    panel._objects_release_done = False
    panel._drop_hold_enabled = False
    panel._trace_ready = False
    panel._tf_ready_state = False
    panel._controllers_ok = False
    panel._controllers_reason = "gazebo detenido"
    panel._detach_inflight = False
    panel._detach_attempted = False
    panel._detach_auto_disabled = False
    panel._detach_backoff_until = 0.0
    panel._pose_info_ok = False
    panel._gz_running = False
    panel._gz_root_pid = 0
    panel._gz_real_pid = 0
    panel._gz_pgid = 0
    set_led(panel.led_gz, "off")

def start_robot_state_publisher(panel):
    if panel.rsp_proc is not None and panel.rsp_proc.poll() is None:
        return
    try:
        ensure_dir(LOG_DIR)
        rsp_log = os.path.join(LOG_DIR, "ur5_rsp.log")
        rotate_log(rsp_log)
        env = f"export ROS_LOG_DIR='{LOG_DIR}/ros' ; "
        cmd_core = with_line_buffer("ros2 launch ur5_bringup ur5_rsp.launch.py use_sim_time:=true")
        cmd = bash_preamble(panel.ws_dir) + env + f"{cmd_core} > '{rsp_log}' 2>&1"
        panel.rsp_proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            preexec_fn=os.setsid,
        )
        panel._started_rsp = True
        panel._emit_log("[TF] robot_state_publisher lanzado")
    except Exception as exc:
        panel._log_error(f"Error lanzando robot_state_publisher: {exc}")

def start_bridge(panel):
    if panel._block_if_managed("Start bridge"):
        return
    if panel._bridge_launching:
        return
    if not panel._ros_worker_started:
        panel._ensure_ros_worker_started()
    world_name = read_world_name(panel.world_combo.currentText().strip()) or GZ_WORLD
    pose_topic = f"/world/{world_name}/pose/info"
    if panel.ros_worker.node_ready() and panel.ros_worker.topic_has_publishers(pose_topic):
        panel._log_warning("Bridge ya activo (pose/info con publishers)")
        panel._set_status("Bridge ya activo (pose/info detectado)", error=False)
        panel._bridge_running = True
        set_led(panel.led_bridge, "on")
        # Inicializar suscripciones y cámara sin activar deadlines de TF
        panel._ensure_pose_subscription()
        panel._ensure_grasp_rect_subscription()
        panel._start_pose_info_watch()
        panel._start_tf_ready_timer()
        panel._auto_subscribe_joints()
        from PyQt5.QtCore import QTimer
        QTimer.singleShot(300, panel._refresh_camera_topics)
        QTimer.singleShot(600, panel._auto_connect_camera)
        panel._check_camera_topic_health()
        panel.signal_calibration_check.emit()
        panel.signal_refresh_controls.emit()
        return
    panel._log_button("Start bridge")
    panel._camera_stream_ok = False
    panel._calibration_ready = False
    panel._pose_info_ok = False
    panel._pose_info_msg_count = 0
    panel._pose_info_last_age = float("inf")
    panel._pose_info_last_log = 0.0
    panel._tf_no_msgs_logged = False
    # Permitir lanzar bridge en cuanto Gazebo esté realmente arriba (aunque el flag interno tarde en activarse)
    gz_state = panel._gazebo_state()
    if gz_state == "GAZEBO_OFF":
        panel._log_warning("Bridge en espera: Gazebo no activo")
        panel._set_status("Bridge en espera: Gazebo no activo", error=False)
        set_led(panel.led_bridge, "warn")
        return
    panel._ensure_ros_worker_started()
    # Actualizar flag interno si lo detectamos corriendo
    if gz_state != "GAZEBO_OFF" and not panel._gz_running:
        panel._gz_running = True
        set_led(panel.led_gz, "on")
        panel._refresh_controls()
    base_yaml = panel.bridge_edit.text().strip()
    panel._log(f"[BRIDGE] YAML base: {base_yaml}")
    if not base_yaml or not os.path.isfile(base_yaml):
        panel._log_error(f"YAML no encontrado: {base_yaml}")
        panel._set_status("YAML no encontrado", error=True)
        set_led(panel.led_bridge, "error")
        return
    panel._start_robot_state_publisher()
    panel._trace_ready = False
    panel._reset_trace_throttle("bridge start")
    panel._bridge_ready = False
    panel._set_status("Lanzando bridge…")
    panel._bridge_launching = True
    panel._bridge_launch_start = time.time()
    panel._critical_pose_deadline = time.monotonic() + max(0.1, BRIDGE_LAUNCH_TIMEOUT_SEC + 5.0)
    panel._critical_tf_deadline = 0.0
    panel._set_launching_style(panel.btn_bridge_start, True)
    panel.btn_bridge_start.setEnabled(False)
    # Deshabilitar botón mientras arranca para que se vea en gris como Gazebo
    panel._bridge_running = True
    panel._started_bridge = True
    panel._bridge_start_ts = time.time()
    panel._refresh_controls()

    def worker():
        ensure_dir(LOG_DIR)
        runtime_yaml = base_yaml
        world_name = read_world_name(panel.world_combo.currentText().strip()) or GZ_WORLD
        try:
            with open(base_yaml, "r", encoding="utf-8") as f:
                yaml_text = f.read()
            yaml_text = yaml_text.replace(
                "/world/ur5_mesa_objetos/",
                f"/world/{world_name}/",
            )
            runtime_yaml = os.path.join(LOG_DIR, "bridge_runtime.yaml")
            with open(runtime_yaml, "w", encoding="utf-8") as f:
                f.write(yaml_text)
        except Exception:
            runtime_yaml = base_yaml
        br_log = os.path.join(LOG_DIR, "ros_gz_bridge.log")
        rotate_log(br_log)
        env = (
            build_gz_env(resolve_gz_partition(panel.gz_partition))
            + f"export ROS_LOG_DIR='{LOG_DIR}/ros' ; "
        )
        cmd_core = with_line_buffer(
            "ros2 run ros_gz_bridge parameter_bridge --ros-args "
            + f"-p config_file:='{runtime_yaml}' -p lazy:=true"
        )
        cmd = bash_preamble(panel.ws_dir) + env + f"{cmd_core} > '{br_log}' 2>&1"
        try:
            panel.bridge_proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                preexec_fn=os.setsid,
            )
            panel._ui_set_status("Bridge lanzado")
            if not panel._debug_logs_enabled:
                panel._emit_log("[INFO] Bridge lanzado")
            panel.signal_set_led.emit(panel.led_bridge, "on")
            panel._bridge_running = True
            panel._bridge_launching = False
            panel._bridge_launch_start = 0.0
            panel.signal_refresh_controls.emit()
            panel._start_world_tf_publisher(world_name)
            start_gz_pose_bridge(panel, world_name)
            panel._spawn_controllers_async()
            panel.signal_bridge_ready.emit()
        except Exception as exc:
            panel._ui_set_status(f"Error lanzando bridge: {exc}", error=True)
            panel.signal_set_led.emit(panel.led_bridge, "error")
            panel._bridge_running = False
            panel._bridge_launching = False
            panel._bridge_launch_start = 0.0
            panel.signal_refresh_controls.emit()

    panel._run_async(worker)

def stop_bridge(panel):
    if panel._block_if_managed("Stop bridge"):
        return
    panel._log_button("Stop bridge")
    panel._camera_stream_ok = False
    panel._camera_topic_hz = 0.0
    panel._set_status("Deteniendo bridge…")
    panel._kill_proc(panel.bridge_proc, "parameter_bridge")
    panel._kill_proc(panel.rsp_proc, "robot_state_publisher")
    panel._kill_proc(panel.gz_pose_proc, "gz_pose_bridge")
    panel._stop_world_tf_publisher()
    panel.rsp_proc = None
    panel._kill_proc(panel.release_service_proc, "release_objects_service")
    panel._trace_ready = False
    panel._tf_ready_state = False
    panel._reset_trace_throttle("bridge stop")
    if panel._tf_ready_timer:
        panel._tf_ready_timer.stop()
    if panel._pose_info_timer:
        panel._pose_info_timer.stop()
    panel.bridge_proc = None
    panel.gz_pose_proc = None
    panel.release_service_proc = None
    panel._bridge_running = False
    panel._bridge_start_ts = 0.0
    panel._critical_camera_deadline = 0.0
    panel._critical_pose_deadline = 0.0
    panel._critical_tf_deadline = 0.0
    set_led(panel.led_bridge, "off")
    panel._refresh_controls()
    panel._bridge_ready = False
    panel._controllers_ok = False
    panel._controllers_reason = "bridge detenido"
    panel._controller_spawn_inflight = False
    panel._controller_spawn_done = False
    panel._objects_release_done = False
    panel._pose_info_ok = False

def start_moveit(panel):
    if panel._block_if_managed("Start MoveIt"):
        return
    if panel._moveit_launching:
        return
    if not panel._controllers_ok:
        ok, reason = panel._controllers_ready()
        panel._controllers_ok = ok
        panel._controllers_reason = reason
    if not panel._controllers_ok:
        reason = panel._controllers_reason or "controladores no listos"
        panel._log_error(f"[MOVEIT] Abortado: {reason}")
        panel._set_status(f"MoveIt bloqueado: {reason}", error=True)
        panel.signal_moveit_state.emit("ERROR", reason)
        return
    joints_ok, joints_reason = panel._joint_states_status()
    if not joints_ok:
        panel._log_error(f"[MOVEIT] Abortado: /joint_states no listo ({joints_reason})")
        panel._set_status(f"MoveIt bloqueado: /joint_states no listo ({joints_reason})", error=True)
        panel.signal_moveit_state.emit("ERROR", f"joint_states: {joints_reason}")
        return
    tf_ok, tf_reason = panel._tf_chain_ready_status()
    if not tf_ok:
        panel._log_error(f"[MOVEIT] Abortado: TF no listo ({tf_reason})")
        panel._set_status(f"MoveIt bloqueado: TF no listo ({tf_reason})", error=True)
        panel.signal_moveit_state.emit("ERROR", f"tf: {tf_reason}")
        return
    if not panel._ros_worker_started:
        panel._ensure_ros_worker_started()
    if panel._move_group_ready():
        panel.signal_moveit_state.emit("READY", "move_group ya activo")
        panel._set_status("MoveIt ya activo")
        return
    if panel.moveit_proc is not None and panel.moveit_proc.poll() is None:
        panel.signal_moveit_state.emit("WAITING_MOVEIT_READY", "move_group arrancado; esperando disponibilidad")
        panel._run_async(panel._wait_for_moveit_ready)
        return
    panel._log_button("Start MoveIt")
    panel._set_status("Lanzando MoveIt…")
    panel.signal_moveit_state.emit("STARTING", "launching move_group")
    panel._moveit_launching = True
    panel._moveit_launch_start = time.time()
    panel._set_launching_style(panel.btn_moveit_start, True)
    panel.btn_moveit_start.setEnabled(False)
    try:
        ensure_dir(LOG_DIR)
        moveit_log = os.path.join(LOG_DIR, "moveit_bringup.log")
        rotate_log(moveit_log)
        env = f"export ROS_LOG_DIR='{LOG_DIR}/ros' ; "
        use_sim_time = os.environ.get("USE_SIM_TIME", "1") == "1"
        sim_arg = " use_sim_time:=true" if use_sim_time else " use_sim_time:=false"
        cmd_core = with_line_buffer(
            "ros2 launch ur5_moveit_config ur5_moveit_bringup.launch.py "
            f"start_ros2_control:=false launch_rviz:=false{sim_arg}"
        )
        cmd = bash_preamble(panel.ws_dir) + env + f"{cmd_core} > '{moveit_log}' 2>&1"
        panel.moveit_proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            preexec_fn=os.setsid,
        )
        panel._started_moveit = True
        panel._moveit_running = True
        panel._set_status("MoveIt lanzado")
        panel._refresh_controls()
        panel.signal_moveit_state.emit("WAITING_MOVEIT_READY", "esperando move_group")
        panel._run_async(panel._wait_for_moveit_ready)
    except Exception as exc:
        panel._set_status(f"Error lanzando MoveIt: {exc}", error=True)
        panel.signal_moveit_state.emit("ERROR", f"launch failed: {exc}")
        panel._moveit_running = False
        panel._moveit_launching = False
        panel._moveit_launch_start = 0.0

def stop_moveit(panel):
    if panel._block_if_managed("Stop MoveIt"):
        return
    panel._log_button("Stop MoveIt")
    panel._set_status("Deteniendo MoveIt…")
    panel._kill_proc(panel.moveit_proc, "move_group")
    panel.moveit_proc = None
    panel._moveit_running = False
    panel.signal_moveit_state.emit("OFF", "manual")
    panel._refresh_controls()

def start_moveit_bridge(panel):
    if panel._block_if_managed("Start MoveIt bridge"):
        return
    if panel.moveit_bridge_proc is not None and panel.moveit_bridge_proc.poll() is None:
        return
    if not panel._ros_worker_started:
        panel._ensure_ros_worker_started()
    if panel._moveit_bridge_detected():
        bridge_path_ready = False
        pose_subs = 0
        result_pubs = 0
        if panel._ros_worker_started and panel.ros_worker.node_ready():
            pose_subs = panel.ros_worker.topic_subscriber_count("/desired_grasp")
            result_pubs = panel.ros_worker.topic_publisher_count("/desired_grasp/result")
            bridge_path_ready = pose_subs > 0 and result_pubs > 0
        if bridge_path_ready:
            panel._set_status("MoveIt bridge ya activo", error=True)
            panel._moveit_bridge_running = True
            set_led(panel.led_moveit_bridge, "on")
            panel._refresh_controls()
            return
        panel._emit_log(
            "[MOVEIT][BRIDGE] deteccion inconsistente: "
            f"bridge_detected=true pero pose_subs={pose_subs} result_pubs={result_pubs}; "
            "forzando relanzado"
        )
        panel._moveit_bridge_detected_cache = False
        panel._moveit_bridge_detected_ts = 0.0
    panel._log_button("Start MoveIt bridge")
    panel._emit_log(
        "[MOVEIT][BRIDGE] launch_request "
        "node=ur5_moveit_bridge pose_topic=/desired_grasp result_topic=/desired_grasp/result"
    )
    state_name = getattr(panel._moveit_state, "name", panel._moveit_state)
    if str(state_name) != "READY":
        panel._log_warning("MoveIt no está activo; el bridge puede fallar")
    panel._moveit_bridge_launching = True
    panel._moveit_bridge_launch_start = time.time()
    panel._set_launching_style(panel.btn_moveit_bridge_start, True)
    panel.btn_moveit_bridge_start.setEnabled(False)
    try:
        ensure_dir(LOG_DIR)
        bridge_log = os.path.join(LOG_DIR, "moveit_bridge.log")
        rotate_log(bridge_log)
        env = f"export ROS_LOG_DIR='{LOG_DIR}/ros' ; "
        ros_args = [
            "--ros-args",
            "-p",
            "backend:=auto",
            "-p",
            "base_frame:=base_link",
            "-p",
            "ee_frame:=rg2_tcp",
            "-p",
            "result_topic:=/desired_grasp/result",
        ]
        try:
            cm_path = panel._controller_manager_path()
        except Exception:
            cm_path = "/controller_manager"
        ros_args.extend(["-p", f"controller_manager:={cm_path}"])
        use_sim_time = os.environ.get("USE_SIM_TIME", "1") == "1"
        bridge_moveit_py_sim_time_env = str(
            os.environ.get(
                "PANEL_MOVEIT_BRIDGE_SIM_TIME",
                "0",
            )
        ).strip().lower() in ("1", "true", "yes", "on")
        # Default false: MoveItPy on sim-time can abort on qos_overrides./clock in Jazzy.
        bridge_moveit_py_sim_time = bool(bridge_moveit_py_sim_time_env)
        ros_args.extend(["-p", f"use_sim_time:={'true' if use_sim_time else 'false'}"])
        ros_args.extend(
            [
                "-p",
                "moveit_py_use_sim_time:="
                + ("true" if bridge_moveit_py_sim_time else "false"),
            ]
        )
        # Runtime tuning knobs (no recompilar): set via env before launching bridge.
        exec_timeout = _env_float_opt("PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC")
        req_timeout = _env_float_opt("PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC")
        min_plan_interval = _env_float_opt("PANEL_MOVEIT_BRIDGE_MIN_PLAN_INTERVAL_SEC")
        stale_ttl = _env_float_opt("PANEL_MOVEIT_BRIDGE_STALE_REQUEST_TTL_SEC")
        heartbeat_rate = _env_float_opt("PANEL_MOVEIT_BRIDGE_HEARTBEAT_RATE_HZ")
        require_rid = _env_flag("PANEL_MOVEIT_BRIDGE_REQUIRE_REQUEST_ID", True)
        drop_pending = _env_flag("PANEL_MOVEIT_BRIDGE_DROP_PENDING_ON_TAGGED", True)
        dry_run_plan_only = _env_flag("PANEL_MOVEIT_BRIDGE_DRY_RUN", False)
        if exec_timeout is None:
            exec_timeout = 30.0
        if req_timeout is None:
            req_timeout = 35.0
        if exec_timeout is not None:
            ros_args.extend(["-p", f"execute_timeout_sec:={max(1.0, exec_timeout):.3f}"])
        if req_timeout is not None:
            ros_args.extend(["-p", f"request_timeout_sec:={max(2.0, req_timeout):.3f}"])
        if min_plan_interval is not None:
            ros_args.extend(
                ["-p", f"min_plan_interval_sec:={max(0.0, min_plan_interval):.3f}"]
            )
        if stale_ttl is not None:
            ros_args.extend(["-p", f"stale_request_ttl_sec:={max(1.0, stale_ttl):.3f}"])
        if heartbeat_rate is not None:
            ros_args.extend(["-p", f"heartbeat_rate_hz:={max(0.2, heartbeat_rate):.3f}"])
        ros_args.extend(["-p", f"require_request_id:={'true' if require_rid else 'false'}"])
        ros_args.extend(
            [
                "-p",
                "drop_pending_on_tagged_request:="
                + ("true" if drop_pending else "false"),
            ]
        )
        ros_args.extend(
            [
                "-p",
                "dry_run_plan_only:="
                + ("true" if dry_run_plan_only else "false"),
            ]
        )
        path_constraint_tol = _env_float_opt("PANEL_MOVEIT_BRIDGE_PATH_CONSTRAINT_TOL_RAD")
        if path_constraint_tol is None:
            path_constraint_tol = 1.5
        ros_args.extend(["-p", f"path_constraint_joint_tolerance_rad:={max(0.0, path_constraint_tol):.3f}"])
        # Speed scaling for TEST ROBOT (default 0.80 for fast touch probe)
        vel_scale = _env_float_opt("PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE")
        if vel_scale is None:
            vel_scale = 0.80
        accel_scale = _env_float_opt("PANEL_MOVEIT_BRIDGE_ACCEL_SCALE")
        if accel_scale is None:
            accel_scale = 0.80
        ros_args.extend(["-p", f"max_velocity_scaling_factor:={max(0.05, min(1.0, vel_scale)):.2f}"])
        ros_args.extend(["-p", f"max_acceleration_scaling_factor:={max(0.05, min(1.0, accel_scale)):.2f}"])
        panel._emit_log(
            "[MOVEIT][BRIDGE] launch_param "
            f"controller_manager={cm_path} "
            f"use_sim_time={'true' if use_sim_time else 'false'} "
            "base_frame=base_link "
            "ee_frame=rg2_tcp "
            f"moveit_py_use_sim_time={'true' if bridge_moveit_py_sim_time else 'false'} "
            f"execute_timeout_sec={exec_timeout if exec_timeout is not None else 'default'} "
            f"request_timeout_sec={req_timeout if req_timeout is not None else 'default'} "
            f"min_plan_interval_sec={min_plan_interval if min_plan_interval is not None else 'default'} "
            f"stale_request_ttl_sec={stale_ttl if stale_ttl is not None else 'default'} "
            f"heartbeat_rate_hz={heartbeat_rate if heartbeat_rate is not None else 'default'} "
            f"require_request_id={'true' if require_rid else 'false'} "
            f"drop_pending_on_tagged_request={'true' if drop_pending else 'false'} "
            f"dry_run_plan_only={'true' if dry_run_plan_only else 'false'} "
            f"velocity_scaling={vel_scale:.2f} accel_scaling={accel_scale:.2f}"
        )
        cmd_core = with_line_buffer(
            " ".join(["ros2", "run", "ur5_tools", "ur5_moveit_bridge", *ros_args])
        )
        cmd = bash_preamble(panel.ws_dir) + env + f"{cmd_core} > '{bridge_log}' 2>&1"
        panel.moveit_bridge_proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            preexec_fn=os.setsid,
        )
        panel._started_moveit_bridge = True
        panel._moveit_bridge_running = True
        set_led(panel.led_moveit_bridge, "on")
        panel._set_status("MoveIt bridge lanzado")
        panel._refresh_controls()
        panel._emit_log(
            "[MOVEIT][BRIDGE] launched "
            f"pid={int(getattr(panel.moveit_bridge_proc, 'pid', 0) or 0)} "
            "node=ur5_moveit_bridge"
        )
        QTimer.singleShot(800, panel._clear_moveit_bridge_launching)
        def _verify_bridge_ready() -> None:
            pose_topic = "/desired_grasp"
            result_topic = "/desired_grasp/result"
            max_retries = 12
            for attempt in range(1, max_retries + 1):
                if not panel._proc_alive(panel.moveit_bridge_proc):
                    panel._emit_log(
                        f"[MOVEIT][BRIDGE] verify failed attempt={attempt}/{max_retries} reason=process_not_alive"
                    )
                    return
                if not panel._ros_worker_started:
                    panel._ensure_ros_worker_started()
                if panel._ros_worker_started and panel.ros_worker.node_ready():
                    pose_subs = panel.ros_worker.topic_subscriber_count(pose_topic)
                    result_pubs = panel.ros_worker.topic_publisher_count(result_topic)
                    panel._emit_log(
                        "[MOVEIT][BRIDGE] verify "
                        f"attempt={attempt}/{max_retries} pose_subs={pose_subs} result_pubs={result_pubs} "
                        f"pose_topic={pose_topic} result_topic={result_topic}"
                    )
                    if pose_subs > 0 and result_pubs > 0:
                        panel._emit_log(
                            f"[MOVEIT][BRIDGE] connected pose_subs={pose_subs} result_pubs={result_pubs}"
                        )
                        return
                time.sleep(0.4)
            panel._emit_log(
                "[MOVEIT][BRIDGE] WARN no conectado tras reintentos "
                f"pose_topic={pose_topic} result_topic={result_topic}"
            )
        panel._run_async(_verify_bridge_ready)
    except Exception as exc:
        panel._set_status(f"Error lanzando MoveIt bridge: {exc}", error=True)
        set_led(panel.led_moveit_bridge, "error")
        panel._moveit_bridge_running = False
        panel._clear_moveit_bridge_launching()

def stop_moveit_bridge(panel):
    if panel._block_if_managed("Stop MoveIt bridge"):
        return
    panel._log_button("Stop MoveIt bridge")
    panel._set_status("Deteniendo MoveIt bridge…")
    panel._kill_proc(panel.moveit_bridge_proc, "ur5_moveit_bridge")
    panel.moveit_bridge_proc = None
    panel._moveit_bridge_running = False
    panel._moveit_bridge_detected_cache = False
    panel._moveit_bridge_detected_ts = 0.0
    set_led(panel.led_moveit_bridge, "off")
    panel._refresh_controls()
