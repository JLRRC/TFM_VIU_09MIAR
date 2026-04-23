#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py
# Contenido: Configuracion de bringup ROS 2 para lanzar el sistema UR5.
# Uso breve: Colcon/ros2 launch lo usan para arrancar simulacion y componentes principales.
# URL: /home/laboratorio/TFM/agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py
# Resumen: Launch unificado oficial para Gazebo + bridge + ros2_control + panel.
"""Launch unificado oficial del stack UR5 (Gazebo, bridge, ros2_control, panel)."""

from __future__ import annotations

import os
import re
import time
import shutil
from typing import List

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    EmitEvent,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from ur5_tools.gripper_geometry import (
    RG2_TCP_FRAME,
    load_gripper_geometry,
    patch_runtime_model_sdf,
    validate_pick_demo_anchor,
)


def _env_flag(name: str, default: bool) -> bool:
    raw = str(os.environ.get(name, "1" if default else "0") or "").strip().lower()
    return raw in ("1", "true", "yes", "on")


def _env_float(name: str, default: float) -> float:
    raw = str(os.environ.get(name, str(default)) or "").strip()
    try:
        value = float(raw)
    except Exception:
        value = float(default)
    return value


def _prepare_runtime(context, *_args) -> List[object]:
    logger = get_logger("ur5_stack")
    ws_dir = os.environ.get("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws"))
    world_file = LaunchConfiguration("world_file").perform(context)
    strict_physics_mode = (
        str(LaunchConfiguration("strict_physics_mode").perform(context)).strip().lower()
        in ("1", "true", "yes", "on")
    )
    log_dir = os.path.join(ws_dir, "log")
    os.makedirs(log_dir, exist_ok=True)

    world_name = "ur5_mesa_objetos"
    try:
        with open(world_file, "r", encoding="utf-8") as f:
            data = f.read()
        match = re.search(r"<world name=\"([^\"]+)\">", data)
        if match:
            world_name = match.group(1)
    except Exception as exc:
        logger.warning("No se pudo leer world_name desde %s: %s", world_file, exc)

    gz_partition = os.environ.get("GZ_PARTITION", "")
    if not gz_partition:
        gz_partition = f"ur5pro_{int(time.time())}"
    try:
        with open(
            os.path.join(log_dir, "gz_partition.txt"), "w", encoding="utf-8"
        ) as f:
            f.write(gz_partition)
    except Exception:
        pass

    base_yaml = os.path.join(ws_dir, "scripts", "bridge_cameras.yaml")
    panel_settings_yaml = os.path.join(
        ws_dir, "src", "ur5_qt_panel", "config", "panel_settings.yaml"
    )
    runtime_yaml = os.path.join(log_dir, "bridge_runtime.yaml")
    try:
        with open(base_yaml, "r", encoding="utf-8") as f:
            yaml_text = f.read()
        if world_name:
            yaml_text = yaml_text.replace(
                "/world/ur5_mesa_objetos/",
                f"/world/{world_name}/",
            )
        with open(runtime_yaml, "w", encoding="utf-8") as f:
            f.write(yaml_text)
    except Exception as exc:
        logger.warning("No se pudo preparar bridge_runtime.yaml: %s", exc)
        runtime_yaml = base_yaml

    runtime_models_root = os.path.join(log_dir, "gz_models")
    runtime_ur5_model = os.path.join(runtime_models_root, "ur5_rg2")
    src_ur5_model = os.path.join(ws_dir, "models", "ur5_rg2")
    try:
        if os.path.isdir(src_ur5_model):
            shutil.copytree(src_ur5_model, runtime_ur5_model, dirs_exist_ok=True)
    except Exception as exc:
        logger.warning("No se pudo copiar modelo ur5_rg2 a runtime: %s", exc)

    resource_path = (
        f"{runtime_models_root}:{ws_dir}/models:{ws_dir}/worlds:{ws_dir}/install"
    )
    existing_resource = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_resource:
        resource_path = f"{resource_path}:{existing_resource}"
    plugin_path = "/opt/ros/jazzy/lib"
    existing_plugin = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    if existing_plugin:
        plugin_path = f"{plugin_path}:{existing_plugin}"
    render_engine = os.environ.get("GZ_RENDER_ENGINE", "").strip() or "ogre2"
    fastdds_profile = os.path.join(ws_dir, "scripts", "fastdds_no_shm.xml")
    # Prefer the explicit launch argument (camera_required:="0/1") over the env var,
    # because env var propagation via SetEnvironmentVariable is unreliable for
    # ExecuteProcess (panel) when DISPLAY is not usable (offscreen mode).
    launch_arg_camera = LaunchConfiguration("camera_required").perform(context).strip()
    camera_required_env = os.environ.get("PANEL_CAMERA_REQUIRED", "").strip()
    if launch_arg_camera in ("0", "false", "False", "no", "off"):
        camera_required_env = "0"
    elif launch_arg_camera in ("1", "true", "True", "yes", "on"):
        camera_required_env = "1"
    elif not camera_required_env:
        camera_required_env = "1"
    camera_required = camera_required_env in ("1", "true", "True")
    control_backend = (
        LaunchConfiguration("control_backend").perform(context).strip().lower()
    )
    moveit_mode = (
        LaunchConfiguration("moveit_mode").perform(context).strip().lower()
    )
    if moveit_mode not in ("auto", "move_group", "bridge"):
        logger.warning("moveit_mode invalido '%s'; usando auto", moveit_mode)
        moveit_mode = "auto"
    launch_gazebo_val = LaunchConfiguration("launch_gazebo").perform(context)
    launch_ros2_control_val = LaunchConfiguration("launch_ros2_control").perform(
        context
    )
    moveit_start_ros2_control_val = LaunchConfiguration(
        "moveit_start_ros2_control"
    ).perform(context)
    launch_ros2_control_eff = launch_ros2_control_val
    if control_backend in ("gz", "gazebo", "gz_ros2_control"):
        if str(launch_ros2_control_val).lower() in ("1", "true", "yes"):
            logger.warning(
                "control_backend=gz_ros2_control: desactivando ros2_control_node para evitar doble controller_manager."
            )
        launch_ros2_control_eff = "false"
    elif control_backend in ("ros2_control", "ros2_control_node"):
        if str(launch_gazebo_val).lower() in ("1", "true", "yes"):
            logger.error(
                "control_backend=ros2_control_node no es compatible con Gazebo: desactivando ros2_control_node."
            )
            launch_ros2_control_eff = "false"
        else:
            launch_ros2_control_eff = "true"
    else:
        logger.warning(
            "control_backend desconocido '%s'; se mantiene launch_ros2_control=%s",
            control_backend,
            launch_ros2_control_val,
        )
    launch_moveit_eff = LaunchConfiguration("launch_moveit").perform(context)
    panel_auto_bridge_eff = LaunchConfiguration("panel_auto_bridge").perform(context)
    if moveit_mode == "move_group":
        # Solo forzar launch_moveit si no se pasó explícitamente como false.
        # Con PANEL_START_STACK=0 (stack externo), start_panel_v2.sh pasa launch_moveit:=false
        # para evitar un segundo move_group que causaría el dual-bridge bug.
        if str(launch_moveit_eff).lower() not in ("0", "false", "no"):
            launch_moveit_eff = "true"
        if str(panel_auto_bridge_eff).lower() in ("1", "true", "yes"):
            logger.warning(
                "moveit_mode=move_group: desactivando panel_auto_bridge para evitar doble backend."
            )
        panel_auto_bridge_eff = "0"
    elif moveit_mode == "bridge":
        if str(launch_moveit_eff).lower() in ("1", "true", "yes"):
            logger.warning(
                "moveit_mode=bridge: desactivando launch_moveit para evitar doble backend."
            )
        launch_moveit_eff = "false"
    launch_scene_sync_eff = "true" if moveit_mode == "move_group" else "false"
    moveit_start_ros2_control_eff = moveit_start_ros2_control_val
    if str(launch_gazebo_val).lower() in ("1", "true", "yes"):
        if str(moveit_start_ros2_control_val).lower() in ("1", "true", "yes"):
            logger.warning(
                "Gazebo activo: desactivando moveit_start_ros2_control para evitar duplicar controller_manager."
            )
        moveit_start_ros2_control_eff = "false"
    launch_attach_backend_eff = LaunchConfiguration("launch_attach_backend").perform(context)
    if strict_physics_mode and str(launch_attach_backend_eff).lower() in ("1", "true", "yes"):
        logger.warning(
            "strict_physics_mode activo: desactivando launch_attach_backend para evitar agarre asistido por software."
        )
        launch_attach_backend_eff = "false"
    launch_flags = [
        LaunchConfiguration("launch_gazebo").perform(context),
        LaunchConfiguration("launch_rsp").perform(context),
        LaunchConfiguration("launch_bridge").perform(context),
        launch_ros2_control_eff,
        launch_moveit_eff,
    ]
    managed = any(str(flag).lower() in ("1", "true", "yes") for flag in launch_flags)
    managed_str = "1" if managed else "0"
    controllers_yaml = os.path.join(
        get_package_share_directory("ur5_description"),
        "config",
        "ur5_controllers.yaml",
    )
    try:
        model_sdf = os.path.join(runtime_ur5_model, "model.sdf")
        if not os.path.isfile(model_sdf):
            raise RuntimeError(f"Runtime model.sdf no encontrado: {model_sdf}")
        geometry = load_gripper_geometry(ws_dir=ws_dir)
        patch_runtime_model_sdf(
            model_sdf,
            controllers_yaml=controllers_yaml,
            geometry=geometry,
        )
        anchor_ok, anchor_reason = validate_pick_demo_anchor(
            model_sdf,
            geometry=geometry,
            tolerance_m=1e-6,
        )
        if not anchor_ok:
            raise RuntimeError(anchor_reason)
        logger.info(
            "runtime model geometry synced "
            f"source={geometry.tcp.source_path} "
            f"tcp_z={geometry.z_for_frame(RG2_TCP_FRAME):.7f} "
            f"{anchor_reason}"
        )
    except Exception as exc:
        logger.error("Runtime geometry sync failed: %s", exc)
        raise RuntimeError(f"Runtime geometry sync failed: {exc}") from exc

    runtime_world = world_file
    try:
        if os.path.isfile(world_file):
            with open(world_file, "r", encoding="utf-8") as f:
                world_text = f.read()
            headless_mode = LaunchConfiguration("headless").perform(context)
            keep_cameras = os.environ.get("PANEL_KEEP_CAMERAS", "").strip() in (
                "1",
                "true",
                "True",
            )
            if not keep_cameras:
                keep_cameras = camera_required
            if str(headless_mode).lower() in ("1", "true", "yes") and not keep_cameras:
                logger.warning(
                    "Headless sin cámaras: el sistema fallará por requisito crítico de visión."
                )
                for cam_name in (
                    "camera_overhead",
                    "camera_debug_top",
                    "camera_north",
                    "camera_south",
                    "camera_east",
                    "camera_west",
                ):
                    pattern = rf"<model\s+name=\"{cam_name}\">.*?</model>"
                    world_text = re.sub(pattern, "", world_text, flags=re.DOTALL)
            world_text = world_text.replace(
                "<uri>model://ur5_rg2</uri>",
                f"<uri>file://{runtime_ur5_model}</uri>",
            )
            runtime_world = os.path.join(log_dir, "world_runtime.sdf")
            with open(runtime_world, "w", encoding="utf-8") as f:
                f.write(world_text)
    except Exception as exc:
        logger.warning("No se pudo preparar world_runtime.sdf: %s", exc)
        runtime_world = world_file
    return [
        SetEnvironmentVariable("WS_DIR", ws_dir),
        SetEnvironmentVariable("GZ_PARTITION", gz_partition),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", plugin_path),
        SetEnvironmentVariable("GZ_RENDER_ENGINE", render_engine),
        SetEnvironmentVariable(
            "__EGL_VENDOR_LIBRARY_FILENAMES",
            "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
        ),
        SetEnvironmentVariable("UR5_CONTROLLERS_YAML", controllers_yaml),
        SetEnvironmentVariable("UR5_CONTROLLERS_FILE", controllers_yaml),
        SetEnvironmentVariable(
            "PANEL_CONTROLLER_MANAGER", LaunchConfiguration("controller_manager")
        ),
        SetEnvironmentVariable(
            "PANEL_AUTO_BRIDGE", panel_auto_bridge_eff
        ),
        SetEnvironmentVariable(
            "PANEL_AUTO_BRIDGE_DELAY_MS",
            LaunchConfiguration("panel_auto_bridge_delay_ms"),
        ),
        SetEnvironmentVariable("PANEL_MANAGED", managed_str),
        SetEnvironmentVariable("PANEL_CAMERA_REQUIRED", camera_required_env),
        SetEnvironmentVariable(
            "PANEL_DIRECT_DEBUG_ROOT",
            os.environ.get("PANEL_DIRECT_DEBUG_ROOT", "/home/laboratorio/TFM/historico"),
        ),
        SetEnvironmentVariable(
            # Default Z step is 0.025m → 2 segments over 35mm descent.
            # With 17mm jumps the IK can switch solution branch → 48mm Y deviation.
            # 5mm steps → 7 segments; each IK seeds from the previous (close) joints
            # and stays on the same branch throughout the descent.
            "PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_Z_STEP_M",
            os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_Z_STEP_M", "0.005"),
        ),
        SetEnvironmentVariable(
            # Use MoveIt computeCartesianPath for GRASP_DOWN instead of the
            # segmented IK loop. MoveIt generates a dense joint trajectory that
            # produces a straight Cartesian line in the arm model, avoiding the
            # j2 arc that the independent-IK path introduces. Falls back to the
            # conservative IK segmented approach if MoveIt is unavailable or the
            # planning fails. Set to "0" to disable and use the IK path.
            "PANEL_PICK_DEMO_GRASP_DOWN_USE_MOVEIT_CARTESIAN",
            os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_USE_MOVEIT_CARTESIAN", "1"),
        ),
        SetEnvironmentVariable(
            # IK seed weight during GRASP_DOWN descent. 0.035 matches
            # run_directo_validation.sh: IK finds near-identity joint changes so the
            # robot barely moves during GRASP_DOWN; GRASP_ALIGN_IK does the actual
            # 130mm descent. Higher values (0.65) cause the segmented IK to drift
            # the arm to a wrong position during the 7-segment descent.
            "PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT",
            os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT", "0.035"),
        ),
        SetEnvironmentVariable(
            # IK positional error tolerance for GRASP_DOWN fallback IK. The numeric
            # solver returns pos_err≈0.15m when the model-frame vs base_link sign
            # convention causes an apparent offset; the robot still converges to
            # near-approach Z because GRASP_ALIGN_IK does the final 130mm descent.
            # 0.200 matches run_directo_validation.sh so GRASP_DOWN passes and
            # GRASP_ALIGN_IK executes the contact-z correction.
            "PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL",
            os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_IK_ERR_TOL", "0.200"),
        ),
        SetEnvironmentVariable(
            # threshold for inherit_xy: if approach TCP-to-object XY < this, GRASP_DOWN
            # inherits the validated APPROACH_COARSE XY instead of reanchoring to the
            # object center. 3 mm was too tight for the observed coarse residuals and
            # was flipping GRASP_DOWN into unnecessary XY reanchors right on the
            # boundary, even after a successful APPROACH_COARSE gate.
            "PANEL_PICK_DEMO_GRASP_DOWN_KEEP_XY_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_KEEP_XY_TOL_M", "0.005"),
        ),
        SetEnvironmentVariable(
            # Second-step runtime guard: when APPROACH_COARSE already validated the XY,
            # keep that XY in GRASP_DOWN even if a later live TCP sample momentarily
            # drifts outside KEEP_XY_TOL_M.
            "PANEL_PICK_DEMO_GRASP_DOWN_FORCE_INHERIT_XY",
            os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_FORCE_INHERIT_XY", "1"),
        ),
        SetEnvironmentVariable(
            # Disable the permissive local GRASP_DOWN fallback that can reintroduce a
            # wrong IK branch mid-descent. The waypoint/joint-preset path remains.
            "PANEL_PICK_DEMO_GRASP_DOWN_DISABLE_PERMISSIVE_FALLBACK",
            os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_DISABLE_PERMISSIVE_FALLBACK", "1"),
        ),
        # Keep the direct pick gates aligned with the tighter panel defaults so
        # CLOSE/ATTACH only succeed when the object is really centered in the gripper.
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_ATTACH_XY_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_ATTACH_XY_TOL_M", "0.020"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_ATTACH_Z_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_ATTACH_Z_TOL_M", "0.010"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M",
            os.environ.get("PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M", "0.040"),
        ),
        # AttachGateEvaluator: ventana temporal de estabilidad objeto-TCP
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M",
            os.environ.get("PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M", "0.012"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC",
            os.environ.get("PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC", "0.35"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES",
            os.environ.get("PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES", "5"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M",
            os.environ.get("PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M", "0.020"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M",
            os.environ.get("PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M", "0.020"),
        ),
        # CLOSE: tolerancias y reintento de confirmación de pinza
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC",
            os.environ.get("PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC", "3.0"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM",
            os.environ.get("PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM", "0.01"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_GRIPPER_TARGET_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_GRIPPER_TARGET_TOL_M", "0.12"),
        ),
        # GRASP_DOWN: confirmación de estabilidad TF post-movimiento
        SetEnvironmentVariable(
            "PANEL_PICK_OBJECT_GRASP_TF_STABLE_TOL_M",
            os.environ.get("PANEL_PICK_OBJECT_GRASP_TF_STABLE_TOL_M", "0.045"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_OBJECT_GRASP_TF_STABLE_SAMPLES",
            os.environ.get("PANEL_PICK_OBJECT_GRASP_TF_STABLE_SAMPLES", "5"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_OBJECT_GRASP_TF_STABLE_MIN_OK",
            os.environ.get("PANEL_PICK_OBJECT_GRASP_TF_STABLE_MIN_OK", "4"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M",
            os.environ.get("PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M", "0.035"),
        ),
        SetEnvironmentVariable(
            # PRE_CLOSE xy tolerance. TCP arrives ~16-19mm from object after
            # GRASP_ALIGN_IK due to DH/SDF divergence (~13mm known offset).
            # 20mm accommodates this without compromising grasp reliability
            # (RG2 finger span is ~80mm so 20mm centering error is acceptable).
            "PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M", "0.020"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_PRE_CLOSE_Z_ERR_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_PRE_CLOSE_Z_ERR_TOL_M", "0.010"),
        ),
        SetEnvironmentVariable(
            # FK/trace freshness tolerance for POSE_CONSISTENCY gate.
            # Default 0.200s fails at PRE_CLOSE when panel_fk is ~220ms old
            # (normal after a slow settle). 0.400s covers this without accepting
            # truly stale data.
            "PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC",
            os.environ.get("PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC", "0.400"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_CLOSE_XY_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_CLOSE_XY_TOL_M", "0.008"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M", "0.008"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL",
            os.environ.get("PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL", "0.200"),
        ),
        SetEnvironmentVariable(
            # Exit XY tolerance for GRASP_ALIGN_IK convergence check.
            # TCP arrives at xy≈0.006m; the strict 0.006 fails in floating point
            # (real value 0.00635). 10mm clears the boundary and aligns with
            # the z-exit tolerance and PRE_CLOSE tolerances.
            "PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M", "0.020"),
        ),
        SetEnvironmentVariable(
            # Exit z-tolerance for GRASP_ALIGN_IK. The DH/SDF divergence at
            # grasp height leaves a ~6-7mm systematic residual after the bias
            # loop converges; 10mm accepts this without impacting grasp quality
            # (cylinder h=50mm, gripper contacts upper half at z≈0.031).
            "PANEL_PICK_DEMO_ALIGN_EXIT_Z_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_ALIGN_EXIT_Z_TOL_M", "0.010"),
        ),
        SetEnvironmentVariable(
            # z-bias correction activates when residual > this threshold.
            # Default in panel_pick_demo.py is 0.015, but the DH/SDF divergence
            # at grasp height is ~13mm — below 15mm, so bias never fired.
            # Lowering to 0.008 lets the bias kick in at attempt 2 and converge
            # in ≤3 attempts (bias overshoots to z≈0.032, attempt 4 exits OK
            # once exit_z_tol=0.010 accepts z_error≈0.007).
            "PANEL_PICK_DEMO_ALIGN_Z_RESIDUAL_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_ALIGN_Z_RESIDUAL_TOL_M", "0.008"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_PRE_CLOSE_REALIGN_RETRIES",
            os.environ.get("PANEL_PICK_DEMO_PRE_CLOSE_REALIGN_RETRIES", "2"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_GRASP_DOWN_STRICT_XY_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_STRICT_XY_TOL_M", "0.015"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_GRASP_DOWN_STRICT_Z_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_STRICT_Z_TOL_M", "0.008"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_GRASP_DOWN_STRICT_DIST_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_STRICT_DIST_TOL_M", "0.012"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_GRASP_DOWN_MAX_ATTEMPTS",
            os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_MAX_ATTEMPTS", "4"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_GRASP_DOWN_UTIL_XY_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_GRASP_DOWN_UTIL_XY_TOL_M", "0.015"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_GRASP_DOWN_UTIL_Z_ERR_TOL_M",
            "0.025",  # hard-coded: shell env can override to stale 0.008
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_APPROACH_COARSE_GATE_XY_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_APPROACH_COARSE_GATE_XY_TOL_M", "0.012"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_APPROACH_COARSE_GATE_Z_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_APPROACH_COARSE_GATE_Z_TOL_M", "0.012"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_POSE_SOURCE_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_POSE_SOURCE_TOL_M", "0.006"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_PHASE_JUMP_TOL_M",
            os.environ.get("PANEL_PICK_DEMO_PHASE_JUMP_TOL_M", "0.010"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SEC",
            os.environ.get("PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SEC", "2.5"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_DELTA_M",
            os.environ.get("PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_DELTA_M", "0.003"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SAMPLES",
            os.environ.get("PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SAMPLES", "3"),
        ),
        SetEnvironmentVariable(
            "PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_POLL_SEC",
            os.environ.get("PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_POLL_SEC", "0.10"),
        ),
        # NOTA (2026-04-16): La negación de X e Y en la ruta DIRECT IK es CORRECTA y
        # permanente. El solver DH usa base_link_inertia como raíz cinemática, que tiene
        # una rotación Rz(π) respecto a base_link. Para convertir un target expresado en
        # base_link al frame del modelo IK hay que negar X e Y — esto ocurre siempre en
        # panel_pick_demo.py. No existe env var que lo controle; la variable
        # PANEL_PICK_DEMO_DIRECT_IK_NEGATE_XY fue eliminada por ser letra muerta
        # (nunca se leía en el código) y su comentario anterior era incorrecto.
        SetEnvironmentVariable(
            "PANEL_MOVEIT_REQUIRED", launch_moveit_eff
        ),
        SetEnvironmentVariable("PANEL_MOVEIT_MODE", moveit_mode),
        SetEnvironmentVariable("PANEL_SETTINGS_YAML", panel_settings_yaml),
        SetEnvironmentVariable(
            "STRICT_SELF_COLLISION",
            "1" if strict_physics_mode else "0",
        ),
        SetEnvironmentVariable(
            "PANEL_STRICT_PHYSICS_MODE",
            "1" if strict_physics_mode else "0",
        ),
        SetEnvironmentVariable(
            "RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")
        ),
        SetEnvironmentVariable("RMW_FASTRTPS_USE_SHM", "0"),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile),
        SetLaunchConfiguration("runtime_yaml", runtime_yaml),
        SetLaunchConfiguration("controllers_file", controllers_yaml),
        SetLaunchConfiguration("world_file", runtime_world),
        SetLaunchConfiguration("world_name", world_name),
        SetLaunchConfiguration(
            "gz_delete_service",
            f"/world/{world_name}/remove/blocking",
        ),
        SetLaunchConfiguration(
            "gz_spawn_service",
            f"/world/{world_name}/create/blocking",
        ),
        SetLaunchConfiguration("camera_required", camera_required_env),
        SetLaunchConfiguration("panel_managed", managed_str),
        SetLaunchConfiguration("launch_moveit", launch_moveit_eff),
        SetLaunchConfiguration("panel_auto_bridge", panel_auto_bridge_eff),
        SetLaunchConfiguration("launch_ros2_control", launch_ros2_control_eff),
        SetLaunchConfiguration("launch_attach_backend", launch_attach_backend_eff),
        SetLaunchConfiguration("launch_scene_sync", launch_scene_sync_eff),
        SetLaunchConfiguration(
            "moveit_start_ros2_control", moveit_start_ros2_control_eff
        ),
    ]


def _maybe_moveit(context, *_args) -> List[object]:
    launch_moveit = LaunchConfiguration("launch_moveit").perform(context)
    if str(launch_moveit).lower() not in ("1", "true", "yes"):
        return []
    moveit_start_ros2_control = LaunchConfiguration("moveit_start_ros2_control")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur5_moveit_config"),
                        "launch",
                        "ur5_moveit_bringup.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "start_ros2_control": moveit_start_ros2_control,
                "launch_rviz": "false",
                "use_sim_time": use_sim_time,
                "strict_self_collision": LaunchConfiguration("strict_physics_mode"),
            }.items(),
        )
    ]


def generate_launch_description():
    ws_dir = os.environ.get("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws"))
    world_default = os.path.join(ws_dir, "worlds", "ur5_mesa_objetos.sdf")
    gui_config_default = (
        "/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config"
    )

    headless = LaunchConfiguration("headless")
    launch_panel = LaunchConfiguration("launch_panel")
    launch_bridge = LaunchConfiguration("launch_bridge")
    launch_gazebo = LaunchConfiguration("launch_gazebo")
    launch_rsp = LaunchConfiguration("launch_rsp")
    launch_ros2_control = LaunchConfiguration("launch_ros2_control")
    launch_world_tf = LaunchConfiguration("launch_world_tf")
    launch_release_service = LaunchConfiguration("launch_release_service")
    launch_attach_backend = LaunchConfiguration("launch_attach_backend")
    launch_scene_sync = LaunchConfiguration("launch_scene_sync")
    launch_system_state = LaunchConfiguration("launch_system_state")
    launch_moveit = LaunchConfiguration("launch_moveit")
    camera_required = LaunchConfiguration("camera_required")
    bootstrap_controllers = LaunchConfiguration("bootstrap_controllers")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world_file = LaunchConfiguration("world_file")
    render_engine = LaunchConfiguration("render_engine")
    gui_config_file = LaunchConfiguration("gui_config_file")

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur5_bringup"), "launch", "ur5_rsp.launch.py"]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(launch_rsp),
    )

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur5_bringup"),
                    "launch",
                    "ur5_ros2_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "controllers_yaml": "ur5_mock_controllers.yaml",
        }.items(),
        condition=IfCondition(launch_ros2_control),
    )

    controller_bootstrap = Node(
        package="ur5_tools",
        executable="controller_bootstrap",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"controller_manager": LaunchConfiguration("controller_manager")},
            {"wait_for_clock": True},
            {"clock_timeout_sec": 12.0},
            {"service_timeout_sec": 8.0},
            {"autostart": True},
            {"stay_alive": True},
        ],
        condition=IfCondition(bootstrap_controllers),
    )

    gz_headless = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-s",
            "-r",
            "--headless-rendering",
            "--render-engine",
            render_engine,
            world_file,
        ],
        output="screen",
        condition=IfCondition(headless),
    )
    gz_gui_server = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-s",
            "-r",
            "--headless-rendering",
            "--render-engine",
            render_engine,
            world_file,
        ],
        output="screen",
        condition=UnlessCondition(headless),
    )
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-g", "--gui-config", gui_config_file],
        output="screen",
        condition=UnlessCondition(headless),
    )
    gz_group = GroupAction(
        actions=[gz_headless, gz_gui_server, gz_gui],
        condition=IfCondition(launch_gazebo),
    )
    gz_shutdown_headless = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_headless,
            on_exit=[EmitEvent(event=Shutdown(reason="gz sim exited (headless)"))],
        ),
        condition=IfCondition(launch_gazebo),
    )
    gz_shutdown_gui = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_gui_server,
            on_exit=[EmitEvent(event=Shutdown(reason="gz sim server exited (gui mode)"))],
        ),
        condition=IfCondition(launch_gazebo),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_main",
        output="screen",
        arguments=["--ros-args", "-p", "use_sim_time:=true"],
        parameters=[
            {"config_file": LaunchConfiguration("runtime_yaml")},
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        condition=IfCondition(launch_bridge),
    )
    bridge_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason="parameter_bridge exited"))],
        ),
        condition=IfCondition(launch_bridge),
    )

    gz_control_guard = Node(
        package="ur5_tools",
        executable="gz_ros_control_guard",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"hold_joints": False},
        ],
        condition=IfCondition(launch_gazebo),
    )

    gz_pose_bridge = Node(
        package="ur5_tools",
        executable="gz_pose_bridge",
        output="screen",
        parameters=[
            {"world_name": LaunchConfiguration("world_name")},
            {"world_frame": "world"},
            {"startup_timeout_sec": 5.0},
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(launch_gazebo),
    )
    gz_pose_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_pose_bridge,
            on_exit=[EmitEvent(event=Shutdown(reason="gz_pose_bridge exited"))],
        ),
        condition=IfCondition(launch_gazebo),
    )

    world_tf = Node(
        package="ur5_tools",
        executable="world_tf_publisher",
        output="screen",
        parameters=[
            {"world_name": LaunchConfiguration("world_name")},
            {"model_name": "ur5_rg2"},
            {"base_frame": "base_link"},
            {"world_frame": "world"},
            {"clock_timeout_sec": 20.0},
            {"pose_timeout_sec": 12.0},
            {"use_sim_time": use_sim_time},
            # static_grace_sec < 0: publish world->base_link on /tf_static immediately
            # from the world file pose, without waiting for /clock or Gazebo.
            # Eliminates the TF-null window during the first 3-5 minutes of startup.
            {"static_grace_sec": -1.0},
        ],
        condition=IfCondition(launch_world_tf),
    )
    # world_tf_publisher is helpful but non-critical; do not shutdown whole stack if it exits.
    world_tf_guard = GroupAction(actions=[])

    system_state = Node(
        package="ur5_tools",
        executable="system_state_manager",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("ur5_bringup"), "config", "system_state_manager.yaml"]
            ),
            {"use_sim_time": use_sim_time},
            {"world_name": LaunchConfiguration("world_name")},
            {"model_name": "ur5_rg2"},
            {"base_frame": "base_link"},
            {"world_frame": "world"},
            {"ee_frame": "rg2_tcp"},
            {"camera_topic": "/camera_overhead/image"},
            {"camera_required": ParameterValue(camera_required, value_type=bool)},
            {"controller_manager": LaunchConfiguration("controller_manager")},
            {"moveit_required": ParameterValue(launch_moveit, value_type=bool)},
            {
                "geometry_offset_tol_m": float(
                    os.environ.get("SYSTEM_STATE_GEOMETRY_OFFSET_TOL_M", "0.002")
                )
            },
            {
                "geometry_pair_tol_m": float(
                    os.environ.get("SYSTEM_STATE_GEOMETRY_PAIR_TOL_M", "0.001")
                )
            },
            {
                "startup_timeout_sec": float(
                    os.environ.get("SYSTEM_STATE_STARTUP_TIMEOUT_SEC", "15.0")
                )
            },
        ],
        condition=IfCondition(launch_system_state),
    )

    system_state_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=system_state,
            on_exit=[EmitEvent(event=Shutdown(reason="system_state_manager exited"))],
        ),
        condition=IfCondition(launch_system_state),
    )

    release_service = Node(
        package="ur5_tools",
        executable="release_objects_service",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"world_sdf": LaunchConfiguration("world_file")},
            {"world_name": LaunchConfiguration("world_name")},
            {"delete_service": LaunchConfiguration("gz_delete_service")},
            {"spawn_service": LaunchConfiguration("gz_spawn_service")},
            {"settle_timeout_sec": 12.0},
            {"settle_confirmations": 6},
        ],
        condition=IfCondition(launch_release_service),
    )

    demo_transport_objects_env = os.environ.get(
        "ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS",
        "pick_demo",
    )
    demo_transport_objects = [
        value.strip()
        for value in demo_transport_objects_env.split(",")
        if value.strip()
    ]

    gripper_attach_backend_params = [
        {"use_sim_time": use_sim_time},
        {"attach_mode": LaunchConfiguration("attach_backend_mode")},
        {"tcp_frame": "rg2_pinch_center"},
        {"tool_anchor_prefix": "/gripper_anchor"},
        {
            "max_pose_age_sec": LaunchConfiguration(
                "attach_backend_max_pose_age_sec"
            )
        },
        {
            "follow_rate_hz": LaunchConfiguration(
                "attach_backend_follow_rate_hz"
            )
        },
        {
            "follow_break_dist_m": LaunchConfiguration(
                "attach_backend_follow_break_dist_m"
            )
        },
        # FIX-ATTACH-THRESHOLD: tighten from 0.15 m to 0.05 m so that the backend
        # only attaches when the TCP is geometrically close to the object (~contact).
        # 0.15 m allowed false grasps; 0.05 m requires the gripper to be within ~5 cm.
        {
            "attach_max_dist_m": LaunchConfiguration(
                "attach_backend_max_dist_m"
            )
        },
        {"gz_service_timeout_ms": int(os.environ.get("ATTACH_BACKEND_GZ_SERVICE_TIMEOUT_MS", "2000"))},
        {"gz_cmd_timeout_sec": float(os.environ.get("ATTACH_BACKEND_GZ_CMD_TIMEOUT_SEC", "3.0"))},
        # detachable_shadow_follow=False: disables the physics-joint shadow path for
        # non-demo-transport objects. pick_demo goes through demo_transport exclusively
        # (demo_transport_objects check fires first in _on_gripper_attach), so this
        # flag only affects other objects that use detachable_joint mode.
        {"detachable_shadow_follow": False},
        # For the demo object we prefer deterministic transport once the grasp
        # geometry gate has been passed. Leaving the list empty keeps the default
        # follow_tcp behavior for the rest of the objects.
        {"demo_transport_objects": demo_transport_objects},
    ]
    if not demo_transport_objects:
        gripper_attach_backend_params = gripper_attach_backend_params[:-1]

    gripper_attach_backend = Node(
        package="ur5_tools",
        executable="gripper_attach_backend",
        output="screen",
        parameters=gripper_attach_backend_params,
        condition=IfCondition(launch_attach_backend),
    )

    planning_scene_sync = Node(
        package="ur5_tools",
        executable="planning_scene_sync",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"world_name": LaunchConfiguration("world_name")},
            {"world_file": LaunchConfiguration("world_file")},
            {"world_frame": "world"},
            {"base_frame": "base_link"},
            {"ee_frame": "rg2_tcp"},
        ],
        condition=IfCondition(launch_scene_sync),
    )

    # FIX-DESIRED-GRASP: launch ur5_moveit_bridge as a standalone node so that
    # /desired_grasp always has a real subscriber in the normal boot.
    # Previously Subscription count was 0 because the bridge was only started
    # on-demand via the panel button; this makes it part of the managed launch.
    # When MoveIt (move_group) is not running the bridge will subscribe but will
    # fall back to a no-op, ensuring the topic is live without crashing the stack.
    bridge_exec_timeout = max(1.0, _env_float("PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC", 150.0))
    bridge_request_timeout = max(2.0, _env_float("PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC", 180.0))
    bridge_stale_request_ttl_sec = max(
        20.0,
        _env_float("PANEL_MOVEIT_BRIDGE_STALE_REQUEST_TTL_SEC", 120.0),
    )
    bridge_joint_state_timeout = max(0.2, _env_float("PANEL_MOVEIT_BRIDGE_JOINT_STATE_TIMEOUT_SEC", 6.0))
    bridge_joint_state_max_age = max(0.1, _env_float("PANEL_MOVEIT_BRIDGE_JOINT_STATE_MAX_AGE_SEC", 2.5))
    bridge_force_fjt_direct = _env_flag("PANEL_MOVEIT_BRIDGE_FORCE_FJT_DIRECT", False)
    bridge_unwrap_continuous = _env_flag("PANEL_MOVEIT_BRIDGE_UNWRAP_CONTINUOUS_JOINTS", False)
    bridge_require_request_id = _env_flag("PANEL_MOVEIT_BRIDGE_REQUIRE_REQUEST_ID", True)
    bridge_drop_pending = _env_flag("PANEL_MOVEIT_BRIDGE_DROP_PENDING_ON_TAGGED", True)
    bridge_dry_run = _env_flag("PANEL_MOVEIT_BRIDGE_DRY_RUN", False)
    bridge_path_constraint_tol = max(0.0, _env_float("PANEL_MOVEIT_BRIDGE_PATH_CONSTRAINT_TOL_RAD", 0.35))
    bridge_controller_goal_time_tol = max(
        0.0,
        _env_float("PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TIME_TOL_SEC", 45.0),
    )
    bridge_controller_expected_goal_time = max(
        0.0,
        _env_float(
            "PANEL_MOVEIT_BRIDGE_CONTROLLER_EXPECTED_GOAL_TIME_SEC",
            max(45.0, bridge_controller_goal_time_tol),
        ),
    )
    bridge_controller_path_tol = max(
        0.0,
        _env_float("PANEL_MOVEIT_BRIDGE_CONTROLLER_PATH_TOL_RAD", 4.0),
    )
    bridge_controller_goal_tol = max(
        0.0,
        _env_float("PANEL_MOVEIT_BRIDGE_CONTROLLER_GOAL_TOL_RAD", 0.20),
    )
    # FIX-VELOCITY-SCALING: wire up PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE /
    # PANEL_MOVEIT_BRIDGE_ACCEL_SCALE env vars (set in start_panel_v2.sh) to the
    # bridge node parameters.  Previously these were exported but never read by the
    # bridge, so it silently used the hard-coded default of 0.30.  Default kept at
    # 0.30 here; set to 0.15 in start_panel_v2.sh for a more conservative profile.
    bridge_velocity_scale = max(
        0.05,
        min(1.0, _env_float("PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE", 0.30)),
    )
    bridge_accel_scale = max(
        0.05,
        min(1.0, _env_float("PANEL_MOVEIT_BRIDGE_ACCEL_SCALE", 0.30)),
    )
    moveit_bridge = Node(
        package="ur5_tools",
        executable="ur5_moveit_bridge",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"backend": "auto"},
            {"base_frame": "base_link"},
            {"ee_frame": "rg2_tcp"},
            {"result_topic": "/desired_grasp/result"},
            {"controller_manager": LaunchConfiguration("controller_manager")},
            {"execute_timeout_sec": bridge_exec_timeout},
            {"request_timeout_sec": bridge_request_timeout},
            {"stale_request_ttl_sec": bridge_stale_request_ttl_sec},
            {"joint_state_valid_timeout_sec": bridge_joint_state_timeout},
            {"joint_state_valid_max_age_sec": bridge_joint_state_max_age},
            {"force_fjt_direct_for_walltime_sim": bridge_force_fjt_direct},
            {"unwrap_continuous_joints": bridge_unwrap_continuous},
            {"require_request_id": bridge_require_request_id},
            {"drop_pending_on_tagged_request": bridge_drop_pending},
            {"dry_run_plan_only": bridge_dry_run},
            {"path_constraint_joint_tolerance_rad": bridge_path_constraint_tol},
            {"controller_goal_time_tolerance_sec": bridge_controller_goal_time_tol},
            {"controller_expected_goal_time_sec": bridge_controller_expected_goal_time},
            {"controller_path_tolerance_rad": bridge_controller_path_tol},
            {"controller_goal_tolerance_rad": bridge_controller_goal_tol},
            {"max_velocity_scaling_factor": bridge_velocity_scale},
            {"max_acceleration_scaling_factor": bridge_accel_scale},
        ],
        condition=IfCondition(LaunchConfiguration("launch_moveit_bridge")),
    )

    panel_python = os.environ.get("PANEL_PYTHON", "")
    if panel_python:
        panel_cmd = [panel_python, "-m", "ur5_qt_panel.panel_v2"]
    else:
        panel_cmd = ["ros2", "run", "ur5_qt_panel", "panel_v2"]
    panel = ExecuteProcess(
        cmd=panel_cmd,
        output="screen",
        additional_env={"PANEL_CAMERA_REQUIRED": LaunchConfiguration("camera_required")},
        condition=IfCondition(launch_panel),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=world_default),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("launch_panel", default_value="true"),
            DeclareLaunchArgument("launch_bridge", default_value="true"),
            DeclareLaunchArgument("launch_gazebo", default_value="true"),
            DeclareLaunchArgument("launch_rsp", default_value="true"),
            DeclareLaunchArgument("launch_ros2_control", default_value="false"),
            DeclareLaunchArgument("control_backend", default_value="gz_ros2_control"),
            DeclareLaunchArgument("launch_world_tf", default_value="true"),
            DeclareLaunchArgument("launch_release_service", default_value="true"),
            DeclareLaunchArgument("launch_attach_backend", default_value="true"),
            DeclareLaunchArgument(
                "attach_backend_mode",
                default_value=os.environ.get("ATTACH_BACKEND_MODE", "follow_tcp"),
            ),
            DeclareLaunchArgument(
                "strict_physics_mode",
                default_value=os.environ.get("STRICT_PHYSICS_MODE", "false"),
            ),
            DeclareLaunchArgument(
                "attach_backend_max_pose_age_sec",
                default_value=os.environ.get("ATTACH_BACKEND_MAX_POSE_AGE_SEC", "1.5"),
            ),
            DeclareLaunchArgument(
                "attach_backend_follow_rate_hz",
                default_value=os.environ.get("ATTACH_BACKEND_FOLLOW_RATE_HZ", "20.0"),
            ),
            DeclareLaunchArgument(
                "attach_backend_follow_break_dist_m",
                default_value=os.environ.get("ATTACH_BACKEND_FOLLOW_BREAK_DIST_M", "0.18"),
            ),
            # FIX-ATTACH-THRESHOLD: max 3D distance (m) for backend to accept attach.
            # Was 0.15 (default in gripper_attach_backend.py). Tightened for real
            # contact, but 0.050 was too strict for the direct grasp and rejected a
            # valid manual-like close at 0.0509 m. 0.080 accommodates small object
            # movements during descent while remaining far below 0.150 false-grasp risk.
            DeclareLaunchArgument(
                "attach_backend_max_dist_m",
                default_value=os.environ.get("ATTACH_BACKEND_MAX_DIST_M", "0.08"),
            ),
            # FIX-MOVEIT-BRIDGE: launch ur5_moveit_bridge so /desired_grasp has a real subscriber.
            DeclareLaunchArgument("launch_moveit_bridge", default_value="true"),
            DeclareLaunchArgument("launch_scene_sync", default_value="true"),
            DeclareLaunchArgument("launch_system_state", default_value="true"),
            DeclareLaunchArgument("launch_moveit", default_value="false"),
            DeclareLaunchArgument("moveit_mode", default_value="auto"),
            DeclareLaunchArgument("camera_required", default_value="1"),
            DeclareLaunchArgument("moveit_start_ros2_control", default_value="false"),
            DeclareLaunchArgument("bootstrap_controllers", default_value="true"),
            DeclareLaunchArgument(
                "controller_manager", default_value="/controller_manager"
            ),
            DeclareLaunchArgument("panel_auto_bridge", default_value="0"),
            DeclareLaunchArgument("panel_auto_bridge_delay_ms", default_value="1200"),
            DeclareLaunchArgument("panel_managed", default_value="1"),
            DeclareLaunchArgument(
                "rmw_implementation", default_value="rmw_fastrtps_cpp"
            ),
            DeclareLaunchArgument("render_engine", default_value="ogre2"),
            DeclareLaunchArgument(
                "gui_config_file",
                default_value=gui_config_default,
            ),
            OpaqueFunction(function=_prepare_runtime),
            OpaqueFunction(function=_maybe_moveit),
            rsp_launch,
            ros2_control_launch,
            gz_group,
            gz_shutdown_headless,
            gz_shutdown_gui,
            bridge,
            bridge_guard,
            controller_bootstrap,
            gz_control_guard,
            gz_pose_bridge,
            gz_pose_guard,
            world_tf,
            world_tf_guard,
            system_state,
            system_state_guard,
            release_service,
            gripper_attach_backend,
            planning_scene_sync,
            moveit_bridge,
            panel,
        ]
    )
