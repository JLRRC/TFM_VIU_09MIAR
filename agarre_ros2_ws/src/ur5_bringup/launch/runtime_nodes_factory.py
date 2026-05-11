#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/launch/runtime_nodes_factory.py
# Contenido: F17 — factory de actions de los nodos runtime del stack (TF, state, attach, MoveIt bridge).
"""Factory de actions de los nodos runtime del stack UR5.

Extraído de ``ur5_stack.launch.py`` en F17 (2026-05-01) para reducir
el tamaño del launch principal y mejorar la separación de
responsabilidades.

Incluye:
  - ``world_tf_publisher`` (LifecycleNode F13).
  - ``system_state_manager`` (LifecycleNode F13).
  - ``release_objects_service``.
  - ``gripper_attach_backend`` (LifecycleNode F13 observable).
  - ``planning_scene_sync``.
  - ``ur5_moveit_bridge`` (LifecycleNode F13 observable).

Uso desde ``ur5_stack.launch.py``::

    from runtime_nodes_factory import build_runtime_node_actions
    runtime_actions = build_runtime_node_actions(
        use_sim_time=use_sim_time,
        ...
    )

Devuelve una lista de actions lista para incluir en el
``LaunchDescription``. No cambia el comportamiento runtime — solo
extrae el wiring.
"""

from __future__ import annotations

from typing import Any, Dict, List

from launch.actions import (
    EmitEvent,
    GroupAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _build_gripper_attach_backend_node(
    *,
    use_sim_time,
    attach_backend_mode,
    attach_backend_max_pose_age_sec,
    attach_backend_follow_rate_hz,
    attach_backend_follow_break_dist_m,
    attach_backend_max_dist_m,
    attach_extras,
    demo_transport_objects,
    launch_attach_backend,
):
    """F3-step29a: ensambla el Node gripper_attach_backend con sus 11+ params (~35 LOC)."""
    gripper_attach_backend_params: List[Dict[str, Any]] = [
        {"use_sim_time": use_sim_time},
        {"attach_mode": attach_backend_mode},
        {"tcp_frame": "rg2_pinch_center"},
        {"tool_anchor_prefix": "/gripper_anchor"},
        {"max_pose_age_sec": attach_backend_max_pose_age_sec},
        {"follow_rate_hz": attach_backend_follow_rate_hz},
        {"follow_break_dist_m": attach_backend_follow_break_dist_m},
        {"attach_max_dist_m": attach_backend_max_dist_m},
        *attach_extras,
        {"detachable_shadow_follow": False},
    ]
    if demo_transport_objects:
        gripper_attach_backend_params.append(
            {"demo_transport_objects": demo_transport_objects}
        )
    return Node(
        package="ur5_tools",
        executable="gripper_attach_backend",
        output="screen",
        parameters=gripper_attach_backend_params,
        condition=IfCondition(launch_attach_backend),
    )


def _build_orchestrator_service_nodes(
    *,
    use_sim_time,
    world_name,
    launch_tf_geometry_service,
    launch_object_pose_resolver,
    launch_plan_to_pose_server,
    launch_pick_orchestrator_lifecycle=None,
    pick_orchestrator_use_stubs=None,
):
    """F3-step29b + F5-step8: tf_geometry_service + object_pose_resolver +
    plan_to_pose_server + pick_orchestrator_lifecycle (~70 LOC).

    Devuelve los 4 nodos necesarios para el orchestrator real. Cada uno
    con auto_activate o gating equivalente. F5-step8 añade el lifecycle
    node con auto_activate=True para que /pick_place esté siempre vivo.
    """
    tf_geometry_service = Node(
        package="ur5_tools",
        executable="tf_geometry_service",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"world_frame": "world"},
            {"base_frame": "base_link"},
            {"tf_timeout_sec": 0.2},
        ],
        condition=IfCondition(launch_tf_geometry_service),
    )
    object_pose_resolver = Node(
        package="ur5_tools",
        executable="object_pose_resolver_service",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"world_name": world_name},
            {"world_frame": "world"},
            {"max_pose_age_sec": 1.5},
            {"auto_activate": True},
        ],
        condition=IfCondition(launch_object_pose_resolver),
    )
    plan_to_pose_server_node = Node(
        package="ur5_tools",
        executable="plan_to_pose_server",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"action_name": "/orchestrator/plan_to_pose"},
            {"step_delay_sec": 0.10},
            # B-iter3-bis (2026-05-03): mode=MOVEIT_DIRECT por default.
            # Cliente directo a /move_action de MoveIt 2 (sin bridge al panel).
            # Esto hace al orchestrator INDEPENDIENTE del panel para las fases
            # APPROACH/LIFT/TRANSPORT. Antes (F5-step6d): use_real_bridge=true
            # publicaba a /desired_grasp y esperaba al panel — ese path queda
            # como fallback (mode=REAL_BRIDGE) seteable por param si MoveIt no
            # responde en algún despliegue.
            # 2026-05-07: dejado en MOVEIT_DIRECT. REAL_BRIDGE también falla
            # en sim por bug de path_tolerance_violation del joint_trajectory
            # _controller cuando max_start_err >> 0 (robot en pose post-aborto).
            # Bug arquitectónico del ur5_moveit_bridge ↔ controller pendiente
            # de sesión dedicada (ver auditoria/bugs_pendientes/BUG_BRIDGE_PATH_TOLERANCE.md).
            # MOVEIT_DIRECT da diagnostics más limpias (TIMED_OUT → CONTROL_FAILED
            # tras subir allowed_execution_duration_scaling 1.2→4.0).
            {"mode": "MOVEIT_DIRECT"},
            {"moveit_action_name": "/move_action"},
            {"moveit_group_name": "manipulator"},
            # Coherencia de frames: el group `manipulator` del SRDF usa
            # `rg2_pinch_center` como tip del chain.
            {"moveit_tip_link_override": "rg2_pinch_center"},
            # 2026-05-07: subido planning_time 10→25 + relajadas tolerancias.
            # Validación live ronda 10 mostró APPROACH timeout sistemático
            # con 10s y pos_tol=0.02. El sesgo determinista FK panel↔TF live
            # (~10mm pose pre-grasp) hace que MoveIt no encuentre solución
            # dentro de 10s con tolerancia 20mm. 25s + tol 0.05 da margen.
            {"moveit_planning_time_sec": 25.0},
            {"moveit_position_tol_m": 0.012},
            {"moveit_orientation_tol_rad": 0.20},
            {"moveit_tf_recovery_tol_m": 0.020},
            # 2026-05-07: subido a 400s. Con scaling=30 el upper bound de
            # MoveIt es ~315s; el orchestrator necesita esperar más allá
            # para no cortar antes de tiempo.
            {"moveit_result_timeout_sec": 400.0},
            # Backwards compat: si alguien fija mode="" + use_real_bridge=true,
            # el server cae a REAL_BRIDGE (mode efectivo).
            {"use_real_bridge": False},
            # F1.24 / H9 LIVE (2026-05-08): bypass MoveIt en APPROACH via FJT
            # directo. Evita el bug BUG_CONTROLLER_FEEDBACK_HANG del path
            # MoveIt simple_controller_manager. HOME_INITIAL ya usa FJT directo
            # y siempre funciona — extender ese patrón a APPROACH/LIFT/TRANSPORT.
            # Si IK falla o controller no responde, falls back a MoveIt path.
            {"bypass_moveit_for_short_paths": True},
            {"fjt_direct_allow_cartesian_bypass": False},
            {"fjt_direct_postcheck_tol_m": 0.015},
            {"fjt_direct_postcheck_timeout_sec": 0.35},
        ],
        condition=IfCondition(launch_plan_to_pose_server),
    )
    # F5-step8 (2026-05-03): pick_orchestrator_lifecycle auto-launch.
    # Action server /pick_place + clientes a los 6 services orchestrator.
    # Necesita configure+activate para aceptar goals; el lifecycle hace
    # ambas transitions automáticas via auto_activate dentro del propio
    # nodo (F9 pattern). Default true para que el dispatcher
    # PANEL_PICK_DEMO_USE_ORCHESTRATOR=1 funcione fuera de la caja.
    pick_orchestrator_lifecycle_node = Node(
        package="tfm_orchestrator",
        executable="pick_orchestrator_lifecycle",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"use_stubs": pick_orchestrator_use_stubs if pick_orchestrator_use_stubs is not None else False},
            {"snapshot_tcp_frame": "rg2_pinch_center"},
            {"auto_activate": True},
        ],
        condition=(
            IfCondition(launch_pick_orchestrator_lifecycle)
            if launch_pick_orchestrator_lifecycle is not None
            else None
        ),
    )
    return tf_geometry_service, object_pose_resolver, plan_to_pose_server_node, pick_orchestrator_lifecycle_node


def build_runtime_node_actions(
    *,
    use_sim_time: LaunchConfiguration,
    world_name: LaunchConfiguration,
    world_file: LaunchConfiguration,
    camera_required: LaunchConfiguration,
    controller_manager: LaunchConfiguration,
    launch_moveit: LaunchConfiguration,
    launch_world_tf: LaunchConfiguration,
    launch_system_state: LaunchConfiguration,
    launch_release_service: LaunchConfiguration,
    launch_attach_backend: LaunchConfiguration,
    launch_scene_sync: LaunchConfiguration,
    launch_tf_geometry_service: LaunchConfiguration,
    launch_object_pose_resolver: LaunchConfiguration,
    launch_plan_to_pose_server: LaunchConfiguration,
    launch_pick_orchestrator_lifecycle: "LaunchConfiguration | None" = None,
    pick_orchestrator_use_stubs: "LaunchConfiguration | None" = None,
    gz_delete_service: LaunchConfiguration,
    gz_spawn_service: LaunchConfiguration,
    attach_backend_mode: LaunchConfiguration,
    attach_backend_max_pose_age_sec: LaunchConfiguration,
    attach_backend_follow_rate_hz: LaunchConfiguration,
    attach_backend_follow_break_dist_m: LaunchConfiguration,
    attach_backend_max_dist_m: LaunchConfiguration,
    demo_transport_objects: List[str],
    bridge_params: List[Dict[str, Any]],
    system_state_yaml,
    system_state_extras: List[Dict[str, Any]],
    attach_extras: List[Dict[str, Any]],
) -> List[object]:
    """Construye las actions de los nodos runtime del stack UR5.

    Cada Node está condicionado por su flag de launch. Los nodos críticos
    al pipeline (system_state) tienen un ``RegisterEventHandler`` que
    derriba el stack si terminan inesperadamente; el ``world_tf_publisher``
    es no-crítico y no propaga shutdown al stack.
    """
    world_tf = Node(
        package="ur5_tools",
        executable="world_tf_publisher",
        output="screen",
        parameters=[
            {"world_name": world_name},
            {"model_name": "ur5_rg2"},
            {"base_frame": "base_link"},
            {"world_frame": "world"},
            # F1.9 (audit-v4 2026-05-08): subido 20 → 60s para sim con muchos plugins.
            {"clock_timeout_sec": 60.0},
            {"pose_timeout_sec": 12.0},
            {"use_sim_time": use_sim_time},
            # static_grace_sec < 0: publish world->base_link on /tf_static
            # immediately from the world file pose, without waiting for
            # /clock or Gazebo. Eliminates the TF-null window during the
            # first 3-5 minutes of startup.
            {"static_grace_sec": -1.0},
        ],
        condition=IfCondition(launch_world_tf),
    )
    # world_tf_publisher is helpful but non-critical; do not shutdown the
    # whole stack if it exits.
    world_tf_guard = GroupAction(actions=[])

    system_state = Node(
        package="ur5_tools",
        executable="system_state_manager",
        output="screen",
        parameters=[
            system_state_yaml,
            {"use_sim_time": use_sim_time},
            {"world_name": world_name},
            {"model_name": "ur5_rg2"},
            {"base_frame": "base_link"},
            {"world_frame": "world"},
            {"ee_frame": "rg2_tcp"},
            {"camera_topic": "/camera_overhead/image"},
            {"camera_required": ParameterValue(camera_required, value_type=bool)},
            {"controller_manager": controller_manager},
            {"moveit_required": ParameterValue(launch_moveit, value_type=bool)},
            *system_state_extras,
        ],
        condition=IfCondition(launch_system_state),
    )

    system_state_guard = RegisterEventHandler(
        OnProcessExit(
            target_action=system_state,
            on_exit=[
                EmitEvent(event=Shutdown(reason="system_state_manager exited"))
            ],
        ),
        condition=IfCondition(launch_system_state),
    )

    release_service = Node(
        package="ur5_tools",
        executable="release_objects_service",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"world_sdf": world_file},
            {"world_name": world_name},
            {"delete_service": gz_delete_service},
            {"spawn_service": gz_spawn_service},
            # F1.10 (audit-v4 2026-05-08): release_objects no asienta TODOS los
            # 10 objetos consistentemente. Causa raíz: el detach publisher
            # skip-on-sub_count==0 falla si el SDF plugin subscriptor no
            # ha conectado aún (ROS↔gz_ros2_bridge tarda). Subimos retries
            # de 4 (default) → 20 + retry_sec 0.25 → 0.5 (total 10s ventana).
            # Settle timeout 12 → 30s para física más lenta en sim.
            {"detach_publish_retries": 20},
            {"detach_publish_retry_sec": 0.5},
            {"settle_timeout_sec": 30.0},
            {"settle_confirmations": 4},
        ],
        condition=IfCondition(launch_release_service),
    )

    gripper_attach_backend = _build_gripper_attach_backend_node(
        use_sim_time=use_sim_time,
        attach_backend_mode=attach_backend_mode,
        attach_backend_max_pose_age_sec=attach_backend_max_pose_age_sec,
        attach_backend_follow_rate_hz=attach_backend_follow_rate_hz,
        attach_backend_follow_break_dist_m=attach_backend_follow_break_dist_m,
        attach_backend_max_dist_m=attach_backend_max_dist_m,
        attach_extras=attach_extras,
        demo_transport_objects=demo_transport_objects,
        launch_attach_backend=launch_attach_backend,
    )

    planning_scene_sync = Node(
        package="ur5_tools",
        executable="planning_scene_sync",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"world_name": world_name},
            {"world_file": world_file},
            {"world_frame": "world"},
            {"base_frame": "base_link"},
            {"ee_frame": "rg2_tcp"},
        ],
        condition=IfCondition(launch_scene_sync),
    )

    # 2026-05-09: ur5_moveit_bridge BORRADO con el path MoveIt-classic.
    # Antes: bridge subscribía a /desired_grasp para el botón panel "Agarre
    # Objeto MoveIt". Ese botón quedó deprecated; el path canónico es
    # orchestrator → /pick_place → plan_to_pose_server → FJT directo.

    tf_geometry_service, object_pose_resolver, plan_to_pose_server_node, pick_orchestrator_lifecycle_node = (
        _build_orchestrator_service_nodes(
            use_sim_time=use_sim_time,
            world_name=world_name,
            launch_tf_geometry_service=launch_tf_geometry_service,
            launch_object_pose_resolver=launch_object_pose_resolver,
            launch_plan_to_pose_server=launch_plan_to_pose_server,
            launch_pick_orchestrator_lifecycle=launch_pick_orchestrator_lifecycle,
            pick_orchestrator_use_stubs=pick_orchestrator_use_stubs,
        )
    )

    actions = [
        world_tf,
        world_tf_guard,
        system_state,
        system_state_guard,
        release_service,
        gripper_attach_backend,
        planning_scene_sync,
        # 2026-05-09: moveit_bridge eliminado del actions list (path borrado).
        tf_geometry_service,
        object_pose_resolver,
        plan_to_pose_server_node,
    ]
    if launch_pick_orchestrator_lifecycle is not None:
        actions.append(pick_orchestrator_lifecycle_node)
    return actions
