#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/phase_dispatch.py
# Contenido: F5-step1 — dispatch puro de fases pick_place compartido entre Node y LifecycleNode.
"""Dispatch puro de fases del pick_place orchestrator.

Hasta F5-step1, la lógica de ``_dispatch_phase_service`` vivía duplicada
entre ``pick_orchestrator_node.py`` (Node legacy F5/F6) y
``pick_orchestrator_lifecycle_node.py`` (LifecycleNode canónico F9+).
El segundo sólo cubría ``SELECT_OBJECT`` y devolvía ``_no_op`` para el
resto, lo que dejaba el path canónico sin paridad funcional con el legacy.

Este módulo extrae el dispatch a una capa pura: recibe ``node`` y un
contexto explícito (``service_map``, ``client_cache``, timeouts) en lugar
de leer atributos del nodo directamente. Esto permite:

* Reuso 1:1 entre ambos orchestrators (un único punto de verdad).
* Tests unitarios sin levantar ROS — basta inyectar callers mock vía
  los parámetros ``service_caller`` y ``action_caller``.
* Evolución futura (F5-step2/3) sin tocar los nodos: la lógica vive aquí.

Contrato del callable:
    dispatch(phase, ctx) -> tuple[bool, str]

Donde:
    phase: PickPhase
    ctx:   PickContext (object_name + drop_xyz_world)
    return: (success, reason). reason describe el resultado para el FSM.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional, Tuple

from .pick_fsm import PickContext, PickPhase, is_no_hint, normalize_pose_hint
from .service_clients import (
    ActionCallResult,
    PhaseServiceMap,
    ServiceCallResult,
    call_action_with_timeout,
    call_service_with_timeout,
)


# F5-step2: clearance Z por defecto sobre la pose del objeto para approach.
# El cliente puede sobreescribir vía PhaseDispatchContext si fuese necesario.
_APPROACH_Z_CLEARANCE_M_DEFAULT = 0.10


def pose_msg_to_tuple7(pose_msg: Any) -> Optional[tuple]:
    """Convierte un ``geometry_msgs/Pose`` a tuple ``(x,y,z,qx,qy,qz,qw)``.

    Devuelve None si ``pose_msg`` es None o no expone los atributos
    esperados — fail-soft (deja que ``is_no_hint`` lo trate como "sin hint").
    """
    if pose_msg is None:
        return None
    try:
        pos = pose_msg.position
        ori = pose_msg.orientation
        return (
            float(pos.x), float(pos.y), float(pos.z),
            float(ori.x), float(ori.y), float(ori.z), float(ori.w),
        )
    except (AttributeError, TypeError, ValueError):
        return None


# Tipos para los inyectables (testabilidad).
ServiceCallerFn = Callable[..., ServiceCallResult]
ActionCallerFn = Callable[..., ActionCallResult]


@dataclass
class PhaseDispatchContext:
    """Contexto de dispatch: agrupa los recursos del orchestrator.

    Pasado a ``dispatch_phase`` para evitar lecturas implícitas del nodo.

    Atributos:
        node: ROS 2 Node usado por los callers para crear clients.
        service_map: nombres de services/actions por fase.
        client_cache: dict reusable para no recrear clients.
        discovery_timeout_sec: timeout de wait_for_service / wait_for_server.
        call_timeout_sec: timeout de la llamada al service.
        action_accept_timeout_sec: timeout del accept del goal (action).
        action_result_timeout_sec: timeout del result tras accept (action).
        service_caller: inyectable para tests (default = real).
        action_caller: inyectable para tests (default = real).
        approach_z_clearance_m: clearance vertical sobre la pose del objeto
            cuando el ctx tiene hint válido (F5-step2).
        resolve_object_pose_discovery_timeout_sec: timeout corto para
            sondar el service ``ResolveObjectPoseWorld`` (F5-step3). Si
            el server no responde en este tiempo, dispatch_phase usa
            placeholder en lugar de bloquear.
    """

    node: Any
    service_map: PhaseServiceMap = field(default_factory=PhaseServiceMap)
    client_cache: Dict[str, Any] = field(default_factory=dict)
    discovery_timeout_sec: float = 2.0
    call_timeout_sec: float = 10.0
    action_accept_timeout_sec: float = 3.0
    action_result_timeout_sec: float = 60.0
    service_caller: ServiceCallerFn = call_service_with_timeout
    action_caller: ActionCallerFn = call_action_with_timeout
    approach_z_clearance_m: float = _APPROACH_Z_CLEARANCE_M_DEFAULT
    resolve_object_pose_discovery_timeout_sec: float = 0.3

    def common_service_kwargs(self) -> Dict[str, Any]:
        return dict(
            discovery_timeout_sec=self.discovery_timeout_sec,
            call_timeout_sec=self.call_timeout_sec,
            client_cache=self.client_cache,
        )

    def common_action_kwargs(self) -> Dict[str, Any]:
        return dict(
            discovery_timeout_sec=self.discovery_timeout_sec,
            accept_timeout_sec=max(self.discovery_timeout_sec, self.action_accept_timeout_sec),
            result_timeout_sec=max(self.call_timeout_sec, self.action_result_timeout_sec),
            client_cache=self.client_cache,
        )


# ---------------------------------------------------------------------------
# Goal builders (puros — no dependen de ROS, sólo construyen mensajes)
# ---------------------------------------------------------------------------


def build_plan_to_pose_goal_for_approach(
    ctx: PickContext,
    *,
    z_clearance_m: float = _APPROACH_Z_CLEARANCE_M_DEFAULT,
) -> Any:
    """Goal de APPROACH para el PlanToPose action server.

    F5-step2: si ``ctx.object_pose_world_hint`` es un hint válido (no
    cumple ``is_no_hint``), construye el goal usando esa pose con
    ``z_clearance_m`` añadidos en Z y la orientación del hint. Si no
    hay hint, mantiene el placeholder F5-step1 (pose neutra origen).

    Nota: la pose viaja como "world" en el target (el server PlanToPose
    es responsable de la conversión world→base si hace falta).
    """
    from geometry_msgs.msg import Pose, Point, Quaternion
    from ur5_panel_interfaces.action import PlanToPose

    goal = PlanToPose.Goal()
    if not is_no_hint(ctx.object_pose_world_hint):
        norm = normalize_pose_hint(ctx.object_pose_world_hint)
        # norm garantizado != None aquí (is_no_hint cubre None y mal-formados).
        x, y, z, qx, qy, qz, qw = norm  # type: ignore[misc]
        goal.target_pose_base = Pose(
            position=Point(x=x, y=y, z=z + float(z_clearance_m)),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        )
    else:
        goal.target_pose_base = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    goal.ee_frame = "rg2_pinch_center"
    goal.cartesian = False
    goal.timeout_sec = 0.0
    return goal


def build_plan_to_pose_goal_for_lift(ctx: PickContext) -> Any:
    """Goal de LIFT: Z+ relativo (placeholder hasta F5-step2)."""
    from geometry_msgs.msg import Pose, Point, Quaternion
    from ur5_panel_interfaces.action import PlanToPose

    goal = PlanToPose.Goal()
    goal.target_pose_base = Pose(
        position=Point(x=0.0, y=0.0, z=0.20),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    goal.ee_frame = "rg2_pinch_center"
    goal.cartesian = True
    goal.timeout_sec = 0.0
    return goal


def build_plan_to_pose_goal_for_transport(ctx: PickContext) -> Any:
    """Goal de TRANSPORT: drop_xyz_world del request original."""
    from geometry_msgs.msg import Pose, Point, Quaternion
    from ur5_panel_interfaces.action import PlanToPose

    goal = PlanToPose.Goal()
    goal.target_pose_base = Pose(
        position=Point(
            x=float(ctx.drop_xyz_world[0]),
            y=float(ctx.drop_xyz_world[1]),
            z=float(ctx.drop_xyz_world[2]),
        ),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    goal.ee_frame = "rg2_pinch_center"
    goal.cartesian = False
    goal.timeout_sec = 0.0
    return goal


# ---------------------------------------------------------------------------
# Dispatch principal
# ---------------------------------------------------------------------------


def dispatch_phase(
    dispatch_ctx: PhaseDispatchContext,
    phase: PickPhase,
    ctx: PickContext,
) -> Tuple[bool, str]:
    """Mapea fase → service/action call. Devuelve ``(ok, reason)``.

    Las fases ``INITIAL_SNAPSHOT`` / ``HOME_INITIAL`` / ``DONE`` no
    disparan dispatch (el caller las filtra o las maneja como no-ops).
    """
    # Imports lazy para no exigir ROS en imports de módulo (tests puros).
    from ur5_panel_interfaces.action import PlanToPose
    from ur5_panel_interfaces.srv import (
        Attach as AttachSrv,
        Close as CloseSrv,
        Detach as DetachSrv,
        Open as OpenSrv,
    )

    common = dispatch_ctx.common_service_kwargs()

    if phase == PickPhase.SELECT_OBJECT:
        # B-iter1 (2026-05-03): SELECT_OBJECT ahora es INTERNAL-ONLY.
        # El object_name ya viaja en el goal de PickPlace (ctx.object_name) y
        # las fases siguientes (APPROACH/GRASP/etc) lo consumen directamente.
        # Anteriormente esta fase llamaba `/panel/select_object` para notificar
        # al panel UI, lo que creaba dependencia circular orchestrator↔panel
        # y bloqueaba el path orchestrator cuando el panel estaba ocupado.
        # Eliminada la llamada: el orchestrator es ahora INDEPENDIENTE del panel
        # para el flujo pick. La selección visual del panel sigue disponible
        # vía el botón Qt del panel (camino paralelo, no bloqueante).
        name = str(ctx.object_name or "").strip()
        if not name:
            return False, "select_object:empty_object_name_in_goal"
        return True, f"select_object:internal_ok:object={name}"

    if phase == PickPhase.INITIAL_SNAPSHOT:
        # B-iter2 (2026-05-03): INITIAL_SNAPSHOT explícitamente no-op a nivel
        # dispatch. El propósito (capturar TCP/joints/objeto al inicio del
        # ciclo) será implementado vía service ResolveObjectPoseWorld + TF
        # client en una iteración futura. Hoy: marker semántico para timing
        # y feedback estructurado al cliente.
        name = str(ctx.object_name or "").strip()
        return True, f"initial_snapshot:scaffold_ok:object={name or 'none'}"

    if phase == PickPhase.HOME_INITIAL:
        # B-iter2 (2026-05-03): HOME_INITIAL explícitamente no-op a nivel
        # dispatch. La transición real a pose HOME se delegará a un
        # JointTrajectoryAction directo (evitando la dependencia del
        # panel) en una iteración futura. Hoy: marker semántico.
        return True, "home_initial:scaffold_ok"

    if phase == PickPhase.APPROACH:
        # F5-step3: si no hay hint, intentar resolver pose del objeto vía
        # service ResolveObjectPoseWorld. Si el server no está, fallback
        # silencioso al placeholder (comportamiento F5-step2 previo).
        ctx_with_pose = ctx
        if is_no_hint(ctx.object_pose_world_hint) and ctx.object_name:
            resolved = try_resolve_object_pose_world(
                dispatch_ctx, ctx.object_name,
            )
            if resolved is not None:
                ctx_with_pose = PickContext(
                    object_name=ctx.object_name,
                    drop_xyz_world=ctx.drop_xyz_world,
                    object_pose_world_hint=resolved,
                    current_phase=ctx.current_phase,
                    detail=ctx.detail,
                    history=list(ctx.history),
                )
        goal = build_plan_to_pose_goal_for_approach(
            ctx_with_pose, z_clearance_m=dispatch_ctx.approach_z_clearance_m,
        )
        r = _call_plan_to_pose(dispatch_ctx, goal)
        return r.success, f"approach:{r.reason}"

    if phase == PickPhase.GRASP:
        # 1) Cerrar gripper.
        r_close = dispatch_ctx.service_caller(
            dispatch_ctx.node,
            CloseSrv,
            dispatch_ctx.service_map.gripper_close,
            CloseSrv.Request(),
            **common,
        )
        if not r_close.success:
            return False, f"grasp_close:{r_close.reason}"
        # 2) Attach lógico al objeto seleccionado.
        req_attach = AttachSrv.Request()
        req_attach.object_name = ctx.object_name
        r_att = dispatch_ctx.service_caller(
            dispatch_ctx.node,
            AttachSrv,
            dispatch_ctx.service_map.attach,
            req_attach,
            **common,
        )
        return r_att.success, f"grasp_attach:{r_att.reason}"

    if phase == PickPhase.LIFT:
        goal = build_plan_to_pose_goal_for_lift(ctx)
        r = _call_plan_to_pose(dispatch_ctx, goal)
        return r.success, f"lift:{r.reason}"

    if phase == PickPhase.TRANSPORT:
        goal = build_plan_to_pose_goal_for_transport(ctx)
        r = _call_plan_to_pose(dispatch_ctx, goal)
        return r.success, f"transport:{r.reason}"

    if phase == PickPhase.RELEASE:
        # 1) Detach.
        req_det = DetachSrv.Request()
        req_det.object_name = ctx.object_name
        r_det = dispatch_ctx.service_caller(
            dispatch_ctx.node,
            DetachSrv,
            dispatch_ctx.service_map.detach,
            req_det,
            **common,
        )
        if not r_det.success:
            return False, f"release_detach:{r_det.reason}"
        # 2) Abrir gripper.
        r_open = dispatch_ctx.service_caller(
            dispatch_ctx.node,
            OpenSrv,
            dispatch_ctx.service_map.gripper_open,
            OpenSrv.Request(),
            **common,
        )
        return r_open.success, f"release_open:{r_open.reason}"

    # Fase no esperada en este punto (INITIAL_SNAPSHOT/HOME_INITIAL/DONE).
    return True, f"{phase.value}_no_op"


def _call_plan_to_pose(
    dispatch_ctx: PhaseDispatchContext,
    goal: Any,
) -> ActionCallResult:
    """Wrapper para invocar PlanToPose.action."""
    from ur5_panel_interfaces.action import PlanToPose

    return dispatch_ctx.action_caller(
        dispatch_ctx.node,
        PlanToPose,
        dispatch_ctx.service_map.plan_to_pose_action,
        goal,
        **dispatch_ctx.common_action_kwargs(),
    )


def try_resolve_object_pose_world(
    dispatch_ctx: PhaseDispatchContext,
    object_name: str,
) -> Optional[tuple]:
    """F5-step3: intenta resolver pose world del objeto vía service.

    Llama a ``ResolveObjectPoseWorld`` con timeout corto. Si el server
    no aparece (no implementado todavía) o devuelve success=False, vuelve
    None — el caller debe usar fallback (placeholder).

    Devuelve tuple(7) ``(x, y, z, qx, qy, qz, qw)`` si OK, None si no.
    Importante: NO levanta excepciones — siempre devuelve None ante
    cualquier error, lo que mantiene el flujo de la fase APPROACH
    funcional aun sin el server real.
    """
    from ur5_panel_interfaces.srv import ResolveObjectPoseWorld

    req = ResolveObjectPoseWorld.Request()
    req.object_name = str(object_name or "").strip()
    if not req.object_name:
        return None

    # Timeout corto para no bloquear cuando el server no está disponible
    # — ese es el caso normal mientras F5-step4 (server real) no exista.
    discovery_short = float(dispatch_ctx.resolve_object_pose_discovery_timeout_sec)
    try:
        result = dispatch_ctx.service_caller(
            dispatch_ctx.node,
            ResolveObjectPoseWorld,
            dispatch_ctx.service_map.resolve_object_pose_world,
            req,
            discovery_timeout_sec=discovery_short,
            call_timeout_sec=dispatch_ctx.call_timeout_sec,
            client_cache=dispatch_ctx.client_cache,
        )
    except Exception:
        return None

    if not result.success:
        return None
    payload = result.payload
    if payload is None:
        return None
    return pose_msg_to_tuple7(getattr(payload, "pose_world", None))
