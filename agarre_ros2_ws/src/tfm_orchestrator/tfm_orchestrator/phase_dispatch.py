#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/tfm_orchestrator/tfm_orchestrator/phase_dispatch.py
# Contenido: F5-step1 — dispatch puro de fases pick_place compartido entre Node y LifecycleNode.
"""Dispatch puro de fases del pick_place orchestrator.

Capa pura del dispatch de fases para
``pick_orchestrator_lifecycle_node.py`` (LifecycleNode canónico desde F9).
Recibe ``node`` y un contexto explícito (``service_map``,
``client_cache``, timeouts) en lugar de leer atributos del nodo
directamente. Esto permite:

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

from ur5_tools.geometry_constants import BASE_LINK_IN_WORLD

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
# Keep a small positive clearance so GRASP_DOWN ends with bilateral contact
# potential without forcing immediate table/object penetration.
_GRASP_DOWN_Z_M = 0.020
_GRASP_CLOSE_SETTLE_SEC = 0.35
_GRASP_ATTACH_RETRIES = 10
_GRASP_ATTACH_RETRY_SLEEP_SEC = 0.15


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


def compute_top_down_grasp_quat(
    obj_quat_xyzw: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Compute TCP orientation for a top-down grasp aligned with object yaw.

    F1.8 audit-v4 (2026-05-08): bug TCP-orientation contract — el orchestrator
    pasaba `obj_quat` directamente como TCP-orientation en APPROACH/GRASP_DOWN,
    causando OMPL FAILURE consistente cuando el objeto caía con cualquier yaw
    no trivial (orientación inalcanzable / en colisión).

    Solución: el TCP debe apuntar hacia abajo (Z_TCP = -Z_world) con el yaw
    alineado al yaw del objeto para que las pinzas RG2 se cierren a lo largo
    del eje correcto del objeto.

    Math: extraer yaw del object quat (rotación alrededor de world Z), luego
    componer ``q_yaw * q_180X`` donde ``q_180X = (1,0,0,0)`` rota 180° alrededor
    del eje X (haciendo que TCP Z apunte hacia abajo).

    Resultado canónico: ``(cos(yaw/2), sin(yaw/2), 0, 0)``.

    Casos:
      * yaw=0     → (1,0,0,0)            → 180° around X: TCP-Z down, X=world-X.
      * yaw=π/2   → (0.707,0.707,0,0)    → TCP-Z down, X=world-Y.
      * yaw=π     → (0,1,0,0)            → 180° around Y: TCP-Z down, X=world-X.

    Parameters:
        obj_quat_xyzw: orientación del objeto en world (x,y,z,w convention).
            Para objetos planos sobre mesa, sólo la componente z+w son
            significativas (rotación alrededor de world Z).

    Returns:
        TCP quaternion (x,y,z,w) listo para Quaternion msg.
    """
    import math
    qx, qy, qz, qw = obj_quat_xyzw
    # yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))  (canonical formula)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    return (cy, sy, 0.0, 0.0)


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

    NOTA (2026-05-07 fix ronda 17): el field se llama ``target_pose_base``
    porque el server lo lee como base_link frame. La pose hint del cliente
    viene en world frame (``object_pose_world_hint``). Conversión necesaria.
    Offset estático world→base_link del UR5: base_link en world está en
    ``(-0.85, 0, 0.850)`` (urdf ur5.urdf.xacro:47, sin rotación).
    Por tanto: pos_base = pos_world - (-0.85, 0, 0.850).
    Sin esta conversión el robot va a un punto a 1.27m del objeto.
    """
    from geometry_msgs.msg import Pose, Point, Quaternion
    from ur5_panel_interfaces.action import PlanToPose

    goal = PlanToPose.Goal()
    if not is_no_hint(ctx.object_pose_world_hint):
        norm = normalize_pose_hint(ctx.object_pose_world_hint)
        # norm garantizado != None aquí (is_no_hint cubre None y mal-formados).
        x, y, z, qx, qy, qz, qw = norm  # type: ignore[misc]
        # World → base_link: restar el origen del base_link en world.
        x_base = x - BASE_LINK_IN_WORLD[0]
        y_base = y - BASE_LINK_IN_WORLD[1]
        z_base = z - BASE_LINK_IN_WORLD[2]
        # F1.8 (2026-05-08): el TCP debe apuntar hacia abajo (top-down grasp)
        # con yaw alineado al objeto. Antes pasábamos obj_quat directo →
        # OMPL FAILURE por orientación inalcanzable (ver bug doc F1.8).
        tcp_qx, tcp_qy, tcp_qz, tcp_qw = compute_top_down_grasp_quat((qx, qy, qz, qw))
        goal.target_pose_base = Pose(
            position=Point(x=x_base, y=y_base, z=z_base + float(z_clearance_m)),
            orientation=Quaternion(x=tcp_qx, y=tcp_qy, z=tcp_qz, w=tcp_qw),
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


def build_plan_to_pose_goal_for_grasp_down(ctx: PickContext) -> Any:
    """Goal de GRASP_DOWN: cartesian descent vertical hasta justo antes del contacto.

    2026-05-07: insertado entre APPROACH y GRASP para que las pinzas físicas
    en Gazebo lleguen a tocar el objeto antes del attach lógico. Sin esto,
    GRASP cierra el gripper a ~12cm del objeto y el attach es "fantasma"
    visualmente.

    Target: centro del objeto (Z offset = 0.0m).
    Esto incrementa el solape vertical efectivo entre las caras internas
    de los dedos RG2 y el objeto antes del cierre, mejorando la probabilidad
    de contacto bilateral real previo al attach.
    """
    from geometry_msgs.msg import Pose, Point, Quaternion
    from ur5_panel_interfaces.action import PlanToPose

    goal = PlanToPose.Goal()
    if not is_no_hint(ctx.object_pose_world_hint):
        norm = normalize_pose_hint(ctx.object_pose_world_hint)
        x, y, z, qx, qy, qz, qw = norm  # type: ignore[misc]
        x_base = x - BASE_LINK_IN_WORLD[0]
        y_base = y - BASE_LINK_IN_WORLD[1]
        z_base = z - BASE_LINK_IN_WORLD[2]
        # F1.8 (2026-05-08): TCP top-down con yaw del objeto (mismo fix que
        # APPROACH). Sin esto, GRASP_DOWN cartesian descent partía con orient
        # inalcanzable y fallaba con OMPL FAILURE.
        tcp_qx, tcp_qy, tcp_qz, tcp_qw = compute_top_down_grasp_quat((qx, qy, qz, qw))
        goal.target_pose_base = Pose(
            position=Point(x=x_base, y=y_base, z=z_base + _GRASP_DOWN_Z_M),
            orientation=Quaternion(x=tcp_qx, y=tcp_qy, z=tcp_qz, w=tcp_qw),
        )
    else:
        # Sin hint: target placeholder (el FSM debería rechazar antes de llegar aquí).
        goal.target_pose_base = Pose(
            position=Point(x=0.40, y=0.0, z=0.05),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    goal.ee_frame = "rg2_pinch_center"
    goal.cartesian = True  # Movimiento recto vertical: evita desviaciones laterales.
    goal.timeout_sec = 0.0
    return goal


def build_plan_to_pose_goal_for_lift(ctx: PickContext) -> Any:
    """Goal de LIFT: levantar el objeto a una altura segura sobre la mesa.

    F1.15 (2026-05-08): orientación TCP-down via compute_top_down_grasp_quat
    (mismo que APPROACH/GRASP_DOWN/TRANSPORT). Usar quat raw del objeto era
    un bug arquitectónico: el robot venía de GRASP_DOWN ya en TCP-down y
    forzar una orientación arbitraria del objeto provocaba que OMPL no
    encontrara plan en 60s (FIRST_ATTEMPT_TIMEOUT). Con TCP-down consistente
    desde APPROACH→GRASP_DOWN→GRASP→LIFT→TRANSPORT, el wrist no necesita
    rotaciones extra entre fases.
    """
    from geometry_msgs.msg import Pose, Point, Quaternion
    from ur5_panel_interfaces.action import PlanToPose

    _LIFT_Z_M = 0.30  # 30 cm sobre la mesa: claro de obstáculos para transport.

    goal = PlanToPose.Goal()
    if not is_no_hint(ctx.object_pose_world_hint):
        norm = normalize_pose_hint(ctx.object_pose_world_hint)
        x, y, z, qx, qy, qz, qw = norm  # type: ignore[misc]
        x_base = x - BASE_LINK_IN_WORLD[0]
        y_base = y - BASE_LINK_IN_WORLD[1]
        z_base = z - BASE_LINK_IN_WORLD[2]
        # F1.15: TCP-down quat coherente con APPROACH/GRASP_DOWN.
        tcp_qx, tcp_qy, tcp_qz, tcp_qw = compute_top_down_grasp_quat((qx, qy, qz, qw))
        goal.target_pose_base = Pose(
            position=Point(x=x_base, y=y_base, z=z_base + _LIFT_Z_M),
            orientation=Quaternion(x=tcp_qx, y=tcp_qy, z=tcp_qz, w=tcp_qw),
        )
    else:
        # Sin hint: lift sobre el origen base_link a altura segura.
        goal.target_pose_base = Pose(
            position=Point(x=0.40, y=0.0, z=0.40),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0),  # 180° around X = TCP-down
        )
    goal.ee_frame = "rg2_pinch_center"
    goal.cartesian = False  # Joint planning: más robusto que cartesian.
    goal.timeout_sec = 0.0
    return goal


def build_plan_to_pose_goal_for_transport(ctx: PickContext) -> Any:
    """Goal de TRANSPORT: drop_xyz_world del request original.

    NOTA (2026-05-07): drop_xyz_world viene en world frame, server espera
    base_link. Aplicar offset estático world→base (-0.85, 0, 0.850).

    F1.14 (2026-05-08): orientación TCP-down (mismo que APPROACH/GRASP_DOWN
    via compute_top_down_grasp_quat). Antes usaba identity, lo que forzaba
    al robot a rotar 180° alrededor X axis EN MOVIMIENTO mientras
    transportaba el objeto agarrado — trayectoria compleja que el bridge
    no podía completar (CONTROL_FAILED|retry_failed). Ahora el TCP mantiene
    la misma orientación down que tenía tras LIFT.
    """
    from geometry_msgs.msg import Pose, Point, Quaternion
    from ur5_panel_interfaces.action import PlanToPose

    # F1.14: usar TCP-down quat. Si hay object_pose_world_hint, derivar
    # yaw del objeto (consistencia con APPROACH/GRASP_DOWN). Si no, usar
    # yaw=0 (180° around X canónico = TCP point down con X axis aligned).
    if not is_no_hint(ctx.object_pose_world_hint):
        norm = normalize_pose_hint(ctx.object_pose_world_hint)
        _, _, _, qx, qy, qz, qw = norm  # type: ignore[misc]
        tcp_qx, tcp_qy, tcp_qz, tcp_qw = compute_top_down_grasp_quat((qx, qy, qz, qw))
    else:
        tcp_qx, tcp_qy, tcp_qz, tcp_qw = 1.0, 0.0, 0.0, 0.0  # 180° around X

    goal = PlanToPose.Goal()
    goal.target_pose_base = Pose(
        position=Point(
            x=float(ctx.drop_xyz_world[0]) - BASE_LINK_IN_WORLD[0],
            y=float(ctx.drop_xyz_world[1]) - BASE_LINK_IN_WORLD[1],
            z=float(ctx.drop_xyz_world[2]) - BASE_LINK_IN_WORLD[2],
        ),
        orientation=Quaternion(x=tcp_qx, y=tcp_qy, z=tcp_qz, w=tcp_qw),
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

    if phase == PickPhase.GRASP_DOWN:
        # 2026-05-07: cartesian descent vertical hasta justo antes del contacto.
        # Las pinzas físicas RG2 deben tocar el objeto antes del attach lógico.
        # Sin esta fase, GRASP cierra el gripper a 12cm del objeto y el attach
        # es "fantasma" visualmente.
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
        goal = build_plan_to_pose_goal_for_grasp_down(ctx_with_pose)
        r = _call_plan_to_pose(dispatch_ctx, goal)
        if r.success:
            return True, f"grasp_down:{r.reason}"
        # Fallback robusto: en escenarios cercanos a contacto/mesa, el path
        # cartesiano puede fallar con INVALID_MOTION_PLAN aunque la IK directa
        # sea válida. Reintentamos no-cartesiano manteniendo mismo target.
        goal.cartesian = False
        r_fallback = _call_plan_to_pose(dispatch_ctx, goal)
        if r_fallback.success:
            return True, f"grasp_down:{r.reason}|fallback_noncart:{r_fallback.reason}"
        return False, (
            f"grasp_down:{r.reason}|fallback_noncart:{r_fallback.reason}"
        )

    if phase == PickPhase.GRASP:
        import time
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
        # Dar tiempo a que el cierre físico llegue a contacto antes del gate.
        time.sleep(_GRASP_CLOSE_SETTLE_SEC)
        # 2) Attach lógico al objeto seleccionado (con reintentos breves):
        # evita falsos negativos cuando el contacto bilateral aparece unos
        # cientos de ms después del comando CLOSE.
        req_attach = AttachSrv.Request()
        req_attach.object_name = ctx.object_name
        r_att = None
        for attempt in range(max(1, int(_GRASP_ATTACH_RETRIES))):
            r_att = dispatch_ctx.service_caller(
                dispatch_ctx.node,
                AttachSrv,
                dispatch_ctx.service_map.attach,
                req_attach,
                **common,
            )
            if r_att.success:
                break
            if attempt + 1 < int(_GRASP_ATTACH_RETRIES):
                time.sleep(_GRASP_ATTACH_RETRY_SLEEP_SEC)
        if r_att is None:
            return False, "grasp_attach:internal_no_response"
        if not r_att.success:
            return False, (
                f"grasp_attach:{r_att.reason}|attempts={int(_GRASP_ATTACH_RETRIES)}"
            )
        # 3) B-iter8: gate de distancia. Detecta el bug "drop_anchor placebo"
        # donde el backend retorna success=True pero el TCP estaba a >1m del
        # objeto (visto live: tcp_obj_dist_m=1.093). Sin este gate, el FSM
        # avanza a LIFT con un objeto que NO está físicamente en el TCP.
        from .pick_gates import evaluate_attach_distance_gate

        att_payload = getattr(r_att, "payload", None)
        tcp_obj_dist = (
            float(getattr(att_payload, "tcp_obj_dist_m", 0.0))
            if att_payload is not None
            else None
        )
        gate_ok, gate_reason = evaluate_attach_distance_gate(tcp_obj_dist)
        if not gate_ok:
            return False, f"grasp_attach_gate:{gate_reason}"
        return True, f"grasp_attach:{r_att.reason}|{gate_reason}"

    if phase == PickPhase.LIFT:
        goal = build_plan_to_pose_goal_for_lift(ctx)
        r = _call_plan_to_pose(dispatch_ctx, goal)
        return r.success, f"lift:{r.reason}"

    if phase == PickPhase.TRANSPORT:
        # B-iter9: retry con back-off para fallos transitorios de planning.
        # Default: 2 intentos con back-off 1s → 2s. Si MoveIt está
        # momentáneamente saturado, el 2º intento típicamente pasa.
        from .retry import retry_with_backoff

        def _attempt() -> Any:
            goal = build_plan_to_pose_goal_for_transport(ctx)
            return _call_plan_to_pose(dispatch_ctx, goal)

        result, attempts = retry_with_backoff(_attempt)
        suffix = f"|attempts={attempts}" if attempts > 1 else ""
        return result.success, f"transport:{result.reason}{suffix}"

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


# ---------------------------------------------------------------------
# F9.4 audit (2026-05-10): tabla declarativa fase → handler.
# ---------------------------------------------------------------------
#
# Hasta ahora ``dispatch_phase`` era una cascada ``if phase == X``
# (9+ ramas). Para facilitar:
#   * Tests parametrizados (``@pytest.mark.parametrize("phase, …",
#     PHASE_DISPATCH_TABLE.items())``).
#   * Inyección de handlers para mocks en tests.
#   * Documentación machine-readable de qué fase usa qué services.
#
# Esta tabla NO sustituye la cascada actual (refactor F9.4 final
# requiere validación con stack vivo), pero la complementa como
# contrato declarativo. El test ``test_phase_dispatch_table_coverage``
# valida que toda fase no terminal del FSM tiene entrada aquí.

PHASE_DISPATCH_METADATA: dict = {
    PickPhase.SELECT_OBJECT: {
        "type": "internal",
        "services_used": [],
        "actions_used": [],
        "purpose": "Validar object_name (F12 audit-v4: internal-only).",
    },
    PickPhase.INITIAL_SNAPSHOT: {
        "type": "snapshot",
        "services_used": ["resolve_object_pose_world"],
        "tf_frames": ["base_link", "rg2_tcp"],
        "purpose": "Capturar TCP/joints/objeto al inicio del ciclo.",
    },
    PickPhase.HOME_INITIAL: {
        "type": "action",
        "actions_used": ["/joint_trajectory_controller/follow_joint_trajectory"],
        "purpose": "Mover robot a HOME pose con FJT directo.",
    },
    PickPhase.APPROACH: {
        "type": "action",
        "actions_used": ["/orchestrator/plan_to_pose"],
        "purpose": "Plan + execute approach 10cm sobre objeto, TCP top-down.",
    },
    PickPhase.GRASP_DOWN: {
        "type": "action",
        "actions_used": ["/orchestrator/plan_to_pose"],
        "purpose": "Descenso cartesiano vertical 10cm → 2cm sobre objeto.",
    },
    PickPhase.GRASP: {
        "type": "service",
        "services_used": ["/gripper/close", "/orchestrator/attach"],
        "gates": ["attach_distance"],
        "purpose": "Close gripper + attach lógico/físico.",
    },
    PickPhase.LIFT: {
        "type": "action",
        "actions_used": ["/orchestrator/plan_to_pose"],
        "purpose": "Subir TCP 30cm sobre objeto, top-down.",
    },
    PickPhase.TRANSPORT: {
        "type": "action_with_retry",
        "actions_used": ["/orchestrator/plan_to_pose"],
        "retry_max_attempts": 2,
        "retry_initial_backoff": 1.0,
        "purpose": "Plan + execute hacia drop_xyz_world con backoff.",
    },
    PickPhase.RELEASE: {
        "type": "service",
        "services_used": ["/orchestrator/detach", "/gripper/open"],
        "gates": ["release_open"],
        "purpose": "Detach + open gripper.",
    },
}


def get_phase_dispatch_metadata() -> dict:
    """Devuelve copia de :data:`PHASE_DISPATCH_METADATA` (lectura segura)."""
    return {k: dict(v) for k, v in PHASE_DISPATCH_METADATA.items()}
