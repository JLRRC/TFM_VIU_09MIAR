# Base de Conocimiento Técnica — Sistema UR5 + RG2 Pick & Place

Fecha de generación: 2026-04-20

Documento generado automáticamente a partir del estado real del workspace, sin depender de servicios externos. El artefacto principal de esta ejecución es Markdown; el PDF queda opcional y desactivado por defecto.

## 1. Resumen Ejecutivo del Sistema

- Workspace inspeccionado: /home/laboratorio/TFM
- Launch principal revisado: agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py
- Orquestador del pick revisado: agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py
- Backend de attach/transporte revisado: agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_backend.py
- Estado actual confirmado: el pipeline separa el attach lógico del transporte físico. ATTACH_GATE puede aprobar y dejar el objeto en estado lógico CARRIED, pero la confirmación física solo llega cuando CARRY pasa.
- Artefacto principal generado por este script: 2026-04-20_base_conocimiento_tecnica_TFM.md

## 2. Fuentes Verificadas

### 2.1 Código fuente inspeccionado

- agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py
- agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py
- agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_attach_backend.py
- agarre_ros2_ws/src/ur5_description/urdf/ur5.urdf.xacro
- agarre_ros2_ws/models/ur5_rg2/model.sdf

### 2.2 Auditorías, histórico y reports usados

- auditoria/informe_fix_visual_grasp_20260419.md
- auditoria/spatial_20260418/directo_validation_20260418_201902/helper.log
- auditoria/spatial_20260419/directo_validation_20260419_164750/helper.log
- auditoria/spatial_20260419/directo_validation_20260419_184049/stack.log
- auditoria/spatial_20260419/directo_validation_20260419_164750/stack.log
- reports/evidence/ros2/moveit2_system_status.json
- reports/2026-04-18_base_conocimiento_tecnica_TFM.pdf

## 3. Arquitectura del Proyecto

- El launch del stack publica el entorno de pick directo y lanza el backend de attach cuando launch_attach_backend=true.
- El panel ejecuta la secuencia de grasp, cierre, attach, lift, carry, transporte a cesta y release.
- El backend implementa dos comportamientos conceptuales distintos:
  - follow_tcp como modo general de attach cinemático.
  - demo_transport para pick_demo, con rama world_locked activa por defecto.
- Según reports/evidence/ros2/moveit2_system_status.json, Mueve fisicamente objetos siguiendo rg2_tcp en modo follow_tcp cuando esta habilitado., lo que refuerza que follow_tcp sigue siendo la semántica base del backend fuera del caso pick_demo.

## 4. Frames y Offsets

- Offset semántico tool0 -> rg2_tcp en URDF: 0 0 0.175
- Offset semántico tool0 -> rg2_pinch_center en URDF: 0 0 0.175
- Pose actual de ur5_hand_joint en SDF: relative_to=wrist_3_link, pose=0 0.0823 0 1.570796325 0 1.570796325
- Lectura operativa:
  - La cadena TF semántica sitúa tanto rg2_tcp como rg2_pinch_center a +0.175 m de tool0.
  - El modelo SDF sigue describiendo la mano visible desde wrist_3_link mediante ur5_hand_joint; esto debe tratarse como geometría visual de Gazebo, no como definición del TCP semántico.
  - La consistencia exacta entre geometría visible y TCP semántico no se recalcula de nuevo dentro de este script; por tanto cualquier afirmación visual fina debe considerarse pendiente de validación runtime si se necesita precisión subcentimétrica.

## 5. Flujo Completo del Pick

El pipeline directo actual, consolidando panel y snapshots históricos, recorre estas fases lógicas:

1. Aproximación y descenso al grasp.
2. PRE_CLOSE y CLOSE de la pinza.
3. ATTACH_GATE para aceptar el attach lógico con chequeo geométrico y ventana temporal.
4. LIFT corto post-grasp.
5. CARRY para validar movimiento físico observable del objeto.
6. HOME_WITH_OBJECT y transporte hacia cesta si el carry fue válido.
7. RELEASE y retorno final.

El punto crítico no es el attach lógico, sino la transición ATTACH_GATE -> CARRY.

## 6. Variables de Entorno y Parámetros Críticos

### 6.1 ATTACH_GATE por defecto en el launch

- PANEL_PICK_DEMO_ATTACH_XY_TOL_M = 0.020
- PANEL_PICK_DEMO_ATTACH_Z_TOL_M = 0.010
- PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M = 0.040
- PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M = 0.012
- PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC = 0.35
- PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES = 5
- PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M = 0.020
- PANEL_PICK_DEMO_GRIPPER_CLOSED_OPENING_THR_M = 0.020
- PANEL_PICK_DEMO_ATTACH_SETTLE_SEC en panel = 1.8
- PANEL_PICK_DEMO_POST_ATTACH_HOLD_SEC en panel = 0.90

### 6.2 Backend de attach/transporte

- attach_backend_mode por defecto en el launch = follow_tcp
- ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS por defecto = pick_demo
- attach_backend_max_pose_age_sec por defecto = 1.5
- attach_backend_follow_rate_hz por defecto = 20.0
- attach_backend_max_dist_m por defecto = 0.08
- El código del backend marca pick_demo como demo_transport y, al activarlo, fija use_world_locked_pose=True.
- Esto significa que pick_demo no sigue la misma ruta que el resto de objetos cuando entra por demo_transport; el modo nominal del backend sigue siendo follow_tcp, pero el objeto demo entra por world_locked salvo reconfiguración explícita.

## 7. Nueva Sección Obligatoria — Validación Física Post-Grasp / Carry

### 7.1 ATTACH_GATE correcto vs carry_validation fallido

- ATTACH_GATE correcto no equivale a grasp físico confirmado.
- En el código actual, tras ATTACH_GATE se deja follow_confirmed=false y se registra la nota follow_confirmed_only_after_carry. Estado de esta afirmación en fuente: confirmado.
- La confirmación física solo se marca cuando _validate_demo_carry(...) retorna OK durante CARRY.
- Por tanto:
  - ATTACH_GATE correcto = attach lógico aceptado, objeto publicado/kinemático disponible, proximidad validada en ventana temporal.
  - carry_validation fallido = el objeto no demuestra transporte físico suficiente con respecto al TCP y/o a la elevación esperada.

### 7.2 Timeout específico y thresholds activos en post_grasp_lift

#### Llamada real activa en panel_pick_demo.py para phase=post_grasp_lift

- timeout_sec = 3.0
- min_obj_move_m = 0.020
- min_lift_delta_m = 0.025
- max_tcp_dist_m = 0.120
- live_world_fn = _fresh_gazebo_object_world
- Espera previa de asentamiento del carry: PANEL_PICK_DEMO_CARRY_SETTLE_SEC = 3.0

#### Llamada real activa en panel_pick_demo.py para phase=home_with_object

- timeout_sec = 1.2
- min_obj_move_m = 0.080
- min_lift_delta_m = 0.060
- max_tcp_dist_m = env PANEL_PICK_DEMO_CARRY_HOME_MAX_TCP_DIST_M, default 0.200

#### Discrepancias internas todavía presentes en el código

- Metadata de _phase_begin("CARRY"): min_obj_move_m=0.030, min_lift_delta_m=0.060, max_tcp_dist_m=0.080
- Llamada real a _validate_demo_carry para post_grasp_lift: min_obj_move_m=0.020, min_lift_delta_m=0.025, max_tcp_dist_m=0.120
- FINAL_TRACE de inicio de CARRY usa timeout=3.00
- FINAL_TRACE de cierre de CARRY sigue codificando timeout=1.60
- Conclusión: hoy existen discrepancias entre metadata de fase, comentarios adyacentes, llamada efectiva y timeout trazado en cierre.

### 7.3 Modos de transporte y seguimiento

- follow_tcp:
  - Es el modo base del backend por launch.
  - Sigue el TCP con offset relativo para objetos que no entran por demo_transport.
  - También es la semántica que describe reports/evidence/ros2/moveit2_system_status.json.
- world_locked:
  - Se activa para pick_demo porque ATTACH_BACKEND_DEMO_TRANSPORT_OBJECTS incluye pick_demo y _activate_demo_transport_attachment fija use_world_locked_pose=True.
  - En esta rama el backend calcula desired = tcp + world_offset fijo y mantiene la orientación world_q* almacenada en el attach.
  - Evidencia literal:

    ```text
    [gripper_attach_backend-11] [INFO] [1776610345.928557082] [gripper_attach_backend]: [ATTACH_BACKEND] demo_transport_follow_tick object=pick_demo mode=world_locked desired=(0.483,0.142,0.680) tcp=(0.493,0.141,0.695)
    ```

### 7.4 Efecto de world_locked cuando el objeto no sigue realmente al TCP

- Si la pose del TCP llega vieja o el backend no actualiza la pose del objeto a tiempo, world_locked sigue publicando ticks sobre una referencia retrasada.
- En ese escenario puede verse attach lógico correcto pero carry físico fallido.
- El efecto práctico observado es uno de estos dos:
  - El objeto nunca abandona la mesa: best_obj_move=0.000 y best_lift_delta=0.000.
  - El objeto sí cambia de pose, pero no acompaña el lift del TCP: best_lift_delta<0 y best_tcp_dist crece por encima del máximo.

### 7.5 Impacto de stale_tcp_pose_soft_follow y relación con latencia/ventana de validación

- El backend emite stale_tcp_pose_soft_follow cuando tcp_age supera attach_backend_max_pose_age_sec=1.5, pero aún no llega al hard_age calculado en el propio backend.
- Mientras no se alcance el hard_age, la ruta de soft-follow sigue intentando mover el objeto con una pose de TCP envejecida.
- Evidencia literal:

    ```text
    [gripper_attach_backend-11] [WARN] [1776617123.692018990] [gripper_attach_backend]: [ATTACH_BACKEND] stale_tcp_pose_soft_follow age=2.824s max=1.500s hard=4.500s src=base_chain base_chain_ok=True base_chain_age=2.824 world_tcp_ok=True world_tcp_age=2.832 cache_ok=None cache_age=None tool_fallback_ok=None tool_fallback_age=None world_base_ok=True base_tcp_ok=True
    ```

- La propia lógica del panel añade una espera previa CARRY_SETTLE de 3.0 s y comenta que el backend puede actualizar a ritmo efectivo cercano a 1 Hz en headless, a pesar de que el parámetro nominal attach_backend_follow_rate_hz está en 20.0 Hz.
- En los logs de world_locked inspeccionados, la separación observada entre dos ticks consecutivos es 2.444 s entre dos ticks consecutivos del mismo log, lo que refuerza que la latencia efectiva del transporte puede dominar la ventana de validación física.
- Relación operativa confirmada:
  - TCP fresco -> menor riesgo de falso stale y de arrastrar una referencia retrasada.
  - Backend lento o stale -> mayor probabilidad de que CARRY evalúe la pose del objeto antes de que world_locked/follow_tcp haya reflejado el lift real.
  - Ventana de validación corta + TCP viejo -> más fallos del tipo object_not_updated o carry_follow_lost.

### 7.6 Discrepancia entre timeout configurado y timeout observado en logs

- Timeout real pasado a _validate_demo_carry en post_grasp_lift: 3.0 s.
- Timeout trazado al inicio de CARRY: 3.00 s.
- Timeout trazado al cierre de CARRY en el código: 1.60 s.
- En helper.log históricos recientes reaparece ese cierre con timeout=1.60 incluso cuando el código actual llama con 3.0 s.
- Esto debe tratarse como inconsistencia abierta de telemetría, no como un simple detalle cosmético, porque dificulta correlacionar código y ejecución real.

### 7.7 Criterios de diagnóstico solicitados

- Si best_obj_move < umbral:
  - Diagnóstico primario: el objeto no se ha movido lo suficiente desde su pose inicial.
  - Caso extremo: object_not_updated, típico cuando el objeto permanece sobre la mesa.
  - Evidencia:

    ```text
    [2026-04-18T20:23:30] [PICK][DIRECT][FINAL_TRACE] phase=CARRY event=wait_done expected=carry_validation_ok received=false timeout=1.60 request=none logical=CARRIED physical=none reason=demo_carry_validation_failed phase=post_grasp_lift best_obj_move=0.000 best_lift_delta=0.000 best_tcp_dist=0.098 fail_reasons=obj_move_below_min,lift_delta_below_min,tcp_dist_above_max last_obj_world=(-0.420,0.000,0.875) last_obj_base=(0.430,0.000,0.025) last_tcp_base=(0.445,0.002,0.144)
    ```

  - Evidencia complementaria:

    ```text
    [2026-04-19T18:24:30] [PICK][DIRECT][PHYSICS] phase=post_grasp_lift carry_follow_lost=true detail=object_never_moved best_obj_move=0.000 hint=check_demo_transport_set_pose_ok_in_backend_logs last_obj_world=(-0.420,0.000,0.875)
    ```

- Si best_lift_delta < 0:
  - Diagnóstico primario: el objeto cambió de pose, pero no acompañó el lift del TCP; la elevación respecto a la referencia inicial fue negativa.
  - Suele venir combinado con carry_follow_lost y/o tcp_dist_above_max.
  - Evidencia:

    ```text
    [2026-04-19T16:52:50] [PICK][DIRECT][FINAL_TRACE] phase=CARRY event=wait_done expected=carry_validation_ok received=false timeout=1.60 request=none logical=CARRIED physical=none reason=demo_carry_validation_failed phase=post_grasp_lift best_obj_move=0.870 best_lift_delta=-0.194 best_tcp_dist=0.881 fail_reasons=lift_delta_below_min,tcp_dist_above_max carry_detail=carry_follow_lost last_obj_world=(0.396,0.222,0.681) last_obj_base=(1.246,0.222,-0.169) last_tcp_base=(0.440,-0.001,0.125)
    ```

- Si best_tcp_dist > máximo:
  - Diagnóstico primario: el objeto quedó demasiado lejos del TCP durante el tramo que debería demostrar transporte físico coherente.
  - Esto invalida el carry incluso si hubo movimiento del objeto, porque el movimiento no es coherente con un grasp estable.

### 7.8 Separación conceptual pedida

- CAPA 1 = geometría / unión visual
  - Incluye URDF, SDF, meshes, offsets visuales y consistencia tool0/TCP/gripper visible.
  - Puede dar una pinza visualmente razonable aunque todavía no exista prueba física de transporte.
- CAPA 2 = attach / carry / seguimiento TCP / validación física
  - Incluye ATTACH_GATE, attach backend, demo_transport/follow_tcp, stale_tcp_pose_soft_follow y _validate_demo_carry.
  - Es la capa que decide si el objeto fue realmente transportado con el TCP o si solo hubo attach lógico/kinemático.

## 8. Evidencia Cruzada desde Auditoría e Histórico

### 8.1 Síntesis en auditoría

```text
[PICK] ✗ Error: demo_carry_validation_failed
  phase=post_grasp_lift
  best_obj_move=0.000
  best_lift_delta=0.000
  best_tcp_dist=0.113
```

### 8.2 Patrones confirmados en logs

- Patrón A: objeto inmóvil tras lift lógico.
- Patrón B: objeto con movimiento pero sin lift coherente y alejamiento respecto al TCP.
- Patrón C: warnings stale_tcp_pose_soft_follow intercalados con world_locked, señal de que la frescura de la pose del TCP influye directamente en el carry observado.

## 9. Topics y Semántica de Attach

- El backend publica y consume topics con el patrón /gripper/<objeto>/attach, /gripper/<objeto>/detach y /gripper/<objeto>/state.
- La decisión de ruta del attach distingue entre demo_transport, tool_anchor y follow_tcp.
- Para pick_demo, la configuración actual del launch lo encamina por demo_transport.

## 10. Estado Actual del Sistema

- Confirmado en código:
  - Markdown debe ser el artefacto principal del script de base de conocimiento.
  - ATTACH_GATE y CARRY representan capas distintas y no deben confundirse.
  - pick_demo entra por demo_transport y usa world_locked por defecto.
  - Hay incoherencias internas entre la telemetría y la llamada real de carry validation.
- Confirmado en logs/auditoría:
  - Existen fallos repetidos de post_grasp_lift con objeto inmóvil.
  - Existen fallos repetidos con best_lift_delta negativo y tcp_dist_above_max.
  - stale_tcp_pose_soft_follow aparece en histórico reciente con edades superiores a max_pose_age_sec.
- Pendiente de validación adicional si se necesita cierre definitivo:
  - Medida estadística completa de latencia backend por campaña, no solo muestras puntuales.
  - Revalidación visual actual fina de la geometría visible RG2 frente al TCP semántico en la rama vigente.

## 11. Riesgos Abiertos

- Riesgo de interpretación errónea si se usan los valores de metadata de CARRY en vez de la llamada real a _validate_demo_carry.
- Riesgo de telemetría inconsistente mientras FINAL_TRACE siga cerrando con timeout=1.60 para CARRY.
- Riesgo de falsos diagnósticos si se observa solo ATTACH_GATE y no se revisa la validación física posterior.
- Riesgo de degradación por frescura insuficiente del TCP cuando aparecen warnings stale_tcp_pose_soft_follow.

## 12. Apéndice de Fuentes Usadas

El índice completo de fuentes usadas por este generador queda en:

- reports/BaseDeConocimiento/.tmp_base_conocimiento_2026-04-20/fuentes_verificadas.txt
- reports/BaseDeConocimiento/.tmp_base_conocimiento_2026-04-20/generacion.log
