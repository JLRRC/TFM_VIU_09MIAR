# Validación Automática DIRECTO — 2026-04-20T15:53–15:57

## Contexto

- **Workspace**: `/home/laboratorio/TFM/agarre_ros2_ws`
- **Run ID**: `auto_validation/20260420_161037`
- **Corrida analizada**: `stack_manual_20260420_155332.log` (stack iniciado a las 15:53:32)
- **Trigger de pick**: `[BTN] PICK MESA → CESTA` a las 15:53:59
- **Inicio de ejecución de fases**: 15:55:08 (tras warm-up del sistema)
- **Fin observado**: 15:57:18 (SHUTDOWN externo del proceso)
- **Robot**: UR5 + RG2
- **TCP frame**: `rg2_pinch_center`
- **Baseline geométrico**: ur5_hand_joint Y=0.0823, rg2_tcp_joint Z=0.175, offset=0.175m

---

## Configuración de parámetros clave (extraída del launch file)

| Parámetro | Valor |
|-----------|-------|
| GRASP_CONTACT_Z_OFFSET_M | 0.0 |
| PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M | 0.035 |
| PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M | 0.020 |
| PANEL_PICK_DEMO_PRE_CLOSE_Z_ERR_TOL_M | 0.010 |
| PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC | 3.0 (120 en run_directo_validation.sh) |
| PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM | 0.01 |
| PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M | 0.040 |
| PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M | 0.012 |
| PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC | 0.35 |
| PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES | 5 |
| PANEL_PICK_DEMO_ATTACH_XY_TOL_M | 0.020 |
| PANEL_PICK_DEMO_DIRECT_IK_TCP_OFFSET_M | 0.175 |
| CARRY_SETTLE_SEC | 3.0 (baseline) |
| min_lift_delta | (evaluado en LIFT gate) |

---

## Comandos ejecutados durante esta validación

```bash
# Verificación de estructura del workspace
ls /home/laboratorio/TFM/agarre_ros2_ws/src/
ros2 control list_controllers
ros2 topic list
ros2 node list
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link rg2_pinch_center
printenv | grep -E "PANEL_|GRASP_|ATTACH_|CARRY_|MOVEIT_|ROS_"
```

## Estado del sistema en el momento de validación

- **Gazebo**: SIN INSTANCIA PROPIA (stack iniciado previamente por usuario)
- **Nodos activos**: controller_bootstrap, gripper_attach_backend (x2), moveit, planning_scene_sync, system_state_manager, ur5_moveit_bridge, ur5_moveit_py
- **controller_manager service**: NO disponible (stack externo recién cerrado al iniciar análisis)
- **Topics ROS activos**: /clock, /gripper/pick_demo/attach, /drop_anchor/*, /joint_states (inferido)
- **GZ_PARTITION**: ur5pro_manual_1776693212

---

## Análisis por Fases

### HOME
- **Estado**: No hay registro explícito de HOME inicial. El robot arranca en posición home por configuración del launch.
- **Veredicto**: INCONCLUSIVE (sin log explícito de HOME inicial, pero el panel inicia movimiento desde posición válida)

### APPROACH_COARSE
- **Inicio**: 15:55:08
- **Fin**: 15:55:46 (~38 segundos incluyendo warm-up IK)
- **Target**: base_link (0.436, -0.000, 0.060) = objeto + extra_z 0.035m
- **Resultado TCP**: (0.437, 0.006, 0.049) — dist al target = 0.014m
- **Gate check**: xy_err=0.007m/tol=0.012m ✓, z_err=-0.011m/tol=0.012m ✓
- **IK**: retry con seed diverso → err_norm=0.135, pos_err=0.007m, success=true
- **Veredicto**: **PASS** — result=OK, gate satisfied
- **Nota**: PHASE_CHECK aprobado con xy_ok=true, z_ok=true

### GRASP_DOWN (GRASP_DOWN_JOINT)
- **Inicio**: 15:55:46
- **Fin**: 15:56:09 (~23 segundos)
- **Cartesian path**: FALLIDO (cartesian_low_fraction=0.000) → fallback IK
- **Fallback IK**: strategy=permissive_z_only_descent, z_gap=0.025m
- **Target**: base_link (0.439, 0.005, 0.025) — sobre el objeto
- **Resultado TCP**: (0.443, 0.001, 0.040) — dist=0.016m/tol=0.020m
- **Gate check**: xy_err=0.008m/tol=0.015m ✓, z_err=0.015m/tol=0.025m ✓
- **Veredicto**: **PASS** — result=OK
- **Nota**: Cartesian path failed, IK fallback exitoso. Mismatch DH/SDF ≈13mm evidente en z_err=15mm.

### GRASP_ALIGN_IK
- **Inicio**: 15:56:09
- **Intentos**: 2 pasos IK
- **Intento 1**: target=(0.443,0.001,0.025), tcp_after=(0.443,0.001,0.039), target_err=0.014m, xy_dist=0.007m
- **Intento 2**: target=(0.443,0.001,0.015), tcp_after=(0.442,0.002,0.030), target_err=0.015m, xy_dist=0.007m
- **Estado final**: `LISTO PARA COGER` — tcp_obj_dist=0.008m, xy=0.007m/tol=0.016m, z=0.005m/tol=0.010m
- **Veredicto**: **PASS** — "AVISO: LISTO PARA COGER" emitido
- **Nota**: IK TOL WARN para ik_err_tol=0.200 (>safe_max=0.015). Residual DH/SDF de ~14mm entre IK model y TCP real.

### PRE_CLOSE
- **Inicio**: 15:56:16
- **TCP**: (0.442, 0.002, 0.029) — obj=(0.436, -0.000, 0.025)
- **xy_dist**: 0.0071m / tol=0.016m ✓
- **z_error**: 0.0038m / tol=0.010m ✓
- **consecutive_ok**: 3/3 ✓
- **Gripper**: abierto (opening_sum=1.0)
- **Snapshot JSON**: confirmado phase=PRE_CLOSE, ok=true
- **Veredicto**: **PASS** — gate_result=true en 1 intento
- **Evidencia visual**: `04_PRE_CLOSE_enter_overhead.png`, `04_PRE_CLOSE_exit_overhead.png`

### CLOSE
- **Inicio**: 15:56:16 (inmediato tras PRE_CLOSE)
- **Señal de cierre enviada**
- **Gripper result**: opening_sum=0.749 (antes=1.0, después=0.749 → delta=0.251)
- **min_delta**: 0.251 > 0.01 ✓
- **confirmed=True**, mode=closing_delta_ok
- **Snapshot ATTACH_GATE JSON**: opening_sum=0.106 (gripper casi completamente cerrado)
- **joint_positions**: rg2_finger_joint1=0.053, rg2_finger_joint2=0.053 (≈5.3° de máximo)
- **force_estimate**: 0.5
- **Veredicto**: **PASS** — gripper cerrado con objeto capturado

### ATTACH_GATE
- **Inicio**: 15:56:16
- **Fin**: 15:56:16 (muy rápido)
- **Geometría**: xy_dist=0.007m/tol=0.020m ✓, z_err=0.003m/tol=0.010m ✓
- **Attach publish**: topic=/gripper/pick_demo/attach → sub=3 ✓
- **Backend response**: route=demo_transport, dist=0.0073m/max=0.060m ✓
- **demo_transport_enabled**: mode=respawn_kinematic_no_collision
- **Logical state**: CARRIED, owner=ROBOT ✓
- **Veredicto**: **PASS** — spawn_confirmed=true, logical_attached=true
- **Evidencia visual**: `06_ATTACH_GATE_enter_overhead.png`, `06_ATTACH_GATE_exit_overhead.png`

### LIFT
- **Inicio**: 15:56:17
- **Timeout configurado**: 23s
- **Lift target**: +0.12m desde TCP actual (~0.147 en base_link)
- **Proceso observado** (por PANEL_TRACE):
  - 15:56:17: TCP z=0.027 (inicio LIFT)
  - 15:56:26 → TCP z=0.019 (settle de 3s CARRY_SETTLE_SEC)
  - 15:56:33 → TCP z=0.020 (settle completado, robot estacionario)
  - 15:56:35 → TCP z=0.021, obj z=0.004 (ascenso iniciado)
  - 15:56:50 → TCP z=0.043, obj z=0.026 (+19mm)
  - 15:56:56 → TCP z=0.057, obj z=0.040 (+33mm)
  - 15:57:04 → TCP z=0.076, obj z=0.059 (+52mm)
  - 15:57:16 → TCP z=0.107, obj z=0.091 (+84mm)
- **Progreso al SHUTDOWN**: ~87mm de 120mm objetivo (73% completado)
- **ATTACH_BACKEND follow_tcp**: activo y respondiendo gz_service_ok durante todo el LIFT
- **Veredicto**: **INCONCLUSIVE — INTERRUMPIDO EXTERNAMENTE** (SIGINT al proceso a las 15:57:18, no fallo de fase)
- **Nota**: El objeto SÍ estaba subiendo consistentemente. La interrupción es externa, no un fallo de LIFT.

### TRANSPORT / HOME_WITH_OBJECT
- **Estado**: NO ALCANZADO (LIFT aún activo en momento de SHUTDOWN)
- **Veredicto**: **SIN EVIDENCIA** — ciclo no completado
- **Nota**: Riesgo conocido de HOME_WITH_OBJECT (objeto se suelta en transporte largo) NO pudo confirmarse ni descartarse.

### CESTA_RELEASE
- **Estado**: NO ALCANZADO
- **Veredicto**: **SIN EVIDENCIA**

### HOME_FINAL
- **Estado**: NO ALCANZADO
- **Veredicto**: **SIN EVIDENCIA**

---

## Problemas encontrados

### P1: Cartesian path GRASP_DOWN_JOINT falla (cartesian_low_fraction=0.000)
- **Severidad**: MEDIA (resuelto por fallback IK)
- **Causa probable**: Posición inicial de GRASP_DOWN muy próxima al objeto → Cartesian path no encuentra arco sin colisión
- **Impacto**: Retardo extra en GRASP_DOWN, usa IK directo en lugar de trayectoria suave

### P2: IK_ERR_TOL GRASP_ALIGN_IK = 0.200 (demasiado grande)
- **Severidad**: BAJA-MEDIA
- **Causa**: Configuración explícita en run_directo_validation.sh para garantizar convergencia con mismatch DH/SDF ~13mm
- **Riesgo**: IK puede aceptar soluciones con >15mm de error posicional
- **Mitigación activa**: Gate de PRE_CLOSE (xy_tol=0.020, z_tol=0.010) detectaría malas soluciones

### P3: Ciclo INTERRUMPIDO a las 15:57:18
- **Severidad**: ALTA (para esta corrida)
- **Causa**: SIGINT externo (SIGTERM al proceso de launch) durante fase LIFT
- **Impacto**: LIFT, HOME_WITH_OBJECT, CESTA_RELEASE, HOME_FINAL no completados
- **Nota**: NO es un fallo del pipeline, es interrupción del usuario/sistema

### P4: Mismatch DH/SDF ~13-15mm
- **Severidad**: CONOCIDA/BAJA (tolerancias ajustadas para compensar)
- **Evidencia**: En GRASP_ALIGN_IK, target_err=0.014-0.015m después de IK con pos_err≈0
- **Estado**: Mitigado por PRE_CLOSE_XY_TOL=0.020m y ATTACH_XY_TOL=0.020m

---

## Causa raíz del ciclo incompleto

**La corrida fue interrumpida externamente (SIGINT al proceso ROS launch) a las 15:57:18, mientras el robot ejecutaba la fase LIFT.** No hubo fallo de ninguna fase del pipeline. Las fases APPROACH_COARSE, GRASP_DOWN, GRASP_ALIGN_IK, PRE_CLOSE, CLOSE y ATTACH_GATE completaron con éxito. LIFT estaba a ~73% de completarse.

---

## Evidencia visual

- **03_GRASP_ALIGN_IK_enter_overhead.png**: TCP sobre objeto en GRASP_ALIGN_IK entrada
- **03_GRASP_ALIGN_IK_exit_overhead.png**: TCP alineado al objeto tras GRASP_ALIGN_IK
- **04_PRE_CLOSE_enter_overhead.png**: Posición TCP en entrada PRE_CLOSE
- **04_PRE_CLOSE_exit_overhead.png**: Posición TCP en salida PRE_CLOSE (gate pasado)
- **06_ATTACH_GATE_enter_overhead.png**: Estado en entrada ATTACH_GATE (gripper cerrándose)
- **06_ATTACH_GATE_exit_overhead.png**: Estado tras ATTACH (objeto CARRIED)
- **camera_debug_top_20260420/**: 62 frames periódicos de múltiples cámaras durante la sesión

---

## Recomendaciones

1. **Completar ciclo con nueva corrida sin interrupción**: La evidencia muestra que el pipeline hasta LIFT es sólido. Ejecutar `./scripts/run_directo_validation.sh` con stack limpio para obtener ciclo completo.

2. **Confirmar TRANSPORT/HOME_WITH_OBJECT**: El riesgo conocido (objeto se suelta durante transporte largo) no pudo evaluarse. Es necesaria una corrida completa para validar o reproducir este fallo.

3. **Reducir IK_ERR_TOL para GRASP_ALIGN_IK**: Actual=0.200 genera warnings. Considerar reducir a 0.030-0.050 dado que el DH/SDF mismatch real es ~13mm, y el gate PRE_CLOSE captura errores residuales.

4. **Resolver cartesian_low_fraction en GRASP_DOWN**: El path cartesiano genera fraction=0.000 consistentemente. Investigar si `PANEL_PICK_DEMO_GRASP_DOWN_USE_MOVEIT_CARTESIAN=0` mejoraría latencia de GRASP_DOWN.

5. **Monitorear lift_delta durante LIFT**: Asegurar que CARRY_SETTLE_SEC=3.0 no cause falsos negativos en el gate de lift_delta si la posición de settle es muy similar a la posición de contact.
