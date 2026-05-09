# Diff contra documento anterior

```diff
--- report/BaseDeConocimiento/2026-04-20_base_conocimiento_tecnica_TFM.md
+++ 2026-04-20_base_conocimiento_tecnica_TFM.md
@@ -1158,110 +1158,10 @@
 - `ATTACH_BACKEND_MAX_DIST_M`: launch actual=0.08; wrapper `start_panel_v2.sh` exporta 0.06 por defecto.
 - CARRY metadata vs llamada real: metadata CARRY=(0.030, 0.060, 0.080) frente a llamada post_grasp_lift=(0.020, 0.025, 0.120).
 
-### Detalle histórico recuperado (2026-04-18)
-
-Las variables se inyectan en `ur5_stack.launch.py` mediante `os.environ.get(VAR, DEFAULT)`. Se pueden sobreescribir con `export VAR=valor` antes de ejecutar el launch.
-
-#### 6.1 Tabla completa de variables de entorno
-
-##### Geometría y altura de agarre
-
-| Variable | Default | Archivo | Fase | Unidades | Efecto | Riesgo si mal configurada |
-|---|---|---|---|---|---|---|
-| `GRASP_CONTACT_Z_OFFSET_M` | 0.13 | `ur5_stack.launch.py` | GRASP_DOWN | metros | Posición Z del TCP respecto a Z del objeto (13 cm = ecuador del cilindro) | Muy alto: si < real, robot golpea mesa; si > real, no agarra |
-| `PANEL_PICK_DEMO_APPROACH_COARSE_EXTRA_Z_M` | 0.035 | `ur5_stack.launch.py` | APPROACH_COARSE | metros | Altura extra sobre el objeto en approach (35 mm de margen) | Medio: si muy pequeño, puede colisionar en descenso |
-
-##### GRASP_DOWN
-
-| Variable | Default | Archivo | Fase | Unidades | Efecto | Riesgo |
-|---|---|---|---|---|---|---|
-| `PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_Z_STEP_M` | 0.005 | `ur5_stack.launch.py` | GRASP_DOWN | metros | Tamaño de cada paso Z en descenso segmentado | Alto: > 17 mm causa saltos de rama IK, error ~48 mm Y |
-| `PANEL_PICK_DEMO_GRASP_DOWN_USE_MOVEIT_CARTESIAN` | 1 | `ur5_stack.launch.py` | GRASP_DOWN | bool (0/1) | Usar MoveIt computeCartesianPath vs IK segmentado propio | Medio: si =0, usa IK segmentado (más lento pero más robusto) |
-| `PANEL_PICK_DEMO_GRASP_DOWN_IK_SEED_WEIGHT` | 0.65 | `ur5_stack.launch.py` | GRASP_DOWN | [0,1] | Peso semilla IK (mantiene rama de solución) | Medio: demasiado bajo = cambio de rama; demasiado alto = no converge |
-| `PANEL_PICK_DEMO_GRASP_DOWN_KEEP_XY_TOL_M` | 0.003 | `ur5_stack.launch.py` | GRASP_DOWN | metros | Si XY de APPROACH < 3 mm, hereda pose exact | Bajo: si > 3 mm, resetea al centro del objeto |
-| `PANEL_PICK_DEMO_GRASP_DOWN_STRICT_XY_TOL_M` | 0.008 | `ur5_stack.launch.py` | GRASP_DOWN | metros | Tolerancia XY post-GRASP_DOWN | Medio: muy permisivo → mal alineado para CLOSE |
-| `PANEL_PICK_DEMO_GRASP_DOWN_STRICT_Z_TOL_M` | 0.008 | `ur5_stack.launch.py` | GRASP_DOWN | metros | Tolerancia Z post-GRASP_DOWN | Medio |
-| `PANEL_PICK_DEMO_GRASP_DOWN_STRICT_DIST_TOL_M` | 0.012 | `ur5_stack.launch.py` | GRASP_DOWN | metros | Distancia 3D post-GRASP_DOWN | Medio |
-| `PANEL_PICK_DEMO_GRASP_DOWN_MAX_ATTEMPTS` | 4 | `ur5_stack.launch.py` | GRASP_DOWN | int | Intentos max por segmento IK | Bajo |
-
-##### GRASP_ALIGN_IK
-
-| Variable | Default | Archivo | Fase | Unidades | Efecto | Riesgo |
-|---|---|---|---|---|---|---|
-| `PANEL_PICK_DEMO_ALIGN_IK_ERR_TOL` | 0.08 | `ur5_stack.launch.py` | GRASP_ALIGN | metros | Tolerancia error IK (80 mm) | Medio: muy permisivo puede aceptar configuración incorrecta |
-| `PANEL_PICK_DEMO_ALIGN_EXIT_XY_TOL_M` | 0.010 | `ur5_stack.launch.py` | GRASP_ALIGN | metros | XY exit del loop de alineación | Medio: más estricto = más reintentos |
-| `PANEL_PICK_DEMO_ALIGN_EXIT_Z_TOL_M` | 0.010 | `ur5_stack.launch.py` | GRASP_ALIGN | metros | Z exit del loop de alineación | Medio |
-| `PANEL_PICK_DEMO_ALIGN_Z_RESIDUAL_TOL_M` | 0.008 | `ur5_stack.launch.py` | GRASP_ALIGN | metros | Umbral para activar bias Z (si error > 8 mm → aplica bias) | Alto: si muy bajo, bias se activa innecesariamente |
-| `PANEL_PICK_DEMO_PRE_CLOSE_REALIGN_RETRIES` | 2 | `ur5_stack.launch.py` | PRE_CLOSE | int | Reintentos de realineación en PRE_CLOSE | Bajo |
-
-##### CLOSE
-
-| Variable | Default | Archivo | Fase | Unidades | Efecto | Riesgo |
-|---|---|---|---|---|---|---|
-| `PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC` | 3.0 | `ur5_stack.launch.py` | CLOSE | segundos | Timeout confirmación cierre gripper | Alto: muy corto → falso fallo; muy largo → cuelga sistema |
-| `PANEL_PICK_DEMO_CLOSE_MIN_DELTA_SUM` | 0.01 | `ur5_stack.launch.py` | CLOSE | metros | Delta mínimo suma joints para confirmar cierre (10 mm) | Alto: muy alto → nunca confirma; muy bajo → falso positivo |
-| `PANEL_PICK_DEMO_GRIPPER_TARGET_TOL_M` | 0.12 | `ur5_stack.launch.py` | CLOSE | metros | Tolerancia objetivo gripper (120 mm) | Medio |
-| `PANEL_PICK_DEMO_CLOSE_XY_TOL_M` | 0.008 | `ur5_stack.launch.py` | CLOSE | metros | XY tolerance en CLOSE | Medio |
-| `PANEL_PICK_DEMO_CLOSE_Z_ERR_TOL_M` | 0.008 | `ur5_stack.launch.py` | CLOSE | metros | Z error en CLOSE | Medio |
-
-##### ATTACH_GATE
-
-| Variable | Default | Archivo | Fase | Unidades | Efecto | Riesgo |
-|---|---|---|---|---|---|---|
-| `PANEL_PICK_DEMO_ATTACH_XY_TOL_M` | 0.008 | `ur5_stack.launch.py` | ATTACH_GATE | metros | Tolerancia XY para gate | Medio |
-| `PANEL_PICK_DEMO_ATTACH_Z_TOL_M` | 0.010 | `ur5_stack.launch.py` | ATTACH_GATE | metros | Tolerancia Z para gate | Medio |
-| `PANEL_PICK_DEMO_ATTACH_FOLLOW_MAX_TCP_DIST_M` | 0.040 | `ur5_stack.launch.py` | ATTACH_GATE | metros | Dist máxima TCP-objeto para gate (40 mm) | Alto: muy restrictivo → gate nunca pasa |
-| `PANEL_PICK_DEMO_ATTACH_MAX_REL_DRIFT_M` | 0.012 | `ur5_stack.launch.py` | ATTACH_GATE | metros | Drift relativo máximo en ventana temporal | Alto: muy restrictivo → gate falla con sim jitter |
-| `PANEL_PICK_DEMO_ATTACH_STABLE_WINDOW_SEC` | 0.35 | `ur5_stack.launch.py` | ATTACH_GATE | segundos | Ventana temporal de estabilidad | Medio |
-| `PANEL_PICK_DEMO_ATTACH_MIN_STABLE_SAMPLES` | 5 | `ur5_stack.launch.py` | ATTACH_GATE | int | Muestras mínimas en ventana (~28-30 Hz) | Medio |
-| `PANEL_PICK_DEMO_ATTACH_MAX_TF_VISUAL_GAP_M` | 0.020 | `ur5_stack.launch.py` | ATTACH_GATE | metros | Gap máximo TF vs visual (warning) | Bajo (solo warning) |
-| `ATTACH_BACKEND_MAX_DIST_M` | 0.06 | `ur5_stack.launch.py` | ATTACH_GATE | metros | Distancia máxima para crear detachable joint | Alto: si objeto más lejos de 60 mm, no se adjunta |
-
-##### PRE_CLOSE
-
-| Variable | Default | Fase | Unidades | Efecto |
-|---|---|---|---|---|
-| `PANEL_PICK_DEMO_PRE_CLOSE_XY_TOL_M` | 0.010 | PRE_CLOSE | metros | Validación XY pre-cierre (10 mm) |
-| `PANEL_PICK_DEMO_PRE_CLOSE_Z_ERR_TOL_M` | 0.010 | PRE_CLOSE | metros | Validación Z pre-cierre (10 mm) |
-| `PANEL_PICK_DEMO_SKIP_ALIGN_IF_REACHABLE` | 0 | PRE_CLOSE | bool (0/1) | Saltar realineación si pose es alcanzable |
-
-##### APPROACH_COARSE gate
-
-| Variable | Default | Fase | Unidades | Efecto |
-|---|---|---|---|---|
-| `PANEL_PICK_DEMO_APPROACH_COARSE_GATE_XY_TOL_M` | 0.012 | APPROACH | metros | Gate XY approach coarse |
-| `PANEL_PICK_DEMO_APPROACH_COARSE_GATE_Z_TOL_M` | 0.012 | APPROACH | metros | Gate Z approach coarse |
-
-##### Pose freshness y fuentes
-
-| Variable | Default | Fase | Unidades | Efecto |
-|---|---|---|---|---|
-| `PANEL_PICK_DEMO_POSE_SOURCE_AGE_TOL_SEC` | 0.400 | Global | segundos | Edad máxima FK/trace válida (400 ms) |
-| `PANEL_PICK_DEMO_POSE_SOURCE_TOL_M` | 0.006 | Global | metros | Tolerancia fuente pose (6 mm) |
-| `PANEL_PICK_DEMO_PHASE_JUMP_TOL_M` | 0.010 | Global | metros | Tolerancia salto de fase (10 mm) |
-| `PANEL_PICK_DEMO_OBJECT_SOURCE_DIVERGENCE_TOL_M` | 0.150 | Global | metros | Umbral divergencia snapshot vs stable cache (150 mm) |
-
-##### Settle IK directo
-
-| Variable | Default | Fase | Unidades | Efecto |
-|---|---|---|---|---|
-| `PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SEC` | 2.5 | IK directo | segundos | Tiempo máximo de settle post-IK |
-| `PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_DELTA_M` | 0.003 | IK directo | metros | Delta posición para considerar settled (3 mm) |
-| `PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_SAMPLES` | 3 | IK directo | int | Muestras para confirmar settle |
-| `PANEL_PICK_DEMO_DIRECT_IK_RUNTIME_SETTLE_POLL_SEC` | 0.10 | IK directo | segundos | Intervalo poll settle (100 ms) |
-
-##### MoveIt bridge
-
-| Variable | Default | Archivo | Unidades | Efecto |
-|---|---|---|---|---|
-| `PANEL_MOVEIT_BRIDGE_EXECUTE_TIMEOUT_SEC` | 150.0 | `ur5_stack.launch.py` | segundos | Timeout ejecución MoveIt |
-| `PANEL_MOVEIT_BRIDGE_REQUEST_TIMEOUT_SEC` | 180.0 | `ur5_stack.launch.py` | segundos | Timeout petición total |
-| `PANEL_MOVEIT_BRIDGE_VELOCITY_SCALE` | 0.30 | `ur5_stack.launch.py` | [0,1] | Escala velocidad MoveIt (30% conservador) |
-| `PANEL_MOVEIT_BRIDGE_ACCEL_SCALE` | 0.30 | `ur5_stack.launch.py` | [0,1] | Escala aceleración |
-| `PANEL_MOVEIT_BRIDGE_JOINT_STATE_TIMEOUT_SEC` | 6.0 | `ur5_stack.launch.py` | segundos | Timeout joint state |
-| `PANEL_MOVEIT_BRIDGE_JOINT_STATE_MAX_AGE_SEC` | 2.5 | `ur5_stack.launch.py` | segundos | Edad máxima joint state |
-
----
+### 7.99 Nota sobre trazabilidad histórica
+
+- El bloque histórico extenso de variables ya no se inserta entero en el cuerpo principal para evitar ruido operativo.
+- Su contenido queda absorbido en la columna `Histórico` y en la columna `Discrepancia` del apéndice 15.5, donde se conserva el inventario exhaustivo actual con trazabilidad de overrides y defaults previos.
 
 ## 8. Validación Física Post-Grasp / Carry
 
```
