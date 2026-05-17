# Decisión técnica — Cierre hito `pick_demo PASS`

Fecha: 2026-05-17
Responsable de cierre: validación reproducida + documentación congelada
Resultado: **PASS CON LIMITACIONES DOCUMENTADAS**

---

## Resumen ejecutivo

El sistema UR5+RG2 simulado puede:

- Mover el brazo físicamente en Gazebo Harmonic con `gz_ros2_control` (effort interface).
- Resolver IK con MoveIt 2 usando `ik_link_name="rg2_pinch_center"` (TCP operativo).
- Ejecutar trayectorias JTC que alcanzan poses cercanas con error sub-mm a 1 cm.
- Realizar **pick_demo** (un único objeto pickable) mediante attach por proximidad y kinematic-follow del TCP durante un lift rápido (4 s, delta shoulder_lift -0.20 rad), con error TCP↔objeto **mean 0.41 cm / max 1.01 cm** y sin pérdida de coherencia.

## Métricas validadas (re-medidas en cierre)

| Métrica | Valor | Criterio | Veredicto |
|---|---|---|---|
| MoveIt err_pos (TCP +0.03 m Z) | **0.5 mm** | ideal <20 mm | ✓ supera 40× |
| MoveIt err_rot | 0.72° | — | negligible |
| pick_demo attach success | True, method=follow_tcp, dist=5.7 mm | <50 mm | ✓ |
| pick_demo fast lift mean diff | **0.41 cm** | mínimo <2 cm, ideal <1 cm | ✓✓ ideal |
| pick_demo fast lift max diff | **1.01 cm** | <2 cm | ✓ |
| coherence_lost / auto-detach | 0 / 0 | =0 | ✓ |
| duplicate name warnings | 0 | =0 | ✓ |
| NaN/Inf | none | =0 | ✓ |

## Configuración oficial congelada

| Parámetro | Valor | Razón |
|---|---|---|
| `enable_detachable_joints` (URDF arg) | **false** | H1: reactivar bloquea motion-control |
| `attach_mode` | `follow_tcp` | único path operativo tras H1 |
| `require_contact_before_attach` | `False` | gripper físico no converge; proximity attach |
| `attach_max_dist_m` | 0.05 | gate de distancia activo (seguridad) |
| `gripper_closed_rad` | -5.0 | effort negativo (best-effort cierre) |
| `set_pose_async_cli` | **True** | Popen paralelo > bridge serializado |
| `demo_transport_objects` | `[pick_demo]` | NO escalado a 10 objetos |
| Bridge SetEntityPose | disponible | fallback opcional (`set_pose_async_cli=False`) |

## Decisiones técnicas razonadas

### 1. Causa raíz del bloqueador motion-control
**Las 11 instancias del plugin `gz::sim::systems::DetachableJoint`** definidas dentro del modelo `ur5_rg2` (vía `<xacro:detachable_joint obj="X" parent_link="wrist_3_link"/>`) saturaban el constraint solver de DART, generando ~10 warnings duplicate name por spawn y bloqueando shoulder_pan/wrist_2 incluso con +11 N·m sostenidos.

**Bisección estructural** (sesión `fix_ur5_rg2_model_bisection`):
- H1 (xacro:if wrap con default false) → PASS
- H2-H4 (empty world, base mass, gravity tags) → no necesarias
- Confirmado vía `gz_ros2_control_minimal_probe` (paquete diagnostic creado en sesión `comparativa_gz_ros2_control_minimal`): instalación gz_ros2_control + DART funcionan; bug específico del modelo UR5+RG2.

### 2. Backend follow_tcp = async Popen
**Empíricamente medido**:

| Variante | mean diff | max diff |
|---|---|---|
| `subprocess.run` blocking + `/set_pose/blocking` | FAIL (coherence_lost) | — |
| `Popen` async + `/set_pose` (non-blocking) | **0.41–1.27 cm** | **1.0–1.9 cm** |
| ROS service via `parameter_bridge` (serializado) | 3.8 cm | 15 cm |

Popen permite ~8 dispatches paralelos → gz physics rate efectivo mayor. El bridge ROS serializa (1 future pendiente, ~10 Hz cap).

### 3. Proximity attach vs cierre físico real
El gripper RG2 prismático **no converge a cierre completo** (opening_sum permanece ~42 mm aún con effort -20 N sostenido 5 s). Probable conflicto entre `<param initial_value="0.0">` del position command_interface (URDF) y el effort enviado por `gripper_effort_controller` — reapliando cada step.

**Workaround aceptado para demo**: `require_contact_before_attach=False` salta el contact_gate bilateral. El gate de distancia (`attach_max_dist_m=0.05`) sigue activo como condición de seguridad.

## Limitaciones documentadas

1. Cierre físico RG2 no converge → workaround proximity.
2. Backend follow_tcp via async_cli Popen (no ros_service por defecto).
3. Solo `pick_demo` validado; 10 objetos coloreados NO escalados.
4. NO reactivar los 11 DetachableJoint internos.
5. NO tocar TCP semántica `rg2_pinch_center` (validada y congelada).

## Genealogía (9 sesiones, 5 commits atómicos)

1. `fix_robot_definitivo_20260517_130121` — β1.2/β1.3 FAIL (bisección anchor URDF)
2. `fix_control_position_R1_20260517_134427` — R1 FAIL → rollback
3. `comparativa_gz_ros2_control_minimal_20260517_140752` — bug aislado al modelo UR5+RG2 (minimal probe PASS)
4. `fix_ur5_rg2_model_bisection_20260517_143423` — **H1 PASS** (commit 1)
5. `fix_pick_attach_external_anchor_20260517_153347` — attach migrated a follow_tcp (commit 2)
6. `fix_gripper_close_pick_demo_20260517_155955` — proximity attach (commit 2)
7. `fix_follow_tcp_rate_20260517_163003` — async Popen (commit 3)
8. `fix_follow_tcp_ros_set_entity_pose_20260517_165033` — bridge + async prevalece (commits 3 y 4)
9. `cierre_hito_pick_demo_pass_20260517_171343` — validación + documentación (commit 5)

## Siguiente fase recomendada (NO mezclar)

- **A) Escalado a objetos coloreados**: añadir uno por uno a `demo_transport_objects` y re-validar fast lift por cada uno.
- **B) Mejora sub-cm con pose extrapolation predictiva**: TCP_target_t+1 = TCP_current + v·dt en el follow loop.
- **C) Cierre documental TFM**: usar esta entrada + sesiones genealógicas como base del capítulo de validación experimental.

## Evidencia detallada (fuera de git por política del repo)

- `historico/fix_*_20260517_*/` — 8 sesiones de fix con diffs, validaciones, logs runtime
- `historico/cierre_hito_pick_demo_pass_20260517_171343/` — validación reproducida + checklist 16/16 + limitaciones detalladas
- `historico/commit_hito_pick_demo_pass_20260517_174729/` — plan y artefactos de versionado
- `auditoria/comparativa_gz_ros2_control_minimal_20260517_140752/` — bisección minimal probe
- `reports/cierre_hito_pick_demo_pass_20260517_171343.md` — resumen ejecutivo
