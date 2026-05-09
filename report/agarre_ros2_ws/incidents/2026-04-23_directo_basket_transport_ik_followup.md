# Follow-up tecnico: transporte DIRECTO a cesta

Fecha: 2026-04-23
Estado: abierto
Sistema: UR5 + RG2 + ROS 2 Jazzy + Gazebo Sim + MoveIt 2
Relacion con incidente TCP: separado; el TCP ya esta cerrado

## Alcance

Este follow-up cubre el atasco residual del flujo DIRECTO entre `lift_with_object` y `basket_drop`.

No cubre:

- geometria del TCP;
- `rg2_tcp`;
- `rg2_pinch_center`;
- gate geometrico de arranque.

## Hallazgo principal

El transporte a cesta sigue fallando porque algunos `joint_goal` calculados para `CESTA_STAGE_1` y sus `RECOVER_*` no reproducen el target cartesiano esperado dentro de tolerancia.

La señal mas fiable no es el log de IK en si, sino la combinacion de:

- `runtime_target_dist`
- `model_target_err`
- evidencia visual

## Evidencia

### 1. Post-check de transporte

Corrida:

- `../auditoria/manual_directo_postcheck_20260423_092724`

Fallo representativo:

- `CESTA_STAGE_1_RECOVER_1_postcheck_failed`
- `runtime_target_dist=0.055/0.040`
- `model_target_err=0.055/0.040`

Interpretacion:

- el solver encontro una solucion con `pos_err_m` pequeno en la llamada de IK;
- pero el `fk_ur5` del goal ejecutado y el runtime real quedaron fuera de tolerancia.

### 2. Micro-replan local adicional

Corrida:

- `../auditoria/manual_directo_postcheck_min035_20260423_093612`

Configuracion relevante:

- `PANEL_PICK_DEMO_TRANSPORT_STAGE_REPLAN_MIN_REMAINING_DIST_M=0.035`

Resultados:

- `CESTA_STAGE_1_RECOVER_1_RECOVER_2` si entro en tolerancia:
  - `runtime_target_dist=0.038/0.040`
  - `model_target_err=0.038/0.040`
- `CESTA_STAGE_1_RECOVER_1_RECOVER_3` volvio a fallar:
  - `runtime_target_dist=0.050/0.040`
  - `model_target_err=0.050/0.040`

Interpretacion:

- el micro-replan ayuda;
- pero la granularidad por si sola no resuelve el corredor completo.

## Estado actual del codigo

Ya existe en `panel_pick_demo.py`:

- retry de startup y limpieza de stack en el harness
- guard pre-ejecucion FK/modelo para `CESTA_STAGE_*`
- prep segmentado de transporte
- salto temprano de `PREP` a `TRANSPORT_REPLAN`
- `TRANSPORT_POSTCHECK` runtime/modelo
- replan segmentado de residual cartesiano

Lo que falta no es otro ajuste del TCP. Falta mejorar la calidad de la solucion IK que se acepta para cesta.

## Hipotesis tecnica de trabajo

La cadena actual entra en un corredor donde:

- la solucion IK numerica es formalmente valida en la llamada;
- pero el goal normalizado/ejecutado acaba en una rama o seed que deja `fk_ur5(solved_q)` fuera de tolerancia respecto a `target_ik`.

El patron repetido en los fallos es:

- error pequeño en la llamada IK;
- `fk_after` fuera de target tras ejecucion;
- residual de hombro persistente;
- runtime sin progreso real adicional.

## Trabajo pendiente

1. Añadir guard pre-ejecucion de coherencia de modelo en cesta.
   Rechazar `solved_q` si `fk_ur5(solved_q)` no cae dentro de tolerancia frente a `target_ik`.

2. Implementar reseeding/branching explicito para `CESTA_STAGE_1`.
   Probar al menos:
   - `live_joints`
   - ultimo sub-stage valido
   - seed de prep anterior
   - variante de hombro/codo para el corredor de cesta

3. Reintentar solo con soluciones que pasen:
   - guard de modelo
   - runtime target
   - post-check final

4. Revalidar DIRECTO completo.
   Objetivo:
   - `helper_final_rc=0`
   - `basket_drop.png`
   - `visual_smoke_complete=1`

5. Ejecutar batch de aceptacion.
   - `RUNS=5`
   - `RUNS=10`

6. Repetir la validacion manual final con:
   - `./lanzar_panelc2.sh`

## Criterios de salida del follow-up

- `CESTA_STAGE_1` y sus `RECOVER_*` no producen `postcheck_failed`
- `runtime_target_dist <= 0.040` en stages aceptados
- `model_target_err <= 0.040` en stages aceptados
- se genera `basket_drop.png`
- DIRECTO termina con `helper_final_rc=0`
- `RUNS=5` y `RUNS=10` pasan
- la repeticion manual final con `./lanzar_panelc2.sh` queda firmada
