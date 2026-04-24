# Auditoria global del proyecto TFM

Fecha: 2026-04-24

## 1. Alcance y criterio de esta auditoria

Esta auditoria consolida el estado actual del workspace completo `TFM` con foco en:

- documentacion canónica realmente util para la entrega y la defensa
- inventario operativo del stack ROS 2
- consistencia entre panel, pipeline de pick, pipeline CNN y artefactos de memoria
- riesgos abiertos que impiden declarar el proyecto completamente cerrado

Se prioriza como fuente de verdad el estado actual del repo, seguido de artefactos curados en `reports/` y de informes técnicos recientes en `auditoria/`.

## 2. PDFs canónicos cerrados

### 2.1 Set canónico identificado

- `reports/TFM_Lozano_Rodriguez-Jesus.pdf`
  - Rol: memoria final de referencia del TFM en este workspace.
  - Estado: documento principal que debe tomarse como referencia editorial y de defensa.
- `reports/exports/chapter_artifacts/08_anexos/Artefacto_8-A1_documento_pdf_del_tfm_utilizado_como_referencia_editorial.pdf`
  - Rol: exportación/artefacto derivado de la memoria final para anexos y trazabilidad editorial.
  - Estado: secundario respecto al PDF principal; útil como copia curada, no como fuente primaria independiente.
- `reports/BaseDeConocimiento/2026-04-18_base_conocimiento_tecnica_TFM.pdf`
  - Rol: base técnica histórica exportada a PDF.
  - Estado: material de apoyo técnico e histórico, no sustituto de la memoria final.

### 2.2 Documento técnico canónico de apoyo

- `reports/BaseDeConocimiento/2026-04-23_base_conocimiento_tecnica_TFM.md`
  - Rol: base de conocimiento técnica actualizada con inventario de paquetes, launches, frames, geometría y flujo pick.
  - Estado: referencia técnica operativa más útil para auditoría interna que el PDF histórico de base de conocimiento.

### 2.3 Cierre de la tarea

La tarea de “Extraer PDFs canónicos” queda cerrada en el sentido de clasificación y jerarquía documental:

- fuente primaria de entrega: `reports/TFM_Lozano_Rodriguez-Jesus.pdf`
- copia editorial derivada: `reports/exports/chapter_artifacts/08_anexos/Artefacto_8-A1_documento_pdf_del_tfm_utilizado_como_referencia_editorial.pdf`
- soporte técnico histórico: `reports/BaseDeConocimiento/2026-04-18_base_conocimiento_tecnica_TFM.pdf`
- soporte técnico vivo para auditoría: `reports/BaseDeConocimiento/2026-04-23_base_conocimiento_tecnica_TFM.md`

## 3. Inventario de paquetes y launch files

### 3.1 Paquetes ROS 2 reales en `agarre_ros2_ws/src`

Se confirma la existencia de 7 paquetes ROS 2 reales:

- `ur5_bringup`
- `ur5_description`
- `ur5_moveit_config`
- `ur5_qt_panel`
- `ur5_tools`
- `ur5_panel_interfaces`
- `tfm_grasping`

No aparecen paquetes adicionales bajo `agarre_ros2_ws/src` fuera de ese conjunto.

### 3.2 Launch files Python canónicos

Se confirman 4 launch files Python reales:

- `agarre_ros2_ws/src/ur5_bringup/launch/ur5_stack.launch.py`
- `agarre_ros2_ws/src/ur5_bringup/launch/ur5_rsp.launch.py`
- `agarre_ros2_ws/src/ur5_bringup/launch/ur5_ros2_control.launch.py`
- `agarre_ros2_ws/src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py`

### 3.3 Wrappers y orquestadores operativos relevantes

Además de los launch files, el workspace usa wrappers y orquestadores con valor canónico real:

- `./lanzar_panelc2.sh`: entrypoint canónico para arranque manual final.
- `./lanzar_panelv2.sh`: alias de compatibilidad.
- `agarre_ros2_ws/scripts/start_panel_v2.sh`: wrapper interno de arranque.
- `agarre_ros2_ws/scripts/run_directo_validation.sh`: runner canónico de validación DIRECTO.
- `agarre_ros2_ws/scripts/run_directo2_validation.sh`: runner de validación DIRECTO2.
- `agarre_ros2_ws/scripts/run_moveit_lift_validation.sh`: validación focalizada de LIFT/MoveIt.
- `agarre_ros2_ws/scripts/run_directo_batch_validation.sh`: batch runner de validación DIRECTO.

### 3.4 Estado del inventario

El inventario básico de paquetes y launches queda cerrado y consistente con la base técnica curada en `reports/BaseDeConocimiento/2026-04-23_base_conocimiento_tecnica_TFM.md`.

## 4. Pendiente técnico acotado abierto: `CESTA_STAGE_1_RECOVER_2`

### 4.1 Estado confirmado

- Se implementó micro-recovery local en `panel_pick_demo.py`.
- Se añadieron pruebas en `test_panel_pick_demo_transport_follow.py`.
- Tests OK: `30 passed`.
- Build `ur5_qt_panel` OK.
- La corrida canónica no ejerció `RECOVER_2` porque volvió a bloquear antes en `CESTA_STAGE_1_RECOVER_1`.

### 4.2 Fallo observado en la corrida canónica más reciente

- `runtime_target_dist=0.050/0.040`
- `model_target_err=0.048/0.040`
- `TRANSPORT_POSTCHECK ok=false`
- `PICK_FAIL`

### 4.3 Consecuencias observadas

- No aparecen logs `RECOVER2_MICRO`.
- No se alcanza `CESTA_RELEASE`.
- No se alcanza `HOME_FINAL`.

### 4.4 Conclusión operativa

- El parche de `RECOVER_2` está preparado y cubierto por tests, pero no validado en runtime canónico.
- Para validarlo haría falta una rerun diagnóstica con bypass temporal `0.050` en `RECOVER_1`.
- Ese bypass no debe promoverse todavía a fix definitivo.

## 5. Auditoría del panel y del mapa real de botones

### 5.1 Superficie real del panel

El panel operativo real reside en `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py`.

`main_panel.py` existe, pero solo como wrapper de nomenclatura y entry point alternativo; no contiene lógica operativa propia.

### 5.2 Grupos de botones presentes en la UI actual

Se identifican estos grupos reales de botones:

- arranque de sistema: Gazebo, bridge, rosbag, MoveIt y MoveIt bridge
- control manual UR5: `UR5 → HOME`, `UR5 → Mesa`, `UR5 → Cesta`, `Cerrar/Abrir gripper`
- baseline pick: `Agarre Objeto (Directo)`, `Iniciar APPROACH_COARSE`, `Agarre Objeto (MoveIT)`
- bloque TFM: `Caso TFM Memoria`, `Aplicar experimento`, `Inferir agarre`, `Comparar grasp/ref`, `Agarre objeto`, `Reset TFM`

### 5.3 Estado del gating UI

La lógica central de habilitación y bloqueo está en `panel_ui_state.py` y actualmente protege botones opcionales mediante `getattr(..., None)`.

Esto es relevante porque la capa de gating todavía contempla botones que no están presentes en la UI runtime actual:

- `btn_test_robot`
- `btn_tfm_publish`
- `btn_pick_demo2`

### 5.4 Conclusión del panel

- El crash por botones ausentes queda mitigado en la versión actual gracias al blindaje con atributos opcionales.
- La UI real está razonablemente alineada con el README ROS 2 y con el flujo actual del panel.
- Permanece, no obstante, una deriva menor entre la capa de gating genérica y la superficie efectiva de botones expuesta al usuario.

Veredicto del bloque panel: funcional y robustecido para runtime, con una inconsistencia residual de bajo impacto en la definición de botones opcionales.

## 6. Revisión del pipeline pick completo

### 6.1 Estado global

El pipeline pick DIRECTO tiene hoy dos zonas claramente distintas:

- zona delantera relativamente consolidada: arranque, approach, grasp, pre-close, close y attach
- zona trasera todavía no cerrada completamente: transporte a cesta, recoveries y vuelta final

### 6.2 Runner canónico

La validación operativa seria del flujo DIRECTO se articula sobre:

- `agarre_ros2_ws/scripts/run_directo_validation.sh`
- `agarre_ros2_ws/scripts/panel_runtime_validated.env`
- overlay de `install/setup.bash`

Esto importa porque el workspace ha mostrado contaminación real entre `src`, `build` e `install`. Por tanto, fuera del lanzador canónico y de un rebuild explícito, la verdad runtime puede no coincidir con el código fuente recién editado.

### 6.3 Estado técnico actual del pipeline

Hallazgos ya confirmados en esta línea de trabajo:

- el seed override de transporte quedó identificado y corregido en el slice correspondiente
- el conteo de segmentos de `TRANSPORT_PREP` quedó corregido
- el guard `wrong_direction_drift` fue suavizado para permitir detours iniciales sin falso fallo
- `RECOVER_2` ya dispone de micro-recovery local y cobertura unitaria
- tests del slice de transporte: `30 passed`
- build de `ur5_qt_panel`: OK

### 6.4 Bloqueo abierto real

El ciclo completo sigue sin validarse en runtime canónico porque el corredor vuelve a bloquear antes, en `CESTA_STAGE_1_RECOVER_1`, impidiendo ejercer el parche de `RECOVER_2`.

Consecuencia práctica:

- el fix de `RECOVER_2` no puede declararse validado de extremo a extremo
- `CESTA_RELEASE` y `HOME_FINAL` siguen fuera de alcance en la corrida canónica fallida reciente

### 6.5 Veredicto del pipeline pick

- El pipeline no está roto de forma generalizada: hay evidencia suficiente de que la parte delantera y varios slices de transporte están controlados y cubiertos.
- Tampoco está completamente cerrado para una defensa si se exige una demo robusta de cesta a home final bajo el runner canónico.

Veredicto del bloque pick: parcialmente validado, pero todavía con un bug abierto de severidad alta en el tramo de transporte a cesta.

## 7. Revisión del pipeline CNN

### 7.1 Estado del bloque experimental oficial

La separación conceptual y operativa está ahora bien explicitada:

- `EXP1..EXP4`: bloque experimental oficial de la memoria
- `EXP1.1` y `EXP1.2`: implementación en código del diseño ligero teórico de `4.6.2`
- `exp_methodology_v2.yaml` y `METHODOLOGY_ALIGNMENT.md`: vía posterior, más alineada metodológicamente, sin reescribir resultados oficiales

### 7.2 Coherencia actual observada

Se confirma coherencia entre:

- `agarre_inteligente/README.md`
- `agarre_inteligente/METHODOLOGY_ALIGNMENT.md`
- `agarre_inteligente/config/exp_methodology_v2.yaml`
- documentación específica de modelos bajo `agarre_inteligente/docs/modelos/`

La documentación deja claro que:

- los resultados oficiales del TFM siguen apoyándose en `SmoothL1Loss` e `iou_axis_aligned_boxes`
- la alineación metodológica posterior introduce `GraspLoss`, `EvaluatorOriented` e IoU orientada sin tocar los artefactos oficiales

### 7.3 Riesgo narrativo restante

Aunque el bloque CNN está técnicamente ordenado, hay un riesgo de defensa claro si no se verbaliza bien la separación entre:

- resultados publicados del TFM
- mejoras metodológicas incorporadas después en el repo

No es un fallo de implementación actual, pero sí un punto que debe quedar explicado con precisión para no mezclar “resultado oficial” con “mejora posterior del workspace”.

### 7.4 Split Cornell

El canon operativo del repo queda fijado en `3541/1569`.

Durante esta auditoría se detectó y corrigió una incoherencia viva en `reports/cornell_audit/README.md`, donde se mezclaba el conteo materializado `3542` con el dataset efectivamente utilizable. Tras la corrección:

- `3542` queda como huella de materialización previa al descarte
- `3541/1569` queda como referencia técnica efectiva para ejecución y validación

Veredicto del bloque CNN: consistente y defendible, con riesgo principal de interpretación y no de implementación.

## 8. Contraste de READMEs y documentos auxiliares

### 8.1 Alineaciones ya consolidadas

Los documentos canónicos principales quedan alineados en los puntos críticos:

- `README.md`: fija `./lanzar_panelc2.sh` como launcher canónico, señala `reports/TFM_Lozano_Rodriguez-Jesus.pdf` como PDF principal y explica el split `3541/1569`
- `agarre_ros2_ws/README.md`: mantiene el mismo launcher canónico y trata `lanzar_panelv2.sh` como alias de compatibilidad
- `reports/README.md`: identifica el PDF final del TFM y deja claro que `auditoria/` contiene la referencia runtime más reciente
- `agarre_inteligente/README.md`: separa resultados oficiales, experimentos auxiliares y metodología posterior

### 8.2 Material histórico

Persisten referencias a `lanzar_panelv2.sh` en documentación histórica, especialmente incidentes y reportes de cierre previos. En los casos localizados, el propio texto ya lo presenta como alias o como launcher usado en corridas históricas, no como canon vigente.

### 8.3 Conclusión documental

- El núcleo documental canónico está hoy razonablemente alineado.
- Los residuos principales quedan concentrados en material histórico o en documentos auxiliares secundarios, no en los README principales.
- La incoherencia viva más clara encontrada durante esta auditoría era la del `cornell_audit/README.md`, y ya ha sido corregida.

## 9. Síntesis final de riesgos y gaps

### 9.1 Riesgos altos

- El flujo DIRECTO completo hasta `CESTA_RELEASE` y `HOME_FINAL` no puede darse por cerrado en runtime canónico mientras siga abierto el bloqueo en `CESTA_STAGE_1_RECOVER_1`.
- Si la defensa depende de una demo completa de pick directo a cesta con retorno final, este es el principal frente técnico pendiente.

### 9.2 Riesgos medios

- Sigue existiendo riesgo estructural de contaminación entre `src`, `build` e `install`; el proyecto depende de usar el launcher y overlay canónicos para que la verdad runtime coincida con el código esperado.
- El panel arrastra una ligera deriva entre la lógica de gating y la superficie real de botones disponibles. Ahora es segura, pero evidencia deuda menor de consolidación UI.
- El bloque CNN exige una explicación disciplinada en defensa para separar resultados oficiales del TFM frente a mejoras metodológicas posteriores del repositorio.

### 9.3 Riesgos bajos o ya mitigados

- La jerarquía documental principal está alineada en launcher, PDF canónico y relación entre bloques del proyecto.
- La inconsistencia viva detectada en `reports/cornell_audit/README.md` ha quedado corregida.
- `main_panel.py` ya no es una fuente de ambigüedad real: existe como wrapper explícito y la lógica operativa sigue centralizada en `panel_v2.py`.

## 10. Conclusión ejecutiva

El proyecto no está en estado “caótico” ni pendiente de una reestructuración amplia. La mayor parte del trabajo de cierre grueso ya está hecha:

- documentación principal bastante alineada
- inventario ROS 2 acotado y entendible
- panel funcional con blindajes importantes ya introducidos
- bloque CNN técnicamente ordenado

Lo que separa el repo de un cierre limpio no es una lista larga de inconsistencias repartidas, sino un conjunto corto de puntos de control:

- cerrar el bug de transporte a cesta en el flujo DIRECTO canónico
- mantener disciplina estricta sobre launcher/overlay para evitar contaminación runtime
- defender con claridad la separación entre resultados oficiales y mejoras metodológicas posteriores

Prioridad recomendada antes de declarar el proyecto listo para defensa técnica:

1. Cerrar la validación runtime del flujo DIRECTO completo.
2. Mantener la ejecución final solo por `./lanzar_panelc2.sh` y rutas canónicas.
3. Preparar una explicación breve y consistente del bloque CNN oficial frente a la alineación metodológica posterior.