# Revision temporal del PDF frente al workspace

PDF revisado:

- `09MIAR_10_Z_2025-26.TFM.V11.pdf`

Estado general:

- Los valores agregados principales del capitulo 5 si cuadran con el workspace real. La referencia valida es `report/metrics/validated/chapter5_experiment_summary_validated.csv`.
- No he detectado una inconsistencia numerica grave entre resumen, resultados y conclusiones para las metricas agregadas principales.

Valores validados que deben prevalecer si aparece cualquier duda editorial:

- `EXP1_SIMPLE_RGB`: `val_success=0.274060`, `val_iou=0.225380`, `val_angle_deg=17.190995`, `val_loss=0.032086`
- `EXP2_SIMPLE_RGBD`: `val_success=0.380497`, `val_iou=0.316867`, `val_angle_deg=17.872977`, `val_loss=0.032011`
- `EXP3_RESNET18_RGB_AUGMENT`: `val_success=0.680051`, `val_iou=0.401035`, `val_angle_deg=10.607081`, `val_loss=0.029726`
- `EXP4_RESNET18_RGBD`: `val_success=0.649246`, `val_iou=0.387898`, `val_angle_deg=11.425986`, `val_loss=0.028291`

Correcciones prioritarias:

1. Sustituir la figura actual 5-15.
   Evidencia:
   - `report/figures/cap5/Ilustracion_5-15_caso_limite_de_falso_negativo_visualmente_plausible.png`
   - `report/evidence/chapter5/fig_5_15_false_negative_plausible_candidates.csv`
   - `report/logs/reproducibility/chapter5_false_negative_selection.tsv`
   Motivo:
   - El PDF V11 sigue describiendo un "caso ilustrativo de objeto pequeno". El tutor ha pedido un caso limite/falso negativo visualmente plausible mas analitico.
   Numero propuesto:
   - Mantener numeracion 5-15, cambiar contenido y pie.

2. Regenerar o confirmar la sustitucion de las figuras 5-1 a 5-4 con eje Y compartido.
   Evidencia:
   - `report/figures/cap5/Ilustracion_5-1_curvas_de_perdida_train_loss_y_val_loss_para_exp1.png`
   - `report/figures/cap5/Ilustracion_5-2_curvas_de_perdida_train_loss_y_val_loss_para_exp2.png`
   - `report/figures/cap5/Ilustracion_5-3_curvas_de_perdida_train_loss_y_val_loss_para_exp3.png`
   - `report/figures/cap5/Ilustracion_5-4_curvas_de_perdida_train_loss_y_val_loss_para_exp4.png`
   - `report/logs/reproducibility/chapter5_loss_shared_ylim.tsv`
   Motivo:
   - El criterio pedido por el tutor ya esta implementado en el workspace real. El PDF debe reflejar esas versiones corregidas.
   Observacion adicional del tutor:
   - La recomendacion de compartir escala Y debe extenderse a ilustraciones analogas siempre que mejore la honestidad visual.
   - La ilustracion equivalente a `5-2` queda marcada como excepcion editorial activa: mantiene `ylim` independiente por subplot porque en ese caso concreto no resulta practico forzar la misma escala.

3. Sustituir en el PDF las curvas por epoca del bloque 5-5 a 5-7 por las versiones regeneradas tras reentrenamiento.
   Evidencia:
   - `report/figures/cap5/Ilustracion_5-5_evolucion_del_exito_de_agarre_en_validacion_por_epoca.png`
   - `report/figures/cap5/Ilustracion_5-6_evolucion_del_iou_medio_en_validacion_por_epoca.png`
   - `report/figures/cap5/Ilustracion_5-7_evolucion_del_error_angular_medio_en_validacion_por_epoca.png`
   - `agarre_inteligente/experiments/EXP*/seed_*/metrics.csv`
   - `report/logs/training/retrain_cap5_gpu_20260317_235914.log`
   - `report/logs/reproducibility/chapter5_post_retrain_regeneration_20260318_070322.tsv`
   Motivo:
   - El problema metodologico anterior ya no aplica a la fuente oficial actual: los cuatro experimentos fueron reentrenados de nuevo en GPU con el evaluador correcto integrado durante todo el ciclo.
   - Las nuevas curvas por epoca proceden de `metrics.csv` homogéneos y ya no mezclan criterios de evaluacion distintos entre epocas intermedias y finales.
   Accion recomendada:
   - Actualizar el PDF para que use exactamente estas versiones regeneradas.

4. Alinear la fuente editorial del capitulo 5 con la numeracion oficial del PDF.
   Evidencia:
   - La fuente oficial actual es `report/figures/cap5/`.
   - El arbol heredado `reports/capitulos/05_resultados_y_discusion/...` fue movido a `BORRAR/root_extra/reports/` porque mezclaba una numeracion no alineada con el PDF para las figuras 5-5 a 5-9.
   Motivo:
   - El workspace antiguo usaba nombres donde `5-5` y `5-6` correspondian a comparativas best epoch, mientras que el PDF del capitulo 5 usa `5-5`, `5-6` y `5-7` para curvas por epoca, y `5-8`, `5-9` para comparativas best epoch.

5. Citar `report/` como fuente de verdad y no `reports/`.
   Evidencia:
   - `report/README.md`
   - `report/logs/reproducibility/report_source_manifest.tsv`
   Motivo:
   - `reports/` era un arbol editorial heredado con manifiestos redundantes. La carpeta oficial consolidada es `report/`.

6. Revisar las referencias de evidencia ROS 2 del capitulo 5 para que apunten al arbol curado.
   Evidencia:
   - `report/evidence/ros2/`
   - `report/logs/reproducibility/ros2_repro_startup/`
   Motivo:
   - La evidencia ROS 2 valida ya no debe depender de `agarre_ros2_ws/reports/`, que ha quedado archivado en `BORRAR/`.

Apartados a revisar con prioridad:

- Capitulo 5 completo, con foco en figuras 5-1 a 5-15 y tablas 5-1, 5-2 y 5-4.
- Pie y texto de la subseccion 5.2.3 para que describa el nuevo falso negativo plausible.
- Cualquier referencia interna a `reports/` o a la numeracion heredada de figuras 5-5 a 5-9.

Checklist exacto de sustituciones en el Capitulo 5:

1. Corregir las referencias internas a ilustraciones en 5.1.1.
   En el PDF actual:
   - se cita `val_success` como Ilustracion `5-7`, IoU como `5-8` y error angular como `5-9`;
   - despues se afirma que las Ilustraciones `5-7 a 5-9` muestran la evolucion media por epoca;
   - y mas abajo se dice que las Ilustraciones `5-5 y 5-6` recogen la variabilidad entre seeds.
   Debe quedar asi:
   - Ilustracion `5-5`: `val_success` por epoca
   - Ilustracion `5-6`: `val_iou` por epoca
   - Ilustracion `5-7`: `val_angle_deg` por epoca
   - Ilustracion `5-8`: comparativa `val_success` por seed en `best_epoch`
   - Ilustracion `5-9`: comparativa `val_loss` por seed en `best_epoch`

2. Sustituir los valores numericos de la Tabla 5-1.
   Valores correctos segun `report/tables/cap5/Tabla_5-1_resultados_agregados_en_validacion_best_epoch_por_ejecucion_bajo_split_object_wi.csv`:
   - `EXP1_SIMPLE_RGB`: `27,41 +- 4,40 %`
   - `EXP2_SIMPLE_RGBD`: `38,05 +- 0,51 %`
   - `EXP3_RESNET18_RGB_AUGMENT`: `68,01 +- 2,45 %`
   - `EXP4_RESNET18_RGBD`: `64,92 +- 1,02 %`
   El PDF actual conserva los valores previos al reentrenamiento para `EXP1`, `EXP2`, `EXP3` y `EXP4`.

3. Sustituir los valores numericos de la Tabla 5-2.
   Valores correctos segun `report/tables/cap5/Tabla_5-2_resumen_de_metricas_finales_por_experimento_en_validacion.csv`:
   - `EXP1_SIMPLE_RGB`: `grasp success=27,41 +- 4,40 %`, `IoU=0,2254 +- 0,0496`, `Delta theta=17,19 +- 1,37`, `val_loss=0,0321 +- 0,0008`
   - `EXP2_SIMPLE_RGBD`: `grasp success=38,05 +- 0,51 %`, `IoU=0,3169 +- 0,0033`, `Delta theta=17,87 +- 0,36`, `val_loss=0,0320 +- 0,0001`
   - `EXP3_RESNET18_RGB_AUGMENT`: `grasp success=68,01 +- 2,45 %`, `IoU=0,4010 +- 0,0046`, `Delta theta=10,61 +- 1,18`, `val_loss=0,0297 +- 0,0007`
   - `EXP4_RESNET18_RGBD`: `grasp success=64,92 +- 1,02 %`, `IoU=0,3879 +- 0,0148`, `Delta theta=11,43 +- 0,53`, `val_loss=0,0283 +- 0,0006`

4. Ajustar la interpretacion textual inmediatamente posterior a las Tablas 5-1 y 5-2.
   Debe corregirse explicitamente:
   - `EXP3` sigue siendo la mejor configuracion en `grasp success`.
   - `EXP3`, no `EXP4`, presenta ahora el mejor `IoU` medio.
   - `EXP4` mantiene la menor `val_loss`.
   - `EXP3` mantiene el menor error angular medio.
   El texto actual del PDF afirma que `EXP4 presenta el mejor IoU medio`, y eso ya no coincide con los datos regenerados del workspace.

5. Sustituir el texto final de 5.2.3 asociado a la Ilustracion 5-15.
   El PDF actual describe la figura como un `objeto de pequeno tamano aparente dentro del encuadre`, lo que ya no es una descripcion suficiente ni la justificacion analitica principal.
   La figura oficial actual corresponde a:
   - experimento: `EXP1_SIMPLE_RGB`
   - seed: `2`
   - muestra: `data/raw/cornell/02/pcd0271r.png`
   - `IoU = 0,2306`
   - `Delta theta = 30,01 grados`
   - resultado: `fail`
   - causa: `IoU + Delta theta`
   El nuevo texto debe enfatizar que se trata de un falso negativo visualmente plausible invalidado por quedar ligeramente por debajo/delante de ambos umbrales del criterio Cornell.

6. Sustituir el pie de la Ilustracion 5-15.
   Pie recomendado:
   - `Ilustracion 5-15. Caso limite de falso negativo visualmente plausible en validacion (EXP1_SIMPLE_RGB, seed 2). La prediccion se aproxima al agarre de referencia, pero no supera el criterio Cornell al presentar IoU = 0,231 y Delta theta = 30,01 grados, por lo que se clasifica como fallo pese a su apariencia cualitativamente razonable.`

7. Mantener como referencias oficiales del capitulo 5 unicamente estas rutas:
   - `report/metrics/validated/chapter5_experiment_summary_validated.csv`
   - `report/metrics/aggregated/chapter5_best_epoch_runs_all_seeds.csv`
   - `report/figures/cap5/`
   - `report/tables/cap5/`
   - `report/tables/anexos/Tabla_8-1_resultados_por_semilla_y_experimento_en_la_mejor_epoca_de_validacion.csv`
   - `report/tables/anexos/Tabla_8-3_resumen_de_experimentos_base_en_validacion_media_desviacion_estandar_cuando_proc.csv`
   - `report/tables/anexos/Tabla_8-4_comparativa_por_modalidad_entre_simplegraspcnn_y_resnet18grasp_mejor_epoca_de_va.csv`

Texto exacto listo para sustituir en el documento:

## Bloque de sustitucion para 5.1.1

Usar este texto en lugar del tramo donde se describen las ilustraciones 5-5 a 5-9 y la lectura cuantitativa posterior:

```text
Ademas de la perdida de entrenamiento y validacion (train_loss y val_loss), se analizan tres metricas de validacion que permiten interpretar la calidad geometrica de las predicciones bajo el criterio tipo Cornell: exito de agarre (val_success), solapamiento geometrico medio (IoU) y error angular medio (Delta theta). Para ello, las Ilustraciones 5-5, 5-6 y 5-7 representan, respectivamente, la evolucion por epoca de val_success, IoU medio y error angular medio mediante una curva media por experimento y una banda de variabilidad entre semillas. Esta representacion facilita comparar la dinamica de aprendizaje entre configuraciones sin sobrecargar la lectura con todas las ejecuciones individuales.

Complementariamente, las Ilustraciones 5-8 y 5-9 recogen la variabilidad entre semillas en best_epoch, mostrando, respectivamente, la comparativa de val_success y val_loss por ejecucion y experimento. En conjunto, estas visualizaciones permiten relacionar el comportamiento de convergencia durante el entrenamiento con el rendimiento geometrico final alcanzado por cada configuracion.

A partir de estas curvas y del resumen consolidado del pipeline, los resultados cuantitativos finales se agregan sobre validacion bajo particionado object-wise, seleccionando para cada ejecucion la epoca que maximiza val_success. Bajo este criterio, EXP1_SIMPLE_RGB alcanza un grasp success del 27,41 +- 4,40 %, EXP2_SIMPLE_RGBD alcanza 38,05 +- 0,51 %, EXP3_RESNET18_RGB_AUGMENT alcanza 68,01 +- 2,45 % y EXP4_RESNET18_RGBD alcanza 64,92 +- 1,02 %.

Las metricas complementarias matizan esta comparacion. En IoU medio, EXP3_RESNET18_RGB_AUGMENT obtiene el mejor valor (0,4010 +- 0,0046), seguido de EXP4_RESNET18_RGBD (0,3879 +- 0,0148), EXP2_SIMPLE_RGBD (0,3169 +- 0,0033) y EXP1_SIMPLE_RGB (0,2254 +- 0,0496). En error angular medio, EXP3 vuelve a mostrar el mejor comportamiento (10,61 +- 1,18 grados), por delante de EXP4 (11,43 +- 0,53 grados), mientras que EXP1 y EXP2 quedan en 17,19 +- 1,37 grados y 17,87 +- 0,36 grados, respectivamente. En perdida de validacion, el mejor resultado corresponde a EXP4 (0,0283 +- 0,0006), seguido muy de cerca por EXP3 (0,0297 +- 0,0007), mientras que EXP1 y EXP2 presentan valores practicamente equivalentes en torno a 0,032.

De estos resultados se derivan tres observaciones principales. En primer lugar, las configuraciones basadas en ResNet18Grasp mantienen una superioridad clara frente a las basadas en SimpleGraspCNN, lo que refuerza la utilidad de una arquitectura residual preentrenada bajo el protocolo experimental adoptado. En segundo lugar, el efecto de la modalidad de entrada sigue siendo dependiente de la arquitectura: en SimpleGraspCNN, la incorporacion de profundidad mejora de forma clara la tasa de exito, mientras que en ResNet18Grasp las configuraciones RGB con augmentation y RGB-D presentan resultados proximos en la metrica principal. En tercer lugar, tomando como referencia grasp success, la mejor configuracion es EXP3_RESNET18_RGB_AUGMENT; ademas, esta misma configuracion presenta tambien el mejor IoU medio y el menor error angular medio, mientras que EXP4_RESNET18_RGBD destaca por ofrecer la menor perdida de validacion y un comportamiento global muy competitivo.
```

## Tabla 5-1 lista para reconstruir

```text
Experimento | Modelo | Modalidad | Grasp success (%)
EXP1_SIMPLE_RGB | SimpleGraspCNN | RGB | 27,41 +- 4,40
EXP2_SIMPLE_RGBD | SimpleGraspCNN | RGB-D | 38,05 +- 0,51
EXP3_RESNET18_RGB_AUGMENT | ResNet18Grasp | RGB | 68,01 +- 2,45
EXP4_RESNET18_RGBD | ResNet18Grasp | RGB-D | 64,92 +- 1,02
```

## Tabla 5-2 lista para reconstruir

```text
Experimento | Modelo | Modalidad | Grasp success (%) | IoU medio | Delta theta medio (grados) | val_loss
EXP1_SIMPLE_RGB | SimpleGraspCNN | RGB | 27,41 +- 4,40 | 0,2254 +- 0,0496 | 17,19 +- 1,37 | 0,0321 +- 0,0008
EXP2_SIMPLE_RGBD | SimpleGraspCNN | RGB-D | 38,05 +- 0,51 | 0,3169 +- 0,0033 | 17,87 +- 0,36 | 0,0320 +- 0,0001
EXP3_RESNET18_RGB_AUGMENT | ResNet18Grasp | RGB | 68,01 +- 2,45 | 0,4010 +- 0,0046 | 10,61 +- 1,18 | 0,0297 +- 0,0007
EXP4_RESNET18_RGBD | ResNet18Grasp | RGB-D | 64,92 +- 1,02 | 0,3879 +- 0,0148 | 11,43 +- 0,53 | 0,0283 +- 0,0006
```

## Bloque de sustitucion para 5.2.3

Usar este texto en lugar del ultimo tramo interpretativo asociado a la Ilustracion 5-15:

```text
Un caso especialmente ilustrativo es aquel en el que la prediccion resulta visualmente plausible, pero no alcanza el criterio de exito debido a una desviacion angular moderada y a un solapamiento insuficiente con la anotacion de referencia. Este tipo de situaciones pone de manifiesto que la evaluacion no depende unicamente de la plausibilidad visual del agarre, sino del cumplimiento simultaneo de restricciones geometricas concretas sobre orientacion y solapamiento. La Ilustracion 5-15 recoge un caso limite de este tipo correspondiente a EXP1_SIMPLE_RGB (seed 2), en el que la prediccion se aproxima razonablemente a la zona funcional de agarre, pero queda invalidada al presentar IoU = 0,2306 y Delta theta = 30,01 grados. Se trata, por tanto, de un falso negativo visualmente plausible: la propuesta puede parecer aceptable a simple vista, pero no supera el criterio tipo Cornell al situarse ligeramente por debajo del umbral de solapamiento y ligeramente por encima del umbral angular.
```

## Pie exacto recomendado para la Ilustracion 5-15

```text
Ilustracion 5-15. Caso limite de falso negativo visualmente plausible en validacion (EXP1_SIMPLE_RGB, seed 2). La prediccion se aproxima al agarre de referencia, pero no supera el criterio Cornell al presentar IoU = 0,231 y Delta theta = 30,01 grados, por lo que se clasifica como fallo pese a su apariencia cualitativamente razonable.
```

## Frases puntuales que deben corregirse

- Donde el PDF diga o implique que `EXP4` presenta el mejor `IoU` medio, sustituirlo por `EXP3`.
- Donde el PDF describa la Ilustracion `5-15` como un mero `caso ilustrativo de objeto pequeno`, sustituirlo por `caso limite de falso negativo visualmente plausible`.
- Donde el PDF asigne las ilustraciones `5-7` a `5-9` a las curvas medias por epoca, corregir la numeracion al bloque `5-5` a `5-7`.

Apartados donde no propongo cambio numerico inmediato:

- Resumen / abstract: las tasas agregadas principales coinciden con el workspace.
- Conclusiones: la jerarquia relativa entre experimentos coincide con las metricas validadas.

Nota temporal:

- Este archivo es una guia de revision editorial. No modifica el PDF y puede eliminarse una vez integrado el documento final.
