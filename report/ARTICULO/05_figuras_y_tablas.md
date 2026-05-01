# Figuras y tablas

## Figuras recomendadas para el cuerpo principal

| archivo | uso recomendado | motivo editorial |
| --- | --- | --- |
| `best_epoch_success_barras.png` | Figura principal de resultados. | Resume la metrica central del articulo y permite identificar `EXP3` como mejor configuracion oficial. |
| `best_epoch_iou_barras.png` | Figura secundaria o panel combinado. | Contrasta success con solapamiento geometrico. |
| `best_epoch_angle_barras.png` | Figura secundaria o panel combinado. | Refuerza la lectura de precision angular. |
| `comparacion_modalidad_success.png` | Figura de discusion. | Ayuda a explicar que RGB-D no mejora de forma uniforme. |
| `evolucion_val_success_todos.png` | Suplementaria o figura de metodologia. | Muestra dinamica por epoca, pero puede ser densa para un resumen/articulo corto. |

## Figuras para material suplementario

| archivo | motivo |
| --- | --- |
| `curvas_loss_EXP1_SIMPLE_RGB.png` | Curva individual util para auditoria, no imprescindible en el cuerpo. |
| `curvas_loss_EXP2_SIMPLE_RGBD.png` | Curva individual util para auditoria, no imprescindible en el cuerpo. |
| `curvas_loss_EXP3_RESNET18_RGB_AUGMENT.png` | Puede incluirse si se necesita evidenciar estabilidad del mejor modelo. |
| `curvas_loss_EXP4_RESNET18_RGBD.png` | Curva individual util para auditoria, no imprescindible en el cuerpo. |
| `curvas_loss_EXP1.1_SIMPLEGRASP_RGB.png` | Solo si se discuten experimentos auxiliares. |
| `curvas_loss_EXP1.2_SIMPLEGRASP_RGBD.png` | Solo si se discuten experimentos auxiliares. |
| `evolucion_val_iou_todos.png` | Suplementaria por densidad visual. |
| `evolucion_val_angle_todos.png` | Suplementaria por densidad visual. |
| `best_epoch_loss_barras.png` | Puede usarse para matizar que menor perdida no equivale siempre a mayor success. |
| `latencia_inferencia_log.png` | Material secundario; no debe desplazar el foco metodologico. |
| `tamano_modelo_parametros.png` | Material secundario para trade-off de complejidad. |
| `delta_pre_post_retrain_success.png` | Solo si se explica como control de reproducibilidad, no como resultado principal. |

## Tablas recomendadas

- Tabla principal: resultados oficiales `EXP1..EXP4` con `success`, IoU, error angular y `val_loss`.
- Tabla metodologica: modelo, modalidad, augmentation, epocas, criterio y estado del experimento.
- Tabla suplementaria: experimentos auxiliares `EXP1.1/EXP1.2` y configuraciones metodologicas no consolidadas.

## Tablas disponibles

- `recursos/tablas/inventario_modelos.csv`: modelos implementados y referencias legacy.
- `recursos/tablas/inventario_experimentos.csv`: configs y experimentos ejecutados/localizados.
- `recursos/tablas/resultados_best_epoch_por_seed.csv`: resultados por seed en best epoch.
- `recursos/tablas/resumen_metricas_por_experimento.csv`: medias y desviaciones por experimento.
- `recursos/tablas/chapter5_experiment_summary_validated.csv`: resumen validado de `EXP1..EXP4`.
- `recursos/tablas/latencia_inferencia_cap5.csv`: tabla de latencia, recomendada como secundaria.
- `recursos/tablas/comparativa_modalidad_simplecnn_resnet18.csv`: comparativa oficial por modalidad.

## Criterio editorial

El cuerpo principal debe priorizar metodologia y resultados. Las figuras de latencia, tamano de modelo e integracion aplicada solo deben entrar si el limite de paginas lo permite o si el venue pide evidencia de aplicabilidad.
