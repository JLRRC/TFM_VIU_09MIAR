# Figuras y tablas

## Figuras esenciales para el articulo

| archivo | papel en el articulo | motivo |
| --- | --- | --- |
| `best_epoch_success_barras.png` | Figura principal. | Da el gancho cuantitativo: `EXP3_RESNET18_RGB_AUGMENT` destaca en success medio. |
| `best_epoch_iou_barras.png` | Figura de precision geometrica. | Complementa success con solapamiento de rectangulos. |
| `best_epoch_angle_barras.png` | Figura de precision angular. | Refuerza la lectura geometrica del grasp. |
| `comparacion_modalidad_success.png` | Figura interpretativa. | Ayuda a contar RGB vs RGB-D y arquitectura ligera vs residual. |

## Tablas esenciales

- Tabla cuantitativa `EXP1..EXP4`: success, IoU, error angular y `val_loss`.
- Tabla metodologica: modelo, modalidad, augmentation, epocas, criterio y estado.
- Tabla de eficiencia: latencia CPU/CUDA, parametros y tamano, si el limite de paginas lo permite.

## Figuras de apoyo

| archivo | uso recomendado |
| --- | --- |
| `evolucion_val_success_todos.png` | Mostrar dinamica de aprendizaje si hay espacio. |
| `evolucion_val_iou_todos.png` | Apoyo para comportamiento geometrico por epoca. |
| `evolucion_val_angle_todos.png` | Apoyo para estabilidad angular por epoca. |
| `best_epoch_loss_barras.png` | Matizar que menor perdida no siempre implica mayor success. |
| `curvas_loss_EXP3_RESNET18_RGB_AUGMENT.png` | Evidenciar estabilidad del mejor modelo. |
| `curvas_loss_EXP4_RESNET18_RGBD.png` | Comparar contra la segunda configuracion fuerte. |

## Figuras de apoyo y auditoria

| archivo | criterio |
| --- | --- |
| `curvas_loss_EXP1_SIMPLE_RGB.png` | Util para trazabilidad, poco atractiva como figura principal. |
| `curvas_loss_EXP2_SIMPLE_RGBD.png` | Figura de apoyo si se discute la CNN ligera oficial. |
| `curvas_loss_EXP1.1_SIMPLEGRASP_RGB.png` | Puede usarse en el bloque complementario de `EXP1.1/EXP1.2`. |
| `curvas_loss_EXP1.2_SIMPLEGRASP_RGBD.png` | Puede usarse en el bloque complementario de `EXP1.1/EXP1.2`. |
| `delta_pre_post_retrain_success.png` | Solo tiene sentido si se explica como control de reproducibilidad. |

## Figuras de eficiencia y aplicabilidad

| archivo | uso recomendado |
| --- | --- |
| `latencia_inferencia_log.png` | Breve apoyo sobre eficiencia; no debe desplazar el foco experimental. |
| `tamano_modelo_parametros.png` | Util para trade-off de complejidad, preferiblemente junto a latencia. |

## Figuras con gancho para congreso

1. `best_epoch_success_barras.png`: comunica rapidamente el resultado principal.
2. `comparacion_modalidad_success.png`: convierte el articulo en una historia sobre decisiones experimentales.
3. `best_epoch_angle_barras.png`: muestra que no solo importa acertar, sino acertar geometricamente.
4. Evidencia visual de inferencia/integracion funcional, si se incorpora desde figuras externas inventariadas, debe ocupar un cierre breve.

## Piezas pendientes o deseables

- Analisis cualitativo de aciertos y fallos: falta seleccionar ejemplos visuales concretos si se quiere incluir.
- Figura combinada success + IoU + error angular: podria mejorar legibilidad frente a tres graficas separadas.
- Tabla compacta de eficiencia: recomendable si el articulo discute aplicabilidad funcional.
