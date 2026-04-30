<!-- generado-articulo-tfm -->

# Figuras y tablas

## Figuras generadas para el articulo

| archivo | representa | fuente | experimento_modelo | apta_articulo | retoque |
| --- | --- | --- | --- | --- | --- |
| curvas_loss_EXP1.1_SIMPLEGRASP_RGB.png | Curvas train_loss y val_loss por seed. | agarre_inteligente/experiments/*/seed_*/metrics.csv | EXP1.1_SIMPLEGRASP_RGB | si, si se simplifica visualmente segun limite de paginas | posible reduccion a paneles agregados |
| curvas_loss_EXP1.2_SIMPLEGRASP_RGBD.png | Curvas train_loss y val_loss por seed. | agarre_inteligente/experiments/*/seed_*/metrics.csv | EXP1.2_SIMPLEGRASP_RGBD | si, si se simplifica visualmente segun limite de paginas | posible reduccion a paneles agregados |
| curvas_loss_EXP1_SIMPLE_RGB.png | Curvas train_loss y val_loss por seed. | agarre_inteligente/experiments/*/seed_*/metrics.csv | EXP1_SIMPLE_RGB | si, si se simplifica visualmente segun limite de paginas | posible reduccion a paneles agregados |
| curvas_loss_EXP2_SIMPLE_RGBD.png | Curvas train_loss y val_loss por seed. | agarre_inteligente/experiments/*/seed_*/metrics.csv | EXP2_SIMPLE_RGBD | si, si se simplifica visualmente segun limite de paginas | posible reduccion a paneles agregados |
| curvas_loss_EXP3_RESNET18_RGB_AUGMENT.png | Curvas train_loss y val_loss por seed. | agarre_inteligente/experiments/*/seed_*/metrics.csv | EXP3_RESNET18_RGB_AUGMENT | si, si se simplifica visualmente segun limite de paginas | posible reduccion a paneles agregados |
| curvas_loss_EXP4_RESNET18_RGBD.png | Curvas train_loss y val_loss por seed. | agarre_inteligente/experiments/*/seed_*/metrics.csv | EXP4_RESNET18_RGBD | si, si se simplifica visualmente segun limite de paginas | posible reduccion a paneles agregados |
| evolucion_val_success_todos.png | Media por epoca de val_success para experimentos con metricas disponibles. | agarre_inteligente/experiments/*/seed_*/metrics.csv | EXP1..EXP4 + EXP1.1/EXP1.2 si hay metricas | si | revisar densidad de leyenda |
| evolucion_val_iou_todos.png | Media por epoca de val_iou para experimentos con metricas disponibles. | agarre_inteligente/experiments/*/seed_*/metrics.csv | EXP1..EXP4 + EXP1.1/EXP1.2 si hay metricas | si | revisar densidad de leyenda |
| evolucion_val_angle_todos.png | Media por epoca de val_angle_deg para experimentos con metricas disponibles. | agarre_inteligente/experiments/*/seed_*/metrics.csv | EXP1..EXP4 + EXP1.1/EXP1.2 si hay metricas | si | revisar densidad de leyenda |
| best_epoch_success_barras.png | Media y desviacion por seed de val_success_mean. | best_epoch_summary.csv por experimento | todos los experimentos ejecutados | si | posible acortar nombres de experimentos |
| best_epoch_iou_barras.png | Media y desviacion por seed de val_iou_mean. | best_epoch_summary.csv por experimento | todos los experimentos ejecutados | si | posible acortar nombres de experimentos |
| best_epoch_angle_barras.png | Media y desviacion por seed de val_angle_deg_mean. | best_epoch_summary.csv por experimento | todos los experimentos ejecutados | si | posible acortar nombres de experimentos |
| best_epoch_loss_barras.png | Media y desviacion por seed de val_loss_mean. | best_epoch_summary.csv por experimento | todos los experimentos ejecutados | si | posible acortar nombres de experimentos |
| comparacion_modalidad_success.png | Comparacion de success final entre modelo ligero y ResNet18 por modalidad. | report/tables/cap5/Tabla_5-4 y best_epoch_summary oficiales | EXP1..EXP4 | si | no imprescindible |
| latencia_inferencia_log.png | Latencia media CPU/CUDA para batch 1. | report/tables/cap5/Tabla_5-3_medicion_de_latencia_de_inferencia_por_experimento_y_dispositivo.csv | EXP1..EXP4 | si | quizas separar CPU y CUDA si el venue exige legibilidad en B/N |
| tamano_modelo_parametros.png | Numero de parametros por checkpoint medido en benchmark. | Tabla de latencia capitulo 5 | EXP1..EXP4 | si | no imprescindible |
| delta_pre_post_retrain_success.png | Diferencia de val_success entre snapshot previo y resultados validados. | report/metrics/aggregated/chapter5_pre_vs_post_retrain_comparison.csv | EXP1..EXP4 | solo si se explica como control de reproducibilidad, no como resultado principal | decidir si incluir o dejar en material suplementario |

## Tablas nuevas generadas

- `recursos/tablas/inventario_modelos.csv`: modelos implementados y referencias legacy.
- `recursos/tablas/inventario_experimentos.csv`: configs y experimentos ejecutados/localizados.
- `recursos/tablas/resultados_best_epoch_por_seed.csv`: resultados por seed en best epoch.
- `recursos/tablas/resumen_metricas_por_experimento.csv`: medias y desviaciones por experimento.
- `recursos/tablas/latencia_inferencia_cap5.csv`: copia curada de la tabla de latencia del capitulo 5.
- `recursos/tablas/comparativa_modalidad_simplecnn_resnet18.csv`: comparativa oficial por modalidad.

## Figuras existentes reutilizables

- `report/figures/cap5/Ilustracion_5-10_exito_final_de_agarre_en_validacion_agregado_por_experimento.png`
- `report/figures/cap5/Ilustracion_5-11_iou_medio_final_en_validacion_agregado_por_experimento.png`
- `report/figures/cap5/Ilustracion_5-12_error_angular_medio_final_en_validacion_agregado_por_experimento.png`
- `report/figures/cap5/Ilustracion_5-17_resultado_de_inferencia_del_modelo_exp3_resnet18_rgb_augment_sobre_la_imagen_sim.png`
- `report/figures/cap5/Ilustracion_5-18_evidencia_funcional_adicional_del_pipeline_percepcion_publicacion_consumo_en_ros.png`

No se han copiado esas figuras para evitar duplicar artefactos pesados; estan inventariadas como fuentes reutilizables.
