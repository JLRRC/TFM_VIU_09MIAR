<!-- generado-articulo-tfm -->

# Resultados clave

| experimento | modelo | modalidad | estado | success | IoU | error angular | val_loss |
| --- | --- | --- | --- | --- | --- | --- | --- |
| EXP1.1_SIMPLEGRASP_RGB | SimpleGrasp | RGB | auxiliar | 0.3646 +/- 0.1014 | 0.2477 +/- 0.0808 | 14.5377 +/- 1.5389 | 0.0336 +/- 0.0024 |
| EXP1.2_SIMPLEGRASP_RGBD | SimpleGrasp | RGB-D | auxiliar | 0.4143 +/- 0.0404 | 0.3079 +/- 0.0517 | 17.1200 +/- 0.2046 | 0.0334 +/- 0.0006 |
| EXP1_SIMPLE_RGB | SimpleGraspCNN | RGB | oficial | 0.2741 +/- 0.0539 | 0.2254 +/- 0.0608 | 17.1910 +/- 1.6834 | 0.0321 +/- 0.0010 |
| EXP2_SIMPLE_RGBD | SimpleGraspCNN | RGB-D | oficial | 0.3805 +/- 0.0063 | 0.3169 +/- 0.0041 | 17.8730 +/- 0.4418 | 0.0320 +/- 0.0002 |
| EXP3_RESNET18_RGB_AUGMENT | ResNet18Grasp | RGB | final | 0.6801 +/- 0.0300 | 0.4010 +/- 0.0057 | 10.6071 +/- 1.4451 | 0.0297 +/- 0.0008 |
| EXP4_RESNET18_RGBD | ResNet18Grasp | RGB-D | oficial | 0.6492 +/- 0.0125 | 0.3879 +/- 0.0182 | 11.4260 +/- 0.6538 | 0.0283 +/- 0.0008 |

## Lectura principal

- Mejor resultado medio ejecutado: `EXP3_RESNET18_RGB_AUGMENT` con `val_success` 0.6801.
- Caso final documentado para inferencia reproducible: `EXP3_RESNET18_RGB_AUGMENT`.
- `ResNet18Grasp` supera a `SimpleGraspCNN` en las dos modalidades oficiales.
- En ResNet, RGB con augmentation (`EXP3`) supera a RGB-D sin augmentation (`EXP4`) en `val_success` medio.
- En la CNN ligera oficial, RGB-D con augmentation (`EXP2`) supera a RGB sin augmentation (`EXP1`).

## Fuentes

- `agarre_inteligente/experiments/*/best_epoch_summary.csv`
- `report/metrics/validated/chapter5_experiment_summary_validated.csv`
- `report/tables/cap5/Tabla_5-2_resumen_de_metricas_finales_por_experimento_en_validacion.csv`
