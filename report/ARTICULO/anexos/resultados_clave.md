# Resultados clave

## Tabla principal recomendada para el articulo

| experimento | modelo | modalidad | success | IoU | error angular | val_loss |
| --- | --- | --- | --- | --- | --- | --- |
| `EXP1_SIMPLE_RGB` | `SimpleGraspCNN` | RGB | 0,2741 +/- 0,0440 | 0,2254 +/- 0,0496 | 17,1910 +/- 1,3745 | 0,0321 +/- 0,0008 |
| `EXP2_SIMPLE_RGBD` | `SimpleGraspCNN` | RGB-D | 0,3805 +/- 0,0051 | 0,3169 +/- 0,0033 | 17,8730 +/- 0,3608 | 0,0320 +/- 0,0001 |
| `EXP3_RESNET18_RGB_AUGMENT` | `ResNet18Grasp` | RGB | 0,6801 +/- 0,0245 | 0,4010 +/- 0,0046 | 10,6071 +/- 1,1800 | 0,0297 +/- 0,0007 |
| `EXP4_RESNET18_RGBD` | `ResNet18Grasp` | RGB-D | 0,6492 +/- 0,0102 | 0,3879 +/- 0,0148 | 11,4260 +/- 0,5339 | 0,0283 +/- 0,0006 |

## Lectura principal

- Mejor resultado medio oficial: `EXP3_RESNET18_RGB_AUGMENT` con `val_success` 0,6801.
- `ResNet18Grasp` supera a `SimpleGraspCNN` en las dos modalidades oficiales.
- En ResNet, RGB con augmentation (`EXP3`) supera a RGB-D sin augmentation (`EXP4`) en `val_success` medio.
- En la CNN ligera oficial, RGB-D con augmentation (`EXP2`) supera a RGB sin augmentation (`EXP1`) en success, pero no en error angular.
- La modalidad de entrada y la augmentation deben discutirse como factores dependientes de la arquitectura.

## Resultados auxiliares

`EXP1.1_SIMPLEGRASP_RGB` y `EXP1.2_SIMPLEGRASP_RGBD` quedan como evidencia complementaria de una variante ligera posterior. No deben mezclarse con la tabla principal salvo que el articulo incluya una seccion explicita de material suplementario.

## Fuentes

- `recursos/tablas/chapter5_experiment_summary_validated.csv`
- `agarre_inteligente/experiments/*/best_epoch_summary.csv`
- `report/tables/cap5/Tabla_5-2_resumen_de_metricas_finales_por_experimento_en_validacion.csv`
