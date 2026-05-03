# Resultados clave

## Tabla principal recomendada

| experimento | modelo | modalidad | success | IoU | error angular | val_loss |
| --- | --- | --- | --- | --- | --- | --- |
| `EXP1_SIMPLE_RGB` | `SimpleGraspCNN` | RGB | 0,2741 +/- 0,0440 | 0,2254 +/- 0,0496 | 17,1910 +/- 1,3745 | 0,0321 +/- 0,0008 |
| `EXP2_SIMPLE_RGBD` | `SimpleGraspCNN` | RGB-D | 0,3805 +/- 0,0051 | 0,3169 +/- 0,0033 | 17,8730 +/- 0,3608 | 0,0320 +/- 0,0001 |
| `EXP3_RESNET18_RGB_AUGMENT` | `ResNet18Grasp` | RGB | 0,6801 +/- 0,0245 | 0,4010 +/- 0,0046 | 10,6071 +/- 1,1800 | 0,0297 +/- 0,0007 |
| `EXP4_RESNET18_RGBD` | `ResNet18Grasp` | RGB-D | 0,6492 +/- 0,0102 | 0,3879 +/- 0,0148 | 11,4260 +/- 0,5339 | 0,0283 +/- 0,0006 |

## Lectura editorial

- Resultado con mas fuerza para congreso: `EXP3_RESNET18_RGB_AUGMENT` alcanza el mejor `val_success` medio.
- La arquitectura residual supera claramente a la CNN ligera bajo el protocolo oficial.
- RGB-D mejora a la CNN ligera, pero no supera a RGB con augmentation en ResNet.
- El error angular mas bajo tambien corresponde a `EXP3`, lo que refuerza precision geometrica.
- `EXP4` obtiene la menor `val_loss`, dato util para discutir que las metricas no siempre cuentan la misma historia.

## Resultados complementarios

`EXP1.1_SIMPLEGRASP_RGB` y `EXP1.2_SIMPLEGRASP_RGBD` iran dentro del articulo como tabla o parrafo complementario. Pueden servir para trazabilidad de variantes, pero no conviene mezclarlos con la tabla principal `EXP1..EXP4`.

| experimento | modelo | modalidad | success | IoU | error angular | val_loss |
| --- | --- | --- | --- | --- | --- | --- |
| `EXP1.1_SIMPLEGRASP_RGB` | `SimpleGrasp` | RGB | 0,3646 +/- 0,1014 | 0,2477 +/- 0,0808 | 14,5377 +/- 1,5389 | 0,0336 +/- 0,0024 |
| `EXP1.2_SIMPLEGRASP_RGBD` | `SimpleGrasp` | RGB-D | 0,4143 +/- 0,0404 | 0,3079 +/- 0,0517 | 17,1200 +/- 0,2046 | 0,0334 +/- 0,0006 |

## Fuentes

- `recursos/tablas/chapter5_experiment_summary_validated.csv`
- `agarre_inteligente/experiments/*/best_epoch_summary.csv`
- `report/tables/cap5/Tabla_5-2_resumen_de_metricas_finales_por_experimento_en_validacion.csv`
