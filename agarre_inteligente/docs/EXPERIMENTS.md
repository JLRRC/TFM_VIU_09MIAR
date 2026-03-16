# EXPERIMENTS

Registro consolidado de experimentos y resultados de `agarre_inteligente`.

## Estado

- Experimentos base completados: `EXP1..EXP4`
- Semillas por experimento: `n=3`
- Dataset: Cornell object-wise split (`train.csv`, `val.csv`)
- Evaluacion: protocolo Cornell con comparacion multi-GT por imagen

## Configuraciones ejecutadas

1. `EXP1_SIMPLE_RGB`
- Config: `config/exp1_simple_rgb.yaml`
- Modelo: `SimpleCNN`
- Modalidad: `RGB`
- Epocas: 10
- Seeds: `0,1,2`

2. `EXP2_SIMPLE_RGBD`
- Config: `config/exp2_simple_rgbd.yaml`
- Modelo: `SimpleCNN`
- Modalidad: `RGB-D`
- Epocas: 10
- Seeds: `0,1,2`

3. `EXP3_RESNET18_RGB_AUGMENT`
- Config: `config/exp3_resnet18_rgb_augment.yaml`
- Modelo: `ResNetGrasp`
- Modalidad: `RGB`
- Epocas: 50
- Seeds: `0,1,2`

4. `EXP4_RESNET18_RGBD`
- Config: `config/exp4_resnet18_rgbd.yaml`
- Modelo: `ResNetGrasp`
- Modalidad: `RGB-D`
- Epocas: 50
- Seeds: `0,1,2`

## Resultados agregados vigentes

Fuente: `reports/tables/summary_results.csv`

| Experimento | val_success_mean +- std | val_iou_mean +- std | val_angle_deg_mean +- std | val_loss_mean +- std |
|---|---|---|---|---|
| EXP1_SIMPLE_RGB | 0.2029 +- 0.0318 | 0.2327 +- 0.0183 | 19.93 +- 0.72 | 0.0766 +- 0.0528 |
| EXP2_SIMPLE_RGBD | 0.3786 +- 0.0081 | 0.3236 +- 0.0058 | 17.88 +- 0.39 | 0.0322 +- 0.0002 |
| EXP3_RESNET18_RGB_AUGMENT | 0.6769 +- 0.0303 | 0.3809 +- 0.0129 | 10.33 +- 1.44 | 0.0292 +- 0.0005 |
| EXP4_RESNET18_RGBD | 0.6541 +- 0.0137 | 0.3820 +- 0.0044 | 11.66 +- 1.21 | 0.0286 +- 0.0011 |

## Interpretacion rapida

- `ResNetGrasp` supera a `SimpleCNN` en ambas modalidades.
- En `SimpleCNN`, anadir profundidad mejora claramente (`EXP2 > EXP1`).
- En `ResNetGrasp`, `RGB + augmentation` y `RGB-D` quedan cercanos en success rate.

## Artefactos clave

- Metricas por seed: `experiments/<EXP>/seed_<N>/metrics.csv`
- Seleccion por experimento: `experiments/<EXP>/best_epoch_summary.csv`
- Resumen global: `reports/tables/summary_results.csv`
- Tablas: `reports/tables/table_*.csv`
- Figuras: `reports/figures/*.png`

## Nota sobre historial

Documentacion historica y resultados previos se mantienen en `OLD/`.
Ese contenido no representa el estado final actual del pipeline.

**Ultima actualizacion:** 2026-03-07
