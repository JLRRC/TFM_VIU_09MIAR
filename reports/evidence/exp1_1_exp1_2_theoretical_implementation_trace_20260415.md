# EXP1.1 y EXP1.2 como implementacion teorica de 4.6.2

Fecha: 2026-04-15

## Alcance

Este artefacto deja constancia de que `EXP1.1` y `EXP1.2` existen en el workspace como soporte de implementacion del diseño teorico descrito en el apartado `4.6.2` de la memoria, sin sustituir la base experimental oficial del documento.

## Configuraciones asociadas

- `agarre_inteligente/config/exp1_1_simplegrasp_rgb.yaml`
- `agarre_inteligente/config/exp1_2_simplegrasp_rgbd.yaml`

## Correspondencia con la memoria

- `EXP1.1_SIMPLEGRASP_RGB` implementa la variante `SimpleGrasp` en modalidad RGB.
- `EXP1.2_SIMPLEGRASP_RGBD` implementa la variante `SimpleGrasp` en modalidad RGB-D.
- Ambas variantes materializan en codigo la arquitectura objetivo descrita en `4.6.2`.
- Los resultados oficiales de la memoria se mantienen, no obstante, sobre `EXP1`, `EXP2`, `EXP3` y `EXP4`, porque son los experimentos realmente consolidados en las tablas y figuras finales.

## Resumen operativo actual

- `EXP1.1`: `agarre_inteligente/experiments/EXP1.1_SIMPLEGRASP_RGB/best_epoch_summary.csv`
- `EXP1.2`: `agarre_inteligente/experiments/EXP1.2_SIMPLEGRASP_RGBD/best_epoch_summary.csv`
- En ambos casos, el panel los trata como variantes auxiliares de inferencia.
- Cuando `val_success` no discrimina entre seeds, el workspace prioriza `val_loss` para seleccionar una seed utilizable en panel.

## Restriccion de uso en recreacion oficial

- `EXP1.1` y `EXP1.2` no deben entrar en la regeneracion oficial de:
  - `reports/tables/summary_results.csv`
  - `reports/tables/results_by_seed.csv`
  - figuras y tablas del bloque oficial del TFM
- El validador `agarre_inteligente/scripts/validate_official_scope.py` se encarga de comprobar esa separacion.
