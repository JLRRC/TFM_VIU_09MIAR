# Reentrenamiento y Reevaluacion - 2026-03-07

## Estado

Completado.

## Resumen tecnico

Durante la reconstruccion se identificaron dos puntos clave:

1. Escalado de coordenadas GT al tamano de entrada (`224x224`) para consistencia entre imagen y target.
2. Correccion del evaluador Cornell para comparar cada prediccion contra todos los grasps GT de la imagen (multi-GT).

La reevaluacion de checkpoints existentes se realizo sin reentrenar modelos.

## Configuracion final ejecutada

- `EXP1_SIMPLE_RGB`: 3 seeds, 10 epocas
- `EXP2_SIMPLE_RGBD`: 3 seeds, 10 epocas
- `EXP3_RESNET18_RGB_AUGMENT`: 3 seeds, 50 epocas
- `EXP4_RESNET18_RGBD`: 3 seeds, 50 epocas

## Resultados finales (media por experimento)

Fuente: `reports/tables/summary_results.csv`

- `EXP1_SIMPLE_RGB`: val_success `0.2029`, val_iou `0.2327`, val_angle_deg `19.93`
- `EXP2_SIMPLE_RGBD`: val_success `0.3786`, val_iou `0.3236`, val_angle_deg `17.88`
- `EXP3_RESNET18_RGB_AUGMENT`: val_success `0.6769`, val_iou `0.3809`, val_angle_deg `10.33`
- `EXP4_RESNET18_RGBD`: val_success `0.6541`, val_iou `0.3820`, val_angle_deg `11.66`

## Artefactos verificados

- `experiments/EXP*/seed_*/metrics.csv`
- `experiments/EXP*/best_epoch_summary.csv`
- `reports/tables/summary_results.csv`
- `reports/figures/*.png`
- `reports/tables/table_*.csv`

Validacion ejecutada:

```bash
python scripts/validate_artifacts.py --strict
```

Resultado:

- `[OK] Artefactos minimos presentes`

## Nota historica

La carpeta `OLD/` conserva evidencia historica y comparativas previas.
No debe usarse como fuente del estado final vigente.
