# Experiments Directory

Esta carpeta contiene artefactos de entrenamiento y evaluacion por experimento y seed.

## Estructura vigente

```text
experiments/
├── EXP1_SIMPLE_RGB/
│   ├── seed_0/
│   │   ├── metrics.csv
│   │   ├── predictions.csv
│   │   ├── best_epoch.txt
│   │   ├── config_snapshot.yaml
│   │   └── checkpoints/
│   │       ├── best.pth
│   │       └── last.pth
│   ├── seed_1/
│   ├── seed_2/
│   └── best_epoch_summary.csv
├── EXP2_SIMPLE_RGBD/
├── EXP3_RESNET18_RGB_AUGMENT/
└── EXP4_RESNET18_RGBD/
```

## Archivos clave

- `seed_<N>/metrics.csv`: metricas por epoca para una seed.
- `seed_<N>/checkpoints/best.pth`: mejor checkpoint por `val_success`.
- `seed_<N>/checkpoints/last.pth`: ultimo estado de entrenamiento.
- `best_epoch_summary.csv`: resumen agregado por experimento usando las seeds disponibles.

## Flujo de regeneracion

1. Entrenar por seed (`scripts/train.py`).
2. Seleccionar mejor epoca (`scripts/select_best_epoch.py`).
3. Resumir globalmente (`scripts/summarize_results.py`).
4. Generar figuras/tablas (`scripts/generate_figures.py`, `scripts/generate_tables.py`).

## Nota de versionado

Los artefactos en `experiments/` son generados localmente y no deben tratarse como codigo fuente.

**Ultima actualizacion:** 2026-03-07
