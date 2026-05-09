# Notas de rigor del workspace

Fecha: 2026-04-15

## Split Cornell operativo

- El split activo del workspace queda fijado en `3541` muestras de entrenamiento y `1569` de validacion.
- Esta cifra corresponde al contenido realmente utilizable de:
  - `agarre_inteligente/data/processed/cornell/splits/object_wise/train.csv`
  - `agarre_inteligente/data/processed/cornell/splits/object_wise/val.csv`
- El ajuste respecto a recuentos historicos previos se debe al saneado de una fila corrupta en `train.csv` con valores no finitos.
- El objetivo del saneado fue alinear:
  - el CSV activo
  - el dataset cargado por el pipeline
  - la validacion interna de tamaños en entrenamiento

## Experimentos oficiales frente a implementacion teorica

- La recreacion oficial del TFM se mantiene sobre `EXP1`, `EXP2`, `EXP3` y `EXP4`.
- `EXP1.1` y `EXP1.2` se conservan como implementacion en codigo del diseño objetivo descrito en el apartado `4.6.2` de la memoria.
- Esta separacion no corrige ni reinterpreta retroactivamente los resultados publicados; preserva la trazabilidad del documento tal como fue presentado.

## Uso en panel

- `EXP1.1` y `EXP1.2` permanecen disponibles en el panel para inferencia como variantes auxiliares.
- No deben considerarse parte de la recreacion oficial de tablas, figuras o conclusiones del capitulo experimental.
