# cornell_audit

Evidencia minima de trazabilidad de la preparacion reproducible del Cornell Grasping Dataset.

## Contenido

- `clean_idx_train_v2.txt`: resumen del conjunto de entrenamiento generado (`N=3542`).
- `clean_idx_val.txt`: resumen del conjunto de validacion generado (`N=1569`).

Estos ficheros no replican las filas completas del split; actuan como huella minima de auditoria del proceso de preparacion.

## Relacion con los CSV operativos

El pipeline activo usa los CSV:

- `agarre_inteligente/data/processed/cornell/splits/object_wise/train.csv`
- `agarre_inteligente/data/processed/cornell/splits/object_wise/val.csv`

Sus recuentos actuales son:

- `train.csv`: 3543 lineas totales, 3542 muestras utiles mas cabecera.
- `val.csv`: 1570 lineas totales, 1569 muestras utiles mas cabecera.

## Regeneracion

Los indices de auditoria y los CSV object-wise se regeneran con:

```bash
python agarre_inteligente/scripts/prepare_cornell_csv.py \
  --raw-dir agarre_inteligente/data/raw/cornell \
  --out-dir agarre_inteligente/data/processed/cornell \
  --val-split 0.3 \
  --seed 42
```

## Nota operativa

Los YAML activos de `EXP1` a `EXP4` y el entrenamiento validan estos tamanos esperados para evitar desalineaciones entre configuracion y dataset real.
