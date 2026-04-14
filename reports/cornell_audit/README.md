# cornell_audit

Evidencia minima de trazabilidad de la preparacion reproducible del Cornell Grasping Dataset.

Contenido:

- `clean_idx_train_v2.txt`: recuento del conjunto de entrenamiento generado.
- `clean_idx_val.txt`: recuento del conjunto de validacion generado.

Estos indices se regeneran mediante:

```bash
/home/laboratorio/.venv-tfm/bin/python \
  agarre_inteligente/scripts/prepare_cornell_csv.py \
  --raw-dir agarre_inteligente/data/raw/cornell \
  --out-dir agarre_inteligente/data/processed/cornell \
  --val-split 0.3 \
  --seed 42
```

Los artefactos operativos asociados al split object-wise utilizado por el pipeline final son:

- `agarre_inteligente/data/processed/cornell/splits/object_wise/train.csv`
- `agarre_inteligente/data/processed/cornell/splits/object_wise/val.csv`
