# Data Directory

Esta carpeta contiene los datos Cornell usados por el pipeline de entrenamiento y evaluacion.

## Estructura

```text
data/
├── raw/
│   └── cornell/
│       ├── 01/ ...
│       ├── 02/ ...
│       └── ...
├── processed/
│   └── cornell/
│       └── splits/
│           └── object_wise/
│               ├── train.csv
│               └── val.csv
└── external/
```

## Formato de CSV procesado

Columnas:

- `image_path`
- `depth_path`
- `cx`
- `cy`
- `w`
- `h`
- `angle_deg`

Cada fila representa un grasp anotado para una imagen.

## Estadisticas vigentes del split

- Train: 3542 muestras
- Validation: 1569 muestras
- Total: 5111 muestras anotadas
- Split: object-wise (70/30 por objetos)

## Notas tecnicas

- Resolucion original Cornell: `640x480`.
- Entrada del modelo: `224x224`.
- El loader escala coordenadas GT al tamano final antes de normalizar.
- Para modalidad `rgbd`, se toleran archivos depth truncados con fallback seguro.

## Reproducibilidad

Para regenerar los CSVs:

```bash
python3 scripts/prepare_cornell_csv.py \
  --raw-dir data/raw/cornell \
  --out-dir data/processed/cornell \
  --val-split 0.3 \
  --seed 42
```

## Referencia

- Cornell Grasping Dataset: http://pr.cs.cornell.edu/grasping/rect_data/data.php
