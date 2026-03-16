# Configuracion del Cornell Grasping Dataset

Guia de referencia para verificar y reproducir la preparacion del dataset Cornell.

## Estado actual (2026-03-07)

- Dataset descargado: SI
- CSVs generados: SI
- Modo de trabajo vigente: datos reales Cornell

## Estructura esperada

```text
data/
├── raw/cornell/
│   ├── 01/
│   ├── 02/
│   └── ...
└── processed/cornell/
    └── splits/object_wise/
        ├── train.csv
        └── val.csv
```

## Formato CSV

Columnas esperadas:

- `image_path`
- `depth_path`
- `cx`
- `cy`
- `w`
- `h`
- `angle_deg`

Ejemplo:

```csv
image_path,depth_path,cx,cy,w,h,angle_deg
data/raw/cornell/01/pcd0100r.png,data/raw/cornell/01/pcd0100d.tiff,320.5,240.3,80.2,40.1,15.7
```

## Tamano actual del split

- `train.csv`: 3542 filas de muestras
- `val.csv`: 1569 filas de muestras

Verificacion rapida:

```bash
wc -l data/processed/cornell/splits/object_wise/train.csv
wc -l data/processed/cornell/splits/object_wise/val.csv
```

Salida esperada aproximada (incluye header):

- `3543` en train
- `1570` en val

## Preparacion automatica

```bash
cd /home/laboratorio/TFM/agarre_inteligente
./scripts/download_cornell.sh
```

## Preparacion manual (si ya tienes raw)

```bash
cd /home/laboratorio/TFM/agarre_inteligente
source venv/bin/activate
python3 scripts/prepare_cornell_csv.py \
  --raw-dir /ruta/a/cornell \
  --out-dir data/processed/cornell \
  --val-split 0.3 \
  --seed 42
```

## Notas operativas

- El loader maneja archivos depth truncados con fallback para no romper entrenamiento.
- Las imagenes se redimensionan a `224x224` durante carga.
- El target se normaliza a `[cx/224, cy/224, w/224, h/224, angle/90]`.

## Referencias

- Cornell Grasping Dataset: http://pr.cs.cornell.edu/grasping/rect_data/data.php
- Script de parsing: `scripts/prepare_cornell_csv.py`
