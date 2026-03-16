# Guia actualizacion TFM

Fecha: 2026-03-07

## 1. Resumen ejecutivo del trabajo realizado

Durante esta sesion se reconstruyo, ejecuto y valido el bloque experimental de `agarre_inteligente` con datos reales de Cornell.

Objetivos cumplidos:

1. Organizar y dejar operativo el dataset Cornell en estructura reproducible.
2. Completar los 4 experimentos base (`EXP1..EXP4`) con `n=3` seeds.
3. Detectar y corregir un error critico de evaluacion que estaba infravalorando los resultados.
4. Regenerar resumenes, tablas y figuras finales.
5. Actualizar documentacion `.md` para que refleje el estado real.

Resultado clave:

- La causa principal de la caida de metricas no fue PyTorch, sino el protocolo de evaluacion (single-GT en lugar de multi-GT por imagen).

## 2. Cronologia tecnica (guia para explicar en el TFM)

### Fase A: Preparacion y ejecucion inicial

1. Se preparo el entorno y estructura del proyecto.
2. Se configuro Cornell y se entrenaron los experimentos base.
3. Se amplio la ejecucion para tener `n=3` seeds en todos los experimentos.

### Fase B: Deteccion de discrepancia

Problema observado:

- Los success rates eran anormalmente bajos respecto a lo esperado para Cornell.
- Las perdidas y curvas no sugerian un colapso total del entrenamiento, por lo que se sospecho problema de evaluacion o pipeline de datos.

### Fase C: Hipotesis y descarte

Hipotesis inicial:

- Diferencias de version de PyTorch / pesos pretrained.

Verificacion:

- Se compararon resultados en modelos con y sin pretrained.
- Se vio que la degradacion no quedaba explicada por versionado de PyTorch de forma consistente.

Conclusion:

- PyTorch no era la causa raiz principal.

### Fase D: Causa raiz y fix

Causa raiz real:

- Cornell tiene multiples grasps GT por imagen (aprox 5-6).
- El evaluador estaba comparando contra un solo GT, penalizando predicciones validas que coincidian con otro grasp de la misma imagen.

Fix aplicado:

1. Cargar todos los GT por `image_path` desde CSV.
2. Comparar cada prediccion contra todos los GT de esa imagen.
3. Tomar mejor match para metricas de IoU/angulo y success (criterio Cornell correcto).

Impacto:

- Recuperacion fuerte de metricas en EXP3/EXP4 sin necesidad de reentrenar.

### Fase E: Regeneracion final de artefactos

1. Reevaluacion de seeds con evaluador corregido.
2. Regeneracion de `best_epoch_summary.csv` por experimento.
3. Regeneracion de resumen global, tablas y figuras.
4. Validacion de artefactos.

## 3. Gestion del dataset (punto clave para defensa)

### 3.1 Estructura final

- `data/raw/cornell/` contiene dataset original.
- `data/processed/cornell/splits/object_wise/` contiene:
  - `train.csv` (3542 muestras)
  - `val.csv` (1569 muestras)

### 3.2 Formato de etiquetas usado

Columnas de CSV:

- `image_path, depth_path, cx, cy, w, h, angle_deg`

### 3.3 Transformacion y normalizacion

1. Imagen redimensionada a `224x224`.
2. Coordenadas GT reescaladas al tamano final.
3. Target normalizado para entrenamiento de regresion:
   - `[cx/224, cy/224, w/224, h/224, angle_deg/90]`.

### 3.4 Leccion aprendida de evaluacion Cornell

- En Cornell, una imagen puede tener varios agarres validos.
- Evaluar contra un unico GT subestima injustamente la calidad del modelo.
- Protocolo correcto: comparacion multi-GT por imagen.

## 4. Experimentos y resultados finales

Fuente: `reports/tables/summary_results.csv`

1. `EXP1_SIMPLE_RGB` (SimpleCNN, RGB):
- val_success_mean: `0.2029`

2. `EXP2_SIMPLE_RGBD` (SimpleCNN, RGB-D):
- val_success_mean: `0.3786`

3. `EXP3_RESNET18_RGB_AUGMENT` (ResNetGrasp, RGB + aug):
- val_success_mean: `0.6769`

4. `EXP4_RESNET18_RGBD` (ResNetGrasp, RGB-D):
- val_success_mean: `0.6541`

Mensajes clave para el TFM:

1. ResNetGrasp supera claramente a SimpleCNN.
2. RGB-D aporta mejora notable en SimpleCNN.
3. Con protocolo de evaluacion correcto, resultados son coherentes y robustos.
4. Se uso `n=3` seeds en todos los experimentos para homogeneidad estadistica.

## 5. Problemas encontrados y como se resolvieron

1. Desalineacion de escala entre imagen y coordenadas GT.
- Sintoma: metricas invalidas o degradadas.
- Solucion: reescalado de GT al tamano final antes de normalizar.

2. Evaluacion single-GT en Cornell.
- Sintoma: success rate artificialmente bajo.
- Solucion: evaluacion multi-GT por imagen en `Evaluator`.

3. Resumenes agregados desfasados.
- Sintoma: `summary_results.csv` no reflejaba metricas corregidas.
- Solucion: regenerar `best_epoch_summary.csv` y luego regenerar resumen/figuras/tablas.

4. Documentacion dispersa y parcialmente desactualizada.
- Solucion: actualizacion de `.md` operativos y marcado de `OLD/` como historico.

## 6. Checklist de actualizacion del TFM (figuras, tablas y artefactos)

### 6.1 Figuras a revisar/actualizar en memoria y texto

Ubicacion: `reports/figures/`

1. `loss_train_by_epoch.png`
2. `loss_val_by_epoch.png`
3. `val_success_by_epoch.png`
4. `val_iou_by_epoch.png`
5. `val_angle_deg_by_epoch.png`
6. `bar_val_success_final.png`
7. `bar_val_iou_final.png`
8. `bar_val_angle_final.png`

### 6.2 Tablas a revisar/actualizar en el documento

Ubicacion: `reports/tables/`

1. `summary_results.csv` (fuente maestra)
2. `results_by_seed.csv`
3. `table_validation_aggregated.csv`
4. `table_metrics_final.csv`
5. `table_ab_comparison_by_modality.csv`
6. `TABLES_SUMMARY.md` (resumen legible)

### 6.3 Artefactos de experimento para trazabilidad

Ubicacion: `experiments/EXP*/`

1. `best_epoch_summary.csv` por experimento
2. `seed_*/metrics.csv`
3. `seed_*/best_epoch.txt`
4. `seed_*/checkpoints/best.pth`
5. `seed_*/checkpoints/last.pth`

Nota:

- `reports/bench/` esta vacio actualmente; si el TFM incluye benchmark de latencia, hay que regenerarlo con `scripts/benchmark_latency.py`.

## 7. Inventario de todos los .md del workspace (proyecto)

### 7.1 Markdown operativo actual

1. `README.md`
- Vision general, estructura, ejecucion reproducible y estado actual.

2. `REENTRENAMIENTO_2026_03_07.md`
- Cierre tecnico del reentrenamiento/reevaluacion y resultados finales.

3. `docs/EXPERIMENTS.md`
- Registro consolidado de configuraciones y resultados finales.

4. `docs/DATASET_SETUP.md`
- Preparacion/verificacion de Cornell y formato de datos.

5. `docs/API.md`
- API real del codigo (modelos, dataset, trainer, evaluator, scripts).

6. `docs/TRAZABILIDAD_TFM.md`
- Cadena de trazabilidad de config -> metricas -> tablas/figuras -> conclusion.

7. `docs/REGISTRO_RECONSTRUCCION.md`
- Bitacora extensa de reconstruccion del proyecto.

8. `data/README.md`
- Estructura de datos y estadisticas vigentes del split.

9. `experiments/README.md`
- Estructura real de artefactos por experimento/seed.

10. `models/README.md`
- Uso de checkpoints/modelos finales para inferencia.

11. `reports/tables/TABLES_SUMMARY.md`
- Resumen legible de tablas finales generadas.

### 7.2 Markdown historico (archivo)

1. `OLD/CHANGELOG_ACTUALIZACION_TFM.md`
2. `OLD/README_DATOS_REALES.md`
3. `OLD/SIMPLE_GRASP_CNN_ESPECIFICACION.md`
4. `OLD/RESNET18_GRASP_ESPECIFICACION.md`

Estado:

- Se mantienen como referencia historica, no como fuente vigente.

## 8. Carpetas vacias o de bajo valor: evaluacion de limpieza

### 8.1 Carpetas vacias o solo placeholder detectadas

En `agarre_inteligente`:

1. `reports/bench`
2. `data/external`
3. `experiments/runs`
4. `experiments/checkpoints`
5. `experiments/results`

En el workspace global `/home/laboratorio/TFM`:

6. `/home/laboratorio/TFM/agarre_ros2_ws` (vacia)

### 8.2 Recomendacion

1. Mantener por ahora:
- `data/external` (reserva para futuros datasets).
- `reports/bench` (si se quiere benchmark en el TFM final).

2. Candidatas a eliminacion segura (si no se van a usar):
- `experiments/runs`
- `experiments/checkpoints`
- `experiments/results`

3. Candidata de limpieza a nivel workspace:
- `/home/laboratorio/TFM/agarre_ros2_ws` solo si no se va a usar integracion ROS 2 en esta fase del TFM.

Motivo:

- La estructura vigente usa `experiments/EXP*/seed_*/...`, no esas carpetas legacy de raiz.

### 8.3 Limpieza ejecutada (2026-03-07)

Carpetas eliminadas:

```bash
experiments/runs
experiments/checkpoints
experiments/results
```

Motivo: carpetas legacy no utilizadas en la estructura vigente `experiments/EXP*/seed_*/`.

Carpetas movidas a `OLD/`:

- `inventario/` → `OLD/inventario/` (archivo tar.gz del proyecto antiguo)

Carpetas mantenidas:

- `reports/bench` (reserva para benchmark futuro)
- `data/external` (reserva para datasets adicionales)
- `/home/laboratorio/TFM/agarre_ros2_ws` (a reconstruir mas tarde)

Estado final: workspace limpio y funcional, material histórico consolidado en `OLD/`.

## 9. Guion corto para defensa/explicacion oral

1. Partimos de una reconstruccion reproducible de pipeline y dataset Cornell.
2. Ejecutamos EXP1..EXP4 con `n=3` seeds para comparacion estadistica homogena.
3. Detectamos discrepancia fuerte en metricas y evaluamos hipotesis (incluida versionado PyTorch).
4. Identificamos la causa raiz en evaluacion single-GT sobre dataset multi-anotado.
5. Corregimos evaluador a protocolo Cornell multi-GT y reevaluamos.
6. Regeneramos tablas/figuras y actualizamos documentacion para cierre trazable.
7. Concluimos con resultados consistentes y defendibles para el TFM.

## 10. Estado final

- Workspace funcional y validado (`scripts/validate_artifacts.py --strict`).
- Documentacion operativa alineada al estado real.
- Pendiente solo de trasladar estas tablas/figuras al documento TFM final y, si procede, ejecutar benchmark.
