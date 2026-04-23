# Alineación Metodológica Posterior al TFM — ENTREGA.V2

## Contexto

La memoria del TFM ya está cerrada. Los experimentos oficiales (EXP1..EXP4) y
sus artefactos en `reports/` son **inmutables**. Este documento describe las
discrepancias metodológicas identificadas entre el workspace real y la formulación
teórica del TFM, y cómo se han resuelto **sin** tocar los resultados oficiales.

---

## Discrepancias identificadas y estado de resolución

### A. IoU Cornell con rectángulos orientados

**Discrepancia:** `src/training/metrics.py` implementaba `iou_axis_aligned_boxes`
que ignora el ángulo θ. El criterio Cornell canónico (Jiang et al. 2011, Redmon
& Angelova 2015) requiere calcular el IoU entre los **rectángulos orientados**
reales representados por `(cx, cy, w, h, θ)`.

**Impacto en EXP1..EXP4:** La IoU axis-aligned **sobreestima** la coincidencia
cuando los ángulos difieren, por lo que los `val_success` reportados son una
cota superior del éxito real. Los modelos peores aparecen mejores de lo que son.

**Resolución (POSTERIOR al TFM, no invalida resultados oficiales):**

Nuevas funciones en `src/training/metrics.py`:
- `_rect_vertices(cx, cy, w, h, angle_deg)` — vértices del rectángulo orientado
- `iou_oriented_boxes(pred, gt)` — IoU por intersección de polígonos convexos
  (algoritmo Sutherland-Hodgman, sin dependencias externas)
- `cornell_success_oriented(pred, gt, iou_thr=0.25, angle_thr=30.0)`
- `summarize_batch_oriented(pred, gt)`

Las funciones antiguas (`iou_axis_aligned_boxes`, `cornell_success`,
`summarize_batch`) se **mantienen intactas** para retrocompatibilidad.

**Verificación:** 20 tests en `tests/test_oriented_iou.py`. Propiedades validadas:
- Rectángulo idéntico → IoU = 1.0
- Sin solapamiento → IoU = 0.0
- Con θ=0, coincide exactamente con la axis-aligned (control)
- IoU orientada ≤ IoU axis-aligned cuando hay diferencia de ángulo (más estricta)
- Caso con solapamiento conocido: dos cuadrados desplazados 50% → IoU = 1/3

---

### B. grasp success calculado por imagen

**Discrepancia:** El criterio Cornell canónico define el éxito como que la
predicción de una imagen sea válida si **existe al menos un GT** de esa imagen
que satisfaga simultáneamente IoU ≥ 0.25 y |Δθ| ≤ 30°.

**Estado en el código:** `Evaluator` ya tenía la estructura correcta (`gt_by_image`
con `np.any(successes)` por imagen), pero usaba `iou_axis_aligned_boxes`.

**Resolución:**

Nueva clase `EvaluatorOriented` en `src/evaluation/evaluator.py`:
- Usa `iou_oriented_boxes` y `cornell_success_oriented`
- Mantiene el mismo criterio de éxito por imagen (`np.any`) del `Evaluator` original
- El `Evaluator` original **no se modifica** — EXP1..EXP4 lo siguen usando

**Verificación:** 8 tests en `tests/test_success_per_image.py`. Propiedades validadas:
- Éxito si al menos un GT coincide (aunque otros no)
- Fracaso si ningún GT coincide
- IoU orientada más estricta que axis-aligned con rotación
- Umbral de ángulo inclusivo en el límite (≤ 30°)
- Simetría de 180° de la pinza paralela respetada

---

### C. Función de pérdida más alineada con la formulación teórica

**Discrepancia:** El entrenamiento oficial usó `nn.SmoothL1Loss()` aplicada
uniformemente sobre los 5 outputs `(cx, cy, w, h, angle_norm)`. Esta pérdida
tiene un problema fundamental con el ángulo: no respeta la simetría de 180°
de la pinza paralela. Por ejemplo, `angle_norm=0.99` (≈89°) y `angle_norm=-0.99`
(≈-89°) son ángulos prácticamente equivalentes (diferencia de 2°), pero
`SmoothL1Loss` les asigna una pérdida de ≈1.98 como si fueran muy distintos.

**Resolución:**

Nuevo módulo `src/training/losses.py` con:

```python
class GraspLoss(nn.Module):
    """
    L = geom_weight * SmoothL1(pred[:,:4], gt[:,:4])
      + angle_weight * mean(1 - cos(π · (pred_angle - gt_angle)))
    """
```

La pérdida coseno periódica `1 - cos(π·diff)` tiene periodo 2 en el espacio
normalizado (angle_deg / 90), equivalente a 180° en grados. Esto garantiza
que ángulos equivalentes módulo 180° dan pérdida cero, independientemente de
si el modelo predice +89°/90 o -89°/90.

El script `scripts/train.py` añade dispatch desde el YAML:
```yaml
training:
  criterion: "grasp_loss"          # o "smooth_l1" (por defecto, EXP1..EXP4)
  criterion_kwargs:
    angle_weight: 1.0
    geom_weight: 1.0
```

Los configs de EXP1..EXP4 **no se modifican** — todos tienen `criterion: smooth_l1`
explícito y el comportamiento es idéntico al original.

**Verificación:** 12 tests en `tests/test_grasp_loss.py`. Propiedades validadas:
- Predicción idéntica → pérdida = 0
- Ángulos equivalentes (diff=180°) → pérdida de ángulo = 0
- Diff=90° → pérdida de ángulo = 2 (máximo)
- Diff=30° → pérdida de ángulo = 0.5 (1 - cos(π/3))
- Peso geom_weight=0 → resultado idéntico a SmoothL1 en (cx,cy,w,h)
- `build_criterion` retorna tipos correctos

---

## Separación oficial / alineación metodológica

| Artefacto | Tipo | Estado |
|---|---|---|
| `reports/` (todo el árbol) | OFICIAL TFM | Inmutable — no tocar |
| `experiments/EXP1..EXP4` (checkpoints) | OFICIAL TFM | Inmutable |
| `src/training/metrics.py` — funciones `*_axis_aligned*`, `cornell_success`, `summarize_batch` | OFICIAL TFM (comportamiento) | Conservadas sin cambio |
| `src/evaluation/evaluator.py` — clase `Evaluator` | OFICIAL TFM (comportamiento) | Conservada sin cambio |
| `scripts/train.py` — dispatch de criterion | OFICIAL + extensión | Retrocompatible: smooth_l1 por defecto |
| `src/training/metrics.py` — funciones `*_oriented*`, `_rect_vertices`, Sutherland-Hodgman | **ALINEACIÓN METODOLÓGICA** | Añadidas, no sustituyen |
| `src/training/losses.py` — `GraspLoss`, `build_criterion` | **ALINEACIÓN METODOLÓGICA** | Nuevo módulo |
| `src/evaluation/evaluator.py` — clase `EvaluatorOriented` | **ALINEACIÓN METODOLÓGICA** | Añadida, no sustituye |
| `config/exp_methodology_v2.yaml` | **ALINEACIÓN METODOLÓGICA** | Config de ejemplo, no afecta EXP1..EXP4 |
| `tests/` (todos los archivos) | **ALINEACIÓN METODOLÓGICA** | Tests nuevos, no existían antes |

---

## Impacto sobre los resultados oficiales

Los val_success reportados en `reports/metrics/validated/` para EXP1..EXP4 son
**cota superior** del éxito real con IoU orientada. La diferencia cuantitativa
depende del modelo: un modelo que predice ángulos muy precisos verá poca
diferencia; un modelo con errores de ángulo significativos verá una reducción
notable del val_success orientado respecto al axis-aligned.

Esta diferencia es una limitación metodológica conocida del TFM, documentada
en `README.md` (sección "Nota final sobre desajustes metodológicos no aplicados").
La alineación de este documento no invalida los resultados presentados en la
memoria, sino que proporciona las herramientas para que evaluaciones futuras
sean metodológicamente más rigurosas.

---

## Cómo usar la variante metodológica

### Evaluar un checkpoint existente con IoU orientada

```python
from src.evaluation.evaluator import EvaluatorOriented
import torch.nn as nn

evaluator = EvaluatorOriented(
    model=model,
    dataloader=val_loader,
    device="cpu",
    csv_path="data/processed/cornell/splits/object_wise/val.csv",
    iou_thr=0.25,
    angle_thr=30.0,
)
result = evaluator.evaluate(nn.SmoothL1Loss())
print(f"val_success (orientada): {result.val_success:.4f}")
```

### Entrenar con GraspLoss

```bash
python3 scripts/train.py --config config/exp_methodology_v2.yaml --seed 0
```

Las salidas irán a `experiments/EXP_METHOD_V2_RGB/` para no contaminar los
directorios oficiales EXP1..EXP4.
