# Regeneracion de curvas loss del capitulo 5

- Script modificado: `/home/laboratorio/TFM/agarre_inteligente/scripts/generar_ilustraciones_tfm_5_1.py`
- Criterio de eje Y por defecto: min/max global conjunto de `train_loss` y `val_loss` por experimento, con margen simétrico del 5%, aplicado idénticamente a ambos subplots.
- Soporte de excepciones: se permite configurar `ylim` manual compartido o `ylim` independiente por subplot si existe una justificación editorial explícita y sin recortar datos.
- Nota del tutor incorporada: se fuerza en EXP2 un `ylim` manual compartido [0.0300, 0.0400] para facilitar comparación directa entre train y val.
- Verificacion esperada: cada pareja train/val comparte exactamente el mismo `ylim`; se conservan títulos, leyendas y seeds.

## Figuras regeneradas

### EXP1_SIMPLE_RGB

- Figura fuente: `/tmp/loss_5_1_unified/ilustracion_5_2_curvas_loss_exp1_simple_rgb.png`
- Seeds: Seed 0, Seed 1, Seed 2
- Titulo train: `EXP1_SIMPLE_RGB - Pérdida de Entrenamiento`
- Titulo val: `EXP1_SIMPLE_RGB - Pérdida de Validación`
- Politica aplicada: `shared_global_train_val`
- Rango bruto conjunto: [0.029159, 0.190331]
- Margen aplicado: 0.008059 (5%)
- `ylim` train: [0.021101, 0.198390]
- `ylim` val: [0.021101, 0.198390]

### EXP2_SIMPLE_RGBD

- Figura fuente: `/tmp/loss_5_1_unified/ilustracion_5_3_curvas_loss_exp2_simple_rgbd.png`
- Seeds: Seed 0, Seed 1, Seed 2
- Titulo train: `EXP2_SIMPLE_RGBD - Pérdida de Entrenamiento`
- Titulo val: `EXP2_SIMPLE_RGBD - Pérdida de Validación`
- Politica aplicada: `manual_shared_ylim`
- Rango bruto conjunto: [0.031871, 0.039400]
- Margen aplicado: 0.000376 (5%)
- `ylim` train: [0.030000, 0.040000]
- `ylim` val: [0.030000, 0.040000]

- Nota editorial: Se aplica un `ylim` manual compartido y documentado para EXP2_SIMPLE_RGBD: [0.030000, 0.040000]

### EXP3_RESNET18_RGB_AUGMENT

- Figura fuente: `/tmp/loss_5_1_unified/ilustracion_5_4_curvas_loss_exp3_resnet18_rgb_augment.png`
- Seeds: Seed 0, Seed 1, Seed 2
- Titulo train: `EXP3_RESNET18_RGB_AUGMENT - Pérdida de Entrenamiento`
- Titulo val: `EXP3_RESNET18_RGB_AUGMENT - Pérdida de Validación`
- Politica aplicada: `shared_global_train_val`
- Rango bruto conjunto: [0.024234, 0.081329]
- Margen aplicado: 0.002855 (5%)
- `ylim` train: [0.021379, 0.084183]
- `ylim` val: [0.021379, 0.084183]

### EXP4_RESNET18_RGBD

- Figura fuente: `/tmp/loss_5_1_unified/ilustracion_5_5_curvas_loss_exp4_resnet18_rgbd.png`
- Seeds: Seed 0, Seed 1, Seed 2
- Titulo train: `EXP4_RESNET18_RGBD - Pérdida de Entrenamiento`
- Titulo val: `EXP4_RESNET18_RGBD - Pérdida de Validación`
- Politica aplicada: `shared_global_train_val`
- Rango bruto conjunto: [0.021230, 0.054659]
- Margen aplicado: 0.001671 (5%)
- `ylim` train: [0.019558, 0.056331]
- `ylim` val: [0.019558, 0.056331]
