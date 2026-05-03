# Inventario de modelos implementados

| nombre_codigo | clase_real | archivo_definicion | arquitectura | entradas | salidas | papel editorial |
| --- | --- | --- | --- | --- | --- | --- |
| `SimpleGraspCNN` | `SimpleCNN` | `agarre_inteligente/src/models/simple_cnn.py` | CNN ligera con bloques convolucionales, BatchNorm, ReLU, MaxPool, AdaptiveAvgPool2d y MLP final. | RGB o RGB-D segun `input_channels`. | Vector de 5 parametros: `cx`, `cy`, `w`, `h`, `angle`. | Baseline oficial de baja complejidad para `EXP1/EXP2`. |
| `ResNet18Grasp` | `ResNetGrasp` | `agarre_inteligente/src/models/resnet_variants.py` | ResNet-18 con capa final adaptada a cinco salidas y primera convolucion ajustable a cuatro canales. | RGB o RGB-D segun `input_channels`. | Vector de 5 parametros: `cx`, `cy`, `w`, `h`, `angle`. | Arquitectura residual principal para `EXP3/EXP4`. |
| `SimpleGrasp` | `SimpleGrasp` | `agarre_inteligente/src/models/simple_grasp.py` | CNN ligera posterior con primera convolucion 7x7 stride 2, bloques 3x3, AdaptiveAvgPool2d(7,7) y MLP final. | RGB o RGB-D segun `input_channels`. | Vector de 5 parametros normalizados. | Variante complementaria incluida en el articulo; no sustituye resultados oficiales. |
| `graspnet.models.simple_grasp_cnn.SimpleGraspCNN` | no encontrada | no existe bajo `agarre_inteligente/graspnet/models` | Referencia legacy no materializada. | RGB en codigo legacy. | Vector de 5 parametros segun referencia legacy. | No debe aparecer en el cuerpo principal. |

## Observaciones

- `SimpleGraspCNN` es nombre de configuracion; la clase real actual es `SimpleCNN`.
- `ResNet18Grasp` es nombre de configuracion; la clase real actual es `ResNetGrasp`.
- `SimpleGrasp` esta implementado y entrenado en experimentos complementarios incluidos en el articulo.
- Las referencias legacy quedan fuera del articulo salvo como nota de trazabilidad interna.
