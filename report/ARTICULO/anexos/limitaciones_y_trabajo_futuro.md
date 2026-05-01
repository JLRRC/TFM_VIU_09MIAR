# Limitaciones y trabajo futuro

## Limitaciones

- Evaluacion principal sobre Cornell; no cubre la diversidad de datasets y escenas reales.
- Tres semillas por experimento, suficientes para trazabilidad comparativa pero limitadas para inferencia estadistica fuerte.
- Resultados oficiales calculados con evaluador historico; la variante con IoU orientada y perdida angular requiere ejecucion consolidada.
- Comparacion RGB/RGB-D condicionada por el preprocesamiento, la arquitectura y la configuracion de augmentation.
- La evidencia aplicada no reemplaza una validacion fisica sistematica.

## Trabajo futuro

- Reentrenar el bloque oficial con `GraspLoss` e IoU orientada.
- Comparar con arquitecturas densas modernas para mapas de calidad, angulo y apertura.
- Evaluar transferencia a Jacquard, GraspNet, Acronym u otros datasets.
- Aumentar el numero de semillas y realizar ablations especificas de augmentation/profundidad.
- Validar la configuracion seleccionada en escenarios fisicos o simulaciones con mayor diversidad visual.
