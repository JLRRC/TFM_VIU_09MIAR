# Limitaciones y trabajo futuro

## Limitaciones

- Evaluacion principal sobre Cornell; no cubre por si sola la diversidad de datasets y escenas reales.
- Tres semillas por experimento, suficientes para trazabilidad comparativa pero limitadas para inferencia estadistica fuerte.
- Resultados oficiales calculados con evaluador historico; la variante con IoU orientada y perdida angular requiere ejecucion consolidada.
- Comparacion RGB/RGB-D condicionada por preprocesamiento, arquitectura y configuracion de augmentation.
- La evidencia funcional final muestra aplicabilidad, pero no reemplaza validacion fisica sistematica.
- No se reclama superioridad frente al estado del arte.

## Trabajo futuro

- Reentrenar el bloque oficial con `GraspLoss` e IoU orientada.
- Comparar con arquitecturas densas modernas para mapas de calidad, angulo y apertura.
- Evaluar transferencia a Jacquard, GraspNet, Acronym u otros datasets.
- Aumentar el numero de semillas y realizar ablations especificas de augmentation/profundidad.
- Incorporar analisis cualitativo de aciertos y fallos.
- Validar la configuracion seleccionada en escenarios fisicos o simulaciones con mayor diversidad visual.
