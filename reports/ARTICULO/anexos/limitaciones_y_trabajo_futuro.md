<!-- generado-articulo-tfm -->

# Limitaciones y trabajo futuro

## Limitaciones

- Evaluacion principal sobre Cornell; no cubre diversidad amplia de datasets.
- Tres semillas por experimento, suficientes para trazabilidad pero limitadas para inferencia estadistica fuerte.
- Resultados oficiales calculados con evaluador historico; la variante orientada posterior no reescribe las tablas oficiales.
- Integracion demostrada en simulacion, no como ensayo fisico sistematico.
- RGB-D no mejora de forma uniforme; requiere analisis adicional.

## Trabajo futuro

- Reentrenar bloque oficial con `GraspLoss` e IoU orientada.
- Comparar con arquitecturas densas modernas.
- Evaluar transferencia a Jacquard/GraspNet u otros datasets.
- Validar en robot fisico y medir tasa de agarre real.
- Añadir analisis estadistico formal y ablation de augmentation/profundidad.
