# Metodologia resumida

1. Preparacion del dataset Cornell procesado.
2. Particion object-wise para separar objetos de entrenamiento y validacion.
3. Definicion de experimentos mediante YAML versionados.
4. Seleccion de arquitectura: CNN ligera (`SimpleCNN`) o ResNet-18 adaptada (`ResNetGrasp`).
5. Carga de datos en modalidad RGB o RGB-D con imagenes de 224 x 224 pixeles.
6. Aplicacion controlada de augmentation cuando la configuracion lo especifica.
7. Entrenamiento con PyTorch, Adam y `SmoothL1Loss` en los experimentos oficiales.
8. Ejecucion de tres semillas por configuracion.
9. Registro de metricas por epoca y seleccion de mejor epoca por seed.
10. Evaluacion por `val_success`, `val_iou`, `val_angle_deg` y `val_loss`.
11. Agregacion de resultados por experimento y conservacion de artefactos para auditoria.
12. Verificacion de aplicabilidad mediante inferencia funcional del modelo seleccionado.

## Criterio editorial

La metodologia debe presentarse como un protocolo reproducible de comparacion. La integracion aplicada aparece como cierre de aplicabilidad, no como eje tecnico dominante.
