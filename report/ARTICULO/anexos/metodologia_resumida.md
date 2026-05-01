# Metodologia resumida

1. Preparacion del dataset Cornell procesado con particion object-wise.
2. Definicion de experimentos mediante archivos YAML versionados.
3. Seleccion de arquitectura: CNN ligera (`SimpleCNN`) o ResNet-18 adaptada (`ResNetGrasp`).
4. Carga de datos en modalidad RGB o RGB-D con imagenes de 224 x 224 pixeles.
5. Aplicacion controlada de augmentation cuando la configuracion lo especifica.
6. Entrenamiento con PyTorch, Adam y `SmoothL1Loss` en los experimentos oficiales.
7. Ejecucion de tres semillas por configuracion.
8. Registro de metricas por epoca y seleccion de mejor epoca por seed.
9. Evaluacion por `val_success`, `val_iou`, `val_angle_deg` y `val_loss`.
10. Agregacion de resultados por experimento y conservacion de artefactos para auditoria.

## Criterio editorial

La metodologia debe presentarse como un protocolo reproducible de comparacion. Los artefactos de inferencia o integracion aplicada pueden citarse como contexto, pero no forman parte del argumento experimental principal.
