<!-- generado-articulo-tfm -->

# Metodologia resumida

1. Preparacion del dataset Cornell procesado con particion object-wise.
2. Definicion de experimentos mediante YAML.
3. Construccion del modelo con `src/models/factory.py`.
4. Carga de `GraspDataset` en modalidad RGB o RGB-D.
5. Transformaciones: resize a 224x224; augmentation opcional con flip horizontal, rotacion y color jitter.
6. Entrenamiento con Adam y `SmoothL1Loss` en los experimentos oficiales.
7. Registro de metricas por epoca y seleccion de mejor epoca.
8. Evaluacion por `val_success`, `val_iou`, `val_angle_deg` y `val_loss`.
9. Benchmark de latencia en CPU/CUDA para batch 1.
10. Integracion de checkpoint seleccionado en wrapper ROS 2 para inferencia funcional.
