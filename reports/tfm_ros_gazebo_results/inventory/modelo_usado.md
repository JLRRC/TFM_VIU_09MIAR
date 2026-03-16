# Modelo usado en la integracion ROS 2 + Gazebo

## Inventario de checkpoints detectados

Segun `evidence/terminal/ros_gazebo_checkpoints_index.json`, el panel tenia indexados al menos estos checkpoints principales:

- `EXP1_SIMPLE_RGB/seed_0/checkpoints/best.pth`
- `EXP2_SIMPLE_RGBD/seed_0/checkpoints/best.pth`
- `EXP3_RESNET18_RGB_AUGMENT/seed_0/checkpoints/best.pth`
- `EXP4_RESNET18_RGBD/seed_1/checkpoints/best.pth`

Metricas `val_success` disponibles en el indice:

- `EXP1_SIMPLE_RGB`: `0.2409177820267686`
- `EXP2_SIMPLE_RGBD`: `0.3887826641172721`
- `EXP3_RESNET18_RGB_AUGMENT`: `0.7189292543021033`
- `EXP4_RESNET18_RGBD`: `0.6730401529636711`

## Modelo seleccionado como caso principal para la integracion

Modelo mas fuerte y mejor documentado para el caso de resultados ROS 2 + Gazebo:

- Experimento:
  - `EXP3_RESNET18_RGB_AUGMENT`
- Semilla:
  - `seed_0`
- Checkpoint:
  - `/home/laboratorio/TFM/agarre_inteligente/experiments/EXP3_RESNET18_RGB_AUGMENT/seed_0/checkpoints/best.pth`
- Arquitectura:
  - `resnet18`
- Modalidad:
  - RGB
- Canales de entrada:
  - `3`
- Tamano de entrada:
  - `224`
- Hash del checkpoint observado en `apply_experiment.log`:
  - `50ba7bdb2172cc133a9422c596a0a1f1003d836272ca7d8451ea9dd225c04271`

## Por que este es el modelo mas probable y mas defendible

- `evidence/terminal/ros_gazebo_infer.log` muestra que las inferencias del caso principal del 2026-03-15 usan exactamente ese checkpoint.
- `evidence/terminal/ros_gazebo_apply_experiment.log` registra multiples cargas correctas del mismo checkpoint con `model_name='resnet18'` e `in_channels=3`.
- En el indice de checkpoints es el modelo con mejor `val_success` entre los cuatro casos localizados.
- El mejor caso de publicacion y ejecucion via MoveIt del 2026-03-15 aparece temporalmente alineado con el uso de `EXP3`.

## Como se carga en la integracion real

- La evidencia principal no apunta al nodo standalone `tfm_grasping/grasp_inference.py` como mecanismo central del caso seleccionado.
- La via mejor documentada es el panel `ur5_qt_panel/panel_v2.py`:
  - selecciona el checkpoint desde la UI;
  - invoca `self.tfm_module.load_model(ckpt_path)`;
  - deja trazas en `logs/apply_experiment.log`;
  - usa despues ese modelo en la accion de inferencia del panel.

## Alternativas disponibles pero no seleccionadas como evidencia principal

- `EXP1_SIMPLE_RGB`
  - Cargado en varias ocasiones, pero con peor resultado validado y sin ser el dominante en `infer.log`.
- `EXP2_SIMPLE_RGBD`
  - Detectado en el indice, pero sin ser el mas repetido en el caso fuerte.
- `EXP4_RESNET18_RGBD`
  - Cargable y con buen `val_success`, pero la inferencia mejor documentada del caso fuerte no usa este checkpoint.

## Afirmacion rigurosa

Para la subseccion de resultados ROS 2 + Gazebo, el modelo mas defendible es `EXP3_RESNET18_RGB_AUGMENT`, `seed_0`, `ResNet18 RGB`, porque es el checkpoint mejor trazado en la combinacion `apply -> infer -> execute`.
