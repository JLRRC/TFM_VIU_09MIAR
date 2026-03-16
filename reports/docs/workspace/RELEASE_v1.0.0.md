# Release v1.0.0

Primera version publicada y etiquetada del workspace consolidado del TFM.

## Contenido de la entrega

- estructura raiz reducida a los elementos operativos y de entrega imprescindibles
- carpeta unica `reports/` como destino canonico de figuras, tablas, validaciones y evidencias
- bloque experimental `agarre_inteligente/` mantenido como fuente de verdad de metricas, scripts y trazabilidad
- bloque ROS 2 `agarre_ros2_ws/` mantenido como stack de simulacion, control, MoveIt y panel Qt
- `README.md` raiz con clonado, bootstrap, build y arranque rapido

## Validaciones relevantes incluidas

- arranque funcional del panel V2 tras corregir compatibilidad de entorno
- compatibilidad restaurada entre `cv_bridge` y `numpy` mediante uso de `numpy 1.26.4`
- correccion del conflicto Qt/OpenCV usando `opencv-python-headless`
- flujo remoto validado de `select -> tfm_infer -> pick -> release`
- workspace ROS 2 reconstruido y operativo con `colcon build --symlink-install`

## Ajustes de publicacion

- eliminacion del checkpoint pesado no compatible con el limite de GitHub
- reglas `.gitignore` ajustadas para evitar volver a versionar checkpoints grandes, builds, logs y artefactos regenerables
- historico publicado limpio y compatible con GitHub

## Estructura final publicada

- `agarre_inteligente/`
- `agarre_ros2_ws/`
- `reports/`
- `lanzar_panelv2.sh`
- `actualizar_reports.sh`
- `README.md`

## Nota para la release de GitHub

Texto breve sugerido:

```text
Primera entrega publicada del workspace TFM consolidado. Incluye el bloque experimental de vision, el workspace ROS 2 con MoveIt y panel Qt, la carpeta unica reports y la documentacion minima de clonacion, build y arranque.
```