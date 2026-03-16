# Entrega Workspace TFM - 2026-03-16

## Objetivo

Dejar el workspace preparado para entrega, separando material no esencial en la carpeta `BORRAR` y revisando el estado general del código y de los bloques funcionales principales.

## Limpieza aplicada

Se ha creado la carpeta `BORRAR/` en la raíz del workspace y se ha movido allí el material claramente no esencial para una entrega académica o técnica del proyecto.

### Movido a BORRAR

- `BORRAR/agarre_inteligente/.venv-tfm`
- `BORRAR/agarre_inteligente/.pytest_cache`
- `BORRAR/agarre_inteligente/logs`
- `BORRAR/agarre_inteligente/debug_rescale.py`
- `BORRAR/agarre_inteligente/debug_training_loop.py`
- `BORRAR/agarre_ros2_ws/build`
- `BORRAR/agarre_ros2_ws/install`
- `BORRAR/agarre_ros2_ws/log`
- `BORRAR/agarre_ros2_ws/old`
- `BORRAR/reports/panel_audit/logs`
- caches `__pycache__` detectadas en el workspace

### Volumen apartado

- `BORRAR/agarre_inteligente`: ~7.4G
- `BORRAR/agarre_ros2_ws`: ~320M
- `BORRAR/__pycache__`: ~443M
- `BORRAR/panel_audit`: ~436K

## Material conservado como esencial

### Proyecto ML

- `agarre_inteligente/src`
- `agarre_inteligente/config`
- `agarre_inteligente/data`
- `agarre_inteligente/experiments`
- `agarre_inteligente/reports`
- `agarre_inteligente/docs`
- `agarre_inteligente/scripts`

### Proyecto ROS 2

- `agarre_ros2_ws/src`
- `agarre_ros2_ws/scripts`
- `agarre_ros2_ws/tools`
- `agarre_ros2_ws/models`
- `agarre_ros2_ws/worlds`
- documentación raíz del workspace ROS 2

### Documentación y trazabilidad

- `AUDITORIA_MOVEIT_GAZEBO_QT.md`
- `VALIDACION_WORKSPACE_2026_03_16.md`
- `reports/`

## Revisión técnica

### Estado general

- No se detectan errores estáticos en `agarre_inteligente/src` ni en `agarre_ros2_ws/src`.
- El workspace tiene estructura coherente para entrega: código fuente, documentación, datos, resultados y stack ROS 2 están claramente separados.
- El código principal parece terminado funcionalmente. La mayor parte de bloques `pass` encontrados corresponden a manejo defensivo de excepciones, tolerancia a fallos o callbacks opcionales, no a huecos de implementación evidentes.

### Deuda funcional explícita detectada

- En `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_v2.py` queda un único `TODO` operativo visible: integrar la lectura real de fuerza del gripper RG2 desde el tópico ROS correspondiente. Actualmente hay una estimación ficticia de fuerza cuando la pinza se considera cerrada.

### Ajuste aplicado durante la sesión

- Se restauró el botón manual de `Soltar objetos` en el panel para poder liberar los objetos desde la interfaz.

## Riesgos y observaciones

- El workspace ROS 2 ha sido reconstruido despues de la limpieza con `colcon build --symlink-install`, asi que vuelve a ser ejecutable localmente.
- El panel ya no depende de que exista exactamente `.venv-tfm`: los wrappers detectan tambien `agarre_inteligente/venv` y `agarre_inteligente/.venv`.
- El `venv` oficial de `agarre_inteligente` se ha completado con `PyQt5`, de forma que el setup reproducible vuelve a cubrir la UI del panel.
- Los datos y resultados experimentales se han conservado. No se han tocado `experiments/`, `data/` ni las figuras/tablas finales.

## Recomendación de entrega

El workspace queda razonablemente preparado para entrega si el objetivo es presentar:

- código fuente limpio
- resultados reproducibles
- documentación de validación
- separación visible de artefactos reconstruibles

El workspace queda limpio para versionado y, al mismo tiempo, ejecutable localmente. Los artefactos locales regenerables siguen fuera de git mediante `.gitignore`.