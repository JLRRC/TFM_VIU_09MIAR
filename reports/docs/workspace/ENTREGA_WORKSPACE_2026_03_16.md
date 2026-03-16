# Entrega Workspace TFM - 2026-03-16

## Objetivo

Dejar el workspace preparado para entrega, versionado y publicacion, con estructura minima en raiz, una unica carpeta canonica `reports/` y bloques funcionales validados.

## Estado final publicado

El estado final del repositorio publicado mantiene en la raiz unicamente estos elementos principales:

- `agarre_inteligente/`
- `agarre_ros2_ws/`
- `reports/`
- `lanzar_panelv2.sh`
- `actualizar_reports.sh`
- `README.md`
- `.gitignore`

La rama principal publicada en GitHub es `main` y la primera entrega formal queda etiquetada como `v1.0.0`.

## Limpieza aplicada

Durante la reorganizacion se aparto el material no esencial y regenerable para llegar a la estructura final de entrega. Ese material ya no forma parte de la raiz activa del workspace y ha quedado archivado dentro de `reports/archive/`.

### Material apartado del producto publicado

- entornos virtuales temporales o historicos
- caches Python y de herramientas
- logs y salidas intermedias regenerables
- artefactos de build de ROS 2 (`build/`, `install/`, `log/`)
- carpetas historicas de trabajo no necesarias para la entrega final

### Archivo historico

- El archivo historico consolidado queda bajo `reports/archive/`.
- El repositorio publicado excluye por `.gitignore` artefactos regenerables, logs, builds y checkpoints pesados incompatibles con GitHub.

## Material conservado como esencial

### Proyecto ML

- `agarre_inteligente/src`
- `agarre_inteligente/config`
- `agarre_inteligente/data`
- `agarre_inteligente/experiments`
- `agarre_inteligente/docs`
- `agarre_inteligente/scripts`
- `agarre_inteligente/venv` como entorno reproducible local cuando se reconstruye con `bootstrap.sh`

### Proyecto ROS 2

- `agarre_ros2_ws/src`
- `agarre_ros2_ws/scripts`
- `agarre_ros2_ws/tools`
- `agarre_ros2_ws/models`
- `agarre_ros2_ws/worlds`
- documentación raíz del workspace ROS 2

### Documentación y trazabilidad

- `reports/INVENTARIO_ARTEFACTOS_TFM.md`
- `reports/docs/workspace/AUDITORIA_MOVEIT_GAZEBO_QT.md`
- `reports/docs/workspace/VALIDACION_WORKSPACE_2026_03_16.md`
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
- El conflicto de plugins Qt con OpenCV se ha resuelto usando `opencv-python-headless` en el entorno de vision para evitar el bloqueo del panel al arrancar.

## Recomendación de entrega

El workspace queda razonablemente preparado para entrega si el objetivo es presentar:

- código fuente limpio
- resultados reproducibles
- documentación de validación
- separación visible de artefactos reconstruibles

El workspace queda limpio para versionado y, al mismo tiempo, ejecutable localmente. Los artefactos locales regenerables siguen fuera de git mediante `.gitignore`.

## Publicacion

- Repositorio remoto: `https://github.com/JLRRC/TFM_VIU_09MIAR`
- Rama principal publicada: `main`
- Etiqueta inicial de entrega: `v1.0.0`