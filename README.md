# Workspace TFM

Este workspace contiene la version consolidada del TFM de agarre inteligente. La raiz se ha simplificado para dejar solo los elementos operativos y de entrega imprescindibles:

- `agarre_inteligente/`: bloque experimental de vision y aprendizaje profundo.
- `agarre_ros2_ws/`: bloque ROS 2, Gazebo, MoveIt y panel de control.
- `reports/`: repositorio unico de artefactos, tablas, ilustraciones, validaciones y evidencias del TFM.
- `lanzar_panelv2.sh`: script raiz para arrancar el panel V2.
- `actualizar_reports.sh`: script raiz para actualizar el contenido consolidado de `reports/`.

## 1. Estructura del workspace

### `agarre_inteligente/`

Contiene el bloque academico-experimental del TFM:

- configuraciones de experimentos (`config/`)
- codigo fuente de modelos, entrenamiento y evaluacion (`src/`)
- scripts de preparacion de dataset, resumen, figuras y tablas (`scripts/`)
- datos procesados y dataset (`data/`)
- experimentos entrenados y checkpoints (`experiments/`)
- documentacion tecnica y de trazabilidad (`docs/`)
- entorno Python reproducible (`venv/`)

Este subproyecto es la fuente de verdad para las metricas, tablas y figuras del TFM. Los artefactos finales ya no se guardan aqui, sino en la carpeta raiz `reports/`.

### `agarre_ros2_ws/`

Contiene el bloque robotico-operativo del TFM:

- paquetes ROS 2 (`src/`)
- scripts de arranque, validacion, diagnosis y utilidades (`scripts/`)
- herramientas auxiliares (`tools/`)
- modelos y mundos de simulacion (`models/`, `worlds/`)
- configuraciones de pruebas y desarrollo (`pytest.ini`, `ruff.toml`, `requirements-dev.txt`)

Este subproyecto proporciona el stack de simulacion y control del UR5 con RG2, junto con el panel Qt de operacion y el modulo de integracion con el bloque TFM.

### `reports/`

Es la carpeta unica de salida y consolidacion del TFM. Reune:

- tablas finales (`reports/tables/`)
- figuras generales (`reports/figures/`)
- ilustraciones finales del capitulo 5 (`reports/tfm_figuras_cap5_1/`)
- evidencia ROS 2 / Gazebo (`reports/tfm_ros_gazebo_results/`)
- auditoria visual (`reports/tfm_visual_revision/`)
- auditoria del panel (`reports/panel_audit/`)
- episodios y logs del panel (`reports/panel_logs/`)
- benchmark y auditoria Cornell (`reports/bench/`, `reports/cornell_audit/`)
- documentos de validacion y trazabilidad del workspace (`reports/docs/workspace/`, `reports/validation/`)
- archivo historico apartado (`reports/archive/`)

El inventario maestro de lo que debe existir en esta carpeta esta en `reports/INVENTARIO_ARTEFACTOS_TFM.md`.

## 2. Scripts raiz permitidos

### `lanzar_panelv2.sh`

Lanza el panel V2 del workspace ROS 2 usando el entorno virtual disponible. Detecta automaticamente alguno de estos entornos, por este orden:

- `agarre_inteligente/.venv-tfm`
- `agarre_inteligente/venv`
- `agarre_inteligente/.venv`

Uso normal:

```bash
./lanzar_panelv2.sh
```

Uso sin entorno grafico visible:

```bash
export PANEL_FORCE_OFFSCREEN=1
./lanzar_panelv2.sh
```

### `actualizar_reports.sh`

Regenera y consolida el contenido de `reports/` desde el bloque experimental y desde cualquier salida temporal que haya aparecido en los subproyectos.

Acciones principales:

- regenera `summary_results.csv`, figuras y tablas del bloque experimental
- sincroniza ilustraciones y artefactos historicos
- consolida cualquier `reports/` que reaparezca dentro de subproyectos
- elimina carpetas `reports/` fuera de la raiz
- reconstruye el inventario `reports/INVENTARIO_ARTEFACTOS_TFM.md`

Uso:

```bash
./actualizar_reports.sh
```

## 3. Flujo de trabajo recomendado

### Para trabajar con el panel

```bash
./lanzar_panelv2.sh
```

### Para refrescar el material entregable del TFM

```bash
./actualizar_reports.sh
```

### Para revisar el inventario de entrega

Consultar:

- `reports/INVENTARIO_ARTEFACTOS_TFM.md`
- `reports/docs/workspace/AUDITORIA_MOVEIT_GAZEBO_QT.md`
- `reports/docs/workspace/VALIDACION_WORKSPACE_2026_03_16.md`
- `reports/docs/workspace/ENTREGA_WORKSPACE_2026_03_16.md`

## 4. Criterio de organizacion aplicado

El workspace se ha reorganizado con estos objetivos:

- una sola carpeta `reports/` como destino canonico de resultados
- raiz minimizada para facilitar subida a git y entrega
- separacion clara entre codigo fuente y artefactos finales
- trazabilidad documental sin duplicar resultados en varias ubicaciones
- posibilidad de reconstruir y actualizar `reports/` desde un unico punto de entrada

## 5. Estado esperado tras la reorganizacion

En la raiz del workspace deben quedar, como elementos visibles principales:

- `agarre_inteligente/`
- `agarre_ros2_ws/`
- `reports/`
- `lanzar_panelv2.sh`
- `actualizar_reports.sh`
- `README.md`

Los artefactos auxiliares o historicos fuera de esta estructura se archivan dentro de `reports/archive/`.