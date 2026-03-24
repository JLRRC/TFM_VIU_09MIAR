# TFM

Raiz operativa del TFM para percepcion y agarre con ROS 2.

## Estructura vigente

- `agarre_inteligente/`: entrenamiento, evaluacion, configuraciones y resultados de vision.
- `agarre_ros2_ws/`: workspace ROS 2 (panel, Gazebo, MoveIt 2 y utilidades).
- `reports/`: artefactos curados de memoria (figuras, tablas, metricas y evidencias).
- `auditoria/`: trazas operativas y auditorias de ejecucion.

## Scripts raiz

- `./lanzar_panelv2.sh`: arranque principal del panel (detecta SSH y activa modo headless remoto).
- `./recrear_experimentos_cap5_gpu.sh`: relanza entrenamientos del capitulo 5.
- `./recrear_artefactos_tfm.sh`: regenera artefactos curados del documento.

## Arranque rapido del panel

```bash
./lanzar_panelv2.sh
```

Parada del stack:

```bash
./agarre_ros2_ws/scripts/stop_panel_v2.sh
```

## Flujo de reproducibilidad

1. Reentrenar (si aplica): `./recrear_experimentos_cap5_gpu.sh`
2. Regenerar artefactos: `./recrear_artefactos_tfm.sh`
3. Revisar salida final en `reports/`
