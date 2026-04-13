# TFM

Raiz del TFM para percepcion y agarre con ROS 2, ordenada para trabajo tecnico y entrega.

## Estructura vigente

- `agarre_inteligente/`: codigo, datos y resultados del bloque de vision.
- `agarre_ros2_ws/`: workspace ROS 2 con codigo fuente, scripts y configuracion activa.
- `reports/`: artefactos curados de la memoria (figuras, tablas, metricas y evidencias finales).

## Scripts raiz

- `./lanzar_panelv2.sh`: arranque principal del panel.
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

1. Reentrenar, si aplica: `./recrear_experimentos_cap5_gpu.sh`
2. Regenerar artefactos: `./recrear_artefactos_tfm.sh`
3. Revisar salida final en `reports/`
