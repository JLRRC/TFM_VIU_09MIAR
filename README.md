# TFM

Raiz principal del proyecto del TFM de agarre inteligente.

## Estructura

- `agarre_inteligente/`: entrenamiento, evaluacion, configuraciones y resultados de vision.
- `agarre_ros2_ws/`: workspace ROS 2 con panel, Gazebo, MoveIt 2 y utilidades de integracion.
- `reports/`: artefactos curados del documento, figuras, tablas, metricas y evidencias.
- `auditoria/`: evidencias operativas y logs de validacion generados durante pruebas del panel.

## Documentos utiles

- `docs/panel_v2/README_VALIDACION_PANEL_V2.md`: secuencia recomendada para validar el panel.
- `docs/panel_v2/ROADMAP_PANEL_V2_OPERATIVO.md`: estado tecnico por fases y pendientes actuales.
- `docs/panel_v2/AUDITORIA_PANEL_V2_TFM.md`: auditoria tecnica consolidada del panel.
- `docs/panel_v2/CHECKLIST_TFM_PANEL_V2.md`: checklist de verificacion para la defensa.

## Arranque rapido

Para lanzar el panel desde la raiz:

```bash
./lanzar_panelv2.sh --fg
```

Para parar el stack:

```bash
./agarre_ros2_ws/scripts/stop_panel_v2.sh
```

## Flujos principales

### Panel ROS 2

- Lanzador principal: `./lanzar_panelv2.sh`
- Lanzador directo del workspace: `./agarre_ros2_ws/scripts/start_panel_v2.sh`
- Estado del stack: `./agarre_ros2_ws/scripts/status_panel_v2.sh`
- Recuperacion del panel: `./agarre_ros2_ws/scripts/recover_panel_v2.sh`

### Vision

- Entrenamiento y evaluacion: `agarre_inteligente/scripts/`
- Resultados oficiales por experimento: `agarre_inteligente/experiments/`

### Documento

- Figuras finales: `reports/figures/`
- Tablas finales: `reports/tables/`
- Metricas validadas: `reports/metrics/validated/`
- Evidencias funcionales: `reports/evidence/`

## Punto de entrada recomendado

Si quieres trabajar con el sistema completo, empieza por:

1. `./lanzar_panelv2.sh --fg`
2. `docs/panel_v2/README_VALIDACION_PANEL_V2.md`
3. `docs/panel_v2/ROADMAP_PANEL_V2_OPERATIVO.md`
