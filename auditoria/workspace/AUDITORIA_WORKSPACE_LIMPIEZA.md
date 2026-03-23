# AUDITORIA_WORKSPACE_LIMPIEZA

## Resumen ejecutivo

Se ejecutó una auditoría real del workspace en `/home/laboratorio/TFM` y una reordenación controlada orientada a mantener operación y trazabilidad.

Resultado principal:

- Se limpió la raíz moviendo cierres técnicos fuera de raíz a `HISTORICO/cierres/...`.
- Se retiraron `build/`, `install/` y `log/` de raíz a `BORRAR/cuarentena_raiz_20260323/`.
- Se mantuvieron operativos los tres scripts raíz críticos.
- Se corrigió documentación operativa y se añadió README en `auditoria/`.
- Se desacopló `recrear_artefactos_tfm.sh` de `BORRAR` para temporales (ahora usa `auditoria/tmp/recreated_tmp`).

## Estructura encontrada (inicio)

Raíz detectada al inicio:

- `agarre_inteligente/`
- `agarre_ros2_ws/`
- `reports/`
- `auditoria/`
- `HISTORICO/`
- `BORRAR/`
- `build/`, `install/`, `log/` (en raíz)
- múltiples cierres técnicos y trazas en raíz (`DIRECTO_*`, `MOVEIT_MASTER_*`, `*_MASTER.*`)
- scripts raíz: `lanzar_panelv2.sh`, `recrear_experimentos_cap5_gpu.sh`, `recrear_artefactos_tfm.sh`

## Problemas detectados

1. Raíz sobrecargada con documentación histórica de cierres.
2. Artefactos de compilación/log en raíz (`build/install/log`) que no son necesarios para uso diario.
3. README raíz desactualizado (referencias a `docs/panel_v2/*` inexistentes).
4. `recrear_artefactos_tfm.sh` dependía de `BORRAR` para temporales.

## Criterios usados

- `VIGENTE_ESENCIAL`: necesario para panel, recreación de experimentos/artefactos y evidencia final.
- `HISTORICO_UTIL`: trazabilidad técnica útil, no necesaria para operación diaria.
- `BORRAR_CANDIDATO`: residuales, duplicados o artefactos intermedios; se conservan en cuarentena controlada.

## Cambios aplicados

### Reubicación a HISTORICO

- Cierres Directo movidos a: `HISTORICO/cierres/directo/20260323/`
- Cierres MoveIt movidos a: `HISTORICO/cierres/moveit/20260323/`
- Cierres globales/autónomos movidos a: `HISTORICO/cierres/global/20260323/`

### Cuarentena BORRAR

- `build/`, `install/`, `log/` de raíz movidos a:
  - `BORRAR/cuarentena_raiz_20260323/build`
  - `BORRAR/cuarentena_raiz_20260323/install`
  - `BORRAR/cuarentena_raiz_20260323/log`

### Documentación revisada

- Actualizado: `README.md` (raíz)
- Actualizado: `HISTORICO/README.md`
- Actualizado: `agarre_ros2_ws/README.md` (nota de compatibilidad de nombre)
- Creado: `auditoria/README.md`

### Ajuste de script operativo

- `recrear_artefactos_tfm.sh`: temporales pasan de `BORRAR/report_extra/recreated_tmp` a `auditoria/tmp/recreated_tmp`.

## Estructura final recomendada en raíz

- `agarre_inteligente/`
- `agarre_ros2_ws/` (se mantiene por compatibilidad de rutas y scripts)
- `reports/`
- `auditoria/`
- `HISTORICO/`
- `BORRAR/` (cuarentena controlada)
- `README.md`
- `.gitignore`
- `lanzar_panelv2.sh`
- `recrear_experimentos_cap5_gpu.sh`
- `recrear_artefactos_tfm.sh`

## Validación posterior a la reordenación

Se validó:

- Sintaxis bash:
  - `lanzar_panelv2.sh`
  - `recrear_experimentos_cap5_gpu.sh`
  - `recrear_artefactos_tfm.sh`
- Presencia de entradas críticas:
  - `agarre_ros2_ws/scripts/start_panel_v2.sh`
  - `agarre_ros2_ws/scripts/stop_panel_v2.sh`
  - `agarre_inteligente/scripts/run_experiment.py`
  - `agarre_inteligente/scripts/regenerate_chapter5_post_retrain.py`
  - rutas de salida en `reports/figures/cap5` y `reports/evidence/chapter5`

Estado: no se detectaron roturas estructurales en panel/experimentos/reportes por los movimientos ejecutados.
